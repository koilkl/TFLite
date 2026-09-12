/* Copyright 2019 The TensorFlow Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

// #include <TensorFlowLite.h>

#include "main_functions.h"
#include "image_provider.h"
#include "model_settings.h"
// Include the exported model data header.
// Rename this include to match your exported model's header file name,
// e.g. "Square_model_data.h" or "MyModel_model_data.h".
#include "tm_model_data.h"

// ── Camera capture resolution — change THIS line only ──
// Students: set the frame side you want (8-512).  The camera library reads
// this at runtime; GetImage() auto-resizes the capture down to the model
// input size (OUT_WIDTH×OUT_HEIGHT = 96×96), so no header/library edits are
// needed — only re-train/re-export if you change the MODEL input size.
int g_imx219_frame_side = 96;


#include "tensorflow/lite/micro/micro_log.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#if __has_include("model_resolver.h")
#include "model_resolver.h"
#define TM_HAS_GENERATED_MODEL_RESOLVER 1
#else
#define TM_HAS_GENERATED_MODEL_RESOLVER 0
#endif
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/system_setup.h"

// Enable ESP-NN acceleration for ESP32-P4 if available.
// ESP-NN provides optimized Conv2D, DepthwiseConv2D, and FullyConnected
// kernels using RISC-V SIMD instructions, giving 10-50x speedup over
// the portable reference kernels.
#if __has_include("esp_nn.h") && !defined(ESP_NN)
#define ESP_NN 1
#endif
#if __has_include("esp_nn_conv2d.h")
#include "esp_nn_conv2d.h"
#endif

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include <SD_MMC.h>
#include <FFat.h>
#include <driver/gpio.h>
#include <errno.h>
#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

// Globals, used for compatibility with Arduino-style sketches.
namespace {
tflite::ErrorReporter* error_reporter = nullptr;
const tflite::Model* model = nullptr;
tflite::MicroInterpreter* interpreter = nullptr;
TfLiteTensor* input = nullptr;

// Bi-directional control: S3 can request P4 to stop/start UART transmission.
// Inference continues regardless; only the TX path is gated.
static volatile bool s_transmit_enabled = true;

// RX diagnostics: count bytes and control packets received from S3.
static volatile uint32_t s_rx_bytes = 0;
static volatile uint32_t s_rx_ack_stop = 0;
static volatile uint32_t s_rx_resume = 0;

// True while the P4 Debug Serial carries a binary frame stream (capture /
// infer+gray modes).  Defined after s_op_mode below; declared here so the
// UART control helpers can use it.
static bool serial_stream_clean();

static void uart_control_enable() {
  s_transmit_enabled = true;
  if (!serial_stream_clean()) Serial.println("UART TX: ENABLED");
}

static void uart_control_disable() {
  s_transmit_enabled = false;
  if (!serial_stream_clean()) Serial.println("UART TX: DISABLED (S3 ack)");
}

static HardwareSerial UartToS3(1);

static constexpr int kDebugBaud = 921600;
static constexpr int kUartBaud = 921600;
static constexpr int kUartRxPin = 10;
static constexpr int kUartTxPin = 11;

static constexpr int kHandshakePwmPin = 9;
static constexpr int kHandshakePwmDuty = 255;

static constexpr uint8_t kSync0 = 0xAA;
static constexpr uint8_t kSync1 = 0x55;
static constexpr uint8_t kMsgTypeInference = 0x01;
static constexpr uint8_t kMsgTypeControl   = 0x02;  // S3 → P4 control messages

// S3 → P4 control commands
static constexpr uint8_t kCtrlAckStop = 0x01;  // S3 confirmed sign → stop TX
static constexpr uint8_t kCtrlResume  = 0x02;  // S3 done → resume TX

static constexpr uint32_t kUartTxTaskStackBytes = 4 * 1024;
static constexpr uint32_t kUartRxTaskStackBytes = 4 * 1024;
// The inference task needs more than 32 KB: the auto search-box chain
// (adaptive band selection -> find_search_box -> BFS) plus ESP-NN kernels
// overflowed a 32 KB stack (observed 2.6 KB past the bound on Core 1).
static constexpr uint32_t kInferenceTaskStackBytes = 64 * 1024;
static constexpr uint32_t kSdTaskStackBytes = 8 * 1024;
static constexpr uint32_t kDebugCmdStackBytes = 6 * 1024;

// ── Debug Serial Command parser (same sync-based layout as UART) ─────
// Frame format on Debug Serial (USB CDC @ 921600):
//   | SYNC0=0xAA | SYNC1=0x55 | SYNC2=0xCC | cmd | len | payload[len..] | xor8(0..4+len) |
// SYNC2=0xCC distinguishes debug commands from:
//   - 0x01 = UART inference, 0x02 = UART control, 0xAA = RGB stream header
// Responses share the SAME layout; len=N on error responses with an ASCII
// null-terminated message payload.  Cmd IDs are listed in kCmd*.
static constexpr uint8_t kDebugSync0 = 0xAA;
static constexpr uint8_t kDebugSync1 = 0x55;
static constexpr uint8_t kDebugSync2 = 0xCC;

// Commands (Host → P4)
static constexpr uint8_t kCmdNop              = 0x00;
static constexpr uint8_t kCmdSetOodThresholds = 0x01;   // 4× float LE (sign_pct_min, sign_pct_max, max_prob_min, entropy_ratio_max)
static constexpr uint8_t kCmdGetOodThresholds = 0x02;   // → 4× float LE response
static constexpr uint8_t kCmdSetMaskThresholds= 0x03;   // 2× uint8 LE (dark, lum)   — Host UI Dark/Lum sliders
static constexpr uint8_t kCmdGetMaskThresholds= 0x04;   // → 2× uint8 LE response
static constexpr uint8_t kCmdSetMode          = 0x05;   // 1× uint8: 0=INFERENCE, 1=CAPTURE_RGB, 2=CAPTURE_GRAY
static constexpr uint8_t kCmdGetMode          = 0x06;   // → 1× uint8 response
static constexpr uint8_t kCmdPing             = 0x7F;   // → ack len=0

// Operation mode (runtime switchable via Debug Serial)
enum class OpMode : uint8_t {
  kInference = 0,   // default: full pipeline → UART / SD / periodic log
  kCaptureRgb= 1,   // PLAIN: AA 55 AA + 96×96×3 RGB, AItraining capture panel 能用
  kCaptureGray= 2,  // PLAIN: AA 55 AA + 96×96×1 GRAY, AItraining 预览/采集 能用
  kInferGray  = 3,  // infer + stream GRAY on Serial (S3 UART still active), no logs
  kExtCaptureRgb =11,// EXTENDED: AA 55 AB + kind/fid/w/h + pixels + xor（脚本工具用）
  kExtCaptureGray=12 // EXTENDED: 同上 GRAY8
};
static volatile OpMode s_op_mode = OpMode::kInference;

// True while the P4 Debug Serial carries a binary frame stream (capture /
// infer+gray modes).  In these modes any text written to Serial splices
// bytes into the AA 55 AA frame stream and corrupts frames for
// AItraining's SerialFrameReader — so asynchronous text logs (SD task,
// UART control prints) must stay off the wire.  The one-time banners and
// the mode-transition line print before the first frame, which is safe.
static bool serial_stream_clean() {
  switch (s_op_mode) {
    case OpMode::kCaptureRgb:
    case OpMode::kCaptureGray:
    case OpMode::kInferGray:
    case OpMode::kExtCaptureRgb:
    case OpMode::kExtCaptureGray:
      return true;
    default:
      return false;
  }
}
static const char *op_mode_name(OpMode m) {
  switch (m) {
    case OpMode::kInference:     return "INFERENCE";
    case OpMode::kCaptureRgb:    return "CAPTURE_RGB/plain(AItraining)";
    case OpMode::kCaptureGray:   return "CAPTURE_GRAY/plain(AItraining)";
    case OpMode::kInferGray:     return "INFER+GRAY (S3 + gray stream)";
    case OpMode::kExtCaptureRgb: return "CAPTURE_RGB/extended";
    case OpMode::kExtCaptureGray:return "CAPTURE_GRAY/extended";
    default: return "?";
  }
}

// ── Capture-mode payload sync on Debug Serial ────────────────────────
//   We support TWO wire formats, chosen per mode keyword below.
//   Defaults (mode gray / mode rgb) match the AItraining app so the
//   existing SerialFrameReader class works without code changes:
//
//   Format A = PLAIN (AItraining-compatible, default) —
//     GRAY : AA 55 AA  +  frame_side*frame_side*1  bytes GRAY8
//     RGB  : AA 55 AA  +  frame_side*frame_side*3  bytes RGB24 (R-G-B interleave)
//   This exactly matches `HEADER = bytes([0xAA,0x55,0xAA])` plus
//   `frame_size = side² * channels` layout used by AItraining's
//   SerialFrameReader.read_frame() — so live serial preview, data
//   capture panel and source capture of "upper" 96×96 grayscale work
//   out of the box with no app changes.
//
//   Format B = EXTENDED (script tools only, reachable via explicit
//   numeric mode 11/12) —
//     0xAA 0x55 0xAB  kind(1)  fid_LE(2)  w_LE(2)  h_LE(2)  <BYTES>  xor8
//   kind 1=RGB24 / 2=GRAY8.  xor8 covers every byte before checksum.
//   Students should not need this — reserved for offline tools that
//   want a strict checksum + metadata for every captured frame.
static constexpr uint8_t kCapSync0 = 0xAA;
static constexpr uint8_t kCapSync1 = 0x55;
static constexpr uint8_t kCapSync2 = 0xAA;  // PLAIN default: match AItraining
static constexpr uint8_t kCapExtSync2  = 0xAB;
static constexpr uint8_t kCapKindRgb = 0x01;
static constexpr uint8_t kCapKindGray= 0x02;

static constexpr uint32_t kSaveEveryNFrames = 10;
static constexpr int kImageWidth = OUT_WIDTH;
static constexpr int kImageHeight = OUT_WIDTH;
static constexpr size_t kImageBytes = (size_t)kImageWidth * (size_t)kImageHeight;
static constexpr size_t kSdQueueDepth = 2;
static constexpr bool kEnableSdLogger = true;
static constexpr bool kUseSdMmcCustomPins = false;
static constexpr int kSdClkPin = -1;
static constexpr int kSdCmdPin = -1;
static constexpr int kSdD0Pin = -1;
static constexpr int kSdD1Pin = -1;
static constexpr int kSdD2Pin = -1;
static constexpr int kSdD3Pin = -1;

static constexpr const char *kMountPoint = "/sdcard";
static constexpr const char *kFfatMountPoint = "/ffat";
enum class StorageBackend : uint8_t { Auto = 0, SdMmc = 1, FFat = 2 };
static constexpr StorageBackend kStorageBackend = StorageBackend::SdMmc;
static constexpr bool kFormatFfatOnFail = true;

#ifndef BOARD_SDMMC_POWER_PIN
#define BOARD_SDMMC_POWER_PIN 45
#endif

#ifndef BOARD_SDMMC_POWER_ON_LEVEL
#define BOARD_SDMMC_POWER_ON_LEVEL 0
#endif

static void sdcard_power_cycle(int on_level) {
  gpio_set_direction((gpio_num_t)BOARD_SDMMC_POWER_PIN, GPIO_MODE_OUTPUT);
  int off_level = on_level ? 0 : 1;
  gpio_set_level((gpio_num_t)BOARD_SDMMC_POWER_PIN, off_level);
  delay(200);
  gpio_set_level((gpio_num_t)BOARD_SDMMC_POWER_PIN, on_level);
  delay(300);
}

struct UartPacket {
  uint16_t frame_id;
  uint8_t label_id;
  uint8_t confidence;
  uint8_t flags;
};

static uint8_t calc_uart_checksum(const uint8_t *data, size_t len) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < len; i++) {
    checksum ^= data[i];
  }
  return checksum;
}

static QueueHandle_t s_uart_queue = nullptr;

struct SdPacket {
  uint16_t frame_id;
  uint8_t  buffer_index;
  uint8_t  label_id;
  uint8_t  confidence;
  uint8_t  flags;
};

static QueueHandle_t s_sd_queue = nullptr;
// Free-list of s_sd_buffers indices.  The inference task takes an index
// from here before filling a buffer; the sd task returns it after the PGM
// write.  Without this, a slow SD write could outlive two queued packets
// and the ring counter would hand the SAME buffer to the producer while
// the sd task is still reading it (torn PGM frames).
static QueueHandle_t s_sd_free_queue = nullptr;
static uint8_t s_sd_buffers[kSdQueueDepth][kImageBytes];
static char s_run_dir[32] = {0};
static bool s_sd_using_sd_mmc = false;
static bool s_sd_using_ffat = false;
static const char *s_sd_mount_point = kMountPoint;
static bool s_sd_full = false;

// In order to use optimized tensorflow lite kernels, a signed int8_t quantized
// model is preferred over the legacy unsigned model format. This means that
// throughout this project, input images must be converted from unisgned to
// signed format. The easiest and quickest way to convert from unsigned to
// signed 8-bit integers is to subtract 128 from the unsigned value to get a
// signed value.

// An area of memory to use for input, output, and intermediate arrays.
// 8 MB: the default AItraining architecture (conv 32/64/128, dense 128)
// exports ~5 MB models on multi-class datasets — a smaller arena fails
// AllocateTensors on the device.  SRAM-first allocation below falls back
// to PSRAM automatically.
constexpr int kTensorArenaSize = 8192 * 1024;
static uint8_t *tensor_arena=nullptr ;

}  // namespace

static bool mount_sd() {
  s_sd_using_sd_mmc = false;
  s_sd_using_ffat = false;
  s_sd_mount_point = kMountPoint;

  auto mount_ffat = []() -> bool {
    FFat.end();
    if (FFat.begin(false, kFfatMountPoint, 5, "ffat")) {
      s_sd_using_ffat = true;
      s_sd_mount_point = kFfatMountPoint;
      return true;
    }
    if (kFormatFfatOnFail && FFat.begin(true, kFfatMountPoint, 5, "ffat")) {
      s_sd_using_ffat = true;
      s_sd_mount_point = kFfatMountPoint;
      return true;
    }
    return false;
  };

  auto mount_sdmmc = []() -> bool {
    gpio_set_pull_mode((gpio_num_t)44, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)39, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)40, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)41, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)42, GPIO_PULLUP_ONLY);

    #ifdef SOC_SDMMC_IO_POWER_EXTERNAL
    SD_MMC.setPowerChannel(-1);
    #endif

    const int on_levels[] = {BOARD_SDMMC_POWER_ON_LEVEL, BOARD_SDMMC_POWER_ON_LEVEL ? 0 : 1};
    for (int attempt = 0; attempt < 12; attempt++) {
      for (size_t lv = 0; lv < (sizeof(on_levels) / sizeof(on_levels[0])); lv++) {
        sdcard_power_cycle(on_levels[lv]);

        SD_MMC.end();
        if (SD_MMC.begin(kMountPoint, false, false, SDMMC_FREQ_DEFAULT)) {
          s_sd_using_sd_mmc = true;
          s_sd_mount_point = kMountPoint;
          return true;
        }

        SD_MMC.end();
        if (SD_MMC.begin(kMountPoint, false, false, 400)) {
          s_sd_using_sd_mmc = true;
          s_sd_mount_point = kMountPoint;
          return true;
        }
      }
      delay(200);
    }

    SD_MMC.end();
    s_sd_using_sd_mmc = false;
    return false;
  };

  if (kStorageBackend == StorageBackend::FFat) {
    return mount_ffat();
  }
  if (kStorageBackend == StorageBackend::SdMmc) {
    return mount_sdmmc();
  }

  if (mount_sdmmc()) {
    return true;
  }
  return mount_ffat();

}

static bool write_pgm(const char *path, const uint8_t *gray) {
  FILE *f = fopen(path, "wb");
  if (!f) {
    return false;
  }
  fprintf(f, "P5\n%d %d\n255\n", kImageWidth, kImageHeight);
  size_t written = fwrite(gray, 1, kImageBytes, f);
  fclose(f);
  return written == kImageBytes;
}

static bool make_run_dir() {
  uint32_t max_id = 0;

  DIR *dir = opendir(s_sd_mount_point);
  if (dir) {
    for (;;) {
      struct dirent *ent = readdir(dir);
      if (!ent) {
        break;
      }
      const char *p = ent->d_name;
      if (strncmp(p, "run_", 4) != 0) {
        continue;
      }
      uint32_t id = atoi(p + 4);
      if (id && id <= 9999 && id > max_id) {
        max_id = id;
      }
    }
    closedir(dir);
  }

  for (uint32_t i = 0; i < 1000; i++) {
    uint32_t id = max_id + 1 + i;
    snprintf(s_run_dir, sizeof(s_run_dir), "%s/run_%04d", s_sd_mount_point, id);
    if (mkdir(s_run_dir, 0775) == 0) {
      return true;
    }
  }

  return false;
}

static void sd_task(void *arg) {
  (void)arg;
  uint32_t saved = 0;
  static bool s_labels_csv_header_written = false;
  while (true) {
    SdPacket pkt;
    if (xQueueReceive(s_sd_queue, &pkt, portMAX_DELAY) != pdTRUE) {
      continue;
    }

    if (s_sd_full) {
      // Buffer never used — return it or the free-list drains and the
      // producer stalls on receive.
      if (s_sd_free_queue) xQueueSend(s_sd_free_queue, &pkt.buffer_index, 0);
      continue;
    }

    char path[96];
    snprintf(path, sizeof(path), "%s/frame_%05u.pgm", s_run_dir, (unsigned)pkt.frame_id);
    bool ok = write_pgm(path, s_sd_buffers[pkt.buffer_index]);

    if (ok) {
      char csv_path[96];
      snprintf(csv_path, sizeof(csv_path), "%s/labels.csv", s_run_dir);
      FILE *fc = fopen(csv_path, "a");
      if (fc) {
        if (!s_labels_csv_header_written) {
          fprintf(fc, "frame_id,label_id,confidence,flags\n");
          s_labels_csv_header_written = true;
        }
        fprintf(fc, "%u,%u,%u,%u\n",
                (unsigned)pkt.frame_id,
                (unsigned)pkt.label_id,
                (unsigned)pkt.confidence,
                (unsigned)pkt.flags);
        fclose(fc);
      } else if (errno == ENOSPC) {
        ok = false;
      }
    }

    if (ok) saved++;

    if (!ok && errno == ENOSPC) {
      s_sd_full = true;
      if (!serial_stream_clean()) {
        Serial.printf("Storage full (ENOSPC). Stop saving frames. Last=%s\n", path);
      }
    }

    // PGM write finished — return the buffer to the free-list so the
    // producer can safely reuse it.
    if (s_sd_free_queue) xQueueSend(s_sd_free_queue, &pkt.buffer_index, 0);

    if ((saved % 10) == 0 && !serial_stream_clean()) {
      Serial.printf("SD saved=%lu last=%s %s\n", (unsigned long)saved, path, ok ? "OK" : "FAIL");
    }
  }
}

static void uart_tx_task(void *arg) {
  (void)arg;
  for (;;) {
    UartPacket pkt;
    if (xQueueReceive(s_uart_queue, &pkt, portMAX_DELAY) != pdTRUE) {
      continue;
    }

    if (!s_transmit_enabled) {
      continue;  // silently drop — S3 hasn't asked us to resume yet
    }

    uint8_t buf[9];
    buf[0] = kSync0;
    buf[1] = kSync1;
    buf[2] = kMsgTypeInference;
    buf[3] = (uint8_t)(pkt.frame_id & 0xFF);
    buf[4] = (uint8_t)((pkt.frame_id >> 8) & 0xFF);
    buf[5] = pkt.label_id;
    buf[6] = pkt.confidence;
    buf[7] = pkt.flags;
    buf[8] = calc_uart_checksum(buf, 8);
    UartToS3.write(buf, sizeof(buf));
  }
}

static void uart_rx_task(void *arg) {
  (void)arg;
  // State machine for reading 5-byte control packets from S3:
  //   [0xAA, 0x55, 0x02, cmd, checksum]
  static uint8_t rx_buf[5];
  static uint8_t rx_idx = 0;
  static uint8_t rx_state = 0;  // 0=wait sync0, 1=wait sync1, 2=wait rest

  uint32_t last_rx_log_ms = 0;

  for (;;) {
    while (UartToS3.available() > 0) {
      uint8_t b = (uint8_t)UartToS3.read();
      s_rx_bytes++;

      if (rx_state == 0) {
        if (b == kSync0) {
          rx_buf[0] = b;
          rx_idx = 1;
          rx_state = 1;
        }
        continue;
      }

      if (rx_state == 1) {
        if (b == kSync1) {
          rx_buf[1] = b;
          rx_idx = 2;
          rx_state = 2;
        } else if (b == kSync0) {
          rx_buf[0] = b;
          rx_idx = 1;
          rx_state = 1;
        } else {
          rx_state = 0;
        }
        continue;
      }

      // rx_state == 2: collecting remaining bytes
      rx_buf[rx_idx++] = b;
      if (rx_idx < sizeof(rx_buf)) {
        continue;
      }

      // Full packet received
      rx_state = 0;
      rx_idx = 0;

      if (rx_buf[2] != kMsgTypeControl) {
        if (!serial_stream_clean()) {
          Serial.printf("UART RX: unknown msg_type 0x%02X (expected 0x%02X)\n",
                        rx_buf[2], kMsgTypeControl);
        }
        continue;
      }

      // Verify checksum: XOR of bytes 0-3
      uint8_t expected_csum = calc_uart_checksum(rx_buf, 4);
      if (rx_buf[4] != expected_csum) {
        if (!serial_stream_clean()) {
          Serial.printf("UART RX: bad checksum (got 0x%02X expected 0x%02X)\n",
                        rx_buf[4], expected_csum);
        }
        continue;
      }

      uint8_t cmd = rx_buf[3];

      if (cmd == kCtrlAckStop) {
        s_rx_ack_stop++;
        if (!serial_stream_clean()) {
          Serial.printf("UART RX: ACK_STOP from S3 (#%lu) — disabling TX\n",
                        (unsigned long)s_rx_ack_stop);
        }
        uart_control_disable();
      } else if (cmd == kCtrlResume) {
        s_rx_resume++;
        if (!serial_stream_clean()) {
          Serial.printf("UART RX: RESUME from S3 (#%lu) — resuming TX\n",
                        (unsigned long)s_rx_resume);
        }
        uart_control_enable();
      } else {
        if (!serial_stream_clean()) {
          Serial.printf("UART RX: unknown control cmd 0x%02X\n", cmd);
        }
      }
    }

    // Periodic heartbeat: show RX stats even when idle.
    // NEVER print while Serial carries a binary frame stream (capture /
    // infer+gray modes) — the text line would be injected between frames
    // and desync the host frame parser.
    uint32_t now = millis();
    if (now - last_rx_log_ms >= 5000) {
      last_rx_log_ms = now;
      if (!serial_stream_clean()) {
        Serial.printf("UART RX: bytes=%lu ack_stop=%lu resume=%lu tx_enabled=%d\n",
                      (unsigned long)s_rx_bytes, (unsigned long)s_rx_ack_stop,
                      (unsigned long)s_rx_resume, (int)s_transmit_enabled);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// ── Debug Serial → Host Response helpers ───────────────────────────────
//  All responses use the SAME sync (0xAA 0x55 0xCC) frame layout as
//  incoming commands.  This matches the UART↔S3 "parser sync" feel you
//  liked.  On error the payload is a null-terminated ASCII string.
static uint8_t debug_xor(const uint8_t *p, size_t n) {
  uint8_t x = 0;
  for (size_t i = 0; i < n; i++) x ^= p[i];
  return x;
}

static void debug_send_ack(uint8_t cmd, const uint8_t *payload, uint8_t len) {
  uint8_t hdr[6] = { kDebugSync0, kDebugSync1, kDebugSync2, cmd, len, 0x00 };
  uint8_t x = debug_xor(hdr, 5);
  for (uint8_t i = 0; i < len; i++) x ^= payload[i];
  Serial.write(hdr, 5);
  if (len > 0 && payload) Serial.write(payload, len);
  Serial.write(x);
  Serial.flush();
}
static void debug_send_ok(uint8_t cmd) {
  debug_send_ack(cmd, nullptr, 0);
}
static void debug_send_error(uint8_t cmd, const char *msg) {
  if (!msg) msg = "err";
  size_t n = strlen(msg) + 1;  // include null terminator
  if (n > 255) n = 255;
  debug_send_ack((uint8_t)(cmd | 0x80), (const uint8_t *)msg, (uint8_t)n);
}

// Read a little-endian float from bytes.  Trivial helper — no unaligned load
// on RISC-V so we byte-by-byte copy to a local aligned.
static float read_f32_le(const uint8_t *p) {
  uint32_t u = (uint32_t)p[0]
             | ((uint32_t)p[1] << 8)
             | ((uint32_t)p[2] << 16)
             | ((uint32_t)p[3] << 24);
  float f;
  memcpy(&f, &u, 4);
  return f;
}
static void write_f32_le(uint8_t *dst, float f) {
  uint32_t u;
  memcpy(&u, &f, 4);
  dst[0] = (uint8_t)(u & 0xFF);
  dst[1] = (uint8_t)((u >> 8) & 0xFF);
  dst[2] = (uint8_t)((u >> 16) & 0xFF);
  dst[3] = (uint8_t)((u >> 24) & 0xFF);
}
static void write_u16_le(uint8_t *dst, uint16_t v) {
  dst[0] = (uint8_t)(v & 0xFF);
  dst[1] = (uint8_t)((v >> 8) & 0xFF);
}

// ── Debug Serial: text + binary dual parser, runtime control + capture ──
//  Two input styles share one serial port so students + scripts coexist:
//
//   Style 1 (students, Arduino Serial Monitor):   plain ASCII lines
//     e.g.  set mask 30 85  →  OK  MASK: dark=30 lum=85
//           mode gray       →  OK  MODE: capture_gray
//           get ood         →  OOD: sp=0.50..70.00 mp=0.700 er=0.650
//           help            →  list of commands
//     Case-insensitive, space/tab separated, line ends with CR or LF.
//
//   Style 2 (Python/scripts): sync-framed binary packets, identical feel
//     to the P4↔S3 UART "flag-based parser" the student asked for:
//        AA 55 CC | cmd | len | payload | xor8
//
//  Runtime mode switch lives at the top of the inference loop; when mode
//  is not kInference we skip Invoke, SD writer and UART TX/RX so nothing
//  clobbers the bus — only capture frames stream out over Debug Serial.
//
// ── ASCII line command helpers (for Arduino Serial Monitor) ─────────────
//   Students type a line (e.g. "set mask 30 85" + Enter) instead of binary.
//   Supported commands (case-insensitive, space/tab separated):
//     help | ?         → print this list
//     ping             → "PONG"
//     get mask         → "MASK: dark=<D> lum=<L>"
//     set mask D L     → Dark/Lum (0..255, auto-clamped, dark<lum enforced)
//     get ood          → "OOD: sp=<min>..<max> mp=<mpmin> er=<ermax>"
//     set ood spmin spmax mpmin ermax
//                      → 4 floats, see image_provider OOD docs
//     get mode         → "MODE: infer | rgb | gray"
//     mode infer       → back to normal inference/SD/UART
//     mode rgb         → dump raw RGB 96×96×3 on Debug Serial (CAPTURE_RGB)
//     mode gray        → dump preprocessed GRAY8 96×96 (what TFLite sees)
static char *skip_ws(char *p) {
  while (*p && (*p==' ' || *p=='\t' || *p=='\r' || *p=='\n')) p++;
  return p;
}
static char *next_tok(char *p, char **out_tok) {
  p = skip_ws(p);
  *out_tok = p;
  while (*p && *p!=' ' && *p!='\t' && *p!='\r' && *p!='\n') {
    *p = (char)tolower((uint8_t)*p);
    p++;
  }
  if (*p) { *p = '\0'; p++; }
  return p;
}
static void ascii_print_help() {
  Serial.println(F(
    "Commands (Serial Monitor, Newline=NL&CR baud 921600):\r\n"
    "  help | ?         → this list\r\n"
    "  ping             → PONG\r\n"
    "  get mask         → MASK dark=?? lum=??  (G channel sign-mask)\r\n"
    "  set mask D L     → D=int L=int e.g.  set mask 30 85\r\n"
    "  get ood          → OOD spmin..spmax mpmin ermax\r\n"
    "  set ood A B C D  → 4 floats, e.g. set ood 0.5 70.0 0.7 0.65\r\n"
    "  get mode         → MODE infer|rgb|gray\r\n"
    "  mode infer       → normal inference + UART + SD\r\n"
    "  mode rgb         → pause, dump RAW RGB24 capture frames on serial\r\n"
    "  mode gray        → pause, dump GRAY8 = what TFLite input tensor sees"
  ));
}
static void ascii_dispatch(char *line) {
  line = skip_ws(line);
  if (!*line) return;
  char *save = nullptr, *tok = nullptr;
  (void)save;
  char *p = next_tok(line, &tok);
  if (!*tok) return;
  if (0 == strcmp(tok, "help") || 0 == strcmp(tok, "?")) {
    ascii_print_help(); return;
  }
  if (0 == strcmp(tok, "ping")) {
    Serial.println(F("PONG")); return;
  }
  if (0 == strcmp(tok, "get")) {
    p = next_tok(p, &tok);
    if (0 == strcmp(tok, "mask")) {
      int d=0,l=0; ImageProviderGetMaskThresholds(&d,&l);
      Serial.printf("MASK: dark=%d lum=%d\r\n", d, l);
    } else if (0 == strcmp(tok, "ood")) {
      float a,b,c,d; ImageProviderGetOodThresholds(&a,&b,&c,&d);
      Serial.printf("OOD: sp=%.2f..%.2f mp=%.3f er=%.3f\r\n",
                    (double)a,(double)b,(double)c,(double)d);
    } else if (0 == strcmp(tok, "mode")) {
      Serial.printf("MODE: %s\r\n", op_mode_name(s_op_mode));
    } else {
      Serial.printf("ERR: unknown get '%s', try 'help'\r\n", tok);
    }
    return;
  }
  if (0 == strcmp(tok, "set")) {
    p = next_tok(p, &tok);
    if (0 == strcmp(tok, "mask")) {
      char *a = nullptr, *b = nullptr;
      p = next_tok(p, &a); p = next_tok(p, &b);
      if (!*a || !*b) { Serial.println(F("ERR: set mask <D> <L>, e.g. set mask 30 85")); return; }
      int d = atoi(a), l = atoi(b);
      ImageProviderSetMaskThresholds(d, l);
      int d2=0,l2=0; ImageProviderGetMaskThresholds(&d2,&l2);
      Serial.printf("OK  MASK: dark=%d lum=%d\r\n", d2, l2);
    } else if (0 == strcmp(tok, "ood")) {
      char *a=nullptr,*b=nullptr,*c=nullptr,*d=nullptr;
      p = next_tok(p, &a); p = next_tok(p, &b); p = next_tok(p, &c); p = next_tok(p, &d);
      if (!*a||!*b||!*c||!*d) { Serial.println(F("ERR: set ood <spmin> <spmax> <mpmin> <ermax>")); return; }
      float fa=(float)atof(a), fb=(float)atof(b), fc=(float)atof(c), fd=(float)atof(d);
      ImageProviderSetOodThresholds(fa,fb,fc,fd);
      float a2,b2,c2,d2; ImageProviderGetOodThresholds(&a2,&b2,&c2,&d2);
      Serial.printf("OK  OOD: sp=%.2f..%.2f mp=%.3f er=%.3f\r\n",
                    (double)a2,(double)b2,(double)c2,(double)d2);
    } else {
      Serial.printf("ERR: unknown set '%s', try 'help'\r\n", tok);
    }
    return;
  }
  if (0 == strcmp(tok, "mode")) {
    p = next_tok(p, &tok);
    OpMode m;
    if      (0==strcmp(tok,"infer")||0==strcmp(tok,"inference")||0==strcmp(tok,"0")) m = OpMode::kInference;
    else if (0==strcmp(tok,"rgb") ||0==strcmp(tok,"1"))                                 m = OpMode::kCaptureRgb;
    else if (0==strcmp(tok,"gray")||0==strcmp(tok,"grey")||0==strcmp(tok,"2"))         m = OpMode::kCaptureGray;
    else if (0==strcmp(tok,"infergray")||0==strcmp(tok,"infer_gray")||0==strcmp(tok,"3")) m = OpMode::kInferGray;
    else if (0==strcmp(tok,"ext_rgb") ||0==strcmp(tok,"11")) m = OpMode::kExtCaptureRgb;
    else if (0==strcmp(tok,"ext_gray")||0==strcmp(tok,"12")) m = OpMode::kExtCaptureGray;
    else { Serial.printf("ERR: mode infer|rgb|gray|infergray|ext_rgb|ext_gray, got '%s'\r\n", tok); return; }
    s_op_mode = m;
    Serial.printf("OK  MODE: %s\r\n", op_mode_name(m));
    return;
  }
  Serial.printf("ERR: unknown command '%s', type 'help'\r\n", tok);
}

static void debug_serial_task(void *arg) {
  (void)arg;
  enum { SYNC0, SYNC1, SYNC2, CMD, LEN, PAY, SUM };
  static uint8_t st = SYNC0;
  static uint8_t buf[5 + 255 + 1];   // hdr 5 + payload 255 + checksum 1
  static uint8_t idx = 0;
  static uint8_t pln = 0;

  static char ascii_buf[256];
  static uint8_t ascii_len = 0;

  Serial.printf("DEBUG: task up @ %u baud — type 'help' for Serial Monitor commands; binary sync=AA 55 CC\r\n", (unsigned)kDebugBaud);

  for (;;) {
    while (Serial.available() > 0) {
      const uint8_t b = (uint8_t)Serial.read();

      // ── ASCII line route (Serial Monitor, human typed) ────────────
      if (st == SYNC0 && b != kDebugSync0) {
        if (b == '\r' || b == '\n') {
          if (ascii_len) {
            ascii_buf[ascii_len] = '\0';
            ascii_dispatch(ascii_buf);
            ascii_len = 0;
          }
          continue;
        }
        if (ascii_len < (uint8_t)(sizeof(ascii_buf)-1)) {
          ascii_buf[ascii_len++] = (char)b;
        }
        continue;
      }

      // ── Binary packet route (Python/script, sync-framed) ──────────
      switch (st) {
        case SYNC0: st = (b == kDebugSync0) ? SYNC1 : SYNC0; continue;
        case SYNC1:
          if (b == kDebugSync1) { st = SYNC2; buf[0]=kDebugSync0; buf[1]=kDebugSync1; idx=2; }
          else if (b == kDebugSync0) { st = SYNC1; }
          else st = SYNC0;
          continue;
        case SYNC2:
          if (b == kDebugSync2) { buf[idx++] = b; st = CMD; }
          else if (b == kDebugSync0) { st = SYNC1; }
          else st = SYNC0;
          continue;
        case CMD:  buf[idx++] = b; st = LEN; continue;
        case LEN:  pln = b; buf[idx++] = b; st = (pln == 0) ? SUM : PAY; continue;
        case PAY:  buf[idx++] = b; if (idx - 5 >= pln) st = SUM; continue;
        case SUM:  break;   // handled below
      }

      // ── SUM state reached: validate checksum then dispatch ──────────
      const uint8_t calc = debug_xor(buf, (size_t)(5 + pln));
      uint8_t cmd = buf[3];
      st = SYNC0;
      if (calc != b) {
        debug_send_error(cmd, "checksum");
        continue;
      }
      const uint8_t *pl = buf + 5;
      bool handled = true;
      switch (cmd) {
        case kCmdNop: {
          debug_send_ok(cmd);
        } break;
        case kCmdPing: {
          debug_send_ok(cmd);
        } break;
        case kCmdSetOodThresholds: {
          if (pln < 16) { handled = false; debug_send_error(cmd, "len16"); break; }
          float spmin = read_f32_le(pl + 0);
          float spmax = read_f32_le(pl + 4);
          float mpmin = read_f32_le(pl + 8);
          float ermax = read_f32_le(pl + 12);
          ImageProviderSetOodThresholds(spmin, spmax, mpmin, ermax);
          Serial.printf("DEBUG: OOD thresholds → sp=%.2f..%.2f mp=%.3f er=%.3f\n",
                        (double)spmin, (double)spmax, (double)mpmin, (double)ermax);
          debug_send_ok(cmd);
        } break;
        case kCmdGetOodThresholds: {
          uint8_t obuf[16]; float spmin,spmax,mpmin,ermax;
          ImageProviderGetOodThresholds(&spmin,&spmax,&mpmin,&ermax);
          write_f32_le(obuf+0, spmin);
          write_f32_le(obuf+4, spmax);
          write_f32_le(obuf+8, mpmin);
          write_f32_le(obuf+12, ermax);
          debug_send_ack(cmd, obuf, 16);
        } break;
        case kCmdSetMaskThresholds: {
          if (pln < 2) { handled = false; debug_send_error(cmd, "len2"); break; }
          int dark = (int)pl[0];
          int lum  = (int)pl[1];
          ImageProviderSetMaskThresholds(dark, lum);
          { int d=0,l=0; ImageProviderGetMaskThresholds(&d,&l);
            Serial.printf("DEBUG: Dark/Lum (mask thresholds) → dark=%d lum=%d\n", d, l); }
          debug_send_ok(cmd);
        } break;
        case kCmdGetMaskThresholds: {
          int d=0,l=0; ImageProviderGetMaskThresholds(&d,&l);
          uint8_t obuf[2] = { (uint8_t)(d&0xFF), (uint8_t)(l&0xFF) };
          debug_send_ack(cmd, obuf, 2);
        } break;
        case kCmdSetMode: {
          if (pln < 1) { handled = false; debug_send_error(cmd, "len1"); break; }
          uint8_t raw = pl[0];
          OpMode new_mode;
          switch (raw) {
            case 0:  new_mode = OpMode::kInference;      break;
            case 1:  new_mode = OpMode::kCaptureRgb;     break;
            case 2:  new_mode = OpMode::kCaptureGray;    break;
            case 3:  new_mode = OpMode::kInferGray;      break;
            case 11: new_mode = OpMode::kExtCaptureRgb;  break;
            case 12: new_mode = OpMode::kExtCaptureGray; break;
            default: handled = false; debug_send_error(cmd, "bad mode"); continue;
          }
          s_op_mode = new_mode;
          Serial.printf("DEBUG: mode → %s (0x%02X)\n", op_mode_name(new_mode), raw);
          debug_send_ok(cmd);
        } break;
        case kCmdGetMode: {
          uint8_t obuf[1] = { (uint8_t)s_op_mode };
          debug_send_ack(cmd, obuf, 1);
        } break;
        default: handled = false; debug_send_error(cmd, "bad cmd"); break;
      }
      (void)handled;
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

// ── Capture-mode frame dump helpers on Debug Serial ────────────────────
//   Frame format:
//     0xAA 0x55 0xAB kind frame_id_LE w_LE h_LE  <BYTES>  xor8
//   kind = 0x01 RGB24 / 0x02 GRAY8.  xor covers every byte BEFORE checksum.
static void dump_capture_frame_header(uint8_t kind, uint16_t frame_id,
                                       uint16_t w, uint16_t h, size_t bytes) {
  uint8_t hdr[3 + 1 + 2 + 2 + 2];   // 3 sync + kind + fid + w + h
  hdr[0] = kCapSync0;  hdr[1] = kCapSync1;  hdr[2] = kCapSync2;
  hdr[3] = kind;
  write_u16_le(hdr + 4, frame_id);
  write_u16_le(hdr + 6, w);
  write_u16_le(hdr + 8, h);
  uint8_t x = debug_xor(hdr, sizeof(hdr));
  (void)bytes;   // implied by w,h,kind; useful for logging
  Serial.write(hdr, sizeof(hdr));
}
static inline void dump_xor_final(uint8_t running_xor, const uint8_t *p, size_t n) {
  for (size_t i = 0; i < n; i++) running_xor ^= p[i];
  Serial.write(p, n);
  Serial.write(running_xor);
  Serial.flush();
}

// Human-readable label helper.  Returns the kCategoryLabels entry for a real
// class (id < kCategoryCount) and "No Sign" for the synthetic OOD class that
// we emit when the 3-layer gating believes there is no sign in frame.
static const char* label_name(uint8_t id) {
  if (id < (uint8_t)kCategoryCount) {
    const char *name = kCategoryLabels[id];
    return name ? name : "?";
  }
  return "No Sign";
}

// OOD / pick helpers use fixed 32-slot score stacks — models with more
// classes than that would silently overflow them.
static_assert(kCategoryCount <= 32, "kCategoryCount exceeds scores_raw[32]");

static void pick_label_and_confidence(TfLiteTensor *output, uint8_t *label_id, uint8_t *confidence) {
  uint8_t best_label = 0;
  int best_score = 0;
  int total_raw = 0;
  // For entropy we need the "probability distribution" approximation.  Since
  // int8 outputs might have the Softmax stripped during PTQ conversion, we
  // reconstruct softmax here over the raw uint8-scaled scores.  The raw
  // values are already interpretable because we scale into [0,255].
  int scores_raw[32];   // up to 32 classes (we never use >8)
  for (int i = 0; i < kCategoryCount; i++) {
    int score = 0;
    if (output->type == kTfLiteInt8) {
      score = (int)output->data.int8[i] + 128;
    } else if (output->type == kTfLiteUInt8) {
      score = (int)output->data.uint8[i];
    } else if (output->type == kTfLiteFloat32) {
      score = (int)(output->data.f[i] * 255.0f);
    }
    if (score < 0) score = 0;
    if (score > 255) score = 255;
    scores_raw[i] = score;
    total_raw += score;
    if (score > best_score) {
      best_score = score;
      best_label = (uint8_t)i;
    }
  }
  *label_id = best_label;
  *confidence = (uint8_t)best_score;
}

// Returns true iff the frame is "in-distribution" (we believe there's a real
// sign in the picture).  Combines (1) sign_mask percentage from the camera
// pipeline (Layer 1, pre-inference), (2) softmax top-1 probability
// (Layer 2), and (3) normalised entropy of the distribution (Layer 3).  Any
// rule that fires → OOD = false → caller should report "No Sign".
static bool ood_is_in_distribution(float sign_pct, TfLiteTensor *output,
                                   float *out_max_prob, float *out_entropy_ratio) {
#if OOD_ENABLE != 1
  if (out_max_prob)     *out_max_prob = 1.0f;
  if (out_entropy_ratio) *out_entropy_ratio = 0.0f;
  return true;
#else
  float min_pct = 0.0f, max_pct = 0.0f, max_prob_min = 0.0f, entropy_max = 0.0f;
  ImageProviderGetOodThresholds(&min_pct, &max_pct, &max_prob_min, &entropy_max);

  // Layer 1: sign-mask prior (cheapest; use first)
  if (min_pct > 0.0f && sign_pct < min_pct) { if (out_max_prob) *out_max_prob = 0.0f; if (out_entropy_ratio) *out_entropy_ratio = 1.0f; return false; }
  if (max_pct > 0.0f && sign_pct > max_pct) { if (out_max_prob) *out_max_prob = 0.0f; if (out_entropy_ratio) *out_entropy_ratio = 1.0f; return false; }

  // Aggregate raw scores for max_prob + entropy.  Use same [0,255] int-score
  // normalisation as pick_label_and_confidence to keep them consistent.
  int raw_total = 0;
  int raw_best = 0;
  int raw_arr[32];
  for (int i = 0; i < kCategoryCount; i++) {
    int v = 0;
    if (output->type == kTfLiteInt8) v = (int)output->data.int8[i] + 128;
    else if (output->type == kTfLiteUInt8) v = (int)output->data.uint8[i];
    else if (output->type == kTfLiteFloat32) v = (int)(output->data.f[i] * 255.0f);
    if (v < 0) v = 0;
    if (v > 255) v = 255;
    raw_arr[i] = v;
    raw_total += v;
    if (v > raw_best) raw_best = v;
  }
  if (raw_total <= 0) raw_total = 1;

  float max_prob = (float)raw_best / (float)raw_total;
  if (out_max_prob) *out_max_prob = max_prob;

  // Layer 2: max_prob minimum confidence
  if (max_prob_min > 0.0f && max_prob < max_prob_min) {
    if (out_entropy_ratio) *out_entropy_ratio = 1.0f;
    return false;
  }

  // Layer 3: normalised entropy (0..1).  Use natural log; normalise by ln(N).
  float entropy = 0.0f;
  for (int i = 0; i < kCategoryCount; i++) {
    float p = (float)raw_arr[i] / (float)raw_total;
    if (p <= 1e-6f) continue;
    entropy -= p * logf(p);
  }
  float max_entropy = (kCategoryCount > 1) ? logf((float)kCategoryCount) : 1.0f;
  if (max_entropy < 1e-6f) max_entropy = 1.0f;
  float ent_ratio = entropy / max_entropy;
  if (ent_ratio > 1.0f) ent_ratio = 1.0f;
  if (out_entropy_ratio) *out_entropy_ratio = ent_ratio;

  if (entropy_max > 0.0f && ent_ratio > entropy_max) return false;
  return true;
#endif  // OOD_ENABLE == 1
}

static void inference_task(void *arg) {
  (void)arg;
  uint16_t frame_id = 0;
  uint32_t sd_dropped = 0;
  static OpMode s_last_mode = OpMode::kInference;

  // Per-mode one-time state so we can re-initialise when the user flips
  // between INFERENCE and a CAPTURE mode, or across CAPTURE_A ↔ CAPTURE_B.
  static bool s_caprgb_ready   = false;
  static bool s_caprgb_banner  = false;
  static bool s_capgray_banner = false;
  static bool s_infergray_banner = false;
  static bool s_extrgb_ready   = false;
  static bool s_extrgb_banner  = false;
  static bool s_extgray_banner = false;

  // Timing accumulators (microseconds)
  uint64_t total_capture_us = 0;
  uint64_t total_invoke_us = 0;
  uint64_t total_loop_us = 0;
  uint32_t timed_frames = 0;

  for (;;) {
    // Snapshot runtime mode once per iteration so behaviour stays consistent
    // even if debug_serial_task writes to the volatile between checks.
    const OpMode mode = s_op_mode;
    if (mode != s_last_mode) {
      // Suppress the transition banner when the NEW mode streams binary
      // frames on Serial — the text would be injected at the stream start.
      if (!serial_stream_clean()) {
        Serial.printf("MODE: %s → %s  [frame=%u]\r\n",
                      op_mode_name(s_last_mode), op_mode_name(mode), (unsigned)frame_id);
        Serial.flush();
      }
      // On any mode transition: reset "consecutive" timing so the first
      // batch of CAPTURE frames doesn't skew averages wrong.
      total_capture_us = 0; total_invoke_us = 0; total_loop_us = 0;
      timed_frames = 0;
      // Reset per-mode ready/banner flags so the user can bounce between
      // capture modes (e.g. mode rgb → mode infer → mode rgb) and the
      // camera re-initialises + banner prints correctly every time.
      s_caprgb_ready   = false;
      s_caprgb_banner  = false;
      s_capgray_banner = false;
      s_infergray_banner = false;
      s_extrgb_ready   = false;
      s_extrgb_banner  = false;
      s_extgray_banner = false;
      s_last_mode = mode;
    }

    // ─────────────────────────────────────────────────────────
    // Plain mode kCaptureRgb — AItraining default capture panel format.
    // Wire: AA 55 AA  +  96×96×3 RGB24  (no metadata, no checksum)
    // This matches AItraining's SerialFrameReader (default sync=AA 55 AA,
    // channels=3) exactly — so data capture panel → source capture →
    // training set works without app changes.
    // ─────────────────────────────────────────────────────────
    if (mode == OpMode::kCaptureRgb) {
      if (!s_caprgb_ready) {
        // Begin-once guard: the library begin() is NOT re-entrant — a
        // second call on a runtime mode switch can hang the I2C bus.
        s_caprgb_ready = ImageProviderEnsureCamera();
        if (!s_caprgb_ready) {
          Serial.println("CAPTURE_RGB/plain: CameraBegin failed");
          Serial.flush();
          vTaskDelay(pdMS_TO_TICKS(500));
          continue;
        }
      }
      if (!s_caprgb_banner) {
        Serial.printf("CAPTURE_RGB/plain: stream sync=AA 55 AA + %dx%dx3 (AItraining)\r\n",
                      IMG_SIZE, IMG_SIZE);
        Serial.flush();
        s_caprgb_banner = true;
      }
      bool got_frame = false;
      for (int tries = 0; tries < 100 && !got_frame; tries++) {
        if (CameraUpdate()) got_frame = true;
        else delayMicroseconds(2000);
      }
      if (!got_frame) { vTaskDelay(pdMS_TO_TICKS(2)); continue; }
      frame_id++;

      // IMG_SIZE-sized copy (OV5647's 160×160 library buffer must be
      // resized first — streaming its raw prefix tears the frame).
      const uint8_t *rgb = CameraGetRgbImgSized();
      const size_t bytes = (size_t)IMG_SIZE * (size_t)IMG_SIZE * 3;

      static const uint8_t sync[3] = { kCapSync0, kCapSync1, kCapSync2 };  // AA 55 AA
      Serial.write(sync, 3);
      Serial.write(rgb, bytes);
      Serial.flush();

      taskYIELD();
      continue;
    }

    // ─────────────────────────────────────────────────────────
    // Plain mode kCaptureGray — AItraining "upper" 96×96 grayscale default.
    // Wire: AA 55 AA  +  96×96×1 GRAY8  (int8 tensor + 128 → uint8)
    // Matches AItraining's SerialFrameReader default sync + channels=1
    // exactly → live serial preview / source capture work out of the box.
    // ─────────────────────────────────────────────────────────
    if (mode == OpMode::kCaptureGray) {
      if (!input || !interpreter) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }
      if (!s_capgray_banner) {
        Serial.printf("CAPTURE_GRAY/plain: stream sync=AA 55 AA + %dx%dx1 GRAY8 (AItraining tensor input)\r\n",
                      IMG_SIZE, IMG_SIZE);
        Serial.flush();
        s_capgray_banner = true;
      }

      uint64_t t_capture_start = esp_timer_get_time();
      if (kTfLiteOk != GetImage(error_reporter, OUT_WIDTH, OUT_HEIGHT, 1, input->data.int8)) {
        vTaskDelay(pdMS_TO_TICKS(1));
        continue;
      }
      const uint64_t t_capture_us = esp_timer_get_time() - t_capture_start;
      total_capture_us += t_capture_us;
      frame_id++;

      static uint8_t gray[IMG_SIZE * IMG_SIZE];
      for (size_t i = 0; i < (size_t)IMG_SIZE * (size_t)IMG_SIZE; i++) {
        gray[i] = (uint8_t)((int)input->data.int8[i] + 128);
      }

      static const uint8_t sync[3] = { kCapSync0, kCapSync1, kCapSync2 };  // AA 55 AA
      Serial.write(sync, 3);
      Serial.write(gray, sizeof(gray));
      Serial.flush();

      total_loop_us += (esp_timer_get_time() - t_capture_start) + t_capture_us;
      timed_frames++;
      const float sign_pct = ImageProviderLastSignPct();
      if ((frame_id % 30) == 0 && timed_frames > 0) {
        const uint32_t avg_cap = (uint32_t)(total_capture_us / timed_frames);
        Serial.printf("CAPTURE_GRAY/plain frame=%u sign=%.1f%% avg_cap=%lums (metadata only, not sent on stream)\r\n",
                      (unsigned)frame_id, (double)sign_pct, (unsigned long)(avg_cap/1000));
        Serial.flush();
      }
      taskYIELD();
      continue;
    }

    // ─────────────────────────────────────────────────────────
    // Extended mode kExtCaptureRgb — 脚本工具专用，带 metadata + checksum。
    // Wire: AA 55 AB  kind(1)  fid_LE(2)  w_LE(2)  h_LE(2)  <RGB bytes>  xor8
    // kind=0x01=RGB24, xor8 覆盖 header + pixel 部分，不包含 checksum byte。
    // 学生/默认 AItraining 不需要用这个模式。
    // ─────────────────────────────────────────────────────────
    if (mode == OpMode::kExtCaptureRgb) {
      if (!s_extrgb_ready) {
        // Begin-once guard: see kCaptureRgb above.
        s_extrgb_ready = ImageProviderEnsureCamera();
        if (!s_extrgb_ready) {
          Serial.println("CAPTURE_RGB/extended: CameraBegin failed");
          Serial.flush();
          vTaskDelay(pdMS_TO_TICKS(500));
          continue;
        }
      }
      if (!s_extrgb_banner) {
        Serial.printf("CAPTURE_RGB/extended: stream sync=AA 55 AB + kind/fid/w/h + pixels + xor8\r\n");
        Serial.flush();
        s_extrgb_banner = true;
      }
      bool got_frame = false;
      for (int tries = 0; tries < 100 && !got_frame; tries++) {
        if (CameraUpdate()) got_frame = true;
        else delayMicroseconds(2000);
      }
      if (!got_frame) { vTaskDelay(pdMS_TO_TICKS(2)); continue; }
      frame_id++;
      const uint8_t *rgb = CameraGetRgbImgSized();  // IMG_SIZE-sized copy (see kCaptureRgb)
      const size_t bytes = (size_t)IMG_SIZE * (size_t)IMG_SIZE * 3;
      uint8_t hdr[3 + 1 + 2 + 2 + 2];
      hdr[0] = kCapSync0;  hdr[1] = kCapSync1;  hdr[2] = kCapExtSync2;
      hdr[3] = kCapKindRgb;
      write_u16_le(hdr + 4, frame_id);
      write_u16_le(hdr + 6, (uint16_t)IMG_SIZE);
      write_u16_le(hdr + 8, (uint16_t)IMG_SIZE);
      uint8_t x = debug_xor(hdr, sizeof(hdr));
      Serial.write(hdr, sizeof(hdr));
      for (size_t i = 0; i < bytes; i++) x ^= rgb[i];
      Serial.write(rgb, bytes);
      Serial.write(x);
      Serial.flush();
      taskYIELD();
      continue;
    }

    // ─────────────────────────────────────────────────────────
    // Extended mode kExtCaptureGray — 同上 GRAY8。
    // ─────────────────────────────────────────────────────────
    if (mode == OpMode::kExtCaptureGray) {
      if (!input || !interpreter) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }
      if (!s_extgray_banner) {
        Serial.printf("CAPTURE_GRAY/extended: stream sync=AA 55 AB + kind/fid/w/h + GRAY8 + xor8\r\n");
        Serial.flush();
        s_extgray_banner = true;
      }
      const uint64_t t_capture_start = esp_timer_get_time();
      if (kTfLiteOk != GetImage(error_reporter, OUT_WIDTH, OUT_HEIGHT, 1, input->data.int8)) {
        vTaskDelay(pdMS_TO_TICKS(1));
        continue;
      }
      const uint64_t t_capture_us = esp_timer_get_time() - t_capture_start;
      total_capture_us += t_capture_us;
      frame_id++;
      static uint8_t gray[IMG_SIZE * IMG_SIZE];
      for (size_t i = 0; i < (size_t)IMG_SIZE * (size_t)IMG_SIZE; i++) {
        gray[i] = (uint8_t)((int)input->data.int8[i] + 128);
      }
      uint8_t hdr[3 + 1 + 2 + 2 + 2];
      hdr[0] = kCapSync0;  hdr[1] = kCapSync1;  hdr[2] = kCapExtSync2;
      hdr[3] = kCapKindGray;
      write_u16_le(hdr + 4, frame_id);
      write_u16_le(hdr + 6, (uint16_t)IMG_SIZE);
      write_u16_le(hdr + 8, (uint16_t)IMG_SIZE);
      uint8_t x = debug_xor(hdr, sizeof(hdr));
      Serial.write(hdr, sizeof(hdr));
      const size_t bytes = sizeof(gray);
      for (size_t i = 0; i < bytes; i++) x ^= gray[i];
      Serial.write(gray, bytes);
      Serial.write(x);
      Serial.flush();
      total_loop_us += (esp_timer_get_time() - t_capture_start) + t_capture_us;
      timed_frames++;
      const float sign_pct = ImageProviderLastSignPct();
      if ((frame_id % 30) == 0 && timed_frames > 0) {
        const uint32_t avg_cap = (uint32_t)(total_capture_us / timed_frames);
        Serial.printf("CAPTURE_GRAY/extended frame=%u sign=%.1f%% avg_cap=%lums\r\n",
                      (unsigned)frame_id, (double)sign_pct, (unsigned long)(avg_cap/1000));
        Serial.flush();
      }
      taskYIELD();
      continue;
    }

    // ─────────────────────────────────────────────────────────
    // kInference mode (default).
    // ─────────────────────────────────────────────────────────
#if PREPROCESS_MODE == PREPROCESS_MODE_RGB
    // ── RGB data-collection mode (compile-time switch) ──
    static bool s_stream_ready = false;
    if (!s_stream_ready) {
      s_stream_ready = CameraBegin();
      if (!s_stream_ready) {
        Serial.println("CameraBegin failed");
        vTaskDelay(pdMS_TO_TICKS(1000));
        continue;
      }
      Serial.printf("RGB stream mode: 0xAA 0x55 0xAA + %dx%dx3\n", IMG_SIZE, IMG_SIZE);
    }
    for (int tries = 0; tries < 100; tries++) {
      if (CameraUpdate()) {
        CameraSendRgbToSerialWb(200, 200);  // fixed ×2.0 WB, matches AItraining
        break;
      }
      delayMicroseconds(2000);
    }
    taskYIELD();
    continue;
#else
    if (!input || !interpreter) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    uint64_t t_loop_start = esp_timer_get_time();

    uint64_t t_capture_start = esp_timer_get_time();
    if (kTfLiteOk != GetImage(error_reporter, OUT_WIDTH, OUT_HEIGHT, kNumChannels, input->data.int8)) {
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }
#endif  // PREPROCESS_MODE == RGB
    uint64_t t_capture_us = esp_timer_get_time() - t_capture_start;
    frame_id++;

    // ── Snapshot the model input BEFORE Invoke ──────────────────────────
    // CRITICAL: interpreter->Invoke() may reuse the input tensor's arena
    // memory for intermediate tensors, so input->data.int8 is only
    // guaranteed valid until Invoke is called.  The kInferGray stream and
    // the SD logger must read this snapshot — reading input afterwards
    // produced deterministic moiré garbage (conv activation scratch data).
    static int8_t input_snapshot[IMG_SIZE * IMG_SIZE];
    memcpy(input_snapshot, input->data.int8, sizeof(input_snapshot));

    if (mode == OpMode::kInferGray) {
      if (!s_infergray_banner) {
        Serial.printf("INFER_GRAY: stream sync=AA 55 AA + %dx%dx1 GRAY8 (model input) — UART→S3 still active\r\n",
                      IMG_SIZE, IMG_SIZE);
        Serial.flush();
        s_infergray_banner = true;
      }
      // Same wire format as kCaptureGray: AA 55 AA + 96×96×1 GRAY8.
      static uint8_t gray[IMG_SIZE * IMG_SIZE];
      for (size_t i = 0; i < (size_t)IMG_SIZE * (size_t)IMG_SIZE; i++) {
        gray[i] = (uint8_t)((int)input_snapshot[i] + 128);
      }
      static const uint8_t sync[3] = { kCapSync0, kCapSync1, kCapSync2 };  // AA 55 AA
      Serial.write(sync, 3);
      Serial.write(gray, sizeof(gray));
      Serial.flush();
    }

    uint64_t t_invoke_start = esp_timer_get_time();
    if (kTfLiteOk != interpreter->Invoke()) {
      Serial.println("Invoke failed");
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }
    uint64_t t_invoke_us = esp_timer_get_time() - t_invoke_start;

    // Accumulate timing
    total_capture_us += t_capture_us;
    total_invoke_us += t_invoke_us;
    total_loop_us += esp_timer_get_time() - t_loop_start;
    timed_frames++;

    TfLiteTensor* output = interpreter->output(0);
    uint8_t label_id = 0;
    uint8_t confidence = 0;
    pick_label_and_confidence(output, &label_id, &confidence);

    // ── OOD rejection (layers 1+2+3) ─────────────────────────────────────
    // Softmax hallucinates high confidence on any input; we combine a
    // pre-inference sign-mask percentage (cheap!) with post-inference
    // max_prob + normalised-entropy to suppress "No Sign" frames.
    //
    // Signalling contract: flags is the ONLY discriminator between a real
    // sign and "No Sign".  Receivers MUST check:
    //   (flags & 0xF0) == 0xF0  →  No Sign   (ignore label_id / confidence)
    //   (flags & 0xF0) == 0x00  →  Real sign (use   label_id / confidence)
    //
    // label_id / confidence are kept unmodified on OOD (they hold the
    // softmax hallucination) so offline CSV analysis can compare what the
    // model *would* have said vs. what OOD rejected.
    float sign_pct = ImageProviderLastSignPct();
    float max_prob = 0.0f, entropy_ratio = 1.0f;
    bool in_distribution = ood_is_in_distribution(sign_pct, output, &max_prob, &entropy_ratio);
    bool ood_suppressed = !in_distribution;

    uint8_t pkt_flags = 2;  // sign_ready (junction mode removed)
    if (ood_suppressed) {
      pkt_flags = (uint8_t)((pkt_flags & 0x0F) | 0xF0);  // high nibble 0xF = No Sign
    }

    // Debug: print raw output values for first 10 frames to diagnose model output
    if (mode == OpMode::kInference && (frame_id <= 10 || (frame_id % 30) == 0)) {
      Serial.print("  ood: sign_pct=");
      Serial.print(sign_pct, 1);
      Serial.print("% max_prob=");
      Serial.print(max_prob, 3);
      Serial.print(" entropy=");
      Serial.print(entropy_ratio, 3);
      {
        int cb_x1 = 0, cb_y1 = 0, cb_side = 0;
        ImageProviderLastCropBox(&cb_x1, &cb_y1, &cb_side);
        Serial.print(" box=[");
        Serial.print(cb_x1); Serial.print(",");
        Serial.print(cb_y1); Serial.print(",");
        Serial.print(cb_side); Serial.print("]");
      }
      Serial.print(" → ");
      Serial.print(ood_suppressed ? "SUPPRESS(No Sign)" : "OK");
      Serial.print("  raw=[");
      for (int ri = 0; ri < kCategoryCount; ri++) {
        int raw_score = 0;
        if (output->type == kTfLiteInt8)       raw_score = (int)output->data.int8[ri] + 128;
        else if (output->type == kTfLiteUInt8) raw_score = (int)output->data.uint8[ri];
        else if (output->type == kTfLiteFloat32) raw_score = (int)(output->data.f[ri] * 255.0f);
        Serial.print(raw_score);
        if (ri < kCategoryCount - 1) Serial.print(",");
      }
      Serial.print("] best=");
      Serial.print((int)label_id);
      Serial.print(":");
      Serial.print(confidence);
      Serial.print(" (");
      if (ood_suppressed) Serial.print("No Sign");
      else                Serial.print(label_name(label_id));
      Serial.println(")");
    }

    if (mode == OpMode::kInference || mode == OpMode::kInferGray) {
      if (kEnableSdLogger && s_sd_queue && !s_sd_full && (frame_id % kSaveEveryNFrames) == 0) {
        // Take a FREE buffer first (see s_sd_free_queue comment) — never
        // overwrite a buffer the sd task may still be writing.
        uint8_t buf_idx = 0;
        if (!s_sd_free_queue || xQueueReceive(s_sd_free_queue, &buf_idx, 0) != pdTRUE) {
          sd_dropped++;
        } else {
          uint8_t *dst = s_sd_buffers[buf_idx];
          // Copy from the pre-Invoke snapshot — input->data.int8 is invalid
          // after Invoke (arena reuse), it would save moiré scratch data.
          for (size_t i = 0; i < kImageBytes; i++) {
            dst[i] = (uint8_t)((int)input_snapshot[i] + 128);
          }

          SdPacket sp;
          sp.frame_id = frame_id;
          sp.buffer_index = buf_idx;
          sp.flags = pkt_flags;
          sp.label_id = label_id;
          sp.confidence = confidence;
          if (xQueueSend(s_sd_queue, &sp, 0) != pdTRUE) {
            // sd task lagging behind — give the buffer back immediately.
            xQueueSend(s_sd_free_queue, &buf_idx, 0);
            sd_dropped++;
          }
        }
      }

      if (s_uart_queue) {
        UartPacket pkt;
        pkt.frame_id = frame_id;
        pkt.label_id = label_id;
        pkt.confidence = confidence;
        pkt.flags = pkt_flags;
        xQueueOverwrite(s_uart_queue, &pkt);
      }
    }

    // NOTE: kInferGray streaming moved BEFORE interpreter->Invoke() — the
    // arena planner may reuse the input tensor's memory for intermediate
    // tensors during Invoke, so reading input->data.int8 afterwards yields
    // structured garbage (moiré).  See the pre-Invoke block above.

    if (mode == OpMode::kInference && (frame_id % 1) == 0 && timed_frames > 0) {
      uint32_t avg_cap = (uint32_t)(total_capture_us / timed_frames);
      uint32_t avg_inv = (uint32_t)(total_invoke_us / timed_frames);
      uint32_t avg_loop = (uint32_t)(total_loop_us / timed_frames);
      Serial.print("frame=");
      Serial.print(frame_id);
      Serial.print(" label=");
      Serial.print(label_id);
      Serial.print(" (");
      if (ood_suppressed) Serial.print("No Sign");
      else                Serial.print(label_name(label_id));
      Serial.print(") conf=");
      Serial.print(confidence);
      Serial.print(ood_suppressed ? " OOD" : " ID ");
      Serial.print(" sign=");
      Serial.print(sign_pct, 1);
      Serial.print("% flags=0x");
      Serial.print(pkt_flags, 16);
      Serial.print(" | cap=");
      Serial.print(avg_cap / 1000);
      Serial.print("ms inv=");
      Serial.print(avg_inv / 1000);
      Serial.print("ms loop=");
      Serial.print(avg_loop / 1000);
      Serial.print("ms fps≈");
      Serial.print(avg_loop > 0 ? 1000000UL / avg_loop : 0);
      Serial.print(" tx=");
      Serial.print(s_transmit_enabled ? "ON" : "OFF");
      Serial.print(" rx_bytes=");
      Serial.print((unsigned long)s_rx_bytes);
      if (kEnableSdLogger) {
        Serial.print(" sd_drop=");
        Serial.print(sd_dropped);
      }
      Serial.println();
      total_capture_us = 0;
      total_invoke_us = 0;
      total_loop_us = 0;
      timed_frames = 0;
    }

    taskYIELD();
  }
}



// The name of this function is important for Arduino compatibility.
void setup() {
  // Set up logging. Google style is to avoid globals or statics because of
  // lifetime uncertainty, but since this has a trivial destructor it's okay.
  // NOLINTNEXTLINE(runtime-global-variables)
  static tflite::MicroErrorReporter micro_error_reporter;
  error_reporter = &micro_error_reporter;

  Serial.begin(kDebugBaud);
  delay(100);
  Serial.println("P4 TFLite start");
#if PREPROCESS_MODE == PREPROCESS_MODE_BG
  Serial.printf("Preprocess: B-G  | Camera: %s\n", CameraGetName());
#elif PREPROCESS_MODE == PREPROCESS_MODE_GRAY
  Serial.printf("Preprocess: GRAY (BT.601)  | Camera: %s\n", CameraGetName());
#elif PREPROCESS_MODE == PREPROCESS_MODE_RGB
  Serial.printf("Preprocess: RGB stream (data collection)  | Camera: %s\n", CameraGetName());
#endif

  pinMode(kHandshakePwmPin, OUTPUT);
  analogWrite(kHandshakePwmPin, kHandshakePwmDuty);

  if (kEnableSdLogger) {
    if (kUseSdMmcCustomPins) {
      if (!SD_MMC.setPins(kSdClkPin, kSdCmdPin, kSdD0Pin, kSdD1Pin, kSdD2Pin, kSdD3Pin)) {
        Serial.println("SD_MMC setPins failed");
      }
    }

    if (!mount_sd()) {
      Serial.println("SD mount failed");
    } else if (s_sd_using_ffat) {
      if (!make_run_dir()) {
        Serial.println("Create run dir failed");
      } else {
        Serial.print("Run dir: ");
        Serial.println(s_run_dir);
        s_sd_queue = xQueueCreate(kSdQueueDepth, sizeof(SdPacket));
        s_sd_free_queue = xQueueCreate(kSdQueueDepth, sizeof(uint8_t));
        for (uint8_t i = 0; i < kSdQueueDepth; i++) xQueueSend(s_sd_free_queue, &i, 0);
      }
    } else if (s_sd_using_sd_mmc) {
      if (SD_MMC.cardType() == CARD_NONE) {
        Serial.println("No SD card");
      } else if (!make_run_dir()) {
        Serial.println("Create run dir failed");
      } else {
        Serial.print("Run dir: ");
        Serial.println(s_run_dir);
        s_sd_queue = xQueueCreate(kSdQueueDepth, sizeof(SdPacket));
        s_sd_free_queue = xQueueCreate(kSdQueueDepth, sizeof(uint8_t));
        for (uint8_t i = 0; i < kSdQueueDepth; i++) xQueueSend(s_sd_free_queue, &i, 0);
      }
    } else {
      Serial.println("Storage mount state invalid");
    }
  }

  // GPIO10/GPIO11 crosstalk mitigation (per repo CLAUDE.md): pull the RX
  // pin down BEFORE starting the UART so idle/noise on the S3 line can't
  // be misread as control packets (and vice versa on the S3 side).
  pinMode(kUartRxPin, INPUT_PULLDOWN);
  UartToS3.begin(kUartBaud, SERIAL_8N1, kUartRxPin, kUartTxPin);
  s_uart_queue = xQueueCreate(1, sizeof(UartPacket));

  // Map the model into a usable data structure. This doesn't involve any
  // copying or parsing, it's a very lightweight operation.
  model = tflite::GetModel(g_tm_model_data);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    TF_LITE_REPORT_ERROR(error_reporter,
                         "Model provided is schema version %d not equal "
                         "to supported version %d.",
                         model->version(), TFLITE_SCHEMA_VERSION);
    return;
  }

  #if TM_HAS_GENERATED_MODEL_RESOLVER
  static ModelOpResolver micro_op_resolver;
  static bool resolver_registered = false;
  if (!resolver_registered) {
    if (RegisterModelOps(micro_op_resolver) != kTfLiteOk) {
      TF_LITE_REPORT_ERROR(error_reporter, "RegisterModelOps() failed");
      return;
    }
    resolver_registered = true;
  }
  #else
  static tflite::MicroMutableOpResolver<10> micro_op_resolver;
  static bool resolver_registered = false;
  if (!resolver_registered) {
    if (micro_op_resolver.AddAveragePool2D() != kTfLiteOk) return;
    if (micro_op_resolver.AddMaxPool2D() != kTfLiteOk) return;
    if (micro_op_resolver.AddConv2D() != kTfLiteOk) return;
    if (micro_op_resolver.AddDepthwiseConv2D() != kTfLiteOk) return;
    if (micro_op_resolver.AddShape() != kTfLiteOk) return;
    if (micro_op_resolver.AddStridedSlice() != kTfLiteOk) return;
    if (micro_op_resolver.AddPack() != kTfLiteOk) return;
    if (micro_op_resolver.AddReshape() != kTfLiteOk) return;
    if (micro_op_resolver.AddSoftmax() != kTfLiteOk) return;
    if (micro_op_resolver.AddFullyConnected() != kTfLiteOk) return;
    resolver_registered = true;
  }
  #endif

  // Build an interpreter to run the model with.
  // NOLINTNEXTLINE(runtime-global-variables)
  // Try internal SRAM first (fast, low latency), fall back to PSRAM.
  // Internal SRAM gives ~10x memory bandwidth over PSRAM — critical for
  // inference speed. Reduce model size if it doesn't fit

  printf("SRAM free: %d total, %d largest block (requesting %d)\n",
      heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
      heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
      kTensorArenaSize);

  tensor_arena = (uint8_t*) heap_caps_aligned_alloc(
        16,
        kTensorArenaSize,
        MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT
    );
  const char *arena_location = "SRAM";

  if (tensor_arena == nullptr) {
    tensor_arena = (uint8_t*) heap_caps_aligned_alloc(
          16,
          kTensorArenaSize,
          MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
      );
    arena_location = "PSRAM (SRAM full — reduce model size for speed)";
  }

    if (tensor_arena == nullptr) {
        printf("Failed to allocate tensor arena!\n");
        return;
    }
    printf("Tensor arena: %d bytes in %s\n", kTensorArenaSize, arena_location);

  static tflite::MicroInterpreter static_interpreter(
      model, micro_op_resolver, tensor_arena, kTensorArenaSize, nullptr, nullptr, false);
  interpreter = &static_interpreter;


  // Allocate memory from the tensor_arena for the model's tensors.
  TfLiteStatus allocate_status = interpreter->AllocateTensors();
  if (allocate_status != kTfLiteOk) {
    TF_LITE_REPORT_ERROR(error_reporter, "AllocateTensors() failed");
    return;
  }

  // Get information about the memory area to use for the model's input.
  input = interpreter->input(0);

  // --- Diagnostics ---
  Serial.print("Arena used: ");
  Serial.print(interpreter->arena_used_bytes());
  Serial.print(" / ");
  Serial.print(kTensorArenaSize);
  Serial.print(" (");
  Serial.print((int)(interpreter->arena_used_bytes() * 100LL / kTensorArenaSize));
  Serial.println("%)");

  Serial.print("Input: ");
  Serial.print(input->dims->data[1]);
  Serial.print("x");
  Serial.print(input->dims->data[2]);
  Serial.print("x");
  Serial.print(input->dims->data[3]);
  Serial.print(" type=");
  Serial.println(input->type == kTfLiteInt8 ? "int8" : input->type == kTfLiteFloat32 ? "float32" : "other");

  {
    TfLiteTensor* out = interpreter->output(0);
    Serial.print("Output: ");
    Serial.print(out->dims->data[1]);
    Serial.print(" type=");
    Serial.print(out->type == kTfLiteInt8 ? "int8" : out->type == kTfLiteUInt8 ? "uint8" : out->type == kTfLiteFloat32 ? "float32" : "other");
    if (out->type == kTfLiteInt8) {
      Serial.print(" q=(");
      Serial.print(out->params.scale);
      Serial.print(",");
      Serial.print(out->params.zero_point);
      Serial.print(")");
    }
    Serial.println();
  }

  Serial.print("Ops: ");
  auto subgraph = model->subgraphs()->Get(0);
  auto opcodes = model->operator_codes();
  for (size_t i = 0; i < subgraph->operators()->size(); i++) {
    auto op = subgraph->operators()->Get(i);
    auto opcode = opcodes->Get(op->opcode_index());
    if (opcode->builtin_code() == tflite::BuiltinOperator_CONV_2D) Serial.print("Conv2D ");
    else if (opcode->builtin_code() == tflite::BuiltinOperator_MAX_POOL_2D) Serial.print("MaxPool ");
    else if (opcode->builtin_code() == tflite::BuiltinOperator_FULLY_CONNECTED) Serial.print("FC ");
    else if (opcode->builtin_code() == tflite::BuiltinOperator_SOFTMAX) Serial.print("Softmax ");
    else if (opcode->builtin_code() == tflite::BuiltinOperator_RESHAPE) Serial.print("Reshape ");
    else Serial.print("op#"); Serial.print(opcode->builtin_code()); Serial.print(" ");
  }
  Serial.println();

#if defined(ESP_NN)
  Serial.println("ESP-NN: ENABLED (optimized kernels)");
#else
  Serial.println("ESP-NN: NOT FOUND (using reference kernels — SLOW)");
  Serial.println("Install esp-nn library for 10-50x inference speedup.");
#endif
  // --- End Diagnostics ---

#if defined(portNUM_PROCESSORS) && (portNUM_PROCESSORS > 1)
  if (kEnableSdLogger && s_sd_queue) {
    xTaskCreatePinnedToCore(sd_task, "sd", kSdTaskStackBytes, nullptr, 1, nullptr, 0);
  }
  xTaskCreatePinnedToCore(uart_tx_task, "uart_tx", kUartTxTaskStackBytes, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(uart_rx_task, "uart_rx", kUartRxTaskStackBytes, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(debug_serial_task, "dbgser", kDebugCmdStackBytes, nullptr, 1, nullptr, 0);
  xTaskCreatePinnedToCore(inference_task, "tflm", kInferenceTaskStackBytes, nullptr, 3, nullptr, 1);
#else
  if (kEnableSdLogger && s_sd_queue) {
    xTaskCreate(sd_task, "sd", kSdTaskStackBytes, nullptr, 1, nullptr);
  }
  xTaskCreate(uart_tx_task, "uart_tx", kUartTxTaskStackBytes, nullptr, 2, nullptr);
  xTaskCreate(uart_rx_task, "uart_rx", kUartRxTaskStackBytes, nullptr, 2, nullptr);
  xTaskCreate(debug_serial_task, "dbgser", kDebugCmdStackBytes, nullptr, 1, nullptr);
  xTaskCreate(inference_task, "tflm", kInferenceTaskStackBytes, nullptr, 3, nullptr);
#endif
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
