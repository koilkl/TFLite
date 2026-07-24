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

// Globals, used for compatibility with Arduino-style sketches.
namespace {
tflite::ErrorReporter* error_reporter = nullptr;
const tflite::Model* model = nullptr;
tflite::MicroInterpreter* interpreter = nullptr;
TfLiteTensor* input = nullptr;

// State machine for junction/sign detection
// State 0 = junction mode (road/cross), State 1 = sign mode (sign classes)
// volatile: written by uart_rx_task (core 0) on RESUME_JUNCTION, read by
// inference_task (core 1) every frame — prevents the core-1 compiler from
// caching a stale value in a register.
static volatile int s_detection_state = 0;
static volatile int s_no_sign_frames = 0;
static constexpr int kNoSignThreshold = 10;

// Bi-directional control: S3 can request P4 to stop/start UART transmission.
// Inference continues regardless; only the TX path is gated.
static volatile bool s_transmit_enabled = true;

// RX diagnostics: count bytes and control packets received from S3.
static volatile uint32_t s_rx_bytes = 0;
static volatile uint32_t s_rx_ack_stop = 0;
static volatile uint32_t s_rx_resume_junction = 0;

static void uart_control_enable() {
  s_transmit_enabled = true;
  Serial.println("UART TX: ENABLED");
}

static void uart_control_disable() {
  s_transmit_enabled = false;
  Serial.println("UART TX: DISABLED (S3 ack)");
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
static constexpr uint8_t kCtrlAckStop        = 0x01;  // S3 confirmed sign → stop TX
static constexpr uint8_t kCtrlResumeJunction = 0x02;  // S3 done → resume TX + junction mode

static constexpr uint32_t kUartTxTaskStackBytes = 4 * 1024;
static constexpr uint32_t kUartRxTaskStackBytes = 4 * 1024;
static constexpr uint32_t kInferenceTaskStackBytes = 32 * 1024;
static constexpr uint32_t kSdTaskStackBytes = 8 * 1024;

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

// extern volatile int s_crop_mode;
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
  uint8_t buffer_index;
};

static QueueHandle_t s_sd_queue = nullptr;
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
constexpr int kTensorArenaSize = 375 * 1024; // 380KB — fits P4 sram_high block 
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
  while (true) {
    SdPacket pkt;
    if (xQueueReceive(s_sd_queue, &pkt, portMAX_DELAY) != pdTRUE) {
      continue;
    }

    if (s_sd_full) {
      continue;
    }

    char path[96];
    snprintf(path, sizeof(path), "%s/frame_%05u.pgm", s_run_dir, (unsigned)pkt.frame_id);
    bool ok = write_pgm(path, s_sd_buffers[pkt.buffer_index]);
    saved++;

    if (!ok && errno == ENOSPC) {
      s_sd_full = true;
      Serial.printf("Storage full (ENOSPC). Stop saving frames. Last=%s\n", path);
    }

    if ((saved % 10) == 0) {
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
        Serial.printf("UART RX: unknown msg_type 0x%02X (expected 0x%02X)\n",
                      rx_buf[2], kMsgTypeControl);
        continue;
      }

      // Verify checksum: XOR of bytes 0-3
      uint8_t expected_csum = calc_uart_checksum(rx_buf, 4);
      if (rx_buf[4] != expected_csum) {
        Serial.printf("UART RX: bad checksum (got 0x%02X expected 0x%02X)\n",
                      rx_buf[4], expected_csum);
        continue;
      }

      uint8_t cmd = rx_buf[3];

      if (cmd == kCtrlAckStop) {
        s_rx_ack_stop++;
        Serial.printf("UART RX: ACK_STOP from S3 (#%lu) — disabling TX\n",
                      (unsigned long)s_rx_ack_stop);
        uart_control_disable();
      } else if (cmd == kCtrlResumeJunction) {
        s_rx_resume_junction++;
        Serial.printf("UART RX: RESUME_JUNCTION from S3 (#%lu) — resuming TX + junction mode\n",
                      (unsigned long)s_rx_resume_junction);
        s_detection_state = 0;
        s_no_sign_frames = 0;
        uart_control_enable();
      } else {
        Serial.printf("UART RX: unknown control cmd 0x%02X\n", cmd);
      }
    }

    // Periodic heartbeat: show RX stats even when idle
    uint32_t now = millis();
    if (now - last_rx_log_ms >= 5000) {
      last_rx_log_ms = now;
      Serial.printf("UART RX: bytes=%lu ack_stop=%lu resume=%lu tx_enabled=%d\n",
                    (unsigned long)s_rx_bytes, (unsigned long)s_rx_ack_stop,
                    (unsigned long)s_rx_resume_junction, (int)s_transmit_enabled);
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

static void pick_label_and_confidence(TfLiteTensor *output, uint8_t *label_id, uint8_t *confidence, int detection_state) {
  uint8_t best_label = 0;
  int best_score = 0;

  for (int i = 0; i < kCategoryCount; i++) {
    int score = 0;
    if (output->type == kTfLiteInt8) {
      score = (int)output->data.int8[i] + 128;
    } else if (output->type == kTfLiteUInt8) {
      score = (int)output->data.uint8[i];
    } else if (output->type == kTfLiteFloat32) {
      score = (int)(output->data.f[i] * 255.0f);
    }

    // Per-class masking: in junction mode (0), only road classes (kClassTypes==1);
    // in sign mode (1), only sign classes (kClassTypes==0).
    // Classes not matching the current state have their score zeroed.
    uint8_t expected_type = (detection_state == 0) ? 1 : 0;
    if (i < kCategoryCount && kClassTypes[i] != expected_type) {
      score = 0;
    }

    if (score > best_score) {
      best_score = score;
      best_label = (uint8_t)i;
    }
  }

  *label_id = best_label;
  *confidence = (uint8_t)best_score;
}

static void inference_task(void *arg) {
  (void)arg;
  uint16_t frame_id = 0;
  uint32_t sd_dropped = 0;
  uint8_t next_sd_buffer = 0;

  // Timing accumulators (microseconds)
  uint64_t total_capture_us = 0;
  uint64_t total_invoke_us = 0;
  uint64_t total_loop_us = 0;
  uint32_t timed_frames = 0;

  for (;;) {
    if (!input || !interpreter) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    uint64_t t_loop_start = esp_timer_get_time();

    // Set crop mode based on current detection state
    SetCropMode(s_detection_state == 0 ? CROP_MODE_JUNCTION : CROP_MODE_SIGN);

    uint64_t t_capture_start = esp_timer_get_time();
    if (kTfLiteOk != GetImage(error_reporter, OUT_WIDTH, OUT_HEIGHT, kNumChannels, input->data.int8)) {
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }
    uint64_t t_capture_us = esp_timer_get_time() - t_capture_start;
    frame_id++;

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
    pick_label_and_confidence(output, &label_id, &confidence, s_detection_state);

    // Debug: print raw output values for first 10 frames to diagnose model output
    if (frame_id <= 10) {
      Serial.print("  raw=[");
      for (int ri = 0; ri < kCategoryCount; ri++) {
        int raw_score = 0;
        if (output->type == kTfLiteInt8)       raw_score = (int)output->data.int8[ri] + 128;
        else if (output->type == kTfLiteUInt8) raw_score = (int)output->data.uint8[ri];
        else if (output->type == kTfLiteFloat32) raw_score = (int)(output->data.f[ri] * 255.0f);
        Serial.print(raw_score);
        if (ri < kCategoryCount - 1) Serial.print(",");
      }
      Serial.print("] st=");
      Serial.print(s_detection_state);
      Serial.print(" best=");
      Serial.print(label_id);
      Serial.print(":");
      Serial.println(confidence);
    }

    // State machine: junction <-> sign detection
    if (s_detection_state == 0) {
      // Junction mode: if a "road" class is detected with confidence, switch to sign mode
      if (label_id < kCategoryCount && kClassTypes[label_id] == 1 && confidence > 32) {
        s_detection_state = 1;
        s_no_sign_frames = 0;
      }
    } else {
      // Sign mode: if a "sign" class is detected with confidence, stay; otherwise count down
      if (label_id < kCategoryCount && kClassTypes[label_id] == 0 && confidence > 32) {
        s_no_sign_frames = 0;
      } else {
        s_no_sign_frames++;
        if (s_no_sign_frames >= kNoSignThreshold) {
          s_detection_state = 0;  // Switch back to junction mode
        }
      }
    }

    if (kEnableSdLogger && s_sd_queue && !s_sd_full && (frame_id % kSaveEveryNFrames) == 0) {
      uint8_t *dst = s_sd_buffers[next_sd_buffer];
      for (size_t i = 0; i < kImageBytes; i++) {
        dst[i] = (uint8_t)((int)input->data.int8[i] + 128);
      }

      SdPacket sp;
      sp.frame_id = frame_id;
      sp.buffer_index = next_sd_buffer;
      if (xQueueSend(s_sd_queue, &sp, 0) == pdTRUE) {
        next_sd_buffer = (uint8_t)((next_sd_buffer + 1) % kSdQueueDepth);
      } else {
        sd_dropped++;
      }
    }

    if (s_uart_queue) {
      UartPacket pkt;
      pkt.frame_id = frame_id;
      pkt.label_id = label_id;
      pkt.confidence = confidence;
      pkt.flags = (uint8_t)(s_detection_state + 1);  // 1=junction_ready, 2=sign_ready
      xQueueOverwrite(s_uart_queue, &pkt);
    }

    if ((frame_id % 1) == 0 && timed_frames > 0) {
      uint32_t avg_cap = (uint32_t)(total_capture_us / timed_frames);
      uint32_t avg_inv = (uint32_t)(total_invoke_us / timed_frames);
      uint32_t avg_loop = (uint32_t)(total_loop_us / timed_frames);
      Serial.print("frame=");
      Serial.print(frame_id);
      Serial.print(" label=");
      Serial.print(label_id);
      Serial.print(" conf=");
      Serial.print(confidence);
      Serial.print(" | cap=");
      Serial.print(avg_cap / 1000);
      Serial.print("ms inv=");
      Serial.print(avg_inv / 1000);
      Serial.print("ms loop=");
      Serial.print(avg_loop / 1000);
      Serial.print("ms fps≈");
      Serial.print(avg_loop > 0 ? 1000000UL / avg_loop : 0);
      Serial.print("Chop mode:");
//    #define CROP_MODE_SIGN     0
//    #define CROP_MODE_JUNCTION 1
      if(s_crop_mode==CROP_MODE_JUNCTION)
      Serial.print("JUN");
      else
       Serial.print("SIGN");
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
      }
    } else {
      Serial.println("Storage mount state invalid");
    }
  }

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
  xTaskCreatePinnedToCore(inference_task, "tflm", kInferenceTaskStackBytes, nullptr, 3, nullptr, 1);
#else
  if (kEnableSdLogger && s_sd_queue) {
    xTaskCreate(sd_task, "sd", kSdTaskStackBytes, nullptr, 1, nullptr);
  }
  xTaskCreate(uart_tx_task, "uart_tx", kUartTxTaskStackBytes, nullptr, 2, nullptr);
  xTaskCreate(uart_rx_task, "uart_rx", kUartRxTaskStackBytes, nullptr, 2, nullptr);
  xTaskCreate(inference_task, "tflm", kInferenceTaskStackBytes, nullptr, 3, nullptr);
#endif
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
