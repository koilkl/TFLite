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
#include "person_detect_model_data.h"
#include "tensorflow/lite/micro/micro_log.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/system_setup.h"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <SD_MMC.h>
#include <FFat.h>
#include <driver/gpio.h>
#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>

// Globals, used for compatibility with Arduino-style sketches.
namespace {
tflite::ErrorReporter* error_reporter = nullptr;
const tflite::Model* model = nullptr;
tflite::MicroInterpreter* interpreter = nullptr;
TfLiteTensor* input = nullptr;

static HardwareSerial UartToS3(1);

static constexpr int kDebugBaud = 921600;
static constexpr int kUartBaud = 921600;
static constexpr int kUartRxPin = 38;
static constexpr int kUartTxPin = 37;

static constexpr uint8_t kSync0 = 0xAA;
static constexpr uint8_t kSync1 = 0x55;
static constexpr uint8_t kMsgTypeInference = 0x01;

static constexpr uint32_t kUartTxTaskStackBytes = 4 * 1024;
static constexpr uint32_t kInferenceTaskStackBytes = 32 * 1024;
static constexpr uint32_t kSdTaskStackBytes = 8 * 1024;

static constexpr uint32_t kSaveEveryNFrames = 10;
static constexpr int kImageWidth = 96;
static constexpr int kImageHeight = 96;
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
static constexpr StorageBackend kStorageBackend = StorageBackend::Auto;
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

// In order to use optimized tensorflow lite kernels, a signed int8_t quantized
// model is preferred over the legacy unsigned model format. This means that
// throughout this project, input images must be converted from unisgned to
// signed format. The easiest and quickest way to convert from unsigned to
// signed 8-bit integers is to subtract 128 from the unsigned value to get a
// signed value.

// An area of memory to use for input, output, and intermediate arrays.
constexpr int kTensorArenaSize = 136 * 1024;
static uint8_t tensor_arena[kTensorArenaSize];
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

    char path[96];
    snprintf(path, sizeof(path), "%s/frame_%05u.pgm", s_run_dir, (unsigned)pkt.frame_id);
    bool ok = write_pgm(path, s_sd_buffers[pkt.buffer_index]);
    saved++;

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

    uint8_t buf[8];
    buf[0] = kSync0;
    buf[1] = kSync1;
    buf[2] = kMsgTypeInference;
    buf[3] = (uint8_t)(pkt.frame_id & 0xFF);
    buf[4] = (uint8_t)((pkt.frame_id >> 8) & 0xFF);
    buf[5] = pkt.label_id;
    buf[6] = pkt.confidence;
    buf[7] = pkt.flags;
    UartToS3.write(buf, sizeof(buf));
  }
}

static void pick_label_and_confidence(TfLiteTensor *output, uint8_t *label_id, uint8_t *confidence) {
  uint8_t best_label = 0;
  uint8_t best_conf = 0;

  if (output->type == kTfLiteInt8) {
    int8_t s0 = output->data.int8[0];
    int8_t s1 = output->data.int8[1];
    best_label = (s1 > s0) ? 1 : 0;
    best_conf = (uint8_t)((int)output->data.int8[best_label] + 128);
  } else if (output->type == kTfLiteUInt8) {
    uint8_t s0 = output->data.uint8[0];
    uint8_t s1 = output->data.uint8[1];
    best_label = (s1 > s0) ? 1 : 0;
    best_conf = output->data.uint8[best_label];
  }

  *label_id = best_label;
  *confidence = best_conf;
}

static void inference_task(void *arg) {
  (void)arg;
  uint16_t frame_id = 0;
  uint32_t sd_dropped = 0;
  uint8_t next_sd_buffer = 0;

  for (;;) {
    if (!input || !interpreter) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    if (kTfLiteOk != GetImage(error_reporter, kNumCols, kNumRows, kNumChannels, input->data.int8)) {
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }
    frame_id++;

    if (kTfLiteOk != interpreter->Invoke()) {
      Serial.println("Invoke failed");
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }

    TfLiteTensor* output = interpreter->output(0);
    uint8_t label_id = 0;
    uint8_t confidence = 0;
    pick_label_and_confidence(output, &label_id, &confidence);

    if (kEnableSdLogger && s_sd_queue && (frame_id % kSaveEveryNFrames) == 0) {
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
      pkt.flags = 0;
      xQueueOverwrite(s_uart_queue, &pkt);
    }

    if ((frame_id % 10) == 0) {
      Serial.print("frame=");
      Serial.print(frame_id);
      Serial.print(" label=");
      Serial.print(label_id);
      Serial.print(" conf=");
      Serial.print(confidence);
      if (kEnableSdLogger) {
        Serial.print(" sd_drop=");
        Serial.println(sd_dropped);
      } else {
        Serial.println();
      }
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
  model = tflite::GetModel(g_person_detect_model_data);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    TF_LITE_REPORT_ERROR(error_reporter,
                         "Model provided is schema version %d not equal "
                         "to supported version %d.",
                         model->version(), TFLITE_SCHEMA_VERSION);
    return;
  }

  // Pull in only the operation implementations we need.
  // This relies on a complete list of all the ops needed by this graph.
  // An easier approach is to just use the AllOpsResolver, but this will
  // incur some penalty in code space for op implementations that are not
  // needed by this graph.
  //
  // tflite::AllOpsResolver resolver;
  // NOLINTNEXTLINE(runtime-global-variables)
  static tflite::MicroMutableOpResolver<6> micro_op_resolver;
  micro_op_resolver.AddAveragePool2D();
  micro_op_resolver.AddConv2D();
  micro_op_resolver.AddDepthwiseConv2D();
  micro_op_resolver.AddReshape();
  micro_op_resolver.AddSoftmax();
  micro_op_resolver.AddFullyConnected();

  // Build an interpreter to run the model with.
  // NOLINTNEXTLINE(runtime-global-variables)
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

#if defined(portNUM_PROCESSORS) && (portNUM_PROCESSORS > 1)
  if (kEnableSdLogger && s_sd_queue) {
    xTaskCreatePinnedToCore(sd_task, "sd", kSdTaskStackBytes, nullptr, 1, nullptr, 0);
  }
  xTaskCreatePinnedToCore(uart_tx_task, "uart_tx", kUartTxTaskStackBytes, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(inference_task, "tflm", kInferenceTaskStackBytes, nullptr, 3, nullptr, 1);
#else
  if (kEnableSdLogger && s_sd_queue) {
    xTaskCreate(sd_task, "sd", kSdTaskStackBytes, nullptr, 1, nullptr);
  }
  xTaskCreate(uart_tx_task, "uart_tx", kUartTxTaskStackBytes, nullptr, 2, nullptr);
  xTaskCreate(inference_task, "tflm", kInferenceTaskStackBytes, nullptr, 3, nullptr);
#endif
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
