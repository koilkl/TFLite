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


// Globals, used for compatibility with Arduino-style sketches.
namespace {
tflite::ErrorReporter* error_reporter = nullptr;
const tflite::Model* model = nullptr;
tflite::MicroInterpreter* interpreter = nullptr;
TfLiteTensor* input = nullptr;

static HardwareSerial UartToS3(1);

static constexpr int kDebugBaud = 921600;
static constexpr int kUartBaud = 921600;
static constexpr int kUartRxPin = 11;
static constexpr int kUartTxPin = 10;

static constexpr uint8_t kSync0 = 0xAA;
static constexpr uint8_t kSync1 = 0x55;
static constexpr uint8_t kMsgTypeInference = 0x01;

struct UartPacket {
  uint16_t frame_id;
  uint8_t label_id;
  uint8_t confidence;
  uint8_t flags;
};

static QueueHandle_t s_uart_queue = nullptr;

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
      Serial.println(confidence);
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
  xTaskCreatePinnedToCore(uart_tx_task, "uart_tx", 2048, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(inference_task, "tflm", 8192, nullptr, 3, nullptr, 1);
#else
  xTaskCreate(uart_tx_task, "uart_tx", 2048, nullptr, 2, nullptr);
  xTaskCreate(inference_task, "tflm", 8192, nullptr, 3, nullptr);
#endif
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
