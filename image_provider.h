#ifndef TFLITE_IMAGE_PROVIDER_H_
#define TFLITE_IMAGE_PROVIDER_H_

#include <stdint.h>
#include <stddef.h>

#include <tensorflow/lite/c/common.h>
#if __has_include("tensorflow/lite/micro/micro_error_reporter.h")
#include <tensorflow/lite/micro/micro_error_reporter.h>
#elif __has_include("tensorflow/lite/micro/tflite_bridge/micro_error_reporter.h")
#include <tensorflow/lite/micro/tflite_bridge/micro_error_reporter.h>
#endif

// ── Camera type selection ─────────────────────────────────────────────
//   CAMERA_TYPE_IMX219  →  IMX219 (MIPI CSI, 1536×1232 sensor)
//   CAMERA_TYPE_OV5647  →  OV5647 (MIPI CSI, 1920×1080 sensor)
//   CAMERA_TYPE_AUTO    →  probe the I2C bus at startup (IMX219=0x10,
//                          OV5647=0x36) and pick whichever responds
#define CAMERA_TYPE_IMX219  1
#define CAMERA_TYPE_OV5647  2
#define CAMERA_TYPE_AUTO    3

#ifndef CAMERA_TYPE
#define CAMERA_TYPE CAMERA_TYPE_IMX219
#endif

// ── Preprocessing mode ────────────────────────────────────────────────
// Selects what GetImage produces / how the sketch runs.
//   PREPROCESS_MODE_BG    → B-G colour-difference pipeline (sign detection)
//   PREPROCESS_MODE_GRAY  → plain BT.601 grayscale, no colour difference
//   PREPROCESS_MODE_RGB   → data collection: WB-corrected RGB to Serial
//                           (like the IMX219_RGB_Serial example; no inference)
#define PREPROCESS_MODE_BG    1
#define PREPROCESS_MODE_GRAY  2
#define PREPROCESS_MODE_RGB   3

#ifndef PREPROCESS_MODE
#define PREPROCESS_MODE PREPROCESS_MODE_BG
#endif

// Crop mode: sign ROI.  Junction mode was removed.
#define CROP_MODE_SIGN     0

// ── Working resolution ────────────────────────────────────────────────
// IMG_SIZE controls the pipeline resolution (default 96).  The TFLite
// model must be exported at the same size (AItraining img_size).
#ifndef IMG_SIZE
#define IMG_SIZE 96
#endif

// Constants from main.ino
#define IMG_WIDTH  1536
#define IMG_HEIGHT 1232
#define OUT_WIDTH  IMG_SIZE
#define OUT_HEIGHT IMG_SIZE

// IMG_SIZE must be defined BEFORE ESP32_P4_IMX219.h is included so the
// library's #ifndef guard picks it up.  ESP32_P4_IMX219.cpp also includes
// us via __has_include, so its buffers follow our IMG_SIZE too.
// (The OV5647 library does NOT include us — it stays at its own default
// of 160×160, and GetImage resizes to IMG_SIZE at runtime.)

// ── Camera wrapper API (works for both IMX219 and OV5647) ────────────

// Initialise the selected camera.  Returns true on success.
bool CameraBegin();

// Capture one frame.  Returns true when a new frame is available.
bool CameraUpdate();

// Pointer to the latest RGB buffer (R,G,B interleaved).
const uint8_t* CameraGetRgb();

// Size of the RGB buffer in bytes.
size_t CameraGetRgbSize();

// Width of the RGB image the library produces (96 for IMX219, 160 for OV5647).
int CameraGetRgbWidth();

// Camera name for logging.
const char* CameraGetName();

// Send WB-corrected RGB to Serial with the data-collection sync header
// (0xAA 0x55 0xAA + IMG_SIZE×IMG_SIZE×3).  Call after CameraUpdate().
void CameraSendRgbToSerialWb(uint16_t wb_red, uint16_t wb_blue);

// ── TFLite interface ──────────────────────────────────────────────────
// Returns an IMG_SIZE×IMG_SIZE grayscale image.
// image_data must be pre-allocated: image_width * image_height * channels.
TfLiteStatus GetImage(tflite::ErrorReporter* error_reporter, int image_width,
                      int image_height, int channels, int8_t* image_data);

// Function to clean up resources allocated by the image provider.
void ImageProviderDeinit();

#endif  // TFLITE_IMAGE_PROVIDER_H_
