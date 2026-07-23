#ifndef TFLITE_IMAGE_PROVIDER_H_
#define TFLITE_IMAGE_PROVIDER_H_

#include <stdint.h>

#include <tensorflow/lite/c/common.h>
#if __has_include("tensorflow/lite/micro/micro_error_reporter.h")
#include <tensorflow/lite/micro/micro_error_reporter.h>
#elif __has_include("tensorflow/lite/micro/tflite_bridge/micro_error_reporter.h")
#include <tensorflow/lite/micro/tflite_bridge/micro_error_reporter.h>
#endif

// Crop modes for the image capture pipeline.
// CROP_MODE_SIGN: ROI-based crop of the upper-left portion (sign search window, default).
// CROP_MODE_JUNCTION: lower 40%-75% portion of the image (road/junction).
#define CROP_MODE_SIGN     0
#define CROP_MODE_JUNCTION 1
// Constants from main.ino
#define IMG_WIDTH  1536
#define IMG_HEIGHT 1232
#define OUT_WIDTH  96
#define OUT_HEIGHT 96

// Override IMG_SIZE to match the TFLite model input (96×96).
// Must be defined BEFORE ESP32_P4_IMX219.h is included so the library's
// #ifndef guard picks it up.  ESP32_P4_IMX219.cpp also includes us via
// __has_include, so the library's grayscale buffer is built at 96×96 too.
#ifdef IMG_SIZE
#undef IMG_SIZE
#endif
#define IMG_SIZE 96

// This is the public interface to get an image. It will return a 96x96 grayscale image.
// The image_data buffer should be pre-allocated by the caller to be of size image_width * image_height * channels.
TfLiteStatus GetImage(tflite::ErrorReporter* error_reporter, int image_width, int image_height, int channels, int8_t* image_data);
// Crop mode globals
extern volatile int s_crop_mode;
// Set the crop mode used by GetImage.
// Must be called before GetImage() to select the desired region.
void SetCropMode(int mode);

// Function to clean up resources allocated by the image provider
void ImageProviderDeinit();

#endif  // TFLITE_IMAGE_PROVIDER_H_
