#ifndef TFLITE_IMAGE_PROVIDER_H_
#define TFLITE_IMAGE_PROVIDER_H_

#include <stdint.h>

#include <tensorflow/lite/c/common.h>
#if __has_include("tensorflow/lite/micro/micro_error_reporter.h")
#include <tensorflow/lite/micro/micro_error_reporter.h>
#elif __has_include("tensorflow/lite/micro/tflite_bridge/micro_error_reporter.h")
#include <tensorflow/lite/micro/tflite_bridge/micro_error_reporter.h>
#endif

// This is the public interface to get an image. It will return a 96x96 grayscale image.
// The image_data buffer should be pre-allocated by the caller to be of size image_width * image_height * channels.
TfLiteStatus GetImage(tflite::ErrorReporter* error_reporter, int image_width, int image_height, int channels, int8_t* image_data);

// Function to clean up resources allocated by the image provider
void ImageProviderDeinit();

#endif  // TFLITE_IMAGE_PROVIDER_H_
