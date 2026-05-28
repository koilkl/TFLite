#include "image_provider.h"

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#if __has_include(<sys/ioctl.h>)
#include <sys/ioctl.h>
#endif

#if __has_include(<sys/mman.h>)
#include <sys/mman.h>
#endif

#if __has_include("esp_log.h")
#include "esp_log.h"
#endif

#if __has_include("nvs_flash.h")
#include "nvs_flash.h"
#endif

#if __has_include("driver/ledc.h")
#include "driver/ledc.h"
#endif

#if __has_include("driver/gpio.h")
#include "driver/gpio.h"
#endif

#if __has_include("imx219.h")
#include "imx219.h"
#endif

#if __has_include("esp_heap_caps.h")
#include "esp_heap_caps.h"
#endif

#if defined(ARDUINO_ARCH_ESP32P4)
#include <ESP32_P4_IMX219.h>
#define TFLITE_P4_IMX219_HAS_ARDUINO_IMX219_LIB 1
#elif __has_include("ESP32_P4_IMX219.h")
#include "ESP32_P4_IMX219.h"
#define TFLITE_P4_IMX219_HAS_ARDUINO_IMX219_LIB 1
#elif __has_include(<ESP32_P4_IMX219.h>)
#include <ESP32_P4_IMX219.h>
#define TFLITE_P4_IMX219_HAS_ARDUINO_IMX219_LIB 1
#else
#define TFLITE_P4_IMX219_HAS_ARDUINO_IMX219_LIB 0
#endif

#if !TFLITE_P4_IMX219_HAS_ARDUINO_IMX219_LIB && __has_include("esp_video_init.h")
#ifdef CONFIG_ESP_VIDEO_ENABLE_MIPI_CSI_VIDEO_DEVICE
#undef CONFIG_ESP_VIDEO_ENABLE_MIPI_CSI_VIDEO_DEVICE
#endif
#define CONFIG_ESP_VIDEO_ENABLE_MIPI_CSI_VIDEO_DEVICE 1
#include "esp_video_init.h"
#include "esp_video_ioctl.h"
#include "esp_video_device.h"
#endif

// Constants from main.ino
#define IMG_WIDTH  1536
#define IMG_HEIGHT 1232
#define OUT_WIDTH  96
#define OUT_HEIGHT 96

#if !TFLITE_P4_IMX219_HAS_ARDUINO_IMX219_LIB && __has_include("esp_video_init.h")
#define TFLITE_P4_IMX219_HAS_ESP_VIDEO 1
#else
#define TFLITE_P4_IMX219_HAS_ESP_VIDEO 0
#endif

static int *s_x_lut = NULL;
static int *s_y_lut = NULL;

#if TFLITE_P4_IMX219_HAS_ESP_VIDEO
static int s_fd = -1;
static void *s_mapped_bufs[2] = {0};
static size_t s_mapped_lens[2] = {0};
static uint8_t *s_rgb_buf = NULL;
static bool s_camera_inited = false;

static void camera_deinit(void);

static const gpio_num_t kI2cMasterScl = GPIO_NUM_8;
static const gpio_num_t kI2cMasterSda = GPIO_NUM_7;
static const int kI2cMasterNum = 0;
static const int kI2cMasterFreqHz = 100000;
static const gpio_num_t kXclkPin = GPIO_NUM_20;
static const int kXclkFreqHz = 24000000;

static void enable_xclk(void) {
  ledc_timer_config_t ledc_timer;
  memset(&ledc_timer, 0, sizeof(ledc_timer));
  ledc_timer.timer_num = LEDC_TIMER_0;
  ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
  ledc_timer.duty_resolution = LEDC_TIMER_1_BIT;
  ledc_timer.freq_hz = kXclkFreqHz;
  ledc_timer.clk_cfg = LEDC_AUTO_CLK;
  ledc_timer_config(&ledc_timer);

  ledc_channel_config_t ledc_channel;
  memset(&ledc_channel, 0, sizeof(ledc_channel));
  ledc_channel.channel = LEDC_CHANNEL_0;
  ledc_channel.duty = 1;
  ledc_channel.gpio_num = kXclkPin;
  ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
  ledc_channel.hpoint = 0;
  ledc_channel.timer_sel = LEDC_TIMER_0;
  ledc_channel_config(&ledc_channel);
}

static void *tflite_cam_malloc(size_t size) {
#if __has_include("esp_heap_caps.h")
  void *p = heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
  if (p) {
    return p;
  }
#endif
  return malloc(size);
}

static void tflite_cam_free(void *p) {
#if __has_include("esp_heap_caps.h")
  heap_caps_free(p);
#else
  free(p);
#endif
}
#endif

// Initialize lookup tables for demosaicing and cropping
static void init_demosaic_luts(int width, int height) {
    if (s_x_lut == NULL) {
        s_x_lut = (int *)malloc(OUT_WIDTH * sizeof(int));
    }
    if (s_y_lut == NULL) {
        s_y_lut = (int *)malloc(OUT_HEIGHT * sizeof(int));
    }
    
    int crop_w = 1232;
    int crop_h = 1232;
    int x_offset = (width - crop_w) / 2;
    int y_offset = (height - crop_h) / 2;
    
    float x_step = (float)crop_w / OUT_WIDTH;
    float y_step = (float)crop_h / OUT_HEIGHT;
    
    for (int y = 0; y < OUT_HEIGHT; y++) {
        s_y_lut[y] = (y_offset + (int)(y * y_step + 0.5f)) & ~1;
    }
    for (int x = 0; x < OUT_WIDTH; x++) {
        s_x_lut[x] = (x_offset + (int)(x * x_step + 0.5f)) & ~1;
    }
    
    // In a real application, you might want to log this or return status
    // printf("Sensor: %dx%d, Center crop: %dx%d at offset (%d,%d), Output: %dx%d\n",
    //          width, height, crop_w, crop_h, x_offset, y_offset, OUT_WIDTH, OUT_HEIGHT);
}

// Demosaic BGGR raw 10-bit data to RGB
static void demosaic_bggr_to_rgb(const uint8_t *raw10, uint8_t *rgb, int width, int height) {
    if (s_x_lut == NULL || s_y_lut == NULL) {
        init_demosaic_luts(width, height);
    }

    for (int y = 0; y < OUT_HEIGHT; y++) {
        int src_y = s_y_lut[y];
        int row0 = src_y * (width * 5 / 4); // 10-bit raw data, 4 pixels take 5 bytes
        int row1 = (src_y + 1) * (width * 5 / 4);
        int out_row = y * OUT_WIDTH;
        for (int x = 0; x < OUT_WIDTH; x++) {
            int src_x = s_x_lut[x];
            // Calculate column index for 10-bit packed data
            // Each 4 pixels (BGGR) are packed into 5 bytes
            // Byte 0: B7-0
            // Byte 1: G7-0
            // Byte 2: G7-0
            // Byte 3: R7-0
            // Byte 4: B9-8, G9-8, G9-8, R9-8 (2 bits each)
            // This logic needs to be carefully adapted for 10-bit packed data.
            // The original main.ino code assumes 8-bit access for B, G, R, which might be simplified.
            // Let's assume for now that raw10 is already unpacked or the access pattern is simplified.
            // Re-evaluating main.ino:
            // uint8_t b = raw10[row0 + col];
            // uint8_t g = (raw10[row0 + col + 1] + raw10[row1 + col]) >> 1;
            // uint8_t r = raw10[row1 + col + 1];
            // This looks like it's treating the 10-bit data as if it were 8-bit for simplicity or
            // assuming a specific packing where the lower 8 bits are directly accessible.
            // For a true 10-bit to 8-bit conversion, it would involve bit shifting.
            // Given the original code, I'll replicate it directly.

            int col_byte_idx = (src_x / 4) * 5; // Start byte for a group of 4 pixels
            int pixel_in_group_idx = src_x % 4; // Index within the 4-pixel group

            uint8_t b, g, r;

            // This is a simplified interpretation of 10-bit BGGR.
            // The original code in main.ino directly accesses bytes.
            // For a more accurate 10-bit unpacking, it would be more complex.
            // Assuming the main.ino's simplified access is sufficient for the model.
            // The main.ino code is likely reading the 8 most significant bits or
            // a specific packing format.
            // Let's replicate the main.ino logic directly.
            
            // The original code's `col` calculation:
            // int col = (src_x >> 2) * 5 + (src_x % 4);
            // This `col` is an index into the `raw10` buffer.
            // It seems to be directly indexing into the packed 10-bit data as if it were 8-bit.
            // This is a common simplification for certain hardware or if only 8-bit precision is needed.

            int col = (src_x >> 2) * 5 + (src_x % 4); // This is the byte index in the raw10 buffer

            // The original code's pixel extraction:
            b = raw10[row0 + col];
            g = (raw10[row0 + col + 1] + raw10[row1 + col]) >> 1;
            r = raw10[row1 + col + 1];

            int out_idx = (out_row + x) * 3;
            rgb[out_idx + 0] = r;
            rgb[out_idx + 1] = g;
            rgb[out_idx + 2] = b;
        }
    }
}

// Convert RGB to Grayscale
static void rgb_to_gray(const uint8_t *rgb, uint8_t *gray, int pixel_count) {
    for (int i = 0; i < pixel_count; i++) {
        int idx = i * 3;
        uint8_t r = rgb[idx + 0];
        uint8_t g = rgb[idx + 1];
        uint8_t b = rgb[idx + 2];
        // Standard BT.601 luminance calculation
        gray[i] = (uint8_t)(((uint16_t)r * 30 + (uint16_t)g * 59 + (uint16_t)b * 11) / 100);
    }
}

#if TFLITE_P4_IMX219_HAS_ESP_VIDEO
static TfLiteStatus camera_init_if_needed(tflite::ErrorReporter* error_reporter) {
  if (s_camera_inited) {
    return kTfLiteOk;
  }

  if (s_x_lut == NULL || s_y_lut == NULL) {
    init_demosaic_luts(IMG_WIDTH, IMG_HEIGHT);
  }

#if __has_include("nvs_flash.h")
  nvs_flash_init();
#endif

  enable_xclk();
  usleep(100 * 1000);

#if __has_include("imx219.h")
  imx219_force_link();
#endif

  esp_video_init_csi_config_t csi_config;
  memset(&csi_config, 0, sizeof(csi_config));
  csi_config.sccb_config.init_sccb = true;
  csi_config.sccb_config.i2c_config.port = kI2cMasterNum;
  csi_config.sccb_config.i2c_config.scl_pin = kI2cMasterScl;
  csi_config.sccb_config.i2c_config.sda_pin = kI2cMasterSda;
  csi_config.sccb_config.freq = kI2cMasterFreqHz;
  csi_config.reset_pin = GPIO_NUM_NC;
  csi_config.pwdn_pin = GPIO_NUM_NC;

  esp_video_init_config_t cam_config;
  memset(&cam_config, 0, sizeof(cam_config));
  cam_config.csi = &csi_config;
  esp_err_t init_err = esp_video_init(&cam_config);
  if (init_err != ESP_OK) {
    TF_LITE_REPORT_ERROR(error_reporter, "esp_video_init failed: %d", (int)init_err);
    return kTfLiteError;
  }

  s_fd = open(ESP_VIDEO_MIPI_CSI_DEVICE_NAME, O_RDWR);
  if (s_fd < 0) {
    TF_LITE_REPORT_ERROR(error_reporter, "open CSI device failed: %d (%s)", errno, strerror(errno));
    return kTfLiteError;
  }

  struct v4l2_format fmt;
  memset(&fmt, 0, sizeof(fmt));
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = IMG_WIDTH;
  fmt.fmt.pix.height = IMG_HEIGHT;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_SBGGR10;
  if (ioctl(s_fd, VIDIOC_S_FMT, &fmt) != 0) {
    TF_LITE_REPORT_ERROR(error_reporter, "VIDIOC_S_FMT failed: %d (%s)", errno, strerror(errno));
    camera_deinit();
    return kTfLiteError;
  }

  struct v4l2_requestbuffers req;
  memset(&req, 0, sizeof(req));
  req.count = 2;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  if (ioctl(s_fd, VIDIOC_REQBUFS, &req) != 0) {
    TF_LITE_REPORT_ERROR(error_reporter, "VIDIOC_REQBUFS failed: %d (%s)", errno, strerror(errno));
    camera_deinit();
    return kTfLiteError;
  }
  if (req.count < 2) {
    TF_LITE_REPORT_ERROR(error_reporter, "VIDIOC_REQBUFS returned %d buffers", (int)req.count);
    camera_deinit();
    return kTfLiteError;
  }

  for (int i = 0; i < 2; i++) {
    struct v4l2_buffer b;
    memset(&b, 0, sizeof(b));
    b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    b.memory = V4L2_MEMORY_MMAP;
    b.index = i;
    if (ioctl(s_fd, VIDIOC_QUERYBUF, &b) != 0) {
      TF_LITE_REPORT_ERROR(error_reporter, "VIDIOC_QUERYBUF(%d) failed: %d (%s)", i, errno, strerror(errno));
      camera_deinit();
      return kTfLiteError;
    }

    void *buf = mmap(NULL, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, s_fd, b.m.offset);
    if (buf == MAP_FAILED) {
      TF_LITE_REPORT_ERROR(error_reporter, "mmap(%d) failed: %d (%s)", i, errno, strerror(errno));
      camera_deinit();
      return kTfLiteError;
    }

    s_mapped_bufs[i] = buf;
    s_mapped_lens[i] = b.length;

    if (ioctl(s_fd, VIDIOC_QBUF, &b) != 0) {
      TF_LITE_REPORT_ERROR(error_reporter, "VIDIOC_QBUF(%d) failed: %d (%s)", i, errno, strerror(errno));
      camera_deinit();
      return kTfLiteError;
    }
  }

  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(s_fd, VIDIOC_STREAMON, &type) != 0) {
    TF_LITE_REPORT_ERROR(error_reporter, "VIDIOC_STREAMON failed: %d (%s)", errno, strerror(errno));
    camera_deinit();
    return kTfLiteError;
  }

  s_rgb_buf = (uint8_t *)tflite_cam_malloc(OUT_WIDTH * OUT_HEIGHT * 3);
  if (!s_rgb_buf) {
    TF_LITE_REPORT_ERROR(error_reporter, "alloc rgb buffer failed");
    camera_deinit();
    return kTfLiteError;
  }

  s_camera_inited = true;
  return kTfLiteOk;
}

static void camera_deinit(void) {
  if (s_fd >= 0) {
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(s_fd, VIDIOC_STREAMOFF, &type);
  }

  for (int i = 0; i < 2; i++) {
    if (s_mapped_bufs[i] && s_mapped_lens[i]) {
      munmap(s_mapped_bufs[i], s_mapped_lens[i]);
      s_mapped_bufs[i] = NULL;
      s_mapped_lens[i] = 0;
    }
  }

  if (s_fd >= 0) {
    close(s_fd);
    s_fd = -1;
  }

  if (s_rgb_buf) {
    tflite_cam_free(s_rgb_buf);
    s_rgb_buf = NULL;
  }

  s_camera_inited = false;
}

#endif

// Public function to get the image
TfLiteStatus GetImage(tflite::ErrorReporter* error_reporter, int image_width, int image_height, int channels, int8_t* image_data) {
  if (image_width != OUT_WIDTH || image_height != OUT_HEIGHT || channels != 1) {
    TF_LITE_REPORT_ERROR(error_reporter, "GetImage expects %dx%dx1, got %dx%dx%d", OUT_WIDTH, OUT_HEIGHT, image_width, image_height, channels);
    return kTfLiteError;
  }

#if TFLITE_P4_IMX219_HAS_ARDUINO_IMX219_LIB
  static bool s_ok = false;
  if (!s_ok) {
    s_ok = esp32_p4_imx219_begin();
    if (!s_ok) {
      TF_LITE_REPORT_ERROR(error_reporter, "esp32_p4_imx219_begin failed");
      return kTfLiteError;
    }
  }

  bool updated = false;
  for (int tries = 0; tries < 100; tries++) {
    if (esp32_p4_imx219_update()) {
      updated = true;
      break;
    }
    usleep(2000);
  }
  if (!updated) {
    TF_LITE_REPORT_ERROR(error_reporter, "esp32_p4_imx219_update timeout");
    return kTfLiteError;
  }

  const uint8_t *gray_u8 = esp32_p4_imx219_gray();
  const size_t gray_size = esp32_p4_imx219_gray_size();
  if (!gray_u8 || gray_size < (size_t)(OUT_WIDTH * OUT_HEIGHT)) {
    TF_LITE_REPORT_ERROR(error_reporter, "invalid gray buffer");
    return kTfLiteError;
  }

  for (int i = 0; i < OUT_WIDTH * OUT_HEIGHT; i++) {
    image_data[i] = (int8_t)((int)gray_u8[i] - 128);
  }

  return kTfLiteOk;
#elif TFLITE_P4_IMX219_HAS_ESP_VIDEO
  if (camera_init_if_needed(error_reporter) != kTfLiteOk) {
    return kTfLiteError;
  }

  struct v4l2_buffer buf_dq;
  memset(&buf_dq, 0, sizeof(buf_dq));
  buf_dq.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf_dq.memory = V4L2_MEMORY_MMAP;
  if (ioctl(s_fd, VIDIOC_DQBUF, &buf_dq) != 0) {
    TF_LITE_REPORT_ERROR(error_reporter, "VIDIOC_DQBUF failed: %d (%s)", errno, strerror(errno));
    return kTfLiteError;
  }
  if (buf_dq.index >= 2 || s_mapped_bufs[buf_dq.index] == NULL) {
    TF_LITE_REPORT_ERROR(error_reporter, "invalid dqbuf index: %d", (int)buf_dq.index);
    ioctl(s_fd, VIDIOC_QBUF, &buf_dq);
    return kTfLiteError;
  }

  const uint8_t *raw_data = (const uint8_t *)s_mapped_bufs[buf_dq.index];
  demosaic_bggr_to_rgb(raw_data, s_rgb_buf, IMG_WIDTH, IMG_HEIGHT);

  uint8_t gray_u8[OUT_WIDTH * OUT_HEIGHT];
  rgb_to_gray(s_rgb_buf, gray_u8, OUT_WIDTH * OUT_HEIGHT);
  for (int i = 0; i < OUT_WIDTH * OUT_HEIGHT; i++) {
    image_data[i] = (int8_t)((int)gray_u8[i] - 128);
  }

  if (ioctl(s_fd, VIDIOC_QBUF, &buf_dq) != 0) {
    TF_LITE_REPORT_ERROR(error_reporter, "VIDIOC_QBUF failed: %d (%s)", errno, strerror(errno));
    return kTfLiteError;
  }

  return kTfLiteOk;
#else
  for (int i = 0; i < OUT_WIDTH * OUT_HEIGHT; i++) {
    image_data[i] = 0;
  }
  TF_LITE_REPORT_ERROR(error_reporter, "esp_video headers not found; GetImage returns zeros");
  return kTfLiteOk;
#endif
}

// Function to clean up allocated LUTs
void ImageProviderDeinit() {
#if TFLITE_P4_IMX219_HAS_ESP_VIDEO
    camera_deinit();
#endif
    if (s_x_lut) {
        free(s_x_lut);
        s_x_lut = NULL;
    }
    if (s_y_lut) {
        free(s_y_lut);
        s_y_lut = NULL;
    }
}
