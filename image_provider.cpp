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

#if __has_include("esp_err.h")
#include "esp_err.h"
#endif

#if __has_include("nvs_flash.h")
#include "nvs_flash.h"
#endif

#if __has_include("esp_timer.h")
#include "esp_timer.h"
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

#if __has_include("esp_cam_sensor_xclk.h")
#include "esp_cam_sensor_xclk.h"
#define TFLITE_P4_IMX219_HAS_XCLK_ROUTER 1
#else
#define TFLITE_P4_IMX219_HAS_XCLK_ROUTER 0
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
static const char *kTag = "tflite_cam";

#if __has_include("esp_log.h")
#define TFLITE_CAM_LOGI(...) ESP_LOGI(kTag, __VA_ARGS__)
#define TFLITE_CAM_LOGW(...) ESP_LOGW(kTag, __VA_ARGS__)
#else
#define TFLITE_CAM_LOGI(...) do { printf(__VA_ARGS__); printf("\n"); } while (0)
#define TFLITE_CAM_LOGW(...) do { printf(__VA_ARGS__); printf("\n"); } while (0)
#endif

#if TFLITE_P4_IMX219_HAS_ESP_VIDEO
static int s_fd = -1;
static void *s_mapped_bufs[2] = {0};
static size_t s_mapped_lens[2] = {0};
static uint8_t *s_rgb_buf = NULL;
static bool s_camera_inited = false;
static volatile bool s_in_dqbuf = false;
static volatile uint64_t s_dqbuf_enter_us = 0;
static uint64_t s_capture_start_us = 0;
static uint64_t s_last_stats_us = 0;
static uint32_t s_total_frames = 0;
static uint32_t s_last_stat_frames = 0;
static bool s_summary_logged = false;
static esp_cam_sensor_format_t s_sensor_fmt_patch = {};
#if TFLITE_P4_IMX219_HAS_XCLK_ROUTER
static esp_cam_sensor_xclk_handle_t s_xclk_handle = NULL;
#endif

static void camera_deinit(void);

static const gpio_num_t kI2cMasterScl = GPIO_NUM_26;
static const gpio_num_t kI2cMasterSda = GPIO_NUM_27;
static const int kI2cMasterNum = 0;
static const int kI2cMasterFreqHz = 100000;
static const gpio_num_t kXclkPin = GPIO_NUM_20;
static constexpr uint32_t kXclkScanHz[] = {24000000, 19200000, 12000000, 6000000};
static constexpr size_t kXclkScanIndex = 0;
static_assert(kXclkScanIndex < (sizeof(kXclkScanHz) / sizeof(kXclkScanHz[0])), "kXclkScanIndex out of range");
static constexpr bool kForceSensorFmt = false;
static constexpr int32_t kHsSettleOverride = -1;
static constexpr int32_t kLineSyncOverride = -1;
static constexpr uint32_t kExperimentObserveSeconds = 10;

static uint32_t active_xclk_hz() {
  return kXclkScanHz[kXclkScanIndex];
}

static bool sensor_patch_requested() {
  return kHsSettleOverride >= 0 || kLineSyncOverride >= 0;
}

static void log_scan_configuration() {
  TFLITE_CAM_LOGI("scan config: xclk_idx=%u xclk_hz=%lu hs_settle_override=%ld line_sync_override=%ld force_sensor_fmt=%d observe_s=%lu",
                  (unsigned)kXclkScanIndex, (unsigned long)active_xclk_hz(), (long)kHsSettleOverride,
                  (long)kLineSyncOverride, (int)kForceSensorFmt, (unsigned long)kExperimentObserveSeconds);
  TFLITE_CAM_LOGI("xclk candidates: [0]=24000000 [1]=19200000 [2]=12000000 [3]=6000000");
}

static void log_sensor_fmt(const char *stage, const esp_cam_sensor_format_t *fmt) {
  TFLITE_CAM_LOGI("sensor fmt(%s): w=%u h=%u out_fmt=%u port=%u xclk=%d mipi_clk=%lu lanes=%lu hs_settle=%lu line_sync=%d",
                  stage, (unsigned)fmt->width, (unsigned)fmt->height, (unsigned)fmt->format, (unsigned)fmt->port,
                  (int)fmt->xclk, (unsigned long)fmt->mipi_info.mipi_clk, (unsigned long)fmt->mipi_info.lane_num,
                  (unsigned long)fmt->mipi_info.hs_settle, (int)fmt->mipi_info.line_sync_en);
}

static void warn_if_sensor_mode_changed(const esp_cam_sensor_format_t *before, const esp_cam_sensor_format_t *after) {
  if (after->width == 1920 && after->height == 1080) {
    TFLITE_CAM_LOGW("sensor fmt jumped to 1080p; stop this scan point and avoid mixing it with 1536x1232 RAW10 capture");
  }
  if (before->width != after->width || before->height != after->height || before->format != after->format) {
    TFLITE_CAM_LOGW("sensor mode changed: before=%ux%u fmt=%u after=%ux%u fmt=%u",
                    (unsigned)before->width, (unsigned)before->height, (unsigned)before->format,
                    (unsigned)after->width, (unsigned)after->height, (unsigned)after->format);
  }
}

static void log_v4l2_capture_fmt(const char *stage, const struct v4l2_format *fmt) {
  TFLITE_CAM_LOGI("capture fmt(%s): w=%u h=%u fourcc=%c%c%c%c field=%u bytesperline=%u sizeimage=%u",
                  stage, (unsigned)fmt->fmt.pix.width, (unsigned)fmt->fmt.pix.height,
                  fmt->fmt.pix.pixelformat & 0xff, (fmt->fmt.pix.pixelformat >> 8) & 0xff,
                  (fmt->fmt.pix.pixelformat >> 16) & 0xff, (fmt->fmt.pix.pixelformat >> 24) & 0xff,
                  (unsigned)fmt->fmt.pix.field, (unsigned)fmt->fmt.pix.bytesperline, (unsigned)fmt->fmt.pix.sizeimage);
}

static void maybe_log_capture_stats(void) {
#if __has_include("esp_timer.h")
  uint64_t now = esp_timer_get_time();
  if (s_capture_start_us == 0) {
    s_capture_start_us = now;
    s_last_stats_us = now;
    return;
  }

  if (now - s_last_stats_us >= 1000000ULL) {
    uint64_t dq_ms = 0;
    if (s_in_dqbuf && s_dqbuf_enter_us != 0) {
      dq_ms = (now - s_dqbuf_enter_us) / 1000ULL;
    }
    uint32_t delta_frames = s_total_frames - s_last_stat_frames;
    s_last_stat_frames = s_total_frames;
    s_last_stats_us = now;
    TFLITE_CAM_LOGI("Capture FPS: %lu (in_dqbuf=%d dq_ms=%llu total_frame=%lu)",
                    (unsigned long)delta_frames, (int)s_in_dqbuf,
                    (unsigned long long)dq_ms, (unsigned long)s_total_frames);
  }

  if (!s_summary_logged &&
      (now - s_capture_start_us) >= (uint64_t)kExperimentObserveSeconds * 1000000ULL) {
    uint64_t dq_ms = 0;
    if (s_in_dqbuf && s_dqbuf_enter_us != 0) {
      dq_ms = (now - s_dqbuf_enter_us) / 1000ULL;
    }
    s_summary_logged = true;
    TFLITE_CAM_LOGI("SCAN SUMMARY: xclk_idx=%u xclk_hz=%lu hs_settle_override=%ld line_sync_override=%ld observe_s=%lu total_frame=%lu dq_ms=%llu result=%s",
                    (unsigned)kXclkScanIndex, (unsigned long)active_xclk_hz(), (long)kHsSettleOverride,
                    (long)kLineSyncOverride, (unsigned long)kExperimentObserveSeconds,
                    (unsigned long)s_total_frames, (unsigned long long)dq_ms,
                    s_total_frames > 0 ? "FRAME_OK" : "NO_FRAME");
  }
#endif
}

static void inspect_and_patch_sensor_fmt_if_needed(void) {
  if (s_fd < 0) {
    return;
  }

  esp_cam_sensor_format_t before;
  memset(&before, 0, sizeof(before));
  if (ioctl(s_fd, VIDIOC_G_SENSOR_FMT, &before) != 0) {
    TFLITE_CAM_LOGW("VIDIOC_G_SENSOR_FMT failed: errno=%d (%s)", errno, strerror(errno));
    return;
  }
  log_sensor_fmt("before", &before);

  if (!kForceSensorFmt && !sensor_patch_requested()) {
    return;
  }

  memcpy(&s_sensor_fmt_patch, &before, sizeof(s_sensor_fmt_patch));
  if (kForceSensorFmt) {
    s_sensor_fmt_patch.name = "forced";
    s_sensor_fmt_patch.width = IMG_WIDTH;
    s_sensor_fmt_patch.height = IMG_HEIGHT;
    s_sensor_fmt_patch.format = ESP_CAM_SENSOR_PIXFORMAT_RAW10;
    s_sensor_fmt_patch.port = ESP_CAM_SENSOR_MIPI_CSI;
    s_sensor_fmt_patch.xclk = active_xclk_hz();
    if (s_sensor_fmt_patch.mipi_info.mipi_clk == 0) {
      s_sensor_fmt_patch.mipi_info.mipi_clk = 456000000;
    }
    if (s_sensor_fmt_patch.mipi_info.lane_num == 0) {
      s_sensor_fmt_patch.mipi_info.lane_num = 2;
    }
  } else {
    s_sensor_fmt_patch.name = "patched";
  }

  if (kHsSettleOverride >= 0) {
    s_sensor_fmt_patch.mipi_info.hs_settle = (uint32_t)kHsSettleOverride;
  }
  if (kLineSyncOverride >= 0) {
    s_sensor_fmt_patch.mipi_info.line_sync_en = kLineSyncOverride ? 1 : 0;
  }

  TFLITE_CAM_LOGI("request sensor fmt(%s): xclk=%d mipi_clk=%lu lanes=%lu hs_settle=%lu line_sync=%d",
                  kForceSensorFmt ? "force" : "patch",
                  (int)s_sensor_fmt_patch.xclk, (unsigned long)s_sensor_fmt_patch.mipi_info.mipi_clk,
                  (unsigned long)s_sensor_fmt_patch.mipi_info.lane_num, (unsigned long)s_sensor_fmt_patch.mipi_info.hs_settle,
                  (int)s_sensor_fmt_patch.mipi_info.line_sync_en);
  if (ioctl(s_fd, VIDIOC_S_SENSOR_FMT, &s_sensor_fmt_patch) != 0) {
    TFLITE_CAM_LOGW("VIDIOC_S_SENSOR_FMT failed: errno=%d (%s)", errno, strerror(errno));
  }

  esp_cam_sensor_format_t after;
  memset(&after, 0, sizeof(after));
  if (ioctl(s_fd, VIDIOC_G_SENSOR_FMT, &after) == 0) {
    log_sensor_fmt("after", &after);
    warn_if_sensor_mode_changed(&before, &after);
  }
}

static void enable_xclk(void) {
#if TFLITE_P4_IMX219_HAS_XCLK_ROUTER
  if (s_xclk_handle != NULL) {
    esp_cam_sensor_xclk_stop(s_xclk_handle);
    esp_cam_sensor_xclk_free(s_xclk_handle);
    s_xclk_handle = NULL;
  }

  esp_err_t err = esp_cam_sensor_xclk_allocate(ESP_CAM_SENSOR_XCLK_ESP_CLOCK_ROUTER, &s_xclk_handle);
#if __has_include("esp_err.h")
  TFLITE_CAM_LOGI("xclk_allocate: %s", esp_err_to_name(err));
#else
  TFLITE_CAM_LOGI("xclk_allocate: %d", (int)err);
#endif
  if (err != ESP_OK) {
    return;
  }

  esp_cam_sensor_xclk_config_t cfg;
  memset(&cfg, 0, sizeof(cfg));
  cfg.esp_clock_router_cfg.xclk_pin = kXclkPin;
  cfg.esp_clock_router_cfg.xclk_freq_hz = active_xclk_hz();
  err = esp_cam_sensor_xclk_start(s_xclk_handle, &cfg);
#if __has_include("esp_err.h")
  TFLITE_CAM_LOGI("xclk_start: %s gpio=%d freq=%lu", esp_err_to_name(err), (int)kXclkPin, (unsigned long)active_xclk_hz());
#else
  TFLITE_CAM_LOGI("xclk_start: %d gpio=%d freq=%lu", (int)err, (int)kXclkPin, (unsigned long)active_xclk_hz());
#endif
#else
  ledc_timer_config_t ledc_timer;
  memset(&ledc_timer, 0, sizeof(ledc_timer));
  ledc_timer.timer_num = LEDC_TIMER_0;
  ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
  ledc_timer.duty_resolution = LEDC_TIMER_1_BIT;
  ledc_timer.freq_hz = active_xclk_hz();
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
  TFLITE_CAM_LOGI("xclk_start: ledc gpio=%d freq=%lu", (int)kXclkPin, (unsigned long)active_xclk_hz());
#endif
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

  log_scan_configuration();
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

  inspect_and_patch_sensor_fmt_if_needed();

  struct v4l2_format fmt;
  memset(&fmt, 0, sizeof(fmt));
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = IMG_WIDTH;
  fmt.fmt.pix.height = IMG_HEIGHT;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_SBGGR10;
  log_v4l2_capture_fmt("request", &fmt);
  if (ioctl(s_fd, VIDIOC_S_FMT, &fmt) != 0) {
    TF_LITE_REPORT_ERROR(error_reporter, "VIDIOC_S_FMT failed: %d (%s)", errno, strerror(errno));
    camera_deinit();
    return kTfLiteError;
  }
  log_v4l2_capture_fmt("applied", &fmt);

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
  s_total_frames = 0;
  s_last_stat_frames = 0;
  s_in_dqbuf = false;
  s_dqbuf_enter_us = 0;
  s_capture_start_us = 0;
  s_last_stats_us = 0;
  s_summary_logged = false;
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

#if TFLITE_P4_IMX219_HAS_XCLK_ROUTER
  if (s_xclk_handle != NULL) {
    esp_cam_sensor_xclk_stop(s_xclk_handle);
    esp_cam_sensor_xclk_free(s_xclk_handle);
    s_xclk_handle = NULL;
  }
#endif

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
  static bool s_backend_logged = false;
  if (!s_backend_logged) {
    s_backend_logged = true;
    TFLITE_CAM_LOGI("GetImage backend: arduino_imx219_lib");
  }
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
  static bool s_backend_logged = false;
  if (!s_backend_logged) {
    s_backend_logged = true;
    TFLITE_CAM_LOGI("GetImage backend: esp_video");
  }
  if (camera_init_if_needed(error_reporter) != kTfLiteOk) {
    return kTfLiteError;
  }

  struct v4l2_buffer buf_dq;
  memset(&buf_dq, 0, sizeof(buf_dq));
  buf_dq.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf_dq.memory = V4L2_MEMORY_MMAP;
  s_in_dqbuf = true;
  s_dqbuf_enter_us = 0;
#if __has_include("esp_timer.h")
  s_dqbuf_enter_us = esp_timer_get_time();
#endif
  if (ioctl(s_fd, VIDIOC_DQBUF, &buf_dq) != 0) {
    s_in_dqbuf = false;
    s_dqbuf_enter_us = 0;
    maybe_log_capture_stats();
    TF_LITE_REPORT_ERROR(error_reporter, "VIDIOC_DQBUF failed: %d (%s)", errno, strerror(errno));
    return kTfLiteError;
  }
  s_in_dqbuf = false;
  s_dqbuf_enter_us = 0;
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

  s_total_frames++;
  maybe_log_capture_stats();

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
