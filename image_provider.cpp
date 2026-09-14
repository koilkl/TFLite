#include "image_provider.h"

#include <errno.h>
#include <fcntl.h>
#include <math.h>
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

static const char *kTag = "tflite_cam";

#if __has_include("esp_log.h")
#include "esp_log.h"
#define TFLITE_CAM_LOGI(...) ESP_LOGI(kTag, __VA_ARGS__)
#define TFLITE_CAM_LOGW(...) ESP_LOGW(kTag, __VA_ARGS__)
#define TFLITE_CAM_LOGE(...) ESP_LOGE(kTag, __VA_ARGS__)
#else
#define TFLITE_CAM_LOGI(...) do { printf("[tflite_cam][I] "); printf(__VA_ARGS__); printf("\n"); } while (0)
#define TFLITE_CAM_LOGW(...) do { printf("[tflite_cam][W] "); printf(__VA_ARGS__); printf("\n"); } while (0)
#define TFLITE_CAM_LOGE(...) do { printf("[tflite_cam][E] "); printf(__VA_ARGS__); printf("\n"); } while (0)
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
#define TFLITE_P4_HAS_ARDUINO_CAM_LIB 1
#else
#define TFLITE_P4_HAS_ARDUINO_CAM_LIB 0
#endif

#if TFLITE_P4_HAS_ARDUINO_CAM_LIB
#define TFLITE_HAS_IMX219 (CAMERA_TYPE == CAMERA_TYPE_IMX219 || CAMERA_TYPE == CAMERA_TYPE_AUTO)
#define TFLITE_HAS_OV5647 (CAMERA_TYPE == CAMERA_TYPE_OV5647 || CAMERA_TYPE == CAMERA_TYPE_AUTO)
#if TFLITE_HAS_IMX219
#include <ESP32_P4_IMX219.h>
#endif
#if TFLITE_HAS_OV5647
#include <ESP32_P4_OV5647.h>
#endif
#if !TFLITE_HAS_IMX219 && !TFLITE_HAS_OV5647
#error "CAMERA_TYPE must be CAMERA_TYPE_IMX219, CAMERA_TYPE_OV5647 or CAMERA_TYPE_AUTO"
#endif
#endif

#define TFLITE_P4_IMX219_HAS_ARDUINO_IMX219_LIB TFLITE_P4_HAS_ARDUINO_CAM_LIB

#if !TFLITE_P4_HAS_ARDUINO_CAM_LIB && __has_include("esp_video_init.h")
#ifdef CONFIG_ESP_VIDEO_ENABLE_MIPI_CSI_VIDEO_DEVICE
#undef CONFIG_ESP_VIDEO_ENABLE_MIPI_CSI_VIDEO_DEVICE
#endif
#define CONFIG_ESP_VIDEO_ENABLE_MIPI_CSI_VIDEO_DEVICE 1
#include "esp_video_init.h"
#include "esp_video_ioctl.h"
#include "esp_video_device.h"
#endif

// ── Camera wrapper: thin veneer over both library APIs ────────────────
// Both libraries have identical function signatures, only the prefix differs.
// In AUTO mode the active camera is resolved at runtime by I2C probing.
#if TFLITE_P4_HAS_ARDUINO_CAM_LIB

static int s_active_camera = CAMERA_TYPE;

#if CAMERA_TYPE == CAMERA_TYPE_AUTO

// Both sensors share one I2C bus (SCL=26, SDA=27, port 0).  IMX219 sits at
// 7-bit address 0x10, OV5647 at 0x36.
#include "driver/i2c_master.h"

static bool cam_i2c_probe(uint8_t addr7) {
  i2c_master_bus_config_t bcfg = {};
  bcfg.i2c_port = 0;
  bcfg.sda_io_num = GPIO_NUM_27;
  bcfg.scl_io_num = GPIO_NUM_26;
  bcfg.clk_source = I2C_CLK_SRC_DEFAULT;
  bcfg.glitch_ignore_cnt = 7;
  bcfg.flags.enable_internal_pullup = true;
  i2c_master_bus_handle_t bus = NULL;
  if (i2c_new_master_bus(&bcfg, &bus) != ESP_OK) return false;
  bool found = (i2c_master_probe(bus, addr7, 100) == ESP_OK);
  i2c_del_master_bus(bus);
  return found;
}

static void cam_xclk_temporary(uint32_t freq_hz) {
  // Some sensors only answer I2C once XCLK is running.  Start a temporary
  // LEDC clock on the shared XCLK pin; CameraBegin re-initialises it.
  ledc_timer_config_t t = {};
  t.speed_mode = LEDC_LOW_SPEED_MODE;
  t.timer_num = LEDC_TIMER_0;
  t.duty_resolution = LEDC_TIMER_1_BIT;
  t.freq_hz = freq_hz;
  t.clk_cfg = LEDC_AUTO_CLK;
  ledc_timer_config(&t);
  ledc_channel_config_t c = {};
  c.channel = LEDC_CHANNEL_0;
  c.duty = 1;
  c.gpio_num = GPIO_NUM_20;
  c.speed_mode = LEDC_LOW_SPEED_MODE;
  c.timer_sel = LEDC_TIMER_0;
  ledc_channel_config(&c);
}

static int detect_camera_type() {
  // Try a plain I2C probe first.
  if (cam_i2c_probe(0x10)) return CAMERA_TYPE_IMX219;
  if (cam_i2c_probe(0x36)) return CAMERA_TYPE_OV5647;

  // Retry with XCLK running (required by some sensors for I2C).
  cam_xclk_temporary(24000000);
  delay(10);
  int found = 0;
  if (cam_i2c_probe(0x10)) found = CAMERA_TYPE_IMX219;
  else if (cam_i2c_probe(0x36)) found = CAMERA_TYPE_OV5647;
  ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
  return found;
}
#endif  // CAMERA_TYPE == AUTO

static const char *camera_type_name(int t) {
  if (t == CAMERA_TYPE_IMX219) return "IMX219";
  if (t == CAMERA_TYPE_OV5647) return "OV5647";
  if (t == CAMERA_TYPE_AUTO)   return "AUTO";
  return "UNKNOWN";
}

bool CameraBegin() {
#if CAMERA_TYPE == CAMERA_TYPE_AUTO
  s_active_camera = detect_camera_type();
#endif
  bool ok = false;
  if (s_active_camera == CAMERA_TYPE_IMX219) {
#if TFLITE_HAS_IMX219
    ok = esp32_p4_imx219_begin();
#endif
  } else if (s_active_camera == CAMERA_TYPE_OV5647) {
#if TFLITE_HAS_OV5647
    ok = esp32_p4_ov5647_begin();
#endif
  }
  return ok;
}

bool CameraUpdate() {
  if (s_active_camera == CAMERA_TYPE_IMX219) {
#if TFLITE_HAS_IMX219
    return esp32_p4_imx219_update();
#endif
  }
#if TFLITE_HAS_OV5647
  return esp32_p4_ov5647_update();
#endif
  return false;
}

const uint8_t* CameraGetRgb() {
  if (s_active_camera == CAMERA_TYPE_IMX219) {
#if TFLITE_HAS_IMX219
    return esp32_p4_imx219_rgb();
#endif
  }
#if TFLITE_HAS_OV5647
  return esp32_p4_ov5647_rgb();
#endif
  return nullptr;
}

size_t CameraGetRgbSize() {
  if (s_active_camera == CAMERA_TYPE_IMX219) {
#if TFLITE_HAS_IMX219
    return esp32_p4_imx219_rgb_size();
#endif
  }
#if TFLITE_HAS_OV5647
  return esp32_p4_ov5647_rgb_size();
#endif
  return 0;
}

int CameraGetRgbWidth() {
  size_t sz = CameraGetRgbSize();
  // Square output: width = sqrt(sz / 3)
  int w = (int)sqrtf((float)(sz / 3));
  return (w > 0) ? w : OUT_WIDTH;
}

const char* CameraGetName() {
  if (s_active_camera == CAMERA_TYPE_IMX219) return "IMX219";
  if (s_active_camera == CAMERA_TYPE_OV5647) return "OV5647";
#if CAMERA_TYPE == CAMERA_TYPE_AUTO
  if (s_active_camera == CAMERA_TYPE_AUTO) {
    int t = detect_camera_type();
    if (t != 0) {
      s_active_camera = t;
      if (t == CAMERA_TYPE_IMX219) return "IMX219";
      if (t == CAMERA_TYPE_OV5647) return "OV5647";
    }
  }
#endif
  return "UNKNOWN";
}

// Resize library RGB (width src_w) to OUT_WIDTH×OUT_WIDTH×3 with WB gains,
// nearest-neighbour.  Used by GetImage and the RGB data-collection mode so
// both cameras always produce the same IMG_SIZE×IMG_SIZE format.
static void resize_rgb_wb(const uint8_t *src, int src_w, uint8_t *dst,
                          uint16_t wb_red, uint16_t wb_blue) {
  for (int y = 0; y < OUT_HEIGHT; y++) {
    int src_y = (int)((int64_t)y * src_w / OUT_WIDTH);
    for (int x = 0; x < OUT_WIDTH; x++) {
      int src_x = (int)((int64_t)x * src_w / OUT_WIDTH);
      int si = (src_y * src_w + src_x) * 3;
      int di = (y * OUT_WIDTH + x) * 3;
      int r = (int)src[si + 0] * wb_red / 100;  if (r > 255) r = 255;
      int g = (int)src[si + 1];
      int b = (int)src[si + 2] * wb_blue / 100; if (b > 255) b = 255;
      dst[di + 0] = (uint8_t)r;
      dst[di + 1] = (uint8_t)g;
      dst[di + 2] = (uint8_t)b;
    }
  }
}

void CameraSendRgbToSerialWb(uint16_t wb_red, uint16_t wb_blue) {
  static uint8_t buf[OUT_WIDTH * OUT_HEIGHT * 3];
  resize_rgb_wb(CameraGetRgb(), CameraGetRgbWidth(), buf, wb_red, wb_blue);
  // Data-collection sync header: 0xAA 0x55 0xAA + IMG_SIZE×IMG_SIZE×3
  const uint8_t sync[3] = {0xAA, 0x55, 0xAA};
  Serial.write(sync, 3);
  Serial.write(buf, sizeof(buf));
}

// Latest frame resized to IMG_SIZE×IMG_SIZE×3 (nearest-neighbour, WB
// passthrough 100/100).  Capture modes MUST use this instead of
// CameraGetRgb() directly: the OV5647 library buffer is 160×160×3 and
// streaming its first IMG_SIZE×IMG_SIZE×3 bytes (row-major) tears the
// frame — the host would reshape garbage into 96×96.  IMX219 (library
// output already 96×96) is an identity resize.
const uint8_t* CameraGetRgbImgSized() {
  static uint8_t buf[OUT_WIDTH * OUT_HEIGHT * 3];
  resize_rgb_wb(CameraGetRgb(), CameraGetRgbWidth(), buf, 100, 100);
  return buf;
}

// Begin-once guard shared by GetImage and the capture modes in TFLite.ino.
// The IMX219/OV5647 library begin() is NOT re-entrant: calling it a second
// time (e.g. when a runtime mode switch re-inits a capture mode) re-runs the
// sensor init and can hang the I2C bus — observed on the new P4 board as a
// completely silent device after "mode rgb".  ALL camera paths must go
// through this guard instead of calling CameraBegin() directly.
static bool s_camera_begun_once = false;

bool ImageProviderEnsureCamera() {
  if (s_camera_begun_once) return true;
  s_camera_begun_once = CameraBegin();
  return s_camera_begun_once;
}
#endif  // TFLITE_P4_HAS_ARDUINO_CAM_LIB

static int s_last_lut_mode = -1;

#if !TFLITE_P4_IMX219_HAS_ARDUINO_IMX219_LIB && __has_include("esp_video_init.h")
#define TFLITE_P4_IMX219_HAS_ESP_VIDEO 1
#else
#define TFLITE_P4_IMX219_HAS_ESP_VIDEO 0
#endif

static int *s_x_lut = NULL;
static int *s_y_lut = NULL;
static uint32_t s_raw_bytesperline = 0;  // actual V4L2 stride (may differ from IMG_WIDTH*5/4)

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

    int crop_w, crop_h, x_offset, y_offset;

    // Sign ROI: center bar x:35-65%, y:20-80%
    // Matches Python: _SEARCH_LEFT=0.35 _SEARCH_RIGHT=0.65 _SEARCH_TOP=0.20 _SEARCH_BOTTOM=0.80
    const float sign_search_left_frac   = 0.35f;
    const float sign_search_right_frac  = 0.65f;
    const float sign_search_top_frac    = 0.20f;
    const float sign_search_bottom_frac = 0.80f;

    int search_left   = (int)(width  * sign_search_left_frac);
    int search_right  = (int)(width  * sign_search_right_frac);
    int search_top    = (int)(height * sign_search_top_frac);
    int search_bottom = (int)(height * sign_search_bottom_frac);

    int search_w = search_right - search_left;
    int search_h = search_bottom - search_top;

    // Take the largest square that fits inside the search window.
    crop_w = (search_w < search_h) ? search_w : search_h;
    crop_h = crop_w;

    // Center the crop within the search window.
    x_offset = search_left + (search_w - crop_w) / 2;
    y_offset = search_top  + (search_h - crop_h) / 2;

    // Clamp to image bounds.
    if (x_offset < 0) x_offset = 0;
    if (y_offset < 0) y_offset = 0;
    if (x_offset + crop_w > width)  x_offset = width  - crop_w;
    if (y_offset + crop_h > height) y_offset = height - crop_h;

    float x_step = (float)crop_w / OUT_WIDTH;
    float y_step = (float)crop_h / OUT_HEIGHT;

    for (int y = 0; y < OUT_HEIGHT; y++) {
        s_y_lut[y] = (y_offset + (int)(y * y_step + 0.5f)) & ~1;
    }
    for (int x = 0; x < OUT_WIDTH; x++) {
        s_x_lut[x] = (x_offset + (int)(x * x_step + 0.5f)) & ~1;
    }

    s_last_lut_mode = 0;
}

// Demosaic BGGR raw 10-bit data to RGB
static void demosaic_bggr_to_rgb(const uint8_t *raw10, uint8_t *rgb, int width, int height) {
    if (s_x_lut == NULL || s_y_lut == NULL) {
        init_demosaic_luts(width, height);
    }

    int raw_stride = (int)(s_raw_bytesperline > 0 ? s_raw_bytesperline : (uint32_t)(width * 5 / 4));
    for (int y = 0; y < OUT_HEIGHT; y++) {
        int src_y = s_y_lut[y];
        int row0 = src_y * raw_stride;
        int row1 = (src_y + 1) * raw_stride;
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
  s_raw_bytesperline = fmt.fmt.pix.bytesperline;
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


// ── Auto shadow-search crop box (host _focus_bbox C port) ──────────────
// BIT-IDENTICAL to AItraining image_preprocess.py: deterministic float32
// cumsum box blur, float64 geometry chain, Python round-half-even.
// find_search_box_adaptive() = exposure-adaptive band selection (smoothed-
// histogram valleys -> per-band search -> bright-ring gate), the mirror of
// _focus_bbox_adaptive.  Verified 65/65 synthetic + 22/22 adaptive boxes
// against the host.  Picks the square crop the model sees when
// BG_ENABLE_FOCUS_SEARCH=1.  Keep in lockstep with image_preprocess.py.
typedef struct { int x1, y1, x2, y2; } fb_box_t;

#define FB_W OUT_WIDTH
#define FB_H OUT_HEIGHT

static double py_round_d(double x) {
  double f = floor(x);
  if (x - f == 0.5) {
    long r = (long)f;
    return (r & 1) ? (double)(r + 1) : (double)r;  // round to nearest EVEN
  }
  return floor(x + 0.5);
}
static long py_round_l(double x) { return (long)py_round_d(x); }

static int percentile_from_hist(const uint32_t *hist, int total, double q) {
  if (total <= 0) return 0;
  // rank = max(0, min(total-1, round((total-1)*q)))  (python round ~= floor(x+.5))
  int rank = (int)py_round_d((double)(total - 1) * q);
  if (rank < 0) rank = 0;
  if (rank > total - 1) rank = total - 1;
  uint32_t cdf = 0;
  for (int i = 0; i < 256; i++) {
    cdf += hist[i];
    if (cdf >= (uint32_t)(rank + 1)) {
      int idx = i;
      if (idx < 0) idx = 0;
      if (idx > 255) idx = 255;
      return idx;
    }
  }
  return 255;
}

// region edge map: (|dx| + |dy|) / 2, borders replicate.  Matches _edge_map_u8.
static void edge_map_u8(const uint8_t *region, int w, int h, uint8_t *edge) {
  for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++) {
      int xl = (x > 0) ? region[y * w + x - 1] : region[y * w + x];
      int xr = (x < w - 1) ? region[y * w + x + 1] : region[y * w + x];
      int yu = (y > 0) ? region[(y - 1) * w + x] : region[y * w + x];
      int yd = (y < h - 1) ? region[(y + 1) * w + x] : region[y * w + x];
      int gx = xr - xl; if (gx < 0) gx = -gx;
      int gy = yd - yu; if (gy < 0) gy = -gy;
      int e = (gx + gy) / 2;
      if (e > 255) e = 255;
      edge[y * w + x] = (uint8_t)e;
    }
  }
}

// 4-connected bbox from seed (matches _connected_bbox_from_seed), inclusive.
static int connected_bbox_from_seed(const uint8_t *mask, int w, int h,
                                    int seed_x, int seed_y,
                                    int *out_x1, int *out_y1, int *out_x2, int *out_y2) {
  if (h <= 0 || w <= 0) return 0;
  int sx = seed_x; if (sx < 0) sx = 0; if (sx > w - 1) sx = w - 1;
  int sy = seed_y; if (sy < 0) sy = 0; if (sy > h - 1) sy = h - 1;
  if (!mask[sy * w + sx]) return 0;
  static uint8_t visited[FB_W * FB_H];
  memset(visited, 0, sizeof(visited));
  static int stack[FB_W * FB_H * 2];
  int sp = 0;
  stack[sp++] = sx; stack[sp++] = sy;
  visited[sy * w + sx] = 1;
  int min_x = sx, max_x = sx, min_y = sy, max_y = sy;
  while (sp > 0) {
    int py = stack[--sp];
    int px = stack[--sp];
    if (px < min_x) min_x = px; if (px > max_x) max_x = px;
    if (py < min_y) min_y = py; if (py > max_y) max_y = py;
    if (px > 0 && mask[py * w + px - 1] && !visited[py * w + px - 1]) { visited[py * w + px - 1] = 1; stack[sp++] = px - 1; stack[sp++] = py; }
    if (px + 1 < w && mask[py * w + px + 1] && !visited[py * w + px + 1]) { visited[py * w + px + 1] = 1; stack[sp++] = px + 1; stack[sp++] = py; }
    if (py > 0 && mask[(py - 1) * w + px] && !visited[(py - 1) * w + px]) { visited[(py - 1) * w + px] = 1; stack[sp++] = px; stack[sp++] = py - 1; }
    if (py + 1 < h && mask[(py + 1) * w + px] && !visited[(py + 1) * w + px]) { visited[(py + 1) * w + px] = 1; stack[sp++] = px; stack[sp++] = py + 1; }
  }
  *out_x1 = min_x; *out_y1 = min_y; *out_x2 = max_x; *out_y2 = max_y;
  return 1;
}

// weighted center inside bbox (matches _weighted_center_in_bbox), float32.
static void weighted_center_in_bbox(const float *weight_map, int w, int h,
                                    int bx1, int by1, int bx2, int by2,
                                    double fb_cx, double fb_cy,
                                    double *cx, double *cy) {
  int x0 = bx1; if (x0 < 0) x0 = 0; if (x0 > w - 1) x0 = w - 1;
  int y0 = by1; if (y0 < 0) y0 = 0; if (y0 > h - 1) y0 = h - 1;
  int x1 = bx2 + 1; if (x1 < x0 + 1) x1 = x0 + 1; if (x1 > w) x1 = w;
  int y1 = by2 + 1; if (y1 < y0 + 1) y1 = y0 + 1; if (y1 > h) y1 = h;
  double total = 0.0, wx = 0.0, wy = 0.0;
  for (int y = y0; y < y1; y++) {
    for (int x = x0; x < x1; x++) {
      double v = (double)weight_map[y * w + x];
      if (v < 0.0) v = 0.0;
      total += v;
      wx += v * (double)x;
      wy += v * (double)y;
    }
  }
  if (total <= 1e-6) { *cx = fb_cx; *cy = fb_cy; return; }
  *cx = wx / total;
  *cy = wy / total;
}

// Separable box convolution via running sums (cumsum), zero-padded with the
// kernel centered.  float32, sequential accumulation — BIT-IDENTICAL to the
// host np.cumsum implementation (out[j] = P[j+K] - P[j] over the padded row).
static void box_blur(const float *src, int w, int h, int K, float *tmp, float *dst) {
  int half = K / 2;
  float vpad = 0.0f;  // helper macro-less: inline bounds checks below
  (void)vpad;
  // horizontal pass (axis 1)
  for (int y = 0; y < h; y++) {
    float Pj = 0.0f, Pjk = 0.0f;
    for (int i = 0; i < K; i++) {
      int t = i - half;
      if (t >= 0 && t < w) Pjk += src[y * w + t];
    }
    for (int j = 0; j < w; j++) {
      tmp[y * w + j] = Pjk - Pj;
      int t1 = j + K - half;
      if (t1 >= 0 && t1 < w) Pjk += src[y * w + t1];
      int t2 = j - half;
      if (t2 >= 0 && t2 < w) Pj += src[y * w + t2];
    }
  }
  // vertical pass (axis 0)
  for (int x = 0; x < w; x++) {
    float Pj = 0.0f, Pjk = 0.0f;
    for (int i = 0; i < K; i++) {
      int t = i - half;
      if (t >= 0 && t < h) Pjk += tmp[t * w + x];
    }
    for (int j = 0; j < h; j++) {
      dst[j * w + x] = Pjk - Pj;
      int t1 = j + K - half;
      if (t1 >= 0 && t1 < h) Pjk += tmp[t1 * w + x];
      int t2 = j - half;
      if (t2 >= 0 && t2 < h) Pj += tmp[t2 * w + x];
    }
  }
}

#define FB_SEARCH_LEFT   0.10
#define FB_SEARCH_RIGHT  0.90
#define FB_SEARCH_TOP    0.10
#define FB_SEARCH_BOTTOM 0.90
#define FB_FALLBACK_SIDE_FRAC 0.40
#define FB_FALLBACK_CX   0.50
#define FB_FALLBACK_CY   0.50
#define FB_DARK_OFFSET   10
#define FB_EDGE_PERCENTILE 0.90
#define FB_EDGE_MIN      18
#define FB_EDGE_RELAX    8
#define FB_MIN_FOCUS_PIXELS 8
#define FB_PROJECTION_FRAC 0.16
#define FB_PRIOR_CX      0.50
#define FB_PRIOR_CY      0.50
#define FB_PRIOR_SX      0.18
#define FB_PRIOR_SY      0.16
#define FB_LOCAL_PEAK_RATIO 0.68
#define FB_LOCAL_SIDE_SCALE 2.35
#define FB_SUPPORT_PEAK_RATIO 0.40
#define FB_SUPPORT_SIDE_SCALE 1.90
#define FB_ASPECT_TARGET  1.0
#define FB_ASPECT_TOL     0.75
#define FB_FAR_SIDE_FRAC  0.12
#define FB_FAR_SIDE_BOOST 1.16
#define FB_CLOSE_SIDE_FRAC 0.22
#define FB_CLOSE_SIDE_BOOST 1.42
#define FB_BORDER_TOUCH_PX 2
#define FB_BORDER_SIDE_BOOST 1.24
#define FB_BORDER_SHIFT_FRAC 0.16
#define FB_PEAK_CENTER_BLEND 0.68
#define FB_CENTER_CLAMP_FRAC 0.22
#define FB_MIN_SIDE_FRAC 0.25
#define FB_CENTER_BIAS_X 0.10
#define FB_CENTER_BIAS_Y -0.03
#define FB_MAX_RUNS 4096

static fb_box_t find_search_box(const uint8_t *gray) {
  fb_box_t out = {0, 0, 0, 0};
  static float score_map[FB_W * FB_H];
  static float tmp_buf[FB_W * FB_H];
  static float target_map[FB_W * FB_H];
  static uint8_t region[FB_W * FB_H];
  static uint8_t edge[FB_W * FB_H];
  static uint8_t mask[FB_W * FB_H];
  static uint8_t local_mask[FB_W * FB_H];
  static uint8_t support_mask[FB_W * FB_H];
  static uint8_t visited[FB_W * FB_H];
  static uint32_t hist[256];

  const int w = FB_W, h = FB_H;
  int left  = (int)py_round_d((double)w * FB_SEARCH_LEFT);
  int right = (int)py_round_d((double)w * FB_SEARCH_RIGHT);
  int top   = (int)py_round_d((double)h * FB_SEARCH_TOP);
  int bottom= (int)py_round_d((double)h * FB_SEARCH_BOTTOM);
  if (left < 0) left = 0; if (left > w - 1) left = w - 1;
  if (right < left + 1) right = left + 1; if (right > w) right = w;
  if (top < 0) top = 0; if (top > h - 1) top = h - 1;
  if (bottom < top + 1) bottom = top + 1; if (bottom > h) bottom = h;
  int sw = right - left, sh = bottom - top;

  // region copy
  for (int y = 0; y < sh; y++)
    for (int x = 0; x < sw; x++)
      region[y * sw + x] = gray[(top + y) * w + (left + x)];
  int total = sw * sh;
  if (total <= 0) {
    // fallback bbox (host _fallback_bbox)
    long side = py_round_l((double)(w < h ? w : h) * FB_FALLBACK_SIDE_FRAC);
    if (side < 1) side = 1;
    long cx = py_round_l((double)w * FB_FALLBACK_CX);
    long cy = py_round_l((double)h * FB_FALLBACK_CY);
    out.x1 = cx - side / 2; if (out.x1 < 0) out.x1 = 0; if (out.x1 > w - side) out.x1 = w - side;
    out.y1 = cy - side / 2; if (out.y1 < 0) out.y1 = 0; if (out.y1 > h - side) out.y1 = h - side;
    out.x2 = out.x1 + side; out.y2 = out.y1 + side;
    return out;
  }

  // hist + percentiles
  memset(hist, 0, sizeof(hist));
  for (int i = 0; i < total; i++) hist[region[i]]++;
  int p20 = percentile_from_hist(hist, total, 0.20);
  int p35 = percentile_from_hist(hist, total, 0.35);
  int thr = (p35 < p20 + FB_DARK_OFFSET) ? p35 : (p20 + FB_DARK_OFFSET);
  int dark_thr = thr;  // pure-dark threshold for the support seed

  edge_map_u8(region, sw, sh, edge);
  memset(hist, 0, sizeof(hist));
  for (int i = 0; i < total; i++) hist[edge[i]]++;
  int edge_thr = percentile_from_hist(hist, total, (double)FB_EDGE_PERCENTILE);
  if (edge_thr < FB_EDGE_MIN) edge_thr = FB_EDGE_MIN;

  // mask + target map
  int mask_count = 0;
  for (int y = 0; y < sh; y++) {
    for (int x = 0; x < sw; x++) {
      int i = y * sw + x;
      uint8_t m = (region[i] <= thr && edge[i] >= edge_thr) ? 1 : 0;
      mask[i] = m;
      mask_count += m;
      int ds = thr - (int)region[i]; if (ds < 0) ds = 0;
      int es = (int)edge[i] - edge_thr; if (es < 0) es = 0;
      target_map[i] = (float)ds * ((float)es + 1.0f);
    }
  }
  int relax_thr = edge_thr;
  if (mask_count < FB_MIN_FOCUS_PIXELS) {
    relax_thr = edge_thr - 6;
    if (relax_thr < FB_EDGE_RELAX) relax_thr = FB_EDGE_RELAX;
    dark_thr = p20;
    mask_count = 0;
    for (int y = 0; y < sh; y++) {
      for (int x = 0; x < sw; x++) {
        int i = y * sw + x;
        uint8_t m = (region[i] <= p20 && edge[i] >= relax_thr) ? 1 : 0;
        mask[i] = m;
        mask_count += m;
        int ds = p20 - (int)region[i]; if (ds < 0) ds = 0;
        int es = (int)edge[i] - relax_thr; if (es < 0) es = 0;
        target_map[i] = (float)ds * ((float)es + 1.0f);
      }
    }
  }
  if (mask_count < FB_MIN_FOCUS_PIXELS) goto fallback;

  {
    // score map: separable box convolution + gaussian priors
    int K = (int)py_round_d((double)(sh < sw ? sh : sw) * FB_PROJECTION_FRAC);
    if (K < 3) K = 3;
    if (K % 2 == 0) K += 1;
    box_blur(target_map, sw, sh, K, tmp_buf, score_map);
    double peak = 0.0;
    for (int i = 0; i < total; i++) if (score_map[i] > peak) peak = score_map[i];
    if (peak <= 0.0) goto fallback;

    // priors: linspace in float32 (double math cast to float), exp in double
    // then cast to float32 — matches numpy's float32 prior arrays.
    // host: xs = linspace(f32) then float64 arithmetic: (xs - 0.5)/sigma etc.,
    // exp in float64 -> prior is float64.  Replicate exactly.
    for (int y = 0; y < sh; y++) {
      double ys = (double)(float)((double)y / (double)(sh - 1));
      double dy = (ys - (double)FB_PRIOR_CY) / (double)FB_PRIOR_SY;
      double py = exp(-0.5 * dy * dy);
      for (int x = 0; x < sw; x++) {
        double xs = (double)(float)((double)x / (double)(sw - 1));
        double dx = (xs - (double)FB_PRIOR_CX) / (double)FB_PRIOR_SX;
        double px = exp(-0.5 * dx * dx);
        score_map[y * sw + x] = (score_map[y * sw + x] * py) * px;
      }
    }
    peak = 0.0;
    for (int i = 0; i < total; i++) if (score_map[i] > peak) peak = score_map[i];
    if (peak <= 0.0) goto fallback;

    double local_thr = peak * (double)FB_LOCAL_PEAK_RATIO;
    for (int i = 0; i < total; i++) local_mask[i] = score_map[i] >= local_thr ? 1 : 0;

    memset(visited, 0, sizeof(visited));
    float best_score = -1.0f;
    int best_span_w = 0, best_span_h = 0;
    int best_bx1 = 0, best_by1 = 0, best_bx2 = 0, best_by2 = 0;
    int best_peak_x = 0, best_peak_y = 0;

    for (int y = 0; y < sh; y++) {
      for (int x = 0; x < sw; x++) {
        int i0 = y * sw + x;
        if (!local_mask[i0] || visited[i0]) continue;
        // BFS this component
        static int st[FB_MAX_RUNS * 2]; int sp = 0;  // static: 32 KB would overflow the task stack
        st[sp++] = x; st[sp++] = y;
        visited[i0] = 1;
        int min_x = x, max_x = x, min_y = y, max_y = y;
        double wsum = 0.0, wx = 0.0, wy = 0.0;
        double lpeak = -1.0; int lpx = x, lpy = y;
        while (sp > 0) {
          int py = st[--sp];
          int px = st[--sp];
          int ii = py * sw + px;
          double v = (double)score_map[ii];
          wsum += v; wx += v * (double)px; wy += v * (double)py;
          if (v > lpeak) { lpeak = v; lpx = px; lpy = py; }
          if (px < min_x) min_x = px; if (px > max_x) max_x = px;
          if (py < min_y) min_y = py; if (py > max_y) max_y = py;
          if (px > 0 && local_mask[py * sw + px - 1] && !visited[py * sw + px - 1]) { visited[py * sw + px - 1] = 1; st[sp++] = px - 1; st[sp++] = py; }
          if (px + 1 < sw && local_mask[py * sw + px + 1] && !visited[py * sw + px + 1]) { visited[py * sw + px + 1] = 1; st[sp++] = px + 1; st[sp++] = py; }
          if (py > 0 && local_mask[(py - 1) * sw + px] && !visited[(py - 1) * sw + px]) { visited[(py - 1) * sw + px] = 1; st[sp++] = px; st[sp++] = py - 1; }
          if (py + 1 < sh && local_mask[(py + 1) * sw + px] && !visited[(py + 1) * sw + px]) { visited[(py + 1) * sw + px] = 1; st[sp++] = px; st[sp++] = py + 1; }
        }
        if (wsum <= 0.0) continue;
        // Background rejection: components touching >= 2 borders of the
        // search region are the desk/surface, not the sign.
        {
          int tcount = (min_x <= FB_BORDER_TOUCH_PX ? 1 : 0)
                     + (min_y <= FB_BORDER_TOUCH_PX ? 1 : 0)
                     + (max_x >= sw - 1 - FB_BORDER_TOUCH_PX ? 1 : 0)
                     + (max_y >= sh - 1 - FB_BORDER_TOUCH_PX ? 1 : 0);
          if (tcount >= 2) continue;
        }
        int span_w = max_x - min_x + 1, span_h = max_y - min_y + 1;
        double aspect = (double)span_w / (double)(span_h > 0 ? span_h : 1);
        double aspect_err = aspect - (double)FB_ASPECT_TARGET; if (aspect_err < 0) aspect_err = -aspect_err;
        if (aspect_err > (double)FB_ASPECT_TOL) continue;
        double aspect_score = 1.0 - aspect_err / (double)FB_ASPECT_TOL;
        if (aspect_score < 0.2) aspect_score = 0.2;
        double area_score = (double)(span_w * span_h) / (double)(K * K > 0 ? K * K : 1);
        if (area_score > 1.0) area_score = 1.0;
        double cx = (wx / wsum) / (double)(sw - 1 > 0 ? sw - 1 : 1);
        double cy = (wy / wsum) / (double)(sh - 1 > 0 ? sh - 1 : 1);
        double dx = (cx - (double)FB_PRIOR_CX) / (double)FB_PRIOR_SX;
        double dy = (cy - (double)FB_PRIOR_CY) / (double)FB_PRIOR_SY;
        double csx = exp(-0.5 * dx * dx);
        double csy = exp(-0.5 * dy * dy);
        double center_score = 0.35 + 0.65 * csx * csy;
        double sc = wsum * aspect_score * (0.6 + 0.4 * area_score) * center_score;
        if (sc > best_score) {
          best_score = sc;
          best_span_w = span_w; best_span_h = span_h;
          best_bx1 = min_x; best_by1 = min_y; best_bx2 = max_x; best_by2 = max_y;
          best_peak_x = lpx; best_peak_y = lpy;
        }
      }
    }
    if (best_score < 0.0f) goto fallback;

    double component_side_frac = (double)(best_span_w > best_span_h ? best_span_w : best_span_h) /
                                 (double)((sh < sw ? sh : sw) > 0 ? (sh < sw ? sh : sw) : 1);
    // Support bbox = the DARK-MASK connected component seeded at the peak
    // (NOT the score-based support mask): flat dark sign interiors have ~0
    // edge weight, so the score support ring only covers border fragments
    // and the crop misses large flat signs.  Score mask = fallback.
    int sx1, sy1, sx2, sy2;
    for (int i = 0; i < total; i++) support_mask[i] = region[i] <= dark_thr ? 1 : 0;
    if (!connected_bbox_from_seed(support_mask, sw, sh, best_peak_x, best_peak_y, &sx1, &sy1, &sx2, &sy2)
        || ((sx1 <= FB_BORDER_TOUCH_PX ? 1 : 0) + (sy1 <= FB_BORDER_TOUCH_PX ? 1 : 0)
            + (sx2 >= sw - 1 - FB_BORDER_TOUCH_PX ? 1 : 0) + (sy2 >= sh - 1 - FB_BORDER_TOUCH_PX ? 1 : 0)) >= 2) {
      double sup_thr = (double)peak * (double)FB_SUPPORT_PEAK_RATIO;
      for (int i = 0; i < total; i++) support_mask[i] = (double)score_map[i] >= sup_thr ? 1 : 0;
      if (!connected_bbox_from_seed(support_mask, sw, sh, best_peak_x, best_peak_y, &sx1, &sy1, &sx2, &sy2)) {
        sx1 = best_bx1; sy1 = best_by1; sx2 = best_bx2; sy2 = best_by2;
      }
    }
    int support_span_w = sx2 - sx1 + 1, support_span_h = sy2 - sy1 + 1;
    int support_span = (support_span_w > support_span_h) ? support_span_w : support_span_h;
    double est_a = (double)(best_span_w > best_span_h ? best_span_w : best_span_h) * (double)FB_LOCAL_SIDE_SCALE;
    double est_b = (double)support_span * (double)FB_SUPPORT_SIDE_SCALE;
    long est_side = py_round_l(est_a > est_b ? est_a : est_b);
    if (component_side_frac <= (double)FB_FAR_SIDE_FRAC) est_side = py_round_l((double)est_side * (double)FB_FAR_SIDE_BOOST);
    if (component_side_frac >= (double)FB_CLOSE_SIDE_FRAC) est_side = py_round_l((double)est_side * (double)FB_CLOSE_SIDE_BOOST);
    int touch_left = (sx1 <= FB_BORDER_TOUCH_PX) ? 1 : 0;
    int touch_top  = (sy1 <= FB_BORDER_TOUCH_PX) ? 1 : 0;
    int touch_right = (sx2 >= sw - 1 - FB_BORDER_TOUCH_PX) ? 1 : 0;
    int touch_bottom = (sy2 >= sh - 1 - FB_BORDER_TOUCH_PX) ? 1 : 0;
    if (touch_left || touch_top || touch_right || touch_bottom)
      est_side = py_round_l((double)est_side * (double)FB_BORDER_SIDE_BOOST);
    if (est_side < K) est_side = K;
    long min_side = py_round_l((double)(sh < sw ? sh : sw) * FB_MIN_SIDE_FRAC);
    if (est_side < min_side) est_side = min_side;
    long max_crop_side = (w < h) ? w : h;
    if (est_side > max_crop_side) est_side = max_crop_side;

    double sup_cx, sup_cy;
    weighted_center_in_bbox(target_map, sw, sh, sx1, sy1, sx2, sy2,
                            (double)(sx1 + sx2) * 0.5, (double)(sy1 + sy2) * 0.5,
                            &sup_cx, &sup_cy);
    double peak_cx = (double)best_peak_x, peak_cy = (double)best_peak_y;
    double close_ratio = (component_side_frac - (double)FB_CLOSE_SIDE_FRAC) / (0.40 - (double)FB_CLOSE_SIDE_FRAC);
    if (close_ratio < 0.0) close_ratio = 0.0;
    if (close_ratio > 1.0) close_ratio = 1.0;
    double bias_x = (double)FB_CENTER_BIAS_X * (1.0 - 0.75 * close_ratio);
    double bias_y = (double)FB_CENTER_BIAS_Y * (1.0 - 0.50 * close_ratio);
    double cx = ((double)FB_PEAK_CENTER_BLEND * peak_cx) + ((1.0 - (double)FB_PEAK_CENTER_BLEND) * (double)sup_cx);
    double cy = ((double)FB_PEAK_CENTER_BLEND * peak_cy) + ((1.0 - (double)FB_PEAK_CENTER_BLEND) * (double)sup_cy);
    cx += (double)est_side * (bias_x + (double)FB_BORDER_SHIFT_FRAC * (double)touch_right - (double)FB_BORDER_SHIFT_FRAC * (double)touch_left);
    cy += (double)est_side * (bias_y + (double)FB_BORDER_SHIFT_FRAC * (double)touch_bottom - (double)FB_BORDER_SHIFT_FRAC * (double)touch_top);
    double support_center_x = (double)(sx1 + sx2) * 0.5;
    double support_center_y = (double)(sy1 + sy2) * 0.5;
    double support_span_f = (double)((sx2 - sx1 + 1) > (sy2 - sy1 + 1) ? (sx2 - sx1 + 1) : (sy2 - sy1 + 1));
    double center_limit = 2.0;
    if (support_span_f * 0.5 > center_limit) center_limit = support_span_f * 0.5;
    if ((double)est_side * (double)FB_CENTER_CLAMP_FRAC > center_limit) center_limit = (double)est_side * (double)FB_CENTER_CLAMP_FRAC;
    if (cx < support_center_x - center_limit) cx = support_center_x - center_limit;
    if (cx > support_center_x + center_limit) cx = support_center_x + center_limit;
    if (cy < support_center_y - center_limit) cy = support_center_y - center_limit;
    if (cy > support_center_y + center_limit) cy = support_center_y + center_limit;
    cx += (double)left;
    cy += (double)top;
    if (est_side > max_crop_side) est_side = max_crop_side;
    long crop_left = py_round_l(cx - (double)est_side * 0.5);
    long crop_top  = py_round_l(cy - (double)est_side * 0.5);
    if (crop_left < 0) crop_left = 0;
    if (crop_left > (long)w - est_side) crop_left = (long)w - est_side;
    if (crop_top < 0) crop_top = 0;
    if (crop_top > (long)h - est_side) crop_top = (long)h - est_side;
    out.x1 = (int)crop_left; out.y1 = (int)crop_top; out.x2 = (int)(crop_left + est_side); out.y2 = (int)(crop_top + est_side);
    return out;
  }

fallback:
  {
    long side = py_round_l((double)(w < h ? w : h) * FB_FALLBACK_SIDE_FRAC);
    if (side < 1) side = 1;
    long cx = py_round_l((double)w * FB_FALLBACK_CX);
    long cy = py_round_l((double)h * FB_FALLBACK_CY);
    out.x1 = (int)(cx - side / 2); if (out.x1 < 0) out.x1 = 0; if (out.x1 > (int)w - (int)side) out.x1 = (int)w - (int)side;
    out.y1 = (int)(cy - side / 2); if (out.y1 < 0) out.y1 = 0; if (out.y1 > (int)h - (int)side) out.y1 = (int)h - (int)side;
    out.x2 = out.x1 + (int)side; out.y2 = out.y1 + (int)side;
    return out;
  }
}

// ── Exposure-adaptive band selection (host _focus_bbox_adaptive mirror) ──
// Candidate gray bands from smoothed-histogram valleys; per band run the
// classic search; keep the box whose ring is bright paper (>=35% pixels
// >= 180).  BIT-IDENTICAL sequence to the host wrapper.
static fb_box_t find_search_box_adaptive(const uint8_t *gray_raw) {
  fb_box_t out = {0, 0, 0, 0};
  static double smooth[256];
  static uint8_t band_gray[FB_W * FB_H];
  static uint32_t raw_hist[256];

  const int w = FB_W, h = FB_H;
  int left  = (int)py_round_d((double)w * FB_SEARCH_LEFT);
  int right = (int)py_round_d((double)w * FB_SEARCH_RIGHT);
  int top   = (int)py_round_d((double)h * FB_SEARCH_TOP);
  int bottom= (int)py_round_d((double)h * FB_SEARCH_BOTTOM);
  if (left < 0) left = 0; if (left > w - 1) left = w - 1;
  if (right < left + 1) right = left + 1; if (right > w) right = w;
  if (top < 0) top = 0; if (top > h - 1) top = h - 1;
  if (bottom < top + 1) bottom = top + 1; if (bottom > h) bottom = h;
  int sw = right - left, sh = bottom - top;
  int total = sw * sh;
  if (total <= 0) return find_search_box(gray_raw);

  memset(raw_hist, 0, sizeof(raw_hist));
  for (int y = 0; y < sh; y++)
    for (int x = 0; x < sw; x++)
      raw_hist[gray_raw[(top + y) * w + (left + x)]]++;

  // smooth with a length-11 box, zero-padded (np.convolve 'same')
  for (int i = 0; i < 256; i++) {
    double s = 0.0;
    for (int k = 0; k < 11; k++) {
      int t = i + k - 5;
      if (t >= 0 && t < 256) s += (double)raw_hist[t];
    }
    smooth[i] = s / 11.0;
  }

  int peaks[64]; int n_peaks = 0;
  for (int i = 2; i < 254; i++) {
    if (smooth[i] >= smooth[i - 1] && smooth[i] > smooth[i + 1]
        && smooth[i] >= 0.03 * (double)total / 8.0) {
      if (n_peaks < 64) peaks[n_peaks++] = i;
    }
  }
  if (n_peaks < 2) {
    n_peaks = 2;
    peaks[0] = percentile_from_hist(raw_hist, total, 0.15);
    peaks[1] = percentile_from_hist(raw_hist, total, 0.85);
  }

  // candidate bands: (argmin between a,b, hi=b) pairs + (0, first valley)
  int lo_arr[64], hi_arr[64]; int n_bands = 0;
  for (int k = 0; k + 1 < n_peaks; k++) {
    int a = peaks[k], b = peaks[k + 1];
    int best_i = a;
    double best_v = smooth[a];
    for (int i = a + 1; i <= b; i++) {
      if (smooth[i] < best_v) { best_v = smooth[i]; best_i = i; }
    }
    lo_arr[n_bands] = best_i; hi_arr[n_bands] = b; n_bands++;
  }
  {
    int p0 = peaks[0];
    int best_i = 0;
    double best_v = smooth[0];
    for (int i = 1; i <= p0; i++) {
      if (smooth[i] < best_v) { best_v = smooth[i]; best_i = i; }
    }
    lo_arr[n_bands] = 0;
    hi_arr[n_bands] = p0 + best_i;   // host: v1 = peaks[0] + argmin(smooth[:peaks[0]+1])
    if (hi_arr[n_bands] <= 0) hi_arr[n_bands] = 100;
    n_bands++;
  }

  double best_score = -1.0;
  int have_best = 0;
  for (int k = 0; k < n_bands; k++) {
    int lo = lo_arr[k], hi = hi_arr[k];
    if (hi - lo < 10) continue;
    for (int i = 0; i < w * h; i++) {
      int v = gray_raw[i];
      band_gray[i] = (v > lo && v < hi) ? (uint8_t)v : 255;
    }
    fb_box_t box = find_search_box(band_gray);
    // ring bright fraction on the RAW gray
    int pad = 4;
    int ry1 = box.y1 - pad; if (ry1 < 0) ry1 = 0;
    int ry2 = box.y2 + pad; if (ry2 > h) ry2 = h;
    int rx1 = box.x1 - pad; if (rx1 < 0) rx1 = 0;
    int rx2 = box.x2 + pad; if (rx2 > w) rx2 = w;
    long cnt = 0, bcnt = 0;
    for (int y = ry1; y < ry2; y++) {
      for (int x = rx1; x < rx2; x++) {
        if (x >= box.x1 && x < box.x2 && y >= box.y1 && y < box.y2) continue;
        cnt++;
        if (gray_raw[y * w + x] >= 180) bcnt++;
      }
    }
    double ring = cnt > 0 ? (double)bcnt / (double)cnt : 0.0;
    if (ring < 0.35) continue;
    double score = ring * (double)(box.x2 - box.x1);
    if (!have_best || score > best_score) {
      best_score = score;
      out = box;
      have_best = 1;
    }
  }
  if (have_best) return out;
  return find_search_box(gray_raw);
}

// Contrast-stretch an int8 image (value = gray−128) to full [0,255] range.
// Applied when the span is ≥ 24, matching the B-G pipeline's final step.
// Bilinear sample of the center crop → OUT_WIDTH×OUT_HEIGHT, matching
// PIL Image.BILINEAR used by AItraining image_preprocess.py.
static void crop_resize_bilinear(const uint8_t *rgb, int src_side,
                                 int cx1, int cy1, int cside,
                                 int8_t *image_data) {
  for (int y = 0; y < OUT_HEIGHT; y++) {
    float fy = (float)(y + 0.5) * (float)cside / (float)OUT_HEIGHT - 0.5f;
    int y0 = (int)floorf(fy);
    if (y0 < 0) y0 = 0;
    int y1 = y0 + 1;
    if (y1 >= cside) y1 = cside - 1;
    float wy = fy - (float)y0;
    for (int x = 0; x < OUT_WIDTH; x++) {
      float fx = (float)(x + 0.5) * (float)cside / (float)OUT_WIDTH - 0.5f;
      int x0 = (int)floorf(fx);
      if (x0 < 0) x0 = 0;
      int x1 = x0 + 1;
      if (x1 >= cside) x1 = cside - 1;
      float wx = fx - (float)x0;

      int i00 = ((cy1 + y0) * src_side + (cx1 + x0)) * 3;
      int i10 = ((cy1 + y0) * src_side + (cx1 + x1)) * 3;
      int i01 = ((cy1 + y1) * src_side + (cx1 + x0)) * 3;
      int i11 = ((cy1 + y1) * src_side + (cx1 + x1)) * 3;

      float r = (1.0f - wx) * (1.0f - wy) * (float)rgb[i00 + 0] + wx * (1.0f - wy) * (float)rgb[i10 + 0]
              + (1.0f - wx) * wy * (float)rgb[i01 + 0] + wx * wy * (float)rgb[i11 + 0];
      float g = (1.0f - wx) * (1.0f - wy) * (float)rgb[i00 + 1] + wx * (1.0f - wy) * (float)rgb[i10 + 1]
              + (1.0f - wx) * wy * (float)rgb[i01 + 1] + wx * wy * (float)rgb[i11 + 1];
      float b = (1.0f - wx) * (1.0f - wy) * (float)rgb[i00 + 2] + wx * (1.0f - wy) * (float)rgb[i10 + 2]
              + (1.0f - wx) * wy * (float)rgb[i01 + 2] + wx * wy * (float)rgb[i11 + 2];

      uint8_t lum = (uint8_t)((r * 30.0f + g * 59.0f + b * 11.0f) / 100.0f + 0.5f);
      image_data[y * OUT_WIDTH + x] = (int8_t)((int)lum - 128);
    }
  }
}

static void contrast_stretch_int8(int8_t *image_data) {
  uint8_t min_val = 255, max_val = 0;
  for (int i = 0; i < OUT_WIDTH * OUT_HEIGHT; i++) {
    uint8_t v = (uint8_t)((int)image_data[i] + 128);
    if (v < min_val) min_val = v;
    if (v > max_val) max_val = v;
  }
  int span = (int)max_val - (int)min_val;
  if (span >= 24) {
    for (int i = 0; i < OUT_WIDTH * OUT_HEIGHT; i++) {
      uint8_t v = (uint8_t)((int)image_data[i] + 128);
      int stretched = (((int)v - (int)min_val) * 255 + span / 2) / span;  // round, like host
      image_data[i] = (int8_t)(stretched - 128);
    }
  }
}

// ── Sign-mask stats (used by OOD rejection in TFLite.ino) ──────────────
static float s_last_sign_pct = 0.0f;
static int s_last_crop_x1 = 0, s_last_crop_y1 = 0, s_last_crop_side = 0;
static float s_ood_sign_pct_min      = OOD_SIGN_PCT_MIN;
static float s_ood_sign_pct_max      = OOD_SIGN_PCT_MAX;
static float s_ood_max_prob_min      = OOD_MAX_PROB_MIN;
static float s_ood_entropy_ratio_max = OOD_ENTROPY_RATIO_MAX;

// Runtime-visible mask thresholds (Dark / Lum from Host UI).
// Defaults come from the macros; sketches may override them per-frame via
// ImageProviderSetMaskThresholds().  Clamping + dark<lum invariant applied.
static int s_mask_dark_thresh = (int)BG_MASK_DARK_THRESH;
static int s_mask_lum_thresh  = (int)BG_MASK_LUM_THRESH;

static void _clamp_mask_thresholds_inline() {
  if (s_mask_dark_thresh < 0)   s_mask_dark_thresh = 0;
  if (s_mask_dark_thresh > 255) s_mask_dark_thresh = 255;
  if (s_mask_lum_thresh  < 1)   s_mask_lum_thresh  = 1;
  if (s_mask_lum_thresh  > 255) s_mask_lum_thresh  = 255;
  if (s_mask_dark_thresh >= s_mask_lum_thresh) {
    s_mask_dark_thresh = s_mask_lum_thresh - 1;
  }
}

static void _update_sign_pct_from_rgb_raw(const uint8_t *rgb_raw, int w, int h) {
  if (!rgb_raw || w <= 0 || h <= 0) { s_last_sign_pct = 0.0f; return; }
  int dark = s_mask_dark_thresh;
  int lum  = s_mask_lum_thresh;
  _clamp_mask_thresholds_inline();
  int total = w * h;
  int cnt = 0;
  for (int i = 0; i < total; i++) {
    int g = (int)rgb_raw[i * 3 + 1];
    if (g > s_mask_dark_thresh && g < s_mask_lum_thresh) cnt++;
  }
  s_last_sign_pct = ((float)cnt * 100.0f) / (float)total;
  (void)dark; (void)lum;   // silence unused-warn when inlining was intended above
}

float ImageProviderLastSignPct() { return s_last_sign_pct; }

// Last model-input crop box (pixel coords in 96-space) — diagnostics.
void ImageProviderLastCropBox(int *x1, int *y1, int *side) {
  if (x1) *x1 = s_last_crop_x1;
  if (y1) *y1 = s_last_crop_y1;
  if (side) *side = s_last_crop_side;
}

void ImageProviderSetOodThresholds(float sign_pct_min, float sign_pct_max,
                                   float max_prob_min, float entropy_ratio_max) {
  if (sign_pct_min >= 0.0f)       s_ood_sign_pct_min = sign_pct_min;
  if (sign_pct_max >= 0.0f)       s_ood_sign_pct_max = sign_pct_max;
  if (max_prob_min >= 0.0f)       s_ood_max_prob_min = max_prob_min;
  if (entropy_ratio_max >= 0.0f)  s_ood_entropy_ratio_max = entropy_ratio_max;
}

void ImageProviderGetOodThresholds(float *sign_pct_min, float *sign_pct_max,
                                   float *max_prob_min, float *entropy_ratio_max) {
  if (sign_pct_min)      *sign_pct_min      = s_ood_sign_pct_min;
  if (sign_pct_max)      *sign_pct_max      = s_ood_sign_pct_max;
  if (max_prob_min)      *max_prob_min      = s_ood_max_prob_min;
  if (entropy_ratio_max) *entropy_ratio_max = s_ood_entropy_ratio_max;
}

void ImageProviderSetMaskThresholds(int dark_thresh, int lum_thresh) {
  s_mask_dark_thresh = dark_thresh;
  s_mask_lum_thresh  = lum_thresh;
  _clamp_mask_thresholds_inline();
}

void ImageProviderGetMaskThresholds(int *dark_thresh, int *lum_thresh) {
  _clamp_mask_thresholds_inline();
  if (dark_thresh) *dark_thresh = s_mask_dark_thresh;
  if (lum_thresh)  *lum_thresh  = s_mask_lum_thresh;
}

// Public function to get the image
TfLiteStatus GetImage(tflite::ErrorReporter* error_reporter, int image_width, int image_height, int channels, int8_t* image_data) {
  if (image_width != OUT_WIDTH || image_height != OUT_HEIGHT || channels != 1) {
    TF_LITE_REPORT_ERROR(error_reporter, "GetImage expects %dx%dx1, got %dx%dx%d", OUT_WIDTH, OUT_HEIGHT, image_width, image_height, channels);
    return kTfLiteError;
  }

#if TFLITE_P4_IMX219_HAS_ARDUINO_IMX219_LIB
  if (!ImageProviderEnsureCamera()) {
    TF_LITE_REPORT_ERROR(error_reporter, "CameraBegin failed");
    return kTfLiteError;
  }

  bool updated = false;
  for (int tries = 0; tries < 100; tries++) {
    if (CameraUpdate()) {
      updated = true;
      break;
    }
    usleep(2000);
  }
  if (!updated) {
    TF_LITE_REPORT_ERROR(error_reporter, "CameraUpdate timeout");
    return kTfLiteError;
  }

  const uint8_t *rgb_lib = CameraGetRgb();  // raw sensor, no WB
  const int src_side = CameraGetRgbWidth();  // ORIGINAL capture side (96 or 160...)
  // OV5647 lib outputs at its own IMG_SIZE (160×160); everything below
  // expects IMG_SIZE×IMG_SIZE×3.  Nearest-neighbour resize (identity for
  // IMX219).
  //
  // WB gains 100/100 = passthrough.  CRITICAL for train/device parity:
  // AItraining captures the IMX219_Grayscale_Serial stream, whose pixels
  // are BT.601 luminance of the RAW demosaiced RGB with NO white balance
  // (library rgb_to_gray, weights 30/59/11).  The model was therefore
  // trained on raw (green-tinted) luminance.  The ×2.0 WB exists only in
  // the RGB data-collection path used by other projects — it must NOT be
  // applied to the model input of a grayscale-trained model.
  static uint8_t rgb_resized[OUT_WIDTH * OUT_HEIGHT * 3];
  resize_rgb_wb(rgb_lib, src_side, rgb_resized, 100, 100);
  const uint8_t *rgb_raw = rgb_resized;

  // ── Crop selection (model-input window) ─────────────────────────────
  // BG_ENABLE_FOCUS_SEARCH=1 (default): auto shadow-search box — the host
  // _focus_bbox C port above (bit-identical boxes) — removes background
  // from the model input.  Search failure falls back to the same 40 %-side
  // centered box the host uses.  =0: legacy center 60 % crop
  // (BG_FALLBACK_CENTER_FRAC).  MUST match the crop_mode the model was
  // trained with on the host (retrain after switching!).
  int crop_x1, crop_y1, crop_side;
#if BG_ENABLE_FOCUS_SEARCH
  {
    static uint8_t search_gray[OUT_WIDTH * OUT_HEIGHT];
    // The search runs on the RAW G channel with exposure-adaptive band
    // selection (mirrors host _focus_bbox_adaptive) — a fixed dark/lum mask
    // cannot track the sign when auto-exposure shifts the ink's gray with
    // framing.  The mask thresholds still drive the sign_pct OOD stat only.
    for (int i = 0; i < OUT_WIDTH * OUT_HEIGHT; i++) {
      search_gray[i] = rgb_resized[i * 3 + 1];
    }
    fb_box_t box = find_search_box_adaptive(search_gray);
    crop_x1 = box.x1;
    crop_y1 = box.y1;
    int sx = box.x2 - box.x1;
    int sy = box.y2 - box.y1;
    crop_side = (sx > sy) ? sx : sy;
    if (crop_side < 2) crop_side = 2;
    if (crop_side > OUT_WIDTH) crop_side = OUT_WIDTH;
    if (crop_x1 < 0) crop_x1 = 0;
    if (crop_y1 < 0) crop_y1 = 0;
    if (crop_x1 + crop_side > OUT_WIDTH) crop_x1 = OUT_WIDTH - crop_side;
    if (crop_y1 + crop_side > OUT_HEIGHT) crop_y1 = OUT_HEIGHT - crop_side;
  }
#else
  // Center crop matching host fast_mode _center_bbox (Python
  // image_preprocess.py: side = int(min(h,w)*frac), cx = w//2, half = side//2,
  // left = max(0, cx - half)).  Computed in 96×96 pipeline space: the library
  // output is already 96×96 for IMX219, and OV5647 is resized to 96×96 above,
  // so the 96-space box covers the same 60 % FOV the host crops.
  {
    const float kCropFrac = BG_FALLBACK_CENTER_FRAC;  // 0.60 default
    int center_side = (int)((float)OUT_WIDTH * kCropFrac);   // floor, like Python int()
    if (center_side < 2) center_side = 2;
    if (center_side > OUT_WIDTH) center_side = OUT_WIDTH;
    crop_x1 = OUT_WIDTH / 2 - center_side / 2;   // cx - half, like Python
    crop_y1 = OUT_HEIGHT / 2 - center_side / 2;
    crop_side = center_side;
  }
#endif

#if PREPROCESS_MODE == PREPROCESS_MODE_GRAY
  // ── Grayscale mode: crop (search box or center, per BG_ENABLE_FOCUS_SEARCH)
  // → bilinear → BT.601 luminance of the RAW frame (no WB — matches the
  // grayscale serial stream AItraining trains on).  No B-G / no blob crop. ──
  s_last_crop_x1 = crop_x1; s_last_crop_y1 = crop_y1; s_last_crop_side = crop_side;
  _update_sign_pct_from_rgb_raw(rgb_raw, OUT_WIDTH, OUT_HEIGHT);
  crop_resize_bilinear(rgb_resized, OUT_WIDTH, crop_x1, crop_y1, crop_side, image_data);
  contrast_stretch_int8(image_data);
  return kTfLiteOk;

#elif PREPROCESS_MODE == PREPROCESS_MODE_BG
  _update_sign_pct_from_rgb_raw(rgb_raw, OUT_WIDTH, OUT_HEIGHT);

  // ── B-G difference pipeline ──
  // Blue / purple signs → high B, low G → B-G is large positive.
  // Green objects / foliage → low B, high G → B-G is negative.
  // Grey background (road / wall / sky) → B ≈ G → B-G ≈ 0.
  // Mapping:  diff ∈ [-255, 255]  →  gray = (diff + 255) / 2  →  [0, 255].
  //
  // ROI detection: purple mask (min(R,B) > G + margin) → morphology →
  // largest blob → crop.  Much more specific than a bare B-G threshold.
  //
  // NOTE (END / NO ENTRY / RIGHT black-on-white signs): These signs have
  // NO purple pixels, so the B-G blob detector above will always fire
  // `found_roi = false`.  Historically we fell back to the FULL frame,
  // which was a huge mismatch vs AItraining (cache rebuild always uses a
  // central 60 % crop).  The fall-back is now a central crop of
  // BG_FALLBACK_CENTER_FRAC (default 0.60), matching host fast_mode.
  // For these monochrome-ish datasets you can also set
  // BG_ENABLE_BLOB_SEARCH = 0 to skip the (always failing) B-G search
  // entirely and go straight to the central crop — same behaviour, cheaper.

  // ---- Step 0 (optional): Dynamic Auto White Balance (Gray World) ----
  // Gray-World gains are used ONLY inside the B-G / blob search below.
  // The final pixels always come from the original rgb_raw — the comment
  // in Step 4 explains why.  So disabling AWB only changes WHERE the
  // crop window lands for true purple signs; it has no effect on the
  // END/NO ENTRY/RIGHT fallback path.

  static int16_t bg_raw[OUT_WIDTH * OUT_HEIGHT];   // signed B-G values
  static uint8_t bg_u8[OUT_WIDTH * OUT_HEIGHT];

  uint16_t wb_r = 100, wb_b = 100;  // default: ×1.0 (identity, no AWB)
#if BG_ENABLE_AWB
  {
    // Pass 1: accumulate channel sums for AWB (skip pure black)
    int64_t sum_r = 0, sum_g = 0, sum_b = 0;
    int awb_n = 0;
    for (int i = 0; i < OUT_WIDTH * OUT_HEIGHT; i++) {
      int rr = rgb_raw[i * 3 + 0];
      int gg = rgb_raw[i * 3 + 1];
      int bb = rgb_raw[i * 3 + 2];
      if (rr < 10 && gg < 10 && bb < 10) continue;  // skip pure black
      sum_r += rr;  sum_g += gg;  sum_b += bb;
      awb_n++;
    }

    // Compute Gray World gains: scale R and B to match G average
    if (awb_n > 100) {
      int avg_r = (int)(sum_r / awb_n);
      int avg_g = (int)(sum_g / awb_n);
      int avg_b = (int)(sum_b / awb_n);
      if (avg_r > 0 && avg_g > 0 && avg_b > 0) {
        int gr = (avg_g * 100) / avg_r;  // gain ×100
        int gb = (avg_g * 100) / avg_b;
        if (gr <  50) gr =  50;  if (gr > 400) gr = 400;  // clamp [0.5, 4.0]
        if (gb <  50) gb =  50;  if (gb > 400) gb = 400;
        wb_r = (uint16_t)gr;
        wb_b = (uint16_t)gb;
      }
    }
  }
#endif  // BG_ENABLE_AWB

  // ---- Step 1-3: B-G extraction + mask + morphology + largest blob ----
  // (crop_x1/crop_y1 come from the crop selection above; only the blob
  // search may overwrite them, so crop_w/crop_h init as the full frame.)
  int crop_w = OUT_WIDTH, crop_h = OUT_HEIGHT;
  bool found_roi = false;

#if BG_ENABLE_BLOB_SEARCH
  {
    // ---- Step 1: B-G extraction with (optional) AWB applied ----
    for (int i = 0; i < OUT_WIDTH * OUT_HEIGHT; i++) {
      int r_raw = rgb_raw[i * 3 + 0];
      int g_raw = rgb_raw[i * 3 + 1];
      int b_raw = rgb_raw[i * 3 + 2];

      // Apply AWB gains (identity 100/100 when BG_ENABLE_AWB == 0)
      int r = (r_raw * (int)wb_r) / 100;
      int b = (b_raw * (int)wb_b) / 100;
      int g = g_raw;  // green is reference
      if (r > 255) r = 255;
      if (b > 255) b = 255;

      // Pure-black areas (shadows) can't be a sign.
      if (r < 10 && g < 10 && b < 10) {
        bg_raw[i] = 0;  bg_u8[i] = 128;  continue;
      }
      int diff = b - g;
      bg_raw[i] = (int16_t)diff;
      bg_u8[i]  = (uint8_t)((diff + 255) / 2);
    }

    // ---- Step 2: 5×5 box blur → contrast stretch → binary mask ----
    static int16_t bg_blur[OUT_WIDTH * OUT_HEIGHT];
    int16_t bg_min = 32767, bg_max = -32768;
    for (int y = 2; y < OUT_HEIGHT - 2; y++) {
      for (int x = 2; x < OUT_WIDTH - 2; x++) {
        int sum = 0;
        for (int dy = -2; dy <= 2; dy++)
          for (int dx = -2; dx <= 2; dx++)
            sum += bg_raw[(y + dy) * OUT_WIDTH + (x + dx)];
        int16_t v = (int16_t)(sum / 25);
        int idx = y * OUT_WIDTH + x;
        bg_blur[idx] = v;
        if (v < bg_min) bg_min = v;
        if (v > bg_max) bg_max = v;
      }
    }

    // Contrast stretch blurred B-G to [0,255]
    int bg_span = (int)bg_max - (int)bg_min;
    static uint8_t mask[OUT_WIDTH * OUT_HEIGHT];
    bool any_signal = (bg_span >= 20);   // need meaningful contrast
    if (any_signal) {
      for (int y = 2; y < OUT_HEIGHT - 2; y++) {
        for (int x = 2; x < OUT_WIDTH - 2; x++) {
          int idx = y * OUT_WIDTH + x;
          int stretched = ((int)bg_blur[idx] - (int)bg_min) * 255 / bg_span;
          // Sign is BRIGHTER than background (higher B-G after WB) → above midpoint
          mask[idx] = (stretched > 80) ? 255 : 0;    // lower = more sensitive
        }
      }
    }

    // ---- Step 2b: morphological clean-up (if any purple pixels) ----
    if (any_signal) {
      // 2a. 3×3 erosion (remove isolated noise)
      static uint8_t tmp[OUT_WIDTH * OUT_HEIGHT];
      for (int y = 1; y < OUT_HEIGHT - 1; y++) {
        for (int x = 1; x < OUT_WIDTH - 1; x++) {
          int idx = y * OUT_WIDTH + x;
          tmp[idx] = (mask[idx] && mask[idx - 1] && mask[idx + 1]
                   && mask[idx - OUT_WIDTH] && mask[idx + OUT_WIDTH]) ? 255 : 0;
        }
      }
      // 2b. 3×3 dilation ×2 (reconnect fragments)
      for (int pass = 0; pass < 2; pass++) {
        for (int y = 1; y < OUT_HEIGHT - 1; y++) {
          for (int x = 1; x < OUT_WIDTH - 1; x++) {
            int idx = y * OUT_WIDTH + x;
            mask[idx] = tmp[idx];  // copy back
            tmp[idx] = (mask[idx] || mask[idx - 1] || mask[idx + 1]
                     || mask[idx - OUT_WIDTH] || mask[idx + OUT_WIDTH]) ? 255 : 0;
          }
        }
        // swap
        for (int i = 0; i < OUT_WIDTH * OUT_HEIGHT; i++) {
          uint8_t s = mask[i]; mask[i] = tmp[i]; tmp[i] = s;
        }
      }

      // ---- Step 3: find largest connected component (row-by-row run-length) ----
      // We use a simple two-pass approach: first find all runs, then merge
      // overlapping runs across rows using a union-find label table.
      #define MAX_RUNS 512
      struct { int x1, x2, y, label; } runs[MAX_RUNS];
      int num_runs = 0;

      // First pass: extract runs
      for (int y = 1; y < OUT_HEIGHT - 1 && num_runs < MAX_RUNS; y++) {
        int x = 1;
        while (x < OUT_WIDTH - 1) {
          while (x < OUT_WIDTH - 1 && !mask[y * OUT_WIDTH + x]) x++;
          if (x >= OUT_WIDTH - 1) break;
          int x1 = x;
          while (x < OUT_WIDTH - 1 && mask[y * OUT_WIDTH + x]) x++;
          runs[num_runs].x1 = x1;
          runs[num_runs].x2 = x - 1;
          runs[num_runs].y  = y;
          runs[num_runs].label = num_runs;  // self-label initially
          num_runs++;
        }
      }

      if (num_runs > 1) {
        // Merge overlapping runs on adjacent rows
        int parent[MAX_RUNS];
        for (int i = 0; i < num_runs; i++) parent[i] = i;
        auto find = [&](int a) { while (parent[a] != a) a = parent[a]; return a; };
        auto unite = [&](int a, int b) { parent[find(a)] = find(b); };

        for (int i = 0; i < num_runs; i++) {
          for (int j = i + 1; j < num_runs; j++) {
            if (runs[j].y > runs[i].y + 1) break;  // not adjacent row
            if (runs[j].y == runs[i].y + 1
                && runs[j].x1 <= runs[i].x2 && runs[j].x2 >= runs[i].x1)
              unite(i, j);
          }
        }

        // Count component sizes
        int comp_size[MAX_RUNS] = {0};
        for (int i = 0; i < num_runs; i++) {
          int root = find(i);
          comp_size[root] += runs[i].x2 - runs[i].x1 + 1;
        }

        // Find largest component
        int best_root = 0, best_size = 0;
        for (int i = 0; i < num_runs; i++) {
          int root = find(i);
          if (comp_size[root] > best_size) {
            best_size = comp_size[root];
            best_root = root;
          }
        }

        if (best_size >= 16) {
          // Compute bbox of the largest component
          int min_x = OUT_WIDTH, min_y = OUT_HEIGHT, max_x = 0, max_y = 0;
          for (int i = 0; i < num_runs; i++) {
            if (find(i) == best_root) {
              if (runs[i].x1 < min_x) min_x = runs[i].x1;
              if (runs[i].x2 > max_x) max_x = runs[i].x2;
              if (runs[i].y  < min_y) min_y = runs[i].y;
              if (runs[i].y  > max_y) max_y = runs[i].y;
            }
          }
          int bw = max_x - min_x + 1, bh = max_y - min_y + 1;
          int side = (bw > bh) ? bw : bh;
          int pad = side / 5; if (pad < 1) pad = 1;
          side += pad * 2;
          int ccx = (min_x + max_x) / 2, ccy = (min_y + max_y) / 2;
          int half = side / 2;
          crop_x1 = ccx - half; if (crop_x1 < 0) crop_x1 = 0;
          crop_y1 = ccy - half; if (crop_y1 < 0) crop_y1 = 0;
          crop_w  = side;       if (crop_x1 + crop_w > OUT_WIDTH)  crop_w = OUT_WIDTH  - crop_x1;
          crop_h  = side;       if (crop_y1 + crop_h > OUT_HEIGHT) crop_h = OUT_HEIGHT - crop_y1;
          if (crop_w >= 8 && crop_h >= 8) found_roi = true;
        }
      }
    }
  }
#else   // BG_ENABLE_BLOB_SEARCH == 0 → match host fast_mode exactly
  // Skip the entire B-G/morphology/blob pipeline and go straight to the
  // central BG_FALLBACK_CENTER_FRAC crop.  Same end result for non-purple
  // signs, minus the wasted cycles of a search that always fails.
  (void)bg_raw; (void)bg_u8; (void)wb_r; (void)wb_b;
#endif  // BG_ENABLE_BLOB_SEARCH

  // ---- Fallback: precomputed crop (search box / center 60 %) when the
  // B-G blob search was disabled or failed ----
  if (!found_roi) {
    crop_w  = crop_side; if (crop_x1 + crop_w > OUT_WIDTH)  crop_w = OUT_WIDTH  - crop_x1;
    crop_h  = crop_side; if (crop_y1 + crop_h > OUT_HEIGHT) crop_h = OUT_HEIGHT - crop_y1;
  }

  // ---- Step 4/5: crop raw (no WB) RGB + BT.601 luminance ----
  // ROI detection used B-G for blob search, but the final pixels come from
  // the RAW unmodified source — identical to the pixels AItraining
  // receives over the grayscale serial stream (library BT.601 of raw
  // demosaiced RGB, no white balance).  No per-frame gray-world AWB:
  // training frames carry no WB at all.
  {
    // If blob detection found a purple sign, use its box; otherwise the
    // center fallback box.  The box lives in 96-space and rgb_resized is
    // already 96×96, so no mapping is needed.
    int cw = crop_w;  if (cw  > OUT_WIDTH  - crop_x1) cw  = OUT_WIDTH  - crop_x1;
    int chh = crop_h; if (chh > OUT_HEIGHT - crop_y1) chh = OUT_HEIGHT - crop_y1;
    if (cw < 2) cw = 2;
    if (chh < 2) chh = 2;
    s_last_crop_x1 = crop_x1; s_last_crop_y1 = crop_y1;
    s_last_crop_side = (cw > chh) ? cw : chh;
    crop_resize_bilinear(rgb_resized, OUT_WIDTH, crop_x1, crop_y1, cw > chh ? cw : chh, image_data);
  }

  // ---- Step 6: contrast stretch ----
  contrast_stretch_int8(image_data);

#else  // PREPROCESS_MODE == RGB: handled in the sketch (streaming), not here
  TF_LITE_REPORT_ERROR(error_reporter, "GetImage not used in RGB mode");
  return kTfLiteError;
#endif  // PREPROCESS_MODE

  return kTfLiteOk;
#elif TFLITE_P4_IMX219_HAS_ESP_VIDEO
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

  // B-G difference pipeline — same as main backend.
  // (Simplified: no blob detection; V4L2 backend is for debug / fallback.)
  for (int i = 0; i < OUT_WIDTH * OUT_HEIGHT; i++) {
    int r = s_rgb_buf[i * 3 + 0];
    int g = s_rgb_buf[i * 3 + 1];
    int b = s_rgb_buf[i * 3 + 2];
    r = (int)((float)r * 2.0f); if (r > 255) r = 255;  // WB
    b = (int)((float)b * 2.0f); if (b > 255) b = 255;
    int diff = b - g;
    uint8_t gray = (uint8_t)((diff + 255) / 2);
    image_data[i] = (int8_t)(gray - 128);
  }
  // Contrast stretch
  {
    uint8_t min_val = 255, max_val = 0;
    for (int i = 0; i < OUT_WIDTH * OUT_HEIGHT; i++) {
      uint8_t v = (uint8_t)((int)image_data[i] + 128);
      if (v < min_val) min_val = v;
      if (v > max_val) max_val = v;
    }
    int span = (int)max_val - (int)min_val;
    if (span >= 24) {
      for (int i = 0; i < OUT_WIDTH * OUT_HEIGHT; i++) {
        uint8_t v = (uint8_t)((int)image_data[i] + 128);
        int stretched = ((int)v - (int)min_val) * 255 / span;
        image_data[i] = (int8_t)(stretched - 128);
      }
    }
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
