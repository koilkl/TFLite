#ifndef TFLITE_IMAGE_PROVIDER_H_
#define TFLITE_IMAGE_PROVIDER_H_

#undef ESP32_P4_IMX219_XCLK_GPIO
#define ESP32_P4_IMX219_XCLK_GPIO 45
#ifndef IMX219_BOARD_XCLK_GPIO
#define IMX219_BOARD_XCLK_GPIO 45
#endif

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
#define CAMERA_TYPE 2
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

// ── BG-mode tuning knobs ──────────────────────────────────────────────
// BG_FALLBACK_CENTER_FRAC: When B-G blob detection finds no sign, the
// pipeline used to fall back to the FULL uncropped frame.  That caused
// a huge train/interpret mismatch because AItraining always uses a
// central 60 % crop (see image_preprocess.py _center_bbox frac=0.60)
// for fast-mode / cache-rebuild batches (END / NO ENTRY / RIGHT signs
// are never purple, so the B-G blob detector always misses them).
// Default 0.60 matches host exactly; set to 1.0 to restore old behavior.
#ifndef BG_FALLBACK_CENTER_FRAC
#define BG_FALLBACK_CENTER_FRAC 0.60f
#endif

// BG_ENABLE_AWB: By default the BG pipeline applies per-frame Gray-World
// auto white balance before computing B-G differences.  That matches real
// purple/dyed signs under varying lighting, but AItraining preprocessing
// runs NO AWB — for signs that are pure black/white/red (END, NO ENTRY,
// RIGHT …) disabling AWB here aligns device-side pixels with training.
// Change to 0 on monochrome-ish sign datasets to reduce train/device drift.
#ifndef BG_ENABLE_AWB
#define BG_ENABLE_AWB 1
#endif

// BG_ENABLE_BLOB_SEARCH: When 0, skip the B-G mask + morphology + blob
// step entirely and ALWAYS use the central BG_FALLBACK_CENTER_FRAC crop.
// Use this for datasets where every sign is black-on-white and the
// B-G blob detector never fires (matches host fast_mode exactly).
#ifndef BG_ENABLE_BLOB_SEARCH
#define BG_ENABLE_BLOB_SEARCH 0   // monochrome signs: skip B-G blob (false
                                  // detections crop the wrong region and make
                                  // the input dark) — always center 57×57 crop,
                                  // matching host fast_mode exactly
#endif

// BG_ENABLE_FOCUS_SEARCH: When 1 (default), the model-input crop comes from
// the auto shadow-search box — the C port of the host _focus_bbox
// (bit-identical boxes, verified 65/65 frames) — so the background outside
// the detected sign is removed from the model input.  Search failure falls
// back to the same 40 %-side centered box the host uses.
// MUST match the host training crop_mode ("auto_search"): retrain the model
// after switching.  Set 0 for the legacy deterministic center 60 % crop
// (host crop_mode "center").
#ifndef BG_ENABLE_FOCUS_SEARCH
#define BG_ENABLE_FOCUS_SEARCH 1
#endif

// ── Monochrome-sign (END/NO ENTRY/RIGHT) mask stats + OOD  ─────────────
// We compute a G-channel "sign mask" exactly like AItraining does
// (sign_pct = pixels where dark_thresh < G < lum_thresh, as a percentage of
// the full frame).  We expose the raw percentage to the sketch via
// ImageProviderLastSignPct() so the downstream classifier can reject
// "nothing in frame" scenes, defeating softmax's always-high-confidence
// behaviour.  In the device's default configuration (center 60 % crop,
// BG_ENABLE_BLOB_SEARCH=0) these thresholds do NOT touch the model-input
// pixels — they only drive the sign_pct OOD gate — so they must match the
// host live-predict defaults (AItraining prepare_inference_inputs:
// bg_dark_thresh=0, bg_lum_thresh=100) for the device to reject exactly the
// same frames the host rejects.  Override per-project with -D compiler
// flags (e.g. upper-project aggregate thresholds).
#ifndef BG_MASK_DARK_THRESH
#define BG_MASK_DARK_THRESH  0
#endif
#ifndef BG_MASK_LUM_THRESH
#define BG_MASK_LUM_THRESH   100
#endif

// Inference-time OOD tuning.  If any rule fires, the sketch reports the
// "No Sign" synthetic class (label_id == kCategoryCount, confidence = 0)
// instead of trusting a softmax hallucination.  Defaults match the host
// live-predict gates (record_controller _preview_predict): sign_pct in
// [0.3, 70] %, max_prob >= 0.60, entropy_ratio <= 0.70.  Disable any of
// these gates by setting to zero.
//   SIGN_PCT_MIN / SIGN_PCT_MAX: reject frames where the G-channel mask
//       outside [min,max] % (e.g. empty road <0.3 %, full shadow >70 %).
//   MAX_PROB_MIN:       reject softmax top-1 below this fraction.
//   ENTROPY_RATIO_MAX:  reject "spread out" votes: -sum(p·log p) / log(N)
//                       above this threshold.  1.0 = perfectly uniform.
#ifndef OOD_SIGN_PCT_MIN
#define OOD_SIGN_PCT_MIN       0.3f
#endif
#ifndef OOD_SIGN_PCT_MAX
#define OOD_SIGN_PCT_MAX       70.0f
#endif
#ifndef OOD_MAX_PROB_MIN
#define OOD_MAX_PROB_MIN       0.60f
#endif
#ifndef OOD_ENTROPY_RATIO_MAX
#define OOD_ENTROPY_RATIO_MAX  0.70f
#endif
#ifndef OOD_ENABLE            // master switch
#define OOD_ENABLE            1
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

// Begin-once guard: initialises the camera on the first call and returns the
// cached result afterwards.  The library begin() is NOT re-entrant — calling
// it again on a mode switch can hang the I2C bus.  Use this everywhere
// (GetImage, TFLite.ino capture modes) instead of calling CameraBegin().
bool ImageProviderEnsureCamera();

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

// Latest frame resized to IMG_SIZE×IMG_SIZE×3 (nearest-neighbour, WB
// passthrough).  Use in capture modes — the library buffer is 160×160 for
// OV5647 and streaming its first bytes row-major tears the frame.
const uint8_t* CameraGetRgbImgSized();
const uint8_t* CameraGetGrayImgSized();

// ── TFLite interface ──────────────────────────────────────────────────
// Returns an IMG_SIZE×IMG_SIZE grayscale image.
// image_data must be pre-allocated: image_width * image_height * channels.
TfLiteStatus GetImage(tflite::ErrorReporter* error_reporter, int image_width,
                      int image_height, int channels, int8_t* image_data);

// Function to clean up resources allocated by the image provider.
void ImageProviderDeinit();

// After GetImage() returns kTfLiteOk, returns the "sign_pct" fraction of
// full-frame pixels whose G channel falls in (BG_MASK_DARK_THRESH,
// BG_MASK_LUM_THRESH).  0.0 = no pixel looks like sign-black, 100.0 = every
// pixel looks like sign-black.  The sketch uses this to short-circuit
// out-of-distribution frames even before running inference (see OOD_*).
// Returns 0.0 if called before the first GetImage.
float ImageProviderLastSignPct();

// Last model-input crop box (pixel coords in 96-space): x1, y1, side.
// Diagnostics only — the debug serial can print it for host-vs-device
// search-box comparisons.
void ImageProviderLastCropBox(int *x1, int *y1, int *side);

// Optional: override the per-frame OOD thresholds in-process (e.g. from a
// saved setting).  Defaults come from the OOD_* macros above.  Passing
// negative values preserves the current setting.
void ImageProviderSetOodThresholds(float sign_pct_min, float sign_pct_max,
                                   float max_prob_min, float entropy_ratio_max);

// Read the *current* live OOD thresholds (useful when sketch code wants to
// re-use them without re-including the macro defaults).
void ImageProviderGetOodThresholds(float *sign_pct_min, float *sign_pct_max,
                                   float *max_prob_min, float *entropy_ratio_max);

// ── Dark / Lum (Host UI Dark/Lum sliders, G-channel sign mask) ─────────
// Sign = G in (dark_thresh, lum_thresh).  Mirrors the Host UI bg_dark_thresh
// / bg_lum_thresh.  Defaults come from BG_MASK_DARK/LUM_THRESH macros but
// can be overridden at runtime (e.g. via Debug Serial commands).  Passing
// values outside [0..255] is clamped internally, and dark >= lum is
// corrected so dark = lum-1.
void ImageProviderSetMaskThresholds(int dark_thresh, int lum_thresh);
void ImageProviderGetMaskThresholds(int *dark_thresh, int *lum_thresh);

#endif  // TFLITE_IMAGE_PROVIDER_H_
