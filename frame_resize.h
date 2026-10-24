#ifndef TFLITE_FRAME_RESIZE_H_
#define TFLITE_FRAME_RESIZE_H_

#include <stddef.h>
#include <stdint.h>

// ── Capture -> preprocessing resolution (exact box average) ─────────────
// The camera captures at g_frame_side (exported as kCaptureFrameSide, e.g.
// 160) while the preprocessing front-end works at a smaller frame side
// (kPreprocessFrameSide, e.g. 96 for the glyph sketches or img_size for the
// legacy image pipeline).  The P4 and AItraining's live serial preview MUST
// resample identically or the device sees a different image than the host.
//
// Algorithm (bit-identical to AItraining record_controller
// ._resize_square_box_average_u8):
//   output pixel o covers source [o*src_side/dst_side,
//                                 (o+1)*src_side/dst_side)
//   (clamped to at least one source pixel); every source pixel in that box
//   contributes exactly once, and the sum is divided with round-half-up:
//       (sum + count/2) / count
// No floats, no SIMD and no interpolation weights, so the C and NumPy
// implementations agree on every pixel.  Works for down- and up-scaling;
// an identity resize is a plain copy.
//
// dst holds dst_side*dst_side bytes (GRAY8) or dst_side*dst_side*3 bytes
// (RGB24).  src and dst must not overlap.
static inline void FrameResizeBoxAverageGray(const uint8_t *src, int src_side,
                                             uint8_t *dst, int dst_side) {
  if (src == NULL || dst == NULL || src_side <= 0 || dst_side <= 0) return;
  for (int oy = 0; oy < dst_side; oy++) {
    int sy0 = (int)((int64_t)oy * src_side / dst_side);
    int sy1 = (int)((int64_t)(oy + 1) * src_side / dst_side);
    if (sy1 <= sy0) sy1 = sy0 + 1;
    if (sy1 > src_side) sy1 = src_side;
    for (int ox = 0; ox < dst_side; ox++) {
      int sx0 = (int)((int64_t)ox * src_side / dst_side);
      int sx1 = (int)((int64_t)(ox + 1) * src_side / dst_side);
      if (sx1 <= sx0) sx1 = sx0 + 1;
      if (sx1 > src_side) sx1 = src_side;
      uint32_t sum = 0;
      for (int sy = sy0; sy < sy1; sy++) {
        const uint8_t *row = src + (size_t)sy * (size_t)src_side;
        for (int sx = sx0; sx < sx1; sx++) sum += row[sx];
      }
      const uint32_t count = (uint32_t)(sy1 - sy0) * (uint32_t)(sx1 - sx0);
      dst[(size_t)oy * (size_t)dst_side + (size_t)ox] =
          (uint8_t)((sum + count / 2) / count);
    }
  }
}

// RGB24 variant used by the camera pipeline's model-input resize.  The two
// WB gains (x100, 100 = passthrough) are applied to each SOURCE pixel before
// the box sum, exactly like the old nearest-neighbour resize_rgb_wb(); with
// 100/100 the per-channel arithmetic collapses to FrameResizeBoxAverageGray
// run once per channel.
static inline void FrameResizeBoxAverageRgb(const uint8_t *src, int src_side,
                                            uint8_t *dst, int dst_side,
                                            uint16_t wb_red, uint16_t wb_blue) {
  if (src == NULL || dst == NULL || src_side <= 0 || dst_side <= 0) return;
  for (int oy = 0; oy < dst_side; oy++) {
    int sy0 = (int)((int64_t)oy * src_side / dst_side);
    int sy1 = (int)((int64_t)(oy + 1) * src_side / dst_side);
    if (sy1 <= sy0) sy1 = sy0 + 1;
    if (sy1 > src_side) sy1 = src_side;
    for (int ox = 0; ox < dst_side; ox++) {
      int sx0 = (int)((int64_t)ox * src_side / dst_side);
      int sx1 = (int)((int64_t)(ox + 1) * src_side / dst_side);
      if (sx1 <= sx0) sx1 = sx0 + 1;
      if (sx1 > src_side) sx1 = src_side;
      uint32_t sum_r = 0, sum_g = 0, sum_b = 0;
      for (int sy = sy0; sy < sy1; sy++) {
        const uint8_t *row = src + (size_t)sy * (size_t)src_side * 3u;
        for (int sx = sx0; sx < sx1; sx++) {
          const uint8_t *px = row + (size_t)sx * 3u;
          int r = (int)px[0] * (int)wb_red / 100;
          int b = (int)px[2] * (int)wb_blue / 100;
          if (r > 255) r = 255;
          if (b > 255) b = 255;
          sum_r += (uint32_t)r;
          sum_g += (uint32_t)px[1];
          sum_b += (uint32_t)b;
        }
      }
      const uint32_t count = (uint32_t)(sy1 - sy0) * (uint32_t)(sx1 - sx0);
      uint8_t *out = dst + ((size_t)oy * (size_t)dst_side + (size_t)ox) * 3u;
      out[0] = (uint8_t)((sum_r + count / 2) / count);
      out[1] = (uint8_t)((sum_g + count / 2) / count);
      out[2] = (uint8_t)((sum_b + count / 2) / count);
    }
  }
}

#endif  // TFLITE_FRAME_RESIZE_H_
