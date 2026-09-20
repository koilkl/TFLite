// ═══════════════════════════════════════════════════════════════════════
// USER CONFIG — edit these two values only
//
//   Why edit here:
//     Every Arduino .cpp compiles as its own translation unit, so a
//     #define at the top of TFLite.ino cannot reach image_provider.cpp.
//     Both files include this shared header — edit once, applies
//     everywhere.
//
//   IMG_SIZE
//     Pipeline resolution: drives the MODEL INPUT and the capture/serial
//     streams together.
//     - Export the model from AItraining with the SAME img_size;
//     - Reflash this firmware and regenerate tm_model_data.cpp;
//     - Set the same value in AItraining device settings -> Image Size.
//
//   CAMERA_FLIP_180
//     The camera is mounted upside down on the board; the library rotates
//     the image 180° to match the data-collection (training) orientation.
//     Set to false only if your mount differs.
// ═══════════════════════════════════════════════════════════════════════
#define IMG_SIZE          96
#define CAMERA_FLIP_180   true
