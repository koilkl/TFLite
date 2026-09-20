// ═══════════════════════════════════════════════════════════════════════
// USER CONFIG — 用户只改这两个值 (edit these two values only)
//
//   为什么在这里改:
//     Arduino 的每个 .cpp 是独立编译单元,TFLite.ino 顶部写 #define 无法
//     传给 image_provider.cpp。所以两个值统一放在这个共享头文件里,
//     TFLite.ino 和 image_provider 都包含它 — 改这里一处,全局生效。
//
//   IMG_SIZE
//     分辨率:同时驱动「模型输入」和「采集/串口流」。
//     - 模型必须在 AItraining 里用相同的 img_size 训练并导出;
//     - 重新烧录本固件,并用新模型重新生成 tm_model_data.cpp;
//     - AItraining 设备源设置里的 Image Size 也要设成同一个值。
//
//   CAMERA_FLIP_180
//     摄像头在板上是倒装的,库会把画面 180° 旋转对齐
//     数据采集(训练)方向。安装方向不同才改成 false。
// ═══════════════════════════════════════════════════════════════════════
#define IMG_SIZE          96
#define CAMERA_FLIP_180   true
