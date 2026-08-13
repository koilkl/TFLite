# TFLite

On-device sign inference for ESP32-P4 with TensorFlow Lite Micro. Captures frames from an IMX219 or OV5647 camera, runs the B-G colour-difference pipeline, classifies the sign, and sends the result to the ESP32-S3 over UART.

## Camera Selection

Edit `image_provider.h`:

```cpp
#define CAMERA_TYPE CAMERA_TYPE_IMX219   // or CAMERA_TYPE_OV5647 / CAMERA_TYPE_AUTO
```

- **IMX219**: MIPI CSI, 1536×1232 RAW10. Library builds at `IMG_SIZE` (96×96 default) via the override in `image_provider.h`.
- **OV5647**: MIPI CSI, 1920×1080 RAW10. The library stays at its own 160×160 default — `GetImage()` resizes to `IMG_SIZE` at runtime.
- **AUTO**: probes the shared I2C bus at startup (IMX219=0x10, OV5647=0x36) and picks whichever responds — no recompile needed when swapping cameras.

Both cameras feed the same pipeline, so switching cameras only requires changing `CAMERA_TYPE`.

## Preprocessing Mode

Edit `image_provider.h`:

```cpp
#define PREPROCESS_MODE PREPROCESS_MODE_BG    // or GRAY / RGB
```

| Mode | What GetImage produces | Use case |
|---|---|---|
| `PREPROCESS_MODE_BG` (default) | B-G colour difference + blob auto-crop + contrast | Sign inference (matches `AItraining/image_preprocess.py`) |
| `PREPROCESS_MODE_GRAY` | Plain BT.601 grayscale + contrast, no colour difference | Inference on models trained with plain grayscale |
| `PREPROCESS_MODE_RGB` | No inference — streams WB-corrected IMG_SIZE×IMG_SIZE×3 RGB to Serial (`0xAA 0x55 0xAA` + payload) | Data collection for AItraining, like the `IMX219_RGB_Serial` example |

## Pipeline

Camera → dynamic Gray-World AWB → B-G extraction → blur → contrast stretch → blob detection → auto-crop → resize IMG_SIZE×IMG_SIZE → contrast → TFLite int8 inference → UART to S3.

See the repo-root `CLAUDE.md` for the full 13-step pipeline. Any change to `image_provider.cpp` must be mirrored in `AItraining/image_preprocess.py`.

## UART Protocol (P4 ↔ S3, bidirectional)

Both directions share one UART at 921600 baud (P4 pins: RX=10, TX=11).

**P4 → S3** — inference packet, 9 bytes:

| Offset | Field |
|---|---|
| 0-1 | Sync `0xAA 0x55` |
| 2 | msg_type `0x01` |
| 3-4 | frame_id (uint16 LE) |
| 5 | label_id |
| 6 | confidence |
| 7 | flags (2 = sign_ready) |
| 8 | checksum (XOR of bytes 0-7) |

**S3 → P4** — control packet, 5 bytes:

| Offset | Field |
|---|---|
| 0-1 | Sync `0xAA 0x55` |
| 2 | msg_type `0x02` |
| 3 | command |
| 4 | checksum (XOR of bytes 0-3) |

Commands:
- `0x01` `ACK_STOP` — S3 confirmed the sign → P4 stops transmitting (inference continues)
- `0x02` `RESUME` — S3 done → P4 resumes transmitting

TX gating is a `volatile bool s_transmit_enabled`; when disabled the TX task drains its queue without writing. GPIO10 RX noise is mitigated with `INPUT_PULLDOWN`.

## FreeRTOS Tasks

| Task | Core | Priority | Stack | Role |
|---|---|---|---|---|
| `tflm` | 1 | 3 | 32 KB | Capture → pipeline → inference → queue packet |
| `uart_tx` | 0 | 2 | 4 KB | Send packets; drops when `s_transmit_enabled == false` |
| `uart_rx` | 0 | 2 | 4 KB | Read S3 control packets; toggle `s_transmit_enabled` |
| `sd` | 0 | 1 | 8 KB | Write PGM frames to SD |

## Configurable Parameters

| Param | Default | Description |
|---|---|---|
| `CAMERA_TYPE` | IMX219 | `CAMERA_TYPE_IMX219` / `_OV5647` / `_AUTO` (I2C probe) |
| `PREPROCESS_MODE` | BG | `PREPROCESS_MODE_BG` / `_GRAY` / `_RGB` |
| `kDebugBaud` | 921600 | Debug serial baud |
| `kUartBaud` | 921600 | P4↔S3 UART baud |
| `kEnableSdLogger` | true | Save frames to SD |
| `kSaveEveryNFrames` | 10 | Save every N frames |
