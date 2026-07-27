# TFLite

On-device inference for ESP32-P4 with IMX219 and TensorFlow Lite Micro.

## Pipeline

Camera → Library WB (R×2.0, B×2.0) → B-G extraction → blur → contrast stretch → blob detection → auto-crop → resize 96×96 → contrast → TFLite int8 inference → UART to S3.

See `CLAUDE.md` for the full 13-step pipeline.

## Library

```
esp32_p4_imx219_rgb_wb(200, 200)  // WB-corrected RGB, ×2.0
```

WB gains must match `AItraining/image_preprocess.py` and the example sketch.

## Configurable Parameters

| Param | Default | Description |
|---|---|---|
| `kTfWbRed` | 200 | Red gain ×100 (200 = ×2.0) |
| `kTfWbBlue` | 200 | Blue gain ×100 |
| `kDebugBaud` | 921600 | Debug serial baud |
| `kUartBaud` | 921600 | P4↔S3 UART baud |
| `kEnableSdLogger` | true | Save frames to SD |
| `kSaveEveryNFrames` | 10 | Save every N frames |

<<<<<<< Updated upstream
### UART (P4 <-> S3, bidirectional)

**P4 → S3** inference packets (9 bytes) and **S3 → P4** control packets (5 bytes) share the same UART at 921600 baud.

- `kUartBaud`: UART baud rate (default: 921600)
- `kUartRxPin` / `kUartTxPin`: P4 UART pins (new board default: RX=10, TX=11)
- P4 → S3 packet format (9 bytes):
  - `0xAA 0x55` + `msg_type(0x01)` + `frame_id(uint16 LE)` + `label_id(uint8)` + `confidence(uint8)` + `flags(uint8)` + `checksum(uint8 XOR Byte0..7)`
- S3 → P4 packet format (5 bytes):
  - `0xAA 0x55` + `msg_type(0x02)` + `command(uint8)` + `checksum(uint8 XOR Byte0..3)`
  - Commands:
    - `0x01` (`ACK_STOP`): S3 confirmed sign → P4 stops transmitting (inference continues)
    - `0x02` (`RESUME_JUNCTION`): S3 tasks done → P4 resumes TX + switches to junction crop mode

#### FreeRTOS UART Tasks

| Task | Core | Priority | Stack | Role |
|---|---|---|---|---|
| `uart_tx` | 0 | 2 | 4 KB | Sends 9-byte inference packets; drops when `s_transmit_enabled == false` |
| `uart_rx` | 0 | 2 | 4 KB | Reads 5-byte control packets from S3; sets `s_transmit_enabled` + `s_detection_state` |

The two UART tasks share `HardwareSerial(1)` — the ESP32 HardwareSerial driver is thread-safe for concurrent read/write from different tasks.

#### Transmit gating

`uart_control_enable()` / `uart_control_disable()` set the `volatile bool s_transmit_enabled` flag. When disabled, `uart_tx_task` continues to drain the queue (preventing backpressure on `inference_task`) but does not write to the UART. Inference, crop mode switching, and SD logging all continue unaffected.
=======
## UART

- **P4 → S3** (9 bytes): `0xAA 0x55 0x01` + frame_id(LE16) + label + conf + flags + csum
- **S3 → P4** (5 bytes): `0xAA 0x55 0x02` + cmd + csum  (0x01=STOP, 0x02=RESUME)
- TX gated by `s_transmit_enabled` (volatile bool)
- RX noise mitigated by `INPUT_PULLDOWN` on GPIO10
>>>>>>> Stashed changes

## FreeRTOS Tasks

| Task | Core | Pri | Stack | Role |
|---|---|---|---|---|
| `tflm` | 1 | 3 | 32K | Pipeline + inference |
| `uart_tx` | 0 | 2 | 4K | Send packets |
| `uart_rx` | 0 | 2 | 4K | Read S3 control |
| `sd` | 0 | 1 | 8K | SD logging |
