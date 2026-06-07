# TFLite

Here is the template of using TensorFlow Lite in the Arduino combining with imx219 CSI camera. If you tend to use it in the esp32p4, please install the core first.

## Configurable Parameters

All parameters below are defined at the top of [TFLite.ino](file:///Users/koil/Google-Teachable-Machine-TFLite-model-training/TFLite/TFLite.ino).

### UART (P4 -> S3)

- `kUartBaud`: UART baud rate (default: 921600)
- `kUartRxPin` / `kUartTxPin`: P4 UART pins (default: RX=38, TX=37)
- Packet format (8 bytes):
  - `0xAA 0x55` + `msg_type(0x01)` + `frame_id(uint16 LE)` + `label_id(uint8)` + `confidence(uint8)` + `flags(uint8)`

### Serial Monitor (P4)

- `kDebugBaud`: P4 debug serial baud rate for the Arduino Serial Monitor (default: 921600)

### SD Logger (P4)

- `kEnableSdLogger`: enable/disable SD logging (default: true)
- `kSaveEveryNFrames`: save one frame every N frames (default: 10)
- Output format: `P5` PGM, stored as:
  - `<mount>/run_0001/frame_00010.pgm`, `<mount>/run_0001/frame_00020.pgm`, ...
- Run folder naming:
  - Scans `<mount>` to find the latest `run_XXXX` folder and then creates the next one: `<mount>/run_0001`, `<mount>/run_0002`, ...

### Storage Backend (P4)

- `kStorageBackend`:
  - `StorageBackend::Auto`: try SD\_MMC first, then fall back to FFat
  - `StorageBackend::SdMmc`: SD card only (`/sdcard`)
  - `StorageBackend::FFat`: flash FAT partition only (`/ffat`)
- `kFormatFfatOnFail`:
  - If `true`, FFat will be formatted automatically on first run when no filesystem is detected
  - Formatting only affects the `ffat` partition in the flash

### FFat Capacity

FFat uses the on-board flash partition named `ffat`. With the default partition scheme in this repo (`app3M_fat9.9M_flash16MB`), it is roughly **9MB** usable.

- One 96×96 grayscale frame stored as raw bytes is `96 * 96 = 9216 bytes(Not include the PGM file header)`
- Approx maximum number of frames:
  - `~ 9 * 1024 * 1024 / 9216 ≈ 1000 frames`

PGM has a small header and the filesystem has overhead (FAT tables, directories), so treat this as an approximate upper bound.

### When storage is full

If FFat runs out of space (`ENOSPC`), the sketch will:

- Print `Storage full (ENOSPC). Stop saving frames...`
- Stop saving new frames (inference and UART output keep running)

### SD\_MMC Pins (Optional)

If your board requires custom SD\_MMC pins, enable and set these:

- `kUseSdMmcCustomPins`: set to `true`
- `kSdClkPin`, `kSdCmdPin`, `kSdD0Pin`, `kSdD1Pin`, `kSdD2Pin`, `kSdD3Pin`

If you use the on-board MicroSD slot, keep `kUseSdMmcCustomPins=false` and the sketch will try common SD\_MMC init modes automatically (4-bit/1-bit, with optional format on mount failure).

## Notes

- If you select `StorageBackend::SdMmc` and you see `sdmmc_card_init failed (0x107)` or `send_if_cond returned 0x108`, the SD card link is unstable (power/pull-ups/contact/timing).
- If you select `StorageBackend::FFat`, make sure your partition scheme includes a partition with label `ffat` (your current `app3M_fat9M_16MB` scheme includes it).

## Exporting saved images

- If using SD\_MMC (`/sdcard`): remove the SD card and read it on your computer.
- If using FFat (`/ffat`): files are in on-board flash. Export via USB MSC (if enabled for this board) or by writing a small serial-dump tool.

