# TFLite

On-device sign inference for ESP32-P4 with TensorFlow Lite Micro. Captures frames from an IMX219 or OV5647 camera, applies the center-60 % crop + BT.601-luminance pipeline that exactly matches the AItraining training cache, classifies the sign, and sends the result to the ESP32-S3 over UART.

Out-of-distribution (OOD) rejection is built in: empty scenes / overexposed frames / no-sign inputs are signalled via the UART/SD `flags` byte so receivers never trust a softmax-hallucinated high-confidence label. See **OOD (No Sign) Signalling** below.

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
| `PREPROCESS_MODE_BG` (default) | Center 60 % crop → BT.601 luminance of raw (no-WB) RGB → bilinear → contrast stretch | Sign inference — bit-matches the AItraining training cache |
| `PREPROCESS_MODE_GRAY` | Same center crop + luminance + stretch, no colour logic at all | Identical pixels to BG in the default config |
| `PREPROCESS_MODE_RGB` | No inference — streams WB-corrected IMG_SIZE×IMG_SIZE×3 RGB to Serial (`0xAA 0x55 0xAA` + payload) | Data collection for AItraining purple-sign projects, like the `IMX219_RGB_Serial` example |

The **default configuration already matches the AItraining host** (verified 2026-08-26: 86/86 training frames → 0 label flips vs the training cache):

```cpp
#define BG_ENABLE_BLOB_SEARCH 0        // skip B-G blob; always center-crop (matches host fast_mode=True)
#define BG_FALLBACK_CENTER_FRAC 0.60f  // center 60% window — MUST match host _center_bbox frac
```

`BG_ENABLE_AWB` (Gray-World, default 1) has **no effect on the model input** in the default config — its gains only feed the blob search, which is compiled out. The model input must stay WB-free: the grayscale serial stream AItraining trains on is BT.601 of raw sensor RGB with no white balance.

If the crop geometry or pixel transform diverges from the AItraining host, train/inference drift is the #1 cause of "val_acc=1.0 but deployed predictions are garbage".

## Pipeline

Camera → (resize to 96×96, WB passthrough) → center 60 % crop [20,77) → BT.601 luminance (30/59/11) → bilinear 96×96 → contrast stretch (span ≥ 24) → int8 gray−128 → TFLite int8 inference → (OOD 3-layer gating) → UART to S3 / SD log.

Any change to `image_provider.cpp` must be mirrored in `AItraining/image_preprocess.py` (see repo-root `CLAUDE.md` for the canonical 6-step pipeline).

## OOD (No Sign) Signalling

Softmax always normalises its logits to sum to 1.0, which means it outputs a misleadingly "confident" class even on completely unrelated inputs (empty desk, hand in front of lens, fully black frame). The firmware runs a **3-layer cascade** after inference and marks the frame "No Sign" in the shared `flags` field:

| Layer | Signal | Tunable macro (default) | Meaning |
|---|---|---|---|
| **L1 – pixel prior** | `sign_pct` % of G-channel pixels in `(BG_MASK_DARK_THRESH, BG_MASK_LUM_THRESH)` | `OOD_SIGN_PCT_MIN=0.3`, `OOD_SIGN_PCT_MAX=70.0` | Real signs produce a band of mid-gray G pixels; too few → empty scene / overexposed; too many → full shadow / lens cap. |
| **L2 – max prob** | `max(raw_score_i) / Σraw_scores` on the int8 output tensor | `OOD_MAX_PROB_MIN=0.60` | Softmax can be confidently-wrong on OOD; combined with entropy this rejects the worst cases. |
| **L3 – normalised entropy** | `-Σ p·log(p) / log(N)` where `N = kCategoryCount` | `OOD_ENTROPY_RATIO_MAX=0.70` | 1.0 = perfectly uniform vote (the net is guessing). Sharp decisions score ≤ 0.4. |

The defaults match the AItraining live-predict gates so the device rejects exactly the same frames the host preview rejects.

Master switch: `OOD_ENABLE` (default `1`). Set `=0` to disable gating entirely (debug only — otherwise you get fake high-confidence labels on no-sign inputs).

Each macro is `#ifndef`-guarded so you override per project with compiler flags.

### Unified flags contract (UART + SD + Serial) — receivers MUST use this

`flags` is the **only** authoritative way to distinguish a real sign from "No Sign". Receivers **must NOT** trust `label_id` when the OOD nibble is set.

```
flags byte = [high nibble: status] [low nibble: sign_ready]

low  nibble == 0x2   →  sign_ready (unchanged from legacy protocol)
high nibble == 0x0   →  in-distribution (real sign)
high nibble == 0xF   →  OUT of distribution (No Sign)
```

| `flags` | Meaning | How to read `label_id` / `confidence` |
|---|---|---|
| `0x02` | **Real sign** (in-distribution) | Use them verbatim. `label_id` indexes `kCategoryLabels`. |
| `0xF2` | **No Sign** (OOD suppressed) | Ignore both. They still hold the softmax hallucination for debug/analysis, but the classifier has rejected this frame. |

**C helper for the S3 receiver side:**

```c
static inline bool pkt_is_no_sign(uint8_t flags)  { return (flags & 0xF0) == 0xF0; }
static inline bool pkt_is_real_sign(uint8_t flags){ return (flags & 0x0F) == 0x02 && !pkt_is_no_sign(flags); }
```

Equivalent Python for offline SD analysis:

```python
df["is_real_sign"] = (df["flags"] & 0x0F) == 0x02 & ((df["flags"] & 0xF0) != 0xF0)
df["is_no_sign"]   = (df["flags"] & 0xF0) == 0xF0
```

### L1 `sign_pct` thresholds (BG_MASK_DARK/LUM)

OOD L1 counts pixels whose G channel falls in `(BG_MASK_DARK_THRESH, BG_MASK_LUM_THRESH)`. The defaults are `dark=0 / lum=100` — exactly the AItraining live-predict mask defaults (`bg_dark_thresh=0`, `bg_lum_thresh=100`). In the default config (blob search off) these thresholds drive **only** the sign_pct OOD gate — they never touch the model-input pixels — so keeping them equal to the host's is what makes device and host reject the same frames.

Per-class thresholds saved in the project (e.g. a class configured with dark=20/lum=80) are used by the host training path and by the host `_focus_bbox` preview, not by the device — do not bake per-class values into the firmware; keep the host defaults unless you consistently run live with modified preview sliders.

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
| 7 | flags (`0x02` = real sign_ready, `0xF2` = No Sign, see OOD contract above) |
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

## Debug Serial Commands (USB CDC → P4, runtime tuning + capture mode)

The `dbgser` FreeRTOS task (core 0, prio 1, 6 KB stack) listens on the USB-CDC debug serial at **921600 baud**.  There are **two input styles** so students + Python scripts share the same port — they auto-route so you never have to switch anything:

| Style | Interface | Best for |
|---|---|---|
| **A. Plain text lines ✅ (students, Arduino Serial Monitor)** | Type a line, press Enter | Tuning thresholds live in the lab, no scripts |
| **B. Sync-framed binary (AA 55 CC)** | Same "flag-parser" feel as P4↔S3 UART | Python automation / offline capture scripts |

The runtime control features are identical across both:

1. **Tune Dark/Lum (Host UI sliders equivalent) at runtime**, no recompile.
2. **Tune OOD thresholds (sign_pct/max_prob/entropy) at runtime** so you can eyeball-sign-off on empty-desk / real-sign frames live.
3. **Flip between INFERENCE ↔ CAPTURE_RGB ↔ CAPTURE_GRAY modes** from the host without reset.  In CAPTURE modes, inference / UART to S3 / SD logging is fully paused and frames are streamed on Debug Serial with a length + xor checksum header.

---

### ✅ Style A — Arduino Serial Monitor (teaching, copy-paste commands)

**Serial Monitor setup:**
- Port: your P4 USB-CDC (e.g. `/dev/cu.usbmodem…` on macOS, `COMx` on Windows)
- Baud: **921600**
- Line ending dropdown: **Both NL & CR**

**Boot-up banner:** every reset prints this first so you know the CLI is alive:
```
DEBUG: task up @ 921600 baud — type 'help' for Serial Monitor commands; binary sync=AA 55 CC
```

**Then you just type:**

| You type (one line + Enter) | Device prints back | What it does |
|---|---|---|
| `help` or `?` | 10-line command list + 1 example per command | On-device cheat sheet so students never remember this table |
| `ping` | `PONG` | Cheap connectivity check before tuning anything |
| `get mask` | `MASK: dark=0 lum=100` | Read current Dark/Lum (G-channel sign-mask thresholds) |
| `set mask 0 100` | `OK  MASK: dark=0 lum=100` | Write Dark/Lum — *exactly the Host UI sliders*, auto-clamped so dark < lum, applied on the next GetImage frame |
| `get ood` | `OOD: sp=0.30..70.00 mp=0.600 er=0.700` | 3-layer OOD thresholds right now: sign_pct% window, min max_prob, max normalised entropy ratio |
| `set ood 0.3 70.0 0.60 0.70` | `OK  OOD: sp=0.30..70.00 mp=0.600 er=0.700` | Set the host-aligned OOD defaults (empty scenes with <0.3% "sign-like pixels" still count) |
| `get mode` | `MODE: inference` | Print current OpMode |
| `mode infer` | `OK  MODE: inference` | Back to normal: run inference, queue UART→S3, write every N-th frame to SD |
| `mode rgb` | `OK  MODE: capture_rgb` | **Pause inference + UART + SD**.  Instead the device streams RAW RGB24 96×96 sensor frames on Debug Serial (use Style B Python parser to save PNGs offline) |
| `mode gray` | `OK  MODE: capture_gray` | **Same pause**, but stream the *preprocessed GRAY8 96×96* frame (= exactly what the TFLite input tensor receives).  Every 30th frame also logs `sign_pct` so you can tune Dark/Lum by the numbers. |
| `mode infergray` | `OK  MODE: infer+gray` | **Inference keeps running** (UART→S3 still active, logs muted) **and** the model-input GRAY8 96×96 frame streams on Serial as `AA 55 AA + payload` — AItraining can parse it live while the S3 still gets packets. Best tool for comparing device input vs host preview. |
| (typo) `mask 30 85` | `ERR: unknown command 'mask', type 'help'` | Student-friendly "tell them where to look" error messages instead of silent fail |
| (typo) `set mask foo` | `ERR: set mask <D> <L>, e.g. set mask 0 100` | Inline example printed right on the error line |
| (typo) `mode neon` | `ERR: mode infer\|rgb\|gray\|infergray, got 'neon'` | Always prints the valid keyword enum |

**Classroom session recipe (5 minute warm-up):**
```
1. Plug in, open Serial Monitor → 921600 / Both NL & CR
2. type  ping                       ← see PONG → connection works
3. type  get mask                   ← remember the current values in case you mess up
4. type  set mask 0 100             ← host-aligned sign-mask window (default)
5. type  mode gray                  ← the inference logs stop, CAPTURE_GRAY frames stream
6. (hold up END sign in front of cam)
7. (on host Python) save 5 frames as PNG → compare side-by-side with training images
8. type  mode infer                 ← back to normal, frame_id continues where it left off
```

---

### Style B — Sync-framed binary (Python / scripts / automation)

This is the "flag-parser" style the student asked for; it shares the exact `sync + cmd + len + payload + xor8` structure as the P4↔S3 UART channel.  Use it when you want a script to tune thresholds for you, or to save 1000 capture frames as PNG automatically.

#### Command / Response Frame Format

Every frame on Debug Serial is laid out exactly the same on TX (host→P4, commands) and RX (P4→host, responses):

```
 0    1    2     3    4      5..(5+len-1)   5+len
| AA | 55 | CC | cmd | len | <payload bytes> | xor8(bytes 0..5+len-1) |
```

- `sync = 0xAA 0x55 0xCC` — the 3-byte `CC` tail distinguishes debug commands from:
  - UART inference (`0x01` after `AA 55`), UART control (`0x02`),
  - old RGB data collection stream (`0xAA 0x55 0xAA`),
  - runtime capture frames (`0xAA 0x55 0xAB`).
- `cmd` — byte from the table below.  Responses use the same `cmd` byte as the request on success.
- **On error** the response sets `cmd | = 0x80` and `<payload>` is a null-terminated ASCII message (e.g. `"checksum"`, `"len16"`, `"bad mode"`).
- `xor8` is XOR of every byte from offset 0 through the last payload byte; the checksum byte itself is never included.

#### Command Quick Reference

| Cmd ID | Name | Request payload length | Payload | Response payload |
|---|---|---|---|---|
| `0x00` | `NOP` | 0 | — | 0 bytes (ack) |
| `0x7F` | `PING` | 0 | — | 0 bytes (ack) |
| `0x01` | `SET_OOD_THRESHOLDS` | **16** | `sign_pct_min LE f32` (0..100), `sign_pct_max LE f32`, `max_prob_min LE f32` (0..1), `entropy_ratio_max LE f32` (0..1) | 0 bytes (ack); live thresholds take effect on the next inference frame |
| `0x02` | `GET_OOD_THRESHOLDS` | **0** | — | 16 bytes, same layout as SET |
| `0x03` | `SET_MASK_THRESHOLDS` | **2** | `dark uint8`, `lum uint8` (G-channel sign mask Dark/Lum sliders from AItraining) | 0 bytes (ack); applied next GetImage via `ImageProviderSetMaskThresholds` which auto-clamps dark < lum |
| `0x04` | `GET_MASK_THRESHOLDS` | **0** | — | 2 bytes: `[dark, lum]` |
| `0x05` | `SET_MODE` | **1** | `mode uint8`: `0`=INFERENCE, `1`=CAPTURE_RGB, `2`=CAPTURE_GRAY, `3`=INFER_GRAY (inference + model-input GRAY stream) | 0 bytes (ack); mode takes effect at the start of the next inference-task loop iteration |
| `0x06` | `GET_MODE` | **0** | — | 1 byte: current `mode uint8` |

Error byte pattern examples: `cmd = 0x83` → SET_MASK_THRESHOLDS failed; payload starts with `"len2\0"`.

### Capture Mode Frame Layout (P4 → host on Debug Serial)

There are **two wire formats** for capture mode on Debug Serial.  The defaults (`mode gray` / `mode rgb`) are intentionally shaped to plug directly into the AItraining desktop app's existing serial frame reader **without changing a line of Python** — they are called **Format A – PLAIN**.  Format B (EXTENDED) is kept for script tools that want per-frame metadata + strict checksums.

---

#### ✅ Format A – PLAIN (AItraining-compatible, **default when you type `mode gray` / `mode rgb`**)

Use this when you want:
- Live serial preview in the AItraining dashboard
- "Source capture → training set" flow for your project (e.g. 96×96 grayscale upper road-sign classes)
- No extra binary parser code on the Python side

```
Wire layout for one frame:
+----+----+----+─────────────────────────────────+
| AA | 55 | AA |  <side² × channels> PIXEL BYTES  |
+----+----+----+─────────────────────────────────+
```

| Mode keyword | Channels | Frame size (bytes) | Pixel type | Matches AItraining setting |
|---|---|---|---|---|
| `mode gray` (or `mode 2`) | **1** | 96×96 = 9216 | GRAY8 (`uint8 = int8 + 128`) — exactly what TFLite input tensor receives | Sync=`AA 55 AA`, side=96, channels=1 (default for grayscale projects) |
| `mode rgb` (or `mode 1`) | **3** | 96×96×3 = 27648 | RGB24 R-G-B interleave, WB-neutralised camera output | Sync=`AA 55 AA`, side=96, channels=3 |

**Why this matches AItraining's SerialFrameReader:**
- [SerialFrameReader.read_frame()](file:///Users/koil/Google-Teachable-Machine-TFLite-model-training/AItraining/serial_device.py#L61-L101) looks for header=`AA 55 AA` by default then reads exactly `side² × channels` bytes.
- It validates the *next frame header appears immediately after* (line 93-96), so **no kind/fid/w/h/xor bytes can be inserted between frames** — hence Format A strips them entirely.
- On the firmware side, the `sign_pct` / `avg_cap` timing summary is still *printed as human-readable ASCII text every 30 frames on the debug serial port*, but it is explicitly **not injected into the pixel stream**; it appears in the debug terminal only, after the pixel data is flushed, so it's seen by the human but skipped by the frame reader's `buffer.find(AA 55 AA)` search.

---

#### Format B – EXTENDED (script tools only, reachable via `mode ext_gray` / `mode ext_rgb` or numeric 11/12)

Use this when you're writing a custom Python test harness and need per-frame frame_id + xor8 checksum.

```
Wire layout for one frame:
 0    1    2     3     4      5       6      7       8
| AA | 55 | AB | kind | frame_id LE16 | width LE16 | height LE16 | <BYTES…> | xor8(0..last_byte_before_xor) |
```

| kind value | pixels | byte count |
|---|---|---|
| `0x01` | RGB24, R-G-B triplets, interleaved, sensor WB-neutralised (CameraGetRgb) | `width × height × 3` |
| `0x02` | GRAY8, exactly what the TFLite input tensor would receive (`uint8 = int8 + 128`) | `width × height × 1` |

width/height are always `IMG_SIZE × IMG_SIZE` (96×96 default).  Mode transitions are logged with a line `MODE: INFERENCE → CAPTURE_RGB  [frame=N]` on Debug Serial so you can resynchronise your parser after a SET_MODE; every 30th extended-capture frame also prints `sign_pct` (G∈(Dark,Lum)%) and the GetImage average latency so you can tune Dark/Lum on pixel statistics directly.

---

### Captured vs. inference behaviour quick comparison

| `s_op_mode` (numeric) | GetImage → Invoke | SD logger | UART → S3 | Debug Serial TX (pixel format) |
|---|---|---|---|---|
| **0 INFERENCE** (default) | ✓ | every N-th frame | ✓ (unless S3 ACK_STOP) | periodic logs + debug print every 10/30 frames |
| **1 CAPTURE_RGB / PLAIN** | ✗ — skip interpreter, skip OOD | ✗ | ✗ — pause tx/rx state unchanged | `AA 55 AA + 96×96×3 RGB24` — AItraining ready |
| **2 CAPTURE_GRAY / PLAIN** | ✓ GetImage only, no Invoke | ✗ | ✗ — pause | `AA 55 AA + 96×96×1 GRAY8` — AItraining ready, **what you should use by default for project preview** |
| **3 INFER_GRAY / PLAIN** | ✓ full inference + OOD | every N-th frame | ✓ | `AA 55 AA + 96×96×1 GRAY8` (the model input) — live device-vs-host comparison while S3 stays connected |
| 11 EXT_RGB | ✗ skip Invoke | ✗ | ✗ pause | `AA 55 AB … <RGB bytes> xor8` — extended, script tools only |
| 12 EXT_GRAY | ✓ GetImage only | ✗ | ✗ pause | `AA 55 AB … <GRAY bytes> xor8` — extended, script tools only |

Returning to INFERENCE via `mode infer` (or `SET_MODE 0`) automatically resumes inference, SD logger, and UART TX/RX from where they were — none of the internal queues are reset so consecutive UART frame_id is monotonic across a mode flip.

---

### 🧑‍🎓 Student quick recipe: "Why is Host Preview RIGHT but device wrong?"

Use the ready-to-go script [side_by_side.py](file:///Users/koil/Google-Teachable-Machine-TFLite-model-training/TFLite/side_by_side.py) (right next to this README) — no paths to edit, no Python package hunting beyond what you already installed for AItraining.

**4 steps in two terminals:**

| Terminal A (Serial Monitor, 921600 8N1, NL&CR) | Terminal B (shell, `cd TFLite/`) |
|---|---|
| 1. Hold sign still in front of cam |  |
| 2. `set mask 0 100` → confirm `get mask` matches the Preview slider defaults |  |
| 3. `mode infergray` → inference keeps running AND the model-input GRAY frame streams as `AA 55 AA …` (you can close Serial Monitor now to free the port, or leave it open on a separate machine) |  |
|  | 4. `python3 side_by_side.py --class RIGHT` → takes ~10 s, prints `MAD` score + drops `sbs_02.png` in `sbs_output/` |

Open `sbs_output/sbs_00.png` side-by-side with your Host Preview "Input" crop:
- `MAD < 3 DN` → pixels line up — the device is feeding the model the training transform. If device labels are still wrong, check the firmware version (pre-2026-08-26 full-frame firmware has the signature "END always → RIGHT"), the OOD gate (`ood` / `flags=0xF2` = No Sign), and label name order in `tm_classes.json` ↔ `labels.txt` ↔ `kCategoryNames[]`.
- `MAD > 20 DN` or the two pictures look nothing alike → the flashed firmware predates the center-crop fallback: reflash the current source (defaults are already host-aligned — `BG_ENABLE_BLOB_SEARCH=0`, `BG_FALLBACK_CENTER_FRAC=0.60`, mask 0/100) and retry.

`python3 side_by_side.py --help` lists `--port`, `--baud`, `--host-cache-dir`, `--project-tmproj`, `--frames`, `--channels`, `--sync` overrides for non-default workflows.

### Host-side snippet (Python, building commands for runtime tuning)

```python
def make_cmd(cmd: int, payload: bytes = b"") -> bytes:
    assert 0 <= len(payload) <= 255
    hdr = bytes([0xAA, 0x55, 0xCC, cmd & 0xFF, len(payload)])
    x = 0
    for b in hdr: x ^= b
    for b in payload: x ^= b
    return hdr + payload + bytes([x])

# Set Dark=30 / Lum=85 (mirrors AItraining "upper" project union default)
ser.write(make_cmd(0x03, bytes([30, 85])))

# Switch to PLAIN GRAY capture — this format is what AItraining reads natively
ser.write(make_cmd(0x05, bytes([0x02])))
```

### Host-side snippet (Python, reading EXTENDED capture frames)

```python
def read_capture(ser):
    sync = ser.read(3)
    while sync != bytes([0xAA, 0x55, 0xAB]):
        sync = sync[1:] + ser.read(1)
    head = ser.read(1 + 2 + 2 + 2)   # kind, fid, w, h
    kind, fid, w, h = head[0], int.from_bytes(head[1:3], "little"), \
                       int.from_bytes(head[3:5], "little"), int.from_bytes(head[5:7], "little")
    n = (w * h * 3) if kind == 0x01 else (w * h * 1) if kind == 0x02 else 0
    body = ser.read(n)
    x = 0
    for b in (sync + head + body): x ^= b
    ck = ser.read(1)[0]
    assert ck == x, f"xor {ck:02x}!={x:02x}"
    return kind, fid, w, h, body
```

## SD Logger (frames + label sidecar)

When `kEnableSdLogger == true` (default true), the `sd` FreeRTOS task runs on core 0 at priority 1 and writes every N-th frame to the SD (or FFat) mount point.

Each new session auto-creates a numbered run directory:

```
<mount_point>/run_0001/
├── frame_00000.pgm
├── frame_00010.pgm
├── frame_00020.pgm
└── labels.csv
```

- **frame_XXXXX.pgm** — 96×96 8-bit grayscale PGM (`P5`) of the exact pixels fed into the TFLite int8 input tensor.
- **labels.csv** — one row per written frame, created on first write with a header. Columns:

| Column | Type | Semantics |
|---|---|---|
| `frame_id` | uint16 | Matches the UART packet `frame_id` so you can rejoin live UART and stored frames offline. |
| `label_id` | uint8 | Top-1 class index (int8 argmax) from the TFLite output tensor. If `flags` says No Sign this is the raw softmax hallucination — ignore. |
| `confidence` | uint8 | uint8-scaled best raw score (0–255). Ignore when OOD nibble is set. |
| `flags` | uint8 | **Shared contract with UART**: `(flags & 0xF0) == 0xF0` → No Sign. Low nibble `=2` means sign_ready. |

Use `FFatReader` `bundle_runs` or `SDReader` + a plain card reader to pull the run off the device and then run the same offline analysis notebook you use on UART captures.

## FreeRTOS Tasks

| Task | Core | Priority | Stack | Role |
|---|---|---|---|---|
| `tflm` | 1 | 3 | 32 KB | Capture → pipeline → 3-layer OOD gating → queue UART and SD packets |
| `dbgser` | 0 | 1 | 6 KB | **Debug Serial command parser**: runtime Dark/Lum + OOD tuning + mode switch (INFERENCE ↔ CAPTURE) via sync-framed commands |
| `uart_tx` | 0 | 2 | 4 KB | Send UART inference packets; drops silently when `s_transmit_enabled == false` |
| `uart_rx` | 0 | 2 | 4 KB | Read S3 control packets (`ACK_STOP` / `RESUME`); toggle `s_transmit_enabled` |
| `sd` | 0 | 1 | 8 KB | Write `frame_XXXXX.pgm` + append one row to `labels.csv` per saved frame |

## Configurable Parameters

| Param | Default | Description |
|---|---|---|
| `CAMERA_TYPE` | IMX219 | `CAMERA_TYPE_IMX219` / `_OV5647` / `_AUTO` (I2C probe) |
| `PREPROCESS_MODE` | BG | `PREPROCESS_MODE_BG` / `_GRAY` / `_RGB` |
| `kDebugBaud` | 921600 | Debug serial baud |
| `kUartBaud` | 921600 | P4↔S3 UART baud |
| `kEnableSdLogger` | true | Save frames + labels sidecar to SD/FFat |
| `kSaveEveryNFrames` | 10 | Write every N-th frame (lower = more storage / higher SD bus load) |

### OOD / preprocessing overrides (pass via -D compiler flags, not edit)

All of these are `#ifndef`-guarded in `image_provider.h`. The **compiled defaults already match the AItraining host** for the monochrome road-sign projects (small/upper) — you normally need NO flags:

```
# Host-aligned defaults (these are the in-header defaults — no flags needed)
BG_ENABLE_BLOB_SEARCH=0
BG_FALLBACK_CENTER_FRAC=0.60f
BG_MASK_DARK_THRESH=0
BG_MASK_LUM_THRESH=100
OOD_ENABLE=1
OOD_SIGN_PCT_MIN=0.3f
OOD_SIGN_PCT_MAX=70.0f
OOD_MAX_PROB_MIN=0.60f
OOD_ENTROPY_RATIO_MAX=0.70f
```

For datasets where the signs do have a reliable purple/blue component (the original B-G use case), set `-DBG_ENABLE_BLOB_SEARCH=1` to re-enable the B-G blob auto-crop and tune `BG_MASK_DARK_THRESH` / `BG_MASK_LUM_THRESH` to the union of per-class G-channel sign-mask windows. Note the host's `_find_bg_roi` (the Python mirror of that path) is currently dead code — if you use blob search, port your host-side verification first.
