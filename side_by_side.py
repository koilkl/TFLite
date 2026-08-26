#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
side_by_side.py  —  学生调试脚本（P4 ESP32 TFLite + AItraining upper project）
放在仓库根的 TFLite/ 文件夹里，学生直接跑：

    cd Google-Teachable-Machine-TFLite-model-training/TFLite
    python3 side_by_side.py

=====================================================================
使用说明（学生 4 步走）：

  Step 1. 把 P4 板子插上 USB，开 Arduino Serial Monitor，波特率 921600，
          Line ending 选 Both NL & CR。
          把想测试的标志（END / RIGHT / NO ENTRY）举到摄像头前，
          在 Serial Monitor 打：
              set mask 30 85        ← （和你 Host Preview 截图的 Dark/Lum 一样）
              get mask             ← 确认 dark=30 lum=85
              mode gray            ← 切 CAPTURE_GRAY PLAIN 模式
          你会看到一行 banner：
              CAPTURE_GRAY/plain: stream sync=AA 55 AA + 96x96x1 GRAY8 ...
          别关 Serial Monitor，或者关了也行，只要端口留着。

  Step 2. 另开 Terminal，cd 到 TFLite/ 文件夹，运行：
              python3 side_by_side.py
          脚本会自动：
            · 在 /dev/cu.* 或 COMx 里找 CDC 串口（找不到会让你手动指定）
            · 用 AItraining 的 SerialFrameReader 连上去，读 5 帧 GRAY8 96×96
            · 自动去 upper project 的 processed_cache/ 里找对应类的 5 张
              预处理好的训练样本（RIGHT / END / NO ENTRY 三选一）
            · 拼 side-by-side PNG，存到 ./sbs_output/ 文件夹
            · 打印每一组的  mean(|device - host|)  DN 差

  Step 3. 打开 ./sbs_output/sbs_0.png ~ sbs_4.png：
            · 两张图看起来是不是同一个标志？方向/位置/对比度一致吗？
            · 如果  mean(<5 DN) ：预处理像素对齐，后续问题是 label 映射或别的。
            · 如果  mean(>20 DN) 或 肉眼一看就不是同一张 → ROI 错位/预处理参数
              不一致，回到 README 的 ROI alignment 章节去核对。

  Step 4. 想切别的类，重跑：
              python3 side_by_side.py --class END
              python3 side_by_side.py --class "NO ENTRY"

可选 CLI 参数：
  --port /dev/cu.usbmodem101    指定串口（默认自动搜）
  --baud 921600                 波特率（默认 921600）
  --side 96                     帧边长（默认 96）
  --channels 1                  1=GRAY8，3=RGB24（默认 1）
  --sync "AA 55 AA"             同步头（默认和 AItraining HEADER 一致）
  --frames 5                    抓多少帧（默认 5）
  --class RIGHT                 对比的类别名 RIGHT / END / "NO ENTRY"（默认问你）
  --host-cache-dir /path/to/p   直接指定 processed_cache/RIGHT 这种目录
  --project-tmproj /path/to/p.tmproj  项目文件，自动拿 run_latest 下的 cache
  --timeout 5.0                 抓帧最大超时秒
=====================================================================
"""

from __future__ import annotations

import argparse
import glob
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np


THIS_DIR = Path(__file__).resolve().parent
REPO_ROOT = THIS_DIR.parent
AITRAINING_DIR = REPO_ROOT / "AItraining"
if str(AITRAINING_DIR) not in sys.path:
    sys.path.insert(0, str(AITRAINING_DIR))

from serial_device import SerialFrameReader, list_serial_ports  # noqa: E402

try:
    from PIL import Image
except Exception as e:  # pragma: no cover
    print(
        "[ERROR] Pillow 没装。请先：\n"
        f"    {sys.executable} -m pip install Pillow\n"
        f"  (原始错误：{e})"
    )
    sys.exit(1)


VALID_CLASSES = ("NO ENTRY", "END", "RIGHT")


# ---------------------------------------------------------------------------
# 串口 / 参数
# ---------------------------------------------------------------------------
def pick_default_port() -> Optional[str]:
    ports = list_serial_ports()
    if not ports:
        return None
    # 优先 usbmodem / usbserial / ttyUSB / ttyACM / COM
    def score(p) -> int:
        d = p.device.lower()
        for i, kw in enumerate(
            ("usbmodem", "usbserial", "ttyacm", "ttyusb", "com", "cu.")
        ):
            if kw in d:
                return -i
        return 10
    ports.sort(key=score)
    return ports[0].device


def parse_sync(text: str) -> bytes:
    toks = text.strip().split()
    if len(toks) == 1 and len(toks[0]) % 2 == 0 and set(toks[0].lower()) <= set("0123456789abcdef"):
        toks = [toks[0][i:i + 2] for i in range(0, len(toks[0]), 2)]
    out = []
    for t in toks:
        if not t:
            continue
        out.append(int(t, 16))
    return bytes(out)


# ---------------------------------------------------------------------------
# 训练集 processed_cache 位置
# ---------------------------------------------------------------------------
@dataclass
class CacheHint:
    cls: str
    dir_path: Path


def locate_cache_dir(args: argparse.Namespace) -> Optional[Path]:
    # 1) --host-cache-dir 最高优先级
    if args.host_cache_dir:
        p = Path(args.host_cache_dir).expanduser().resolve()
        if p.is_dir():
            return p
        print(f"[WARN] --host-cache-dir 指定的目录不存在：{p}")
    # 2) 从 tmproj 或者 upper project 默认位置去找 latest run 的 processed_cache
    tmproj = None
    if args.project_tmproj:
        tmproj = Path(args.project_tmproj).expanduser().resolve()
    else:
        candidates = [
            Path.home() / "Documents/TFLiteTraining/upper/project.tmproj",
            Path.home() / "Documents/TFLiteTraining/project.tmproj",
        ]
        for c in candidates:
            if c.exists():
                tmproj = c
                break
    if not tmproj:
        return None
    return _probe_workspace_cache(tmproj, args.cls)


def _probe_workspace_cache(tmproj: Path, cls_name: str) -> Optional[Path]:
    """根据 tmproj 里的 run_latest.json 拿 workspace 路径，再找 processed_cache/<cls>。"""
    import json
    tmproj_root = tmproj if tmproj.is_dir() else tmproj.parent
    latest = tmproj_root / "tm_train_latest.json"
    if not latest.exists():
        return None
    try:
        data = json.loads(latest.read_text(encoding="utf-8"))
    except Exception as e:
        print(f"[WARN] 读 tm_train_latest.json 失败：{e}")
        return None
    workspace = data.get("workspace_dir") or data.get("run_workspace") or ""
    if not workspace:
        run_sub = data.get("run_dir_name") or data.get("latest_run") or ""
        runs_base = Path.home() / "Library/Application Support/TFLiteTraining/workspace"
        if run_sub and runs_base.exists():
            # 猜：找 workspace 下所有 /runs/run_* 最新那个
            try:
                subdirs = sorted(runs_base.glob("*/*/runs/run_*"), key=os.path.getmtime, reverse=True)
                if subdirs:
                    workspace = str(subdirs[0].parent.parent)
            except Exception:
                pass
    if not workspace:
        return None
    cand = Path(workspace).expanduser() / "processed_cache" / cls_name
    if cand.is_dir():
        return cand
    return None


def list_host_samples(cache_dir: Path, n: int) -> List[Path]:
    exts = ("*.png", "*.jpg", "*.jpeg", "*.PNG", "*.JPG", "*.JPEG")
    files: List[Path] = []
    for e in exts:
        files.extend(sorted(cache_dir.glob(e)))
    uniq = []
    seen = set()
    for f in files:
        if str(f) not in seen:
            uniq.append(f)
            seen.add(str(f))
    return uniq[:n]


# ---------------------------------------------------------------------------
# 抓帧 / 对比
# ---------------------------------------------------------------------------
def capture_frames(
    reader: SerialFrameReader,
    frames: int,
    side: int,
    channels: int,
    timeout_s: float,
) -> List[np.ndarray]:
    out: List[np.ndarray] = []
    reader.open()
    try:
        for i in range(frames):
            raw = reader.read_frame(timeout_s=timeout_s)
            arr = np.frombuffer(raw, dtype=np.uint8).reshape(side, side, channels)
            if channels == 1:
                arr = arr.squeeze(axis=-1)
            out.append(arr)
            print(f"  [{i+1}/{frames}] 抓到设备端 1 frame, shape={arr.shape} dtype={arr.dtype}")
    finally:
        reader.close()
    return out


def load_host_gray(path: Path, side: int) -> np.ndarray:
    img = Image.open(path).convert("L")
    if img.size != (side, side):
        img = img.resize((side, side), Image.BILINEAR)
    return np.asarray(img, dtype=np.uint8)


def make_sbs(left: np.ndarray, right: np.ndarray, gap: int = 10) -> Image.Image:
    h, w = left.shape[:2]
    ch = 1 if left.ndim == 2 else left.shape[2]
    if ch == 1:
        sbs = Image.new("L", (w * 2 + gap, h), color=255)
        sbs.paste(Image.fromarray(left, mode="L"), (0, 0))
        sbs.paste(Image.fromarray(right, mode="L"), (w + gap, 0))
    else:
        sbs = Image.new("RGB", (w * 2 + gap, h), color=(255, 255, 255))
        sbs.paste(Image.fromarray(left, mode="RGB"), (0, 0))
        sbs.paste(Image.fromarray(right, mode="RGB"), (w + gap, 0))
    return sbs


def mean_abs_diff(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.mean(np.abs(a.astype(np.int16) - b.astype(np.int16))))


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="学生对比脚本：P4 设备端 CAPTURE_GRAY frame vs Host processed_cache 训练样本",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--port", default=None, help="串口设备路径，留空自动找")
    p.add_argument("--baud", type=int, default=921600, help="Debug Serial 波特率")
    p.add_argument("--side", type=int, default=96, help="帧边长（和 IMG_SIZE 一致）")
    p.add_argument("--channels", type=int, default=1, choices=(1, 3), help="1=GRAY8, 3=RGB24")
    p.add_argument("--sync", default="AA 55 AA", help="十六进制同步头，空格分隔")
    p.add_argument("--frames", type=int, default=5, help="抓多少帧")
    p.add_argument(
        "--class",
        dest="cls",
        default=None,
        help="要对比的类别：NO ENTRY / END / RIGHT；留空会问你",
    )
    p.add_argument("--host-cache-dir", default=None, help="直接指定 processed_cache/<类别> 目录")
    p.add_argument("--project-tmproj", default=None, help="project.tmproj 文件或目录（自动找 cache）")
    p.add_argument("--timeout", type=float, default=5.0, help="每帧最大等待秒数")
    p.add_argument("--output", default=None, help="输出目录，默认 ./sbs_output/")
    return p


def prompt_cls() -> str:
    print("请选择要对比的训练集类别：")
    for i, c in enumerate(VALID_CLASSES, 1):
        print(f"  {i}) {c}")
    while True:
        raw = input("输入数字 (1/2/3) 或类别名：").strip()
        if not raw:
            continue
        if raw in VALID_CLASSES:
            return raw
        if raw in {"1", "2", "3"}:
            return VALID_CLASSES[int(raw) - 1]
        print("  输入无效，再试一次。")


def main(argv: Optional[List[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    sync = parse_sync(args.sync)
    if len(sync) < 3:
        print(f"[ERROR] sync header 过短：{sync!r}，应类似 AA 55 AA")
        return 2

    port = args.port or pick_default_port()
    if not port:
        print("[ERROR] 没找到串口。请用 --port /dev/cu.xxx 或 --port COMx 指定。")
        return 2
    print(f"[INFO] 串口：{port}  波特率：{args.baud}  sync={sync.hex(' ').upper()}  "
          f"side={args.side}×{args.side} ch={args.channels}")

    if not args.cls:
        args.cls = prompt_cls()
    if args.cls not in VALID_CLASSES:
        print(f"[ERROR] 类别名必须是 {VALID_CLASSES} 其中之一，你给的是：{args.cls}")
        return 2
    print(f"[INFO] 对比类别：{args.cls}")

    cache_dir = locate_cache_dir(args)
    if cache_dir is None:
        print(
            "[WARN] 找不到 processed_cache 目录。请用 --host-cache-dir 直接指定，例如：\n"
            "    python3 side_by_side.py --class RIGHT --host-cache-dir "
            "'/Users/you/Library/Application Support/TFLiteTraining/workspace/"
            "xxx/processed_cache/RIGHT'"
        )
        return 3
    print(f"[INFO] Host 训练样本目录：{cache_dir}")

    host_samples = list_host_samples(cache_dir, args.frames)
    if len(host_samples) < args.frames:
        print(f"[WARN] 目录里只有 {len(host_samples)} 张，少于你要抓的 {args.frames} 帧，将只对比 {len(host_samples)} 组。")
        if len(host_samples) == 0:
            print("[ERROR] Host 目录里一张图片都没有。")
            return 3
        args.frames = len(host_samples)

    out_dir = Path(args.output).expanduser().resolve() if args.output else THIS_DIR / "sbs_output"
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f"[INFO] 输出目录：{out_dir}")

    # ── 抓设备端 frames ─────────────────────────────────────────────
    print("[STEP 1/3] 从 P4 抓 {0} 帧 CAPTURE_GRAY...".format(args.frames))
    reader = SerialFrameReader(
        port=port,
        baud=args.baud,
        sync_header=sync,
        frame_side=args.side,
        channels=args.channels,
    )
    try:
        device_frames = capture_frames(reader, args.frames, args.side, args.channels, args.timeout)
    except Exception as e:
        print(f"[ERROR] 抓帧失败：{e}\n  · 是不是忘了在 Serial Monitor 打 mode gray？\n"
              f"  · 是不是 Serial Monitor 没关还占着端口？\n"
              f"  · baud / sync / side / channels 参数和固件一致吗？")
        return 4
    print(f"[ OK ] 抓到 {len(device_frames)} 帧设备端画面。")

    # ── 读 host 样本 ────────────────────────────────────────────────
    print("[STEP 2/3] 读 Host 训练样本并拼 side-by-side PNG ...")
    host_images: List[np.ndarray] = [
        load_host_gray(p, args.side) for p in host_samples
    ]

    # 把 device host 各自单独存一份，方便学生单看
    for i in range(args.frames):
        Image.fromarray(device_frames[i], mode="L").save(out_dir / f"device_{i:02d}.png")
        Image.fromarray(host_images[i], mode="L").save(out_dir / f"host_{i:02d}.png")

    # 拼 sbs
    diffs: List[Tuple[int, float, Path]] = []
    for i in range(args.frames):
        sbs = make_sbs(device_frames[i], host_images[i])
        out_path = out_dir / f"sbs_{i:02d}.png"
        sbs.save(out_path)
        mad = mean_abs_diff(device_frames[i], host_images[i])
        diffs.append((i, mad, out_path))
        tag = (
            "✅ 对得上" if mad < 3.0
            else "⚠️  小偏差（≤8 DN 通常还行）" if mad < 8.0
            else "❌ 差很大（ROI/阈值/通道可能错了）"
        )
        print(f"  [{i+1:02d}/{args.frames}] {out_path.name}  "
              f"mean(|device-host|) = {mad:6.2f} DN  {tag}")

    # ── 打印结论 ────────────────────────────────────────────────────
    print("\n[STEP 3/3] 小结：")
    avg_mad = float(np.mean([d[1] for d in diffs]))
    print(f"  平均 MAD = {avg_mad:.2f} DN")
    if avg_mad < 3.0:
        print("  结论：预处理像素基本一致。如果 Host 识别对了但设备端错了，下一步去核对：")
        print("    a) tm_classes.json ↔ runs/latest/labels.txt ↔ TFLite.ino kCategoryNames 顺序")
        print("    b) 设备端 label_id 映射是不是错位（比如 RIGHT 在 Host 是 2，在固件里是不是当成 0？）")
        print("    c) 是不是开了量化差异，用 Host 量化模型跑同一张 processed_cache 原图看看。")
    elif avg_mad < 8.0:
        print("  结论：预处理有轻微偏差，一般不影响最终分类结果。可以先看 label 映射。")
    else:
        print("  结论：预处理像素偏差很大。打开 sbs_*.png 肉眼看：")
        print("    · 如果 ROI 位置不同 → 设备端 blob search 找错了标志，关 BG_ENABLE_BLOB_SEARCH=0 强制中心裁剪")
        print("    · 如果亮度/对比度差很多 → Dark/Lum 没对齐（set mask <Host 滑块值>），再对比 contrast stretch 步骤")
        print("    · 如果通道反了/方向反了 → 核对 mode gray 是不是喂进去的 tensor int8+128=uint8 的正确转换")

    print(f"\n全部文件保存在：{out_dir}")
    print("学生：把 sbs_0.png 截图发老师问问题就够了。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
