"""Vimba streaming-callback demo — the pattern that should actually work.

The previous `vimba_palette_demo.py` configured TriggerMode='On' and called
`cam.get_frame()` per trigger, but failed with "Frame capturing... timed out"
on every call. The reason: with TriggerMode='On' + TriggerSource='Software',
the camera waits for triggers before producing frames on the stream — but
`get_frame()` doesn't initiate a streaming pipeline, so triggered frames
have no consumer.

This script uses Allied Vision's documented pattern (matches Jacques'
`OxideMBE_RHEED_GUI.py:489-573`):

    cam.TriggerSource.set('Software')
    cam.TriggerSelector.set('FrameStart')
    cam.TriggerMode.set('On')
    cam.AcquisitionMode.set('Continuous')

    def handler(c, s, f):
        img = f.as_numpy_ndarray().copy().squeeze()  # copy BEFORE queueing
        c.queue_frame(f)                              # recycle buffer
        save(img)

    cam.start_streaming(handler)
    for i in range(N):
        cam.TriggerSoftware.run()
        time.sleep(period)
    cam.stop_streaming()

The key call is `cam.start_streaming(handler)` — it registers the consumer.
Without it, triggers fire into the void.

Also critical: `c.queue_frame(f)` inside the handler returns the frame
buffer to the camera's pool. Forgetting it exhausts the buffer pool after
a few frames and silently halts capture. We copy the data out *before*
queueing to avoid a race between the handler reading and the camera
reusing the buffer.

Usage on Bulbasaur (kSA fully closed):
    python scripts/vimba_streaming_demo.py
    python scripts/vimba_streaming_demo.py --duration 15 --rate 1
"""
from __future__ import annotations

import argparse
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import numpy as np

_repo_root = Path(__file__).resolve().parent.parent
if str(_repo_root) not in sys.path:
    sys.path.insert(0, str(_repo_root))

from gui.ksa_palette import apply_palette_fixed_range


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Vimba streaming-callback + BGW palette demo",
    )
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument(
        "--out", type=Path, default=Path("./vimba_streaming_output"),
    )
    parser.add_argument("--bit-depth", type=int, default=12)
    args = parser.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    n_target = max(1, int(args.duration * args.rate))
    period = 1.0 / args.rate

    print(f"Target: {n_target} frames over {args.duration}s at {args.rate} Hz")
    print(f"Output: {args.out}")
    print()

    try:
        from vmbpy import VmbSystem
    except ImportError:
        print("ERROR: vmbpy not installed. Run: python -m pip install vmbpy")
        return 1

    # State shared between vmbpy's handler thread and the main loop.
    state_lock = threading.Lock()
    frame_counter = [0]
    failures: list[tuple[int, str]] = []
    first_frame_info: list[str] = []

    def handler(cam, stream, frame) -> None:
        """Camera-side callback. Runs on vmbpy's thread."""
        try:
            # Copy data OUT of the frame buffer before queueing it back to
            # the camera's pool. as_numpy_ndarray() returns a view; once
            # we queue the frame the camera can reuse that memory.
            img = frame.as_numpy_ndarray().copy().squeeze()

            # Recycle the buffer to the camera's pool. Forgetting this
            # exhausts the pool and stalls capture after a few frames.
            cam.queue_frame(frame)

            with state_lock:
                idx = frame_counter[0]
                frame_counter[0] += 1
                if idx == 0:
                    first_frame_info.append(
                        f"shape={img.shape} dtype={img.dtype} "
                        f"range=[{int(img.min())}, {int(img.max())}] "
                        f"mean={float(img.mean()):.2f}",
                    )
            _save_pair(img, args.out, ts, idx, args.bit_depth)
        except Exception as e:
            with state_lock:
                idx = frame_counter[0]
                failures.append((idx, str(e)))

    print("Connecting to Manta G-033B via Vimba SDK...")
    try:
        with VmbSystem.get_instance() as vmb:
            cams = vmb.get_all_cameras()
            if not cams:
                print(
                    "ERROR: No Allied Vision cameras found.\n"
                    "  - Confirm kSA is FULLY closed (check Task Manager)\n"
                    "  - Verify VimbaX SDK install",
                )
                return 2

            cam = cams[0]
            print(f"Found camera: id={cam.get_id()}")

            with cam:
                try:
                    cam.TriggerSource.set("Software")
                    cam.TriggerSelector.set("FrameStart")
                    cam.TriggerMode.set("On")
                    cam.AcquisitionMode.set("Continuous")
                except Exception as e:
                    print(f"WARNING: trigger config failed: {e}")

                print("Starting streaming pipeline...")
                cam.start_streaming(handler)
                try:
                    for i in range(n_target):
                        try:
                            cam.TriggerSoftware.run()
                        except Exception as e:
                            print(f"  trigger {i:3d}: FAILED - {e}")
                            failures.append((i, f"trigger: {e}"))
                            time.sleep(period)
                            continue

                        # Pace the trigger loop. Handler runs asynchronously.
                        time.sleep(period)
                        with state_lock:
                            saved_so_far = frame_counter[0]
                        print(
                            f"  trigger {i:3d}: sent  "
                            f"(handler-saved so far: {saved_so_far})",
                        )
                finally:
                    print("Stopping streaming...")
                    cam.stop_streaming()
                    # Give the last handler invocation a moment to land.
                    time.sleep(0.5)
    except Exception as e:
        print(f"ERROR during Vimba session: {e}")
        return 3

    with state_lock:
        saved = frame_counter[0]

    print()
    print("=" * 60)
    success = saved >= max(1, int(n_target * 0.8))
    print(f"RESULT: {'PASS' if success else 'FAIL'}")
    print(f"  Triggers sent:    {n_target}")
    print(f"  Frames saved:     {saved}")
    print(f"  Failures:         {len(failures)}")
    if first_frame_info:
        print(f"  First frame:      {first_frame_info[0]}")
    if failures:
        print(f"  First failure:    frame {failures[0][0]} — {failures[0][1]}")
    print(f"  Output dir:       {args.out}")
    print("=" * 60)

    summary_path = args.out / f"vimba_streaming_{ts}_summary.txt"
    lines = [
        f"Result: {'PASS' if success else 'FAIL'}",
        f"Triggers sent: {n_target}",
        f"Frames saved: {saved}",
        f"Failures: {len(failures)}",
    ]
    if first_frame_info:
        lines.append(f"First frame: {first_frame_info[0]}")
    if failures:
        lines.append("")
        lines.append("Failures:")
        for idx, err in failures:
            lines.append(f"  frame {idx}: {err}")
    summary_path.write_text("\n".join(lines) + "\n")

    return 0 if success else 1


def _save_pair(
    img: np.ndarray, out_dir: Path, ts: str, idx: int, bit_depth: int,
) -> None:
    from PIL import Image

    if img.dtype != np.uint8:
        max_val = float((1 << bit_depth) - 1)
        scaled = (img.astype(np.float32) / max_val * 255.0).clip(0, 255)
        gray8 = scaled.astype(np.uint8)
    else:
        gray8 = img

    raw_path = out_dir / f"vimba_streaming_{ts}_{idx:03d}_raw.png"
    Image.fromarray(gray8).save(raw_path)

    rgb = apply_palette_fixed_range(gray8, bit_depth=8)
    bgw_path = out_dir / f"vimba_streaming_{ts}_{idx:03d}_bgw.png"
    Image.fromarray(rgb).save(bgw_path)


if __name__ == "__main__":
    sys.exit(main())
