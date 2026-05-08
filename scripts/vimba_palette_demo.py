"""Standalone demo: direct-camera capture + BGW palette rendering.

Captures frames from the Manta G-033B via the Vimba SDK using the
existing VmbCamera per-call software-trigger pattern, saves each frame
as both raw grayscale and BGW-palette-rendered PNGs, and reports a
PASS/FAIL summary on sustained streaming.

Designed for a quick lab spike on Bulbasaur with two purposes:

  1. Answer "does sustained per-call Vimba streaming work for >1 frame?"
     The May 6 spike captured one frame; this demo runs N frames over
     M seconds and times each one. Result determines whether the
     streaming-callback refactor in path_a_vimba_integration_plan.md
     is necessary.

  2. Demonstrate the BGW palette LUT (gui/ksa_palette.py) on a real
     camera frame, not a synthetic one. Side-by-side raw vs BGW PNGs
     give visual evidence that direct-camera mode can produce frames
     that look like what kSA renders.

USAGE
    # On Bulbasaur, with kSA fully closed:
    python scripts/vimba_palette_demo.py
    python scripts/vimba_palette_demo.py --duration 60 --rate 2
    python scripts/vimba_palette_demo.py --out C:/temp/vimba_demo

    # On Mac for sanity-checking the script:
    python scripts/vimba_palette_demo.py --mock --duration 5

PREREQUISITES
    - kSA 400 must be CLOSED (camera lock is exclusive per GigE Vision)
    - vmbpy installed (already on Bulbasaur per May 6 lab visit)
    - VimbaX SDK installed system-wide

OUTPUT (per frame, in --out directory)
    vimba_demo_<ts>_NNN_raw.png  — grayscale (1-channel)
    vimba_demo_<ts>_NNN_bgw.png  — BGW palette applied (3-channel RGB)
    vimba_demo_<ts>_summary.txt  — timing + pass/fail summary
"""
from __future__ import annotations

import argparse
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np

# Add the repo root to sys.path so we can import gui.ksa_palette when
# invoked from anywhere.
_repo_root = Path(__file__).resolve().parent.parent
if str(_repo_root) not in sys.path:
    sys.path.insert(0, str(_repo_root))

from gui.ksa_palette import apply_palette_fixed_range


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Vimba direct-camera capture + BGW palette demo",
    )
    parser.add_argument(
        "--duration", type=float, default=30.0,
        help="Total capture duration in seconds (default 30)",
    )
    parser.add_argument(
        "--rate", type=float, default=1.0,
        help="Target frame rate in Hz (default 1)",
    )
    parser.add_argument(
        "--out", type=Path, default=Path("./vimba_demo_output"),
        help="Directory for per-frame PNGs and summary",
    )
    parser.add_argument(
        "--bit-depth", type=int, default=12,
        help="Camera ADC bit depth (default 12 for Manta G-033B)",
    )
    parser.add_argument(
        "--mock", action="store_true",
        help="Skip camera, generate synthetic Gaussian frames "
             "(for sanity-checking the script on a machine without a camera)",
    )
    args = parser.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    n_target = max(1, int(args.duration * args.rate))
    period = 1.0 / args.rate

    print(f"Target: {n_target} frames over {args.duration:.0f}s at {args.rate} Hz")
    print(f"Output: {args.out}")
    print(f"Mode:   {'MOCK (synthetic)' if args.mock else 'REAL (Vimba SDK)'}")
    print()

    if args.mock:
        success = _run_mock(n_target, period, args.out, ts, args.bit_depth)
    else:
        success = _run_real(n_target, period, args.out, ts, args.bit_depth)

    return 0 if success else 1


def _run_mock(
    n_target: int, period: float, out_dir: Path, ts: str, bit_depth: int,
) -> bool:
    """Generate synthetic 12-bit Gaussian frames; exercises the save path."""
    h, w = 492, 656
    cy, cx = h // 2, w // 2
    yy, xx = np.ogrid[:h, :w]
    r2 = (xx - cx) ** 2 + (yy - cy) ** 2
    sigma = 80.0
    peak = (1 << bit_depth) - 1
    base = (peak * np.exp(-r2 / (2 * sigma * sigma)))

    times = []
    saved = 0
    for i in range(n_target):
        t0 = time.time()
        # Add a touch of intensity drift so successive frames differ
        intensity = (base * (0.6 + 0.4 * np.sin(i * 0.4))).astype(np.uint16)
        _save_pair(intensity, out_dir, ts, i, bit_depth)
        saved += 1
        times.append(time.time() - t0)
        elapsed = time.time() - t0
        if elapsed < period:
            time.sleep(period - elapsed)
    return _summarize(times, saved, n_target, out_dir, ts)


def _run_real(
    n_target: int, period: float, out_dir: Path, ts: str, bit_depth: int,
) -> bool:
    """Capture from the actual Manta camera via vmbpy."""
    try:
        from vmbpy import VmbSystem
    except ImportError:
        print(
            "ERROR: vmbpy not installed.\n"
            "  On Bulbasaur:  python -m pip install vmbpy\n"
            "  Use --mock if you just want to sanity-check the script.",
        )
        return False

    print("Connecting to Manta G-033B via Vimba SDK...")
    times: list[float] = []
    failures: list[tuple[int, str]] = []
    saved = 0

    try:
        with VmbSystem.get_instance() as vmb:
            cams = vmb.get_all_cameras()
            if not cams:
                print(
                    "ERROR: No Allied Vision cameras found.\n"
                    "  Check that:\n"
                    "  - kSA 400 is FULLY closed (Task Manager — it holds "
                    "the camera even when minimized)\n"
                    "  - Camera is powered and connected via GigE\n"
                    "  - VimbaX SDK is installed",
                )
                return False

            cam = cams[0]
            try:
                cam_id = cam.get_id()
            except Exception:
                cam_id = "?"
            print(f"Found camera: id={cam_id}")

            with cam:
                # Configure software trigger (per existing VmbCamera path)
                try:
                    cam.TriggerSource.set("Software")
                    cam.TriggerSelector.set("FrameStart")
                    cam.TriggerMode.set("On")
                    cam.AcquisitionMode.set("Continuous")
                except Exception as e:
                    print(f"WARNING: trigger configuration failed ({e}); "
                          f"capture may fail at frame 1")

                for i in range(n_target):
                    t0 = time.time()
                    try:
                        cam.TriggerSoftware.run()
                        frame = cam.get_frame(timeout_ms=2000)
                        img = frame.as_numpy_ndarray().squeeze()
                        _save_pair(img, out_dir, ts, i, bit_depth)
                        saved += 1
                        print(
                            f"  frame {i:3d}: OK  "
                            f"shape={img.shape}  dtype={img.dtype}  "
                            f"range=[{int(img.min())}, {int(img.max())}]  "
                            f"time={1000*(time.time()-t0):.0f}ms",
                        )
                    except Exception as e:
                        failures.append((i, str(e)))
                        print(f"  frame {i:3d}: FAILED — {e}")
                    times.append(time.time() - t0)
                    elapsed = time.time() - t0
                    if elapsed < period:
                        time.sleep(period - elapsed)
    except Exception as e:
        print(f"ERROR during Vimba session: {e}")
        return False

    return _summarize(times, saved, n_target, out_dir, ts, failures=failures)


def _save_pair(
    img: np.ndarray, out_dir: Path, ts: str, idx: int, bit_depth: int,
) -> None:
    """Save grayscale + BGW-palette PNGs for one frame."""
    from PIL import Image

    if img.dtype != np.uint8:
        max_val = float((1 << bit_depth) - 1)
        scaled = (img.astype(np.float32) / max_val * 255.0).clip(0, 255)
        gray8 = scaled.astype(np.uint8)
    else:
        gray8 = img

    raw_path = out_dir / f"vimba_demo_{ts}_{idx:03d}_raw.png"
    Image.fromarray(gray8).save(raw_path)

    # apply_palette_fixed_range with bit_depth=8 is a no-op normalization
    # for already-uint8 input; the LUT-lookup is what we actually want here.
    rgb = apply_palette_fixed_range(gray8, bit_depth=8)
    bgw_path = out_dir / f"vimba_demo_{ts}_{idx:03d}_bgw.png"
    Image.fromarray(rgb).save(bgw_path)


def _summarize(
    times: list[float],
    saved: int,
    target: int,
    out_dir: Path,
    ts: str,
    failures: list[tuple[int, str]] = (),
) -> bool:
    avg_ms = (sum(times) / len(times) * 1000.0) if times else 0.0
    total_s = sum(times)
    success = saved == target and not failures

    print()
    print("=" * 60)
    print(f"RESULT: {'PASS' if success else 'FAIL'}")
    print(f"  Target frames:  {target}")
    print(f"  Saved frames:   {saved}")
    print(f"  Failures:       {len(failures)}")
    print(f"  Avg per-frame:  {avg_ms:.1f} ms")
    print(f"  Total elapsed:  {total_s:.1f} s")
    if failures:
        print(f"  First failure:  frame {failures[0][0]} — {failures[0][1]}")
    print(f"  Output dir:     {out_dir}")
    print("=" * 60)

    summary_path = out_dir / f"vimba_demo_{ts}_summary.txt"
    lines = [
        f"Result: {'PASS' if success else 'FAIL'}",
        f"Target frames: {target}",
        f"Saved frames: {saved}",
        f"Failures: {len(failures)}",
        f"Avg per-frame: {avg_ms:.1f} ms",
        f"Total elapsed: {total_s:.1f} s",
    ]
    if failures:
        lines.append("")
        lines.append("Failures:")
        for idx, err in failures:
            lines.append(f"  frame {idx}: {err}")
    summary_path.write_text("\n".join(lines) + "\n")

    return success


if __name__ == "__main__":
    sys.exit(main())
