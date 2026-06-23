"""Validate specular-detection stability across a Rahim STO trajectory.

Sample N frames evenly from a dataset directory, run ``detect_specular`` and
``image_derived_roi`` on each, and produce a 2x2 plot of:
  - specular X over time
  - specular Y over time
  - ROI extent (width, height) over time
  - specular position scatter (X, Y)

Quality signal: low std on (X, Y) means the detector is locked onto specular
across the whole trajectory; high std or a wandering trace means the detector
is being misled by other bright features and we need to harden it before
deploying to a live lab session.

Usage:
    python scripts/validate_specular_stability.py <dataset_dir> [--n 100] [--out path.png]
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
from PIL import Image
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

# Allow running as a top-level script: scripts/ is a sibling of gui/.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from gui.specular import detect_specular, image_derived_roi  # noqa: E402


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("dataset_dir", type=Path)
    p.add_argument("--out", type=Path, default=None)
    p.add_argument("--n", type=int, default=100, help="Sample N frames evenly")
    args = p.parse_args()

    bmps = sorted(args.dataset_dir.rglob("*.bmp"))
    if not bmps:
        print(f"No .bmp files under {args.dataset_dir}")
        return 1

    if len(bmps) > args.n:
        idx = np.linspace(0, len(bmps) - 1, args.n).astype(int)
        bmps = [bmps[i] for i in idx]

    print(f"Processing {len(bmps)} frames from {args.dataset_dir.name}")

    xs: list[int] = []
    ys: list[int] = []
    roi_w: list[int] = []
    roi_h: list[int] = []
    H = W = 0
    for bmp in bmps:
        arr = np.array(Image.open(bmp))
        g = arr.astype(np.float32)
        if g.ndim == 3:
            g = g[:, :, 1]
        H, W = g.shape
        x, y = detect_specular(g)
        roi = image_derived_roi(g, x, y)
        xs.append(x)
        ys.append(y)
        roi_w.append(roi[1].stop - roi[1].start)
        roi_h.append(roi[0].stop - roi[0].start)

    xs_a = np.asarray(xs)
    ys_a = np.asarray(ys)
    rw = np.asarray(roi_w)
    rh = np.asarray(roi_h)

    print(f"  Frame: {W}x{H}")
    print(
        f"  Specular X: mean={xs_a.mean():.1f}, std={xs_a.std():.2f}, "
        f"span={xs_a.max() - xs_a.min()}px ({(xs_a.max() - xs_a.min()) / W * 100:.1f}% of W)"
    )
    print(
        f"  Specular Y: mean={ys_a.mean():.1f}, std={ys_a.std():.2f}, "
        f"span={ys_a.max() - ys_a.min()}px ({(ys_a.max() - ys_a.min()) / H * 100:.1f}% of H)"
    )
    print(f"  ROI W: mean={rw.mean():.0f}, std={rw.std():.1f}px")
    print(f"  ROI H: mean={rh.mean():.0f}, std={rh.std():.1f}px")

    fig, axes = plt.subplots(2, 2, figsize=(14, 8))
    axes[0, 0].plot(xs_a, color="red", linewidth=0.8)
    axes[0, 0].set_title(f"Specular X (mean={xs_a.mean():.0f}, std={xs_a.std():.1f})")
    axes[0, 0].set_xlabel("Frame index")
    axes[0, 0].set_ylabel("X (col)")
    axes[0, 0].set_ylim(0, W)

    axes[0, 1].plot(ys_a, color="cyan", linewidth=0.8)
    axes[0, 1].set_title(f"Specular Y (mean={ys_a.mean():.0f}, std={ys_a.std():.1f})")
    axes[0, 1].set_xlabel("Frame index")
    axes[0, 1].set_ylabel("Y (row)")
    axes[0, 1].set_ylim(0, H)

    axes[1, 0].plot(rw, color="red", linewidth=0.7, label=f"width (mean={rw.mean():.0f})")
    axes[1, 0].plot(rh, color="cyan", linewidth=0.7, label=f"height (mean={rh.mean():.0f})")
    axes[1, 0].set_title("ROI extent over time")
    axes[1, 0].set_xlabel("Frame index")
    axes[1, 0].set_ylabel("px")
    axes[1, 0].legend()

    axes[1, 1].scatter(xs_a, ys_a, s=8, alpha=0.5)
    axes[1, 1].set_title("Specular position scatter")
    axes[1, 1].set_xlabel("X (col)")
    axes[1, 1].set_ylabel("Y (row)")
    axes[1, 1].set_xlim(0, W)
    axes[1, 1].set_ylim(0, H)
    axes[1, 1].invert_yaxis()

    plt.suptitle(
        f"{args.dataset_dir.name}: specular-detection stability ({len(bmps)} frames)"
    )
    plt.tight_layout()
    out_path = args.out or (
        args.dataset_dir.parent / f"{args.dataset_dir.name}_specular_stability.png"
    )
    plt.savefig(out_path, dpi=100, bbox_inches="tight")
    print(f"  Saved: {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
