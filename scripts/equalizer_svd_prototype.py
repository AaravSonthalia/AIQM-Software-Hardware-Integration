#!/usr/bin/env python3
"""SVD Equalizer prototype — MVP for the labeling-game concept.

Per the May 8 group meeting, the proposal is to replace categorical
RHEED labels (1x1, Twinned, c6x2, rt13, HTR) with **continuous** labels:
the grower drags sliders, an algorithm reconstructs an image from those
slider weights, the slider weights become the label.

This script proves the SVD half of the concept end-to-end without UI.
Pipeline:

    1. Load Classifier2's 5-class ideal training set (~159 images)
    2. Preprocess (grayscale, resize, mean-subtract)
    3. Compute SVD; take first K right-singular vectors as the basis
    4. For a new image, find best-fit weights via least-squares
    5. Reconstruct = sum(weight_i * basis_i) + mean
    6. Save: basis grid PNG, per-image reconstruction PNGs, weights JSON

Output dir layout:
    <out_dir>/
      basis_grid.png            # all K basis vectors visualized
      class_means.png           # per-class mean image (sanity check)
      reconstructions/
        <name>_recon.png        # original | reconstructed | weights bar
      weights.json              # {image_name: {weights, l2_error, top_class}}

Usage:
    python scripts/equalizer_svd_prototype.py --out /tmp/equalizer_v1
    python scripts/equalizer_svd_prototype.py --out /tmp/equalizer_v1 \\
        --extra ~/Downloads/rahim_2022_02_06/2022_02_06_renamed/Substrates \\
        --extra-sample 8

Options:
    --k             Number of SVD components (default 5 — one per class)
    --resize        WxH for downsampled processing (default 128x96)
    --extra         Additional dir of frames to score against the basis
    --extra-sample  Number of frames to sample from --extra (default 8)
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Sequence

import numpy as np
from PIL import Image
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


CLASSIFIER2_DATA_ROOT = Path(
    "/Users/aj/test-claude/projects/ai-for-quantum/src/data"
)
CLASS_DIRS = {
    "1x1":     "STO_ideal_1x1",
    "Tw(2x1)": "STO_ideal_Twinned2x1",
    "c(6x2)":  "STO_ideal_c6x2",
    "RT13":    "STO_ideal_RT13",
    "HTR":     "STO_ideal_HTR",
}
IMAGE_EXTS = {".png", ".bmp", ".jpg", ".jpeg", ".tif", ".tiff"}


def load_grayscale(path: Path, target_wh: tuple[int, int]) -> np.ndarray:
    """Load an image as grayscale uint8, resize to target_wh (width, height)."""
    img = Image.open(path).convert("L")
    img = img.resize(target_wh, Image.LANCZOS)
    return np.asarray(img, dtype=np.float32)


def load_class_corpus(
    target_wh: tuple[int, int],
) -> tuple[np.ndarray, list[str], list[str]]:
    """Load all 5 classes; return (X, labels, paths) where X is (N, H*W)."""
    rows: list[np.ndarray] = []
    labels: list[str] = []
    paths: list[str] = []
    for label, subdir in CLASS_DIRS.items():
        class_dir = CLASSIFIER2_DATA_ROOT / subdir
        if not class_dir.is_dir():
            print(f"  WARN: missing class dir {class_dir}")
            continue
        for p in sorted(class_dir.iterdir()):
            if p.suffix.lower() not in IMAGE_EXTS:
                continue
            arr = load_grayscale(p, target_wh)
            rows.append(arr.flatten())
            labels.append(label)
            paths.append(str(p))
    if not rows:
        raise SystemExit("No training images loaded.")
    X = np.stack(rows, axis=0)
    return X, labels, paths


def compute_basis(
    X: np.ndarray, k: int
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return (mean, basis, singular_values).

    mean: (D,) per-pixel mean over training images
    basis: (k, D) first k right-singular vectors
    singular_values: (k,) singular values for those components
    """
    mean = X.mean(axis=0)
    Xc = X - mean
    U, S, Vt = np.linalg.svd(Xc, full_matrices=False)
    return mean, Vt[:k], S[:k]


def fit_weights(
    image_flat: np.ndarray, mean: np.ndarray, basis: np.ndarray
) -> tuple[np.ndarray, float]:
    """Least-squares fit of weights such that image ≈ mean + sum(w_i * basis_i).

    Returns (weights, l2_recon_error) where the error is per-pixel mean
    absolute difference between original and reconstruction (in 0-255 units).
    """
    centered = image_flat - mean
    # basis is (k, D); we want w (k,) such that w @ basis ≈ centered
    weights, _, _, _ = np.linalg.lstsq(basis.T, centered, rcond=None)
    recon = mean + weights @ basis
    err = float(np.mean(np.abs(image_flat - recon)))
    return weights, err


def class_means(
    X: np.ndarray, labels: Sequence[str]
) -> dict[str, np.ndarray]:
    """Mean image per class label."""
    out: dict[str, np.ndarray] = {}
    labels_arr = np.array(labels)
    for cls in CLASS_DIRS:
        mask = labels_arr == cls
        if mask.any():
            out[cls] = X[mask].mean(axis=0)
    return out


def save_basis_grid(
    mean: np.ndarray, basis: np.ndarray, S: np.ndarray, shape_hw: tuple[int, int],
    out_path: Path,
) -> None:
    """Render mean + each basis vector as a single grid PNG."""
    H, W = shape_hw
    k = basis.shape[0]
    fig, axes = plt.subplots(1, k + 1, figsize=(2.5 * (k + 1), 3))
    axes[0].imshow(mean.reshape(H, W), cmap="gray")
    axes[0].set_title("mean")
    axes[0].axis("off")
    for i in range(k):
        ax = axes[i + 1]
        v = basis[i].reshape(H, W)
        # Symmetric color scale around 0 (basis vectors can be negative).
        vmax = float(np.max(np.abs(v)))
        ax.imshow(v, cmap="seismic", vmin=-vmax, vmax=vmax)
        ax.set_title(f"basis_{i + 1}\nσ={S[i]:.0f}")
        ax.axis("off")
    fig.tight_layout()
    fig.savefig(out_path, dpi=110, bbox_inches="tight")
    plt.close(fig)


def save_class_means(
    means: dict[str, np.ndarray], shape_hw: tuple[int, int], out_path: Path,
) -> None:
    """Render per-class mean images side by side."""
    H, W = shape_hw
    n = len(means)
    fig, axes = plt.subplots(1, n, figsize=(2.5 * n, 3))
    if n == 1:
        axes = [axes]
    for ax, (cls, m) in zip(axes, means.items()):
        ax.imshow(m.reshape(H, W), cmap="gray", vmin=0, vmax=255)
        ax.set_title(cls)
        ax.axis("off")
    fig.tight_layout()
    fig.savefig(out_path, dpi=110, bbox_inches="tight")
    plt.close(fig)


def save_reconstruction(
    image_flat: np.ndarray, weights: np.ndarray, mean: np.ndarray,
    basis: np.ndarray, shape_hw: tuple[int, int], title: str,
    out_path: Path,
) -> None:
    """Render original | reconstructed | weights bar chart side by side."""
    H, W = shape_hw
    recon = mean + weights @ basis
    fig, axes = plt.subplots(1, 3, figsize=(11, 3.5))
    axes[0].imshow(image_flat.reshape(H, W), cmap="gray", vmin=0, vmax=255)
    axes[0].set_title(f"original\n{title}")
    axes[0].axis("off")
    axes[1].imshow(np.clip(recon, 0, 255).reshape(H, W), cmap="gray", vmin=0, vmax=255)
    axes[1].set_title("reconstruction")
    axes[1].axis("off")
    axes[2].bar(range(1, len(weights) + 1), weights, color="steelblue")
    axes[2].axhline(0, color="k", lw=0.5)
    axes[2].set_xlabel("basis component")
    axes[2].set_ylabel("weight")
    axes[2].set_title("weights")
    axes[2].set_xticks(range(1, len(weights) + 1))
    fig.tight_layout()
    fig.savefig(out_path, dpi=110, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=Path("/tmp/equalizer_v1"),
                        help="Output directory (default /tmp/equalizer_v1)")
    parser.add_argument("--k", type=int, default=5,
                        help="Number of SVD components (default 5)")
    parser.add_argument("--resize", default="128x96",
                        help="Downsample WxH (default 128x96)")
    parser.add_argument("--extra", type=Path, default=None,
                        help="Optional dir of out-of-distribution frames")
    parser.add_argument("--extra-sample", type=int, default=8,
                        help="Number to sample from --extra (default 8)")
    parser.add_argument("--samples-per-class", type=int, default=2,
                        help="Per-class training reconstructions to render (default 2)")
    args = parser.parse_args()

    W, H = (int(x) for x in args.resize.lower().split("x"))
    target_wh = (W, H)
    shape_hw = (H, W)

    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "reconstructions").mkdir(exist_ok=True)

    print(f"Loading training corpus from {CLASSIFIER2_DATA_ROOT}")
    X, labels, paths = load_class_corpus(target_wh)
    print(f"  loaded {len(labels)} images, shape per row {X.shape[1]} pixels")

    print(f"\nComputing SVD (k={args.k}) ...")
    mean, basis, S = compute_basis(X, args.k)
    print(f"  top {args.k} singular values: {S}")

    print(f"\nWriting basis grid -> {args.out / 'basis_grid.png'}")
    save_basis_grid(mean, basis, S, shape_hw, args.out / "basis_grid.png")

    print(f"Writing class means -> {args.out / 'class_means.png'}")
    means = class_means(X, labels)
    save_class_means(means, shape_hw, args.out / "class_means.png")

    weights_log: dict[str, dict] = {}

    print(f"\nReconstructing {args.samples_per_class} sample(s) per class ...")
    labels_arr = np.array(labels)
    for cls in CLASS_DIRS:
        mask_idx = np.where(labels_arr == cls)[0]
        if mask_idx.size == 0:
            continue
        for idx in mask_idx[: args.samples_per_class]:
            row = X[idx]
            weights, err = fit_weights(row, mean, basis)
            stem = Path(paths[idx]).stem
            out_path = args.out / "reconstructions" / f"{cls}_{stem}_recon.png"
            save_reconstruction(
                row, weights, mean, basis, shape_hw,
                title=f"class={cls}", out_path=out_path,
            )
            weights_log[f"{cls}_{stem}"] = {
                "source": "training",
                "true_class": cls,
                "weights": weights.tolist(),
                "l2_error_mean_abs": err,
            }

    if args.extra is not None and args.extra.is_dir():
        print(f"\nScoring out-of-distribution frames from {args.extra} ...")
        extras = sorted(p for p in args.extra.iterdir()
                        if p.suffix.lower() in IMAGE_EXTS)
        if extras:
            step = max(1, len(extras) // args.extra_sample)
            sampled = extras[::step][: args.extra_sample]
            for p in sampled:
                arr = load_grayscale(p, target_wh)
                weights, err = fit_weights(arr.flatten(), mean, basis)
                out_path = (
                    args.out / "reconstructions" / f"extra_{p.stem}_recon.png"
                )
                save_reconstruction(
                    arr.flatten(), weights, mean, basis, shape_hw,
                    title=f"extra: {p.name}", out_path=out_path,
                )
                weights_log[f"extra_{p.stem}"] = {
                    "source": "extra",
                    "path": str(p),
                    "weights": weights.tolist(),
                    "l2_error_mean_abs": err,
                }

    weights_path = args.out / "weights.json"
    weights_path.write_text(json.dumps(weights_log, indent=2))
    print(f"\nWrote weights log -> {weights_path}")

    print(f"\nDone. Inspect:")
    print(f"  - {args.out / 'basis_grid.png'}     (the K eigen-RHEED images)")
    print(f"  - {args.out / 'class_means.png'}    (per-class mean — sanity check)")
    print(f"  - {args.out / 'reconstructions'}/   (original | reconstructed | weights)")


if __name__ == "__main__":
    main()
