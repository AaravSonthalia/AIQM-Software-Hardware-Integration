#!/usr/bin/env python3
"""Build the Equalizer class-means cache for machines without training data.

Runs on a machine where CLASSIFIER2_DATA_ROOT exists (typically the Mac dev
environment), computes the per-class mean images, and writes a small npz
file to data/equalizer_class_means.npz. Commit the resulting file so other
machines (Bulbasaur) can load the cache directly.

Usage:
    ./.venv/bin/python scripts/build_equalizer_cache.py
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from equalizer_ui import (
    CLASS_DIRS, CLASSIFIER2_DATA_ROOT, IMAGE_EXTS, MEANS_CACHE,
    PROCESS_WH, SAFE_KEYS, load_grayscale,
)


def main() -> None:
    print(f"Reading training data from: {CLASSIFIER2_DATA_ROOT}")
    if not CLASSIFIER2_DATA_ROOT.is_dir():
        print(f"ERROR: training data root missing", file=sys.stderr)
        sys.exit(1)

    means: dict[str, np.ndarray] = {}
    for label, subdir in CLASS_DIRS.items():
        class_dir = CLASSIFIER2_DATA_ROOT / subdir
        if not class_dir.is_dir():
            print(f"  MISSING: {class_dir}", file=sys.stderr)
            continue
        rows: list[np.ndarray] = []
        for p in sorted(class_dir.iterdir()):
            if p.suffix.lower() in IMAGE_EXTS:
                rows.append(load_grayscale(p, PROCESS_WH))
        if rows:
            means[label] = np.stack(rows).mean(axis=0)
            print(f"  OK    {label:<10} {len(rows)} images")

    if not means:
        print("ERROR: no class means computed", file=sys.stderr)
        sys.exit(1)

    MEANS_CACHE.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        MEANS_CACHE,
        **{SAFE_KEYS[k]: v for k, v in means.items()},
    )
    size_kb = MEANS_CACHE.stat().st_size / 1024
    print(f"\nWrote {len(means)} class means to {MEANS_CACHE}")
    print(f"Size: {size_kb:.1f} KB at {PROCESS_WH[0]}x{PROCESS_WH[1]} resolution")


if __name__ == "__main__":
    main()
