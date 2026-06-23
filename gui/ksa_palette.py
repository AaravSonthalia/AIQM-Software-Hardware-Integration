"""kSA 400 Black-Green-White (BGW) palette — empirical reproduction.

The kSA 400 software applies a false-color palette to monochrome RHEED
data at display time. Per the kSA 400 User Guide page 73:

    "The palette lets you colorize the video input. Note that this
    feature only changes the display, and does not alter the image
    itself."

    Note 1: "Palette is available only for images taken with
    black-and-white cameras."

    Note 2: "Applying a palette to an image and then exporting that
    image means that the palette is permanently applied and cannot
    be undone."

The user guide doesn't enumerate specific palette LUTs (it just notes
"There are a large number of palettes available"). The exact palette
the lab uses was reverse-engineered from training BMPs (mode P with
embedded palette). Verified byte-identical across 200 sampled training
files spanning multiple reconstruction folders — see
`ksa_palette_classifier_input.md` in memory for the audit.

The structure is two linear ramps with a saturation knee at index 128:

    indices   0–127:  Black → Pure Green
                      G ramps 0 → 255 linearly
                      R = B = 0
    indices 128–255:  Pure Green → White
                      G saturated at 255
                      R = B ramp 0 → 255 linearly

This module exists because direct-camera mode (Vimba SDK) gives us raw
monochrome frames, and growers are accustomed to kSA's green rendering.
The classifier path doesn't need the palette — Classifier2 strips back
to grayscale via PIL's .convert('L') before inference. The palette is
purely for human visualization, matching kSA's design.
"""
from __future__ import annotations

import numpy as np


def _build_bgw_lut() -> np.ndarray:
    """Build the 256-entry Black-Green-White lookup table.

    Algorithmic reconstruction. Verified to byte-match the palette
    embedded in our lab's training BMPs (max per-channel difference: 0
    across all 256 entries).
    """
    lut = np.zeros((256, 3), dtype=np.uint8)
    # Round half-away-from-zero — matches kSA's apparent rounding
    # convention for the linear-ramp endpoints.
    def _round(x: float) -> int:
        return int(np.floor(x + 0.5))

    for i in range(128):
        # Black → Green: G ramps 0 → 255 over 128 steps; R = B = 0.
        lut[i] = [0, _round(255 * i / 127), 0]
    for i in range(128, 256):
        # Green → White: G saturated at 255; R and B ramp 0 → 255.
        delta = _round(255 * (i - 128) / 127)
        lut[i] = [delta, 255, delta]
    return lut


# 256-entry RGB lookup table. Indexable by uint8 grayscale to produce
# (..., 3) RGB output via numpy's fancy indexing.
KSA_BGW_PALETTE = _build_bgw_lut()


def apply_palette(grayscale: np.ndarray) -> np.ndarray:
    """Apply the kSA BGW palette to a 2-D grayscale image.

    The intensity scaling step before lookup mirrors kSA's "Maximized"
    display option (User Guide page 73), which scales data so that the
    brightest pixel becomes 255 and the darkest becomes 0. Without this,
    a 12-bit camera frame using only the lower half of its range would
    render dark even though the same scene through kSA looks vivid.

    Parameters
    ----------
    grayscale : np.ndarray
        2-D array of shape (H, W). uint8 inputs are looked up directly;
        other dtypes get min-max scaled to 0–255 first.

    Returns
    -------
    np.ndarray
        3-D array of shape (H, W, 3), uint8, RGB. Suitable for
        QImage.Format_RGB888 or matplotlib display.
    """
    if grayscale.ndim != 2:
        raise ValueError(
            f"apply_palette expects 2-D grayscale, got shape {grayscale.shape}"
        )

    if grayscale.dtype != np.uint8:
        gmin = float(grayscale.min())
        gmax = float(grayscale.max())
        if gmax > gmin:
            scaled = (grayscale.astype(np.float32) - gmin) * 255.0 / (gmax - gmin)
            grayscale = scaled.clip(0, 255).astype(np.uint8)
        else:
            grayscale = np.zeros_like(grayscale, dtype=np.uint8)

    # Fancy indexing — KSA_BGW_PALETTE has shape (256, 3); indexing with
    # a (H, W) uint8 array produces a (H, W, 3) RGB array in one
    # vectorized step. ~50× faster than a Python loop on a 656×492 frame.
    return KSA_BGW_PALETTE[grayscale]


def apply_palette_fixed_range(
    grayscale: np.ndarray, bit_depth: int = 12,
) -> np.ndarray:
    """Apply the BGW palette with fixed bit-depth scaling (no auto-stretch).

    Use this variant when you want frame-to-frame visual consistency:
    a steady scene shouldn't shift in apparent brightness just because
    one frame happens to have a slightly higher max pixel. This matches
    `VmbCamera`'s fixed-range normalization (commit 73f1c13).

    Parameters
    ----------
    grayscale : np.ndarray
        2-D array of any integer dtype.
    bit_depth : int
        Source bit depth (default 12 for the Manta G-033B's ADC).

    Returns
    -------
    np.ndarray
        3-D RGB array, shape (H, W, 3), uint8.
    """
    if grayscale.ndim != 2:
        raise ValueError(
            f"apply_palette_fixed_range expects 2-D, got shape {grayscale.shape}"
        )
    max_value = (1 << bit_depth) - 1
    if grayscale.dtype != np.uint8:
        scaled = (grayscale.astype(np.float32) / max_value * 255.0).clip(0, 255)
        grayscale = scaled.astype(np.uint8)
    return KSA_BGW_PALETTE[grayscale]
