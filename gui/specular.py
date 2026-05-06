"""Specular-spot detection and image-derived ROI for RHEED frames.

The auto-capture algorithm benefits from focusing change detection on the
diffraction-active region of the frame rather than the full image. This
module provides:

- ``detect_specular(frame_gray)``: locate the specular spot via column-sum
  argmax (X) and a vertical-strip row-sum argmax (Y around X). Image-derived
  — no electron energy, no incidence angle, no operator entry.
- ``image_derived_roi(frame_gray, x, y)``: expand outward from the detected
  specular until the column-sum / row-sum drops below a fraction of its
  peak. Produces an axis-aligned (slice, slice) suitable for ``frame[roi]``.

Design principle (May 2026): the GUI must operate without manual entry of
physical parameters. Specular detection and ROI bounds are derived from the
image content alone, so the algorithm transfers across cameras, exposures,
and beam settings without configuration.
"""
from __future__ import annotations

import numpy as np

try:
    from scipy.ndimage import gaussian_filter1d
except ImportError:  # pragma: no cover — fallback for environments without scipy
    def gaussian_filter1d(a: np.ndarray, sigma: float) -> np.ndarray:
        n = max(int(sigma * 4), 5)
        if n % 2 == 0:
            n += 1
        x = np.arange(n) - n // 2
        k = np.exp(-(x ** 2) / (2 * sigma ** 2))
        k = k / k.sum()
        return np.convolve(a, k, mode="same")


def detect_specular(
    frame_gray: np.ndarray,
    sigma: float = 10.0,
    strip_half_width: int = 20,
) -> tuple[int, int]:
    """Locate the specular spot in a grayscale RHEED frame.

    Two-stage detection:
    1. X via Gaussian-smoothed column-sum argmax. The specular column is the
       brightest vertical band averaged over rows.
    2. Y via row-sum argmax restricted to a vertical strip x ± strip_half_width.
       Restricting to the specular column is more robust than full-frame
       row-sum, which can be dominated by broad bright bands away from
       specular.

    Args:
        frame_gray: 2-D grayscale frame (any dtype, treated as float).
        sigma: Gaussian smoothing sigma in pixels for the column/row sums.
        strip_half_width: Y detection uses columns x ± this many pixels.

    Returns:
        ``(x, y)`` integer pixel coordinates of the detected specular spot.
    """
    if frame_gray.ndim != 2:
        raise ValueError(
            f"detect_specular expects 2D grayscale, got shape {frame_gray.shape}"
        )

    f = frame_gray.astype(np.float32)
    H, W = f.shape

    col_sum = f.sum(axis=0)
    col_sum_smooth = gaussian_filter1d(col_sum, sigma=sigma)
    x = int(col_sum_smooth.argmax())

    x_lo = max(0, x - strip_half_width)
    x_hi = min(W, x + strip_half_width + 1)
    strip_row_sum = f[:, x_lo:x_hi].sum(axis=1)
    strip_row_sum_smooth = gaussian_filter1d(strip_row_sum, sigma=sigma)
    y = int(strip_row_sum_smooth.argmax())

    return x, y


def image_derived_roi(
    frame_gray: np.ndarray,
    x: int,
    y: int,
    threshold_frac: float = 0.5,
    sigma: float = 10.0,
    min_half_width: int = 20,
    min_half_height: int = 10,
) -> tuple[slice, slice]:
    """Derive an axis-aligned ROI from image content centered on (x, y).

    The ROI rectangle's horizontal extent is the range of columns where the
    smoothed column-sum exceeds ``threshold_frac × peak_col_sum``. Within
    that horizontal band, the vertical extent is the range of rows where
    the smoothed row-sum (taken on the X-strip, not the whole frame) exceeds
    ``threshold_frac × peak_row_sum``. Falls back to fixed minimum
    half-extents if the threshold-derived region is too tight.

    The ROI adapts to the actual diffraction-pattern spread per frame —
    bright wide patterns yield larger ROIs, faint narrow patterns smaller —
    without requiring physical parameters or operator input.

    Args:
        frame_gray: 2-D grayscale frame.
        x, y: Specular position from ``detect_specular``.
        threshold_frac: ROI extends to where the smoothed sum drops below
            this fraction of its peak (0 < threshold_frac < 1).
        sigma: Smoothing sigma applied to column/row sums.
        min_half_width: Lower bound on the ROI half-width in pixels.
        min_half_height: Lower bound on the ROI half-height in pixels.

    Returns:
        ``(row_slice, col_slice)`` — usable directly as
        ``frame[row_slice, col_slice]``.
    """
    f = frame_gray.astype(np.float32)
    H, W = f.shape

    col_sum = gaussian_filter1d(f.sum(axis=0), sigma=sigma)

    col_threshold = threshold_frac * col_sum[x]
    x_lo = x
    while x_lo > 0 and col_sum[x_lo - 1] >= col_threshold:
        x_lo -= 1
    x_hi = x
    while x_hi < W - 1 and col_sum[x_hi + 1] >= col_threshold:
        x_hi += 1

    x_lo = min(x_lo, max(0, x - min_half_width))
    x_hi = max(x_hi, min(W - 1, x + min_half_width))

    strip = f[:, x_lo:x_hi + 1]
    row_sum = gaussian_filter1d(strip.sum(axis=1), sigma=sigma)

    row_threshold = threshold_frac * row_sum[y]
    y_lo = y
    while y_lo > 0 and row_sum[y_lo - 1] >= row_threshold:
        y_lo -= 1
    y_hi = y
    while y_hi < H - 1 and row_sum[y_hi + 1] >= row_threshold:
        y_hi += 1

    y_lo = min(y_lo, max(0, y - min_half_height))
    y_hi = max(y_hi, min(H - 1, y + min_half_height))

    return slice(y_lo, y_hi + 1), slice(x_lo, x_hi + 1)
