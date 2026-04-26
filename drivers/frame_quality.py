"""
RHEED frame quality gate — filters bad captures before saving or classifying.

Checks basic image statistics to detect:
  - All-black frames (gun off, capture failed)
  - All-white / saturated frames (exposure error)
  - Very low signal frames (dim RHEED at idle, nothing useful)
  - Wrong dimensions (corrupted capture)

Usage:
    from drivers.frame_quality import check_frame_quality

    result = check_frame_quality(frame)
    if result.passed:
        # Frame is worth saving / classifying
    else:
        print(f"Frame rejected: {result.reason}")
"""

from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class FrameQualityResult:
    """Result of a frame quality check."""

    passed: bool
    reason: str
    mean_intensity: float
    std_intensity: float
    max_intensity: int
    dimensions: tuple[int, ...]


# Default thresholds — tuned for kSA 400 screengrab (green-scale palette,
# 656x492 camera). These are intentionally conservative; a frame with
# mean intensity of 5 is essentially black.
DEFAULT_MIN_MEAN = 3.0       # reject if mean pixel intensity below this
DEFAULT_MAX_MEAN = 250.0     # reject if mean above this (saturated)
DEFAULT_MIN_STD = 1.0        # reject if std dev below this (uniform image)
DEFAULT_MIN_WIDTH = 100      # reject if width below this
DEFAULT_MIN_HEIGHT = 100     # reject if height below this


def check_frame_quality(
    frame: np.ndarray,
    min_mean: float = DEFAULT_MIN_MEAN,
    max_mean: float = DEFAULT_MAX_MEAN,
    min_std: float = DEFAULT_MIN_STD,
    min_width: int = DEFAULT_MIN_WIDTH,
    min_height: int = DEFAULT_MIN_HEIGHT,
    expected_shape: Optional[tuple[int, ...]] = None,
) -> FrameQualityResult:
    """Check whether a RHEED frame passes basic quality criteria.

    Parameters
    ----------
    frame : np.ndarray
        RGB uint8 image (H, W, 3) or grayscale (H, W).
    min_mean : float
        Minimum mean pixel intensity (0-255).
    max_mean : float
        Maximum mean pixel intensity (0-255).
    min_std : float
        Minimum standard deviation of pixel intensities.
    min_width, min_height : int
        Minimum acceptable frame dimensions.
    expected_shape : tuple, optional
        If provided, frame must match this shape exactly.

    Returns
    -------
    FrameQualityResult
        Dataclass with passed/reason/statistics.
    """
    dims = frame.shape

    # Dimension checks
    h = dims[0]
    w = dims[1] if len(dims) >= 2 else 0

    if h < min_height or w < min_width:
        return FrameQualityResult(
            passed=False,
            reason=f"Frame too small: {w}x{h} (min {min_width}x{min_height})",
            mean_intensity=0.0,
            std_intensity=0.0,
            max_intensity=0,
            dimensions=dims,
        )

    if expected_shape is not None and dims != expected_shape:
        return FrameQualityResult(
            passed=False,
            reason=f"Shape mismatch: got {dims}, expected {expected_shape}",
            mean_intensity=0.0,
            std_intensity=0.0,
            max_intensity=0,
            dimensions=dims,
        )

    # Convert to float for statistics
    pixels = frame.astype(np.float32)
    mean_val = float(np.mean(pixels))
    std_val = float(np.std(pixels))
    max_val = int(np.max(frame))

    # Intensity checks
    if mean_val < min_mean:
        return FrameQualityResult(
            passed=False,
            reason=f"Too dark: mean intensity {mean_val:.1f} < {min_mean}",
            mean_intensity=mean_val,
            std_intensity=std_val,
            max_intensity=max_val,
            dimensions=dims,
        )

    if mean_val > max_mean:
        return FrameQualityResult(
            passed=False,
            reason=f"Saturated: mean intensity {mean_val:.1f} > {max_mean}",
            mean_intensity=mean_val,
            std_intensity=std_val,
            max_intensity=max_val,
            dimensions=dims,
        )

    if std_val < min_std:
        return FrameQualityResult(
            passed=False,
            reason=f"Uniform image: std dev {std_val:.2f} < {min_std}",
            mean_intensity=mean_val,
            std_intensity=std_val,
            max_intensity=max_val,
            dimensions=dims,
        )

    return FrameQualityResult(
        passed=True,
        reason="OK",
        mean_intensity=mean_val,
        std_intensity=std_val,
        max_intensity=max_val,
        dimensions=dims,
    )
