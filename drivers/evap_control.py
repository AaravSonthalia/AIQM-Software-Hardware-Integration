"""
Evaporation Control driver — OCR readout of MBE chamber pressure.

'Evaporation control (Logged in: master)' is a LabVIEW-based chamber
monitor. Canvas-drawn, no UIA controls — we screengrab + OCR the
MBE > Pressure cell (chamber ion gauge, in mbar).

The window must be visible on screen during reads.
"""

import re
from typing import Optional

from drivers.ocr import capture_window, find_window, ocr_crop


# Crop for the 'Pressure' label + value in the MBE column, (x, y, w, h).
# Measured on Bulbasaur at 1497x567 window size. x start is deliberately
# set past the LabVIEW panel's 3D-inset bezel, which occupies x=9..16
# (two stacked 4-px gradients — outer and inner bevel). The first green
# pixel of the '2' digit is at x=19-20, so x=18 leaves ~2 px of clean
# black interior before the text. Earlier shifts (x=8, x=15) still
# captured bezel pixels that tesseract read as a phantom leading '1'.
PRESSURE_BBOX = (18, 130, 100, 50)
CALIBRATION_WINDOW_SIZE = (1497, 567)


def scale_bbox_to_window(
    calibration_bbox: tuple[int, int, int, int],
    calibration_size: tuple[int, int],
    current_size: tuple[int, int],
) -> tuple[int, int, int, int]:
    """Scale a calibrated bbox proportionally to the current window size.

    Same shape-uniform-stretch assumption as the matching helper in
    drivers.mistral. If the LabVIEW panel reflows on resize (the Pressure
    cell moves to a different relative position), this won't help — that
    needs anchor-based detection (template-match the label, crop relative
    to it), tracked as a future improvement.
    """
    cur_w, cur_h = current_size
    cal_w, cal_h = calibration_size
    x, y, w, h = calibration_bbox
    sx = cur_w / cal_w
    sy = cur_h / cal_h
    return (round(x * sx), round(y * sy), round(w * sx), round(h * sy))

OCR_CONFIG = (
    "--psm 6 "
    "-c tessedit_char_whitelist=Pressure0123456789.eE-+ mbar"
)

PRESSURE_REGEX = re.compile(r"([0-9]+(?:\.[0-9]+)?)\s*[eE]\s*([+-]?\d+)")

# UHV chamber plausibility range (mbar)
PRESSURE_RANGE_MBAR = (1e-12, 1e-2)


class EvapControl:
    """Scrapes MBE chamber pressure from the Evap Control LabVIEW window."""

    def __init__(
        self,
        window_title_substring: str = "Evaporation control",
        bbox: Optional[tuple[int, int, int, int]] = None,
    ):
        """``bbox`` overrides the auto-scaled crop. Default ``None`` =
        scale ``PRESSURE_BBOX`` proportionally to the captured window
        size at each read."""
        self._substring = window_title_substring
        self._bbox = bbox
        self._hwnd = 0
        self._connected = False

    def connect(self) -> None:
        self._hwnd = find_window(self._substring)
        if not self._hwnd:
            raise RuntimeError(
                f"Evap Control window not found (substring "
                f"'{self._substring}'). Is the window open?"
            )
        self._connected = True

    def read(self) -> dict[str, Optional[float]]:
        result: dict[str, Optional[float]] = {"chamber_pressure_mbar": None}
        if not self._connected or not self._hwnd:
            return result

        try:
            frame = capture_window(self._hwnd)
        except Exception:
            return result

        if self._bbox is not None:
            bbox = self._bbox
        else:
            h, w = frame.shape[:2]
            bbox = scale_bbox_to_window(
                PRESSURE_BBOX, CALIBRATION_WINDOW_SIZE, (w, h),
            )
        text = ocr_crop(frame, bbox, OCR_CONFIG, label="evap")
        if not text:
            return result

        m = PRESSURE_REGEX.search(text)
        if not m:
            return result

        try:
            val = float(m.group(1)) * (10 ** int(m.group(2)))
        except (ValueError, OverflowError):
            return result

        lo, hi = PRESSURE_RANGE_MBAR
        if not (lo <= val <= hi):
            return result

        result["chamber_pressure_mbar"] = val
        return result

    def disconnect(self) -> None:
        self._hwnd = 0
        self._connected = False

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def hwnd(self) -> int:
        return self._hwnd


class DummyEvapControl:
    """Synthetic driver for offline GUI development — slowly varying pressure."""

    def __init__(self, base_mbar: float = 1e-9):
        self._base = base_mbar
        self._connected = False
        self._read_count = 0

    def connect(self) -> None:
        self._connected = True
        self._read_count = 0

    def read(self) -> dict[str, Optional[float]]:
        if not self._connected:
            return {"chamber_pressure_mbar": None}

        import math
        import random
        self._read_count += 1
        drift = 0.1 * math.sin(self._read_count * 0.02)
        return {"chamber_pressure_mbar": self._base * (1.0 + drift + random.gauss(0, 0.01))}

    def disconnect(self) -> None:
        self._connected = False

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def hwnd(self) -> int:
        return 0
