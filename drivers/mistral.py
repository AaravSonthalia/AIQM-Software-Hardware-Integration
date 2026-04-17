"""
MISTRAL GUI driver — OCR readout of cell V/I setpoints and readback.

MistralGui is Scienta Omicron cell-control software. Its V/I display is a
canvas-painted region with no UIA-navigable controls, so we screengrab
and OCR the labeled region. Both setpoint ('Set') and measured ('Actual')
values are extracted per quantity.

The window must be visible on screen (not occluded) during reads, same
constraint as the kSA RHEED screengrab.
"""

import re
from typing import Optional

from drivers.ocr import capture_window, find_window, ocr_crop


# V/I block crop (x, y, w, h), relative to the MistralGui window top-left.
# Measured on Bulbasaur at 1793x1039 window size.
VI_BBOX = (970, 90, 585, 80)

# PSM 6 = uniform block of text. Whitelist trims OCR search space to the
# actual characters present in the V/I display.
OCR_CONFIG = (
    "--psm 6 "
    "-c tessedit_char_whitelist=SetActualVoltageCurrent:0123456789.- VA"
)

V_RANGE = (-1.0, 500.0)
I_RANGE = (-1.0, 100.0)

_REGEX = {
    "v_set":    re.compile(r"Set\s*Voltage\s*:\s*([-\d.]+)", re.IGNORECASE),
    "i_set":    re.compile(r"Set\s*Current\s*:\s*([-\d.]+)", re.IGNORECASE),
    "v_actual": re.compile(r"Actual\s*Voltage\s*:\s*([-\d.]+)", re.IGNORECASE),
    "i_actual": re.compile(r"Actual\s*Current\s*:\s*([-\d.]+)", re.IGNORECASE),
}


def _in_range(val: float, lo: float, hi: float) -> Optional[float]:
    return val if lo <= val <= hi else None


class MistralGui:
    """Scrapes V/I setpoint + readback from the MistralGui cell window.

    read() always returns a dict with the four expected keys; individual
    values are float or None (OCR failure, parse failure, out-of-range,
    or window not found). Callers should treat None as 'unavailable'.
    """

    def __init__(
        self,
        window_title_substring: str = "MistralGui",
        bbox: tuple[int, int, int, int] = VI_BBOX,
    ):
        self._substring = window_title_substring
        self._bbox = bbox
        self._hwnd = 0
        self._connected = False

    def connect(self) -> None:
        self._hwnd = find_window(self._substring)
        if not self._hwnd:
            raise RuntimeError(
                f"MistralGui window not found (substring "
                f"'{self._substring}'). Is MISTRAL running?"
            )
        self._connected = True

    def read(self) -> dict[str, Optional[float]]:
        result: dict[str, Optional[float]] = {
            "v_set": None, "v_actual": None,
            "i_set": None, "i_actual": None,
        }
        if not self._connected or not self._hwnd:
            return result

        try:
            frame = capture_window(self._hwnd)
        except Exception:
            return result

        text = ocr_crop(frame, self._bbox, OCR_CONFIG)
        if not text:
            return result

        for key, rx in _REGEX.items():
            m = rx.search(text)
            if not m:
                continue
            try:
                v = float(m.group(1))
            except ValueError:
                continue
            lo, hi = V_RANGE if key.startswith("v_") else I_RANGE
            result[key] = _in_range(v, lo, hi)

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
