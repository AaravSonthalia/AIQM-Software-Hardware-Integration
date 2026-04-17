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
# Measured on Bulbasaur at 1497x567 window size.
PRESSURE_BBOX = (8, 130, 112, 50)

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
        bbox: tuple[int, int, int, int] = PRESSURE_BBOX,
    ):
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

        text = ocr_crop(frame, self._bbox, OCR_CONFIG)
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
