"""
Shared OCR helpers — window screen capture and tesseract wrapping.

Tesseract binary path is auto-detected from the standard Windows install
locations; pytesseract does not reliably discover it from PATH if the
installer did not add it (common case on Bulbasaur).
"""

import ctypes
import ctypes.wintypes
from pathlib import Path
from typing import Optional

import numpy as np


TESSERACT_PATHS = [
    r"C:\Program Files\Tesseract-OCR\tesseract.exe",
    r"C:\Program Files (x86)\Tesseract-OCR\tesseract.exe",
]


def configure_tesseract() -> Optional[str]:
    """Point pytesseract at the tesseract binary. Returns the path used,
    or None if no install was found (pytesseract will fall back to PATH)."""
    import pytesseract
    for p in TESSERACT_PATHS:
        if Path(p).exists():
            pytesseract.pytesseract.tesseract_cmd = p
            return p
    return None


def find_window(substring: str) -> int:
    """Return hwnd of the first visible top-level window whose title
    contains substring (case-insensitive), or 0 if none match."""
    user32 = ctypes.windll.user32
    search = substring.lower()
    found = ctypes.c_void_p(0)

    def _title(hwnd: int) -> str:
        n = user32.GetWindowTextLengthW(hwnd)
        if not n:
            return ""
        buf = ctypes.create_unicode_buffer(n + 1)
        user32.GetWindowTextW(hwnd, buf, n + 1)
        return buf.value

    @ctypes.WINFUNCTYPE(ctypes.c_bool, ctypes.wintypes.HWND, ctypes.wintypes.LPARAM)
    def _cb(hwnd, _lp):
        if not user32.IsWindowVisible(hwnd):
            return True
        t = _title(hwnd)
        if t and search in t.lower():
            found.value = hwnd
            return False
        return True

    user32.EnumWindows(_cb, 0)
    return int(found.value or 0)


def capture_window(hwnd: int) -> np.ndarray:
    """Screen-capture the window's on-screen rect as RGB uint8 (H, W, 3).

    Uses mss framebuffer grab — the window must be visible (not occluded).
    If another window is on top, that window's pixels will be returned.
    """
    import mss

    rect = ctypes.wintypes.RECT()
    ctypes.windll.user32.GetWindowRect(hwnd, ctypes.byref(rect))
    monitor = {
        "left": rect.left,
        "top": rect.top,
        "width": rect.right - rect.left,
        "height": rect.bottom - rect.top,
    }
    with mss.mss() as sct:
        img = np.array(sct.grab(monitor))[:, :, :3]
        return img[:, :, ::-1]


def ocr_crop(
    image: np.ndarray,
    bbox: tuple[int, int, int, int],
    config: str = "",
) -> str:
    """Crop image to bbox=(x, y, w, h) and run tesseract on the crop.

    Returns stripped OCR output, or empty string on failure.
    """
    import pytesseract
    from PIL import Image

    configure_tesseract()
    x, y, w, h = bbox
    crop = image[y:y + h, x:x + w]
    if crop.size == 0:
        return ""
    pil = Image.fromarray(crop)
    try:
        return pytesseract.image_to_string(pil, config=config).strip()
    except Exception:
        return ""
