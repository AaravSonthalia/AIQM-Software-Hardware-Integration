"""
Shared OCR helpers — window screen capture and tesseract wrapping.

Tesseract binary path is auto-detected from the standard Windows install
locations; pytesseract does not reliably discover it from PATH if the
installer did not add it (common case on Bulbasaur).
"""

import ctypes
import ctypes.wintypes
import os
import sys
from datetime import datetime
from pathlib import Path
from typing import Optional

import numpy as np


TESSERACT_PATHS = [
    r"C:\Program Files\Tesseract-OCR\tesseract.exe",
    r"C:\Program Files (x86)\Tesseract-OCR\tesseract.exe",
]

# Set AIQM_OCR_DEBUG=1 to print raw OCR output to stderr on every read AND
# save each crop PNG to logs/ocr_debug/. Use for diagnosing wrong/blank
# pressure or V/I readings — the saved crops give a per-call corpus you
# can replay against alternate preprocessing pipelines / PSM modes.
DEBUG = os.environ.get("AIQM_OCR_DEBUG", "").strip().lower() in ("1", "true", "yes")
DEBUG_CROP_DIR = Path("logs/ocr_debug")


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
    label: str = "",
) -> str:
    """Crop image to bbox=(x, y, w, h) and run tesseract on the crop.

    Returns stripped OCR output, or empty string on failure.

    If AIQM_OCR_DEBUG=1 is set in the environment, prints the raw OCR
    result to stderr tagged with `label` (e.g., "mistral", "evap"), so
    the caller doesn't need to wire its own debug path.
    """
    import pytesseract
    from PIL import Image

    configure_tesseract()
    x, y, w, h = bbox
    crop = image[y:y + h, x:x + w]
    if crop.size == 0:
        if DEBUG:
            print(f"[OCR {label}] empty crop for bbox={bbox}", file=sys.stderr, flush=True)
        return ""
    pil = Image.fromarray(crop)

    if DEBUG:
        _save_debug_crop(pil, label)

    try:
        text = pytesseract.image_to_string(pil, config=config).strip()
        if DEBUG:
            print(f"[OCR {label}] {text!r}", file=sys.stderr, flush=True)
        return text
    except Exception as e:
        if DEBUG:
            print(f"[OCR {label}] EXCEPTION: {e}", file=sys.stderr, flush=True)
        return ""


def _save_debug_crop(pil_image, label: str) -> None:
    """Save a crop to logs/ocr_debug/ for offline analysis.

    Filename: ``{label}_{YYYYMMDD_HHMMSS_mmm}.png``. Failures are logged to
    stderr but otherwise swallowed — diagnostic saves must not break the
    OCR read path.
    """
    try:
        DEBUG_CROP_DIR.mkdir(parents=True, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        path = DEBUG_CROP_DIR / f"{label or 'ocr'}_{ts}.png"
        pil_image.save(str(path))
    except Exception as e:
        print(
            f"[OCR {label}] failed to save debug crop: {e}",
            file=sys.stderr,
            flush=True,
        )
