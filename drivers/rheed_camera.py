"""
RHEED camera drivers — abstract interface + concrete implementations.

Two acquisition modes:
  1. VmbCamera: Direct vmbpy access to Allied Vision Manta G-033B (GigE)
  2. ScreenGrabCamera: Captures kSA 400 window via screen grab (primary for ML)

The ScreenGrabCamera mode is preferred because Classifier2 was trained on kSA
false-color screenshots — scraping gives the exact image format the classifier
expects without needing to discover/apply the kSA LUT.
"""

from abc import ABC, abstractmethod
from typing import Optional

import numpy as np


class RheedCamera(ABC):
    """Abstract RHEED frame source."""

    @abstractmethod
    def connect(self) -> None:
        """Open connection to the camera / capture source."""

    @abstractmethod
    def read_frame(self) -> np.ndarray:
        """Return the latest frame as an RGB uint8 numpy array (H, W, 3)."""

    @abstractmethod
    def disconnect(self) -> None:
        """Release the camera / capture source."""

    @property
    @abstractmethod
    def connected(self) -> bool:
        """Whether the source is currently available."""


class VmbCamera(RheedCamera):
    """
    Direct access to Allied Vision camera via vmbpy SDK.

    Requires exclusive camera lock — cannot run simultaneously with kSA 400.
    Frames are monochrome; intensity is placed in the green channel to match
    the convention used by the existing RHEED GUI code.
    """

    def __init__(self, camera_index: int = 0, trigger_hz: float = 1.0):
        self._camera_index = camera_index
        self._trigger_hz = trigger_hz
        self._vmb = None
        self._cam = None
        self._connected = False
        self._latest_frame: Optional[np.ndarray] = None

    def connect(self) -> None:
        try:
            from vmbpy import VmbSystem
        except ImportError:
            raise ImportError(
                "vmbpy not installed. Install the Allied Vision SDK or use "
                "ScreenGrabCamera for screen-scrape mode."
            )

        self._vmb = VmbSystem.get_instance()
        self._vmb.__enter__()

        cams = self._vmb.get_all_cameras()
        if not cams:
            self._vmb.__exit__(None, None, None)
            raise RuntimeError("No Allied Vision cameras found.")

        self._cam = cams[self._camera_index]
        self._cam.__enter__()

        # Configure software trigger for controlled frame rate
        self._cam.TriggerSource.set("Software")
        self._cam.TriggerMode.set("On")
        self._cam.AcquisitionMode.set("Continuous")
        self._connected = True

    def read_frame(self) -> np.ndarray:
        if not self._connected or self._cam is None:
            raise RuntimeError("Camera not connected.")

        self._cam.TriggerSoftware.run()
        frame = self._cam.get_frame(timeout_ms=2000)
        img = frame.as_numpy_ndarray().squeeze()

        # Normalize to uint8
        if img.dtype != np.uint8:
            img = ((img.astype(np.float32) / img.max()) * 255).astype(np.uint8)

        # Build RGB with intensity in green channel (existing convention)
        if img.ndim == 2:
            rgb = np.zeros((*img.shape, 3), dtype=np.uint8)
            rgb[:, :, 1] = img  # green channel
        else:
            rgb = img

        return rgb

    def disconnect(self) -> None:
        if self._cam is not None:
            try:
                self._cam.__exit__(None, None, None)
            except Exception:
                pass
            self._cam = None

        if self._vmb is not None:
            try:
                self._vmb.__exit__(None, None, None)
            except Exception:
                pass
            self._vmb = None

        self._connected = False

    @property
    def connected(self) -> bool:
        return self._connected


class ScreenGrabCamera(RheedCamera):
    """
    Captures frames from the kSA 400 window via screen grab.

    This is the primary acquisition mode for ML classification because
    Classifier2 was trained on kSA colored screenshots (false-color LUT).

    On the MBE PC: uses win32gui/mss to capture the window directly.
    Over remote desktop: uses OpenCV to grab from a VNC/RDP session.

    Falls back to full-screen capture with window title search.

    Parameters
    ----------
    window_title : str
        Substring to search for in window titles (default "Live Video").
    crop_chrome : bool
        If True, crop kSA window chrome (title bar + menu + toolbar above,
        status bar below) from the captured frame so the detector only
        sees the actual RHEED image area. Default True.
    chrome_top_px : int
        Pixels to crop from the top. Default 75 — measured from the
        kSA 400 - AVT Manta_G live-video window screenshot 2026-04-25
        (title=29 + menu=22 + toolbar=24 = 75; covers all chrome above
        the image content). Set crop_chrome=False if the value drifts
        on Bulbasaur (different DPI / theme).
    chrome_bottom_px : int
        Pixels to crop from the bottom. Default 30 — covers the kSA
        status bar ("Exposure: ... NUM") plus the bottom window border.
    """

    def __init__(
        self,
        window_title: str = "Live Video",
        crop_chrome: bool = True,
        chrome_top_px: int = 75,
        chrome_bottom_px: int = 30,
    ):
        self._window_title = window_title
        self._connected = False
        self._capture_method: Optional[str] = None
        self._crop_chrome = crop_chrome
        self._chrome_top_px = chrome_top_px
        self._chrome_bottom_px = chrome_bottom_px

    @staticmethod
    def _find_live_video_window(search_term: str = "Live Video") -> int:
        """Find the kSA Live Video window handle.

        Search strategy (in priority order):
        1. Top-level window containing *search_term* but NOT starting with
           "kSA 400" — this is the detached Live Video window (when the kSA
           option "Keep Live Video inside application" is unchecked).
        2. Child window of the main kSA 400 frame containing *search_term*
           — this is the MDI child pane (default kSA layout).
        3. The main kSA 400 window itself as a fallback.

        Returns the HWND (int) or 0 if kSA 400 is not running.
        """
        import ctypes
        import ctypes.wintypes

        search_lower = search_term.lower()
        user32 = ctypes.windll.user32

        def _get_title(hwnd):
            length = user32.GetWindowTextLengthW(hwnd)
            if length == 0:
                return ""
            buf = ctypes.create_unicode_buffer(length + 1)
            user32.GetWindowTextW(hwnd, buf, length + 1)
            return buf.value

        # --- Step 1: scan ALL top-level windows ---
        main_hwnd = ctypes.c_void_p(0)
        detached_hwnd = ctypes.c_void_p(0)

        @ctypes.WINFUNCTYPE(ctypes.c_bool, ctypes.wintypes.HWND, ctypes.wintypes.LPARAM)
        def _enum_toplevel(hwnd, _lp):
            title = _get_title(hwnd).lower()
            if not title:
                return True
            # Detached Live Video window (priority 1)
            if search_lower in title and not title.startswith("ksa 400"):
                detached_hwnd.value = hwnd
                return False  # found best match, stop
            # Main kSA 400 frame
            if title.startswith("ksa 400") and not main_hwnd.value:
                main_hwnd.value = hwnd
            return True

        user32.EnumWindows(_enum_toplevel, 0)

        # Priority 1: detached top-level Live Video window
        if detached_hwnd.value:
            return int(detached_hwnd.value)

        if not main_hwnd.value:
            return 0

        # --- Step 2: search children of main kSA window ---
        child_hwnd = ctypes.c_void_p(0)

        @ctypes.WINFUNCTYPE(ctypes.c_bool, ctypes.wintypes.HWND, ctypes.wintypes.LPARAM)
        def _enum_children(hwnd, _lp):
            title = _get_title(hwnd).lower()
            if search_lower in title:
                child_hwnd.value = hwnd
                return False
            return True

        user32.EnumChildWindows(int(main_hwnd.value), _enum_children, 0)

        if child_hwnd.value:
            return int(child_hwnd.value)

        # Priority 3: fallback to main window
        return int(main_hwnd.value)

    def connect(self) -> None:
        # Try platform-specific window capture
        import sys

        if sys.platform == "win32":
            self._setup_win32()
        else:
            self._setup_cross_platform()

        self._connected = True

    def _setup_win32(self) -> None:
        """Set up win32-based window capture (runs on MBE PC)."""
        try:
            import mss  # noqa: F401
            self._capture_method = "mss"
        except ImportError:
            raise ImportError("mss required for screen capture: pip install mss")

    def _setup_cross_platform(self) -> None:
        """Set up cross-platform capture (e.g., over VNC from Mac)."""
        try:
            import mss  # noqa: F401
            self._capture_method = "mss"
        except ImportError:
            raise ImportError("mss required for screen capture: pip install mss")

    def read_frame(self) -> np.ndarray:
        if not self._connected:
            raise RuntimeError("Screen grab not connected.")

        import sys

        frame = self._grab_win32() if sys.platform == "win32" else self._grab_cross_platform()
        return self._crop_chrome_pixels(frame)

    def _crop_chrome_pixels(self, frame: np.ndarray) -> np.ndarray:
        """Crop title/menu/toolbar above and status bar below the RHEED image.

        Defensive: if the configured crop bounds are invalid for this frame
        (e.g., window much smaller than expected), returns the frame
        unchanged rather than producing an empty array.
        """
        if not self._crop_chrome:
            return frame
        h = frame.shape[0]
        top = max(0, self._chrome_top_px)
        bottom = h - max(0, self._chrome_bottom_px)
        if bottom <= top:
            return frame
        return frame[top:bottom, :]

    def _grab_win32(self) -> np.ndarray:
        """Capture kSA window on Windows using win32gui + mss."""
        import ctypes
        import ctypes.wintypes
        import mss

        # Find the Live Video child window inside kSA 400's MDI frame.
        hwnd = self._find_live_video_window(self._window_title)
        if not hwnd:
            raise RuntimeError(
                "kSA 400 not found. Check that kSA 400 is running "
                "with the Live Video window open."
            )

        # Get window rectangle
        rect = ctypes.wintypes.RECT()
        ctypes.windll.user32.GetWindowRect(hwnd, ctypes.byref(rect))

        monitor = {
            "left": rect.left,
            "top": rect.top,
            "width": rect.right - rect.left,
            "height": rect.bottom - rect.top,
        }

        with mss.mss() as sct:
            screenshot = sct.grab(monitor)
            img = np.array(screenshot)[:, :, :3]  # Drop alpha, keep BGR
            return img[:, :, ::-1]  # BGR -> RGB

    def _grab_cross_platform(self) -> np.ndarray:
        """Capture via mss full-screen (for remote desktop scenarios)."""
        import mss

        with mss.mss() as sct:
            # Grab primary monitor
            monitor = sct.monitors[1]
            screenshot = sct.grab(monitor)
            img = np.array(screenshot)[:, :, :3]
            return img[:, :, ::-1]  # BGR -> RGB

    def disconnect(self) -> None:
        self._connected = False

    @property
    def connected(self) -> bool:
        return self._connected


class DummyCamera(RheedCamera):
    """Test camera that returns synthetic frames. Useful for GUI development."""

    def __init__(self, width: int = 656, height: int = 492):
        self._width = width
        self._height = height
        self._connected = False
        self._frame_count = 0

    def connect(self) -> None:
        self._connected = True
        self._frame_count = 0

    def read_frame(self) -> np.ndarray:
        if not self._connected:
            raise RuntimeError("Dummy camera not connected.")

        # Generate a test pattern with a moving bright spot
        frame = np.zeros((self._height, self._width, 3), dtype=np.uint8)

        # Moving Gaussian spot to simulate RHEED oscillations
        cx = self._width // 2
        cy = self._height // 2
        intensity = int(128 + 100 * np.sin(self._frame_count * 0.1))

        y, x = np.ogrid[:self._height, :self._width]
        r2 = (x - cx) ** 2 + (y - cy) ** 2
        spot = np.clip(intensity * np.exp(-r2 / (2 * 50**2)), 0, 255).astype(np.uint8)
        frame[:, :, 1] = spot  # green channel

        self._frame_count += 1
        return frame

    def disconnect(self) -> None:
        self._connected = False

    @property
    def connected(self) -> bool:
        return self._connected
