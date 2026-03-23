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
    """

    def __init__(self, window_title: str = "Live Video"):
        self._window_title = window_title
        self._connected = False
        self._capture_method: Optional[str] = None

    @staticmethod
    def _find_window_by_substring(substring: str, exclude: str = "kSA 400 -") -> int:
        """Find the first window whose title contains *substring* but does
        NOT start with *exclude* (case-insensitive).

        This distinguishes the actual kSA Live Video child window from the
        main kSA application window.  The main window title is always
        ``"kSA 400 - {active pane title}"`` so it mirrors whatever child
        window is focused.  By excluding titles that start with "kSA 400 -"
        we reliably grab the small Live Video window.

        Returns the HWND (int) or 0 if not found.
        """
        import ctypes
        import ctypes.wintypes

        result = ctypes.c_void_p(0)
        target = substring.lower()
        exclude_lower = exclude.lower()

        @ctypes.WINFUNCTYPE(ctypes.c_bool, ctypes.wintypes.HWND, ctypes.wintypes.LPARAM)
        def enum_cb(hwnd, _lparam):
            length = ctypes.windll.user32.GetWindowTextLengthW(hwnd)
            if length == 0:
                return True
            buf = ctypes.create_unicode_buffer(length + 1)
            ctypes.windll.user32.GetWindowTextW(hwnd, buf, length + 1)
            title = buf.value.lower()
            if target in title and not title.startswith(exclude_lower):
                result.value = hwnd
                return False  # stop enumeration
            return True

        ctypes.windll.user32.EnumWindows(enum_cb, 0)
        return int(result.value) if result.value else 0

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

        if sys.platform == "win32":
            return self._grab_win32()
        else:
            return self._grab_cross_platform()

    def _grab_win32(self) -> np.ndarray:
        """Capture kSA window on Windows using win32gui + mss."""
        import ctypes
        import ctypes.wintypes
        import mss

        # Find window by partial title match.
        # kSA Live Video title format: "{CameraName} Live Video [live]"
        # e.g. "AVT Manta_G-033B (E0022060) 10 Live Video [live]"
        hwnd = self._find_window_by_substring(self._window_title)
        if not hwnd:
            raise RuntimeError(
                f"No window containing '{self._window_title}' found. "
                "Check that kSA 400 Live Video is open."
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
