"""Window arrangement for OMBE Growth Monitor sessions on Bulbasaur.

Arranges the external windows that AIQM reads from (kSA, MISTRAL,
EvapControl, EX04763 pyrometer) into predictable positions. Stable window
geometry matters because the screengrab + OCR drivers
(`drivers/mistral.py`, `drivers/evap_control.py`, `drivers/ocr.py`) crop
pixels relative to each window's rect. A dragged or resized window
silently breaks OCR for that channel.

Defaults below are estimated from the Apr 29 screenshot
(LIVE-TEST_04292026A) — dual 1920x1080 monitors on Bulbasaur, left
monitor x=0..1919, right monitor x=1920..3839. Numbers will need tuning
in lab; use ``--list`` to inspect actual titles and ``--arrange`` to
commit.

Usage as a CLI (on Bulbasaur):
    python -m gui.window_layout              # dry-run preview
    python -m gui.window_layout --list       # enumerate visible windows
    python -m gui.window_layout --arrange    # actually move windows

Usage from the GUI:
    from gui.window_layout import arrange_windows
    results = arrange_windows()              # {"kSA": True, "Mistral": False, ...}
"""

import ctypes
import ctypes.wintypes
import logging
import sys
from dataclasses import dataclass
from typing import Optional

log = logging.getLogger(__name__)


@dataclass(frozen=True)
class WindowSpec:
    """Target geometry for one external window."""

    name: str
    title_substrings: tuple[str, ...]
    x: int
    y: int
    w: int
    h: int


# Layout matches AJ's deliberate Bulbasaur arrangement (May 14 lab visit).
# Bulbasaur has dual 1920x1080 monitors:
#   Left monitor:  x = -1920 to 0
#   Right monitor: x = 0 to 1920
# Coordinates are rough estimates from Image #15 screenshot — first
# `--arrange` will get us in the neighborhood; tune from there.
DEFAULT_LAYOUT: tuple[WindowSpec, ...] = (
    # Left monitor — AIQM stack
    WindowSpec(
        name="kSA",
        title_substrings=("kSA 400", "AVT Manta"),
        x=-1920, y=0, w=640, h=410,
    ),
    WindowSpec(
        name="GrowthMonitor",
        title_substrings=("OMBE Growth Monitor",),
        x=-1280, y=0, w=520, h=410,
    ),
    WindowSpec(
        name="Pyrometer",
        title_substrings=("TemperaSure",),
        x=-650, y=10, w=320, h=240,
    ),
    WindowSpec(
        name="EvapControl",
        title_substrings=("Evaporation control",),
        x=-1920, y=420, w=1280, h=170,
    ),
    # Right monitor — MISTRAL fullscreen
    WindowSpec(
        name="MistralGui",
        title_substrings=("MistralGui",),
        x=0, y=0, w=1920, h=1080,
    ),
)


# Win32 SetWindowPos / ShowWindow constants.
_HWND_TOP = 0
_SW_RESTORE = 9
_SWP_SHOWWINDOW = 0x0040


def _is_windows() -> bool:
    return sys.platform == "win32"


def _user32():
    return ctypes.windll.user32


def enumerate_visible_windows() -> list[tuple[int, str]]:
    """Return ``[(hwnd, title)]`` for every visible top-level window with a title."""
    if not _is_windows():
        return []
    user32 = _user32()
    out: list[tuple[int, str]] = []

    @ctypes.WINFUNCTYPE(ctypes.c_bool, ctypes.wintypes.HWND, ctypes.wintypes.LPARAM)
    def _cb(hwnd, _lp):
        if not user32.IsWindowVisible(hwnd):
            return True
        n = user32.GetWindowTextLengthW(hwnd)
        if not n:
            return True
        buf = ctypes.create_unicode_buffer(n + 1)
        user32.GetWindowTextW(hwnd, buf, n + 1)
        title = buf.value
        if title.strip():
            out.append((int(hwnd), title))
        return True

    user32.EnumWindows(_cb, 0)
    return out


def find_window(title_substrings: tuple[str, ...]) -> Optional[tuple[int, str]]:
    """Find a visible top-level window whose title contains any substring.

    Case-insensitive. When multiple windows match, prefer the longest title
    (more specific). Returns ``(hwnd, title)`` or ``None``.
    """
    matches: list[tuple[int, str]] = []
    for hwnd, title in enumerate_visible_windows():
        title_lower = title.lower()
        if any(s.lower() in title_lower for s in title_substrings):
            matches.append((hwnd, title))
    if not matches:
        return None
    matches.sort(key=lambda t: len(t[1]), reverse=True)
    return matches[0]


def get_window_rect(hwnd: int) -> tuple[int, int, int, int]:
    """Return ``(left, top, right, bottom)`` for the given hwnd."""
    rect = ctypes.wintypes.RECT()
    _user32().GetWindowRect(hwnd, ctypes.byref(rect))
    return rect.left, rect.top, rect.right, rect.bottom


def arrange_window(spec: WindowSpec) -> bool:
    """Move and resize the window matching ``spec``. Return True on success."""
    if not _is_windows():
        log.warning("Not on Windows; cannot arrange %s", spec.name)
        return False
    match = find_window(spec.title_substrings)
    if match is None:
        log.info(
            "Window not found for %s (substrings: %s)",
            spec.name, spec.title_substrings,
        )
        return False
    hwnd, title = match
    user32 = _user32()
    # Restore first so the new rect actually takes effect on minimized/maximized
    # windows.
    user32.ShowWindow(hwnd, _SW_RESTORE)
    ok = user32.SetWindowPos(
        hwnd, _HWND_TOP,
        spec.x, spec.y, spec.w, spec.h,
        _SWP_SHOWWINDOW,
    )
    if ok:
        log.info(
            "Arranged %s '%s' at (%d, %d) %dx%d",
            spec.name, title, spec.x, spec.y, spec.w, spec.h,
        )
        return True
    log.warning("SetWindowPos failed for %s '%s'", spec.name, title)
    return False


def arrange_windows(
    layout: tuple[WindowSpec, ...] = DEFAULT_LAYOUT,
) -> dict[str, bool]:
    """Arrange every window in ``layout``. Return ``{spec.name: success_bool}``."""
    return {spec.name: arrange_window(spec) for spec in layout}


def _cli() -> None:
    import argparse

    parser = argparse.ArgumentParser(
        description=(
            "Arrange OMBE external windows (kSA, MISTRAL, EvapControl, EX04763)"
        ),
    )
    parser.add_argument(
        "--list", action="store_true",
        help="Print every visible top-level window and exit",
    )
    parser.add_argument(
        "--arrange", action="store_true",
        help="Actually move windows. Without this flag, run a dry-run preview",
    )
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(message)s")

    if not _is_windows():
        print("ERROR: this script must run on Windows (uses Win32 APIs).")
        sys.exit(1)

    if args.list:
        windows = enumerate_visible_windows()
        print(f"{len(windows)} visible top-level windows:\n")
        for hwnd, title in sorted(windows, key=lambda t: t[1].lower()):
            print(f"  [{hwnd:>10}] {title}")
        return

    if args.arrange:
        print("Arranging windows ...")
        results = arrange_windows()
        print("\nResults:")
        for name, ok in results.items():
            status = "OK  " if ok else "MISS"
            print(f"  [{status}] {name}")
        return

    # Default: dry-run preview.
    print("DRY RUN — pass --arrange to actually move windows.\n")
    for spec in DEFAULT_LAYOUT:
        match = find_window(spec.title_substrings)
        if match is None:
            print(
                f"  [MISS] {spec.name:<14} not found (looking for any of "
                f"{spec.title_substrings})"
            )
            continue
        hwnd, title = match
        left, top, right, bottom = get_window_rect(hwnd)
        print(f"  [FOUND] {spec.name:<14} '{title}'")
        print(
            f"           current: ({left}, {top}, {right}, {bottom})  "
            f"size={right - left}x{bottom - top}"
        )
        print(
            f"           target:  ({spec.x}, {spec.y}, "
            f"{spec.x + spec.w}, {spec.y + spec.h})  size={spec.w}x{spec.h}"
        )


if __name__ == "__main__":
    _cli()
