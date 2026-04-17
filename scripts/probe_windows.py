"""
Window discovery probe — MISTRAL + Evap Control.

Purpose: determine whether MISTRAL and the Evap Control window expose their
V/I/pressure values as UIA-readable controls, or whether we need OCR.

Run on Bulbasaur with both MISTRAL and Evap Control open.

Usage:
    python scripts/probe_windows.py

Output (timestamped folder under probe_output/):
    - all_windows.txt       : every visible top-level window title
    - <target>_<i>.png      : screenshot of each matched window
    - <target>_<i>_tree.txt : pywinauto UIA control tree (depth 5)

Share the folder contents back to drive driver design.
"""
import ctypes
import ctypes.wintypes
import io
import sys
import time
from contextlib import redirect_stdout
from pathlib import Path

OUT_DIR = (
    Path(__file__).resolve().parent.parent
    / "probe_output"
    / time.strftime("%Y%m%d_%H%M%S")
)

TARGETS: dict[str, list[str]] = {
    "mistral": ["mistral"],
    "evap_control": ["evaporation control", "evap control"],
}


def enum_top_level() -> list[tuple[int, str]]:
    user32 = ctypes.windll.user32

    def _title(hwnd: int) -> str:
        n = user32.GetWindowTextLengthW(hwnd)
        if not n:
            return ""
        buf = ctypes.create_unicode_buffer(n + 1)
        user32.GetWindowTextW(hwnd, buf, n + 1)
        return buf.value

    found: list[tuple[int, str]] = []

    @ctypes.WINFUNCTYPE(ctypes.c_bool, ctypes.wintypes.HWND, ctypes.wintypes.LPARAM)
    def _cb(hwnd, _lp):
        t = _title(hwnd)
        if t and user32.IsWindowVisible(hwnd):
            found.append((int(hwnd), t))
        return True

    user32.EnumWindows(_cb, 0)
    return found


def match(windows: list[tuple[int, str]], substrings: list[str]) -> list[tuple[int, str]]:
    out = []
    for hwnd, title in windows:
        tl = title.lower()
        if any(s.lower() in tl for s in substrings):
            out.append((hwnd, title))
    return out


def screenshot(hwnd: int, path: Path) -> dict:
    import mss
    import mss.tools

    rect = ctypes.wintypes.RECT()
    ctypes.windll.user32.GetWindowRect(hwnd, ctypes.byref(rect))
    mon = {
        "left": rect.left,
        "top": rect.top,
        "width": rect.right - rect.left,
        "height": rect.bottom - rect.top,
    }
    with mss.mss() as sct:
        img = sct.grab(mon)
        mss.tools.to_png(img.rgb, img.size, output=str(path))
    return mon


def dump_tree(hwnd: int, title: str, path: Path, depth: int = 5) -> None:
    from pywinauto import Application

    app = Application(backend="uia").connect(handle=hwnd, timeout=10)
    win = app.window(handle=hwnd)
    buf = io.StringIO()
    with redirect_stdout(buf):
        win.print_control_identifiers(depth=depth)
    path.write_text(
        f"Window title: {title}\nHWND: {hwnd}\n\n{buf.getvalue()}",
        encoding="utf-8",
    )


def main() -> None:
    try:
        import pywinauto  # noqa: F401
        import mss  # noqa: F401
    except ImportError as e:
        print(f"Missing dependency: {e}")
        print("Install with: pip install pywinauto mss")
        sys.exit(1)

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    windows = enum_top_level()
    print(f"{len(windows)} visible top-level windows")

    (OUT_DIR / "all_windows.txt").write_text(
        "\n".join(
            f"{h:>10}  {t}"
            for h, t in sorted(windows, key=lambda x: x[1].lower())
        ),
        encoding="utf-8",
    )

    for name, subs in TARGETS.items():
        matches = match(windows, subs)
        print(f"\n[{name}] {len(matches)} match(es)")
        for i, (hwnd, title) in enumerate(matches):
            print(f"  {i}: hwnd={hwnd}  title={title!r}")
            png = OUT_DIR / f"{name}_{i}.png"
            tree = OUT_DIR / f"{name}_{i}_tree.txt"
            try:
                mon = screenshot(hwnd, png)
                print(f"     screenshot -> {png.name}  {mon}")
            except Exception as ex:
                print(f"     screenshot FAILED: {ex}")
            try:
                dump_tree(hwnd, title, tree)
                print(f"     tree       -> {tree.name}")
            except Exception as ex:
                print(f"     tree FAILED: {ex}")

    print(f"\nOutput folder: {OUT_DIR}")


if __name__ == "__main__":
    main()
