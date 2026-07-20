"""Standalone Scrubber tab demo — attach an archived session and play with the slide-bar.

Purpose: Mac-side dry-run of the Scrubber tab against an archived growth
session, without needing to launch the full growth_monitor_app or start
a live capture. Useful for demos when the O-MBE is unavailable.

Usage:
    python scripts/scrubber_demo.py
        → opens the Scrubber attached to the default demo session
          (growth_unnamed_20260616_161049, 454 frames, ~19.7 hours)

    python scripts/scrubber_demo.py --session <path>
        → attach a different session directory

    python scripts/scrubber_demo.py --list
        → list available growth sessions with frame counts

Notes:
- Read-only: this script does not modify any session files. Safe to demo.
- The Scrubber tab is a QWidget, so this script wraps it in a QMainWindow
  with a title bar and shows it maximized.
- Frames load lazily from disk on scrub (same as in the full app), so
  attaching a large session is instant.
"""
from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

# Fix Qt platform plugin discovery for Anaconda environments where the
# compiled-in prefix path doesn't match the actual PyQt6 plugin location.
# Mirrors the shim in growth_monitor_app.py (added after the "Could not
# find Qt platform plugin 'cocoa' in ''" error on AJ's base conda env,
# Jul 17 2026). MUST run before `from PyQt6...` imports fire.
import PyQt6  # noqa: E402
_qt_plugins = Path(PyQt6.__file__).parent / "Qt6" / "plugins"
if _qt_plugins.exists() and "QT_QPA_PLATFORM_PLUGIN_PATH" not in os.environ:
    os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = str(_qt_plugins / "platforms")

from PyQt6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget  # noqa: E402

# Ensure repo root on path so `from gui.scrubber_tab import ScrubberTab` works
_REPO_ROOT = Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from gui.scrubber_tab import ScrubberTab, build_frame_index  # noqa: E402


DEFAULT_SESSION = _REPO_ROOT / "logs" / "growths" / "growth_unnamed_20260616_161049"


def _list_sessions() -> None:
    """Print a summary of all available growth sessions with frame counts."""
    growths_dir = _REPO_ROOT / "logs" / "growths"
    if not growths_dir.exists():
        print(f"No growth directory found at {growths_dir}")
        return

    rows: list[tuple[str, int, float]] = []
    for session_dir in sorted(growths_dir.iterdir()):
        if not session_dir.is_dir():
            continue
        try:
            index = build_frame_index(session_dir)
        except Exception as exc:  # pragma: no cover — best-effort inventory
            print(f"  {session_dir.name}: could not index ({exc})")
            continue
        n = len(index)
        span_min = (index[-1].elapsed_s - index[0].elapsed_s) / 60.0 if index else 0.0
        rows.append((session_dir.name, n, span_min))

    if not rows:
        print(f"No sessions found under {growths_dir}")
        return

    print(f"{'Session':<50} {'Frames':>7}   {'Span':>10}")
    print("-" * 72)
    for name, n, span_min in rows:
        span = f"{span_min:>7.1f} min" if span_min < 90 else f"{span_min/60:>6.1f} hr "
        print(f"{name:<50} {n:>7}   {span:>10}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument(
        "--session",
        type=Path,
        default=DEFAULT_SESSION,
        help=f"Session directory to attach (default: {DEFAULT_SESSION.name})",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="List available sessions and exit",
    )
    args = parser.parse_args()

    if args.list:
        _list_sessions()
        return 0

    session_dir: Path = args.session
    if not session_dir.exists():
        print(f"ERROR: session directory does not exist: {session_dir}", file=sys.stderr)
        return 2

    # Quick pre-check so the user sees what's about to load
    index = build_frame_index(session_dir)
    if not index:
        print(f"WARNING: no frames indexed in {session_dir}", file=sys.stderr)
        print("The Scrubber will show a placeholder; nothing to scrub.", file=sys.stderr)
    else:
        span_min = (index[-1].elapsed_s - index[0].elapsed_s) / 60.0
        sources: dict[str, int] = {}
        for entry in index:
            sources[entry.source] = sources.get(entry.source, 0) + 1
        source_str = ", ".join(f"{v} {k}" for k, v in sources.items())
        span_label = f"{span_min:.1f} min" if span_min < 90 else f"{span_min/60:.1f} hr"
        print(f"Attaching: {session_dir.name}")
        print(f"  Frames: {len(index)} ({source_str})")
        print(f"  Span:   {span_label}")

    app = QApplication(sys.argv)

    window = QMainWindow()
    window.setWindowTitle(f"Scrubber demo — {session_dir.name}")
    window.resize(1280, 800)

    central = QWidget()
    layout = QVBoxLayout(central)
    layout.setContentsMargins(0, 0, 0, 0)

    scrubber = ScrubberTab()
    scrubber.attach_session(session_dir)
    layout.addWidget(scrubber)

    window.setCentralWidget(central)
    window.show()

    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
