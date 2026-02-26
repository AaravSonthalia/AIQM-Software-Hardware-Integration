"""Compatibility launcher for scripts/psu_diagnostic.py."""

from pathlib import Path
import runpy


if __name__ == "__main__":
    target = Path(__file__).resolve().parent / "scripts" / "psu_diagnostic.py"
    runpy.run_path(str(target), run_name="__main__")
