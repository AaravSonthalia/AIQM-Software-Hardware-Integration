"""Compatibility launcher for scripts/heater_step_test.py."""

from pathlib import Path
import runpy


if __name__ == "__main__":
    target = Path(__file__).resolve().parent / "scripts" / "heater_step_test.py"
    runpy.run_path(str(target), run_name="__main__")
