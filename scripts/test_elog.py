#!/usr/bin/env python3
"""Quick smoke test for the ELog parser. Reads the live MBE.Pressure value.

Run on Bulbasaur:
    python scripts/test_elog.py

Falls back to find_current_log() if the hardcoded path doesn't exist.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from drivers.elog import find_current_log, latest_value

LIVE_PATH = Path(
    r"C:\_Omicron_Software\EvapControl\evap_control_1.2.0.51"
    r"\evap_control_1.2.0.51\log\log_2026-05-15_000000.elo"
)


def main() -> None:
    path = LIVE_PATH if LIVE_PATH.exists() else find_current_log()
    if path is None or not path.exists():
        sys.exit(f"No live ELog file found at {LIVE_PATH}")

    print(f"Reading: {path}")
    for var in ("MBE.Pressure", "MBE.Temperature", "HTEC 2.PV", "MBE-Mani.PV"):
        try:
            ts, val = latest_value(path, var)
            print(f"  {ts.isoformat()}  {var:<22} = {val:.4g}")
        except KeyError as e:
            print(f"  (skipped {var}: {e})")


if __name__ == "__main__":
    main()
