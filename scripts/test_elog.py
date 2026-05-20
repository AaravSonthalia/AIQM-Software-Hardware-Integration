#!/usr/bin/env python3
"""MISTRAL <-> ELog spot-check.

Reads a panel of variables from the live .elo file and prints them
formatted (using EvapControl's per-var format strings, so categorical
fields like shutter state render as OPEN/CLOSED instead of 1.0/0.0).

Run on Bulbasaur:
    python scripts/test_elog.py

Use this to cross-check values against the MISTRAL display.

Falls back to find_current_log() if the hardcoded path doesn't exist.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from drivers.elog import find_current_log, format_value, latest_record

# Panel of vars to cross-check against MISTRAL. Group by sub-panel so
# the output is easy to read off alongside the MISTRAL UI.
PANEL = [
    # (group label, list of var names)
    ("Chamber",     ["MBE.Pressure", "MBE.Pirani", "MBE.Temperature"]),
    ("HTEC 2",      ["HTEC 2.setpoint", "HTEC 2.PV", "HTEC 2.shutter",
                     "HTEC 2.OP", "HTEC 2.mode"]),
    ("HTEC Y",      ["HTEC Y.setpoint", "HTEC Y.PV", "HTEC Y.shutter"]),
    ("LTEC 2 Eu",   ["LTEC 2 Eu.setpoint", "LTEC 2 Eu.PV", "LTEC 2 Eu.shutter"]),
    ("Substrate",   ["MBE-Mani.setpoint", "MBE-Mani.PV", "MBE-Mani.mode"]),
]


def main() -> None:
    path = find_current_log()
    if path is None or not path.exists():
        sys.exit("No live ELog file for today found. Check EvapControl is running.")

    print(f"Reading: {path}")

    wanted = [v for _, vars_ in PANEL for v in vars_]
    try:
        ts, vals = latest_record(path, wanted)
    except KeyError as e:
        # Show which var failed so we can correct the spelling in PANEL
        sys.exit(f"Schema lookup failed: {e}")

    print(f"Timestamp: {ts.isoformat()}\n")
    for group, names in PANEL:
        print(f"[{group}]")
        for n in names:
            v, fmt = vals[n]
            print(f"  {n:<22} = {format_value(v, fmt):<18}  (raw {v:.4g}, fmt {fmt!r})")
        print()


if __name__ == "__main__":
    main()
