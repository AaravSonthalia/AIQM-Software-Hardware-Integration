"""Batch read-only probe: Beckhoff PLC variables via pyads.

**READ ONLY — DO NOT ADD write_by_name IN THIS FILE.**

Discovery: on Jul 22 2026, MistralGui on Ch-MBE (Chalcogenide MBE) was
found to be a Beckhoff TwinCAT ADS client. The install directory
``C:\\YangLab10 Gui\\VS\\Win32\\ModelDescription.xml`` is the full PLC
symbol table (62KB). This script reads a curated subset via pyads,
which talks to the local TCATSysSrv (Windows service, listens on
TCP:48898) which in turn routes to the remote PLC at NetID
10.0.42.112.1.1:851.

Cross-references:
- Bulbasaur (O-MBE) uses JSON-RPC to Kestrel — different architecture,
  same MistralGui.exe (per-chamber config). See
  ``mistral_psu_backend_discovery_jun23.md`` memory.
- pyads Read=2 and Write=3 are separate ADS opcodes at the protocol
  level; the ``read_by_name`` code path can never emit a Write. This
  file avoids importing any write-related pyads symbol as
  defence-in-depth.

Safety enforcement:
- Only ``Connection.read_by_name`` is called
- Only ``pyads.PLCTYPE_*`` constants are imported (types, not verbs)
- The variable list is a whitelist; nothing is dynamically added

Usage::

    python scripts/test_ads_read.py                # default: Ch-MBE PLC
    python scripts/test_ads_read.py --netid X.Y.Z.W.1.1  # override
"""
from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from typing import Any

try:
    import pyads
except ImportError:
    print("pyads not installed. Install with: pip install pyads",
          file=sys.stderr)
    sys.exit(1)


# ---------------------------------------------------------------------------
# Target endpoint (Ch-MBE Beckhoff PLC — from ModelDescription.xml)
# ---------------------------------------------------------------------------

DEFAULT_NETID = "10.0.42.112.1.1"
DEFAULT_PORT = 851


# ---------------------------------------------------------------------------
# Read whitelist. Every entry is READ-ONLY.
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class ReadTarget:
    """One PLC variable to probe. name = TwinCAT symbol path."""
    name: str
    dtype: Any  # pyads PLCTYPE_* constant
    description: str = ""


READS: list[ReadTarget] = [
    # ---- System-level ----
    ReadTarget("Main.ServiceMode", pyads.PLCTYPE_BOOL,
               "System-wide maintenance mode flag"),
    ReadTarget("Main.realPresInfoMBE", pyads.PLCTYPE_BOOL,
               "Whether real pressure info is available"),

    # ---- Turbo pump 1 status ----
    ReadTarget("TurboProgram.pump1.pumpOn", pyads.PLCTYPE_BOOL,
               "Turbo1 running"),
    ReadTarget("TurboProgram.pump1.pumpStby", pyads.PLCTYPE_BOOL,
               "Turbo1 in standby"),

    # ---- Turbo pump 1 speeds ----
    ReadTarget("TurboProgram.pump1.spdRPM", pyads.PLCTYPE_UINT,
               "Turbo1 actual speed (RPM)"),
    ReadTarget("TurboProgram.pump1.maxRotSpdRPM", pyads.PLCTYPE_UINT,
               "Turbo1 max rotation speed setpoint (RPM)"),
    ReadTarget("TurboProgram.pump1.NominalRotSpdRPM", pyads.PLCTYPE_UINT,
               "Turbo1 nominal rotation speed (RPM)"),

    # ---- Turbo pump 1 telemetry ----
    ReadTarget("TurboProgram.pump1.drvCurrent", pyads.PLCTYPE_LREAL,
               "Turbo1 drive current (A)"),
    ReadTarget("TurboProgram.pump1.TemperatureElectronics", pyads.PLCTYPE_UINT,
               "Turbo1 electronics temperature (C)"),
    ReadTarget("TurboProgram.pump1.TemperatureMotor", pyads.PLCTYPE_UINT,
               "Turbo1 motor temperature (C)"),
    ReadTarget("TurboProgram.pump1.TemperatureBearing", pyads.PLCTYPE_UINT,
               "Turbo1 bearing temperature (C)"),
    ReadTarget("TurboProgram.pump1.TemperaturePumpBottom", pyads.PLCTYPE_UINT,
               "Turbo1 pump body bottom temperature (C)"),

    # ---- Valve interlocks & states ----
    ReadTarget("Main.TurboPumpInterlock1.Valve_StateOpen", pyads.PLCTYPE_BOOL,
               "Valve1 currently open"),
    ReadTarget("Main.TurboPumpInterlock1.Valve_StateClosed", pyads.PLCTYPE_BOOL,
               "Valve1 currently closed"),
    ReadTarget("Main.TurboPumpInterlock1.RotaryOut", pyads.PLCTYPE_BOOL,
               "Rotary pump enable output"),
    ReadTarget("Main.TurboPumpInterlock1.VentModeEnable", pyads.PLCTYPE_BOOL,
               "Vent mode enabled"),
    ReadTarget("Main.PrePump1.Valve_State", pyads.PLCTYPE_INT,
               "Pre-pump 1 valve state (enum)"),
    ReadTarget("Main.PrePump2.Valve_State", pyads.PLCTYPE_INT,
               "Pre-pump 2 valve state (enum)"),

    # ---- UserCommand handles (READING setpoints is safe; writing would
    #      actuate hardware — DO NOT WRITE) ----
    ReadTarget("Main.ValveUserCommand1", pyads.PLCTYPE_BOOL,
               "Valve1 user-command setpoint (READ ONLY)"),
    ReadTarget("Main.TurboUserCommand1", pyads.PLCTYPE_INT,
               "Turbo1 user-command setpoint (READ ONLY)"),
]


# ---------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------

def read_one(plc: "pyads.Connection", target: ReadTarget) -> tuple[Any, str]:
    """Read one target. Returns (value, "") on success, (None, error) on fail."""
    try:
        value = plc.read_by_name(target.name, target.dtype)
        return value, ""
    except Exception as exc:  # noqa: BLE001 - evidence capture per probe pattern
        return None, f"{type(exc).__name__}: {exc}"


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Batch READ-ONLY probe of Beckhoff PLC via pyads."
    )
    parser.add_argument("--netid", default=DEFAULT_NETID,
                        help=f"PLC AmsNetId (default: {DEFAULT_NETID})")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT,
                        help=f"PLC AmsPort (default: {DEFAULT_PORT})")
    args = parser.parse_args()

    print(f"Target: NetID={args.netid} Port={args.port}")
    print(f"Reading {len(READS)} variables (READ-ONLY, no writes possible)")
    print()

    plc = pyads.Connection(args.netid, args.port)
    try:
        plc.open()
    except Exception as exc:  # noqa: BLE001
        print(f"open() FAILED: {type(exc).__name__}: {exc}", file=sys.stderr)
        print("  Common causes: TCATSysSrv not running, route not configured,",
              file=sys.stderr)
        print("  or PLC unreachable. Verify netstat shows :48898 LISTENING",
              file=sys.stderr)
        print("  and TCATSysSrv is Running (Get-Process TCATSysSrv).",
              file=sys.stderr)
        return 1

    successes = 0
    failures = 0
    try:
        # Longest name for alignment
        name_width = max(len(t.name) for t in READS)
        for target in READS:
            value, error = read_one(plc, target)
            if error:
                print(f"  [FAIL] {target.name:<{name_width}} : {error}")
                failures += 1
            else:
                print(f"  [OK]   {target.name:<{name_width}} = {value!r:<12}  ({target.description})")
                successes += 1
    finally:
        try:
            plc.close()
        except Exception:  # noqa: BLE001 - best-effort cleanup
            pass

    print()
    print(f"Read: {successes} succeeded, {failures} failed of {len(READS)}")
    return 0 if successes > 0 else 2


if __name__ == "__main__":
    sys.exit(main())
