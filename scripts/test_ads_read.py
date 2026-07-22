"""Batch READ-ONLY probe: Beckhoff PLC variables on Ch-MBE via pyads.

**READ ONLY — DO NOT ADD write_by_name IN THIS FILE.**

Discovery: on Jul 22 2026, MistralGui on Ch-MBE was found to be a
Beckhoff TwinCAT ADS client. The install directory
``C:\\YangLab10 Gui\\VS\\Win32\\ModelDescription.xml`` (62KB, dated
07-08-2020, PZ) is the full PLC symbol table. This script reads a
curated subset via pyads, which talks to the local TCATSysSrv (Windows
service on TCP:48898) which routes to the remote PLC at NetID
10.0.42.112.1.1 across TWO ports:

  - Port 851 = main PLC (pumps, valves, pressure gauges, bakeout)
  - Port 852 = PID controller (effusion cells, shutters, EBVM)

Two operating modes:

  Batch (default) - single pass, prints per-variable status:

      python scripts/test_ads_read.py

  Shadow log (loop-and-log) - polls at fixed interval, writes CSV:

      python scripts/test_ads_read.py --loop --interval-s 5 \\
          --output C:\\temp\\ads_shadow.csv --duration-s 7200

  Loop stops on Ctrl-C or after --duration-s (0 = forever).

Cross-references:
- Bulbasaur (O-MBE) uses JSON-RPC to Kestrel — different architecture,
  same MistralGui.exe. See ``mistral_psu_backend_discovery_jun23.md``.
- pyads Read=2 and Write=3 are separate ADS opcodes; ``read_by_name``
  can never emit a Write. This file avoids importing any write-related
  pyads symbol as defence-in-depth.

Safety enforcement:
- Only ``Connection.read_by_name`` is ever called
- Only ``pyads.PLCTYPE_*`` constants are imported (types, not verbs)
- The variable list is a static whitelist; no dynamic dispatch
"""
from __future__ import annotations

import argparse
import csv
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any

try:
    import pyads
except ImportError:
    print("pyads not installed. Install with: pip install pyads",
          file=sys.stderr)
    sys.exit(1)


# ---------------------------------------------------------------------------
# Endpoint (Ch-MBE Beckhoff PLC — from ModelDescription.xml)
# ---------------------------------------------------------------------------

DEFAULT_NETID = "10.0.42.112.1.1"
PORT_MAIN = 851   # main PLC: pumps, valves, gauges, bakeout, misc
PORT_PID = 852    # PID controller: effusion cells + shutters + EBVM


# ---------------------------------------------------------------------------
# READ TARGET DATACLASS + WHITELIST
#
# label = short CSV column name (keeps header manageable)
# name  = full TwinCAT symbol path (what pyads reads)
# port  = 851 or 852 (routes to the correct pyads Connection)
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class ReadTarget:
    port: int
    label: str
    name: str
    dtype: Any
    description: str = ""


def _cell_targets(cell_num: int) -> list[ReadTarget]:
    """Standard thermal cell probe set: T, V, I, setpoint, power, state, shutter."""
    prefix = f"PIDProgram.Cell{cell_num}"
    return [
        ReadTarget(PORT_PID, f"Cell{cell_num}_T",       f"{prefix}_pidTDK.ActualTemperature",              pyads.PLCTYPE_LREAL, f"Cell{cell_num} actual temp (°C)"),
        ReadTarget(PORT_PID, f"Cell{cell_num}_SetT",    f"{prefix}_SetPoint",                              pyads.PLCTYPE_LREAL, f"Cell{cell_num} setpoint (°C)"),
        ReadTarget(PORT_PID, f"Cell{cell_num}_V",       f"{prefix}_pidTDK.powerSupply.MeasuredVoltage",    pyads.PLCTYPE_LREAL, f"Cell{cell_num} measured V"),
        ReadTarget(PORT_PID, f"Cell{cell_num}_I",       f"{prefix}_pidTDK.powerSupply.MeasuredCurrent",    pyads.PLCTYPE_LREAL, f"Cell{cell_num} measured I"),
        ReadTarget(PORT_PID, f"Cell{cell_num}_Power",   f"{prefix}_pidTDK.OutputPower",                    pyads.PLCTYPE_LREAL, f"Cell{cell_num} PID output power"),
        ReadTarget(PORT_PID, f"Cell{cell_num}_State",   f"{prefix}_State",                                 pyads.PLCTYPE_INT,   f"Cell{cell_num} state (enum)"),
        ReadTarget(PORT_PID, f"Cell{cell_num}_ShutOpen",   f"{prefix}_Shutter.StatusOpen",                 pyads.PLCTYPE_BOOL,  f"Cell{cell_num} shutter open"),
        ReadTarget(PORT_PID, f"Cell{cell_num}_ShutClosed", f"{prefix}_Shutter.StatusClosed",               pyads.PLCTYPE_BOOL,  f"Cell{cell_num} shutter closed"),
    ]


READS: list[ReadTarget] = []

# --- System status (port 851) ---
READS.extend([
    ReadTarget(PORT_MAIN, "ServiceMode",       "Main.ServiceMode",       pyads.PLCTYPE_BOOL,  "System-wide maintenance mode flag"),
    ReadTarget(PORT_MAIN, "PLCErrorId",        "Main.PLCErrorId",        pyads.PLCTYPE_UINT,  "PLC error state (0 = OK)"),
    ReadTarget(PORT_MAIN, "realPresInfoMBE",   "Main.realPresInfoMBE",   pyads.PLCTYPE_BOOL,  "Real pressure info available"),
])

# --- Pressure gauges (port 851) — TOP PRIORITY for growth ---
READS.extend([
    ReadTarget(PORT_MAIN, "IonGauge1_P",   "PVCXProgram.ionGauge1.IGC_ModbusCtrl.IonGaugePressure",   pyads.PLCTYPE_LREAL, "Ion gauge 1 pressure (Torr)"),
    ReadTarget(PORT_MAIN, "IonGauge2_P",   "PVCXProgram.ionGauge2.IGC_ModbusCtrl.IonGaugePressure",   pyads.PLCTYPE_LREAL, "Ion gauge 2 pressure (Torr)"),
    ReadTarget(PORT_MAIN, "Pirani1_P",     "PVCXProgram.ionGauge1.IGC_ModbusCtrl.PiraniPressure",     pyads.PLCTYPE_LREAL, "Pirani gauge 1 (foreline)"),
    ReadTarget(PORT_MAIN, "Pirani2_P",     "PVCXProgram.ionGauge2.IGC_ModbusCtrl.PiraniPressure",     pyads.PLCTYPE_LREAL, "Pirani gauge 2 (foreline)"),
    ReadTarget(PORT_MAIN, "IGP_Running",   "IGPProgram.igp1.PumpARunning",                            pyads.PLCTYPE_BOOL,  "Ion getter pump running"),
    ReadTarget(PORT_MAIN, "IGP_Pressure",  "IGPProgram.igp1.PumpAPressure",                           pyads.PLCTYPE_LREAL, "Ion getter pump pressure"),
])

# --- Manipulator + bakeout temperatures (port 851) ---
# TemperatureMani is the commented-out entry in BiasSelector; likely still
# in the PLC symbol table even though MistralGui doesn't read it via ADS.
READS.extend([
    ReadTarget(PORT_MAIN, "TemperatureMani",  "Main.TemperatureMani",  pyads.PLCTYPE_LREAL, "Substrate manipulator temp (best-effort)"),
    ReadTarget(PORT_MAIN, "BakeoutT_1",       "Main.Temperature_1",    pyads.PLCTYPE_INT,   "Bakeout channel 1 temp"),
])

# --- Turbo pumps (port 851) ---
READS.extend([
    ReadTarget(PORT_MAIN, "Turbo1_On",       "TurboProgram.pump1.pumpOn",             pyads.PLCTYPE_BOOL,  "Turbo1 running"),
    ReadTarget(PORT_MAIN, "Turbo1_RPM",      "TurboProgram.pump1.spdRPM",             pyads.PLCTYPE_UINT,  "Turbo1 speed (RPM)"),
    ReadTarget(PORT_MAIN, "Turbo1_DrvA",     "TurboProgram.pump1.drvCurrent",         pyads.PLCTYPE_LREAL, "Turbo1 drive current (A)"),
    ReadTarget(PORT_MAIN, "Turbo1_MotorT",   "TurboProgram.pump1.TemperatureMotor",   pyads.PLCTYPE_UINT,  "Turbo1 motor temperature (°C)"),
    ReadTarget(PORT_MAIN, "Turbo2_On",       "TurboProgram.pump2.pumpOn",             pyads.PLCTYPE_BOOL,  "Turbo2 running"),
    ReadTarget(PORT_MAIN, "Turbo2_RPM",      "TurboProgram.pump2.spdRPM",             pyads.PLCTYPE_UINT,  "Turbo2 speed (RPM)"),
    ReadTarget(PORT_MAIN, "Turbo2_DrvA",     "TurboProgram.pump2.drvCurrent",         pyads.PLCTYPE_LREAL, "Turbo2 drive current (A)"),
    ReadTarget(PORT_MAIN, "Turbo2_MotorT",   "TurboProgram.pump2.TemperatureMotor",   pyads.PLCTYPE_UINT,  "Turbo2 motor temperature (°C)"),
])

# --- Valve states (port 851) ---
READS.extend([
    ReadTarget(PORT_MAIN, "V1_Open",       "Main.TurboPumpInterlock1.Valve_StateOpen",   pyads.PLCTYPE_BOOL, "Valve1 open"),
    ReadTarget(PORT_MAIN, "V1_Closed",     "Main.TurboPumpInterlock1.Valve_StateClosed", pyads.PLCTYPE_BOOL, "Valve1 closed"),
    ReadTarget(PORT_MAIN, "PrePump1_St",   "Main.PrePump1.Valve_State",                  pyads.PLCTYPE_INT,  "PrePump1 valve state (enum)"),
    ReadTarget(PORT_MAIN, "PrePump2_St",   "Main.PrePump2.Valve_State",                  pyads.PLCTYPE_INT,  "PrePump2 valve state (enum)"),
])

# --- Cells 1..7 (port 852) — full TDK-Lambda PSU + PID ---
for i in range(1, 8):
    READS.extend(_cell_targets(i))

# --- EBVM (port 852) — no PSU, shutter + coolant only ---
READS.extend([
    ReadTarget(PORT_PID, "EBVM_ShutOpen",   "PIDProgram.EBVM_Shutter.StatusOpen",   pyads.PLCTYPE_BOOL,  "EBVM shutter open"),
    ReadTarget(PORT_PID, "EBVM_ShutClosed", "PIDProgram.EBVM_Shutter.StatusClosed", pyads.PLCTYPE_BOOL,  "EBVM shutter closed"),
    ReadTarget(PORT_PID, "EBVM_Coolant",    "PIDProgram.EBVM_FlowMeter.Flow",       pyads.PLCTYPE_LREAL, "EBVM coolant flow"),
])


# ---------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------

def read_one(plc, target: ReadTarget) -> tuple[Any, str]:
    """Read one target. Returns (value, "") on success, (None, err) on fail."""
    try:
        return plc.read_by_name(target.name, target.dtype), ""
    except Exception as exc:  # noqa: BLE001 - evidence capture per probe pattern
        return None, f"{type(exc).__name__}: {exc}"


def read_all(plc_main, plc_pid, reads: list[ReadTarget]) -> dict[str, tuple[Any, str]]:
    """Return {label: (value, error_str)} for every target."""
    out: dict[str, tuple[Any, str]] = {}
    for t in reads:
        plc = plc_main if t.port == PORT_MAIN else plc_pid
        out[t.label] = read_one(plc, t)
    return out


def print_batch(results: dict[str, tuple[Any, str]], reads: list[ReadTarget]) -> tuple[int, int]:
    """Print pretty batch mode output. Returns (successes, failures)."""
    successes = failures = 0
    label_w = max(len(t.label) for t in reads)
    for t in reads:
        v, err = results[t.label]
        if err:
            print(f"  [FAIL] {t.label:<{label_w}} : {err}")
            failures += 1
        else:
            print(f"  [OK]   {t.label:<{label_w}} = {v!r:<15}  ({t.description})")
            successes += 1
    return successes, failures


def loop_and_log(plc_main, plc_pid, reads: list[ReadTarget], args) -> int:
    """CSV shadow-log mode. Polls at fixed interval, writes each row + flushes."""
    output_path = args.output
    labels = [t.label for t in reads]

    if output_path:
        csv_fh = open(output_path, "w", newline="", encoding="utf-8")
        print(f"Shadow log -> {output_path}", file=sys.stderr)
    else:
        csv_fh = sys.stdout

    writer = csv.writer(csv_fh)
    writer.writerow(["timestamp"] + labels)
    csv_fh.flush()

    start = time.monotonic()
    rows = 0
    print(f"Polling {len(reads)} variables every {args.interval_s}s "
          f"(duration: {'forever' if args.duration_s <= 0 else f'{args.duration_s}s'})",
          file=sys.stderr)
    print("Ctrl-C to stop.", file=sys.stderr)

    try:
        while True:
            iter_start = time.monotonic()
            results = read_all(plc_main, plc_pid, reads)
            ts = datetime.now(timezone.utc).isoformat()
            row = [ts] + [
                (results[label][0] if results[label][1] == "" else "")
                for label in labels
            ]
            writer.writerow(row)
            csv_fh.flush()
            rows += 1

            if rows % 12 == 0:  # every ~minute at 5s interval
                elapsed = time.monotonic() - start
                print(f"  ...{rows} rows written, {elapsed:.0f}s elapsed",
                      file=sys.stderr)

            if args.duration_s > 0 and (time.monotonic() - start) >= args.duration_s:
                break

            iter_took = time.monotonic() - iter_start
            sleep_for = max(0.0, args.interval_s - iter_took)
            time.sleep(sleep_for)

    except KeyboardInterrupt:
        print("\nStopping (Ctrl-C).", file=sys.stderr)
    finally:
        if output_path:
            csv_fh.close()

    print(f"Wrote {rows} rows in {time.monotonic() - start:.0f}s.", file=sys.stderr)
    return 0


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(
        description="Batch READ-ONLY probe of Ch-MBE Beckhoff PLC via pyads.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--netid", default=DEFAULT_NETID,
                        help=f"PLC AmsNetId (default: {DEFAULT_NETID})")
    parser.add_argument("--loop", action="store_true",
                        help="Continuously poll and log to CSV (Ctrl-C to stop)")
    parser.add_argument("--interval-s", type=float, default=5.0,
                        help="Loop poll interval in seconds (default: 5.0)")
    parser.add_argument("--duration-s", type=float, default=0.0,
                        help="Stop after N seconds (default: 0 = forever)")
    parser.add_argument("--output", default=None,
                        help="CSV output path (loop mode; default: stdout)")
    args = parser.parse_args()

    print(f"Target: NetID={args.netid} Ports={PORT_MAIN},{PORT_PID}", file=sys.stderr)
    print(f"Reading {len(READS)} variables (READ-ONLY, no writes possible)",
          file=sys.stderr)
    print(file=sys.stderr)

    plc_main = pyads.Connection(args.netid, PORT_MAIN)
    plc_pid = pyads.Connection(args.netid, PORT_PID)
    try:
        plc_main.open()
        plc_pid.open()
    except Exception as exc:  # noqa: BLE001
        print(f"open() FAILED: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1

    try:
        if args.loop:
            return loop_and_log(plc_main, plc_pid, READS, args)
        else:
            results = read_all(plc_main, plc_pid, READS)
            successes, failures = print_batch(results, READS)
            print(file=sys.stderr)
            print(f"Read: {successes} succeeded, {failures} failed of {len(READS)}",
                  file=sys.stderr)
            return 0 if successes > 0 else 2
    finally:
        for plc in (plc_main, plc_pid):
            try:
                plc.close()
            except Exception:  # noqa: BLE001
                pass


if __name__ == "__main__":
    sys.exit(main())
