"""Pyrometer Modbus discovery — READ-ONLY (baud × device_id) sweep.

Purpose: for a probe that is silent under our default (COM4, 115200,
device_id=1) sweep, this script broadens the search to a superset of
Jacques's script's baud set + a range of Modbus slave IDs. If the probe
was reconfigured (different baud or slave ID) in an earlier session, a
sweep like this will surface the current configuration WITHOUT any
writes to the device.

Ordering (see docs/lab_command_sheet.md pyrometer flowchart):

    Pyrometer silent when TemperaSure closed?
      → Run THIS SCRIPT FIRST.
        - Found identity at any (baud, device_id)?
          → YES: reconfigure driver to that combo. STOP.
        - All combos silent?
          → Continue to pyrometer_force_modbus.py (expert-gated,
            experimental write of the Exactus→Modbus command).

════════════════════════════════════════════════════════════════════════
READ-ONLY invariant — do not weaken:

  * Only Modbus function code 0x03 (Read Holding Registers) is sent.
    Never 0x05 (Write Single Coil), 0x06 (Write Single Register), or
    0x10 (Write Multiple Registers). Reading holding registers does NOT
    modify device state per the Modbus RTU spec §7.3.
  * Data-only reads. The four data registers (REG_VER, REG_NAME0,
    REG_SN0, REG_CH1_TEMP) are the entire read surface — this script
    does NOT read the config registers (REG_ADDR, REG_BAUD, REG_RATE,
    REG_CMD). Reading them is technically safe, but keeping them out
    entirely means a future edit that accidentally adds a probe for one
    of these addresses is caught by the SAFETY guard in
    _probe_modbus_read.
  * Never sends the mode-switch, reset, or reboot commands.
  * If you need to CHANGE device state (mode-switch, reset, set rate),
    that requires a separate write-capable script — deliberately not
    this one. See pyrometer_force_modbus.py (P3 in the plan).
════════════════════════════════════════════════════════════════════════

Usage (Bulbasaur):
    python scripts\\pyrometer_modbus_discover.py                  # defaults
    python scripts\\pyrometer_modbus_discover.py --port COM3
    python scripts\\pyrometer_modbus_discover.py --exhaustive
    python scripts\\pyrometer_modbus_discover.py \
        --initial-wait-s 8.0                                        # after power-cycle
    python scripts\\pyrometer_modbus_discover.py \
        --output-dir C:\\lab_evidence\\2026-07-30\\pyro_discover

Sweep structure: OPEN-ONCE-PER-BAUD. For each baud rate, the serial
port is opened ONCE and all device IDs are probed inside that same
open. Rationale: opening a serial port is ~100 ms on Windows; a naive
open/close per (baud, id) combo would burn ~11 s of pure port-cycle
overhead for the default 5×11 sweep, versus ~500 ms for open-once.
"""
from __future__ import annotations

import argparse
import json
import struct
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

# Reuse the CI-safe environment helpers from pyrometer_physical_debug.
# Those two functions live at module top there (no lab-only deps) so
# importing them here doesn't drag in pyserial/pymodbus. We do NOT reuse
# write_evidence_bundle — its JSON shape (phase_1_port / phase_2_exactus /
# phase_3_modbus) doesn't map onto this script's (baud, id) sweep.
from scripts.pyrometer_physical_debug import (  # noqa: E402
    describe_conflicting_processes,
    list_available_ports,
)


# ---------------------------------------------------------------------------
# Local register constants — do NOT import from drivers.pyrometer
# ---------------------------------------------------------------------------
#
# drivers.pyrometer transitively pulls in heater_control → owon_power_supply
# → pyvisa, which is a Windows-lab-only dep absent from CI (see
# [[gui-lazy-import-pattern]]). Duplicating the four addresses here (four
# lines) keeps this module CI-safe at module-top. Source of truth: BASF
# Exactus manual register map, cross-referenced against
# drivers/pyrometer.py (which reads the same manual). If we ever grow a
# third script that needs these, factor into drivers/_pyrometer_registers.py
# (constants only, no imports beyond stdlib).

REG_CH1_TEMP: int = 0x0000  # Channel 1 temperature (float32 across 2 regs)
REG_NAME0:    int = 0x1100  # Device name (32 ASCII bytes across 32 regs)
REG_VER:      int = 0x1300  # Firmware version (u16: high=major, low=minor)
REG_SN0:      int = 0x1305  # Serial number (9 ASCII bytes across 9 regs)


# ---------------------------------------------------------------------------
# READ-ONLY safety constants
# ---------------------------------------------------------------------------

# Only function code 0x03 (Read Holding Registers) is used. Same rationale
# as pyrometer_physical_debug — tight allowlist so a future edit adding
# 0x04 or (worse) any write function has to think about it.
READ_ONLY_MODBUS_FUNCTIONS: tuple[int, ...] = (0x03,)

# Config registers this script MUST NOT probe, even for reads. Keeping
# them out entirely means an accidental future edit that adds one of
# these addresses is caught by the SAFETY guard in _probe_modbus_read
# before any bytes hit the wire.
CONFIG_REGISTERS_FORBIDDEN: tuple[int, ...] = (
    0x1007,  # REG_ADDR  — device Modbus slave address (bricking risk if written)
    0x1008,  # REG_BAUD  — baud rate (bricking risk if written)
    0x1011,  # REG_RATE  — sample rate
    0x8000,  # REG_CMD   — command register (CMD_SAVE / CMD_REBOOT)
)


# ---------------------------------------------------------------------------
# Sweep constants
# ---------------------------------------------------------------------------

# Superset of pyrometer_physical_debug's baud list. Jacques's script's
# default set. Ordered fast → slow so the common case (default 115200)
# probes first.
CANDIDATE_BAUDS: tuple[int, ...] = (115200, 57600, 38400, 19200, 9600)

# Modbus slave ID sweep.
#   1        — current default (matches ModbusPyrometer / TemperaSure config)
#   2-10     — plausibly re-configured to a low non-default value
#   247      — max valid Modbus slave ID / edge probe
#
# Address 0 is Modbus BROADCAST and is intentionally NOT in the sweep:
# broadcast is a write-only convention per the Modbus RTU spec §4.1.1;
# issuing a read (function 0x03) to broadcast is spec-undefined and can
# either be silently dropped or elicit garbage responses from every
# device on the bus. We stick to unicast addresses only.
CANDIDATE_DEVICE_IDS: tuple[int, ...] = (1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 247)

# The single register probed to test whether a (baud, device_id) combo
# has a live pyrometer at the other end. One 16-bit register = smallest
# possible probe (fastest timeout on failure, minimal chance of triggering
# device-side buffer behavior).
LIGHTWEIGHT_PROBE_REG: int = REG_VER
LIGHTWEIGHT_PROBE_COUNT: int = 1


# ---------------------------------------------------------------------------
# Terminal helpers (mirror pyrometer_physical_debug's style)
# ---------------------------------------------------------------------------

def status(label: str, ok: bool, detail: str = "") -> None:
    tag = "[PASS]" if ok else "[FAIL]"
    line = f"  {tag} {label}"
    if detail:
        line += f" — {detail}"
    print(line)


def info(label: str, detail: str = "") -> None:
    line = f"  [INFO] {label}"
    if detail:
        line += f" — {detail}"
    print(line)


def section(title: str) -> None:
    print()
    print(f"--- {title} ---")


# ---------------------------------------------------------------------------
# Modbus probe (single (baud, id) combo, function 0x03 only)
# ---------------------------------------------------------------------------

def _probe_modbus_read(client, addr: int, count: int, device_id: int) -> dict:
    """Issue ONE Modbus function-0x03 read. Refuses forbidden addresses.

    SAFETY: addr is checked against CONFIG_REGISTERS_FORBIDDEN before
    any wire traffic. This gate exists so a future edit that accidentally
    adds a probe for a config register raises before touching the device.

    Returns: {"response": bool, "registers": list[int] | None, "error": str}
    Never raises — pymodbus runtime exceptions (ModbusIOException on
    timeout, etc.) are captured into result["error"]. Same regression
    guard as pyrometer_physical_debug (fixed there in commit 2725667
    after a silent-device probe crashed the whole script).
    """
    if addr in CONFIG_REGISTERS_FORBIDDEN:
        raise RuntimeError(
            f"SAFETY: refusing to read config register 0x{addr:04X} "
            "(in CONFIG_REGISTERS_FORBIDDEN). See module docstring for "
            "the reasoning behind the ban."
        )
    result: dict = {"response": False, "registers": None, "error": ""}
    rr = None
    try:
        # pymodbus renamed the device-ID kwarg between versions
        # (unit → slave → device_id); loop matches the driver's compat
        # pattern (see drivers/pyrometer.py:_read_holding).
        for kw in ("device_id", "slave", "unit"):
            try:
                rr = client.read_holding_registers(
                    addr, count, **{kw: device_id}
                )
                break
            except TypeError:
                continue
        if rr is None:
            rr = client.read_holding_registers(
                address=addr, count=count, device_id=device_id,
            )
    except Exception as exc:  # noqa: BLE001 — see docstring
        result["error"] = f"{type(exc).__name__}: {exc}"
        return result

    if hasattr(rr, "isError") and rr.isError():
        result["error"] = f"Modbus error: {rr}"
    elif not hasattr(rr, "registers") or len(rr.registers) < count:
        result["error"] = (
            f"empty/short response ({len(getattr(rr, 'registers', []))} regs)"
        )
    else:
        result["response"] = True
        result["registers"] = list(rr.registers)
    return result


def _regs_to_ascii(regs: list[int]) -> str:
    """Convert an ASCII register array (1 byte per reg, low byte) to a string.

    Duplicates the tiny helper in drivers/pyrometer.py to keep this module
    CI-safe (see the register-constants note above). Trims at the first
    NUL byte because the pyrometer pads unused name/serial slots with 0x00.
    """
    b = bytes(r & 0xFF for r in regs)
    return b.split(b"\x00", 1)[0].decode("ascii", errors="ignore")


def _regs_to_f32(hi: int, lo: int) -> float:
    """Convert two 16-bit holding registers to IEEE 754 float32 (big-endian).

    Duplicates the helper in drivers/pyrometer.py — see note above.
    """
    raw = (hi << 16) | lo
    return struct.unpack(">f", raw.to_bytes(4, "big"))[0]


def _read_identity(client, device_id: int) -> dict:
    """After a positive lightweight probe, fetch identity + temperature.

    Returns a dict shaped like:
      { "version": "9.3", "name": "EXI4765", "serial": "...",
        "temp_no_swap_C": 601.2, "temp_swap_C": -4e34 }

    All fields best-effort; any read that fails leaves an empty string /
    None. Nothing here changes device state.
    """
    identity: dict = {
        "version": "", "name": "", "serial": "",
        "temp_no_swap_C": None, "temp_swap_C": None,
    }
    ver = _probe_modbus_read(client, REG_VER, 1, device_id)
    if ver["response"] and ver["registers"]:
        v = ver["registers"][0]
        identity["version"] = f"{v >> 8}.{v & 0xFF}"

    name = _probe_modbus_read(client, REG_NAME0, 32, device_id)
    if name["response"] and name["registers"]:
        try:
            identity["name"] = _regs_to_ascii(name["registers"])
        except Exception:  # noqa: BLE001 — decoder never blocks discovery
            pass

    serial = _probe_modbus_read(client, REG_SN0, 9, device_id)
    if serial["response"] and serial["registers"]:
        try:
            identity["serial"] = _regs_to_ascii(serial["registers"])
        except Exception:  # noqa: BLE001
            pass

    temp = _probe_modbus_read(client, REG_CH1_TEMP, 2, device_id)
    if temp["response"] and temp["registers"] and len(temp["registers"]) >= 2:
        try:
            r0, r1 = temp["registers"][0], temp["registers"][1]
            identity["temp_no_swap_C"] = round(_regs_to_f32(r0, r1), 3)
            identity["temp_swap_C"] = round(_regs_to_f32(r1, r0), 3)
        except Exception:  # noqa: BLE001
            pass

    return identity


# ---------------------------------------------------------------------------
# Sweep (open-once-per-baud)
# ---------------------------------------------------------------------------

def sweep_modbus(
    port: str,
    bauds: tuple[int, ...],
    device_ids: tuple[int, ...],
    per_combo_timeout_s: float,
    exhaustive: bool,
) -> tuple[list[dict], list[dict]]:
    """Sweep (baud, device_id) matrix issuing REG_VER read to each combo.

    OPEN-ONCE-PER-BAUD: the ModbusSerialClient is constructed and
    connected once per baud rate; the inner device-ID loop reuses that
    same client. This is much cheaper than open/close per combo (~100 ms
    Windows serial-open × 55 combos would be ~11 s of pure overhead).

    Returns (hits, misses):
      hits   = list of {"baud", "device_id", "identity": {...}} for every
               combo that answered. Identity dict shape per _read_identity.
      misses = list of {"baud", "device_id", "error"} for every combo
               that didn't answer.

    Early-stop: unless ``exhaustive`` is True, the sweep breaks out of
    the inner device_id loop as soon as any device_id answers on the
    current baud (typical case: probe found → stop probing). The outer
    baud loop still continues to catch multi-baud sanity edge cases,
    but the inner sweep exits early.
    """
    hits: list[dict] = []
    misses: list[dict] = []

    try:
        from pymodbus.client import ModbusSerialClient
    except ImportError:
        # Record as a single miss so the verdict logic can distinguish
        # "no pymodbus" from "actual silence."
        misses.append({
            "baud": 0, "device_id": 0,
            "error": "pymodbus not installed (pip install pymodbus pyserial)",
        })
        return hits, misses

    for baud in bauds:
        info(f"Baud {baud} 8N1: opening once for {len(device_ids)} device IDs")
        client = ModbusSerialClient(
            port=port, baudrate=baud, timeout=per_combo_timeout_s,
            bytesize=8, parity="N", stopbits=1,
        )
        if not client.connect():
            # Client wouldn't even connect — record one miss for the whole
            # baud (not per device_id — the failure is baud-level).
            misses.append({
                "baud": baud, "device_id": None,
                "error": "client.connect() failed",
            })
            continue

        found_this_baud = False
        try:
            for device_id in device_ids:
                r = _probe_modbus_read(
                    client, LIGHTWEIGHT_PROBE_REG,
                    LIGHTWEIGHT_PROBE_COUNT, device_id,
                )
                if r["response"]:
                    # Fetch the fuller identity while we still hold the port.
                    identity = _read_identity(client, device_id)
                    hits.append({
                        "baud": baud,
                        "device_id": device_id,
                        "identity": identity,
                    })
                    status(
                        f"id={device_id} @ {baud} 8N1", True,
                        f"ver={identity.get('version') or '?'}, "
                        f"name={identity.get('name') or '?'!r}",
                    )
                    found_this_baud = True
                    if not exhaustive:
                        break
                else:
                    misses.append({
                        "baud": baud,
                        "device_id": device_id,
                        "error": r["error"] or "no response",
                    })
            if not found_this_baud:
                info(f"Baud {baud}: no device_id in {list(device_ids)} responded")
        finally:
            client.close()

    return hits, misses


# ---------------------------------------------------------------------------
# Verdict
# ---------------------------------------------------------------------------

# Default (baud, device_id) combo that ModbusPyrometer + TemperaSure use.
# A hit on this combo means "device is at the driver's expected config"
# and no follow-up action is needed. Anything else is a re-configuration
# discovery.
DEFAULT_COMBO: tuple[int, int] = (115200, 1)


def build_verdict(hits: list[dict], misses: list[dict]) -> tuple[str, str]:
    """Return (state, recovery_hint) based on the sweep outcome.

    States (in the order they're checked):
      * identity_found_at_default     — hit at (115200, 1). Driver is
                                        already correct; you can stop.
      * identity_found_at_non_default — hit at some other combo. Update
                                        driver config OR reconfigure the
                                        device back to the default.
      * silent_all_combos             — no combo answered. Either the
                                        device is in a non-Modbus mode
                                        OR truly offline. P3
                                        (force_modbus) is a candidate
                                        ONLY if TemperaSure confirms the
                                        probe is alive.
      * pymodbus_missing              — the pymodbus package isn't
                                        installed. Not a device issue.
    """
    # Guard: pymodbus_missing surfaces as a single miss with baud=0.
    if len(hits) == 0 and len(misses) == 1 and misses[0].get("baud") == 0:
        return (
            "pymodbus_missing",
            "Install pymodbus first: `pip install pymodbus pyserial`. "
            "This is a Mac/CI-side env issue, not a device issue.",
        )

    if not hits:
        return (
            "silent_all_combos",
            f"No response from device on any of {len(misses)} probed "
            "(baud, device_id) combinations. Likely causes: (a) probe is "
            "in a non-Modbus mode (Exactus binary streaming per BASF "
            "manual pp. 51-52), (b) probe is powered off / cable "
            "unseated, (c) another app holds the port. "
            "If TemperaSure confirms the probe is alive, this is when "
            "pyrometer_force_modbus.py becomes a candidate — see "
            "docs/lab_command_sheet.md.",
        )

    at_default = [
        h for h in hits
        if (h["baud"], h["device_id"]) == DEFAULT_COMBO
    ]
    if at_default:
        h = at_default[0]
        return (
            "identity_found_at_default",
            f"Device answered at the expected (baud, id) = {DEFAULT_COMBO}. "
            f"Identity: ver={h['identity'].get('version') or '?'}, "
            f"name={h['identity'].get('name') or '?'!r}. "
            "ModbusPyrometer will read this device without reconfiguration.",
        )

    # At least one hit, but none at the default combo → reconfigure story.
    combos = [(h["baud"], h["device_id"]) for h in hits]
    return (
        "identity_found_at_non_default",
        f"Device answered at {combos} instead of the default "
        f"{DEFAULT_COMBO}. Either update ModbusPyrometer construction to "
        "match, OR reconfigure the device back to (115200, 1) via "
        "TemperaSure. This script does NOT write config registers — "
        "reconfiguration is manual.",
    )


# ---------------------------------------------------------------------------
# Evidence bundle (per-script; do NOT reuse pyrometer_physical_debug's)
# ---------------------------------------------------------------------------

def write_evidence_bundle(
    output_dir: Path,
    args_summary: dict,
    environment: dict,
    hits: list[dict],
    misses: list[dict],
    state: str,
    hint: str,
) -> dict[str, Path]:
    """Persist the run's structured results for share/paste-back.

    JSON schema is specific to this script's (baud, device_id) sweep —
    deliberately different from pyrometer_physical_debug's
    phase_1_port / phase_2_exactus / phase_3_modbus shape. See plan P2's
    "Evidence-bundle writer" note for the reasoning.
    """
    output_dir.mkdir(parents=True, exist_ok=True)
    result = {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "args": args_summary,
        "phase_0_env": environment,
        "phase_1_sweep_hits": hits,
        "phase_2_sweep_misses": misses,
        "verdict": {"state": state, "recovery_hint": hint},
    }
    result_path = output_dir / "result.json"
    result_path.write_text(json.dumps(result, indent=2, default=str))
    return {"result.json": result_path}


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _parse_int_list(arg: str) -> tuple[int, ...]:
    """Parse a comma-separated list of ints (e.g. '115200,19200')."""
    return tuple(int(x.strip()) for x in arg.split(",") if x.strip())


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Pyrometer Modbus discovery — READ-ONLY (baud × device_id) sweep. "
            "Broader than pyrometer_physical_debug's default (115200 8N1, "
            "device_id=1). Runs BEFORE any WRITE attempt."
        ),
    )
    parser.add_argument(
        "--port", default="COM4",
        help="Serial port (default: COM4)",
    )
    parser.add_argument(
        "--bauds", type=_parse_int_list, default=None,
        help=(
            "Comma-separated baud list (default: 115200,57600,38400,19200,9600). "
            "Ordered fast → slow so the common case probes first."
        ),
    )
    parser.add_argument(
        "--device-ids", type=_parse_int_list, default=None,
        help=(
            "Comma-separated Modbus slave IDs (default: 1-10 plus 247). "
            "Address 0 is Modbus broadcast and is intentionally rejected."
        ),
    )
    parser.add_argument(
        "--initial-wait-s", type=float, default=0.0,
        help=(
            "One-shot wait before the sweep begins (default: 0.0). Set to "
            "6-10 IF you just power-cycled the probe — BASF manual p. 51 "
            "documents a 5-10 s self-init after power-up. This is NOT a "
            "per-serial-open event, so we sleep ONCE, not per combo."
        ),
    )
    parser.add_argument(
        "--per-combo-timeout-s", type=float, default=1.0,
        help=(
            "pymodbus timeout for each read (default: 1.0). Fast fail is "
            "what makes the sweep tractable — a naive 55-combo sweep with "
            "a 3s timeout takes 165s per silent baud."
        ),
    )
    parser.add_argument(
        "--output-dir", type=Path, default=None,
        help="If set, write result.json for paste-back / archival.",
    )
    parser.add_argument(
        "--exhaustive", action="store_true",
        help=(
            "Do the full baud × device_id matrix instead of early-stopping "
            "on the first hit per baud. Useful for debugging or when "
            "multiple devices share the bus."
        ),
    )
    args = parser.parse_args(argv)

    bauds = args.bauds if args.bauds is not None else CANDIDATE_BAUDS
    device_ids = (
        args.device_ids if args.device_ids is not None
        else CANDIDATE_DEVICE_IDS
    )

    # SAFETY: refuse address 0 up front regardless of user override. Modbus
    # broadcast (address 0) is a write-only convention; issuing a read to
    # it is spec-undefined. Better to reject with a clear message than
    # let pymodbus produce garbage.
    if 0 in device_ids:
        parser.error(
            "device_id=0 is Modbus broadcast (write-only convention per "
            "Modbus RTU spec §4.1.1). Reject to keep the sweep honest — "
            "use --device-ids to explicitly enumerate the addresses you "
            "want to probe."
        )

    print("═══ Pyrometer Modbus discovery — READ-ONLY ═══")
    print(f"Port:               {args.port}")
    print(f"Bauds:              {list(bauds)}")
    print(f"Device IDs:         {list(device_ids)} "
          f"(broadcast address 0 excluded)")
    print(f"Per-combo timeout:  {args.per_combo_timeout_s}s")
    print(f"Initial wait:       {args.initial_wait_s}s "
          f"({'post-power-cycle' if args.initial_wait_s > 0 else 'none'})")
    print(f"Exhaustive:         {args.exhaustive}")
    print("Safety:             Modbus function 0x03 only; data registers only "
          "(REG_VER/NAME/SN/CH1_TEMP); no config regs; no writes")

    # --- Phase 0: Environment inventory -----------------------------------
    section("Phase 0: Environment")
    ports = list_available_ports()
    if ports:
        info(f"{len(ports)} serial port(s) visible to pyserial:")
        target_in_list = False
        for p in ports:
            marker = ""
            if p["device"] == args.port:
                marker = "   ← --port target"
                target_in_list = True
            print(f"    {p['device']:>10}  {p['description']}{marker}")
        if not target_in_list:
            print()
            print(
                f"  [WARN] --port {args.port!r} is NOT in the visible ports "
                "list. Consider one of: "
                f"{sorted({p['device'] for p in ports})}"
            )
    else:
        info(
            "Serial port enumeration unavailable "
            "(pyserial not installed or list_ports failed)"
        )
    conflicts = describe_conflicting_processes()
    if conflicts:
        print()
        info(
            f"{len(conflicts)} process(es) matching known port-hoggers "
            "(TemperaSure / BASF / MistralGui / kSA):"
        )
        for line in conflicts:
            print(f"    {line}")
    environment_snapshot = {
        "available_ports": ports,
        "conflicting_processes": conflicts,
        "target_port_visible": any(p["device"] == args.port for p in ports),
    }

    # --- Phase 0.5: Optional one-shot initial wait ------------------------
    if args.initial_wait_s > 0:
        section(f"Phase 0.5: Post-power-cycle wait ({args.initial_wait_s}s)")
        info(
            f"Sleeping {args.initial_wait_s}s ONCE before sweep. "
            "Per BASF manual p. 51, the probe takes 5-10s to self-init "
            "after power-up; this wait is not per-combo."
        )
        time.sleep(args.initial_wait_s)

    # --- Phase 1: Sweep ---------------------------------------------------
    section(
        f"Phase 1: Modbus sweep (function 0x03 only, "
        f"{len(bauds)}×{len(device_ids)} = {len(bauds)*len(device_ids)} "
        "combos max)"
    )
    hits, misses = sweep_modbus(
        args.port, bauds, device_ids,
        args.per_combo_timeout_s, args.exhaustive,
    )
    print()
    info(f"{len(hits)} hit(s), {len(misses)} miss(es)")

    # --- Verdict ----------------------------------------------------------
    section("Verdict")
    state, hint = build_verdict(hits, misses)
    print(f"State:    {state}")
    print(f"Recovery: {hint}")

    if hits:
        print()
        info(f"{len(hits)} device identity/identities discovered:")
        for h in hits:
            id_ = h["identity"]
            print(
                f"    (baud={h['baud']}, id={h['device_id']})  "
                f"ver={id_.get('version') or '?'}  "
                f"name={id_.get('name') or '?'!r}  "
                f"sn={id_.get('serial') or '?'!r}  "
                f"temp_no_swap={id_.get('temp_no_swap_C')}°C  "
                f"temp_swap={id_.get('temp_swap_C')}°C"
            )

    # --- Evidence bundle (if requested) -----------------------------------
    if args.output_dir:
        written = write_evidence_bundle(
            args.output_dir, vars(args), environment_snapshot,
            hits, misses, state, hint,
        )
        print()
        print("Evidence bundle written:")
        for name, path in written.items():
            print(f"  {name}: {path}")

    # Exit 0 if we found the device (either combo); 1 for silent /
    # missing dep — matches pyrometer_physical_debug's convention.
    if state in ("identity_found_at_default", "identity_found_at_non_default"):
        return 0
    return 1


if __name__ == "__main__":
    sys.exit(main())
