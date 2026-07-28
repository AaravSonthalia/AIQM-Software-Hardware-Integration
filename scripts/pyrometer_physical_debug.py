"""Pyrometer physical-state debug — READ-ONLY diagnostic for BASF Exactus.

Purpose: on any lab day where the pyrometer has been silent, run this
against the wired COM port BEFORE any driver-level testing. Distinguishes:

  * exactus_streaming     — device pushing binary packets (passive listen)
  * modbus_responsive     — device answers Modbus function 0x03 reads
  * transmitting_unknown  — bytes on the line but neither protocol matches
  * silent                — port opens, nothing received, no Modbus response
  * port_down             — cannot open the serial port at all
  * both_active_unexpected — Exactus streaming AND Modbus response (unusual)

For each state, prints a specific recovery hint (physical inspection
targets, mode-switch guidance, cable/converter checks).

════════════════════════════════════════════════════════════════════════
READ-ONLY invariant — do not weaken:

  * Modbus phase issues ONLY Modbus function code 0x03 (Read Holding
    Registers). Never 0x06 (Write Single) or 0x10 (Write Multiple).
    Reading Modbus holding registers does NOT modify device state per
    the Modbus RTU spec §7.3.
  * Exactus phase is passive listen only. Never sends the mode-switch,
    reset, or reboot commands.
  * Never touches REG_ADDR (0x1007), REG_BAUD (0x1008), REG_RATE (0x1011),
    or REG_CMD (0x8000) — even for reads. Reading them is technically
    safe, but keeping them out entirely means a future edit that
    accidentally adds a write to one of these addresses is caught early.
  * If you need to CHANGE device state (switch to Modbus, reset, set rate),
    that requires a separate write-capable script — deliberately not this
    one. Get explicit sign-off before writing to any pyrometer register.
════════════════════════════════════════════════════════════════════════

Usage (Bulbasaur):
    python scripts\\pyrometer_physical_debug.py                    # defaults
    python scripts\\pyrometer_physical_debug.py --port COM3
    python scripts\\pyrometer_physical_debug.py --exactus-listen-s 10
    python scripts\\pyrometer_physical_debug.py --skip-modbus
    python scripts\\pyrometer_physical_debug.py --skip-exactus
    python scripts\\pyrometer_physical_debug.py --device-id 2
"""
from __future__ import annotations

import argparse
import struct
import sys
import time
from pathlib import Path
from typing import Optional

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

# Reuse the register-map constants + decoders from the production driver.
# If the driver's understanding of the register map ever changes, this
# script stays in sync automatically.
from drivers.pyrometer import (  # noqa: E402
    REG_CH1_TEMP,
    REG_NAME0,
    REG_VER,
    _regs_to_ascii,
    _regs_to_f32,
)


# ---------------------------------------------------------------------------
# READ-ONLY safety constants
# ---------------------------------------------------------------------------

# Only these Modbus function codes are used. The BASF Exactus register
# map (per drivers/pyrometer.py) uses holding registers exclusively —
# every field the driver reads is at 0x0000-0x1305 and served by function
# 0x03. Function 0x04 (Read Input Registers) is also read-only per the
# RTU spec, but we don't need it and keeping the allowlist tight means a
# future editor who wants to add it has to think about it. See the
# module docstring's READ-ONLY invariant section for the full rationale.
READ_ONLY_MODBUS_FUNCTIONS: tuple[int, ...] = (0x03,)

# Configuration registers this script MUST NOT touch, even for reads.
# Reading them is technically safe, but keeping them out entirely means
# a future edit that accidentally adds a write for one of these addresses
# is caught by the SAFETY guard in _probe_modbus_read.
CONFIG_REGISTERS_FORBIDDEN: tuple[int, ...] = (
    0x1007,  # REG_ADDR  — device Modbus slave address (bricking risk)
    0x1008,  # REG_BAUD  — baud rate (bricking risk)
    0x1011,  # REG_RATE  — sample rate
    0x8000,  # REG_CMD   — command register (CMD_SAVE / CMD_REBOOT)
)

# Exactus binary packet header byte (per the ExactusSerialPyrometer driver).
EXACTUS_HEADER = 0x81

# Candidate baud rates for the Modbus probe phase. 115200 first because
# it matches ModbusPyrometer's default and the observed Bulbasaur config;
# 19200 and 9600 as fallbacks in case the device was reconfigured.
CANDIDATE_MODBUS_BAUDS: tuple[int, ...] = (115200, 19200, 9600)


# ---------------------------------------------------------------------------
# Terminal helpers
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
# Environment inventory (pre-Phase 1) — surfaces the "wrong COM port" and
# "another process holds the port" failure modes before we even try to open.
# ---------------------------------------------------------------------------

def list_available_ports() -> list[dict]:
    """Enumerate serial ports the OS currently exposes via pyserial.

    Returns a list of ``{"device", "description", "hwid"}`` dicts, or
    ``[]`` if pyserial is missing. Never raises — this is diagnostic
    context, not a hard dependency.
    """
    try:
        from serial.tools import list_ports  # noqa: PLC0415
    except ImportError:
        return []
    ports = []
    for p in list_ports.comports():
        ports.append({
            "device": p.device,
            "description": p.description or "",
            "hwid": p.hwid or "",
        })
    return ports


def describe_conflicting_processes() -> list[str]:
    """Best-effort Windows-only grep of ``tasklist`` for known port-hoggers.

    TemperaSure, BASF-anything, MistralGui, and kSA can each hold a COM
    port exclusively. If our port-open fails, seeing one of these in the
    running process list points at the culprit immediately. Returns the
    matching tasklist CSV lines (stripped) or ``[]`` on non-Windows /
    tasklist unavailable — never raises.
    """
    import platform
    if platform.system() != "Windows":
        return []
    import subprocess
    keywords = ("TemperaSure", "BASF", "MistralGui", "kSA")
    try:
        out = subprocess.run(
            ["tasklist", "/FO", "CSV", "/NH"],
            capture_output=True, text=True, timeout=5.0,
        )
    except Exception:  # noqa: BLE001 — never let diagnostics crash the script
        return []
    if out.returncode != 0:
        return []
    hits: list[str] = []
    for line in out.stdout.splitlines():
        lower = line.lower()
        for kw in keywords:
            if kw.lower() in lower:
                hits.append(line.strip())
                break
    return hits


# ---------------------------------------------------------------------------
# Evidence bundle writer
# ---------------------------------------------------------------------------

def write_evidence_bundle(
    output_dir: Path,
    args_summary: dict,
    port_open: bool,
    port_detail: str,
    environment: dict,
    exactus: dict,
    modbus_results: list[dict],
    state: str,
    hint: str,
) -> dict[str, Path]:
    """Persist the run's structured results for share/paste-back.

    Writes ``result.json`` always. Writes ``raw_sniff.bin`` only if the
    Exactus phase captured bytes (so an empty run doesn't leave a stray
    empty file). Returns a dict of written paths for the CLI to print.

    Rationale (Codex Jul 28 review): stdout output can be lost to Slack
    truncation or copy-paste mistakes; a structured JSON evidence bundle
    is what we actually paste back for review.
    """
    import json
    from datetime import datetime, timezone

    output_dir.mkdir(parents=True, exist_ok=True)
    written: dict[str, Path] = {}

    # Extract the raw byte buffer before JSON serialization (bytes aren't
    # JSON-safe). Write it to a separate .bin only if non-empty.
    exactus_copy = dict(exactus)
    raw_bytes = exactus_copy.pop("_raw_bytes", None)
    if raw_bytes:
        raw_path = output_dir / "raw_sniff.bin"
        raw_path.write_bytes(raw_bytes)
        written["raw_sniff.bin"] = raw_path

    result = {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "args": args_summary,
        "environment": environment,
        "phase_1_port": {"open": port_open, "detail": port_detail},
        "phase_2_exactus": exactus_copy,
        "phase_3_modbus": modbus_results,
        "verdict": {"state": state, "recovery_hint": hint},
    }
    result_path = output_dir / "result.json"
    result_path.write_text(json.dumps(result, indent=2, default=str))
    written["result.json"] = result_path
    return written


# ---------------------------------------------------------------------------
# Phase 1: Serial port availability
# ---------------------------------------------------------------------------

def phase_port_available(port: str, baud: int = 115200) -> tuple[bool, str]:
    """Try to open the serial port; report whether the OS handed it over.

    Uses a short timeout so a wedged port fails fast. Closes immediately —
    we're just checking whether the port CAN be opened. Any data buffered
    at the OS level is deliberately NOT consumed here (that's Phase 2's job).
    """
    try:
        import serial
    except ImportError:
        return False, "pyserial not installed (pip install pyserial)"
    try:
        ser = serial.Serial(port, baud, timeout=0.5)
        ser.close()
    except Exception as exc:  # noqa: BLE001 — pyserial can raise many types
        return False, f"{type(exc).__name__}: {exc}"
    return True, f"opened {port} at {baud} 8N1 and closed cleanly"


# ---------------------------------------------------------------------------
# Phase 2: Passive Exactus listen
# ---------------------------------------------------------------------------

def _is_plausible_temperature(t: float) -> bool:
    """Predicate for the dual-endian plausibility check.

    Range matches the driver's ``TEMP_MIN_C`` / ``TEMP_MAX_C`` bounds but
    additionally rejects near-zero magnitudes (< 1e-10 °C). Rationale: LE
    decodes of valid BE floats (and vice versa) almost always land in the
    IEEE 754 denormal zone (~1e-39 magnitude). Without the near-zero
    filter, those denormals would incorrectly be flagged as "plausible"
    and defeat the whole point of dual-endian disambiguation. A real
    pyrometer reading of exactly 0 °C or 1 nK is unphysical anyway.
    """
    if not (-100.0 < t < 3000.0):
        return False
    if abs(t) < 1e-10:
        return False
    return True


def _parse_exactus_samples(buffer: bytes, max_samples: int = 5) -> list[dict]:
    """Best-effort parse of Exactus 0x81-headered samples in BOTH endian orderings.

    A diagnostic script shouldn't narrow to the driver's big-endian
    assumption too early — if the device is actually little-endian, we
    want to see it. Each returned dict is:

      { "offset": int, "be": float, "le": float,
        "be_plausible": bool, "le_plausible": bool }

    A sample is INCLUDED if either endian decode is plausible, so the
    operator sees the full picture rather than a filtered one. See
    ``_is_plausible_temperature`` for what "plausible" means (and why
    denormals are excluded).
    """
    samples: list[dict] = []
    i = 0
    while i < len(buffer) - 4 and len(samples) < max_samples:
        if buffer[i] == EXACTUS_HEADER:
            body = bytes(buffer[i + 1:i + 5])
            try:
                be = struct.unpack(">f", body)[0]
                le = struct.unpack("<f", body)[0]
            except struct.error:
                i += 1
                continue
            be_plausible = _is_plausible_temperature(be)
            le_plausible = _is_plausible_temperature(le)
            if be_plausible or le_plausible:
                samples.append({
                    "offset": i,
                    "be": round(be, 3),
                    "le": round(le, 3),
                    "be_plausible": be_plausible,
                    "le_plausible": le_plausible,
                })
                i += 5
                continue
        i += 1
    return samples


def phase_passive_exactus_listen(port: str, listen_s: float) -> dict:
    """Open at 115200 and PASSIVELY listen for Exactus binary packets.

    No bytes are sent from the host. Returns:
      { "opened": bool, "bytes_received": int, "exactus_headers": int,
        "sample_temperatures": list[dict], "raw_bytes_hex_head": str,
        "_raw_bytes": bytes | None, "error": str }

    ``sample_temperatures`` contains dual-endian decodes per constraint
    from Codex's Jul 28 review — see ``_parse_exactus_samples`` for the
    dict shape. ``_raw_bytes`` is the entire captured buffer; the CLI
    pops this key before serializing to JSON (bytes aren't JSON-safe) and
    optionally writes it to ``raw_sniff.bin`` via ``--output-dir``.
    ``raw_bytes_hex_head`` is the first 128 bytes hex-encoded, retained
    in the JSON so the evidence bundle is self-describing without needing
    the .bin file alongside.
    """
    result: dict = {
        "opened": False,
        "bytes_received": 0,
        "exactus_headers": 0,
        "sample_temperatures": [],
        "raw_bytes_hex_head": "",
        "_raw_bytes": None,
        "error": "",
    }
    try:
        import serial
    except ImportError:
        result["error"] = "pyserial not installed"
        return result
    try:
        with serial.Serial(port, 115200, timeout=0.1) as ser:
            result["opened"] = True
            deadline = time.time() + listen_s
            buffer = bytearray()
            while time.time() < deadline:
                chunk = ser.read(64)
                if chunk:
                    buffer.extend(chunk)
            raw = bytes(buffer)
            result["bytes_received"] = len(raw)
            result["exactus_headers"] = raw.count(EXACTUS_HEADER)
            result["sample_temperatures"] = _parse_exactus_samples(raw)
            result["raw_bytes_hex_head"] = raw[:128].hex()
            result["_raw_bytes"] = raw
    except Exception as exc:  # noqa: BLE001
        result["error"] = f"{type(exc).__name__}: {exc}"
    return result


# ---------------------------------------------------------------------------
# Phase 3: Modbus request-response (READ-ONLY)
# ---------------------------------------------------------------------------

def _probe_modbus_read(client, addr: int, count: int, device_id: int) -> dict:
    """Issue ONE Modbus function-code-0x03 read. Refuses forbidden addresses.

    SAFETY: addr is checked against CONFIG_REGISTERS_FORBIDDEN before any
    wire traffic. This gate exists so a future edit that accidentally adds
    a probe for a config register (which itself is safe to READ but
    high-blast-radius to write) raises before touching the device.
    """
    if addr in CONFIG_REGISTERS_FORBIDDEN:
        raise RuntimeError(
            f"SAFETY: refusing to read config register 0x{addr:04X} "
            "(in CONFIG_REGISTERS_FORBIDDEN). See module docstring for "
            "the reasoning behind the ban."
        )
    result = {"response": False, "registers": None, "error": ""}
    # pymodbus renamed the device-ID kwarg between versions (unit → slave
    # → device_id); match the driver's compat pattern.
    rr = None
    for kw in ("device_id", "slave", "unit"):
        try:
            rr = client.read_holding_registers(addr, count, **{kw: device_id})
            break
        except TypeError:
            continue
    if rr is None:
        rr = client.read_holding_registers(
            address=addr, count=count, device_id=device_id,
        )
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


def _decode_probe(name: str, registers: Optional[list[int]]) -> str:
    """Human-readable decode of a Modbus read result for the status line."""
    if registers is None:
        return ""
    try:
        if name == "REG_VER":
            v = registers[0]
            return f"version {v >> 8}.{v & 0xFF}"
        if name == "REG_NAME0":
            return f"name={_regs_to_ascii(registers)!r}"
        if name == "REG_CH1_TEMP":
            # Report BOTH word orders — the driver's connect() auto-detects,
            # so a debug script that guesses would mask the ambiguity. We
            # display both so the operator sees which one is plausible.
            t_no_swap = _regs_to_f32(registers[0], registers[1])
            t_swap = _regs_to_f32(registers[1], registers[0])
            return (
                f"temp: word_swap=False→{t_no_swap:.2f}°C, "
                f"word_swap=True→{t_swap:.2f}°C"
            )
    except Exception as exc:  # noqa: BLE001
        return f"decode error: {exc}"
    return ""


def phase_modbus_probe(port: str, device_id: int) -> list[dict]:
    """Sweep candidate bauds; per baud, probe VER → NAME → CH1_TEMP.

    Returns a list of per-probe dicts:
      { "baud": int, "register_name": str, "response": bool,
        "registers": list[int] | None, "error": str, "decoded": str }

    Stops trying more bauds once one baud gets any response — the device
    is only configured for one baud at a time, so continuing to sweep
    other bauds after a success just adds noise.
    """
    results: list[dict] = []
    try:
        from pymodbus.client import ModbusSerialClient
    except ImportError:
        results.append({
            "baud": 0, "register_name": "-", "response": False,
            "registers": None, "error": "pymodbus not installed",
            "decoded": "",
        })
        return results

    for baud in CANDIDATE_MODBUS_BAUDS:
        info(f"Trying baud {baud} 8N1")
        client = ModbusSerialClient(
            port=port, baudrate=baud, timeout=1.5,
            bytesize=8, parity="N", stopbits=1,
        )
        if not client.connect():
            results.append({
                "baud": baud, "register_name": "-", "response": False,
                "registers": None, "error": "client.connect() failed",
                "decoded": "",
            })
            continue
        try:
            probes = (
                ("REG_VER", REG_VER, 1),
                ("REG_NAME0", REG_NAME0, 32),
                ("REG_CH1_TEMP", REG_CH1_TEMP, 2),
            )
            any_response_this_baud = False
            for name, addr, count in probes:
                r = _probe_modbus_read(client, addr, count, device_id)
                r["baud"] = baud
                r["register_name"] = name
                r["decoded"] = _decode_probe(name, r["registers"])
                results.append(r)
                if r["response"]:
                    any_response_this_baud = True
            if any_response_this_baud:
                # Found the device's baud; no need to try the others.
                break
        finally:
            client.close()
    return results


# ---------------------------------------------------------------------------
# Verdict
# ---------------------------------------------------------------------------

def build_verdict(
    port_open: bool,
    exactus: dict,
    modbus_results: list[dict],
) -> tuple[str, str]:
    """Return (state, recovery_hint) based on the phase outcomes.

    States (in the same order they're checked):
      * port_down             — Phase 1 failed
      * both_active_unexpected — Exactus streaming AND Modbus responded
      * exactus_streaming     — 0x81 headers seen
      * modbus_responsive     — any Modbus probe returned data
      * transmitting_unknown  — bytes received, no headers, no Modbus
      * silent                — port opens, everything else empty
    """
    modbus_any = any(r["response"] for r in modbus_results)
    exactus_streaming = exactus.get("exactus_headers", 0) > 0
    exactus_any_bytes = exactus.get("bytes_received", 0) > 0

    if not port_open:
        return (
            "port_down",
            "Serial port could not be opened. Verify: (a) IFD-5 USB "
            "converter is plugged in, (b) Windows Device Manager assigns "
            "it to the expected COM port, (c) no other process "
            "(TemperaSure, serial monitor, Growth Monitor) holds the "
            "port. Replug the USB cable and re-run.",
        )

    if exactus_streaming and modbus_any:
        return (
            "both_active_unexpected",
            "Both Exactus streaming AND Modbus response detected — "
            "unusual. Devices typically operate in one mode at a time. "
            "Report this to the group; meanwhile either driver path "
            "should work.",
        )

    if exactus_streaming:
        samples = exactus.get("sample_temperatures", [])
        be_values = [s["be"] for s in samples if s.get("be_plausible")]
        le_values = [s["le"] for s in samples if s.get("le_plausible")]
        sample_str = ""
        if be_values and le_values:
            sample_str = (
                f"; BE decode {be_values} °C, LE decode {le_values} °C "
                "— BOTH endians produce plausible temps, so the "
                "endianness assumption in the driver (`>f`) needs "
                "cross-check with device documentation"
            )
        elif be_values:
            sample_str = f"; sample values (BE) {be_values} °C"
        elif le_values:
            sample_str = (
                f"; sample values (LE) {le_values} °C — driver assumes "
                "big-endian (`>f`), consider checking whether the device "
                "is actually little-endian"
            )
        return (
            "exactus_streaming",
            f"Device is in Exactus mode and pushing packets "
            f"({exactus['exactus_headers']} headers detected"
            f"{sample_str}). Use ExactusSerialPyrometer to consume the "
            "stream. Modbus probes failed because the device isn't in "
            "Modbus mode right now — switching modes requires a WRITE "
            "command (out of scope for this read-only script).",
        )

    if modbus_any:
        working = [r for r in modbus_results if r["response"]]
        bauds = sorted({r["baud"] for r in working})
        return (
            "modbus_responsive",
            f"Device responds to Modbus at baud {bauds}. Use "
            "ModbusPyrometer with the matching baud rate. If Exactus mode "
            "was expected, the device is currently configured for Modbus "
            "RTU instead — switching modes requires a WRITE command (out "
            "of scope for this read-only script).",
        )

    if exactus_any_bytes:
        return (
            "transmitting_unknown",
            f"Device is transmitting ({exactus['bytes_received']} bytes) "
            "but no Exactus 0x81 headers detected and no Modbus response. "
            "Possible causes: wrong baud rate, unknown protocol, or line "
            "noise. Capture raw bytes with a serial monitor at 115200 8N1 "
            "and inspect for framing patterns.",
        )

    return (
        "silent",
        "Port opens, but the device transmitted nothing and did not "
        "respond to Modbus at any tried baud. Likely causes: (a) "
        "pyrometer powered off, (b) IFD-5 converter unplugged from the "
        "pyrometer end (cable-to-device, not cable-to-PC), (c) device in "
        "an unexpected state after TemperaSure was closed, (d) hardware "
        "failure. Verify: pyrometer power LED, IFD-5 both cable ends "
        "firmly seated, TemperaSure app fully closed (also check the "
        "system tray).",
    )


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Pyrometer physical-state debug — READ-ONLY diagnostic. "
            "Distinguishes Exactus streaming, Modbus responsive, silent, "
            "transmitting-unknown, and physical-layer-down states "
            "without changing device configuration."
        ),
    )
    parser.add_argument(
        "--port", default="COM4",
        help="Serial port (default: COM4 — the Bulbasaur pyrometer wiring)",
    )
    parser.add_argument(
        "--exactus-listen-s", type=float, default=5.0,
        help="Seconds to passively listen for Exactus packets (default: 5.0)",
    )
    parser.add_argument(
        "--skip-modbus", action="store_true",
        help="Skip the Modbus request-response phase (Exactus-only run)",
    )
    parser.add_argument(
        "--skip-exactus", action="store_true",
        help="Skip the Exactus passive-listen phase (Modbus-only run)",
    )
    parser.add_argument(
        "--device-id", type=int, default=1,
        help="Modbus device (slave) ID (default: 1 — the driver default)",
    )
    parser.add_argument(
        "--output-dir", type=Path, default=None,
        help=(
            "If set, write result.json (structured phase results + verdict) "
            "and raw_sniff.bin (Exactus buffer, if bytes were captured) "
            "for paste-back / archival."
        ),
    )
    args = parser.parse_args(argv)

    print("═══ Pyrometer physical debug — READ-ONLY ═══")
    print(f"Port:             {args.port}")
    print(f"Modbus device ID: {args.device_id}")
    print("Safety:           Modbus function 0x03 only; passive Exactus listen")
    print("                  (no writes; no config-register access; no mode-switch)")

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

    # --- Phase 1 ----------------------------------------------------------
    section("Phase 1: Serial port availability")
    port_open, port_detail = phase_port_available(args.port)
    status(f"Open {args.port} at 115200 8N1", port_open, port_detail)
    if not port_open:
        # No point running further phases — port is unusable.
        state, hint = build_verdict(False, {}, [])
        section("Verdict")
        print(f"State:    {state}")
        print(f"Recovery: {hint}")
        if args.output_dir:
            written = write_evidence_bundle(
                args.output_dir, vars(args), port_open, port_detail,
                environment_snapshot, {}, [], state, hint,
            )
            print()
            print("Evidence bundle written:")
            for name, path in written.items():
                print(f"  {name}: {path}")
        return 1

    # --- Phase 2 ----------------------------------------------------------
    exactus: dict = {
        "bytes_received": 0, "exactus_headers": 0,
        "sample_temperatures_c": [],
    }
    if args.skip_exactus:
        section("Phase 2: Passive Exactus listen — SKIPPED (--skip-exactus)")
    else:
        section(
            f"Phase 2: Passive Exactus listen "
            f"({args.exactus_listen_s}s at 115200)"
        )
        info(f"Listening for {args.exactus_listen_s}s; no bytes sent to device")
        exactus = phase_passive_exactus_listen(args.port, args.exactus_listen_s)
        if exactus.get("error"):
            status("Exactus passive listen", False, exactus["error"])
        else:
            detail = (
                f"{exactus['bytes_received']} bytes received, "
                f"{exactus['exactus_headers']} Exactus 0x81 headers"
            )
            if exactus.get("sample_temperatures_c"):
                detail += (
                    f"; sample temps {exactus['sample_temperatures_c']} °C"
                )
            status(
                "Exactus passive listen",
                exactus["exactus_headers"] > 0,
                detail,
            )

    # --- Phase 3 ----------------------------------------------------------
    modbus_results: list[dict] = []
    if args.skip_modbus:
        section("Phase 3: Modbus probes — SKIPPED (--skip-modbus)")
    else:
        section(
            "Phase 3: Modbus probes (function 0x03 only, READ-ONLY, "
            "device_id=%d)" % args.device_id
        )
        modbus_results = phase_modbus_probe(args.port, args.device_id)
        for r in modbus_results:
            label = f"{r['register_name']} @ {r['baud']} 8N1"
            if r["response"]:
                status(
                    label, True,
                    r.get("decoded", "") or f"regs={r['registers']}",
                )
            else:
                status(label, False, r["error"] or "no response")

    # --- Verdict ----------------------------------------------------------
    section("Verdict")
    state, hint = build_verdict(port_open, exactus, modbus_results)
    print(f"State:    {state}")
    print(f"Recovery: {hint}")

    # --- Evidence bundle (if requested) -----------------------------------
    if args.output_dir:
        written = write_evidence_bundle(
            args.output_dir, vars(args), port_open, port_detail,
            environment_snapshot, exactus, modbus_results, state, hint,
        )
        print()
        print("Evidence bundle written:")
        for name, path in written.items():
            print(f"  {name}: {path}")

    # Exit 0 if we learned the device is talking (either mode);
    # exit 1 if further investigation is required.
    if state in ("exactus_streaming", "modbus_responsive", "both_active_unexpected"):
        return 0
    return 1


if __name__ == "__main__":
    sys.exit(main())
