#!/usr/bin/env python3
"""Raw-byte Modbus RTU probe for the BASF Exactus pyrometer — READ-ONLY.

Purpose: see what is ACTUALLY on the wire in response to a correctly
addressed BASF Modbus read. Every Modbus attempt in this repo so far has
gone through pymodbus, which silently discards bytes that do not validate
as a response frame and reports a timeout. On a line with local echo that
behaviour is indistinguishable from a dead device:

    we send    01 03 13 00 00 01 <crc>
    line echoes it back
    pymodbus reads 01 (slave) 03 (function) 13 (byte count = 19!)
    ...waits for 19 more bytes, times out, discards everything
    ...reports "No response received after 3 retries"

If the probe HAD answered, its response would be sitting in the buffer
right behind the echo and pymodbus would never have looked at it. This
script does no framing and no validation before capture: it writes the
request, then reads every byte that arrives during the response window
and reports the lot as hex.

Motivating evidence (Jul 30 2026, Bulbasaur COM4): the active Exactus
query returned `echo_of_sent` — we wrote `02 56 56 03` and received
exactly `02 56 56 03`. Local echo on this path is therefore confirmed,
which means every prior `silent` / `silent_all_combos` verdict was
measured through a polluted receive path.

WIRE CONTRACT (read this before editing)
----------------------------------------
* Modbus function 0x03 (Read Holding Registers) ONLY. Function 0x03 is a
  read; it cannot change device state.
* An ``ALLOWED_REGISTERS`` whitelist is enforced BEFORE the port opens.
  Only data registers appear in it. The config registers (0x1007 device
  address, 0x1008 baud, 0x1011 sample rate) and the command register
  (0x8000, which takes CMD_SAVE / CMD_REBOOT) are deliberately absent —
  reading them is harmless but keeping them out of this script keeps the
  read surface small and auditable, matching pyrometer_modbus_discover.
* No writes. No coil functions. No mode switches. Adding any of those
  belongs in a differently named script with its own safety gate, the
  same way pyrometer_force_modbus.py is separate from the read-only
  tools.

Register addresses are from the BASF Exactus User Manual v5.04 "Modbus
Register Address Map" (page 55), verified against drivers/pyrometer.py
on Jul 30 2026.

Float format, manual page 53: values are 32-bit IEEE-754 occupying two
registers, upper 16 bits at the even (lower) address. The manual's worked
example is 252.55 C -> 437C8CCC stored as 0x0000=437C, 0x0001=8CCC. We
decode that documented order as primary and also report the word-swapped
interpretation, because drivers/pyrometer.py currently auto-detects word
order at runtime and this is our first chance to confirm which is right
against real hardware.

Usage on Bulbasaur (TemperaSure MUST be closed — COM4 is exclusive):

    python scripts\\pyrometer_raw_modbus_probe.py --dry-run
    python scripts\\pyrometer_raw_modbus_probe.py --output-dir C:\\lab_evidence\\2026-07-30\\raw_modbus
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

# --- Wire constants ------------------------------------------------------

DEFAULT_PORT = "COM4"
DEFAULT_BAUD = 115200
DEFAULT_DEVICE_ID = 1
DEFAULT_RESPONSE_WINDOW_S = 1.0

FUNCTION_READ_HOLDING = 0x03

# Data registers only — see the WIRE CONTRACT note above for why the
# config and command registers are excluded rather than merely unused.
# name -> (address, register_count, description)
ALLOWED_REGISTERS: dict[str, tuple[int, int, str]] = {
    "REG_VER":       (0x1300, 1, "Software version (u16: high=major, low=minor)"),
    "REG_CH1_TEMP":  (0x0000, 2, "Channel 1 temperature (IEEE-754, 2 regs)"),
    "REG_CH1_CURR":  (0x0004, 2, "Channel 1 current (IEEE-754, 2 regs)"),
    "REG_AMBIENT":   (0x0800, 2, "Chassis ambient temperature (IEEE-754, 2 regs)"),
    "REG_SN0":       (0x1305, 9, "Serial number (9 ASCII bytes, 1 per register)"),
}

# Probed in this order by default: version first because it is the
# cheapest possible valid read (1 register, 7-byte reply), then the
# temperature that actually matters.
DEFAULT_PROBE_SEQUENCE = ["REG_VER", "REG_CH1_TEMP"]


def crc16_modbus(payload: bytes) -> int:
    """CRC-16/MODBUS: poly 0xA001 reflected, init 0xFFFF.

    Returned as an int; Modbus appends it low byte first.
    """
    crc = 0xFFFF
    for byte in payload:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def build_read_frame(device_id: int, address: int, count: int) -> bytes:
    """Assemble a function-0x03 Read Holding Registers request."""
    body = struct.pack(">BBHH", device_id, FUNCTION_READ_HOLDING, address, count)
    crc = crc16_modbus(body)
    return body + struct.pack("<H", crc)


def hexs(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


def parse_response(data: bytes, device_id: int, count: int) -> dict:
    """Try to read ``data`` as a function-0x03 reply. Never raises.

    Returns a dict with ``valid`` plus whatever could be determined. A
    malformed candidate is described rather than rejected silently — the
    whole point of this script is to see what is there.
    """
    out: dict = {
        "valid": False,
        "reason": "",
        "device_id": None,
        "function": None,
        "byte_count": None,
        "registers": None,
        "crc_ok": None,
    }
    expected_len = 5 + 2 * count  # id + fn + bytecount + payload + crc16
    if len(data) < 5:
        out["reason"] = f"too short ({len(data)} bytes, need >= 5)"
        return out

    out["device_id"] = data[0]
    out["function"] = data[1]

    if data[1] & 0x80:
        # Modbus exception reply: id, fn|0x80, exception code, crc
        out["reason"] = f"exception response, code 0x{data[2]:02X}"
        return out
    if data[1] != FUNCTION_READ_HOLDING:
        out["reason"] = f"function 0x{data[1]:02X} is not 0x03"
        return out

    out["byte_count"] = data[2]
    if len(data) < expected_len:
        out["reason"] = f"short frame ({len(data)} bytes, expected {expected_len})"
        return out

    frame = data[:expected_len]
    body, crc_rx = frame[:-2], struct.unpack("<H", frame[-2:])[0]
    out["crc_ok"] = crc16_modbus(body) == crc_rx
    if not out["crc_ok"]:
        out["reason"] = "CRC mismatch"
        return out
    if out["device_id"] != device_id:
        out["reason"] = f"device id {out['device_id']} != requested {device_id}"
        return out

    regs = list(struct.unpack(f">{count}H", frame[3:3 + 2 * count]))
    out["registers"] = regs
    out["valid"] = True
    return out


def decode_float_pair(regs: list[int]) -> dict:
    """Decode two registers both ways. Manual page 53 documents high-word-first."""
    if len(regs) < 2:
        return {}
    hi_first = struct.unpack(">f", struct.pack(">HH", regs[0], regs[1]))[0]
    lo_first = struct.unpack(">f", struct.pack(">HH", regs[1], regs[0]))[0]
    return {
        "documented_high_word_first": hi_first,
        "word_swapped": lo_first,
        "note": "manual p.53 documents high word at the even (lower) address",
    }


def classify(tx: bytes, rx: bytes) -> str:
    """Observational label for what came back. Describes, never explains.

    Deliberately mirrors pyrometer_active_exactus_query's vocabulary so the
    two scripts' verdicts can be compared directly in the evidence bundles.
    """
    if not rx:
        return "no_response"
    if rx == tx:
        return "echo_of_sent"
    if rx.startswith(tx):
        return "echo_plus_trailing"
    if tx.startswith(rx):
        return "partial_echo"
    return "non_echo_response"


def read_window(ser, window_s: float) -> bytes:
    """Accumulate every byte arriving within the window.

    Reads in small slices rather than one blocking read so that an echo
    followed by a slower device reply lands in the same buffer instead of
    being split across two calls.
    """
    buf = bytearray()
    deadline = time.monotonic() + window_s
    while time.monotonic() < deadline:
        chunk = ser.read(256)
        if chunk:
            buf.extend(chunk)
        else:
            time.sleep(0.01)
    return bytes(buf)


def probe_register(
    ser, name: str, device_id: int, window_s: float
) -> dict:
    """Send one read request and capture everything that comes back."""
    address, count, description = ALLOWED_REGISTERS[name]
    tx = build_read_frame(device_id, address, count)

    ser.reset_input_buffer()
    ser.write(tx)
    ser.flush()
    rx = read_window(ser, window_s)

    result: dict = {
        "register_name": name,
        "address": f"0x{address:04X}",
        "register_count": count,
        "description": description,
        "tx_hex": hexs(tx),
        "tx_len": len(tx),
        "rx_hex": hexs(rx),
        "rx_len": len(rx),
        "classification": classify(tx, rx),
        "expected_reply_len": 5 + 2 * count,
    }

    # The question this script exists to answer: is there anything behind
    # the echo? Parse the trailing bytes, and also parse the whole buffer
    # in case the device answered without echoing.
    candidates: dict[str, bytes] = {}
    if rx.startswith(tx) and len(rx) > len(tx):
        candidates["after_echo"] = rx[len(tx):]
    if rx and not rx.startswith(tx):
        candidates["whole_buffer"] = rx

    parsed = {}
    for label, candidate in candidates.items():
        p = parse_response(candidate, device_id, count)
        p["candidate_hex"] = hexs(candidate)
        if p["valid"] and count == 2 and p["registers"]:
            p["float_decode"] = decode_float_pair(p["registers"])
        if p["valid"] and name == "REG_VER" and p["registers"]:
            raw = p["registers"][0]
            p["version"] = f"{(raw >> 8) & 0xFF}.{raw & 0xFF}"
        parsed[label] = p
    result["response_parse"] = parsed
    result["device_responded"] = any(p.get("valid") for p in parsed.values())
    return result


def info(msg: str) -> None:
    print(f"  [INFO] {msg}")


def describe_environment(port: str) -> dict:
    """Port inventory + known port-hoggers, matching the sibling scripts."""
    ports: list[dict] = []
    try:
        from serial.tools import list_ports  # lazy: keeps CI import-safe
        for p in list_ports.comports():
            ports.append({
                "device": p.device,
                "description": p.description,
                "hwid": p.hwid,
            })
    except Exception as exc:  # noqa: BLE001 — diagnostics must never crash
        ports.append({"error": str(exc)})
    return {
        "available_ports": ports,
        "target_port_visible": any(p.get("device") == port for p in ports),
    }


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Raw-byte Modbus RTU probe (function 0x03, READ-ONLY). Captures "
            "every byte returned so an echoed request can be separated from "
            "a real device reply."
        ),
    )
    parser.add_argument("--port", default=DEFAULT_PORT, help="Serial port (default COM4)")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Baud rate (default 115200)")
    parser.add_argument(
        "--device-id", type=int, default=DEFAULT_DEVICE_ID,
        help="Modbus slave address (default 1, the BASF factory default)",
    )
    parser.add_argument(
        "--registers", default=",".join(DEFAULT_PROBE_SEQUENCE),
        help=f"Comma-separated names from: {', '.join(ALLOWED_REGISTERS)}",
    )
    parser.add_argument(
        "--response-window-s", type=float, default=DEFAULT_RESPONSE_WINDOW_S,
        help="Seconds to keep reading after each request (default 1.0)",
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="Print the frames that would be sent; opens no port",
    )
    parser.add_argument(
        "--output-dir", type=Path, default=None,
        help="Write raw_modbus_probe.json evidence bundle here",
    )
    args = parser.parse_args(argv)

    names = [n.strip() for n in args.registers.split(",") if n.strip()]

    # Whitelist enforcement happens BEFORE any port is opened, so an
    # invalid request cannot put bytes on the wire even transiently.
    unknown = [n for n in names if n not in ALLOWED_REGISTERS]
    if unknown:
        print(f"ERROR: not in ALLOWED_REGISTERS: {unknown}")
        print(f"Allowed: {', '.join(ALLOWED_REGISTERS)}")
        return 2
    if not names:
        print("ERROR: no registers requested")
        return 2

    print("═══ Pyrometer raw Modbus probe — READ-ONLY ═══")
    print(f"Port:              {args.port}")
    print(f"Baud:              {args.baud} 8N1")
    print(f"Device ID:         {args.device_id}")
    print(f"Function:          0x03 Read Holding Registers (read-only)")
    print(f"Registers:         {', '.join(names)}")
    print(f"Response window:   {args.response_window_s}s")
    print(f"Mode:              {'DRY-RUN' if args.dry_run else 'ACTIVE READ'}")
    print()

    bundle: dict = {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "args": {
            "port": args.port,
            "baud": args.baud,
            "device_id": args.device_id,
            "registers": names,
            "response_window_s": args.response_window_s,
            "dry_run": args.dry_run,
            "output_dir": str(args.output_dir) if args.output_dir else None,
        },
    }

    print("--- Phase 0: Environment ---")
    env = describe_environment(args.port)
    bundle["environment"] = env
    info(f"{len(env['available_ports'])} serial port(s) visible to pyserial:")
    for p in env["available_ports"]:
        marker = "   ← --port target" if p.get("device") == args.port else ""
        print(f"        {p.get('device', '?'):6s}  {p.get('description', p.get('error', ''))}{marker}")
    if not env["target_port_visible"]:
        print()
        info(f"--port '{args.port}' is NOT in the visible ports list.")
    print()

    if args.dry_run:
        print("--- Phase 1: Dry-run ---")
        frames = []
        for name in names:
            address, count, _ = ALLOWED_REGISTERS[name]
            tx = build_read_frame(args.device_id, address, count)
            info(f"{name} @ 0x{address:04X} x{count} → {hexs(tx)}")
            frames.append({"register_name": name, "tx_hex": hexs(tx)})
        info("No serial port opened. No bytes written.")
        bundle["dry_run_frames"] = frames
        bundle["verdict"] = {"state": "dry_run"}
    else:
        print("--- Phase 1: Probe + capture ---")
        try:
            import serial  # lazy: keeps this importable on CI without pyserial
        except ImportError:
            print("ERROR: pyserial not installed (pip install pyserial)")
            return 3

        probes: list[dict] = []
        try:
            with serial.Serial(
                args.port, args.baud, bytesize=8, parity="N", stopbits=1,
                timeout=0.1,
            ) as ser:
                for name in names:
                    r = probe_register(ser, name, args.device_id, args.response_window_s)
                    probes.append(r)
                    print(f"  {name} @ {r['address']}")
                    print(f"    TX ({r['tx_len']}): {r['tx_hex']}")
                    print(f"    RX ({r['rx_len']}): {r['rx_hex'] or '(nothing)'}")
                    print(f"    Classification: {r['classification']}")
                    if r["device_responded"]:
                        for label, p in r["response_parse"].items():
                            if p.get("valid"):
                                print(f"    *** DEVICE REPLY in '{label}': registers={p['registers']}")
                                if "version" in p:
                                    print(f"        version: {p['version']}")
                                if "float_decode" in p:
                                    fd = p["float_decode"]
                                    print(f"        float (documented order): {fd['documented_high_word_first']}")
                                    print(f"        float (word-swapped):     {fd['word_swapped']}")
                    elif r["response_parse"]:
                        for label, p in r["response_parse"].items():
                            print(f"    trailing bytes in '{label}' did not parse: {p['reason']}")
                    print()
        except Exception as exc:  # noqa: BLE001 — pyserial raises many types
            print(f"  [FAIL] Port error: {exc}")
            bundle["port_error"] = str(exc)
            bundle["probes"] = probes
            bundle["verdict"] = {"state": "port_error"}
            _write_bundle(args.output_dir, bundle)
            return 1

        bundle["probes"] = probes
        any_reply = any(p["device_responded"] for p in probes)
        classes = {p["classification"] for p in probes}
        if any_reply:
            state = "device_replied"
        elif classes == {"echo_of_sent"}:
            state = "echo_only"
        elif classes == {"no_response"}:
            state = "no_response"
        else:
            state = "mixed_" + "_".join(sorted(classes))
        bundle["verdict"] = {"state": state}

        print("--- Verdict ---")
        print(f"State: {state}")
        print()
        print("Labels are OBSERVATIONAL — they describe what appeared on")
        print("the wire, not causation.")
        print()
        if state == "device_replied":
            print("  A valid function-0x03 reply was parsed. The probe DOES")
            print("  answer Modbus; prior 'silent' verdicts were pymodbus")
            print("  discarding frames it could not align past the echo.")
        elif state == "echo_only":
            print("  Only our own bytes came back, with nothing behind them.")
            print("  Consistent with the probe not answering Modbus AND the")
            print("  line echoing. Neither is proven by this alone.")
        elif state == "no_response":
            print("  Nothing came back at all — not even an echo. Note this")
            print("  differs from the Jul 30 active Exactus query result.")

    _write_bundle(args.output_dir, bundle)
    return 0


def _write_bundle(output_dir: Optional[Path], bundle: dict) -> None:
    """Write the evidence bundle.

    Filename is deliberately NOT the bare ``result.json`` the sibling
    scripts use: on Jul 30 2026 two scripts pointed at the same
    --output-dir and the second silently clobbered the first's bundle.
    """
    if output_dir is None:
        return
    output_dir.mkdir(parents=True, exist_ok=True)
    path = output_dir / "raw_modbus_probe.json"
    path.write_text(json.dumps(bundle, indent=2, default=str))
    print("Evidence bundle written:")
    print(f"  raw_modbus_probe.json: {path}")


if __name__ == "__main__":
    sys.exit(main())
