#!/usr/bin/env python3
"""Pyrometer Modbus smoke test — direct read of BASF Exactus on COM4.

Run this AFTER:
    1. Closing TemperaSure software fully
    2. Physically power-cycling the probe (~10s off, ~10s boot)

TemperaSure leaves the probe in Exactus Protocol mode on exit, which
silently breaks Modbus reads (returns empty registers). The physical
power-cycle is the documented recovery per bulbasaur_env.md (May 6 2026).

Tries both input registers (fn 04) and holding registers (fn 03) for
temperature at 0x0000 and current at 0x0004, decoded as float32 BE.

Usage on Bulbasaur:
    python scripts\\test_pyrometer.py
"""
from __future__ import annotations

import struct
import sys

try:
    from pymodbus.client import ModbusSerialClient
except ImportError:
    sys.exit("pymodbus not installed. Run: pip install pymodbus")


PORT = "COM4"
BAUDRATE = 115200
SLAVE_ID = 1
TEMP_REG = 0x0000
CURR_REG = 0x0004


def decode_f32_be(regs: list[int]) -> float | None:
    """Decode 2 16-bit registers as a single float32, big-endian word order."""
    if regs is None or len(regs) < 2:
        return None
    packed = struct.pack(">HH", regs[0], regs[1])
    return struct.unpack(">f", packed)[0]


def decode_f32_le(regs: list[int]) -> float | None:
    """Decode 2 16-bit registers as a single float32, little-endian word order.

    BASF/Exactus documentation isn't entirely consistent on word order; try
    both and see which gives a physical value (~0-2000 °C for temperature).
    """
    if regs is None or len(regs) < 2:
        return None
    packed = struct.pack(">HH", regs[1], regs[0])
    return struct.unpack(">f", packed)[0]


def try_read(client, fn_name: str, fn, address: int, count: int = 2):
    """Try a Modbus read, print outcome, return registers or None."""
    try:
        result = fn(address=address, count=count, slave=SLAVE_ID)
    except Exception as e:
        print(f"    {fn_name:<32} EXCEPTION: {e}")
        return None
    if result.isError():
        print(f"    {fn_name:<32} ERROR:     {result}")
        return None
    print(f"    {fn_name:<32} OK:        regs={result.registers}")
    return result.registers


def report(label: str, address: int) -> None:
    """Try both register types at ``address`` and report decoded values."""
    print(f"\n{label} (0x{address:04X}):")
    regs_in = try_read(
        client, "input_registers (fn 04)",
        client.read_input_registers, address,
    )
    regs_hold = try_read(
        client, "holding_registers (fn 03)",
        client.read_holding_registers, address,
    )
    for src_name, regs in [
        ("input_registers", regs_in),
        ("holding_registers", regs_hold),
    ]:
        if regs is None:
            continue
        be = decode_f32_be(regs)
        le = decode_f32_le(regs)
        print(f"    {src_name:<20} BE float32: {be:>12.4f}  |  LE float32: {le:>12.4f}")


print(f"Connecting to {PORT} @ {BAUDRATE} 8N1, slave {SLAVE_ID}...")
client = ModbusSerialClient(
    port=PORT, baudrate=BAUDRATE,
    bytesize=8, parity="N", stopbits=1,
    timeout=2,
)
if not client.connect():
    sys.exit(f"Connect failed on {PORT}. Check the cable and that COM4 isn't held by another process (TemperaSure?).")

try:
    report("Temperature", TEMP_REG)
    report("Current",     CURR_REG)
finally:
    client.close()

print("\nIf both register types ERROR'd or returned 0/garbage, the probe is")
print("likely still in Exactus Protocol mode. Power-cycle the probe and re-run.")
