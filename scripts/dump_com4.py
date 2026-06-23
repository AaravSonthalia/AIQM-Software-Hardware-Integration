#!/usr/bin/env python3
"""Raw byte dump from COM4 — diagnostic for the BASF Exactus pyrometer.

Reads whatever is on the wire for ~2 seconds and prints it as hex.
No protocol assumptions. Use this when ExactusSerialPyrometer returns
0 or hangs — the raw bytes tell you whether the probe is even sending
the expected 0x81-headed packets or something else entirely.

Prerequisites:
    - TemperaSure CLOSED (COM4 is exclusive)
    - Growth Monitor STOPPED (it also holds COM4 in exactus mode)
    - No stray python processes on COM4
        Get-Process python | Stop-Process -Force

Usage:
    python scripts\\dump_com4.py
"""
from __future__ import annotations

import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial not installed. Run: pip install pyserial")


PORT = "COM4"
BAUDRATE = 115200
TIMEOUT_S = 2.0
DUMP_BYTES = 200


def main() -> None:
    print(f"Opening {PORT} @ {BAUDRATE}, 8N1, no flow control...")
    try:
        s = serial.Serial(port=PORT, baudrate=BAUDRATE, timeout=TIMEOUT_S)
    except serial.SerialException as e:
        sys.exit(
            f"Failed to open {PORT}: {e}\n\n"
            "Most likely cause: another process is holding COM4. Close\n"
            "TemperaSure, stop Growth Monitor, kill any stray python procs:\n"
            "    Get-Process python | Stop-Process -Force"
        )

    print(f"Waiting {TIMEOUT_S:.0f}s to accumulate bytes...")
    time.sleep(TIMEOUT_S)
    n_available = s.in_waiting
    print(f"Bytes in buffer: {n_available}")

    data = s.read(max(n_available, DUMP_BYTES))
    s.close()

    if not data:
        print("\nNO BYTES received.")
        print(
            "Probe is silent. Either it's not in continuous-transmit mode "
            "(may need a poll command), or it's at a different baud rate, "
            "or it isn't powered."
        )
        return

    print(f"\nReceived {len(data)} bytes:")
    # Print as 16-byte hex rows for readability.
    for i in range(0, len(data), 16):
        chunk = data[i : i + 16]
        hex_str = " ".join(f"{b:02X}" for b in chunk)
        ascii_str = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
        print(f"  {i:04X}  {hex_str:<48}  {ascii_str}")

    # Look for repeating 0x81-headed packets (undergrad's assumed framing).
    n_81 = sum(1 for b in data if b == 0x81)
    print(f"\nCount of 0x81 bytes: {n_81}")
    if n_81 >= 2:
        # Try to print the gap pattern between consecutive 0x81s.
        positions = [i for i, b in enumerate(data) if b == 0x81]
        gaps = [
            positions[i + 1] - positions[i] for i in range(len(positions) - 1)
        ]
        print(f"Positions of 0x81: {positions[:10]}{'...' if len(positions) > 10 else ''}")
        print(f"Gaps between 0x81s: {gaps[:10]}{'...' if len(gaps) > 10 else ''}")
        if all(g == 5 for g in gaps):
            print("→ Consistent 5-byte packets: undergrad's framing assumption looks correct.")
        elif len(set(gaps)) == 1:
            print(f"→ Consistent {gaps[0]}-byte packets: framing is different (not 5 bytes)")
        else:
            print("→ Inconsistent gaps: either mis-syncing on noise, or variable-length packets")


if __name__ == "__main__":
    main()
