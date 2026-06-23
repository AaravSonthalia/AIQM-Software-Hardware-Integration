"""Pyrometer direct-read smoke test via Exactus Modbus RTU.

Validates the Tier 1 direct-read path for the BASF Exactus pyrometer
(IFD-5 interface) on Bulbasaur. Bypasses TemperaSure entirely — opens
COM4, talks Modbus RTU, reads the temperature register. The smallest-
commitment version of the direct-reading migration plan.

Adapted from Jacques' ``pyrometer_script.py`` (register map, version-
tolerant pymodbus wrapper, word-swap fallback).

Usage on Bulbasaur:
    1. Close TemperaSure (it holds an exclusive lock on COM4).
    2. If the probe was last in Exactus Protocol mode rather than
       Modbus, power-cycle it.
    3. Run::

           python scripts/pyrometer_modbus_smoke.py \\
               --duration 60 --output /tmp/pyro_smoke.csv

Default: 30 seconds at 2 Hz, CSV to ``pyrometer_modbus_smoke.csv`` in
cwd. Output columns: ``timestamp_iso, temp_C, current_A``.

Comparison against the screengrab path is done offline by joining this
CSV with the Growth Monitor ``sensor_log.csv`` on timestamp.

Prerequisites (on Bulbasaur, install if not present)::

    pip install pymodbus pyserial
"""
from __future__ import annotations

import argparse
import csv
import struct
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

try:
    from pymodbus.client import ModbusSerialClient
    from pymodbus.exceptions import ModbusException
except ImportError as exc:  # pragma: no cover
    print(
        f"Missing pymodbus: {exc}\n"
        "Install with: pip install pymodbus pyserial",
        file=sys.stderr,
    )
    raise


# Register map from BASF Exactus IFD-5 manual.
# Float32 values span two 16-bit regs (even = hi word, odd = lo word).
REG_CH1_TEMP = 0x0000
REG_CH1_CURR = 0x0004


def _read_holding(client, addr: int, count: int, device_id: int):
    """Pymodbus version-compat wrapper.

    ModbusSerialClient.read_holding_registers has spelled the slave-id
    keyword as ``unit``, ``slave``, then ``device_id`` across releases.
    Try each in turn so the script works on whatever pymodbus is installed.
    """
    for kw in ("device_id", "slave", "unit"):
        try:
            return client.read_holding_registers(addr, count, **{kw: device_id})
        except TypeError:
            continue
    return client.read_holding_registers(
        address=addr, count=count, device_id=device_id
    )


def regs_to_f32(hi: int, lo: int) -> float:
    raw = (hi << 16) | lo
    return struct.unpack(">f", raw.to_bytes(4, "big"))[0]


def read_temp_and_current(
    client, device_id: int = 1, word_swap: bool = False
) -> tuple[float, float]:
    """Read CH1 temperature (°C) and current (A) from the pyrometer."""
    rr_t = _read_holding(client, REG_CH1_TEMP, 2, device_id)
    rr_i = _read_holding(client, REG_CH1_CURR, 2, device_id)
    if rr_t.isError() or rr_i.isError():
        raise ModbusException(f"Bad reply: temp={rr_t}, curr={rr_i}")
    if len(rr_t.registers) < 2 or len(rr_i.registers) < 2:
        raise ModbusException(
            "Empty register response — probe is likely in Exactus Protocol "
            "mode rather than Modbus RTU. Recovery: physical power-cycle "
            "the probe (~10s off, 10s boot), then retry. "
            f"Got temp regs={rr_t.registers!r}, curr regs={rr_i.registers!r}"
        )

    hi_t, lo_t = rr_t.registers
    hi_i, lo_i = rr_i.registers
    if word_swap:
        hi_t, lo_t = lo_t, hi_t
        hi_i, lo_i = lo_i, hi_i

    return regs_to_f32(hi_t, lo_t), regs_to_f32(hi_i, lo_i)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(
        description="Direct-read pyrometer smoke test via Modbus RTU"
    )
    p.add_argument("--port", default="COM4", help="Serial port (default: COM4)")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--id", type=int, default=1, help="Modbus device/slave ID")
    p.add_argument("--hz", type=float, default=2.0, help="Read rate (Hz)")
    p.add_argument(
        "--duration", type=float, default=30.0, help="Total duration in seconds"
    )
    p.add_argument(
        "--output",
        type=Path,
        default=Path("pyrometer_modbus_smoke.csv"),
        help="CSV output path",
    )
    p.add_argument(
        "--word-swap",
        action="store_true",
        help="Force word-swap for float32 (try if temps come out implausible)",
    )
    args = p.parse_args(argv)

    client = ModbusSerialClient(
        port=args.port,
        baudrate=args.baud,
        parity="N",
        stopbits=1,
        bytesize=8,
        timeout=1.0,
    )
    if not client.connect():
        print(
            f"Could not open {args.port}. Close TemperaSure and any other "
            f"serial monitor, then retry.",
            file=sys.stderr,
        )
        return 2

    print(
        f"Connected {args.port} @ {args.baud} (8N1, slave={args.id}); "
        f"reading at {args.hz} Hz for {args.duration:.0f}s"
    )
    print(f"CSV → {args.output}")

    period = 1.0 / max(0.1, args.hz)
    t_end = time.time() + args.duration
    word_swap = args.word_swap
    n_ok = n_err = 0

    try:
        with args.output.open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["timestamp_iso", "temp_C", "current_A"])

            while time.time() < t_end:
                try:
                    temp, curr = read_temp_and_current(
                        client, args.id, word_swap=word_swap
                    )
                    if not (-100.0 < temp < 3000.0) and not word_swap:
                        word_swap = True
                        temp, curr = read_temp_and_current(
                            client, args.id, word_swap=word_swap
                        )
                        print(
                            "Auto-enabled word_swap (initial temp was implausible)."
                        )

                    ts = datetime.now(timezone.utc).isoformat()
                    w.writerow([ts, f"{temp:.3f}", f"{curr:.6g}"])
                    print(
                        f"{ts}  T={temp:7.2f}°C  I={curr:.4g}",
                        end="\r",
                        flush=True,
                    )
                    n_ok += 1
                except ModbusException as e:
                    print(f"\nRead error: {e}")
                    n_err += 1

                time.sleep(period)
        print()
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        client.close()

    print(f"Done. {n_ok} successful reads, {n_err} errors. CSV: {args.output}")
    return 0 if n_ok > 0 else 1


if __name__ == "__main__":
    sys.exit(main())
