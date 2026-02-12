"""
Dracal VCP thermocouple reader.

Reads data lines from a Dracal device running in VCP mode over serial,
validates CRC-16-CCITT (XMODEM), and prints parsed measurements.

Requirements:
    pip install pyserial

Examples:
    python dracal_thermocouple_reader.py --port /dev/tty.usbmodemE246381
    python dracal_thermocouple_reader.py --port COM4 --interval-ms 500 --frac 2
    python dracal_thermocouple_reader.py --csv logs/thermocouple_log.csv
"""

from __future__ import annotations

import argparse
import binascii
import csv
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional

import serial
from serial.tools import list_ports


@dataclass
class ChannelPoint:
    """Single measurement point from one channel."""

    name: str
    value: float | int | str
    unit: str


@dataclass
class DracalRecord:
    """Parsed Dracal VCP line."""

    line_type: str
    product: str
    serial: str
    message: str
    points: list[ChannelPoint]


class DracalVCPReader:
    """Low-level reader for Dracal sensors in VCP mode."""

    def __init__(self, port: str, baudrate: int = 9600, timeout: float = 2.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser: Optional[serial.Serial] = None
        self.channel_names: list[str] = []

    def connect(self) -> None:
        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.timeout,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
        # Discard possible partial lines right after opening the port.
        self.ser.readlines(2)

    def disconnect(self) -> None:
        if self.ser:
            self.ser.close()
            self.ser = None

    def send_command(self, command: str) -> None:
        if not self.ser:
            raise ConnectionError("Serial connection is not open")
        self.ser.write((command.strip() + "\n").encode("ascii"))

    def read_line(self) -> bytes:
        if not self.ser:
            raise ConnectionError("Serial connection is not open")
        return self.ser.readline()

    @staticmethod
    def _compute_crc(payload: bytes) -> int:
        return binascii.crc_hqx(payload, 0)

    @staticmethod
    def _parse_value(text: str) -> float | int | str:
        try:
            return int(text)
        except ValueError:
            try:
                return float(text)
            except ValueError:
                return text

    def parse_line(self, raw_line: bytes) -> DracalRecord:
        line = raw_line.strip()
        if not line:
            raise ValueError("Empty line")

        try:
            payload, crc_text = line.split(b"*", 1)
            read_crc = int(crc_text, 16)
        except ValueError as exc:
            raise ValueError(f"Invalid line format: {line!r}") from exc

        computed_crc = self._compute_crc(payload)
        if computed_crc != read_crc:
            raise ValueError(
                f"CRC mismatch (read=0x{read_crc:04X}, computed=0x{computed_crc:04X})"
            )

        fields = payload.decode("ascii", errors="strict").strip(",").split(",")
        if len(fields) < 4:
            raise ValueError(f"Too few fields: {fields!r}")

        line_type, product, serial_number, message = fields[:4]

        points: list[ChannelPoint] = []
        data_fields = fields[4:]
        for idx in range(0, len(data_fields), 2):
            if idx + 1 >= len(data_fields):
                break
            value_text = data_fields[idx].strip()
            unit = data_fields[idx + 1].strip()
            if value_text == "" and unit == "":
                continue

            if self.channel_names and idx // 2 < len(self.channel_names):
                name = self.channel_names[idx // 2]
            else:
                name = f"ch_{(idx // 2) + 1}"
            value = self._parse_value(value_text)
            points.append(ChannelPoint(name=name, value=value, unit=unit))

        return DracalRecord(
            line_type=line_type,
            product=product,
            serial=serial_number,
            message=message,
            points=points,
        )

    def handle_info_record(self, record: DracalRecord) -> None:
        if record.line_type != "I":
            return

        if record.product == "Product ID":
            # INFO command response contains alternating channel_name/unit pairs.
            self.channel_names = [str(p.value) for p in record.points]


class CsvLogger:
    """Writes each measurement channel as one CSV row."""

    def __init__(self, output_path: Path):
        self.output_path = output_path
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.output_path.open("w", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow(
            [
                "timestamp",
                "elapsed_seconds",
                "line_type",
                "product",
                "serial",
                "channel",
                "value",
                "unit",
            ]
        )

    def write_record(self, elapsed: float, record: DracalRecord) -> None:
        timestamp = datetime.now().isoformat()
        for p in record.points:
            self._writer.writerow(
                [
                    timestamp,
                    f"{elapsed:.3f}",
                    record.line_type,
                    record.product,
                    record.serial,
                    p.name,
                    p.value,
                    p.unit,
                ]
            )
        self._file.flush()

    def close(self) -> None:
        self._file.close()


def auto_detect_port() -> Optional[str]:
    """Best-effort detection for common Dracal VCP port naming."""
    preferred = []
    fallback = []

    for p in list_ports.comports():
        descriptor = " ".join(
            [
                str(p.device or ""),
                str(p.description or ""),
                str(p.manufacturer or ""),
                str(p.hwid or ""),
            ]
        ).lower()

        if "dracal" in descriptor:
            preferred.append(p.device)
            continue

        if "usbmodem" in descriptor or "ttyacm" in descriptor or "usb serial" in descriptor:
            fallback.append(p.device)

    if preferred:
        return preferred[0]
    if fallback:
        return fallback[0]
    return None


def extract_temperature(record: DracalRecord) -> Optional[tuple[str, float | int | str, str]]:
    """Try to locate a temperature channel for quick control-loop integration."""
    for p in record.points:
        if "temp" in p.name.lower() or p.unit.upper() in {"C", "DEGC", "K", "F"}:
            return p.name, p.value, p.unit
    return None


def configure_sensor(reader: DracalVCPReader, interval_ms: int, frac: Optional[int]) -> None:
    reader.send_command("INFO")
    time.sleep(0.3)
    reader.send_command(f"POLL {interval_ms}")
    time.sleep(0.3)
    if frac is not None:
        reader.send_command(f"FRAC {frac}")
        time.sleep(0.3)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Read Dracal VCP thermocouple data")
    parser.add_argument("--port", help="Serial port (ex: /dev/tty.usbmodemXXXX or COM4)")
    parser.add_argument(
        "--interval-ms",
        type=int,
        default=1000,
        help="Polling interval in milliseconds (default: 1000)",
    )
    parser.add_argument(
        "--frac",
        type=int,
        default=2,
        help="Decimal precision (1..7). Use --no-frac to skip command.",
    )
    parser.add_argument(
        "--no-frac",
        action="store_true",
        help="Do not send FRAC command",
    )
    parser.add_argument(
        "--csv",
        type=Path,
        help="Optional CSV output path",
    )
    parser.add_argument(
        "--max-samples",
        type=int,
        default=0,
        help="Stop after this many data samples (0 = infinite)",
    )
    parser.add_argument(
        "--show-info",
        action="store_true",
        help="Print INFO/echo lines in addition to data lines",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    port = args.port or auto_detect_port()
    if not port:
        print("No serial port detected. Provide --port explicitly.")
        return 1

    frac = None if args.no_frac else max(1, min(7, args.frac))

    print(f"Opening Dracal VCP on: {port}")
    reader = DracalVCPReader(port=port)
    csv_logger = CsvLogger(args.csv) if args.csv else None

    sample_count = 0
    start_time = time.time()

    try:
        reader.connect()
        configure_sensor(reader, args.interval_ms, frac)
        print("Streaming... Press Ctrl+C to stop.")

        while True:
            raw = reader.read_line()
            if not raw:
                continue

            elapsed = time.time() - start_time
            try:
                record = reader.parse_line(raw)
            except ValueError as exc:
                print(f"Skipped invalid line: {exc}")
                continue

            reader.handle_info_record(record)

            if record.line_type in {"I"}:
                if args.show_info:
                    if record.message:
                        print(f"[INFO] {record.message}")
                    else:
                        printable = ", ".join(f"{p.name} ({p.unit})" for p in record.points)
                        if printable:
                            print(f"[INFO] Channels: {printable}")
                continue

            sample_count += 1

            temp = extract_temperature(record)
            if temp:
                temp_name, temp_value, temp_unit = temp
                print(
                    f"[{sample_count:6d}] {elapsed:8.2f}s | "
                    f"{temp_name}: {temp_value} {temp_unit} | "
                    f"{record.product} {record.serial}"
                )
            else:
                text = ", ".join(f"{p.name}={p.value} {p.unit}" for p in record.points)
                print(f"[{sample_count:6d}] {elapsed:8.2f}s | {text}")

            if csv_logger:
                csv_logger.write_record(elapsed, record)

            if args.max_samples > 0 and sample_count >= args.max_samples:
                break

    except KeyboardInterrupt:
        print("\nStopped by user.")
    except serial.SerialException as exc:
        print(f"Serial error: {exc}")
        return 1
    finally:
        reader.disconnect()
        if csv_logger:
            csv_logger.close()

    print(f"Captured {sample_count} samples.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
