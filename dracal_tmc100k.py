"""
Dracal TMC100k USB Type-K Thermocouple Module

Provides a TemperatureSensor implementation for the Dracal TMC100k
USB thermocouple converter using VCP (Virtual COM Port) serial protocol.

Protocol: 9600 baud, 8N1. Device streams CSV data with CRC-16-XMODEM checksums.
Data format: D,VCP-TMC100k,SERIAL,,<tc_temp>,C,<cj_temp>,C,*<CRC16>
"""

import serial
import serial.tools.list_ports
import time
from typing import Optional
from crccheck.crc import CrcXmodem

from heater_control import TemperatureSensor


class DracalTMC100k(TemperatureSensor):
    """
    Temperature sensor interface for the Dracal TMC100k USB Type-K
    thermocouple converter in VCP mode.

    The device streams temperature data continuously at a configurable
    poll interval. read_temperature() drains the serial buffer and
    returns the most recent valid reading.
    """

    DEFAULT_BAUD_RATE = 9600
    DEFAULT_POLL_MS = 500
    DEFAULT_FRAC = 3

    def __init__(
        self,
        port: str,
        poll_interval_ms: int = DEFAULT_POLL_MS,
        decimal_places: int = DEFAULT_FRAC,
        timeout: float = 2.0,
    ):
        """
        Initialize the Dracal TMC100k sensor.

        Args:
            port: Serial port path (e.g., '/dev/tty.usbmodem12345')
            poll_interval_ms: Data streaming interval in ms (100-60000)
            decimal_places: Decimal precision for readings (1-7)
            timeout: Serial read timeout in seconds
        """
        self.port = port
        self.poll_interval_ms = poll_interval_ms
        self.decimal_places = decimal_places
        self.timeout = timeout
        self._ser: Optional[serial.Serial] = None
        self._last_tc_temp: Optional[float] = None
        self._last_cj_temp: Optional[float] = None
        self._crc = CrcXmodem()

    def connect(self) -> None:
        """
        Open connection to the TMC100k and configure streaming.

        Sends FRAC and POLL commands, then validates the connection
        by reading and verifying at least one valid data line.

        Raises:
            serial.SerialException: If the port cannot be opened.
            ConnectionError: If no valid data is received after 3 attempts.
        """
        self._ser = serial.Serial(
            self.port,
            baudrate=self.DEFAULT_BAUD_RATE,
            timeout=self.timeout,
        )

        self._send_command(f"POLL {self.poll_interval_ms}")
        time.sleep(0.3)
        self._send_command(f"FRAC {self.decimal_places}")
        time.sleep(0.3)

        # Read through acknowledgment lines until we get a valid data line
        for _ in range(10):
            line = self._read_line()
            if line is None:
                continue
            result = self._parse_data_line(line)
            if result is not None:
                self._last_tc_temp, self._last_cj_temp = result
                return

        raise ConnectionError(
            f"Connected to {self.port} but received no valid data. "
            "Ensure the device is in VCP mode."
        )

    def disconnect(self) -> None:
        """Stop streaming and close the serial connection."""
        if self._ser is not None and self._ser.is_open:
            try:
                self._send_command("POLL 0")
            except Exception:
                pass
            self._ser.close()
        self._ser = None
        self._last_tc_temp = None
        self._last_cj_temp = None

    def read_temperature(self) -> float:
        """
        Read the most recent thermocouple temperature in degrees C.

        Drains the serial buffer and returns the newest valid reading.
        Falls back to the last known reading if the buffer is empty.

        Returns:
            Thermocouple temperature in degrees Celsius.

        Raises:
            ConnectionError: If not connected.
            RuntimeError: If no valid reading is available.
        """
        if self._ser is None or not self._ser.is_open:
            raise ConnectionError("Not connected to sensor")

        # Try to get a fresh reading from the buffer
        result = self._drain_buffer()
        if result is not None:
            self._last_tc_temp, self._last_cj_temp = result
            return self._last_tc_temp

        # Buffer was empty — wait for the next streamed line
        line = self._read_line()
        if line is not None:
            result = self._parse_data_line(line)
            if result is not None:
                self._last_tc_temp, self._last_cj_temp = result
                return self._last_tc_temp

        # Fall back to last known value
        if self._last_tc_temp is not None:
            return self._last_tc_temp

        raise RuntimeError("No valid temperature reading available")

    def read_cold_junction(self) -> float:
        """
        Return the most recent cold junction temperature in degrees C.

        Updated as a side effect of read_temperature().

        Raises:
            RuntimeError: If no reading has been taken yet.
        """
        if self._last_cj_temp is None:
            raise RuntimeError("No cold junction reading available")
        return self._last_cj_temp

    def get_info(self) -> dict:
        """
        Query device information.

        The INFO command returns column headers, not device identity.
        Device identity (product_id, serial_number) comes from data lines.

        Returns:
            Dictionary with product_id, serial_number, and sensor descriptions.

        Raises:
            ConnectionError: If not connected.
        """
        if self._ser is None or not self._ser.is_open:
            raise ConnectionError("Not connected to sensor")

        info = {}

        # Get sensor descriptions from INFO response
        self._ser.reset_input_buffer()
        self._send_command("INFO")
        time.sleep(0.3)

        for _ in range(5):
            line = self._read_line()
            if line is None:
                continue
            idx = line.rfind("*")
            body = line[:idx] if idx > 0 else line
            parts = body.split(",")
            if parts[0] == "I" and len(parts) >= 5:
                info["sensors"] = [parts[i] for i in range(4, len(parts), 2) if parts[i]]
                break
            elif parts[0] == "D" and len(parts) >= 3:
                info["product_id"] = parts[1]
                info["serial_number"] = parts[2]

        # Get product_id and serial_number from a data line
        if "product_id" not in info:
            for _ in range(3):
                line = self._read_line()
                if line is None:
                    continue
                idx = line.rfind("*")
                body = line[:idx] if idx > 0 else line
                parts = body.split(",")
                if parts[0] == "D" and len(parts) >= 3:
                    info["product_id"] = parts[1]
                    info["serial_number"] = parts[2]
                    break

        return info

    # --- Internal methods ---

    def _send_command(self, cmd: str) -> None:
        """Send a command string to the device."""
        self._ser.write((cmd + "\n").encode("ascii"))

    def _read_line(self) -> Optional[str]:
        """
        Read one line from the serial port and validate its CRC.

        Returns:
            The raw line string if CRC is valid, None otherwise.
        """
        try:
            raw = self._ser.readline()
        except serial.SerialException:
            return None

        if not raw:
            return None

        line = raw.decode("ascii", errors="ignore").strip()
        if not line:
            return None

        if self._validate_crc(line):
            return line
        return None

    def _validate_crc(self, line: str) -> bool:
        """Validate the CRC-16-XMODEM checksum on a data line."""
        idx = line.rfind("*")
        if idx < 0 or idx + 1 >= len(line):
            return False

        payload = line[:idx]
        checksum_hex = line[idx + 1:]

        try:
            expected = int(checksum_hex, 16)
        except ValueError:
            return False

        self._crc.process(payload.encode("ascii"))
        computed = self._crc.final()
        self._crc.reset()

        return computed == expected

    def _parse_data_line(self, line: str) -> Optional[tuple[float, float]]:
        """
        Parse a validated data line into (tc_temp, cj_temp).

        Expected format: D,VCP-TMC100k,SERIAL,,<tc>,C,<cj>,C,*XXXX
        The checksum portion after * has already been stripped by _validate_crc
        validation, but the line passed here is the full line.

        Returns:
            (thermocouple_temp, cold_junction_temp) or None if parse fails.
        """
        # Strip the checksum portion for parsing
        idx = line.rfind("*")
        if idx > 0:
            line = line[:idx]

        parts = line.split(",")

        if len(parts) < 7 or parts[0] != "D":
            return None

        try:
            tc_temp = float(parts[4])
            cj_temp = float(parts[6])
            return (tc_temp, cj_temp)
        except (ValueError, IndexError):
            return None

    def _drain_buffer(self) -> Optional[tuple[float, float]]:
        """
        Read all buffered serial data and return the most recent valid reading.

        Returns:
            (tc_temp, cj_temp) from the newest valid line, or None.
        """
        data = b""
        while self._ser.in_waiting > 0:
            data += self._ser.read(self._ser.in_waiting)
            time.sleep(0.01)

        if not data:
            return None

        lines = data.decode("ascii", errors="ignore").strip().split("\n")

        # Process newest lines first
        for line in reversed(lines):
            line = line.strip()
            if not line or not line.startswith("D,"):
                continue
            if self._validate_crc(line):
                result = self._parse_data_line(line)
                if result is not None:
                    return result

        return None

    # --- Context manager ---

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()
        return False


def find_dracal_sensors() -> list[tuple[str, str]]:
    """
    Scan serial ports for Dracal TMC100k sensors in VCP mode.

    First checks port descriptions reported by the OS (fast),
    then falls back to probing via serial commands (slower).

    Returns:
        List of (port_path, identity_string) tuples for found sensors.
    """
    found = []

    for port in serial.tools.list_ports.comports():
        # Check if the OS already identifies this as a Dracal sensor
        description = port.description or ""
        if "TMC100" in description:
            found.append((port.device, description))
            continue

        # Probe the port via serial commands
        try:
            ser = serial.Serial(
                port.device,
                baudrate=DracalTMC100k.DEFAULT_BAUD_RATE,
                timeout=1.5,
            )

            ser.write(b"INFO\n")
            time.sleep(0.5)

            for _ in range(5):
                raw = ser.readline()
                if not raw:
                    break
                line = raw.decode("ascii", errors="ignore").strip()
                if "TMC100" in line:
                    parts = line.split(",")
                    identity = parts[1] if len(parts) > 1 else "TMC100k"
                    found.append((port.device, identity))
                    break

            ser.close()
        except Exception:
            pass

    return found


if __name__ == "__main__":
    print("Scanning for Dracal TMC100k sensors...")
    sensors = find_dracal_sensors()

    if not sensors:
        print("No Dracal sensors found.")
        print("\nAvailable serial ports:")
        for port in serial.tools.list_ports.comports():
            print(f"  {port.device} - {port.description}")
    else:
        for port_path, identity in sensors:
            print(f"Found: {identity}")
            print(f"  Port: {port_path}")

        # Connect to first found sensor
        port_path, identity = sensors[0]
        print(f"\nConnecting to {identity} on {port_path}...")

        with DracalTMC100k(port=port_path) as sensor:
            info = sensor.get_info()
            print(f"Product: {info.get('product_id', 'unknown')}")
            print(f"Serial:  {info.get('serial_number', 'unknown')}")

            print(f"\n{'='*40}")
            print("Reading temperatures (10 samples, 1s interval)")
            print("="*40)

            for i in range(10):
                tc = sensor.read_temperature()
                cj = sensor.read_cold_junction()
                print(f"  [{i+1:2d}] Thermocouple: {tc:.3f} C  |  Cold Junction: {cj:.3f} C")
                time.sleep(1.0)

        print("\nDone.")
