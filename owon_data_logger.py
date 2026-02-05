"""
OWON Power Supply Data Logger

Standalone script to log voltage, current, and power measurements from an OWON
SPE series power supply to a CSV file at user-configurable intervals.

Requirements:
    pip install pyvisa pyvisa-py pyserial

Usage:
    python owon_data_logger.py                     # Auto-detect, 1 second interval
    python owon_data_logger.py -i 0.5              # 500ms interval
    python owon_data_logger.py -o mydata.csv       # Custom output file
    python owon_data_logger.py -p /dev/tty.usb...  # Specify port
    python owon_data_logger.py --interactive       # Interactive mode

Interactive Commands (during logging):
    i <seconds>  - Change interval (e.g., 'i 0.5' for 500ms)
    q            - Quit logging
    s            - Show current status
"""

import argparse
import csv
import os
import sys
import time
import threading
from datetime import datetime
from typing import Optional

from owon_power_supply import OWONPowerSupply, find_owon_supplies


class DataLogger:
    """Logs OWON power supply measurements to CSV."""

    def __init__(
        self,
        resource: str,
        output_file: str = None,
        interval: float = 1.0,
        output_dir: str = "logs"
    ):
        """
        Initialize the data logger.

        Args:
            resource: VISA resource string for the power supply
            output_file: Output CSV filename (auto-generated if None)
            interval: Measurement interval in seconds
            output_dir: Directory for log files
        """
        self.resource = resource
        self.interval = interval
        self.output_dir = output_dir
        self.psu: Optional[OWONPowerSupply] = None
        self.running = False
        self._lock = threading.Lock()

        # Generate output filename if not provided
        if output_file is None:
            os.makedirs(output_dir, exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.output_file = os.path.join(output_dir, f"owon_log_{timestamp}.csv")
        else:
            self.output_file = output_file

        self.sample_count = 0
        self.start_time: Optional[float] = None

    def connect(self) -> bool:
        """Connect to the power supply."""
        try:
            self.psu = OWONPowerSupply(self.resource)
            self.psu.connect()
            idn = self.psu.identify()
            print(f"Connected to: {idn}")
            return True
        except Exception as e:
            print(f"Connection failed: {e}")
            return False

    def disconnect(self):
        """Disconnect from the power supply."""
        if self.psu:
            try:
                self.psu.disconnect()
            except:
                pass
            self.psu = None

    def set_interval(self, interval: float):
        """
        Change the measurement interval.

        Args:
            interval: New interval in seconds (minimum 0.1)
        """
        with self._lock:
            self.interval = max(0.1, interval)
            print(f"Interval changed to: {self.interval:.3f} seconds ({1/self.interval:.1f} Hz)")

    def get_interval(self) -> float:
        """Get the current measurement interval."""
        with self._lock:
            return self.interval

    def start_logging(self):
        """Start logging measurements."""
        if not self.psu:
            print("Not connected to power supply")
            return

        self.running = True
        self.start_time = time.time()
        self.sample_count = 0

        # Create CSV file with headers
        with open(self.output_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp',
                'elapsed_seconds',
                'voltage_V',
                'current_A',
                'power_W',
                'output_enabled'
            ])

        print(f"Logging to: {self.output_file}")
        print(f"Interval: {self.interval:.3f} seconds ({1/self.interval:.1f} Hz)")
        print("Press Ctrl+C to stop (or 'q' in interactive mode)")
        print("-" * 60)

        try:
            while self.running:
                self._log_sample()
                time.sleep(self.get_interval())
        except KeyboardInterrupt:
            print("\nLogging stopped by user")
        finally:
            self.stop_logging()

    def _log_sample(self):
        """Log a single measurement sample."""
        if not self.psu:
            return

        try:
            # Get measurements
            voltage, current, power = self.psu.measure_all()
            output_enabled = self.psu.get_output_state()

            # Calculate timestamps
            now = time.time()
            timestamp = datetime.now().isoformat()
            elapsed = now - self.start_time if self.start_time else 0

            # Write to CSV
            with open(self.output_file, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    timestamp,
                    f"{elapsed:.3f}",
                    f"{voltage:.4f}",
                    f"{current:.4f}",
                    f"{power:.4f}",
                    "ON" if output_enabled else "OFF"
                ])

            self.sample_count += 1

            # Print status line
            status = "ON " if output_enabled else "OFF"
            print(f"[{self.sample_count:6d}] {elapsed:8.2f}s | "
                  f"V: {voltage:7.3f} V | I: {current:7.4f} A | "
                  f"P: {power:7.3f} W | {status}")

        except Exception as e:
            print(f"Measurement error: {e}")

    def stop_logging(self):
        """Stop logging."""
        self.running = False
        if self.sample_count > 0:
            print("-" * 60)
            print(f"Logged {self.sample_count} samples to {self.output_file}")

    def get_status(self) -> dict:
        """Get current logger status."""
        elapsed = time.time() - self.start_time if self.start_time else 0
        return {
            'running': self.running,
            'sample_count': self.sample_count,
            'elapsed_seconds': elapsed,
            'interval': self.get_interval(),
            'output_file': self.output_file
        }


def interactive_mode(logger: DataLogger):
    """
    Run the logger with interactive command input.

    Allows changing interval and other settings while logging.
    """
    import select

    # Start logging in background thread
    log_thread = threading.Thread(target=logger.start_logging)
    log_thread.daemon = True
    log_thread.start()

    print("\nInteractive commands:")
    print("  i <seconds>  - Change interval (e.g., 'i 0.5')")
    print("  s            - Show status")
    print("  q            - Quit")
    print()

    try:
        while logger.running and log_thread.is_alive():
            try:
                # Non-blocking input check
                if sys.stdin in select.select([sys.stdin], [], [], 0.5)[0]:
                    line = sys.stdin.readline().strip().lower()

                    if line == 'q':
                        logger.stop_logging()
                        break
                    elif line == 's':
                        status = logger.get_status()
                        print(f"\n--- Status ---")
                        print(f"Samples: {status['sample_count']}")
                        print(f"Elapsed: {status['elapsed_seconds']:.1f}s")
                        print(f"Interval: {status['interval']:.3f}s")
                        print(f"File: {status['output_file']}")
                        print("-" * 60)
                    elif line.startswith('i '):
                        try:
                            new_interval = float(line[2:])
                            logger.set_interval(new_interval)
                        except ValueError:
                            print("Invalid interval. Use: i <seconds>")
            except (EOFError, OSError):
                # stdin closed or not available
                time.sleep(0.5)

    except KeyboardInterrupt:
        logger.stop_logging()

    log_thread.join(timeout=2.0)


def find_power_supply() -> Optional[str]:
    """Auto-detect OWON power supply."""
    print("Searching for OWON power supply...")
    supplies = find_owon_supplies()

    if not supplies:
        print("No OWON power supply found.")
        return None

    resource, idn = supplies[0]
    print(f"Found: {idn}")
    return resource


def main():
    parser = argparse.ArgumentParser(
        description="Log OWON power supply measurements to CSV"
    )
    parser.add_argument(
        '-p', '--port',
        help="Serial port (e.g., /dev/tty.usbserial-XXXX)"
    )
    parser.add_argument(
        '-i', '--interval',
        type=float,
        default=1.0,
        help="Measurement interval in seconds (default: 1.0)"
    )
    parser.add_argument(
        '-o', '--output',
        help="Output CSV file (default: auto-generated in logs/)"
    )
    parser.add_argument(
        '-d', '--output-dir',
        default='logs',
        help="Output directory for log files (default: logs)"
    )
    parser.add_argument(
        '--interactive',
        action='store_true',
        help="Enable interactive mode (change settings while logging)"
    )

    args = parser.parse_args()

    # Determine resource string
    if args.port:
        resource = f"ASRL{args.port}::INSTR"
    else:
        resource = find_power_supply()
        if not resource:
            sys.exit(1)

    # Create logger
    logger = DataLogger(
        resource=resource,
        output_file=args.output,
        interval=args.interval,
        output_dir=args.output_dir
    )

    # Connect
    if not logger.connect():
        sys.exit(1)

    try:
        if args.interactive:
            interactive_mode(logger)
        else:
            logger.start_logging()
    finally:
        logger.disconnect()


if __name__ == "__main__":
    main()
