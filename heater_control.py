"""
Heater Control Module

Provides safe control of a resistive heater via OWON power supply
with built-in safety limits and data logging.
"""

import time
import csv
import os
from datetime import datetime
from typing import Optional, Callable
from dataclasses import dataclass
from abc import ABC, abstractmethod

from owon_power_supply import OWONPowerSupply

try:
    from dracal_tmc100k import DracalTMC100k, find_dracal_sensors
    DRACAL_AVAILABLE = True
except ImportError:
    DRACAL_AVAILABLE = False


@dataclass
class HeaterLimits:
    """Safety limits for heater operation."""
    max_voltage: float = 24.0       # V - heater rated voltage
    max_current: float = 1.0        # A - heater rated current (24W/24V)
    max_power: float = 24.0         # W - heater rated power
    max_temperature: float = 400.0  # °C - heater max temp rating
    min_temperature: float = 0.0    # °C - minimum allowed temp


@dataclass
class HeaterState:
    """Current state of the heater system."""
    timestamp: datetime
    voltage_setpoint: float
    current_limit: float
    voltage_measured: float
    current_measured: float
    power_measured: float
    output_enabled: bool
    temperature: Optional[float] = None  # None if no sensor connected


class TemperatureSensor(ABC):
    """Abstract base class for temperature sensors."""

    @abstractmethod
    def read_temperature(self) -> float:
        """Read current temperature in °C."""
        pass

    @abstractmethod
    def connect(self) -> None:
        """Connect to the sensor."""
        pass

    @abstractmethod
    def disconnect(self) -> None:
        """Disconnect from the sensor."""
        pass


class DummyTemperatureSensor(TemperatureSensor):
    """
    Placeholder sensor for testing without hardware.
    Simulates temperature based on power input.
    """

    def __init__(self, ambient: float = 25.0, thermal_mass: float = 10.0):
        self.ambient = ambient
        self.thermal_mass = thermal_mass  # °C per Watt at steady state
        self._temperature = ambient
        self._last_update = time.time()
        self._power = 0.0

    def connect(self) -> None:
        pass

    def disconnect(self) -> None:
        pass

    def update_power(self, power: float) -> None:
        """Update the simulated power input."""
        self._power = power

    def read_temperature(self) -> float:
        """Simulate temperature based on power and thermal dynamics."""
        now = time.time()
        dt = now - self._last_update
        self._last_update = now

        # Simple first-order thermal model
        target_temp = self.ambient + (self._power * self.thermal_mass)
        tau = 30.0  # thermal time constant in seconds
        self._temperature += (target_temp - self._temperature) * (1 - pow(0.5, dt/tau))

        return self._temperature


class DataLogger:
    """Logs heater data to CSV files."""

    def __init__(self, log_dir: str = "logs"):
        self.log_dir = log_dir
        self.file = None
        self.writer = None
        self.filename = None

    def start(self, prefix: str = "heater_log") -> str:
        """Start a new log file. Returns the filename."""
        os.makedirs(self.log_dir, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = os.path.join(self.log_dir, f"{prefix}_{timestamp}.csv")

        self.file = open(self.filename, 'w', newline='')
        self.writer = csv.writer(self.file)

        # Write header
        self.writer.writerow([
            "timestamp",
            "elapsed_seconds",
            "voltage_setpoint",
            "current_limit",
            "voltage_measured",
            "current_measured",
            "power_measured",
            "output_enabled",
            "temperature"
        ])
        self.file.flush()
        self._start_time = time.time()

        return self.filename

    def log(self, state: HeaterState) -> None:
        """Log a heater state entry."""
        if self.writer is None:
            raise RuntimeError("Logger not started. Call start() first.")

        elapsed = time.time() - self._start_time

        self.writer.writerow([
            state.timestamp.isoformat(),
            f"{elapsed:.3f}",
            f"{state.voltage_setpoint:.3f}",
            f"{state.current_limit:.3f}",
            f"{state.voltage_measured:.3f}",
            f"{state.current_measured:.3f}",
            f"{state.power_measured:.3f}",
            1 if state.output_enabled else 0,
            f"{state.temperature:.2f}" if state.temperature is not None else ""
        ])
        self.file.flush()

    def stop(self) -> None:
        """Close the log file."""
        if self.file:
            self.file.close()
            self.file = None
            self.writer = None


class HeaterController:
    """
    Safe controller for resistive heater via OWON power supply.

    Includes:
    - Configurable safety limits
    - Automatic limit enforcement
    - Data logging
    - Temperature sensor integration
    """

    def __init__(
        self,
        psu: OWONPowerSupply,
        limits: Optional[HeaterLimits] = None,
        temp_sensor: Optional[TemperatureSensor] = None,
    ):
        self.psu = psu
        self.limits = limits or HeaterLimits()
        self.temp_sensor = temp_sensor
        self.logger = DataLogger()
        self._logging = False
        self._emergency_stop = False

    def _enforce_limits(self, voltage: float, current: float) -> tuple[float, float]:
        """Enforce safety limits on voltage and current."""
        voltage = max(0, min(voltage, self.limits.max_voltage))
        current = max(0, min(current, self.limits.max_current))

        # Also limit by power
        if voltage * current > self.limits.max_power:
            # Scale down current to meet power limit
            current = self.limits.max_power / voltage if voltage > 0 else 0

        return voltage, current

    def _check_temperature_limits(self) -> bool:
        """Check if temperature is within limits. Returns True if safe."""
        if self.temp_sensor is None:
            return True

        try:
            temp = self.temp_sensor.read_temperature()
            if temp > self.limits.max_temperature:
                print(f"WARNING: Temperature {temp:.1f}°C exceeds limit {self.limits.max_temperature}°C!")
                return False
            if temp < self.limits.min_temperature:
                print(f"WARNING: Temperature {temp:.1f}°C below minimum {self.limits.min_temperature}°C!")
                return False
        except Exception as e:
            print(f"WARNING: Failed to read temperature: {e}")
            # Fail-safe: return False if we can't read temperature
            return False

        return True

    def get_state(self) -> HeaterState:
        """Get current heater state."""
        v, i, p = self.psu.measure_all()

        temp = None
        if self.temp_sensor:
            try:
                temp = self.temp_sensor.read_temperature()
            except Exception:
                pass

        return HeaterState(
            timestamp=datetime.now(),
            voltage_setpoint=self.psu.get_voltage_setpoint(),
            current_limit=self.psu.get_current_setpoint(),
            voltage_measured=v,
            current_measured=i,
            power_measured=p,
            output_enabled=self.psu.get_output_state(),
            temperature=temp
        )

    def set_output(self, voltage: float, current: float) -> None:
        """Set voltage and current with safety limits enforced."""
        if self._emergency_stop:
            raise RuntimeError("Emergency stop active. Call reset_emergency() first.")

        voltage, current = self._enforce_limits(voltage, current)
        self.psu.set_voltage(voltage)
        self.psu.set_current(current)

    def enable(self) -> bool:
        """Enable output. Returns False if blocked by safety check."""
        if self._emergency_stop:
            print("Cannot enable: Emergency stop active.")
            return False

        if not self._check_temperature_limits():
            print("Cannot enable: Temperature out of limits.")
            return False

        self.psu.output_on()
        return True

    def disable(self) -> None:
        """Disable output immediately."""
        self.psu.output_off()

    def emergency_stop(self) -> None:
        """Emergency stop - disable output and prevent re-enabling."""
        self._emergency_stop = True
        self.psu.output_off()
        self.psu.set_voltage(0)
        print("!!! EMERGENCY STOP ACTIVATED !!!")

    def reset_emergency(self) -> None:
        """Reset emergency stop state."""
        self._emergency_stop = False
        print("Emergency stop reset.")

    def start_logging(self, prefix: str = "heater_log") -> str:
        """Start data logging. Returns log filename."""
        filename = self.logger.start(prefix)
        self._logging = True
        print(f"Logging to: {filename}")
        return filename

    def stop_logging(self) -> None:
        """Stop data logging."""
        self._logging = False
        self.logger.stop()
        print("Logging stopped.")

    def log_state(self) -> HeaterState:
        """Get current state and log it if logging is active."""
        state = self.get_state()
        if self._logging:
            self.logger.log(state)
        return state

    def print_status(self) -> None:
        """Print current status to console."""
        state = self.get_state()

        print(f"\n{'='*50}")
        print(f"Heater Status - {state.timestamp.strftime('%H:%M:%S')}")
        print(f"{'='*50}")
        print(f"Output:      {'ON' if state.output_enabled else 'OFF'}")
        print(f"V Setpoint:  {state.voltage_setpoint:.2f} V")
        print(f"I Limit:     {state.current_limit:.3f} A")
        print(f"V Measured:  {state.voltage_measured:.3f} V")
        print(f"I Measured:  {state.current_measured:.3f} A")
        print(f"Power:       {state.power_measured:.3f} W")
        if state.temperature is not None:
            print(f"Temperature: {state.temperature:.1f} °C")
        print(f"{'='*50}")


def manual_control_session(resource: str = "ASRL/dev/tty.usbserial-110::INSTR"):
    """
    Interactive manual control session for testing.

    Commands:
        v <value>  - Set voltage
        i <value>  - Set current
        on         - Enable output
        off        - Disable output
        s          - Show status
        log        - Start logging
        stoplog    - Stop logging
        stop       - Emergency stop
        reset      - Reset emergency stop
        q          - Quit
    """
    print("\n" + "="*60)
    print("HEATER MANUAL CONTROL SESSION")
    print("="*60)

    # Define heater-specific limits
    limits = HeaterLimits(
        max_voltage=24.0,
        max_current=1.0,
        max_power=24.0,
        max_temperature=400.0
    )

    print(f"\nSafety Limits:")
    print(f"  Max Voltage: {limits.max_voltage} V")
    print(f"  Max Current: {limits.max_current} A")
    print(f"  Max Power:   {limits.max_power} W")
    print(f"  Max Temp:    {limits.max_temperature} °C")

    with OWONPowerSupply(resource) as psu:
        # Auto-detect Dracal TMC100k sensor, fall back to dummy
        sensor = None
        if DRACAL_AVAILABLE:
            try:
                sensors = find_dracal_sensors()
                if sensors:
                    port, identity = sensors[0]
                    print(f"\nFound {identity} on {port}")
                    sensor = DracalTMC100k(port=port)
                    sensor.connect()
                    print(f"Thermocouple: {sensor.read_temperature():.1f} C")
            except Exception as e:
                print(f"\nFailed to connect to Dracal sensor: {e}")
                sensor = None

        if sensor is None:
            print("\nUsing simulated temperature sensor.")
            sensor = DummyTemperatureSensor()

        controller = HeaterController(psu, limits, sensor)

        # Ensure output is off at start
        controller.disable()
        controller.set_output(0, 0)

        print(f"\nConnected to: {psu.identify()}")
        print("\nCommands: v <V>, i <A>, on, off, s, log, stoplog, stop, reset, q")
        print("-"*60)

        try:
            while True:
                try:
                    cmd = input("\n> ").strip().lower()
                except EOFError:
                    break

                if not cmd:
                    continue

                parts = cmd.split()
                action = parts[0]

                if action == 'q':
                    print("Disabling output and exiting...")
                    controller.disable()
                    break

                elif action == 'v' and len(parts) > 1:
                    try:
                        voltage = float(parts[1])
                        controller.set_output(voltage, psu.get_current_setpoint())
                        print(f"Voltage set to {voltage:.2f} V")
                    except ValueError:
                        print("Invalid voltage value")

                elif action == 'i' and len(parts) > 1:
                    try:
                        current = float(parts[1])
                        controller.set_output(psu.get_voltage_setpoint(), current)
                        print(f"Current limit set to {current:.3f} A")
                    except ValueError:
                        print("Invalid current value")

                elif action == 'on':
                    if controller.enable():
                        print("Output ENABLED")

                elif action == 'off':
                    controller.disable()
                    print("Output DISABLED")

                elif action == 's':
                    controller.print_status()

                elif action == 'log':
                    controller.start_logging()

                elif action == 'stoplog':
                    controller.stop_logging()

                elif action == 'stop':
                    controller.emergency_stop()

                elif action == 'reset':
                    controller.reset_emergency()

                else:
                    print("Unknown command. Try: v, i, on, off, s, log, stoplog, stop, reset, q")

                # Update simulated sensor power (only for dummy sensor)
                if isinstance(sensor, DummyTemperatureSensor):
                    state = controller.get_state()
                    sensor.update_power(state.power_measured)

        finally:
            # Always disable output, stop logging, and disconnect sensor on exit
            controller.disable()
            if controller._logging:
                controller.stop_logging()
            if sensor is not None:
                sensor.disconnect()
            print("\nSession ended. Output disabled.")


if __name__ == "__main__":
    manual_control_session()
