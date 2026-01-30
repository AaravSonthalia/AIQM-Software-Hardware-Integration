"""
OWON SPE Series Power Supply Control Module

Provides a high-level interface for controlling OWON SPE series
programmable DC power supplies via PyVISA.
"""

import pyvisa
from typing import Optional, Tuple
from SCPI_class import SCPIDevice


class OWONPowerSupply(SCPIDevice):
    """Control interface for OWON SPE series power supplies."""

    DEFAULT_BAUD_RATE = 115200

    def __init__(
        self,
        resource_name: str = "ASRL/dev/tty.usbserial-110::INSTR",
        timeout: int = 3000,
        baud_rate: int = DEFAULT_BAUD_RATE,
    ):
        """
        Initialize connection to an OWON SPE power supply.

        Args:
            resource_name: VISA resource string for serial port
            timeout: Communication timeout in milliseconds
            baud_rate: Serial baud rate (default 115200)
        """
        super().__init__(resource_name, timeout)
        self.baud_rate = baud_rate

    def connect(self) -> None:
        """Open connection to the power supply with serial configuration."""
        self.rm = pyvisa.ResourceManager("@py")
        self.instrument = self.rm.open_resource(self.resource_name)
        self.instrument.timeout = self.timeout
        self.instrument.baud_rate = self.baud_rate
        self.instrument.read_termination = "\n"
        self.instrument.write_termination = "\n"

    # Output Control

    def output_on(self) -> None:
        """Enable the power supply output."""
        self.write("OUTP ON")

    def output_off(self) -> None:
        """Disable the power supply output."""
        self.write("OUTP OFF")

    def get_output_state(self) -> bool:
        """
        Get the current output state.

        Returns:
            True if output is enabled, False otherwise
        """
        response = self.query("OUTP?").strip()
        # Manual says returns 1 or 0, but handle ON/OFF just in case
        return response in ("1", "ON", "on")

    # Voltage Control

    def set_voltage(self, voltage: float) -> None:
        """
        Set the output voltage.

        Args:
            voltage: Target voltage in volts
        """
        self.write(f"VOLT {voltage:.3f}")

    def get_voltage_setpoint(self) -> float:
        """
        Get the voltage setpoint.

        Returns:
            Voltage setpoint in volts
        """
        return float(self.query("VOLT?"))

    def measure_voltage(self) -> float:
        """
        Measure the actual output voltage.

        Returns:
            Measured voltage in volts
        """
        return float(self.query("MEAS:VOLT?"))

    # Current Control

    def set_current(self, current: float) -> None:
        """
        Set the current limit.

        Args:
            current: Current limit in amps
        """
        self.write(f"CURR {current:.3f}")

    def get_current_setpoint(self) -> float:
        """
        Get the current limit setpoint.

        Returns:
            Current setpoint in amps
        """
        return float(self.query("CURR?"))

    def measure_current(self) -> float:
        """
        Measure the actual output current.

        Returns:
            Measured current in amps
        """
        return float(self.query("MEAS:CURR?"))

    # Power Measurement

    def measure_power(self) -> float:
        """
        Measure the output power.

        Returns:
            Measured power in watts
        """
        return float(self.query("MEAS:POW?"))

    def measure_all(self) -> Tuple[float, float, float]:
        """
        Query voltage, current, and power in a single command.

        Returns:
            Tuple of (voltage, current, power)
        """
        response = self.query("MEAS:ALL?")
        # Handle both comma-separated and space-separated responses
        if "," in response:
            values = response.split(",")
        else:
            values = response.split()

        v = float(values[0])
        i = float(values[1])
        # Some models return only V,I - calculate power if not provided
        if len(values) >= 3:
            p = float(values[2])
        else:
            p = v * i
        return v, i, p

    def measure_all_info(self) -> dict:
        """
        Query voltage, current, power, and fault/mode status.

        Returns:
            Dictionary with voltage, current, power, and status flags:
            - ovp_fault: True if over-voltage fault
            - ocp_fault: True if over-current fault
            - otp_fault: True if over-temperature fault
            - mode: 0=standby, 1=CV, 2=CC, 3=failure
        """
        response = self.query("MEAS:ALL:INFO?")
        # Handle both comma-separated and space-separated responses
        if "," in response:
            parts = response.split(",")
        else:
            parts = response.split()
        return {
            "voltage": float(parts[0]),
            "current": float(parts[1]),
            "power": float(parts[2]),
            "ovp_fault": parts[3].strip() in ("1", "ON"),
            "ocp_fault": parts[4].strip() in ("1", "ON"),
            "otp_fault": parts[5].strip() in ("1", "ON"),
            "mode": int(parts[6]),
        }

    # Protection Settings (OVP/OCP use :LIMit per OWON manual)

    def set_ovp(self, voltage: float) -> None:
        """
        Set over-voltage protection limit.

        Args:
            voltage: OVP threshold in volts
        """
        self.write(f"VOLT:LIM {voltage:.2f}")

    def get_ovp(self) -> float:
        """
        Get over-voltage protection limit.

        Returns:
            OVP threshold in volts
        """
        return float(self.query("VOLT:LIM?"))

    def set_ocp(self, current: float) -> None:
        """
        Set over-current protection limit.

        Args:
            current: OCP threshold in amps
        """
        self.write(f"CURR:LIM {current:.3f}")

    def get_ocp(self) -> float:
        """
        Get over-current protection limit.

        Returns:
            OCP threshold in amps
        """
        return float(self.query("CURR:LIM?"))

    # System Control

    def set_local(self) -> None:
        """Set power supply to local (front panel) control mode."""
        self.write("SYST:LOC")

    def set_remote(self) -> None:
        """Set power supply to remote control mode."""
        self.write("SYST:REM")

    # Convenience Methods

    def set_output(self, voltage: float, current: float, enable: bool = False) -> None:
        """
        Configure voltage, current, and optionally enable output.

        Args:
            voltage: Target voltage in volts
            current: Current limit in amps
            enable: If True, enable output after setting values
        """
        self.set_voltage(voltage)
        self.set_current(current)
        if enable:
            self.output_on()

    def get_status(self) -> dict:
        """
        Get comprehensive power supply status.

        Returns:
            Dictionary with current status information
        """
        v, i, p = self.measure_all()
        return {
            "voltage_setpoint": self.get_voltage_setpoint(),
            "current_setpoint": self.get_current_setpoint(),
            "voltage_measured": v,
            "current_measured": i,
            "power_measured": p,
            "output_enabled": self.get_output_state(),
        }

    def measure(self) -> Tuple[float, float, float]:
        """
        Get all measurements at once (voltage, current, power).

        Returns:
            Tuple of (voltage, current, power)
        """
        return self.measure_all()


def find_owon_supplies() -> list[str]:
    """
    Scan for OWON power supplies on available serial ports.

    Returns:
        List of resource strings for detected OWON supplies
    """
    import serial.tools.list_ports

    found = []
    rm = pyvisa.ResourceManager("@py")

    for port in serial.tools.list_ports.comports():
        resource = f"ASRL{port.device}::INSTR"
        try:
            inst = rm.open_resource(resource)
            inst.timeout = 2000
            inst.baud_rate = OWONPowerSupply.DEFAULT_BAUD_RATE
            inst.read_termination = "\n"
            inst.write_termination = "\n"

            idn = inst.query("*IDN?")
            if "OWON" in idn.upper():
                found.append((resource, idn.strip()))
            inst.close()
        except Exception:
            pass

    rm.close()
    return found


if __name__ == "__main__":
    print("Scanning for OWON power supplies...")
    supplies = find_owon_supplies()

    if not supplies:
        print("No OWON power supplies found.")
    else:
        for resource, idn in supplies:
            print(f"Found: {idn}")
            print(f"  Resource: {resource}")

        # Connect to first found supply
        resource, idn = supplies[0]
        print(f"\nConnecting to {idn}...")

        with OWONPowerSupply(resource) as psu:
            print(f"\n{'='*40}")
            print("Power Supply Status")
            print("="*40)

            status = psu.get_status()
            print(f"Output:    {'ON' if status['output_enabled'] else 'OFF'}")
            print(f"V Set:     {status['voltage_setpoint']:.3f} V")
            print(f"I Limit:   {status['current_setpoint']:.3f} A")
            print(f"V Actual:  {status['voltage_measured']:.3f} V")
            print(f"I Actual:  {status['current_measured']:.3f} A")
            print(f"P Actual:  {status['power_measured']:.3f} W")
