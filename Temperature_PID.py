"""
SCPI Communication Module using PyVISA

This module provides a base architecture for communicating with
message-based SCPI instruments using PyVISA.
"""

import pyvisa
from typing import Optional


class SCPIDevice:
    """Base class for SCPI instrument communication."""

    def __init__(self, resource_name: str, timeout: int = 5000):
        """
        Initialize connection to a SCPI device.

        Args:
            resource_name: VISA resource string (e.g., 'TCPIP::192.168.1.1::INSTR',
                          'USB0::0x1234::0x5678::SERIAL::INSTR',
                          'GPIB0::1::INSTR')
            timeout: Communication timeout in milliseconds
        """
        self.resource_name = resource_name
        self.timeout = timeout
        self.rm: Optional[pyvisa.ResourceManager] = None
        self.instrument: Optional[pyvisa.resources.MessageBasedResource] = None

    def connect(self) -> None:
        """Open connection to the instrument."""
        self.rm = pyvisa.ResourceManager()
        self.instrument = self.rm.open_resource(self.resource_name)
        self.instrument.timeout = self.timeout
        # Configure for message-based communication
        self.instrument.read_termination = '\n'
        self.instrument.write_termination = '\n'

    def disconnect(self) -> None:
        """Close connection to the instrument."""
        if self.instrument:
            self.instrument.close()
            self.instrument = None
        if self.rm:
            self.rm.close()
            self.rm = None

    def write(self, command: str) -> None:
        """
        Send a command to the instrument (no response expected).

        Args:
            command: SCPI command string
        """
        if not self.instrument:
            raise ConnectionError("Not connected to instrument")
        self.instrument.write(command)

    def read(self) -> str:
        """
        Read response from the instrument.

        Returns:
            Response string from instrument
        """
        if not self.instrument:
            raise ConnectionError("Not connected to instrument")
        return self.instrument.read().strip()

    def query(self, command: str) -> str:
        """
        Send a command and read the response.

        Args:
            command: SCPI query command (typically ends with '?')

        Returns:
            Response string from instrument
        """
        if not self.instrument:
            raise ConnectionError("Not connected to instrument")
        return self.instrument.query(command).strip()

    def query_binary(self, command: str, datatype: str = 'f') -> list:
        """
        Query binary data from the instrument.

        Args:
            command: SCPI query command for binary data
            datatype: Data type format character (e.g., 'f' for float, 'd' for double)

        Returns:
            List of values
        """
        if not self.instrument:
            raise ConnectionError("Not connected to instrument")
        return self.instrument.query_binary_values(command, datatype=datatype)

    # Standard SCPI Commands (IEEE 488.2)

    def identify(self) -> str:
        """Query instrument identification string."""
        return self.query("*IDN?")

    def reset(self) -> None:
        """Reset instrument to default state."""
        self.write("*RST")

    def clear_status(self) -> None:
        """Clear status registers."""
        self.write("*CLS")

    def self_test(self) -> int:
        """
        Run self-test and return result.

        Returns:
            0 if test passed, non-zero if failed
        """
        return int(self.query("*TST?"))

    def wait_complete(self) -> None:
        """Wait for all pending operations to complete."""
        self.query("*OPC?")

    def get_error(self) -> str:
        """
        Query the error queue.

        Returns:
            Error string (typically 'code,"message"')
        """
        return self.query("SYST:ERR?")

    def check_errors(self) -> list[str]:
        """
        Read all errors from the error queue.

        Returns:
            List of error strings
        """
        errors = []
        while True:
            error = self.get_error()
            if error.startswith("0,") or error.startswith("+0,"):
                break
            errors.append(error)
        return errors

    def __enter__(self):
        """Context manager entry."""
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.disconnect()
        return False


def list_resources() -> list[str]:
    """
    List all available VISA resources.

    Returns:
        List of resource strings
    """
    rm = pyvisa.ResourceManager()
    resources = list(rm.list_resources())
    rm.close()
    return resources


# Example usage
if __name__ == "__main__":
    # List available instruments
    print("Available VISA resources:")
    for resource in list_resources():
        print(f"  {resource}")

    # Example connection (uncomment and modify resource_name to use)
    # resource_name = "TCPIP::192.168.1.100::INSTR"
    #
    # with SCPIDevice(resource_name) as device:
    #     print(f"Connected to: {device.identify()}")
    #     device.reset()
    #     device.wait_complete()
    #     errors = device.check_errors()
    #     if errors:
    #         print(f"Errors: {errors}")
