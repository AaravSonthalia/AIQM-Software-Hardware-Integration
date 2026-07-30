"""
Eurotherm Temperature Controller via Modbus TCP/IP.

This is a Python port of temp_pid.m for the CloudMBE system.
The Eurotherm controller uses Modbus TCP protocol on port 502.
"""

import time
import logging
from collections import deque
from typing import Optional, Tuple
from pymodbus.client import ModbusTcpClient
from pymodbus.exceptions import ModbusException

logger = logging.getLogger(__name__)


class TempPID:
    """
    Eurotherm Temperature PID Controller.

    Communicates via Modbus TCP/IP on port 502.

    Attributes:
        ip: IP address of the Eurotherm controller
        setpoint_rate: Temperature ramp rate in C/minute
        max_temp: Maximum allowed temperature (safety limit)

    Example:
        >>> controller = TempPID("10.120.40.170")
        >>> controller.connect()
        >>> print(f"Current temp: {controller.get_temp_c()}C")
        >>> controller.set_temp(200.0)
        >>> controller.disconnect()
    """

    # Modbus register addresses (pymodbus uses 0-based, MATLAB uses 1-based)
    # All addresses are MATLAB address - 1
    REG_TEMP_PV = 1          # Process variable (current temp) - MATLAB reg 2
    REG_TEMP_SP = 2          # Final setpoint - MATLAB reg 3
    REG_CURRENT_SP = 5       # Current (ramping) setpoint - MATLAB reg 6
    REG_PID_P = 6            # Proportional gain - MATLAB reg 7
    REG_PID_I = 8            # Integral time - MATLAB reg 9
    REG_PID_D = 9            # Derivative time - MATLAB reg 10
    REG_SP_RATE = 35         # Setpoint rate - MATLAB reg 36
    REG_OUT_RATE = 36        # Output power rate - MATLAB reg 37
    REG_MAX_TEMP = 111       # Maximum temperature limit - MATLAB reg 112
    REG_AUTO_MANUAL = 273    # Auto (0) / Manual (1) mode - MATLAB reg 274
    REG_RAMP_ENABLE = 276    # Ramp enable - MATLAB reg 277
    REG_TARGET_SP = 485      # Target setpoint - MATLAB reg 486

    # Safeguards from MATLAB GUI (GUI_2024_6_7_multi.mlapp line 385, 2072)
    TEMP_MIN = 20.0    # Minimum allowed temperature (C)
    TEMP_MAX = 930.0   # Maximum allowed temperature (C) - thilo = [20 930]
    HIGH_TEMP_THRESHOLD = 890.0  # Above this, reduce ramp rate (line 827-831)
    HIGH_TEMP_RAMP_RATE = 2.0    # Ramp rate at high temps (C/min)
    NORMAL_RAMP_RATE = 5.0       # Normal ramp rate (C/min)

    def __init__(
        self,
        ip: str = "10.120.40.170",
        port: int = 502,
        setpoint_rate: float = 5.0,
        max_temp: float = 600.0,
        memory_length: int = 800,
        t_stability: float = 300.0,
        t_error: float = 0.15,
    ):
        """
        Initialize the Eurotherm temperature controller.

        Args:
            ip: IP address of the controller
            port: Modbus TCP port (default 502)
            setpoint_rate: Temperature ramp rate in C/minute
            max_temp: Maximum allowed temperature (silver melting point default)
            memory_length: Number of temperature samples to store
            t_stability: Time in seconds to check for stability
            t_error: Allowed temperature fluctuation for stability check
        """
        super().__init__()
        self.ip = ip
        self.port = port
        self.setpoint_rate = setpoint_rate
        self.max_temp = max_temp
        self.memory_length = memory_length
        self.t_stability = t_stability
        self.t_error = t_error

        self._client: Optional[ModbusTcpClient] = None
        self._setpoint = -1.0
        self._final_sp = -1.0
        self._temp_history: deque = deque(maxlen=memory_length)
        self._out_rate = 10

    def connect(self, max_retries: int = 100, delay: float = 10.0):
        """
        Connect to the Eurotherm controller.

        Args:
            max_retries: Maximum number of connection attempts
            delay: Delay between retries in seconds
        """
        for attempt in range(1, max_retries + 1):
            try:
                self._client = ModbusTcpClient(self.ip, port=self.port, timeout=20)
                if self._client.connect():
                    self._connected = True
                    # Try to initialize settings (may fail if power supply is off)
                    try:
                        self._write_register(self.REG_SP_RATE, int(self.setpoint_rate * 10))
                        self._write_register(self.REG_MAX_TEMP, int(self.max_temp * 10))
                    except Exception as e:
                        logger.warning(f"Could not write initial settings (power off?): {e}")
                    logger.info(f"Connected to Eurotherm at {self.ip}")
                    return
            except Exception as e:
                logger.warning(f"Connection attempt {attempt} failed: {e}")
                if attempt < max_retries:
                    time.sleep(delay)

        raise ConnectionError(f"Failed to connect to Eurotherm after {max_retries} retries")

    def disconnect(self):
        """Disconnect from the controller."""
        if self._client:
            self._client.close()
            self._client = None
            self._connected = False
            logger.info("Disconnected from Eurotherm")

    def reconnect(self):
        """Reconnect to the controller (for error recovery)."""
        self.disconnect()
        self.connect()

    def is_connected(self) -> bool:
        """Check if controller is responding."""
        try:
            temp = self.get_temp_c()
            return temp != -1
        except Exception:
            return False

    def _read_register(self, address: int, count: int = 1) -> int:
        """Read a holding register."""
        if not self._client:
            raise ConnectionError("Not connected to controller")
        result = self._client.read_holding_registers(address=address, count=count)
        if result.isError():
            raise ModbusException(f"Failed to read register {address}")
        return result.registers[0] if count == 1 else result.registers

    def _write_register(self, address: int, value: int):
        """Write to a holding register."""
        if not self._client:
            raise ConnectionError("Not connected to controller")
        result = self._client.write_register(address=address, value=value)
        if result.isError():
            raise ModbusException(f"Failed to write to register {address}")

    def get_temp_c(self) -> float:
        """Get current temperature in Celsius."""
        try:
            return self._read_register(self.REG_TEMP_PV) / 10.0
        except Exception as e:
            logger.error(f"Failed to read temperature: {e}")
            return -1.0

    def get_final_setpoint(self) -> float:
        """Get the final target setpoint."""
        self._final_sp = self._read_register(self.REG_TEMP_SP) / 10.0
        return self._final_sp

    def get_current_setpoint(self) -> float:
        """Get the current (ramping) setpoint."""
        self._setpoint = self._read_register(self.REG_CURRENT_SP) / 10.0
        return self._setpoint

    def set_temp(self, temp_c: float):
        """
        Set target temperature.

        Args:
            temp_c: Target temperature in Celsius

        Raises:
            ValueError: If temperature is outside allowed range
        """
        # Safeguard: Enforce temperature limits (from MATLAB GUI line 2105)
        if temp_c < self.TEMP_MIN:
            logger.warning(f"Temperature {temp_c}C below minimum {self.TEMP_MIN}C, clamping")
            temp_c = self.TEMP_MIN
        elif temp_c > self.TEMP_MAX:
            logger.warning(f"Temperature {temp_c}C above maximum {self.TEMP_MAX}C, clamping")
            temp_c = self.TEMP_MAX

        self._setpoint = round(temp_c, 1)
        self._temp_history.clear()

        # Enable ramp if not already enabled
        if not self._read_register(self.REG_RAMP_ENABLE):
            self._write_register(self.REG_RAMP_ENABLE, 1)

        self._write_register(self.REG_TARGET_SP, int(temp_c * 10))
        logger.info(f"Set temperature to {temp_c}C")

    def auto_adjust_ramp_rate(self):
        """
        Automatically adjust ramp rate based on current temperature.

        From MATLAB GUI lines 827-831: If temp > 890C, reduce ramp rate to 2C/min.
        """
        temp = self.get_temp_c()
        if temp > self.HIGH_TEMP_THRESHOLD:
            self.set_setpoint_rate(self.HIGH_TEMP_RAMP_RATE)
            logger.info(f"High temperature ({temp}C), reduced ramp rate to {self.HIGH_TEMP_RAMP_RATE}C/min")
        else:
            self.set_setpoint_rate(self.NORMAL_RAMP_RATE)

    def is_at_setpoint(self) -> bool:
        """
        Check if temperature is stable at setpoint.

        Returns:
            True if temperature has been stable at setpoint for t_stability seconds
        """
        self.get_final_setpoint()
        current_time = time.time()

        # Get recent samples within stability window
        recent_samples = [
            (temp, ts)
            for temp, ts in self._temp_history
            if current_time - ts < self.t_stability and ts != 0
        ]

        if not recent_samples:
            return False

        # Check if all recent samples are within error tolerance
        all_within_error = all(
            abs(temp - self._final_sp) <= self.t_error for temp, _ in recent_samples
        )

        # Check for at least one old sample (long-term stability)
        old_samples = [
            (temp, ts)
            for temp, ts in self._temp_history
            if abs(current_time - ts - self.t_stability) <= 10
        ]
        has_old_stable = any(
            abs(temp - self._final_sp) <= self.t_error for temp, _ in old_samples
        )

        return all_within_error and has_old_stable

    def push_temp_c(self) -> float:
        """
        Read temperature and add to history buffer.

        Returns:
            Current temperature
        """
        temp = self.get_temp_c()
        self._temp_history.append((temp, time.time()))
        return temp

    def update_pid(self):
        """
        Update PID parameters based on current temperature.

        Uses temperature-dependent PID tuning for optimal control.
        """
        temp = self.get_temp_c()

        # PID values for different temperature ranges
        p_values = [62, 76, 183]
        i_values = [136, 49, 30]
        d_values = [23, 8, 5]
        thresholds = [400, 750]
        ramp_smoother = 100

        if temp < thresholds[0] - 0.5 * ramp_smoother:
            pid = [p_values[0], i_values[0], d_values[0]]
        elif temp < thresholds[0] + 0.5 * ramp_smoother:
            r = (temp - thresholds[0] + ramp_smoother / 2) / ramp_smoother
            pid = [
                p_values[0] + (p_values[1] - p_values[0]) * r,
                i_values[0] + (i_values[1] - i_values[0]) * r,
                d_values[0] + (d_values[1] - d_values[0]) * r,
            ]
        elif temp < thresholds[1] - 0.5 * ramp_smoother:
            pid = [p_values[1], i_values[1], d_values[1]]
        elif temp < thresholds[1] + 0.5 * ramp_smoother:
            r = (temp - thresholds[1] + ramp_smoother / 2) / ramp_smoother
            pid = [
                p_values[1] + (p_values[2] - p_values[1]) * r,
                i_values[1] + (i_values[2] - i_values[1]) * r,
                d_values[1] + (d_values[2] - d_values[1]) * r,
            ]
        else:
            pid = [p_values[2], i_values[2], d_values[2]]

        # Write PID values (P is scaled by 10)
        self._write_register(self.REG_PID_P, int(pid[0] * 10))
        self._write_register(self.REG_PID_I, int(pid[1]))
        self._write_register(self.REG_PID_D, int(pid[2]))

    def abort(self):
        """Abort current temperature ramp."""
        self._write_register(self.REG_RAMP_ENABLE, 0)
        logger.info("Temperature ramp aborted")

    def set_auto_mode(self):
        """Set controller to automatic mode."""
        self._write_register(self.REG_AUTO_MANUAL, 0)

    def set_manual_mode(self):
        """Set controller to manual mode."""
        self._write_register(self.REG_AUTO_MANUAL, 1)

    def set_setpoint_rate(self, rate: float):
        """
        Set temperature ramp rate.

        Args:
            rate: Ramp rate in C/minute
        """
        self.setpoint_rate = rate
        self._write_register(self.REG_SP_RATE, int(rate * 10))

    def set_output_power_rate(self, rate: int):
        """
        Set output power rate.

        Args:
            rate: Output power rate
        """
        self._out_rate = rate
        self._write_register(self.REG_OUT_RATE, rate)

    def get_status(self) -> dict:
        """Get current controller status."""
        return {
            "temperature": self.get_temp_c(),
            "current_setpoint": self.get_current_setpoint(),
            "final_setpoint": self.get_final_setpoint(),
            "at_setpoint": self.is_at_setpoint(),
            "connected": self.is_connected(),
        }
