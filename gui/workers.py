"""
Background worker threads for instrument communication.
"""

import time
from typing import Optional

import numpy as np
from PyQt6.QtCore import QThread, pyqtSignal
import pyvisa

from gui.state import PowerSupplyState, TemperatureState, CameraState, PyrometerState
from owon_power_supply import OWONPowerSupply


class PowerSupplyWorker(QThread):
    """Background thread for power supply communication."""

    state_updated = pyqtSignal(PowerSupplyState)

    def __init__(self, resource: str, poll_interval: float = 0.5):
        super().__init__()
        self.resource = resource
        self.poll_interval = poll_interval
        self.psu: Optional[OWONPowerSupply] = None
        self.running = False
        self._command_queue = []
        self._consecutive_failures = 0
        self._poll_counter = 0

    def run(self):
        """Main worker loop."""
        self.running = True
        state = PowerSupplyState()

        # Connect
        try:
            self.psu = OWONPowerSupply(self.resource)
            self.psu.connect()
            state.connected = True
        except Exception as e:
            state.connected = False
            state.error = str(e)
            self.state_updated.emit(state)
            return

        # Main polling loop
        while self.running:
            try:
                # Process any queued commands
                while self._command_queue:
                    cmd, args = self._command_queue.pop(0)
                    self._execute_command(cmd, args)

                # Read high-rate telemetry each cycle
                v, i, p = self.psu.measure_all()
                state.voltage_measured = v
                state.current_measured = i
                state.power_measured = p
                state.output_enabled = self.psu.get_output_state()

                # Read slower-moving values less frequently to reduce bus load
                if self._poll_counter % 5 == 0:
                    state.voltage_setpoint = self.psu.get_voltage_setpoint()
                    state.current_setpoint = self.psu.get_current_setpoint()
                    state.ovp_limit = self.psu.get_ovp()
                    state.ocp_limit = self.psu.get_ocp()

                self._poll_counter += 1
                state.connected = True
                state.error = ""
                self._consecutive_failures = 0

            except Exception as e:
                state.error = str(e)
                self._consecutive_failures += 1

                # If repeated VISA timeouts happen, force reconnect path
                if self._is_timeout_error(e) and self._consecutive_failures >= 3:
                    state.connected = False
                    if self._reconnect():
                        state.connected = True
                        state.error = ""
                        self._consecutive_failures = 0
                        self._poll_counter = 0

            self.state_updated.emit(state)
            time.sleep(self.poll_interval)

        # Cleanup
        if self.psu:
            try:
                self.psu.disconnect()
            except Exception:
                pass

    def _is_timeout_error(self, exc: Exception) -> bool:
        """Return True if exception is a VISA timeout."""
        if isinstance(exc, pyvisa.errors.VisaIOError):
            return exc.error_code == pyvisa.constants.VI_ERROR_TMO
        return "VI_ERROR_TMO" in str(exc)

    def _reconnect(self) -> bool:
        """Attempt to re-establish PSU connection after repeated failures."""
        try:
            if self.psu:
                try:
                    self.psu.disconnect()
                except Exception:
                    pass
            self.psu = OWONPowerSupply(self.resource)
            self.psu.connect()
            return True
        except Exception:
            return False

    def _execute_command(self, cmd: str, args: tuple):
        """Execute a command on the power supply. Raises on failure so the
        caller can surface the error through state.error."""
        if not self.psu:
            return

        if cmd == "set_voltage":
            self.psu.set_voltage(args[0])
        elif cmd == "set_current":
            self.psu.set_current(args[0])
        elif cmd == "output_on":
            self.psu.output_on()
        elif cmd == "output_off":
            self.psu.output_off()
        elif cmd == "set_ovp":
            self.psu.set_ovp(args[0])
        elif cmd == "set_ocp":
            self.psu.set_ocp(args[0])
        elif cmd == "emergency_stop":
            self.psu.output_off()
            self.psu.set_voltage(0)
            self.psu.set_current(0)

    def queue_command(self, cmd: str, *args):
        """Queue a command for execution."""
        self._command_queue.append((cmd, args))

    def stop(self):
        """Stop the worker thread."""
        self.running = False


class ThermocoupleWorker(QThread):
    """Background thread for Dracal TMC100k thermocouple communication."""

    state_updated = pyqtSignal(TemperatureState)

    def __init__(self, port: Optional[str] = None, poll_interval: float = 0.5):
        super().__init__()
        self.port = port
        self.poll_interval = poll_interval
        self.running = False
        self._sensor = None

    def run(self):
        """Main worker loop."""
        self.running = True
        state = TemperatureState()

        # Import here to avoid hard dependency at module level
        try:
            from dracal_tmc100k import DracalTMC100k, find_dracal_sensors
        except ImportError:
            state.error = "dracal_tmc100k module not available"
            self.state_updated.emit(state)
            return

        # Auto-detect port if not specified
        port = self.port
        if not port:
            try:
                sensors = find_dracal_sensors()
                if sensors:
                    port = sensors[0][0]
                    state.device_info = sensors[0][1]
            except Exception:
                pass

        if not port:
            state.error = "No Dracal TMC100k sensor detected."
            self.state_updated.emit(state)
            return

        # Connect
        try:
            self._sensor = DracalTMC100k(port=port)
            self._sensor.connect()
            state.connected = True

            # Get device info
            try:
                info = self._sensor.get_info()
                state.product_id = info.get("product_id", "")
                state.serial_number = info.get("serial_number", "")
                state.device_info = f"{state.product_id} {state.serial_number}".strip()
            except Exception:
                state.device_info = port

            self.state_updated.emit(state)
        except Exception as e:
            state.connected = False
            state.error = str(e)
            self.state_updated.emit(state)
            return

        # Main polling loop
        while self.running:
            try:
                tc_temp = self._sensor.read_temperature()
                state.temperature = tc_temp

                try:
                    state.cold_junction = self._sensor.read_cold_junction()
                except RuntimeError:
                    pass

                state.connected = True
                state.error = ""
                self.state_updated.emit(state)

            except Exception as e:
                state.error = str(e)
                self.state_updated.emit(state)

            time.sleep(self.poll_interval)

        # Cleanup
        if self._sensor:
            try:
                self._sensor.disconnect()
            except Exception:
                pass

    def stop(self):
        """Stop the thermocouple worker thread."""
        self.running = False


class RheedCameraWorker(QThread):
    """Background thread for RHEED camera frame acquisition."""

    state_updated = pyqtSignal(CameraState)

    def __init__(self, mode: str = "dummy", poll_interval: float = 1.0):
        super().__init__()
        self.mode = mode
        self.poll_interval = poll_interval
        self.running = False
        self._camera = None

    def run(self):
        """Main worker loop — connect camera and emit frames."""
        self.running = True
        state = CameraState(mode=self.mode)

        # Create camera driver based on mode
        try:
            self._camera = self._create_camera()
            self._camera.connect()
            state.connected = True
        except Exception as e:
            state.connected = False
            state.error = str(e)
            self.state_updated.emit(state)
            return

        frame_count = 0
        fps_start = time.time()
        fps_frame_count = 0

        while self.running:
            try:
                frame = self._camera.read_frame()
                frame_count += 1
                fps_frame_count += 1

                # Compute FPS over a rolling 1-second window
                elapsed = time.time() - fps_start
                if elapsed >= 1.0:
                    state.fps = fps_frame_count / elapsed
                    fps_start = time.time()
                    fps_frame_count = 0

                state.frame = frame
                state.frame_number = frame_count
                state.height, state.width = frame.shape[:2]
                state.connected = True
                state.error = ""

            except Exception as e:
                state.error = str(e)
                state.frame = None

            self.state_updated.emit(state)
            time.sleep(self.poll_interval)

        # Cleanup
        if self._camera is not None:
            try:
                self._camera.disconnect()
            except Exception:
                pass

    def _create_camera(self):
        """Factory method — import and instantiate camera driver."""
        if self.mode == "direct":
            from drivers.rheed_camera import VmbCamera
            return VmbCamera()
        elif self.mode == "screengrab":
            from drivers.rheed_camera import ScreenGrabCamera
            return ScreenGrabCamera()
        else:
            from drivers.rheed_camera import DummyCamera
            return DummyCamera()

    def stop(self):
        """Stop the camera worker thread."""
        self.running = False


class PyrometerWorker(QThread):
    """Background thread for pyrometer temperature polling."""

    state_updated = pyqtSignal(PyrometerState)

    def __init__(self, mode: str = "dummy", poll_interval: float = 0.5):
        super().__init__()
        self.mode = mode
        self.poll_interval = poll_interval
        self.running = False
        self._sensor = None

    def run(self):
        """Main worker loop — connect pyrometer and emit temperatures."""
        self.running = True
        state = PyrometerState(mode=self.mode)

        try:
            self._sensor = self._create_sensor()
            self._sensor.connect()
            state.connected = True

            # Get device info if available
            if hasattr(self._sensor, "get_info"):
                try:
                    info = self._sensor.get_info()
                    parts = []
                    if "name" in info:
                        parts.append(info["name"])
                    if "serial" in info:
                        parts.append(f"S/N: {info['serial']}")
                    state.device_info = " | ".join(parts) if parts else self.mode
                except Exception:
                    state.device_info = self.mode
            else:
                state.device_info = self.mode

            self.state_updated.emit(state)

        except Exception as e:
            state.connected = False
            state.error = str(e)
            self.state_updated.emit(state)
            return

        while self.running:
            try:
                state.temperature = self._sensor.read_temperature()
                state.connected = True
                state.error = ""
            except Exception as e:
                state.error = str(e)

            self.state_updated.emit(state)
            time.sleep(self.poll_interval)

        # Cleanup
        if self._sensor is not None:
            try:
                self._sensor.disconnect()
            except Exception:
                pass

    def _create_sensor(self):
        """Factory method — import and instantiate pyrometer driver."""
        if self.mode == "modbus":
            from drivers.pyrometer import ModbusPyrometer
            return ModbusPyrometer()
        elif self.mode == "screengrab":
            from drivers.pyrometer import ScreenGrabPyrometer
            return ScreenGrabPyrometer()
        else:
            from drivers.pyrometer import DummyPyrometer
            return DummyPyrometer()

    def stop(self):
        """Stop the pyrometer worker thread."""
        self.running = False
