"""
Background worker threads for instrument communication.
"""

import time
from typing import Optional

from PyQt6.QtCore import QThread, pyqtSignal

from gui.state import PowerSupplyState, TemperatureState
from owon_power_supply import OWONPowerSupply


class PowerSupplyWorker(QThread):
    """Background thread for power supply communication."""

    state_updated = pyqtSignal(PowerSupplyState)

    def __init__(self, resource: str):
        super().__init__()
        self.resource = resource
        self.psu: Optional[OWONPowerSupply] = None
        self.running = False
        self._command_queue = []

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

                # Read current state
                v, i, p = self.psu.measure_all()
                state.voltage_measured = v
                state.current_measured = i
                state.power_measured = p
                state.voltage_setpoint = self.psu.get_voltage_setpoint()
                state.current_setpoint = self.psu.get_current_setpoint()
                state.output_enabled = self.psu.get_output_state()
                state.ovp_limit = self.psu.get_ovp()
                state.ocp_limit = self.psu.get_ocp()
                state.connected = True
                state.error = ""

            except Exception as e:
                state.error = str(e)

            self.state_updated.emit(state)
            time.sleep(0.5)  # 2 Hz update rate

        # Cleanup
        if self.psu:
            try:
                self.psu.disconnect()
            except Exception:
                pass

    def _execute_command(self, cmd: str, args: tuple):
        """Execute a command on the power supply."""
        if not self.psu:
            return

        try:
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
        except Exception as e:
            print(f"Command error: {e}")

    def queue_command(self, cmd: str, *args):
        """Queue a command for execution."""
        self._command_queue.append((cmd, args))

    def stop(self):
        """Stop the worker thread."""
        self.running = False


class ThermocoupleWorker(QThread):
    """Background thread for Dracal TMC100k thermocouple communication."""

    state_updated = pyqtSignal(TemperatureState)

    def __init__(self, port: Optional[str] = None):
        super().__init__()
        self.port = port
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

            time.sleep(0.5)

        # Cleanup
        if self._sensor:
            try:
                self._sensor.disconnect()
            except Exception:
                pass

    def stop(self):
        """Stop the worker thread."""
        self.running = False
