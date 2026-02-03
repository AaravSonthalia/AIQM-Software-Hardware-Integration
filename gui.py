"""
OWON Power Supply GUI

Dashboard-style interface for controlling OWON SPE series power supplies.
Features Simple and Advanced view modes.

Requirements:
    pip install PyQt6 pyqtgraph pyvisa pyvisa-py pyserial

Usage:
    python gui.py
    python gui.py /dev/tty.usbserial-XXXX  # specify port manually
"""

import sys
import time
from datetime import datetime
from typing import Optional
from dataclasses import dataclass

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QPushButton, QLineEdit, QGroupBox,
    QFrame, QMessageBox, QComboBox, QCheckBox, QSpinBox,
    QDoubleSpinBox, QStatusBar, QSplitter
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QThread
from PyQt6.QtGui import QFont, QColor, QPalette

# Try to import pyqtgraph for plotting (optional)
try:
    import pyqtgraph as pg
    HAS_PLOTTING = True
except ImportError:
    HAS_PLOTTING = False
    print("Note: pyqtgraph not installed. Plotting disabled.")
    print("      Install with: pip install pyqtgraph")

from owon_power_supply import OWONPowerSupply, find_owon_supplies


@dataclass
class PowerSupplyState:
    """Current state of the power supply."""
    voltage_setpoint: float = 0.0
    current_setpoint: float = 0.0
    voltage_measured: float = 0.0
    current_measured: float = 0.0
    power_measured: float = 0.0
    output_enabled: bool = False
    ovp_limit: float = 0.0
    ocp_limit: float = 0.0
    connected: bool = False
    error: str = ""


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
            except:
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


class ValueDisplay(QFrame):
    """Widget for displaying a labeled value."""

    def __init__(self, label: str, unit: str = "", decimals: int = 3):
        super().__init__()
        self.unit = unit
        self.decimals = decimals

        self.setFrameStyle(QFrame.Shape.Box | QFrame.Shadow.Raised)
        self.setLineWidth(1)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 5, 10, 5)

        self.label = QLabel(label)
        self.label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.value = QLabel("---")
        self.value.setAlignment(Qt.AlignmentFlag.AlignCenter)
        font = QFont("Monospace", 24, QFont.Weight.Bold)
        self.value.setFont(font)

        layout.addWidget(self.label)
        layout.addWidget(self.value)

    def set_value(self, value: float):
        """Update the displayed value."""
        self.value.setText(f"{value:.{self.decimals}f} {self.unit}")

    def set_color(self, color: str):
        """Set the value text color."""
        self.value.setStyleSheet(f"color: {color};")


class ControlPanel(QGroupBox):
    """Panel for controlling voltage and current."""

    voltage_changed = pyqtSignal(float)
    current_changed = pyqtSignal(float)
    output_toggled = pyqtSignal(bool)

    def __init__(self, max_voltage: float = 24.0, max_current: float = 1.0):
        super().__init__("Controls")

        layout = QGridLayout(self)

        # Voltage control
        layout.addWidget(QLabel("Voltage (V):"), 0, 0)
        self.voltage_spin = QDoubleSpinBox()
        self.voltage_spin.setRange(0, max_voltage)
        self.voltage_spin.setDecimals(2)
        self.voltage_spin.setSingleStep(0.5)
        layout.addWidget(self.voltage_spin, 0, 1)

        self.voltage_btn = QPushButton("Set")
        self.voltage_btn.clicked.connect(self._on_voltage_set)
        layout.addWidget(self.voltage_btn, 0, 2)

        # Current control
        layout.addWidget(QLabel("Current (A):"), 1, 0)
        self.current_spin = QDoubleSpinBox()
        self.current_spin.setRange(0, max_current)
        self.current_spin.setDecimals(3)
        self.current_spin.setSingleStep(0.1)
        layout.addWidget(self.current_spin, 1, 1)

        self.current_btn = QPushButton("Set")
        self.current_btn.clicked.connect(self._on_current_set)
        layout.addWidget(self.current_btn, 1, 2)

        # Output toggle
        self.output_btn = QPushButton("OUTPUT OFF")
        self.output_btn.setCheckable(True)
        self.output_btn.setMinimumHeight(50)
        self.output_btn.clicked.connect(self._on_output_toggle)
        self._update_output_button(False)
        layout.addWidget(self.output_btn, 2, 0, 1, 3)

    def _on_voltage_set(self):
        self.voltage_changed.emit(self.voltage_spin.value())

    def _on_current_set(self):
        self.current_changed.emit(self.current_spin.value())

    def _on_output_toggle(self):
        enabled = self.output_btn.isChecked()
        self.output_toggled.emit(enabled)

    def _update_output_button(self, enabled: bool):
        if enabled:
            self.output_btn.setText("OUTPUT ON")
            self.output_btn.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
        else:
            self.output_btn.setText("OUTPUT OFF")
            self.output_btn.setStyleSheet("background-color: #666; color: white;")

    def update_state(self, state: PowerSupplyState):
        """Update controls to reflect current state."""
        self.output_btn.setChecked(state.output_enabled)
        self._update_output_button(state.output_enabled)


class ProtectionPanel(QGroupBox):
    """Panel for OVP/OCP settings."""

    ovp_changed = pyqtSignal(float)
    ocp_changed = pyqtSignal(float)

    def __init__(self):
        super().__init__("Protection Settings")

        layout = QGridLayout(self)

        # OVP
        layout.addWidget(QLabel("OVP (V):"), 0, 0)
        self.ovp_spin = QDoubleSpinBox()
        self.ovp_spin.setRange(0, 82)
        self.ovp_spin.setDecimals(1)
        layout.addWidget(self.ovp_spin, 0, 1)

        self.ovp_btn = QPushButton("Set")
        self.ovp_btn.clicked.connect(lambda: self.ovp_changed.emit(self.ovp_spin.value()))
        layout.addWidget(self.ovp_btn, 0, 2)

        # OCP
        layout.addWidget(QLabel("OCP (A):"), 1, 0)
        self.ocp_spin = QDoubleSpinBox()
        self.ocp_spin.setRange(0, 5.2)
        self.ocp_spin.setDecimals(2)
        layout.addWidget(self.ocp_spin, 1, 1)

        self.ocp_btn = QPushButton("Set")
        self.ocp_btn.clicked.connect(lambda: self.ocp_changed.emit(self.ocp_spin.value()))
        layout.addWidget(self.ocp_btn, 1, 2)

    def update_state(self, state: PowerSupplyState):
        """Update displayed protection values."""
        self.ovp_spin.setValue(state.ovp_limit)
        self.ocp_spin.setValue(state.ocp_limit)


class MainWindow(QMainWindow):
    """Main application window."""

    def __init__(self, resource: Optional[str] = None):
        super().__init__()

        self.setWindowTitle("OWON Power Supply Control")
        self.setMinimumSize(800, 600)

        self.worker: Optional[PowerSupplyWorker] = None
        self.resource = resource
        self.advanced_mode = False

        # Data for plotting
        self.time_data = []
        self.voltage_data = []
        self.current_data = []
        self.power_data = []
        self.start_time = time.time()

        self._setup_ui()
        self._connect_signals()

        # Auto-connect if resource provided
        if self.resource:
            self._connect_to_psu()

    def _setup_ui(self):
        """Set up the user interface."""
        central = QWidget()
        self.setCentralWidget(central)

        main_layout = QVBoxLayout(central)

        # Top bar: connection and view toggle
        top_bar = QHBoxLayout()

        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self._on_connect_clicked)
        top_bar.addWidget(self.connect_btn)

        self.status_label = QLabel("Disconnected")
        top_bar.addWidget(self.status_label)

        top_bar.addStretch()

        self.view_toggle = QCheckBox("Advanced View")
        self.view_toggle.toggled.connect(self._toggle_view)
        top_bar.addWidget(self.view_toggle)

        main_layout.addLayout(top_bar)

        # Main content area
        content = QHBoxLayout()

        # Left side: measurements
        left_panel = QVBoxLayout()

        # Measurement displays
        measurements = QGroupBox("Measurements")
        meas_layout = QGridLayout(measurements)

        self.voltage_display = ValueDisplay("Voltage", "V", 3)
        self.current_display = ValueDisplay("Current", "A", 3)
        self.power_display = ValueDisplay("Power", "W", 3)

        meas_layout.addWidget(self.voltage_display, 0, 0)
        meas_layout.addWidget(self.current_display, 0, 1)
        meas_layout.addWidget(self.power_display, 0, 2)

        left_panel.addWidget(measurements)

        # Setpoints display (simple view)
        self.setpoints_group = QGroupBox("Setpoints")
        setpoints_layout = QGridLayout(self.setpoints_group)

        self.voltage_sp_display = ValueDisplay("V Setpoint", "V", 2)
        self.current_sp_display = ValueDisplay("I Limit", "A", 3)

        setpoints_layout.addWidget(self.voltage_sp_display, 0, 0)
        setpoints_layout.addWidget(self.current_sp_display, 0, 1)

        left_panel.addWidget(self.setpoints_group)

        # Plot area (advanced view only)
        if HAS_PLOTTING:
            self.plot_widget = pg.PlotWidget(title="Power Output")
            self.plot_widget.setLabel('left', 'Value')
            self.plot_widget.setLabel('bottom', 'Time', 's')
            self.plot_widget.addLegend()
            self.plot_widget.showGrid(x=True, y=True)

            self.voltage_curve = self.plot_widget.plot(pen='y', name='Voltage (V)')
            self.current_curve = self.plot_widget.plot(pen='c', name='Current (A)')
            self.power_curve = self.plot_widget.plot(pen='m', name='Power (W)')

            self.plot_widget.setVisible(False)
            left_panel.addWidget(self.plot_widget)

        left_panel.addStretch()
        content.addLayout(left_panel, stretch=2)

        # Right side: controls
        right_panel = QVBoxLayout()

        # Control panel
        self.control_panel = ControlPanel(max_voltage=24.0, max_current=1.0)
        right_panel.addWidget(self.control_panel)

        # Protection panel (advanced view)
        self.protection_panel = ProtectionPanel()
        self.protection_panel.setVisible(False)
        right_panel.addWidget(self.protection_panel)

        # Emergency stop
        self.estop_btn = QPushButton("EMERGENCY STOP")
        self.estop_btn.setMinimumHeight(60)
        self.estop_btn.setStyleSheet("""
            QPushButton {
                background-color: #d32f2f;
                color: white;
                font-size: 18px;
                font-weight: bold;
                border-radius: 10px;
            }
            QPushButton:hover {
                background-color: #b71c1c;
            }
            QPushButton:pressed {
                background-color: #ff5252;
            }
        """)
        self.estop_btn.clicked.connect(self._on_emergency_stop)
        right_panel.addWidget(self.estop_btn)

        right_panel.addStretch()
        content.addLayout(right_panel, stretch=1)

        main_layout.addLayout(content)

        # Status bar
        self.statusBar().showMessage("Ready")

    def _connect_signals(self):
        """Connect control signals."""
        self.control_panel.voltage_changed.connect(
            lambda v: self.worker.queue_command("set_voltage", v) if self.worker else None
        )
        self.control_panel.current_changed.connect(
            lambda i: self.worker.queue_command("set_current", i) if self.worker else None
        )
        self.control_panel.output_toggled.connect(self._on_output_toggled)

        self.protection_panel.ovp_changed.connect(
            lambda v: self.worker.queue_command("set_ovp", v) if self.worker else None
        )
        self.protection_panel.ocp_changed.connect(
            lambda i: self.worker.queue_command("set_ocp", i) if self.worker else None
        )

    def _on_connect_clicked(self):
        """Handle connect button click."""
        if self.worker and self.worker.isRunning():
            self._disconnect_from_psu()
        else:
            self._connect_to_psu()

    def _connect_to_psu(self):
        """Connect to the power supply."""
        if not self.resource:
            # Auto-detect
            supplies = find_owon_supplies()
            if not supplies:
                QMessageBox.warning(self, "Not Found", "No OWON power supply detected.")
                return
            self.resource, idn = supplies[0]
            self.statusBar().showMessage(f"Found: {idn}")

        self.worker = PowerSupplyWorker(self.resource)
        self.worker.state_updated.connect(self._on_state_updated)
        self.worker.start()

        self.connect_btn.setText("Disconnect")
        self.status_label.setText("Connecting...")

    def _disconnect_from_psu(self):
        """Disconnect from the power supply."""
        if self.worker:
            self.worker.stop()
            self.worker.wait()
            self.worker = None

        self.connect_btn.setText("Connect")
        self.status_label.setText("Disconnected")
        self.resource = None

    def _on_state_updated(self, state: PowerSupplyState):
        """Handle state update from worker."""
        if not state.connected:
            self.status_label.setText(f"Error: {state.error}")
            return

        self.status_label.setText("Connected")

        # Update displays
        self.voltage_display.set_value(state.voltage_measured)
        self.current_display.set_value(state.current_measured)
        self.power_display.set_value(state.power_measured)

        self.voltage_sp_display.set_value(state.voltage_setpoint)
        self.current_sp_display.set_value(state.current_setpoint)

        # Update controls
        self.control_panel.update_state(state)
        self.protection_panel.update_state(state)

        # Color code based on output state
        if state.output_enabled:
            self.voltage_display.set_color("#4CAF50")
            self.current_display.set_color("#4CAF50")
        else:
            self.voltage_display.set_color("#888")
            self.current_display.set_color("#888")

        # Update plot data
        if HAS_PLOTTING and self.advanced_mode:
            now = time.time() - self.start_time
            self.time_data.append(now)
            self.voltage_data.append(state.voltage_measured)
            self.current_data.append(state.current_measured)
            self.power_data.append(state.power_measured)

            # Keep last 60 seconds
            while self.time_data and self.time_data[0] < now - 60:
                self.time_data.pop(0)
                self.voltage_data.pop(0)
                self.current_data.pop(0)
                self.power_data.pop(0)

            self.voltage_curve.setData(self.time_data, self.voltage_data)
            self.current_curve.setData(self.time_data, self.current_data)
            self.power_curve.setData(self.time_data, self.power_data)

    def _on_output_toggled(self, enabled: bool):
        """Handle output toggle."""
        if self.worker:
            if enabled:
                self.worker.queue_command("output_on")
            else:
                self.worker.queue_command("output_off")

    def _on_emergency_stop(self):
        """Handle emergency stop."""
        if self.worker:
            self.worker.queue_command("emergency_stop")
            self.statusBar().showMessage("!!! EMERGENCY STOP ACTIVATED !!!")

    def _toggle_view(self, advanced: bool):
        """Toggle between simple and advanced view."""
        self.advanced_mode = advanced

        self.protection_panel.setVisible(advanced)

        if HAS_PLOTTING:
            self.plot_widget.setVisible(advanced)

    def closeEvent(self, event):
        """Handle window close."""
        self._disconnect_from_psu()
        event.accept()


def main():
    # Check for manual port argument
    resource = None
    if len(sys.argv) > 1:
        port = sys.argv[1]
        resource = f"ASRL{port}::INSTR"
        print(f"Using specified port: {port}")

    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    window = MainWindow(resource)
    window.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
