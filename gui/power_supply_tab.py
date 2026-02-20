"""
Power Supply tab widget.
"""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel,
    QPushButton, QGroupBox, QCheckBox,
)
from PyQt6.QtCore import pyqtSignal

from gui.state import PowerSupplyState
from gui.widgets import ValueDisplay, ControlPanel, ProtectionPanel
from gui.action_logger import ActionLogger


class PowerSupplyTab(QWidget):
    """Power supply control and monitoring tab."""

    connect_requested = pyqtSignal()
    disconnect_requested = pyqtSignal()
    command_requested = pyqtSignal(str, tuple)  # (command_name, args)

    def __init__(self, action_logger: ActionLogger, parent=None):
        super().__init__(parent)
        self.action_logger = action_logger
        self.advanced_mode = True

        self._build_ui()
        self._connect_signals()

    def _build_ui(self):
        layout = QVBoxLayout(self)

        # Top bar: connection and view toggle
        top_bar = QHBoxLayout()

        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self._on_connect_clicked)
        top_bar.addWidget(self.connect_btn)

        self.status_label = QLabel("Disconnected")
        top_bar.addWidget(self.status_label)
        top_bar.addStretch()

        self.view_toggle = QCheckBox("Advanced View")
        self.view_toggle.setChecked(True)
        self.view_toggle.toggled.connect(self._toggle_view)
        top_bar.addWidget(self.view_toggle)

        layout.addLayout(top_bar)

        # Main content area
        content = QHBoxLayout()

        # Left panel: measurements + plot
        left_panel = QVBoxLayout()

        measurements = QGroupBox("Measurements")
        meas_layout = QGridLayout(measurements)
        self.voltage_display = ValueDisplay("Voltage", "V", 3)
        self.current_display = ValueDisplay("Current", "A", 3)
        self.power_display = ValueDisplay("Power", "W", 3)
        meas_layout.addWidget(self.voltage_display, 0, 0)
        meas_layout.addWidget(self.current_display, 0, 1)
        meas_layout.addWidget(self.power_display, 0, 2)
        left_panel.addWidget(measurements)

        self.setpoints_group = QGroupBox("Setpoints")
        setpoints_layout = QGridLayout(self.setpoints_group)
        self.voltage_sp_display = ValueDisplay("V Setpoint", "V", 2)
        self.current_sp_display = ValueDisplay("I Limit", "A", 3)
        setpoints_layout.addWidget(self.voltage_sp_display, 0, 0)
        setpoints_layout.addWidget(self.current_sp_display, 0, 1)
        left_panel.addWidget(self.setpoints_group)

        left_panel.addStretch()
        content.addLayout(left_panel, stretch=2)

        # Right panel: controls
        right_panel = QVBoxLayout()

        self.control_panel = ControlPanel(max_voltage=24.0, max_current=1.0)
        right_panel.addWidget(self.control_panel)

        self.protection_panel = ProtectionPanel()
        right_panel.addWidget(self.protection_panel)

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

        layout.addLayout(content)

    def _connect_signals(self):
        self.control_panel.voltage_changed.connect(self._on_set_voltage)
        self.control_panel.current_changed.connect(self._on_set_current)
        self.control_panel.output_toggled.connect(self._on_output_toggled)
        self.protection_panel.ovp_changed.connect(self._on_set_ovp)
        self.protection_panel.ocp_changed.connect(self._on_set_ocp)

    # --- Signal handlers that log actions and forward commands ---

    def _on_connect_clicked(self):
        if self.connect_btn.text() == "Connect":
            self.connect_requested.emit()
            self.action_logger.log("Power Supply", "Connect", "Requested connection")
        else:
            self.disconnect_requested.emit()
            self.action_logger.log("Power Supply", "Disconnect", "Requested disconnection")

    def _on_set_voltage(self, voltage: float):
        self.command_requested.emit("set_voltage", (voltage,))
        self.action_logger.log("Power Supply", "Set Voltage", f"{voltage:.2f} V")

    def _on_set_current(self, current: float):
        self.command_requested.emit("set_current", (current,))
        self.action_logger.log("Power Supply", "Set Current", f"{current:.3f} A")

    def _on_output_toggled(self, enabled: bool):
        cmd = "output_on" if enabled else "output_off"
        self.command_requested.emit(cmd, ())
        self.action_logger.log(
            "Power Supply",
            "Output On" if enabled else "Output Off",
            ""
        )

    def _on_set_ovp(self, voltage: float):
        self.command_requested.emit("set_ovp", (voltage,))
        self.action_logger.log("Power Supply", "Set OVP", f"{voltage:.1f} V")

    def _on_set_ocp(self, current: float):
        self.command_requested.emit("set_ocp", (current,))
        self.action_logger.log("Power Supply", "Set OCP", f"{current:.2f} A")

    def _on_emergency_stop(self):
        self.command_requested.emit("emergency_stop", ())
        self.action_logger.log("Power Supply", "Emergency Stop", "")

    def _toggle_view(self, advanced: bool):
        self.advanced_mode = advanced
        self.protection_panel.setVisible(advanced)
        self.action_logger.log(
            "Power Supply",
            "Toggle View",
            "Advanced" if advanced else "Simple",
        )

    # --- State update handler ---

    def update_state(self, state: PowerSupplyState):
        """Handle state update from worker."""
        if not state.connected:
            self.status_label.setText(f"Error: {state.error}")
            return

        self.status_label.setText("Connected")
        self.connect_btn.setText("Disconnect")

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

    def on_disconnected(self):
        """Reset UI on disconnect."""
        self.connect_btn.setText("Connect")
        self.status_label.setText("Disconnected")
        self.protection_panel.reset_initialized()
