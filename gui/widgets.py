"""
Reusable widgets for the hardware control GUI.
"""

from PyQt6.QtWidgets import (
    QFrame, QVBoxLayout, QGridLayout, QLabel, QPushButton,
    QGroupBox, QDoubleSpinBox,
)
from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QFont

from gui.state import PowerSupplyState


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
            self.output_btn.setStyleSheet(
                "background-color: #4CAF50; color: white; font-weight: bold;"
            )
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
        self._initialized = False

        layout = QGridLayout(self)

        # OVP
        layout.addWidget(QLabel("OVP (V):"), 0, 0)
        self.ovp_spin = QDoubleSpinBox()
        self.ovp_spin.setRange(0, 82)
        self.ovp_spin.setDecimals(1)
        layout.addWidget(self.ovp_spin, 0, 1)

        self.ovp_btn = QPushButton("Set")
        self.ovp_btn.clicked.connect(
            lambda: self.ovp_changed.emit(self.ovp_spin.value())
        )
        layout.addWidget(self.ovp_btn, 0, 2)

        # OCP
        layout.addWidget(QLabel("OCP (A):"), 1, 0)
        self.ocp_spin = QDoubleSpinBox()
        self.ocp_spin.setRange(0, 5.2)
        self.ocp_spin.setDecimals(2)
        layout.addWidget(self.ocp_spin, 1, 1)

        self.ocp_btn = QPushButton("Set")
        self.ocp_btn.clicked.connect(
            lambda: self.ocp_changed.emit(self.ocp_spin.value())
        )
        layout.addWidget(self.ocp_btn, 1, 2)

    def update_state(self, state: PowerSupplyState):
        """Update displayed protection values only on initial load or if not focused."""
        if not self._initialized:
            self.ovp_spin.setValue(state.ovp_limit)
            self.ocp_spin.setValue(state.ocp_limit)
            self._initialized = True
        else:
            if not self.ovp_spin.hasFocus():
                self.ovp_spin.setValue(state.ovp_limit)
            if not self.ocp_spin.hasFocus():
                self.ocp_spin.setValue(state.ocp_limit)

    def reset_initialized(self):
        """Reset initialization flag (call on disconnect)."""
        self._initialized = False
