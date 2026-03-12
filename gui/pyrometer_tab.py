"""
Pyrometer monitoring tab — live temperature display and trend plot.

Separate from the thermocouple (temperature_tab.py) since these are
different physical instruments measuring different things:
  - Thermocouple: substrate heater temperature (contact)
  - Pyrometer: substrate surface temperature (optical, non-contact)
"""

import time
from collections import deque

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QGroupBox,
    QComboBox,
)
from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QFont

import pyqtgraph as pg

from gui.state import PyrometerState
from gui.action_logger import ActionLogger


class PyrometerTab(QWidget):
    """Pyrometer monitoring tab for BASF Exactus via IFD-5 or TemperaSure scrape."""

    connect_requested = pyqtSignal(str)  # mode: "modbus", "screengrab", "dummy"
    disconnect_requested = pyqtSignal()

    def __init__(self, action_logger: ActionLogger, parent=None):
        super().__init__(parent)
        self.action_logger = action_logger

        # Trend plot buffers
        self._plot_start = time.time()
        self._plot_times = deque(maxlen=600)
        self._plot_temps = deque(maxlen=600)

        self._build_ui()

    def _build_ui(self):
        layout = QVBoxLayout(self)

        # ─── Top bar ───
        top_bar = QHBoxLayout()

        self.mode_combo = QComboBox()
        self.mode_combo.addItems([
            "Modbus (Direct RS-422)",
            "Screen Grab (TemperaSure)",
            "Dummy (Test)",
        ])
        top_bar.addWidget(QLabel("Mode:"))
        top_bar.addWidget(self.mode_combo)

        self.connect_btn = QPushButton("Connect Pyrometer")
        self.connect_btn.clicked.connect(self._on_connect_clicked)
        top_bar.addWidget(self.connect_btn)

        self.status_label = QLabel("Disconnected")
        top_bar.addWidget(self.status_label)
        top_bar.addStretch()

        layout.addLayout(top_bar)

        # ─── Live temperature display ───
        temp_group = QGroupBox("Pyrometer Temperature")
        temp_layout = QVBoxLayout(temp_group)

        self.temp_value_label = QLabel("--.- C")
        self.temp_value_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.temp_value_label.setFont(QFont("Monospace", 36, QFont.Weight.Bold))
        self.temp_value_label.setStyleSheet("color: #FF6600;")  # orange for pyrometer
        temp_layout.addWidget(self.temp_value_label)

        self.emissivity_label = QLabel("Emissivity: --")
        self.emissivity_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.emissivity_label.setFont(QFont("Monospace", 14))
        temp_layout.addWidget(self.emissivity_label)

        self.mode_label = QLabel("Mode: --")
        self.mode_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        temp_layout.addWidget(self.mode_label)

        self.device_label = QLabel("Device: --")
        self.device_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        temp_layout.addWidget(self.device_label)

        layout.addWidget(temp_group)

        # ─── Temperature trend plot ───
        trend_group = QGroupBox("Pyrometer Trend")
        trend_layout = QVBoxLayout(trend_group)
        trend_layout.setContentsMargins(4, 4, 4, 4)

        self.temp_plot = pg.PlotWidget()
        self.temp_plot.setLabel("left", "Temperature", "C")
        self.temp_plot.setLabel("bottom", "Time", "s")
        self.temp_plot.showGrid(x=True, y=True)
        self.temp_curve = self.temp_plot.plot(
            pen=pg.mkPen("#FF6600", width=2), name="Pyrometer"
        )

        trend_layout.addWidget(self.temp_plot)
        layout.addWidget(trend_group)

        layout.addStretch()

    def _mode_string(self) -> str:
        idx = self.mode_combo.currentIndex()
        return ["modbus", "screengrab", "dummy"][idx]

    def _on_connect_clicked(self):
        if self.connect_btn.text() == "Connect Pyrometer":
            mode = self._mode_string()
            self.connect_requested.emit(mode)
            self.action_logger.log("Pyrometer", "Connect", f"Mode: {mode}")
        else:
            self.disconnect_requested.emit()
            self.action_logger.log("Pyrometer", "Disconnect", "Requested disconnection")

    def update_state(self, state: PyrometerState):
        """Handle pyrometer state update from worker."""
        if not state.connected and state.error:
            self.status_label.setText(f"Error: {state.error}")
            return

        if not state.connected:
            return

        self.status_label.setText("Connected")
        self.connect_btn.setText("Disconnect Pyrometer")
        self.mode_combo.setEnabled(False)

        self.temp_value_label.setText(f"{state.temperature:.1f} {state.unit}")
        if state.emissivity is not None:
            self.emissivity_label.setText(f"Emissivity: {state.emissivity:.3f}")
        else:
            self.emissivity_label.setText("Emissivity: --")
        self.mode_label.setText(f"Mode: {state.mode}")
        self.device_label.setText(f"Device: {state.device_info}")

        # Update trend plot
        now = time.time() - self._plot_start
        self._plot_times.append(now)
        self._plot_temps.append(state.temperature)

        t = list(self._plot_times)
        self.temp_curve.setData(t, list(self._plot_temps))

    def on_disconnected(self):
        """Reset UI on disconnect."""
        self.connect_btn.setText("Connect Pyrometer")
        self.mode_combo.setEnabled(True)
        self.status_label.setText("Disconnected")
