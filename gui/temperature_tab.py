"""
Temperature monitoring tab widget.
"""

import time
from collections import deque

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QGroupBox,
)
from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QFont

import pyqtgraph as pg

from gui.state import TemperatureState
from gui.action_logger import ActionLogger


class TemperatureTab(QWidget):
    """Thermocouple monitoring tab using DracalTMC100k."""

    connect_requested = pyqtSignal()
    disconnect_requested = pyqtSignal()

    def __init__(self, action_logger: ActionLogger, parent=None):
        super().__init__(parent)
        self.action_logger = action_logger

        # Live plot buffers
        self.temp_start = time.time()
        self.temp_time = deque(maxlen=240)
        self.temp_tc = deque(maxlen=240)
        self.temp_cj = deque(maxlen=240)

        self._build_ui()

    def _build_ui(self):
        layout = QVBoxLayout(self)

        # Top bar
        top_bar = QHBoxLayout()
        self.connect_btn = QPushButton("Connect Thermocouple")
        self.connect_btn.clicked.connect(self._on_connect_clicked)
        top_bar.addWidget(self.connect_btn)

        self.status_label = QLabel("Disconnected")
        top_bar.addWidget(self.status_label)
        top_bar.addStretch()
        layout.addLayout(top_bar)

        # Temperature display
        temp_group = QGroupBox("Live Temperature")
        temp_layout = QVBoxLayout(temp_group)

        self.temp_value_label = QLabel("--.- C")
        self.temp_value_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.temp_value_label.setFont(QFont("Monospace", 36, QFont.Weight.Bold))
        temp_layout.addWidget(self.temp_value_label)

        # Cold junction display
        self.cj_label = QLabel("Cold Junction: --.- C")
        self.cj_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.cj_label.setFont(QFont("Monospace", 16))
        temp_layout.addWidget(self.cj_label)

        self.channel_label = QLabel("Channel: --")
        self.channel_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        temp_layout.addWidget(self.channel_label)

        self.device_label = QLabel("Device: --")
        self.device_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        temp_layout.addWidget(self.device_label)

        layout.addWidget(temp_group)

        # Temperature trend plot
        trend_group = QGroupBox("Temperature Trend")
        trend_layout = QVBoxLayout(trend_group)
        trend_layout.setContentsMargins(4, 4, 4, 4)

        self.temp_plot = pg.PlotWidget()
        self.temp_plot.setLabel("left", "Temperature", "C")
        self.temp_plot.setLabel("bottom", "Time", "s")
        self.temp_plot.addLegend()
        self.temp_plot.showGrid(x=True, y=True)
        self.tc_curve = self.temp_plot.plot(pen=pg.mkPen("r", width=2), name="Thermocouple")
        self.cj_curve = self.temp_plot.plot(pen=pg.mkPen("b", width=2), name="Cold Junction")

        trend_layout.addWidget(self.temp_plot)
        layout.addWidget(trend_group)

        layout.addStretch()

    def _on_connect_clicked(self):
        if self.connect_btn.text() == "Connect Thermocouple":
            self.connect_requested.emit()
            self.action_logger.log("Temperature", "Connect", "Requested connection")
        else:
            self.disconnect_requested.emit()
            self.action_logger.log("Temperature", "Disconnect", "Requested disconnection")

    def update_state(self, state: TemperatureState):
        """Handle temperature state update from worker."""
        if not state.connected and state.error:
            self.status_label.setText(f"Error: {state.error}")
            return

        if not state.connected:
            return

        self.status_label.setText("Connected")
        self.connect_btn.setText("Disconnect Thermocouple")

        self.temp_value_label.setText(f"{state.temperature:.2f} {state.unit}")
        self.cj_label.setText(f"Cold Junction: {state.cold_junction:.2f} {state.unit}")
        self.channel_label.setText(f"Channel: {state.channel}")
        self.device_label.setText(f"Device: {state.device_info}")

        # Update live plot
        now = time.time() - self.temp_start
        self.temp_time.append(now)
        self.temp_tc.append(state.temperature)
        self.temp_cj.append(state.cold_junction)

        t = list(self.temp_time)
        self.tc_curve.setData(t, list(self.temp_tc))
        self.cj_curve.setData(t, list(self.temp_cj))

    def on_disconnected(self):
        """Reset UI on disconnect."""
        self.connect_btn.setText("Connect Thermocouple")
        self.status_label.setText("Disconnected")
