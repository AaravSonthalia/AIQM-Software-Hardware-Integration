"""Floating pyrometer temperature vs time window.

Receives PyrometerState updates via on_pyrometer_state(); growers open it
with the 'Temp Plot' button in the Monitor tab. Can be dragged to a second
monitor and kept open independently of the main GUI.
"""
from __future__ import annotations

import time
from collections import deque
from typing import Optional

from PyQt6.QtWidgets import QMainWindow, QPushButton, QVBoxLayout, QWidget

import pyqtgraph as pg

from gui.state import PyrometerState


class PyrometerWindow(QMainWindow):
    _MAXLEN = 7200  # 60 min at 2 Hz

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Pyrometer — Temperature vs Time")
        self.setMinimumSize(600, 300)
        self._t0: Optional[float] = None
        self._times: deque[float] = deque(maxlen=self._MAXLEN)
        self._temps: deque[float] = deque(maxlen=self._MAXLEN)

        plot = pg.PlotWidget()
        plot.setBackground("#0f1117")
        plot.setLabel("left", "Temperature", units="°C")
        plot.setLabel("bottom", "Elapsed", units="min")
        plot.getPlotItem().showGrid(x=True, y=True, alpha=0.3)
        self._curve = plot.plot(pen=pg.mkPen("#f97316", width=2.0))

        clear_btn = QPushButton("Clear")
        clear_btn.setFixedWidth(80)
        clear_btn.clicked.connect(self.reset)

        central = QWidget()
        layout = QVBoxLayout(central)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(4)
        layout.addWidget(plot, 1)
        layout.addWidget(clear_btn)
        self.setCentralWidget(central)

    def on_pyrometer_state(self, state: PyrometerState) -> None:
        if not state.connected or not state.valid:
            return
        now = time.monotonic()
        if self._t0 is None:
            self._t0 = now
        self._times.append((now - self._t0) / 60.0)
        self._temps.append(state.temperature)
        self._curve.setData(list(self._times), list(self._temps))

    def reset(self) -> None:
        self._t0 = None
        self._times.clear()
        self._temps.clear()
        self._curve.setData([], [])
