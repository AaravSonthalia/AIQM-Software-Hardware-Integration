"""
MBE Growth Monitor — application orchestrator.

Manages worker lifecycle, signal fan-out from workers to the monitor widget,
and ARM / START / STOP / DISARM session state transitions.
"""
from __future__ import annotations

import logging
from pathlib import Path
from typing import Optional

from PyQt6.QtWidgets import QMainWindow
from PyQt6.QtCore import pyqtSlot, QTimer

log = logging.getLogger(__name__)

from gui.state import CameraState, PyrometerState
from gui.workers import RheedCameraWorker, PyrometerWorker
from gui.growth_monitor import GrowthMonitor
from gui.growth_logger import GrowthLogger


class GrowthApp(QMainWindow):
    """Main window for the OMBE Growth Monitor application."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("OMBE Growth Monitor")
        self.setMinimumSize(1000, 700)

        self.camera_worker: Optional[RheedCameraWorker] = None
        self.pyrometer_worker: Optional[PyrometerWorker] = None
        self.growth_log = GrowthLogger()

        # Periodic sensor logging timer (1 second interval while running)
        self._sensor_log_timer = QTimer(self)
        self._sensor_log_timer.setInterval(1000)
        self._sensor_log_timer.timeout.connect(self._log_sensors)

        # Central widget
        self.monitor = GrowthMonitor()
        self.setCentralWidget(self.monitor)

        # Connect monitor signals → app handlers
        self.monitor.arm_requested.connect(self._on_arm)
        self.monitor.disarm_requested.connect(self._on_disarm)
        self.monitor.start_requested.connect(self._on_start)
        self.monitor.stop_requested.connect(self._on_stop)
        self.monitor.commit_requested.connect(self._on_commit)
        self.monitor.export_requested.connect(self._on_export)

    # --- ARM / DISARM ------------------------------------------------------

    @pyqtSlot()
    def _on_arm(self):
        """Connect camera and pyrometer workers."""
        camera_mode = self.monitor.config_camera_mode.currentText()
        pyrometer_mode = self.monitor.config_pyrometer_mode.currentText()

        # Camera
        if not self.camera_worker or not self.camera_worker.isRunning():
            self.camera_worker = RheedCameraWorker(
                mode=camera_mode, poll_interval=1.0,
            )
            self.camera_worker.state_updated.connect(self._on_camera_state)
            self.camera_worker.start()

        # Pyrometer
        if not self.pyrometer_worker or not self.pyrometer_worker.isRunning():
            self.pyrometer_worker = PyrometerWorker(
                mode=pyrometer_mode, poll_interval=0.5,
            )
            self.pyrometer_worker.state_updated.connect(self._on_pyrometer_state)
            self.pyrometer_worker.start()

        self.monitor.set_state("armed")
        self.statusBar().showMessage("Armed \u2014 live readings active")

    @pyqtSlot()
    def _on_disarm(self):
        """Disconnect all hardware workers."""
        self._stop_worker(self.camera_worker)
        self.camera_worker = None

        self._stop_worker(self.pyrometer_worker)
        self.pyrometer_worker = None

        self.monitor.reset_displays()
        self.monitor.set_state("idle")
        self.statusBar().showMessage("Disarmed \u2014 idle")

    # --- START / STOP ------------------------------------------------------

    @pyqtSlot()
    def _on_start(self):
        """Begin a growth session — start logging."""
        # Apply config panel settings
        save_path = self.monitor.config_save_path.text().strip()
        prefix = self.monitor.config_prefix.text().strip()
        if save_path:
            self.growth_log._base_dir = Path(save_path)
        if prefix:
            self.growth_log._filename_prefix = prefix

        sample_id = self.monitor.sample_id_input.text().strip() or "unnamed"
        self.growth_log.start_session(sample_id)

        interval_ms = int(self.monitor.config_interval_spin.value() * 1000)
        self._sensor_log_timer.setInterval(interval_ms)
        self._sensor_log_timer.start()

        self.monitor.set_state("running")
        self.statusBar().showMessage(f"Running \u2014 {sample_id}")

    @pyqtSlot()
    def _on_stop(self):
        """End a growth session — stop logging and save metadata."""
        self._sensor_log_timer.stop()

        # Save session metadata
        self.growth_log.save_session_metadata(
            self.monitor.get_session_metadata(),
        )

        self.growth_log.end_session()
        self.monitor.set_state("armed")
        self.statusBar().showMessage("Stopped \u2014 session saved")

    # --- LOG ENTRY ---------------------------------------------------------

    @pyqtSlot(dict)
    def _on_commit(self, entry: dict):
        """Save a timestamped log entry with optional RHEED frame."""
        # Save frame if available
        frame = self.monitor.get_current_frame()
        if frame is not None:
            ts = entry.get("timestamp", "").replace(":", "").split(".")[0][-6:]
            path = self.growth_log.save_frame(frame, ts)
            entry["frame_path"] = path

        self.growth_log.log_commit(entry)
        self.statusBar().showMessage("Entry logged", 3000)

    # --- Export ------------------------------------------------------------

    @pyqtSlot()
    def _on_export(self):
        """Export the current session as a growth log."""
        metadata = self.monitor.get_session_metadata()
        path = self.growth_log.export_growth_log(metadata)
        if path:
            self.statusBar().showMessage(f"Growth log exported: {path}", 5000)
        else:
            self.statusBar().showMessage("Export failed \u2014 no session data", 5000)

    # --- Periodic sensor logging -------------------------------------------

    def _log_sensors(self):
        if not self.growth_log.active:
            return
        pyro_temp = (
            self.monitor._latest_pyro.temperature
            if self.monitor._latest_pyro and self.monitor._latest_pyro.connected
            else None
        )
        self.growth_log.log_sensors(
            pyro_temp, self.monitor.get_elapsed_seconds(),
        )

        # Update sensor log table in UI
        from datetime import datetime
        self.monitor.add_sensor_log_row(
            datetime.now().strftime("%H:%M:%S"),
            pyro_temp,
        )

    # --- Worker state fan-out to monitor -----------------------------------

    @pyqtSlot(CameraState)
    def _on_camera_state(self, state: CameraState):
        self.monitor.update_camera_state(state)

    @pyqtSlot(PyrometerState)
    def _on_pyrometer_state(self, state: PyrometerState):
        self.monitor.update_pyrometer_state(state)

    # --- Helpers -----------------------------------------------------------

    @staticmethod
    def _stop_worker(worker):
        if worker is not None:
            worker.stop()
            worker.wait(5000)

    # --- Shutdown ----------------------------------------------------------

    def closeEvent(self, event):
        self._sensor_log_timer.stop()
        if self.growth_log.active:
            self.growth_log.save_session_metadata(
                self.monitor.get_session_metadata(),
            )
            self.growth_log.end_session()
        self._stop_worker(self.camera_worker)
        self._stop_worker(self.pyrometer_worker)
        event.accept()
