"""
MBE Growth Monitor — application orchestrator.

Manages worker lifecycle, signal fan-out from workers to the monitor widget,
and ARM / START / STOP / DISARM session state transitions.
"""
from __future__ import annotations

import logging
from pathlib import Path
from typing import Optional

import numpy as np
from PyQt6.QtWidgets import QMainWindow
from PyQt6.QtCore import pyqtSlot, QTimer

log = logging.getLogger(__name__)

from gui.auto_capture import AutoCaptureEngine, PixelDiffChangeDetector
from gui.state import CameraState, EvapControlState, MistralState, PyrometerState
from gui.workers import (
    EvapControlWorker, MistralWorker, PyrometerWorker, RheedCameraWorker,
)
from gui.growth_monitor import GrowthMonitor
from gui.growth_logger import GrowthLogger


# Tuned against Rahim's 2022_02_04 STO trajectory.
#
# Rahim's data was captured at ~30 s cadence (manual). At our 1 Hz live
# cadence, a 20-frame buffer covers 20 s — much shorter than Rahim's
# 10-minute equivalent — so slow ramp drift contributes negligibly to
# the live baseline. That makes our live buffer-mean(20)@1Hz behaviorally
# closer to Rahim's *previous-frame* mode than to his *buffer-mean(20)*:
#   - previous-frame on Rahim:   baseline 0.5,  peaks 2.5-9.0 → threshold 1.5
#   - buffer-mean(5) on Rahim:   baseline 0.70, peaks 6.4-9.0 → similar
#   - buffer-mean(20) on Rahim:  baseline 1.15, peaks 4.9-8.5 (loses small
#                                events) — NOT representative of live behavior
# We expect live baseline ~0.5-1.0; smallest real transitions ~2.5+. A
# threshold of 2.0 sits ~3x above the expected baseline and below the
# smallest expected event. First lab session is the real ground truth —
# adjust after reviewing auto_capture_events.csv vs grower notes.
AUTO_CAPTURE_THRESHOLD = 2.0
AUTO_CAPTURE_BUFFER_SIZE = 20
AUTO_CAPTURE_COOLDOWN_S = 10.0


class GrowthApp(QMainWindow):
    """Main window for the OMBE Growth Monitor application."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("OMBE Growth Monitor")
        self.setMinimumSize(1000, 700)

        self.camera_worker: Optional[RheedCameraWorker] = None
        self.pyrometer_worker: Optional[PyrometerWorker] = None
        self.mistral_worker: Optional[MistralWorker] = None
        self.evap_worker: Optional[EvapControlWorker] = None
        self.growth_log = GrowthLogger()

        # Periodic sensor logging timer (1 second interval while running)
        self._sensor_log_timer = QTimer(self)
        self._sensor_log_timer.setInterval(1000)
        self._sensor_log_timer.timeout.connect(self._log_sensors)

        # Auto-capture engine — shadow-mode pixel-diff change detection.
        # Engine is disarmed at construction; armed in _on_start, disarmed
        # in _on_stop. Frame ingestion happens in _on_camera_state.
        self.auto_capture_engine = AutoCaptureEngine(
            threshold=AUTO_CAPTURE_THRESHOLD,
            cooldown_s=AUTO_CAPTURE_COOLDOWN_S,
            warmup_frames=AUTO_CAPTURE_BUFFER_SIZE,
        )
        self.auto_capture_engine.set_detector(
            PixelDiffChangeDetector(buffer_size=AUTO_CAPTURE_BUFFER_SIZE),
        )
        self.auto_capture_engine.frame_captured.connect(
            self._on_auto_capture_event,
        )
        self._auto_capture_event_count = 0

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
        """Connect camera, pyrometer, MISTRAL, and Evap Control workers."""
        camera_mode = self.monitor.config_camera_mode.currentText()
        pyrometer_mode = self.monitor.config_pyrometer_mode.currentText()
        mistral_mode = self.monitor.config_mistral_mode.currentText()
        evap_mode = self.monitor.config_evap_mode.currentText()

        if not self.camera_worker or not self.camera_worker.isRunning():
            self.camera_worker = RheedCameraWorker(
                mode=camera_mode, poll_interval=1.0,
            )
            self.camera_worker.state_updated.connect(self._on_camera_state)
            self.camera_worker.start()

        if not self.pyrometer_worker or not self.pyrometer_worker.isRunning():
            self.pyrometer_worker = PyrometerWorker(
                mode=pyrometer_mode, poll_interval=0.5,
            )
            self.pyrometer_worker.state_updated.connect(self._on_pyrometer_state)
            self.pyrometer_worker.start()

        if not self.mistral_worker or not self.mistral_worker.isRunning():
            self.mistral_worker = MistralWorker(
                mode=mistral_mode, poll_interval=1.0,
            )
            self.mistral_worker.state_updated.connect(self._on_mistral_state)
            self.mistral_worker.start()

        if not self.evap_worker or not self.evap_worker.isRunning():
            self.evap_worker = EvapControlWorker(
                mode=evap_mode, poll_interval=1.0,
            )
            self.evap_worker.state_updated.connect(self._on_evap_state)
            self.evap_worker.start()

        self.monitor.set_state("armed")
        self.statusBar().showMessage("Armed \u2014 live readings active")

    @pyqtSlot()
    def _on_disarm(self):
        """Disconnect all hardware workers."""
        self._stop_worker(self.camera_worker)
        self.camera_worker = None

        self._stop_worker(self.pyrometer_worker)
        self.pyrometer_worker = None

        self._stop_worker(self.mistral_worker)
        self.mistral_worker = None

        self._stop_worker(self.evap_worker)
        self.evap_worker = None

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

        # Arm shadow-mode auto-capture for this session
        self.auto_capture_engine.reset()
        self.auto_capture_engine.enabled = True
        self._auto_capture_event_count = 0
        self.monitor.set_auto_capture_status(
            "Auto-capture: armed (warmup)"
        )

        self.monitor.set_state("running")
        self.statusBar().showMessage(f"Running \u2014 {sample_id}")

    @pyqtSlot()
    def _on_stop(self):
        """End a growth session — save metadata, auto-export, stop logging."""
        self._sensor_log_timer.stop()

        # Disarm auto-capture; engine state cleaned up so the next session
        # starts fresh in _on_start.
        self.auto_capture_engine.enabled = False
        self.monitor.set_auto_capture_status(
            f"Auto-capture: idle "
            f"({self._auto_capture_event_count} events this session)"
        )

        metadata = self.monitor.get_session_metadata()

        # Save session metadata
        self.growth_log.save_session_metadata(metadata)

        # Auto-export growth log before closing session files
        path = self.growth_log.export_growth_log(metadata)

        self.growth_log.end_session()
        self.monitor.set_state("armed")

        if path:
            self.statusBar().showMessage(
                f"Session saved \u2014 growth log: {path}", 8000,
            )
        else:
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
        m = self.monitor._latest_mistral
        e = self.monitor._latest_evap
        mistral_ok = m is not None and m.connected
        evap_ok = e is not None and e.connected
        self.growth_log.log_sensors(
            pyro_temp,
            self.monitor.get_elapsed_seconds(),
            v_set=m.v_set if mistral_ok else None,
            v_actual=m.v_actual if mistral_ok else None,
            i_set=m.i_set if mistral_ok else None,
            i_actual=m.i_actual if mistral_ok else None,
            chamber_pressure_mbar=(
                e.chamber_pressure_mbar if evap_ok else None
            ),
        )

        from datetime import datetime
        self.monitor.add_sensor_log_row(
            datetime.now().strftime("%H:%M:%S"),
            pyro_temp,
            voltage=m.v_actual if mistral_ok else None,
            current=m.i_actual if mistral_ok else None,
            pressure=e.chamber_pressure_mbar if evap_ok else None,
        )

    # --- Worker state fan-out to monitor -----------------------------------

    @pyqtSlot(CameraState)
    def _on_camera_state(self, state: CameraState):
        self.monitor.update_camera_state(state)

        # Feed the auto-capture engine. Engine internally guards on `enabled`,
        # so this is a no-op outside an active session.
        if state.frame is not None and state.connected:
            self.auto_capture_engine.evaluate(state.frame)
            if self.auto_capture_engine.enabled:
                self.monitor.set_auto_capture_status(
                    f"Auto-capture: armed | "
                    f"score: {self.auto_capture_engine.latest_score:.2f} | "
                    f"events: {self._auto_capture_event_count}"
                )

    @pyqtSlot(np.ndarray, float)
    def _on_auto_capture_event(self, frame: np.ndarray, score: float):
        """Engine flagged a frame — log to auto_capture_events.csv.

        Shadow mode: we only record the event metadata, not the frame
        bytes. Cross-reference this CSV against grower notes + sensor_log
        after the session to validate detector behavior.
        """
        self._auto_capture_event_count += 1
        pyro_temp = (
            self.monitor._latest_pyro.temperature
            if self.monitor._latest_pyro and self.monitor._latest_pyro.connected
            else None
        )
        self.growth_log.log_auto_capture_event(
            event_idx=self._auto_capture_event_count,
            score=score,
            elapsed_s=self.monitor.get_elapsed_seconds(),
            pyro_temp=pyro_temp,
        )

    @pyqtSlot(PyrometerState)
    def _on_pyrometer_state(self, state: PyrometerState):
        self.monitor.update_pyrometer_state(state)

    @pyqtSlot(MistralState)
    def _on_mistral_state(self, state: MistralState):
        self.monitor.update_mistral_state(state)

    @pyqtSlot(EvapControlState)
    def _on_evap_state(self, state: EvapControlState):
        self.monitor.update_evap_state(state)

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
        self._stop_worker(self.mistral_worker)
        self._stop_worker(self.evap_worker)
        event.accept()
