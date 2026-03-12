"""
MBE Growth Monitor — application orchestrator.

Manages worker lifecycle, signal fan-out from workers to the monitor widget,
and ARM / START / STOP / DISARM session state transitions.  Optionally loads
the Classifier2 ML model for real-time AI classification.
"""
from __future__ import annotations

import logging
from pathlib import Path
from typing import Optional

import numpy as np
from PyQt6.QtWidgets import QMainWindow, QMessageBox
from PyQt6.QtCore import pyqtSlot, QTimer

log = logging.getLogger(__name__)

from owon_power_supply import find_owon_supplies
from gui.state import PowerSupplyState, CameraState, PyrometerState
from gui.workers import PowerSupplyWorker, RheedCameraWorker, PyrometerWorker
from gui.growth_monitor import GrowthMonitor
from gui.growth_logger import GrowthLogger
from gui.auto_capture import AutoCaptureEngine, ClassificationChangeDetector


class GrowthApp(QMainWindow):
    """Main window for the MBE Growth Monitor application."""

    # Default path to the AI_for_quantum repository (override via kwarg).
    _DEFAULT_AI_REPO = Path.home() / "test-claude" / "AI_for_quantum"

    def __init__(
        self,
        psu_resource: Optional[str] = None,
        ai_repo_root: Optional[str | Path] = None,
        classifier_every_n: int = 1,
    ):
        super().__init__()
        self.setWindowTitle("MBE Growth Monitor")
        self.setMinimumSize(1000, 700)

        self.psu_resource = psu_resource
        self.psu_worker: Optional[PowerSupplyWorker] = None
        self.camera_worker: Optional[RheedCameraWorker] = None
        self.pyrometer_worker: Optional[PyrometerWorker] = None
        self.growth_log = GrowthLogger()

        # --- Classifier2 bridge (disabled for v1) ----------------------------
        self._classifier = None
        self._classify_every_n = max(1, classifier_every_n)
        self._frame_counter = 0
        self._tier3_detector = ClassificationChangeDetector()
        # Skip ML loading for v1 — existing `if self._classifier` guards
        # handle None gracefully throughout the codebase.
        # ai_root = Path(ai_repo_root) if ai_repo_root else self._DEFAULT_AI_REPO
        # self._load_classifier(ai_root)

        # Auto-capture engine
        self.auto_capture = AutoCaptureEngine(parent=self)
        self.auto_capture.frame_captured.connect(self._on_auto_capture)

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
        self.monitor.psu_command_requested.connect(self._on_psu_command)

        # Auto-capture UI wiring
        self.monitor.auto_capture_checkbox.toggled.connect(self._on_auto_capture_toggled)
        self.monitor.auto_capture_threshold.valueChanged.connect(
            lambda v: setattr(self.auto_capture, 'threshold', v)
        )

    # --- Classifier2 loading -----------------------------------------------

    def _load_classifier(self, ai_root: Path) -> None:
        """Attempt to load ClassifierBridge; log and continue if unavailable."""
        model_path = ai_root / "Classifier2" / "artifacts" / "best_model.pth"
        if not model_path.exists():
            log.info("Classifier2 model not found at %s — AI classification disabled", model_path)
            return
        try:
            from gui.classifier_bridge import ClassifierBridge
            self._classifier = ClassifierBridge(ai_repo_root=ai_root)
            log.info("Classifier2 loaded successfully")
        except Exception:
            log.warning("Failed to load Classifier2 — AI classification disabled", exc_info=True)

    # --- ARM / DISARM ------------------------------------------------------

    @pyqtSlot()
    def _on_arm(self):
        """Connect all hardware workers."""
        # Read modes from config panel
        camera_mode = self.monitor.config_camera_mode.currentText()
        pyrometer_mode = self.monitor.config_pyrometer_mode.currentText()

        # Camera
        if not self.camera_worker or not self.camera_worker.isRunning():
            self.camera_worker = RheedCameraWorker(mode=camera_mode, poll_interval=1.0)
            self.camera_worker.state_updated.connect(self._on_camera_state)
            self.camera_worker.start()

        # Pyrometer
        if not self.pyrometer_worker or not self.pyrometer_worker.isRunning():
            self.pyrometer_worker = PyrometerWorker(mode=pyrometer_mode, poll_interval=0.5)
            self.pyrometer_worker.state_updated.connect(self._on_pyrometer_state)
            self.pyrometer_worker.start()

        # PSU (config resource > constructor arg > auto-detect)
        if not self.psu_worker or not self.psu_worker.isRunning():
            resource = self.monitor.config_psu_resource.text().strip() or self.psu_resource
            if not resource:
                try:
                    supplies = find_owon_supplies()
                    if supplies:
                        resource = supplies[0][0]
                except Exception:
                    pass
            if resource:
                self.psu_worker = PowerSupplyWorker(resource, poll_interval=0.5)
                self.psu_worker.state_updated.connect(self._on_psu_state)
                self.psu_worker.start()

        self.monitor.set_state("armed")
        self.statusBar().showMessage("Armed — live readings active")

    @pyqtSlot()
    def _on_disarm(self):
        """Disconnect all hardware workers."""
        self._stop_worker(self.camera_worker)
        self.camera_worker = None

        self._stop_worker(self.pyrometer_worker)
        self.pyrometer_worker = None

        self._stop_worker(self.psu_worker)
        self.psu_worker = None

        self.auto_capture.enabled = False
        self.monitor.reset_displays()
        self.monitor.set_state("idle")
        self.statusBar().showMessage("Disarmed — idle")

    # --- START / STOP ------------------------------------------------------

    @pyqtSlot()
    def _on_start(self):
        """Begin a growth session — start logging, turn PSU on."""
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

        # Reset auto-capture for new session
        self.auto_capture.reset()

        # Turn PSU output on
        if self.psu_worker:
            self.psu_worker.queue_command("output_on")

        self.monitor.set_state("running")
        self.statusBar().showMessage(f"Running — {sample_id}")

    @pyqtSlot()
    def _on_stop(self):
        """End a growth session — stop logging, turn PSU off."""
        self._sensor_log_timer.stop()
        self.auto_capture.enabled = False

        if self.psu_worker:
            self.psu_worker.queue_command("output_off")

        self.growth_log.end_session()
        self.monitor.set_state("armed")
        self.statusBar().showMessage("Stopped — still armed")

    # --- COMMIT ------------------------------------------------------------

    @pyqtSlot(dict)
    def _on_commit(self, entry: dict):
        """Save annotated snapshot to commit log."""
        # Save frame if available
        frame = self.monitor.get_current_frame()
        if frame is not None:
            ts = entry.get("timestamp", "").replace(":", "").split(".")[0][-6:]
            path = self.growth_log.save_frame(frame, ts)
            entry["frame_path"] = path

        self.growth_log.log_commit(entry)
        self.statusBar().showMessage("Observation saved", 3000)

    # --- Auto-capture ------------------------------------------------------

    def _on_auto_capture_toggled(self, checked: bool):
        self.auto_capture.enabled = checked

    @pyqtSlot(np.ndarray, float)
    def _on_auto_capture(self, frame: np.ndarray, score: float):
        """Handle an auto-capture trigger — save frame + log."""
        if not self.growth_log.active:
            return

        elapsed_s = self.monitor.get_elapsed_seconds()
        pyro_temp = (
            self.monitor._latest_pyro.temperature
            if self.monitor._latest_pyro and self.monitor._latest_pyro.connected
            else None
        )
        psu_v = psu_i = None
        if self.monitor._latest_psu and self.monitor._latest_psu.connected:
            psu_v = self.monitor._latest_psu.voltage_measured
            psu_i = self.monitor._latest_psu.current_measured

        self.growth_log.log_auto_capture(
            frame, score, elapsed_s, pyro_temp, psu_v, psu_i,
        )
        self.statusBar().showMessage(f"Auto-capture (score={score:.2f})", 3000)

    # --- PSU commands ------------------------------------------------------

    @pyqtSlot(str, tuple)
    def _on_psu_command(self, cmd: str, args: tuple):
        if self.psu_worker:
            self.psu_worker.queue_command(cmd, *args)

    # --- Periodic sensor logging -------------------------------------------

    def _log_sensors(self):
        if not self.growth_log.active:
            return
        pyro_temp = (
            self.monitor._latest_pyro.temperature
            if self.monitor._latest_pyro and self.monitor._latest_pyro.connected
            else None
        )
        psu_v = psu_i = psu_p = None
        if self.monitor._latest_psu and self.monitor._latest_psu.connected:
            psu_v = self.monitor._latest_psu.voltage_measured
            psu_i = self.monitor._latest_psu.current_measured
            psu_p = self.monitor._latest_psu.power_measured

        self.growth_log.log_sensors(
            pyro_temp, psu_v, psu_i, psu_p,
            self.monitor.get_elapsed_seconds(),
        )

    # --- Worker state fan-out to monitor -----------------------------------

    @pyqtSlot(PowerSupplyState)
    def _on_psu_state(self, state: PowerSupplyState):
        self.monitor.update_psu_state(state)

    @pyqtSlot(CameraState)
    def _on_camera_state(self, state: CameraState):
        self.monitor.update_camera_state(state)

        if state.frame is None:
            return

        # --- Classifier2 inference (every Nth frame) -----------------------
        self._frame_counter += 1
        if self._classifier and (self._frame_counter % self._classify_every_n == 0):
            try:
                result = self._classifier.classify(state.frame)
                self.monitor.update_ai_classification(result)

                # Feed scores to Tier 3 detector (for auto-capture).
                from gui.growth_monitor import RECON_LABELS
                scores_list = [
                    result["classification_scores"].get(lbl, 0.0)
                    for lbl in RECON_LABELS
                ]
                self._tier3_detector.set_scores(scores_list)
            except Exception:
                log.debug("Classifier2 inference failed", exc_info=True)

        # Feed frame to auto-capture engine
        self.auto_capture.evaluate(state.frame)
        # Update delta display
        score = self.auto_capture.latest_score
        self.monitor.auto_capture_delta_label.setText(f"delta: {score:.1%}")

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
            self.growth_log.end_session()
        self._stop_worker(self.camera_worker)
        self._stop_worker(self.pyrometer_worker)
        self._stop_worker(self.psu_worker)
        event.accept()
