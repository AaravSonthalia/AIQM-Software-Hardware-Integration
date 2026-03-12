"""
Session data logger for MBE growth monitoring.

Creates a session directory with periodic sensor CSV, commit log CSV,
and saved RHEED frames.
"""

import csv
import os
from datetime import datetime
from pathlib import Path
from typing import Optional

import numpy as np


class GrowthLogger:
    """Logs sensor data and user-annotated commits during a growth session."""

    SENSOR_FIELDS = [
        "timestamp", "elapsed_s", "pyrometer_temp_C",
        "psu_voltage_V", "psu_current_A", "psu_power_W",
    ]
    COMMIT_FIELDS = [
        "timestamp", "elapsed_s", "sample_id",
        "pyrometer_temp_C", "psu_voltage_V", "psu_current_A",
        "ai_classification", "human_classification",
        "ai_instructions", "human_instructions", "frame_path",
    ]
    AUTO_CAPTURE_FIELDS = [
        "timestamp", "elapsed_s", "change_score",
        "pyrometer_temp_C", "psu_voltage_V", "psu_current_A",
        "frame_path",
    ]

    def __init__(self, base_dir: str = "logs/growths"):
        self._base_dir = Path(base_dir)
        self._filename_prefix: str = "growth"
        self._session_dir: Optional[Path] = None
        self._sensor_file = None
        self._sensor_writer = None
        self._commit_file = None
        self._commit_writer = None
        self._commit_counter = 0
        self._auto_capture_file = None
        self._auto_capture_writer = None
        self._auto_capture_counter = 0

    @property
    def active(self) -> bool:
        return self._session_dir is not None

    def start_session(self, sample_id: str):
        """Create session directory and open CSV files."""
        tag = datetime.now().strftime("%Y%m%d_%H%M%S")
        safe_id = sample_id.strip().replace(" ", "_") or "unnamed"
        prefix = self._filename_prefix or "growth"
        self._session_dir = self._base_dir / f"{prefix}_{safe_id}_{tag}"
        self._session_dir.mkdir(parents=True, exist_ok=True)
        (self._session_dir / "frames").mkdir(exist_ok=True)

        sensor_path = self._session_dir / "sensor_log.csv"
        self._sensor_file = open(sensor_path, "w", newline="")
        self._sensor_writer = csv.DictWriter(self._sensor_file, fieldnames=self.SENSOR_FIELDS)
        self._sensor_writer.writeheader()

        commit_path = self._session_dir / "commit_log.csv"
        self._commit_file = open(commit_path, "w", newline="")
        self._commit_writer = csv.DictWriter(self._commit_file, fieldnames=self.COMMIT_FIELDS)
        self._commit_writer.writeheader()

        ac_path = self._session_dir / "auto_capture_log.csv"
        self._auto_capture_file = open(ac_path, "w", newline="")
        self._auto_capture_writer = csv.DictWriter(
            self._auto_capture_file, fieldnames=self.AUTO_CAPTURE_FIELDS,
        )
        self._auto_capture_writer.writeheader()

        self._commit_counter = 0
        self._auto_capture_counter = 0

    def log_sensors(self, pyro_temp, psu_v, psu_i, psu_p, elapsed_s):
        """Append a row to sensor_log.csv."""
        if not self._sensor_writer:
            return
        self._sensor_writer.writerow({
            "timestamp": datetime.now().isoformat(),
            "elapsed_s": f"{elapsed_s:.2f}",
            "pyrometer_temp_C": f"{pyro_temp:.1f}" if pyro_temp is not None else "",
            "psu_voltage_V": f"{psu_v:.3f}" if psu_v is not None else "",
            "psu_current_A": f"{psu_i:.3f}" if psu_i is not None else "",
            "psu_power_W": f"{psu_p:.3f}" if psu_p is not None else "",
        })
        self._sensor_file.flush()

    def log_commit(self, entry: dict):
        """Append a row to commit_log.csv."""
        if not self._commit_writer:
            return
        row = {field: entry.get(field, "") for field in self.COMMIT_FIELDS}
        self._commit_writer.writerow(row)
        self._commit_file.flush()

    def save_frame(self, frame: np.ndarray, timestamp: str = "") -> str:
        """Save frame as PNG to session frames/ subdir, return path."""
        if self._session_dir is None:
            return ""
        self._commit_counter += 1
        ts = timestamp or datetime.now().strftime("%H%M%S")
        fname = f"commit_{self._commit_counter:03d}_{ts}.png"
        path = self._session_dir / "frames" / fname

        try:
            from PIL import Image
            img = Image.fromarray(frame)
            img.save(str(path))
        except ImportError:
            # Fallback: save raw with cv2 if available, else skip
            try:
                import cv2
                cv2.imwrite(str(path), cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
            except ImportError:
                return ""

        return str(path)

    def log_auto_capture(
        self,
        frame: np.ndarray,
        score: float,
        elapsed_s: float,
        pyro_temp=None,
        psu_v=None,
        psu_i=None,
    ) -> str:
        """Save an auto-captured frame and log the event."""
        if self._session_dir is None:
            return ""

        self._auto_capture_counter += 1
        ts = datetime.now().strftime("%H%M%S")
        fname = f"auto_{self._auto_capture_counter:03d}_{ts}.png"
        path = self._session_dir / "frames" / fname

        try:
            from PIL import Image
            img = Image.fromarray(frame)
            img.save(str(path))
        except ImportError:
            try:
                import cv2
                cv2.imwrite(str(path), cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
            except ImportError:
                path = ""

        if self._auto_capture_writer:
            self._auto_capture_writer.writerow({
                "timestamp": datetime.now().isoformat(),
                "elapsed_s": f"{elapsed_s:.2f}",
                "change_score": f"{score:.4f}",
                "pyrometer_temp_C": f"{pyro_temp:.1f}" if pyro_temp is not None else "",
                "psu_voltage_V": f"{psu_v:.3f}" if psu_v is not None else "",
                "psu_current_A": f"{psu_i:.3f}" if psu_i is not None else "",
                "frame_path": str(path),
            })
            self._auto_capture_file.flush()

        return str(path)

    def end_session(self):
        """Close CSV files."""
        for f in (self._sensor_file, self._commit_file, self._auto_capture_file):
            if f and not f.closed:
                f.close()
        self._sensor_file = None
        self._sensor_writer = None
        self._commit_file = None
        self._commit_writer = None
        self._auto_capture_file = None
        self._auto_capture_writer = None
        self._session_dir = None
