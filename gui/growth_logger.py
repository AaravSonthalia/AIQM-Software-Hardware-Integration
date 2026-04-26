"""
Session data logger for MBE growth monitoring.

Creates a session directory with periodic sensor CSV, commit log CSV,
saved RHEED frames, session metadata JSON, and growth log export.
"""

import csv
import json
from datetime import datetime
from pathlib import Path
from typing import Optional

import numpy as np


class GrowthLogger:
    """Logs sensor data and timestamped entries during a growth session."""

    SENSOR_FIELDS = [
        "timestamp", "elapsed_s", "pyrometer_temp_C",
        "mistral_v_set_V", "mistral_v_actual_V",
        "mistral_i_set_A", "mistral_i_actual_A",
        "chamber_pressure_mbar",
    ]
    COMMIT_FIELDS = [
        "timestamp", "time_display", "elapsed_s", "sample_id", "grower",
        "pyrometer_temp_C", "voltage_V", "current_A",
        "recon_1x1", "recon_Twinned (2x1)", "recon_c(6x2)",
        "recon_rt13xrt13", "recon_HTR",
        "note", "frame_path",
    ]
    AUTO_CAPTURE_FIELDS = [
        "timestamp", "elapsed_s", "event_idx",
        "change_score", "pyrometer_temp_C",
    ]
    HEARTBEAT_FIELDS = [
        "timestamp", "elapsed_s", "heartbeat_idx",
        "pyrometer_temp_C", "frame_path",
    ]

    def __init__(self, base_dir: str = "logs/growths"):
        self._base_dir = Path(base_dir)
        self._filename_prefix: str = "growth"
        self._session_dir: Optional[Path] = None
        self._sensor_file = None
        self._sensor_writer = None
        self._commit_file = None
        self._commit_writer = None
        self._auto_capture_file = None
        self._auto_capture_writer = None
        self._heartbeat_file = None
        self._heartbeat_writer = None
        self._commit_counter = 0
        self._heartbeat_counter = 0
        self._entries: list[dict] = []  # Accumulated entries for export

    @property
    def active(self) -> bool:
        """True while CSV files are open and accepting writes."""
        return self._sensor_writer is not None

    @property
    def session_dir(self) -> Optional[Path]:
        return self._session_dir

    def start_session(self, sample_id: str):
        """Create session directory and open CSV files."""
        # Clear any previous session data
        self._session_dir = None
        self._entries = []
        tag = datetime.now().strftime("%Y%m%d_%H%M%S")
        safe_id = sample_id.strip().replace(" ", "_") or "unnamed"
        prefix = self._filename_prefix or "growth"
        self._session_dir = self._base_dir / f"{prefix}_{safe_id}_{tag}"
        self._session_dir.mkdir(parents=True, exist_ok=True)
        (self._session_dir / "frames").mkdir(exist_ok=True)

        sensor_path = self._session_dir / "sensor_log.csv"
        self._sensor_file = open(sensor_path, "w", newline="")
        self._sensor_writer = csv.DictWriter(
            self._sensor_file, fieldnames=self.SENSOR_FIELDS,
        )
        self._sensor_writer.writeheader()

        commit_path = self._session_dir / "commit_log.csv"
        self._commit_file = open(commit_path, "w", newline="")
        self._commit_writer = csv.DictWriter(
            self._commit_file, fieldnames=self.COMMIT_FIELDS,
        )
        self._commit_writer.writeheader()

        auto_capture_path = self._session_dir / "auto_capture_events.csv"
        self._auto_capture_file = open(auto_capture_path, "w", newline="")
        self._auto_capture_writer = csv.DictWriter(
            self._auto_capture_file, fieldnames=self.AUTO_CAPTURE_FIELDS,
        )
        self._auto_capture_writer.writeheader()

        heartbeat_path = self._session_dir / "heartbeat_log.csv"
        self._heartbeat_file = open(heartbeat_path, "w", newline="")
        self._heartbeat_writer = csv.DictWriter(
            self._heartbeat_file, fieldnames=self.HEARTBEAT_FIELDS,
        )
        self._heartbeat_writer.writeheader()

        self._commit_counter = 0
        self._heartbeat_counter = 0
        self._entries = []

    def log_sensors(
        self, pyro_temp, elapsed_s,
        v_set=None, v_actual=None, i_set=None, i_actual=None,
        chamber_pressure_mbar=None,
    ):
        """Append a row to sensor_log.csv. All values may be None."""
        if not self._sensor_writer:
            return

        def _f(val, places):
            return f"{val:.{places}f}" if val is not None else ""

        def _sci(val):
            return f"{val:.3e}" if val is not None else ""

        self._sensor_writer.writerow({
            "timestamp": datetime.now().isoformat(),
            "elapsed_s": f"{elapsed_s:.2f}",
            "pyrometer_temp_C": _f(pyro_temp, 1),
            "mistral_v_set_V":    _f(v_set, 3),
            "mistral_v_actual_V": _f(v_actual, 3),
            "mistral_i_set_A":    _f(i_set, 3),
            "mistral_i_actual_A": _f(i_actual, 3),
            "chamber_pressure_mbar": _sci(chamber_pressure_mbar),
        })
        self._sensor_file.flush()

    def log_commit(self, entry: dict):
        """Append a row to commit_log.csv and accumulate for export."""
        if not self._commit_writer:
            return
        row = {field: entry.get(field, "") for field in self.COMMIT_FIELDS}
        self._commit_writer.writerow(row)
        self._commit_file.flush()
        self._entries.append(entry)

    def save_heartbeat_frame(
        self, frame: np.ndarray, timestamp: str = ""
    ) -> str:
        """Save a heartbeat anchor frame as ``heartbeat_NNN_HHMMSS.png``.

        Same quality gate as ``save_frame``; failed frames return "" and
        are not counted toward the heartbeat counter.
        """
        if self._session_dir is None:
            return ""

        try:
            from drivers.frame_quality import check_frame_quality
            qa = check_frame_quality(frame)
            if not qa.passed:
                import sys
                print(
                    f"[GrowthLogger] heartbeat frame rejected: {qa.reason}",
                    file=sys.stderr,
                    flush=True,
                )
                return ""
        except ImportError:
            pass

        self._heartbeat_counter += 1
        ts = timestamp or datetime.now().strftime("%H%M%S")
        fname = f"heartbeat_{self._heartbeat_counter:03d}_{ts}.png"
        path = self._session_dir / "frames" / fname

        try:
            from PIL import Image
            Image.fromarray(frame).save(str(path))
        except ImportError:
            try:
                import cv2
                cv2.imwrite(str(path), cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
            except ImportError:
                return ""

        return str(path)

    def log_heartbeat(
        self,
        elapsed_s: float,
        pyro_temp: Optional[float] = None,
        frame_path: str = "",
    ):
        """Append a row to heartbeat_log.csv. Pairs with save_heartbeat_frame."""
        if not self._heartbeat_writer:
            return
        self._heartbeat_writer.writerow({
            "timestamp": datetime.now().isoformat(),
            "elapsed_s": f"{elapsed_s:.2f}",
            "heartbeat_idx": self._heartbeat_counter,
            "pyrometer_temp_C": (
                f"{pyro_temp:.1f}" if pyro_temp is not None else ""
            ),
            "frame_path": frame_path,
        })
        self._heartbeat_file.flush()

    def log_auto_capture_event(
        self,
        event_idx: int,
        score: float,
        elapsed_s: float,
        pyro_temp: Optional[float] = None,
    ):
        """Append a row to auto_capture_events.csv for shadow-mode logging.

        Called by GrowthApp when AutoCaptureEngine emits frame_captured.
        Writes timestamp + change score + temp at trigger time, so the
        flagged moments can be cross-referenced against grower notes
        and pyrometer trajectory after the session.
        """
        if not self._auto_capture_writer:
            return
        self._auto_capture_writer.writerow({
            "timestamp": datetime.now().isoformat(),
            "elapsed_s": f"{elapsed_s:.2f}",
            "event_idx": event_idx,
            "change_score": f"{score:.4f}",
            "pyrometer_temp_C": (
                f"{pyro_temp:.1f}" if pyro_temp is not None else ""
            ),
        })
        self._auto_capture_file.flush()

    def save_frame(self, frame: np.ndarray, timestamp: str = "") -> str:
        """Save frame as PNG to session frames/ subdir, return path.

        Runs the frame quality gate first — black/saturated/uniform/
        undersized frames are rejected with a stderr warning and an
        empty path returned. Callers should treat "" as "no frame saved"
        (current convention).
        """
        if self._session_dir is None:
            return ""

        try:
            from drivers.frame_quality import check_frame_quality
            qa = check_frame_quality(frame)
            if not qa.passed:
                import sys
                print(
                    f"[GrowthLogger] frame rejected by quality gate: {qa.reason}",
                    file=sys.stderr,
                    flush=True,
                )
                return ""
        except ImportError:
            pass  # Quality gate optional; degrade to old always-save behavior.

        self._commit_counter += 1
        ts = timestamp or datetime.now().strftime("%H%M%S")
        fname = f"entry_{self._commit_counter:03d}_{ts}.png"
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
                return ""

        return str(path)

    def save_session_metadata(self, metadata: dict):
        """Save session metadata to a JSON file."""
        if self._session_dir is None:
            return
        meta = {
            **metadata,
            "session_end": datetime.now().isoformat(),
            "total_entries": len(self._entries),
        }
        meta_path = self._session_dir / "session_metadata.json"
        with open(meta_path, "w") as f:
            json.dump(meta, f, indent=2)

    def export_growth_log(self, metadata: dict) -> str:
        """Export session as OMBE growth log. Returns file path or empty string."""
        if self._session_dir is None or not self._entries:
            return ""

        try:
            return self._export_xlsx(metadata)
        except ImportError:
            return self._export_csv_log(metadata)

    def _export_xlsx(self, metadata: dict) -> str:
        """Export as xlsx matching OMBE growth log template."""
        from openpyxl import Workbook
        from openpyxl.styles import Font

        wb = Workbook()
        ws = wb.active
        ws.title = "Growth Log"

        bold = Font(bold=True)
        bold_large = Font(bold=True, size=12)

        # --- Header section ---
        ws['B2'] = 'Date:'
        ws['B2'].font = bold
        ws['D2'] = metadata.get('date', '')
        ws['G2'] = 'Substrate:'
        ws['G2'].font = bold

        ws['B3'] = 'Grower:'
        ws['B3'].font = bold
        ws['D3'] = metadata.get('grower', '')
        ws['G3'] = 'Sample ID:'
        ws['G3'].font = bold
        ws['I3'] = metadata.get('sample_id', '')

        ws['B4'] = 'Base pressure (mbar):'
        ws['B4'].font = bold
        ws['G4'] = 'Growth pressure (mbar):'
        ws['G4'].font = bold

        # --- Source parameter headers (OMBE elements) ---
        elements = ['Substrate', 'Sr', 'Ti', 'Y', 'Er', 'Eu', 'O', 'Al', 'Ta']
        for i, elem in enumerate(elements):
            cell = ws.cell(row=5, column=3 + i, value=elem)
            cell.font = bold
        ws.cell(row=5, column=12, value='Flux ratio').font = bold

        ws['B6'] = 'Temperature (\u2103)'
        ws['B6'].font = bold
        ws['B7'] = 'Flux (mbar)'
        ws['B7'].font = bold
        ws['B8'] = 'Time (min)'
        ws['B8'].font = bold

        # --- Growth notes section ---
        ws['B9'] = 'Growth Notes'
        ws['B9'].font = bold_large

        ws['B10'] = 'Time (hh:mm)'
        ws['B10'].font = bold
        ws['C10'] = 'Temp (\u2103)'
        ws['C10'].font = bold
        ws['D10'] = 'Operation'
        ws['D10'].font = bold

        # Pre/post annealing headers
        ws['C11'] = 'Pre-growth annealing T (\u2103)'
        ws['F11'] = 'Pre-growth annealing time (min)'
        ws['H11'] = 'Post-growth annealing T (\u2103)'
        ws['K11'] = 'Post-growth annealing time (min)'

        # --- Operations log entries ---
        for i, entry in enumerate(self._entries):
            row = 13 + i
            ws.cell(row=row, column=2, value=entry.get('time_display', ''))
            temp = entry.get('pyrometer_temp_C', '')
            if temp:
                try:
                    ws.cell(row=row, column=3, value=float(temp))
                except ValueError:
                    ws.cell(row=row, column=3, value=temp)
            ws.cell(row=row, column=4, value=entry.get('note', ''))

        # --- Column widths ---
        ws.column_dimensions['B'].width = 18
        ws.column_dimensions['C'].width = 14
        ws.column_dimensions['D'].width = 60
        ws.column_dimensions['G'].width = 22
        ws.column_dimensions['I'].width = 20

        export_path = self._session_dir / "growth_log.xlsx"
        wb.save(str(export_path))
        return str(export_path)

    def _export_csv_log(self, metadata: dict) -> str:
        """Fallback CSV export if openpyxl is not available."""
        if self._session_dir is None:
            return ""
        export_path = self._session_dir / "growth_log_export.csv"
        with open(export_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["OMBE Growth Log"])
            writer.writerow(["Date", metadata.get('date', '')])
            writer.writerow(["Grower", metadata.get('grower', '')])
            writer.writerow(["Sample ID", metadata.get('sample_id', '')])
            writer.writerow([])
            writer.writerow([
                "Time", "Temp (\u2103)", "Voltage (V)", "Current (A)", "Note",
            ])
            for entry in self._entries:
                writer.writerow([
                    entry.get("time_display", ""),
                    entry.get("pyrometer_temp_C", ""),
                    entry.get("voltage_V", ""),
                    entry.get("current_A", ""),
                    entry.get("note", ""),
                ])
        return str(export_path)

    def end_session(self):
        """Close CSV files. Preserves session_dir and entries for post-stop export."""
        for f in (
            self._sensor_file, self._commit_file,
            self._auto_capture_file, self._heartbeat_file,
        ):
            if f and not f.closed:
                f.close()
        self._sensor_file = None
        self._sensor_writer = None
        self._commit_file = None
        self._commit_writer = None
        self._auto_capture_file = None
        self._auto_capture_writer = None
        self._heartbeat_file = None
        self._heartbeat_writer = None
        # NOTE: _session_dir and _entries intentionally preserved
        # so Export Growth Log works after STOP.
