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


# Auto-capture event states (used by event_state column in
# auto_capture_events.csv). Set to PENDING at fire time; transitions to one
# of the other three when the grower interacts with the AutoCaptureBanner or
# the keep-default countdown fires. AUTO_SKIPPED is set at log time when
# the quality gate rejected all 20 buffer frames — there's nothing to
# review, so the event has no banner and no resolution path. Marking it
# at log time gives the row a terminal state instead of leaving it as
# pending forever.
EVENT_STATE_PENDING = "pending"
EVENT_STATE_KEPT_EXPLICIT = "kept_explicit"
EVENT_STATE_KEPT_DEFAULT = "kept_default"
EVENT_STATE_DISCARDED = "discarded"
EVENT_STATE_AUTO_SKIPPED = "auto_skipped"


class GrowthLogger:
    """Logs sensor data and timestamped entries during a growth session."""

    SENSOR_FIELDS = [
        "timestamp", "elapsed_s",
        "pyrometer_temp_C", "pyrometer_temp_std_C", "pyrometer_temp_n",
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
        "buffer_count", "buffer_dir",
        "event_state", "state_changed_at",
    ]
    HEARTBEAT_FIELDS = [
        "timestamp", "elapsed_s", "heartbeat_idx",
        "pyrometer_temp_C", "frame_path",
    ]
    SET_CHANGE_FIELDS = [
        "timestamp", "elapsed_s", "event_idx",
        "channel", "old_value", "new_value", "delta",
        "pyrometer_temp_C",
    ]
    # Event labels written by the Events tab labeling form. The from/to
    # columns are reserved for the deferred reconstruction-transition
    # dropdowns — they exist now so future UI additions don't require a
    # CSV schema migration. Each row is per-event_idx (upsert-by-key).
    EVENT_LABEL_FIELDS = [
        "event_idx",
        "primary_reconstruction",
        "change_from",
        "change_to",
        "notes",
        "label_timestamp_iso",
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
        self._set_change_file = None
        self._set_change_writer = None
        self._commit_counter = 0
        self._heartbeat_counter = 0
        self._set_change_counter = 0
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

        set_change_path = self._session_dir / "set_change_events.csv"
        self._set_change_file = open(set_change_path, "w", newline="")
        self._set_change_writer = csv.DictWriter(
            self._set_change_file, fieldnames=self.SET_CHANGE_FIELDS,
        )
        self._set_change_writer.writeheader()

        self._commit_counter = 0
        self._heartbeat_counter = 0
        self._set_change_counter = 0
        self._entries = []

    def log_sensors(
        self, pyro_temp, elapsed_s,
        v_set=None, v_actual=None, i_set=None, i_actual=None,
        chamber_pressure_mbar=None,
        pyro_temp_std=None, pyro_temp_n=None,
    ):
        """Append a row to sensor_log.csv. All values may be None.

        ``pyro_temp_std`` and ``pyro_temp_n`` capture the per-poll
        statistical spread when the pyrometer worker takes multiple
        sub-readings per cycle. Empty strings if not provided.
        """
        if not self._sensor_writer:
            return

        def _f(val, places):
            return f"{val:.{places}f}" if val is not None else ""

        def _sci(val):
            return f"{val:.3e}" if val is not None else ""

        self._sensor_writer.writerow({
            "timestamp": datetime.now().isoformat(),
            "elapsed_s": f"{elapsed_s:.2f}",
            "pyrometer_temp_C":     _f(pyro_temp, 1),
            "pyrometer_temp_std_C": _f(pyro_temp_std, 2),
            "pyrometer_temp_n":     pyro_temp_n if pyro_temp_n is not None else "",
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

    def log_set_change_event(
        self,
        elapsed_s: float,
        channel: str,
        old_value: float,
        new_value: float,
        pyro_temp: Optional[float] = None,
    ):
        """Append a row to set_change_events.csv.

        Pairs operator setpoint changes (Set Voltage / Set Current button
        presses on the MISTRAL GUI) with timestamps, so the model can
        learn from when V/I are deliberately adjusted vs drifting.
        """
        if not self._set_change_writer:
            return
        self._set_change_counter += 1
        self._set_change_writer.writerow({
            "timestamp": datetime.now().isoformat(),
            "elapsed_s": f"{elapsed_s:.2f}",
            "event_idx": self._set_change_counter,
            "channel": channel,
            "old_value": f"{old_value:.4f}",
            "new_value": f"{new_value:.4f}",
            "delta": f"{new_value - old_value:+.4f}",
            "pyrometer_temp_C": (
                f"{pyro_temp:.1f}" if pyro_temp is not None else ""
            ),
        })
        self._set_change_file.flush()

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
        buffer_count: int = 0,
        buffer_dir: str = "",
        event_state: str = EVENT_STATE_PENDING,
    ):
        """Append a row to auto_capture_events.csv for shadow-mode logging.

        Called by GrowthApp when AutoCaptureEngine emits frame_captured.
        Writes timestamp + change score + temp at trigger time, so the
        flagged moments can be cross-referenced against grower notes
        and pyrometer trajectory after the session. ``buffer_count`` and
        ``buffer_dir`` capture how many context frames were dumped and
        where, so post-hoc analysis can locate them.

        ``event_state`` defaults to ``"pending"`` at fire time and is updated
        to one of ``"kept_explicit"`` / ``"kept_default"`` / ``"discarded"``
        when the grower interacts with the AutoCaptureBanner (see
        ``update_auto_capture_state``).
        """
        if not self._auto_capture_writer:
            return
        timestamp = datetime.now().isoformat()
        # Non-pending initial states (auto_skipped, etc.) are terminal at
        # log time — set state_changed_at so the CSV stays internally
        # consistent with the rule that any non-pending row has a stamp.
        state_changed_at = (
            timestamp if event_state != EVENT_STATE_PENDING else ""
        )
        self._auto_capture_writer.writerow({
            "timestamp": timestamp,
            "elapsed_s": f"{elapsed_s:.2f}",
            "event_idx": event_idx,
            "change_score": f"{score:.4f}",
            "pyrometer_temp_C": (
                f"{pyro_temp:.1f}" if pyro_temp is not None else ""
            ),
            "buffer_count": buffer_count,
            "buffer_dir": buffer_dir,
            "event_state": event_state,
            "state_changed_at": state_changed_at,
        })
        self._auto_capture_file.flush()

    def update_auto_capture_state(
        self,
        event_idx: int,
        new_state: str,
    ) -> bool:
        """Update event_state and state_changed_at for an existing event row.

        Called when the grower interacts with the AutoCaptureBanner (Keep
        Now, Discard) or when the keep-default countdown fires. Rewrites
        auto_capture_events.csv in place — small file, infrequent updates,
        simple semantics. Returns True if the row was found and updated.

        Non-destructive design: discard updates the row state but does NOT
        remove the buffer directory. The grower can recover discarded
        events from the Events tab if they change their mind. See
        ``feedback_aiqm_grower_friction.md`` for the design rationale.
        """
        if self._session_dir is None:
            return False
        csv_path = self._session_dir / "auto_capture_events.csv"
        if not csv_path.exists():
            return False

        # Close the writer's file handle so we can rewrite in-place.
        if self._auto_capture_file and not self._auto_capture_file.closed:
            self._auto_capture_file.close()

        with open(csv_path, "r", newline="") as f:
            reader = csv.DictReader(f)
            rows = list(reader)

        timestamp = datetime.now().isoformat()
        found = False
        for row in rows:
            if str(row.get("event_idx", "")) == str(event_idx):
                row["event_state"] = new_state
                row["state_changed_at"] = timestamp
                found = True
                break

        if found:
            with open(csv_path, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=self.AUTO_CAPTURE_FIELDS)
                writer.writeheader()
                writer.writerows(rows)

        # Reopen for append so subsequent log_auto_capture_event calls work.
        # No header write — the file already has the header from start_session
        # (or the rewrite above when found=True).
        self._auto_capture_file = open(csv_path, "a", newline="")
        self._auto_capture_writer = csv.DictWriter(
            self._auto_capture_file, fieldnames=self.AUTO_CAPTURE_FIELDS,
        )
        return found

    def read_event_labels(self) -> dict[int, dict]:
        """Load all rows from events_labels.csv keyed by event_idx.

        Returns an empty dict if no session is active, the file doesn't
        exist yet (no labels applied), or the file can't be read. The
        Events tab uses this once per session attach to seed an in-memory
        cache; the cache is then kept in sync as the grower applies new
        labels through update_event_label.
        """
        if self._session_dir is None:
            return {}
        csv_path = self._session_dir / "events_labels.csv"
        if not csv_path.exists():
            return {}
        labels: dict[int, dict] = {}
        try:
            with open(csv_path, "r", newline="") as f:
                reader = csv.DictReader(f)
                for row in reader:
                    try:
                        idx = int(row.get("event_idx", "") or 0)
                    except (TypeError, ValueError):
                        continue
                    labels[idx] = dict(row)
        except OSError:
            return {}
        return labels

    def update_event_label(
        self,
        event_idx: int,
        primary_reconstruction: Optional[str] = None,
        change_from: Optional[str] = None,
        change_to: Optional[str] = None,
        notes: Optional[str] = None,
    ) -> bool:
        """Atomically upsert a labeling row in events_labels.csv.

        Reads the existing file (if any), updates or appends the row for
        ``event_idx``, and rewrites the whole file. Each call is a
        per-change atomic write — the design accepts ~3-5 rewrites per
        labeled event (one per dropdown / notes commit) in exchange for
        no "save button" cognitive load on the grower.

        Only fields explicitly passed are updated; ``None`` means "leave
        existing value alone." This lets the EventsTab call with just
        ``primary_reconstruction=...`` when the dropdown changes without
        clobbering a previously-typed notes string. ``label_timestamp_iso``
        is always refreshed to record when the label was last touched.

        File is created on first write — sessions with no labeling
        activity won't leave behind an empty events_labels.csv.

        Returns True on successful write, False on no-session or I/O error.
        """
        if self._session_dir is None:
            return False
        csv_path = self._session_dir / "events_labels.csv"

        rows: list[dict] = []
        if csv_path.exists():
            try:
                with open(csv_path, "r", newline="") as f:
                    reader = csv.DictReader(f)
                    rows = [dict(r) for r in reader]
            except OSError:
                return False

        target = str(event_idx)
        existing = next(
            (r for r in rows if str(r.get("event_idx", "")) == target),
            None,
        )
        if existing is None:
            existing = {f: "" for f in self.EVENT_LABEL_FIELDS}
            existing["event_idx"] = target
            rows.append(existing)

        if primary_reconstruction is not None:
            existing["primary_reconstruction"] = primary_reconstruction
        if change_from is not None:
            existing["change_from"] = change_from
        if change_to is not None:
            existing["change_to"] = change_to
        if notes is not None:
            existing["notes"] = notes
        existing["label_timestamp_iso"] = datetime.now().isoformat()

        try:
            with open(csv_path, "w", newline="") as f:
                writer = csv.DictWriter(
                    f, fieldnames=self.EVENT_LABEL_FIELDS,
                )
                writer.writeheader()
                writer.writerows(rows)
        except OSError:
            return False
        return True

    def save_auto_capture_buffer(
        self,
        event_idx: int,
        frames: list[np.ndarray],
    ) -> tuple[int, str]:
        """Save the auto-capture context buffer for a flagged event.

        Each frame is written to a per-event subdirectory under frames/ so
        that sessions with many events stay browsable. The quality gate is
        applied per-frame; rejected frames are silently skipped (they don't
        carry information worth keeping).

        Returns ``(saved_count, relative_dir)``. ``relative_dir`` is empty
        if no session is active.
        """
        if self._session_dir is None or not frames:
            return 0, ""

        try:
            from drivers.frame_quality import check_frame_quality
        except ImportError:
            check_frame_quality = None

        event_dir = self._session_dir / "frames" / f"auto_event_{event_idx:03d}"
        event_dir.mkdir(parents=True, exist_ok=True)

        ts_tag = datetime.now().strftime("%H%M%S")
        saved = 0
        for pos, frame in enumerate(frames):
            if check_frame_quality is not None:
                qa = check_frame_quality(frame)
                if not qa.passed:
                    continue
            fname = f"buf_{pos:02d}_{ts_tag}.png"
            path = event_dir / fname
            try:
                from PIL import Image
                Image.fromarray(frame).save(str(path))
                saved += 1
            except ImportError:
                try:
                    import cv2
                    cv2.imwrite(str(path), cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
                    saved += 1
                except ImportError:
                    break

        rel_dir = str(event_dir.relative_to(self._session_dir))
        return saved, rel_dir

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
            self._set_change_file,
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
        self._set_change_file = None
        self._set_change_writer = None
        # NOTE: _session_dir and _entries intentionally preserved
        # so Export Growth Log works after STOP.
