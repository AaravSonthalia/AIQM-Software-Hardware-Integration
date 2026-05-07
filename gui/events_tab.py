"""
Events tab — master/detail review surface for auto-capture events.

Locked design in `events_tab_design.md` (May 7 2026). Three jobs in one
surface: real-time review during a growth, walk-away catch-up, and
labeling input for Classifier2 training data. Catch-up is the primary
use case — most cognitively expensive — and the master/detail layout is
optimized around it.

v1 skeleton:
  - QSplitter master/detail layout
  - Master list populated from `auto_capture_events.csv` on session
    attach, then live-updated via `AutoCaptureEngine.frame_captured`
  - Detail pane is a placeholder

Image viewer, two-dropdown labeling UI, unreviewed badge, and the
default-hide-discarded filter are follow-up commits per the design's
implementation breakdown.
"""
from __future__ import annotations

import csv
from datetime import datetime
from pathlib import Path
from typing import Optional

import numpy as np
from PyQt6.QtCore import Qt, pyqtSlot
from PyQt6.QtWidgets import (
    QAbstractItemView, QHeaderView, QLabel, QSplitter,
    QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget,
)


# Master-list column indices. Score-color cell coloring and a label-state
# column come in follow-up commits along with the labeling UI.
COL_EVENT_IDX = 0
COL_TIME = 1
COL_SCORE = 2
COL_TEMP = 3
COL_STATE = 4
COLUMN_HEADERS = ["#", "Time", "Score", "Temp (℃)", "State"]


class EventsTab(QWidget):
    """Master/detail review surface for auto-capture events.

    The master list is the source of truth for which events exist and
    their CSV-recorded state. It is populated two ways:

    1. ``attach_session(session_dir)`` — called by GrowthApp on session
       start to load any pre-existing rows (covers the GUI-restart-mid-
       session case). Clears the table first.
    2. ``on_frame_captured(frame, score)`` — slot for
       ``AutoCaptureEngine.frame_captured``. The engine signal carries no
       event_idx, so we re-read the CSV and append rows past the
       watermark. Connection order matters: GrowthApp's handler must be
       wired first so the CSV row is on disk before this slot reads it.
    """

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._session_dir: Optional[Path] = None
        # Highest event_idx already in the table — guards repeat appends
        # when the CSV is re-read on every frame_captured.
        self._last_seen_event_idx: int = 0
        self._build_ui()

    # --- UI ---------------------------------------------------------------

    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.setChildrenCollapsible(False)

        splitter.addWidget(self._build_master_pane())
        splitter.addWidget(self._build_detail_pane())

        # 2:3 master:detail. The detail pane will host the image viewer
        # next; growers want pixel real estate for 656x492 RHEED frames.
        splitter.setStretchFactor(0, 2)
        splitter.setStretchFactor(1, 3)
        splitter.setSizes([400, 600])

        layout.addWidget(splitter)

    def _build_master_pane(self) -> QWidget:
        master = QWidget()
        master_layout = QVBoxLayout(master)
        master_layout.setContentsMargins(0, 0, 0, 0)
        master_layout.setSpacing(4)

        title = QLabel("Auto-capture Events")
        title.setStyleSheet("font-weight: bold; font-size: 13px;")
        master_layout.addWidget(title)

        self.events_table = QTableWidget(0, len(COLUMN_HEADERS))
        self.events_table.setHorizontalHeaderLabels(COLUMN_HEADERS)
        header = self.events_table.horizontalHeader()
        header.setSectionResizeMode(
            COL_EVENT_IDX, QHeaderView.ResizeMode.ResizeToContents,
        )
        header.setSectionResizeMode(
            COL_TIME, QHeaderView.ResizeMode.ResizeToContents,
        )
        header.setSectionResizeMode(
            COL_SCORE, QHeaderView.ResizeMode.ResizeToContents,
        )
        header.setSectionResizeMode(
            COL_TEMP, QHeaderView.ResizeMode.ResizeToContents,
        )
        header.setSectionResizeMode(
            COL_STATE, QHeaderView.ResizeMode.Stretch,
        )
        self.events_table.setEditTriggers(
            QAbstractItemView.EditTrigger.NoEditTriggers,
        )
        self.events_table.setSelectionBehavior(
            QAbstractItemView.SelectionBehavior.SelectRows,
        )
        self.events_table.verticalHeader().setVisible(False)
        master_layout.addWidget(self.events_table)

        return master

    def _build_detail_pane(self) -> QWidget:
        detail = QWidget()
        detail_layout = QVBoxLayout(detail)
        detail_layout.setContentsMargins(8, 8, 8, 8)

        self._detail_placeholder = QLabel(
            "Select an event to view buffer frames.\n\n"
            "Image viewer and labeling controls coming in follow-up commits."
        )
        self._detail_placeholder.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._detail_placeholder.setWordWrap(True)
        self._detail_placeholder.setStyleSheet(
            "color: #888; font-style: italic;"
        )
        detail_layout.addWidget(self._detail_placeholder)

        return detail

    # --- Public API used by GrowthApp -------------------------------------

    def attach_session(self, session_dir: Optional[Path]) -> None:
        """Point the tab at a session's CSV and reload the master list.

        Call from GrowthApp._on_start after GrowthLogger.start_session.
        Clearing + reloading on attach handles GUI-restart-mid-session and
        keeps the tab consistent with whatever's on disk.
        """
        self._session_dir = (
            Path(session_dir) if session_dir is not None else None
        )
        self._last_seen_event_idx = 0
        self.events_table.setRowCount(0)
        self._load_csv_rows()

    @pyqtSlot(np.ndarray, float)
    def on_frame_captured(self, frame: np.ndarray, score: float) -> None:  # noqa: ARG002
        """Slot for ``AutoCaptureEngine.frame_captured``.

        The engine emits ``(frame, score)`` only, so we re-read the CSV to
        recover ``event_idx``, ``timestamp``, and ``pyrometer_temp_C`` that
        GrowthApp's earlier-connected handler just wrote. Args are kept
        for signal compatibility but ignored — the CSV row is authoritative.
        """
        self._load_csv_rows()

    # --- Internal ---------------------------------------------------------

    def _load_csv_rows(self) -> None:
        """Append rows past the watermark from auto_capture_events.csv."""
        if self._session_dir is None:
            return
        csv_path = self._session_dir / "auto_capture_events.csv"
        if not csv_path.exists():
            return
        try:
            with open(csv_path, "r", newline="") as f:
                reader = csv.DictReader(f)
                for row in reader:
                    try:
                        idx = int(row.get("event_idx", "") or 0)
                    except (TypeError, ValueError):
                        continue
                    if idx <= self._last_seen_event_idx:
                        continue
                    self._add_event_row(row)
                    self._last_seen_event_idx = idx
        except OSError:
            return

    def _add_event_row(self, row: dict) -> None:
        """Insert one CSV row at the top of the master list (newest first)."""
        timestamp = row.get("timestamp", "")
        try:
            time_str = datetime.fromisoformat(timestamp).strftime("%H:%M:%S")
        except (TypeError, ValueError):
            time_str = timestamp[-8:] if timestamp else "---"

        score_raw = row.get("change_score", "")
        try:
            score_str = f"{float(score_raw):.2f}"
        except (TypeError, ValueError):
            score_str = score_raw or "---"

        temp_raw = row.get("pyrometer_temp_C", "")
        try:
            temp_str = (
                f"{float(temp_raw):.1f}" if temp_raw not in ("", None) else "---"
            )
        except (TypeError, ValueError):
            temp_str = "---"

        state = row.get("event_state", "") or "---"

        # Insert at top so the most-recent event surfaces first; iterating
        # the CSV oldest→newest with insertRow(0) leaves the table in
        # newest-at-top order.
        self.events_table.insertRow(0)
        self.events_table.setItem(
            0, COL_EVENT_IDX,
            QTableWidgetItem(str(row.get("event_idx", ""))),
        )
        self.events_table.setItem(0, COL_TIME, QTableWidgetItem(time_str))
        self.events_table.setItem(0, COL_SCORE, QTableWidgetItem(score_str))
        self.events_table.setItem(0, COL_TEMP, QTableWidgetItem(temp_str))
        self.events_table.setItem(0, COL_STATE, QTableWidgetItem(state))
