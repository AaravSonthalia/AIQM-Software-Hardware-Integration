"""
Events tab — master/detail review surface for auto-capture events.

Locked design in `events_tab_design.md` (May 7 2026). Three jobs in one
surface: real-time review during a growth, walk-away catch-up, and
labeling input for Classifier2 training data. Catch-up is the primary
use case — most cognitively expensive — and the master/detail layout is
optimized around it.

Current scope:
  - QSplitter master/detail layout
  - Master list populated from `auto_capture_events.csv` on session
    attach, then live-updated via `AutoCaptureEngine.frame_captured`
  - Detail pane image viewer with metadata header, slider-scrubable
    buffer frames, and a placeholder for empty / missing buffer cases
  - Live row-state updates from `auto_capture_decision`
  - Unreviewed-count badge surfaced via `unreviewed_count_changed`
  - Per-event labeling form: primary reconstruction dropdown + notes
    field, atomic-per-change writes through GrowthLogger, in-memory
    label cache loaded on session attach

Reconstruction-transition labeling (change_from / change_to) and the
default-hide-discarded filter are deferred. The CSV schema reserves
the change_from / change_to columns from day one so the deferred UI
lands without migration.
"""
from __future__ import annotations

import csv
from datetime import datetime
from pathlib import Path
from typing import Optional

import numpy as np
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, pyqtSlot
from PyQt6.QtGui import QPixmap
from PyQt6.QtWidgets import (
    QAbstractItemView, QComboBox, QFormLayout, QGroupBox, QHBoxLayout,
    QHeaderView, QLabel, QLineEdit, QSizePolicy, QSlider, QSplitter,
    QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget,
)

from gui.growth_logger import (
    EVENT_STATE_AUTO_SKIPPED,
    EVENT_STATE_DISCARDED,
    EVENT_STATE_KEPT_EXPLICIT,
    GrowthLogger,
)


# Five canonical RHEED reconstruction names + meta-categories for events
# the grower can't or won't tag with one. Spellings match the Monitor tab's
# recon sliders verbatim — see feedback_recon_naming.md for the canonical
# list and the "use ASCII x, lowercase rt13" rule.
RECON_LABEL_OPTIONS = [
    "1x1",
    "Twinned (2x1)",
    "c(6x2)",
    "rt13xrt13",
    "HTR",
    "unknown",
    "artifact",
]
# Sentinel data value for "no label assigned" — distinguishable from any
# valid reconstruction name in the dropdown's itemData.
RECON_UNLABELED = ""


# An event is "unreviewed" if the grower hasn't made an explicit
# Keep/Discard decision on it. kept_default (banner timed out) counts as
# unreviewed because the timeout means the grower wasn't actually looking
# — exactly the walk-away catch-up case the badge needs to surface.
# auto_skipped events have no buffer to review (quality gate rejected
# all 20 frames), so they don't need grower attention either.
_REVIEWED_STATES = (
    EVENT_STATE_KEPT_EXPLICIT,
    EVENT_STATE_DISCARDED,
    EVENT_STATE_AUTO_SKIPPED,
)


class _ScalingImageLabel(QLabel):
    """QLabel that keeps an aspect-ratio-scaled view of a source pixmap.

    The QSplitter resizes the *inner* detail pane when its handle is
    dragged, not the outer EventsTab — so the right place to react is
    the label's own resizeEvent. Storing the original pixmap means each
    resize re-scales from full resolution rather than compounding lossy
    rescales on already-shrunk pixels.
    """

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._original: Optional[QPixmap] = None
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setStyleSheet(
            "background-color: #000; border: 1px solid #555;"
        )
        self.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding,
        )
        self.setMinimumSize(320, 240)

    def setOriginalPixmap(self, pixmap: Optional[QPixmap]) -> None:
        if pixmap is None or pixmap.isNull():
            self._original = None
            self.clear()
            return
        self._original = pixmap
        self._rescale()

    def clearImage(self) -> None:
        self._original = None
        self.clear()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._rescale()

    def _rescale(self) -> None:
        if self._original is None:
            return
        scaled = self._original.scaled(
            self.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        self.setPixmap(scaled)


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

    Live state changes flow in via ``on_decision_made`` (slot for
    ``GrowthMonitor.auto_capture_decision``). Each transition refreshes
    the unreviewed-count badge through ``unreviewed_count_changed``,
    which GrowthMonitor relays to the tab header so the catch-up case
    (return after walk-away, see "Events (12)") works at a glance.
    """

    # Emitted whenever the count of unreviewed events changes. GrowthMonitor
    # listens and rewrites the tab header text. "Unreviewed" is currently
    # state-based (pending or kept_default) — when labeling lands it will
    # also include events without a primary_reconstruction tag.
    unreviewed_count_changed = pyqtSignal(int)

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._session_dir: Optional[Path] = None
        # GrowthLogger reference for label CSV reads/writes. None when no
        # session is attached, set in attach_session.
        self._growth_logger: Optional[GrowthLogger] = None
        # Highest event_idx already in the table — guards repeat appends
        # when the CSV is re-read on every frame_captured.
        self._last_seen_event_idx: int = 0
        # Pre-decoded pixmaps for the currently-selected event's buffer.
        # Cleared on selection change and on attach_session.
        self._cached_pixmaps: list[QPixmap] = []
        self._cached_paths: list[Path] = []
        # In-memory label cache — single source of truth for "what label
        # does each event have right now." Loaded from events_labels.csv
        # on session attach, kept in sync as the grower applies labels.
        # Single-writer model (only this widget touches the file), so
        # staleness can't happen.
        self._labels_cache: dict[int, dict] = {}
        # Tracks which event the labeling form is currently bound to, so
        # form-change handlers know which event_idx to write under.
        self._currently_displayed_event_idx: Optional[int] = None
        # Debounce notes-text writes — saving on every keystroke would
        # both spam disk and persist garbage like "substrate fla". Save
        # 500ms after the user stops typing, or immediately on selection
        # change (flushed in _on_selection_changed).
        self._notes_save_timer = QTimer(self)
        self._notes_save_timer.setSingleShot(True)
        self._notes_save_timer.setInterval(500)
        self._notes_save_timer.timeout.connect(self._flush_notes_to_disk)
        self._build_ui()
        self.events_table.itemSelectionChanged.connect(
            self._on_selection_changed,
        )

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
        """Detail pane: placeholder when nothing is selected, otherwise
        an image viewer with metadata header, scrubable slider, and
        position counter for the buffer frames of the selected event.

        Placeholder + content are siblings in the same QVBoxLayout, both
        with stretch=1; show/hide swaps which one fills the pane.
        """
        detail = QWidget()
        layout = QVBoxLayout(detail)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        self._detail_placeholder = QLabel()
        self._detail_placeholder.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._detail_placeholder.setWordWrap(True)
        self._detail_placeholder.setStyleSheet(
            "color: #888; font-style: italic;"
        )
        layout.addWidget(self._detail_placeholder, 1)

        self._detail_content = QWidget()
        content = QVBoxLayout(self._detail_content)
        content.setContentsMargins(0, 0, 0, 0)
        content.setSpacing(6)

        self._metadata_label = QLabel()
        self._metadata_label.setStyleSheet(
            "font-weight: bold; font-size: 13px; padding: 2px 4px;"
        )
        self._metadata_label.setWordWrap(True)
        content.addWidget(self._metadata_label)

        self._image_label = _ScalingImageLabel()
        content.addWidget(self._image_label, 1)

        slider_row = QHBoxLayout()
        slider_row.setSpacing(8)
        self._slider = QSlider(Qt.Orientation.Horizontal)
        self._slider.setRange(0, 0)
        self._slider.setStyleSheet(
            "QSlider::groove:horizontal { background: #333; height: 6px; }"
            "QSlider::handle:horizontal { background: #0d9488; width: 14px; "
            "margin: -5px 0; border-radius: 7px; }"
            "QSlider::sub-page:horizontal { background: #0d9488; }"
            "QSlider::add-page:horizontal { background: #555; }"
        )
        self._slider.valueChanged.connect(self._display_frame_at)
        slider_row.addWidget(self._slider, 1)

        self._frame_position_label = QLabel("0 / 0")
        self._frame_position_label.setStyleSheet(
            "color: #aaa; font-size: 11px;"
        )
        self._frame_position_label.setMinimumWidth(60)
        self._frame_position_label.setAlignment(
            Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
        )
        slider_row.addWidget(self._frame_position_label, 0)
        content.addLayout(slider_row)

        content.addWidget(self._build_label_form())

        self._detail_content.hide()
        layout.addWidget(self._detail_content, 1)

        # Default placeholder copy — replaced by _set_placeholder() with
        # context-specific text when an event is selected but its buffer
        # is missing or empty.
        self._set_placeholder("Select an event to view buffer frames.")

        return detail

    def _build_label_form(self) -> QGroupBox:
        """Labeling form — primary reconstruction dropdown + notes field.

        The change_from / change_to dropdowns are deferred to a follow-up
        commit per AJ's scoping decision. The CSV schema reserves their
        columns from day one so the future UI lands without migration.

        Uses ``activated`` (not ``currentIndexChanged``) on the dropdown
        so programmatic ``setCurrentIndex`` during selection-change
        loading doesn't trigger spurious writes. Notes use a debounced
        timer so per-keystroke writes don't spam disk or persist
        garbage like "substrate fla".
        """
        box = QGroupBox("Label")
        form = QFormLayout(box)
        form.setContentsMargins(10, 16, 10, 8)
        form.setSpacing(6)

        self._primary_recon_combo = QComboBox()
        self._primary_recon_combo.addItem("(unlabeled)", RECON_UNLABELED)
        for name in RECON_LABEL_OPTIONS:
            self._primary_recon_combo.addItem(name, name)
        self._primary_recon_combo.activated.connect(
            self._on_primary_recon_activated,
        )
        form.addRow("Primary reconstruction:", self._primary_recon_combo)

        self._notes_input = QLineEdit()
        self._notes_input.setPlaceholderText("Optional notes…")
        self._notes_input.textChanged.connect(self._on_notes_text_changed)
        self._notes_input.editingFinished.connect(self._flush_notes_to_disk)
        form.addRow("Notes:", self._notes_input)

        return box

    # --- Public API used by GrowthApp -------------------------------------

    def attach_session(self, growth_logger: Optional[GrowthLogger]) -> None:
        """Point the tab at the active session's logger and reload the list.

        Call from GrowthApp._on_start after GrowthLogger.start_session.
        Clearing + reloading on attach handles GUI-restart-mid-session and
        keeps the tab consistent with whatever's on disk. Also resets the
        detail pane back to its placeholder so a stale selection from the
        previous session can't render against the new session_dir, and
        repopulates the label cache from events_labels.csv (if any).

        Takes the GrowthLogger rather than a bare session_dir because the
        Events tab now owns label CSV writes, which need the logger's
        update_event_label method.
        """
        self._growth_logger = growth_logger
        self._session_dir = (
            growth_logger.session_dir
            if growth_logger is not None and growth_logger.session_dir is not None
            else None
        )
        self._last_seen_event_idx = 0
        self.events_table.setRowCount(0)
        self._cached_pixmaps = []
        self._cached_paths = []
        self._image_label.clearImage()
        self._labels_cache = (
            growth_logger.read_event_labels()
            if growth_logger is not None else {}
        )
        self._currently_displayed_event_idx = None
        self._notes_save_timer.stop()
        self._set_placeholder("Select an event to view buffer frames.")
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

    @pyqtSlot(int, str, str)
    def on_decision_made(
        self, event_idx: int, buffer_dir: str, state: str,  # noqa: ARG002
    ) -> None:
        """Slot for ``GrowthMonitor.auto_capture_decision``.

        Updates the matching row's state cell + cached UserRole dict and
        — if that row is the currently-selected one — re-renders the
        detail pane's metadata header. Refreshes the unreviewed-count
        badge afterward. ``buffer_dir`` is part of the signal payload
        for GrowthApp's CSV writer; the tab doesn't need it.

        Connection order: GrowthApp's _on_auto_capture_decision is wired
        first and rewrites auto_capture_events.csv via
        update_auto_capture_state. By the time this slot runs, the CSV
        row already reflects the new state — but we don't need to re-read
        it because ``state`` is in the signal payload.
        """
        target = str(event_idx)
        for row_idx in range(self.events_table.rowCount()):
            idx_item = self.events_table.item(row_idx, COL_EVENT_IDX)
            if idx_item is None or idx_item.text() != target:
                continue

            state_item = self.events_table.item(row_idx, COL_STATE)
            if state_item is not None:
                state_item.setText(state)
            else:
                self.events_table.setItem(
                    row_idx, COL_STATE, QTableWidgetItem(state),
                )

            row_data = idx_item.data(Qt.ItemDataRole.UserRole)
            if isinstance(row_data, dict):
                row_data["event_state"] = state
                idx_item.setData(Qt.ItemDataRole.UserRole, row_data)
                if self.events_table.currentRow() == row_idx:
                    self._metadata_label.setText(
                        self._format_metadata(row_data)
                    )
            break

        self._refresh_unreviewed_badge()

    # --- Internal ---------------------------------------------------------

    def _load_csv_rows(self) -> None:
        """Append rows past the watermark from auto_capture_events.csv.

        Wrapped in try/finally so the unreviewed badge always refreshes
        — including the early-return paths (no session, missing CSV)
        and OSError. Without this, attaching to a fresh session whose
        CSV file doesn't exist yet would leave the badge stale at the
        previous session's count.
        """
        try:
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
        finally:
            self._refresh_unreviewed_badge()

    def _refresh_unreviewed_badge(self) -> None:
        """Recompute unreviewed-event count and emit the change signal.

        An event "needs attention" if:
          - state ∈ (pending, kept_default), OR
          - state == kept_explicit AND no primary_reconstruction label

        Discarded events never count — the explicit "no" decision is
        final, and they don't need labeling. The kept_explicit-without-
        label clause makes the badge useful through the labeling phase
        of the workflow, not just the keep/discard phase.

        Walks the table + label cache fresh each time rather than
        maintaining a counter — cheap (≤ ~50 rows per session) and stays
        consistent when other operations (e.g., a future hide-discarded
        filter) change the displayed-vs-existing row set.
        """
        count = 0
        for row_idx in range(self.events_table.rowCount()):
            state_item = self.events_table.item(row_idx, COL_STATE)
            if state_item is None:
                continue
            state = state_item.text()
            if state == EVENT_STATE_DISCARDED:
                continue
            if state not in _REVIEWED_STATES:
                count += 1
                continue
            # state == EVENT_STATE_KEPT_EXPLICIT — also unreviewed if no label
            idx_item = self.events_table.item(row_idx, COL_EVENT_IDX)
            try:
                event_idx = int(idx_item.text()) if idx_item else None
            except (TypeError, ValueError):
                event_idx = None
            label = (
                self._labels_cache.get(event_idx)
                if event_idx is not None else None
            )
            if not label or not label.get("primary_reconstruction"):
                count += 1
        self.unreviewed_count_changed.emit(count)

    def _add_event_row(self, row: dict) -> None:
        """Insert one CSV row at the top of the master list (newest first).

        The full CSV row dict is attached to the event_idx item via
        Qt.UserRole so the selection handler can recover buffer_dir,
        timestamp, etc. without re-reading the CSV.
        """
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
        idx_item = QTableWidgetItem(str(row.get("event_idx", "")))
        idx_item.setData(Qt.ItemDataRole.UserRole, dict(row))
        self.events_table.setItem(0, COL_EVENT_IDX, idx_item)
        self.events_table.setItem(0, COL_TIME, QTableWidgetItem(time_str))
        self.events_table.setItem(0, COL_SCORE, QTableWidgetItem(score_str))
        self.events_table.setItem(0, COL_TEMP, QTableWidgetItem(temp_str))
        self.events_table.setItem(0, COL_STATE, QTableWidgetItem(state))

    # --- Detail pane ------------------------------------------------------

    def _set_placeholder(self, text: str) -> None:
        """Show the placeholder copy and hide the image viewer."""
        self._detail_placeholder.setText(text)
        self._detail_placeholder.show()
        self._detail_content.hide()

    def _show_detail_content(self) -> None:
        """Show the image viewer and hide the placeholder."""
        self._detail_placeholder.hide()
        self._detail_content.show()

    def _on_selection_changed(self) -> None:
        """Handler for ``events_table.itemSelectionChanged``.

        Resolves the selected row's CSV dict (stored on the event_idx
        item's UserRole) and routes to the detail loader. Empty
        selections — including the cleared-table state right after
        attach_session — fall back to the placeholder.

        Flushes any pending debounced notes write for the *previous*
        event before switching, so unsaved typing doesn't get lost when
        the user clicks another row mid-sentence.
        """
        # Flush before switching — the user may have typed in notes for the
        # previous event without explicitly tabbing out.
        if self._notes_save_timer.isActive():
            self._notes_save_timer.stop()
            self._flush_notes_to_disk()

        row = self.events_table.currentRow()
        if row < 0 or not self.events_table.selectedItems():
            self._set_placeholder("Select an event to view buffer frames.")
            self._cached_pixmaps = []
            self._cached_paths = []
            self._image_label.clearImage()
            self._currently_displayed_event_idx = None
            return

        idx_item = self.events_table.item(row, COL_EVENT_IDX)
        if idx_item is None:
            return
        row_data = idx_item.data(Qt.ItemDataRole.UserRole)
        if not isinstance(row_data, dict):
            return
        self._load_detail(row_data)

    def _load_detail(self, row_data: dict) -> None:
        """Populate the detail pane for the given CSV row.

        Walks through the failure-mode ladder (no buffer dir recorded →
        dir missing on disk → dir empty) and shows a context-specific
        placeholder for each. On success, decodes all PNGs once into the
        pixmap cache so slider scrubbing is instantaneous.
        """
        event_idx = row_data.get("event_idx", "?")
        buffer_dir = row_data.get("buffer_dir", "") or ""

        if self._session_dir is None or not buffer_dir:
            self._set_placeholder(
                f"Event #{event_idx} has no buffer directory recorded."
            )
            return

        full_dir = self._session_dir / buffer_dir
        if not full_dir.exists() or not full_dir.is_dir():
            self._set_placeholder(
                f"Event #{event_idx} — buffer directory not found:\n{buffer_dir}"
            )
            return

        frame_paths = sorted(full_dir.glob("*.png"))
        if not frame_paths:
            self._set_placeholder(
                f"Event #{event_idx} — buffer directory is empty."
            )
            return

        self._cached_paths = frame_paths
        self._cached_pixmaps = [QPixmap(str(p)) for p in frame_paths]

        self._metadata_label.setText(self._format_metadata(row_data))
        self._show_detail_content()

        # Bind the labeling form to this event before we populate it,
        # so any side-effect signals from setText/setCurrentIndex don't
        # write under a stale event_idx.
        try:
            self._currently_displayed_event_idx = int(event_idx)
        except (TypeError, ValueError):
            self._currently_displayed_event_idx = None
        self._populate_label_form(self._currently_displayed_event_idx)

        # Default to the trigger frame (last one buffered before the
        # signal fired) — most informative single frame for the grower.
        last_idx = len(self._cached_pixmaps) - 1
        self._slider.blockSignals(True)
        self._slider.setRange(0, last_idx)
        self._slider.setValue(last_idx)
        self._slider.blockSignals(False)
        self._display_frame_at(last_idx)

    @staticmethod
    def _format_metadata(row_data: dict) -> str:
        """Render the metadata header line for an event."""
        event_idx = row_data.get("event_idx", "?")

        timestamp = row_data.get("timestamp", "")
        try:
            time_str = datetime.fromisoformat(timestamp).strftime("%H:%M:%S")
        except (TypeError, ValueError):
            time_str = "—"

        score_raw = row_data.get("change_score", "")
        try:
            score_str = f"{float(score_raw):.2f}"
        except (TypeError, ValueError):
            score_str = "—"

        temp_raw = row_data.get("pyrometer_temp_C", "")
        try:
            temp_str = (
                f"{float(temp_raw):.1f} ℃" if temp_raw not in ("", None) else "—"
            )
        except (TypeError, ValueError):
            temp_str = "—"

        state = row_data.get("event_state", "?") or "?"

        return (
            f"Event #{event_idx}  ·  {time_str}  ·  "
            f"score {score_str}  ·  {temp_str}  ·  {state}"
        )

    def _populate_label_form(self, event_idx: Optional[int]) -> None:
        """Pre-fill the labeling form from the cache for ``event_idx``.

        Programmatic setCurrentIndex on the QComboBox doesn't fire
        ``activated`` (only user interaction does), so no blockSignals
        is needed there. QLineEdit.setText DOES fire ``textChanged``,
        which would start the debounce timer pointlessly — so we block
        signals around the notes setText.
        """
        label = (
            self._labels_cache.get(event_idx)
            if event_idx is not None else None
        )
        primary = (label or {}).get(
            "primary_reconstruction", RECON_UNLABELED,
        )
        notes = (label or {}).get("notes", "")

        target_idx = self._primary_recon_combo.findData(primary)
        if target_idx < 0:
            target_idx = 0  # fall back to "(unlabeled)"
        self._primary_recon_combo.setCurrentIndex(target_idx)

        self._notes_input.blockSignals(True)
        self._notes_input.setText(notes)
        self._notes_input.blockSignals(False)

    @pyqtSlot(int)
    def _on_primary_recon_activated(self, idx: int) -> None:
        """Persist the new Primary reconstruction selection.

        ``activated`` fires only on user interaction, so getting here
        means the grower changed the dropdown. Writes atomically through
        the logger, keeps the cache in sync, and refreshes the badge —
        the new label may flip a kept_explicit-without-label event into
        the reviewed bucket (or the reverse, if "(unlabeled)" was picked).
        """
        if (
            self._currently_displayed_event_idx is None
            or self._growth_logger is None
        ):
            return
        primary = self._primary_recon_combo.itemData(idx)
        self._growth_logger.update_event_label(
            self._currently_displayed_event_idx,
            primary_reconstruction=primary,
        )
        cached = self._labels_cache.setdefault(
            self._currently_displayed_event_idx,
            {f: "" for f in GrowthLogger.EVENT_LABEL_FIELDS},
        )
        cached["event_idx"] = str(self._currently_displayed_event_idx)
        cached["primary_reconstruction"] = primary
        cached["label_timestamp_iso"] = datetime.now().isoformat()
        self._refresh_unreviewed_badge()

    @pyqtSlot()
    def _on_notes_text_changed(self) -> None:
        """Restart the debounced notes-save timer on every keystroke."""
        self._notes_save_timer.start()

    def _flush_notes_to_disk(self) -> None:
        """Write the current notes-field text to events_labels.csv.

        Idempotent — called from the debounce timer, the field's
        editingFinished signal, and the selection-change flush path.
        Notes don't affect the badge (only primary_reconstruction does),
        so no refresh is emitted from here.
        """
        self._notes_save_timer.stop()
        if (
            self._currently_displayed_event_idx is None
            or self._growth_logger is None
        ):
            return
        notes = self._notes_input.text()
        self._growth_logger.update_event_label(
            self._currently_displayed_event_idx,
            notes=notes,
        )
        cached = self._labels_cache.setdefault(
            self._currently_displayed_event_idx,
            {f: "" for f in GrowthLogger.EVENT_LABEL_FIELDS},
        )
        cached["event_idx"] = str(self._currently_displayed_event_idx)
        cached["notes"] = notes
        cached["label_timestamp_iso"] = datetime.now().isoformat()

    @pyqtSlot(int)
    def _display_frame_at(self, idx: int) -> None:
        """Show the frame at slider index ``idx``.

        Bounds-checked because Qt may emit valueChanged transiently
        during setRange/setValue calls; we already block signals around
        those, but defensive guarding is cheap.
        """
        if not (0 <= idx < len(self._cached_pixmaps)):
            return
        self._image_label.setOriginalPixmap(self._cached_pixmaps[idx])
        self._frame_position_label.setText(
            f"{idx + 1} / {len(self._cached_pixmaps)}"
        )
