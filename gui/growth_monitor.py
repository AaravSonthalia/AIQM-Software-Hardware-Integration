"""
MBE Growth Monitor — OMBE Growth Log Assistant.

Provides live RHEED + pyrometer display, timestamped operations logging,
and growth log export for OMBE growths.

Tab layout:
  Monitor  — value displays, live RHEED image, note entry, LOG ENTRY button
  Session  — config (upper-left), sensor log (upper-right),
             growth notes (bottom half, full width), export
"""

import sys
from datetime import datetime
from pathlib import Path
from typing import Optional

import numpy as np
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QPushButton, QLineEdit, QTextEdit, QSizePolicy, QFrame,
    QDoubleSpinBox, QComboBox, QFormLayout, QFileDialog,
    QTabWidget, QTableWidget, QTableWidgetItem, QHeaderView,
    QAbstractItemView, QGroupBox, QSlider, QCheckBox,
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QImage, QPixmap, QFont, QShortcut, QKeySequence


def _default_save_path() -> str:
    """Pick the default growth-log save path.

    On Bulbasaur (Windows with the OMBE SSD mounted at E:), prefer
    ``E:\\OMBE\\GrowthMonitor`` per PI directive that all growth data
    written by this GUI should go to the SSD. Falls back to the original
    ``logs/growths`` (relative to the repo) when the SSD isn't present —
    keeps the default sane on Mac, on Windows boxes without the SSD
    attached, and on any other deployment.
    """
    ssd_root = Path(r"E:\OMBE")
    if sys.platform == "win32" and ssd_root.exists():
        return str(ssd_root / "GrowthMonitor")
    return "logs/growths"

from gui.state import (
    CameraState, ClassifierState, EvapControlState, MistralState,
    PowerSupplyState, PyrometerState,
)
from gui.widgets import ValueDisplay
from gui.growth_logger import (
    EVENT_STATE_DISCARDED,
    EVENT_STATE_KEPT_DEFAULT,
    EVENT_STATE_KEPT_EXPLICIT,
)
from gui.events_tab import EventsTab


# ---------------------------------------------------------------------------
# Stylesheet
# ---------------------------------------------------------------------------

DARK_STYLESHEET = """
QWidget {
    background-color: #111;
    color: #ddd;
    font-size: 13px;
}
QLineEdit, QTextEdit {
    background-color: #fff;
    color: #111;
    border: 1px solid #555;
    padding: 4px;
}
QDoubleSpinBox, QComboBox {
    border: 1px solid #555;
    border-radius: 0px;
    padding: 4px;
    background-color: #fff;
    color: #111;
}
QComboBox QAbstractItemView {
    background-color: #fff;
    color: #111;
    selection-background-color: #2563eb;
    selection-color: #fff;
    border: 1px solid #555;
}
QComboBox::drop-down {
    border: none;
}
QLabel {
    background-color: transparent;
}
QPushButton {
    border: 1px solid #555;
    padding: 8px 16px;
    font-weight: bold;
    background-color: #333;
    color: #ddd;
}
QPushButton:disabled {
    background-color: #222;
    color: #666;
    border-color: #333;
}
QTabWidget::pane {
    border: 1px solid #555;
    background-color: #111;
}
QTabBar::tab {
    background: #222;
    color: #aaa;
    padding: 8px 16px;
    border: 1px solid #555;
    border-bottom: none;
}
QTabBar::tab:selected {
    background: #333;
    color: #fff;
}
QTableWidget {
    background-color: #1a1a1a;
    color: #ddd;
    gridline-color: #333;
    border: 1px solid #555;
}
QTableWidget::item {
    padding: 4px;
}
QHeaderView::section {
    background-color: #222;
    color: #ddd;
    padding: 6px;
    border: 1px solid #333;
    font-weight: bold;
}
QGroupBox {
    border: 1px solid #555;
    margin-top: 8px;
    padding-top: 16px;
    font-weight: bold;
}
QGroupBox::title {
    color: #ddd;
    subcontrol-position: top left;
    padding: 2px 8px;
}
"""

BTN_ARM = "QPushButton { background-color: #2563eb; color: white; }"
BTN_DISARM = "QPushButton { background-color: #7c3aed; color: white; }"
BTN_START = "QPushButton { background-color: #16a34a; color: white; }"
BTN_STOP = "QPushButton { background-color: #dc2626; color: white; }"

MAX_SENSOR_DISPLAY_ROWS = 500


# ---------------------------------------------------------------------------
# GrowthMonitor
# ---------------------------------------------------------------------------

class AutoCaptureBanner(QFrame):
    """Non-modal interrupt banner shown when auto-capture flags an event.

    Default behavior is to *keep* the just-saved context buffer — the
    countdown auto-confirms keep on timeout. The grower can hit Discard
    to delete the buffer directory if the flag looks spurious. Discard
    only deletes the visual frames; the CSV row in
    ``auto_capture_events.csv`` stays as a record that the detector
    fired (with `buffer_count` reflecting the original save).

    Apr 17 design — the "progress bar" interrupt mechanism that builds
    grower trust in the detector's prompt cadence before the
    classifier-driven version takes over.
    """

    decision_made = pyqtSignal(int, str, str)  # emits (event_idx, buffer_dir, state)

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setFrameShape(QFrame.Shape.StyledPanel)
        self.setStyleSheet(
            "AutoCaptureBanner { background-color: #fff5e6; "
            "border-left: 4px solid #d97706; border-radius: 2px; }"
            "QLabel { color: #333; background: transparent; }"
            "QPushButton { padding: 4px 14px; font-size: 12px; }"
        )
        self.setFixedHeight(44)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(10, 4, 10, 4)
        layout.setSpacing(10)

        self._icon_label = QLabel("⚡")  # lightning bolt
        self._icon_label.setStyleSheet("font-size: 18px; color: #d97706;")
        layout.addWidget(self._icon_label)

        self._message_label = QLabel("")
        self._message_label.setStyleSheet("font-size: 13px; font-weight: 600;")
        layout.addWidget(self._message_label)

        layout.addStretch(1)

        self._countdown_label = QLabel("")
        self._countdown_label.setStyleSheet("font-size: 12px; color: #666;")
        layout.addWidget(self._countdown_label)

        self._discard_btn = QPushButton("Discard")
        self._discard_btn.setStyleSheet(
            "QPushButton { background-color: #fff; color: #b91c1c; "
            "border: 1px solid #b91c1c; border-radius: 3px; }"
            "QPushButton:hover { background-color: #fee2e2; }"
        )
        self._discard_btn.clicked.connect(self._on_discard_clicked)
        layout.addWidget(self._discard_btn)

        self._keep_btn = QPushButton("Keep Now")
        self._keep_btn.setStyleSheet(
            "QPushButton { background-color: #fff; color: #15803d; "
            "border: 1px solid #15803d; border-radius: 3px; }"
            "QPushButton:hover { background-color: #dcfce7; }"
        )
        self._keep_btn.clicked.connect(self._on_keep_clicked)
        layout.addWidget(self._keep_btn)

        self._countdown_timer = QTimer(self)
        self._countdown_timer.setInterval(1000)
        self._countdown_timer.timeout.connect(self._tick)
        self._countdown_remaining = 0
        self._current_event_idx = 0
        self._current_buffer_dir = ""
        self.hide()

    def show_event(
        self,
        event_idx: int,
        score: float,
        buffer_dir: str,
        countdown_s: int = 10,
    ) -> None:
        """Display the banner for one flagged event."""
        self._current_event_idx = event_idx
        self._current_buffer_dir = buffer_dir
        self._countdown_remaining = max(1, int(countdown_s))
        self._message_label.setText(
            f"Auto-capture event #{event_idx} flagged (score {score:.2f})"
        )
        self._update_countdown_label()
        self.show()
        self._countdown_timer.start()

    def _tick(self) -> None:
        self._countdown_remaining -= 1
        if self._countdown_remaining <= 0:
            self._on_keep_default_timeout()
        else:
            self._update_countdown_label()

    def _update_countdown_label(self) -> None:
        self._countdown_label.setText(
            f"Auto-keeping in {self._countdown_remaining}s"
        )

    def _on_discard_clicked(self) -> None:
        self._emit_decision(EVENT_STATE_DISCARDED)

    def _on_keep_clicked(self) -> None:
        self._emit_decision(EVENT_STATE_KEPT_EXPLICIT)

    def _on_keep_default_timeout(self) -> None:
        self._emit_decision(EVENT_STATE_KEPT_DEFAULT)

    def _emit_decision(self, state: str) -> None:
        """Stop countdown, emit decision_made, hide the banner."""
        self._countdown_timer.stop()
        self.decision_made.emit(
            self._current_event_idx,
            self._current_buffer_dir,
            state,
        )
        self.hide()


class GrowthMonitor(QWidget):
    """Main growth monitoring widget — OMBE growth log assistant."""

    arm_requested = pyqtSignal()
    disarm_requested = pyqtSignal()
    start_requested = pyqtSignal()
    stop_requested = pyqtSignal()
    commit_requested = pyqtSignal(dict)
    export_requested = pyqtSignal()
    # True = user wants auto-capture paused; False = wants it resumed.
    auto_capture_pause_toggled = pyqtSignal(bool)
    # Forwarded from the banner — emits (event_idx, buffer_dir, state).
    # State is one of EVENT_STATE_KEPT_EXPLICIT / EVENT_STATE_KEPT_DEFAULT /
    # EVENT_STATE_DISCARDED. GrowthApp connects this to update the row in
    # auto_capture_events.csv via GrowthLogger.update_auto_capture_state.
    auto_capture_decision = pyqtSignal(int, str, str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet(DARK_STYLESHEET)

        self._state = "idle"  # idle | armed | running
        self._start_time: Optional[datetime] = None
        self._latest_psu: Optional[PowerSupplyState] = None
        self._latest_pyro: Optional[PyrometerState] = None
        self._latest_camera: Optional[CameraState] = None
        self._latest_mistral: Optional[MistralState] = None
        self._latest_evap: Optional[EvapControlState] = None
        # Classifier state — cached by update_classifier_state so _on_commit
        # can pair the classifier's smoothed_percent with the grower's slider
        # values in every log entry (for Yuxin's #1 active-comparisons signal).
        self._latest_classifier: Optional[ClassifierState] = None
        # Grower-correction UI state (Jul 6 2026 — deliverable #5):
        #   _correction_active — True while the ✎ Correct toggle is on.
        #   _adjusting — reentrancy guard for Pattern A proportional
        #     adjustment; without it, the valueChanged signal cascade from
        #     one slider recursively re-triggers via the others we
        #     programmatically set, and Qt happily blows the stack.
        self._correction_active: bool = False
        self._adjusting: bool = False
        self._current_frame: Optional[np.ndarray] = None

        self._build_ui()
        self._apply_state()

        # Elapsed timer — 10 ms tick
        self._elapsed_timer = QTimer(self)
        self._elapsed_timer.setInterval(10)
        self._elapsed_timer.timeout.connect(self._tick_elapsed)

    # ----- Layout ----------------------------------------------------------

    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(12, 8, 12, 8)
        root.setSpacing(8)

        # === Top bar: Grower, Sample ID, buttons (always visible) ===
        top = QHBoxLayout()

        top.addWidget(QLabel("Grower:"))
        self.grower_input = QLineEdit()
        self.grower_input.setPlaceholderText("Enter grower name...")
        self.grower_input.setFixedWidth(180)
        top.addWidget(self.grower_input)

        top.addWidget(QLabel("  Sample ID:"))
        self.sample_id_input = QLineEdit()
        self.sample_id_input.setPlaceholderText("e.g. STO15_SY250702B")
        self.sample_id_input.setFixedWidth(220)
        top.addWidget(self.sample_id_input)

        top.addStretch()

        self.arm_btn = QPushButton("ARM")
        self.start_btn = QPushButton("START")
        self.stop_btn = QPushButton("STOP")
        for btn in (self.arm_btn, self.start_btn, self.stop_btn):
            btn.setFixedWidth(100)
        top.addWidget(self.arm_btn)
        top.addWidget(self.start_btn)
        top.addWidget(self.stop_btn)
        root.addLayout(top)

        # Separator
        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.HLine)
        sep.setStyleSheet("color: #555;")
        root.addWidget(sep)

        # === Tab widget ===
        self._tabs = QTabWidget()
        self._build_monitor_tab()
        self._build_events_tab()
        self._build_session_tab()
        root.addWidget(self._tabs, 1)

        # === Connect button signals ===
        self.arm_btn.clicked.connect(self._on_arm_clicked)
        self.start_btn.clicked.connect(self._on_start_clicked)
        self.stop_btn.clicked.connect(self._on_stop_clicked)

    # ----- Monitor Tab -----------------------------------------------------

    def _build_monitor_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        # Auto-capture event banner — hidden by default, surfaces when the
        # detector flags an event and disappears on Keep / Discard / timeout.
        self.auto_capture_banner = AutoCaptureBanner()
        self.auto_capture_banner.decision_made.connect(
            self._on_auto_capture_decision_internal,
        )
        layout.addWidget(self.auto_capture_banner)

        # Value displays at top of monitor tab
        vals = QHBoxLayout()
        self.elapsed_display = ValueDisplay("Elapsed Time", "", 0)
        self.elapsed_display.value.setText("00:00:00.00")
        self.temp_display = ValueDisplay("Temperature", "\u2103", 0)
        self.voltage_display = ValueDisplay("Voltage", "V", 2)
        self.current_display = ValueDisplay("Current", "A", 3)
        self.pressure_display = ValueDisplay("Pressure", "mbar", 2)
        for d in (self.elapsed_display, self.temp_display,
                  self.voltage_display, self.current_display,
                  self.pressure_display):
            d.setStyleSheet(
                "QFrame { border: 1px solid #555; } "
                "QLabel { background: transparent; }"
            )
            vals.addWidget(d)
        layout.addLayout(vals)

        # Main content: RHEED (left) | Note + Button (right)
        content = QHBoxLayout()
        content.setSpacing(12)

        # --- Left: RHEED image ---
        self.rheed_image_label = QLabel()
        self.rheed_image_label.setMinimumSize(400, 300)
        self.rheed_image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.rheed_image_label.setStyleSheet(
            "background-color: #000; border: 1px solid #555;"
        )
        self.rheed_image_label.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding,
        )
        content.addWidget(self.rheed_image_label, 2)

        # --- Right: Recon sliders + Note input + LOG ENTRY ---
        right = QVBoxLayout()
        right.setSpacing(8)

        # Live classification sliders — driven by the classifier worker
        # (see gui/workers.py::ClassifierWorker). Read-only for MVP; the
        # grower-override flow is a follow-up. Section header, canonical
        # label list from gui.recon_labels, and a per-cycle status line
        # below the sliders showing the raw-score sum + inference time.
        from gui.recon_labels import RECON_LABELS

        # Header row: title on the left, ✎ Correct toggle on the right.
        # Toggle unlocks the sliders for grower correction (Pattern A
        # proportional adjustment; sum always sits at 100). Auto-locks
        # after LOG ENTRY so every correction is a per-entry decision,
        # not a session-wide setting. See _on_correction_toggled +
        # _on_grower_slider_changed for the interaction logic.
        recon_header = QHBoxLayout()
        recon_header.setContentsMargins(0, 0, 0, 0)
        recon_label = QLabel("Live Classification (%)")
        recon_label.setStyleSheet("font-size: 13px; font-weight: bold;")
        recon_header.addWidget(recon_label, 1)

        self.correction_btn = QPushButton("✎ Correct")
        self.correction_btn.setCheckable(True)
        self.correction_btn.setFixedHeight(24)
        self.correction_btn.setStyleSheet(
            "QPushButton { background-color: #1a1a1a; color: #aaa; "
            "border: 1px solid #444; padding: 2px 10px; font-size: 11px; }"
            "QPushButton:checked { background-color: #7f1d3d; color: #fff; "
            "border: 1px solid #e11d48; }"
            "QPushButton:disabled { color: #444; border-color: #2a2a2a; }"
        )
        self.correction_btn.setToolTip(
            "Toggle to override the classifier's live prediction. Sliders "
            "become draggable and stay summed to 100% (proportional "
            "adjustment). The next LOG ENTRY captures both your belief and "
            "the classifier's — the pair feeds Yuxin's active-comparisons "
            "training signal. Auto-locks after each LOG ENTRY so every "
            "correction is a fresh decision."
        )
        self.correction_btn.clicked.connect(self._on_correction_toggled)
        recon_header.addWidget(self.correction_btn, 0)
        right.addLayout(recon_header)

        self._recon_sliders: dict[str, QSlider] = {}
        self._recon_value_labels: dict[str, QLabel] = {}
        recon_grid = QHBoxLayout()
        recon_grid.setSpacing(6)
        for name in RECON_LABELS:
            col = QVBoxLayout()
            col.setSpacing(2)
            lbl = QLabel(name)
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            lbl.setStyleSheet("font-size: 11px;")
            col.addWidget(lbl)
            slider = QSlider(Qt.Orientation.Vertical)
            slider.setRange(0, 100)
            slider.setValue(0)
            slider.setEnabled(False)  # Option A: read-only display
            slider.setTickPosition(QSlider.TickPosition.TicksRight)
            slider.setTickInterval(25)
            slider.setFixedHeight(90)
            slider.setStyleSheet(self._SLIDER_STYLE_LIVE)
            col.addWidget(slider, alignment=Qt.AlignmentFlag.AlignHCenter)
            val_lbl = QLabel("0%")
            val_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            val_lbl.setStyleSheet("font-size: 11px; color: #0d9488;")
            col.addWidget(val_lbl)
            recon_grid.addLayout(col)
            self._recon_sliders[name] = slider
            self._recon_value_labels[name] = val_lbl
        right.addLayout(recon_grid)

        # Grower-correction Pattern A signal wiring. The handler no-ops
        # unless self._correction_active is True, so classifier-driven
        # slider updates (which use blockSignals for their own reasons)
        # never trigger it. Default-arg capture on ``n`` is required to
        # dodge Python's late-binding closure-over-loop-variable trap.
        for name, slider in self._recon_sliders.items():
            slider.valueChanged.connect(
                lambda v, n=name: self._on_grower_slider_changed(n, v)
            )

        # Transparency label: shows raw-score sum, inference time, OOD flag,
        # or error text depending on classifier state. Same slot updates
        # both this label and the slider values (see update_classifier_state).
        self._recon_status_label = QLabel("Classifier idle")
        self._recon_status_label.setStyleSheet("font-size: 10px; color: #888;")
        self._recon_status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        right.addWidget(self._recon_status_label)

        # Note input
        note_label = QLabel("Log Entry")
        note_label.setStyleSheet("font-size: 14px; font-weight: bold;")
        right.addWidget(note_label)

        self.log_note_input = QTextEdit()
        self.log_note_input.setPlaceholderText("What's happening now?")
        self.log_note_input.setStyleSheet(
            "QTextEdit { font-size: 15px; padding: 8px; font-family: Arial; }"
        )
        right.addWidget(self.log_note_input, 1)  # stretch to fill

        # LOG ENTRY button — right below input
        self.commit_btn = QPushButton("LOG ENTRY  (Ctrl+S)")
        self.commit_btn.setStyleSheet(
            "QPushButton { background-color: #0d9488; color: white; "
            "font-size: 16px; font-weight: bold; }"
            "QPushButton:disabled { background-color: #222; color: #666; }"
        )
        self.commit_btn.setFixedHeight(54)
        self.commit_btn.clicked.connect(self._on_commit)
        right.addWidget(self.commit_btn)

        # Keyboard shortcut
        self._save_shortcut = QShortcut(QKeySequence("Ctrl+S"), self)
        self._save_shortcut.activated.connect(self._on_commit)

        content.addLayout(right, 1)
        layout.addLayout(content, 1)

        # Thin diagnostic footer for the auto-capture engine. Stays out of
        # the way during normal use; growers can ignore it unless they want
        # to see the live change-score / event count.
        footer = QHBoxLayout()
        footer.setContentsMargins(0, 0, 0, 0)
        footer.setSpacing(8)
        self.auto_capture_label = QLabel("Auto-capture: idle")
        self.auto_capture_label.setStyleSheet(
            "color: #888; font-size: 11px; padding: 4px 8px; "
            "background-color: #1a1a1a; border-top: 1px solid #333;"
        )
        self.auto_capture_label.setAlignment(Qt.AlignmentFlag.AlignLeft)
        footer.addWidget(self.auto_capture_label, 1)

        # Pause toggle — soft halt of auto-capture only. Sensor logging,
        # heartbeat, and frame display all keep running. Emergency stop
        # (full session halt) remains on the top-bar STOP button.
        self.pause_auto_capture_btn = QPushButton("Pause Auto-Capture")
        self.pause_auto_capture_btn.setCheckable(True)
        self.pause_auto_capture_btn.setEnabled(False)
        self.pause_auto_capture_btn.setStyleSheet(
            "QPushButton { background-color: #1a1a1a; color: #aaa; "
            "border: 1px solid #444; padding: 2px 10px; font-size: 11px; }"
            "QPushButton:checked { background-color: #7c2d12; color: #fff; "
            "border: 1px solid #c2410c; }"
            "QPushButton:disabled { color: #555; border-color: #2a2a2a; }"
        )
        self.pause_auto_capture_btn.clicked.connect(
            self._on_pause_auto_capture_clicked,
        )
        footer.addWidget(self.pause_auto_capture_btn, 0)
        layout.addLayout(footer)

        self._tabs.addTab(tab, "Monitor")

    # ----- Events Tab ------------------------------------------------------

    def _build_events_tab(self):
        """Mount the EventsTab — review surface for auto-capture events.

        Lives between Monitor (live) and Session (config + notes + export)
        in the tab order so the grower's natural left-to-right scan is
        now → just-fired events → session admin.

        The tab's text gets a "(N)" badge whenever there are unreviewed
        events — pending or kept_default — so the catch-up case after a
        walk-away is legible at a glance from any other tab.
        """
        self.events_tab = EventsTab()
        self._events_tab_index = self._tabs.addTab(self.events_tab, "Events")
        self.events_tab.unreviewed_count_changed.connect(
            self._on_unreviewed_count_changed,
        )

    def _on_unreviewed_count_changed(self, count: int):
        """Repaint the Events tab header with the unreviewed-event count.

        Empty badge when count == 0 to keep the header quiet during a
        fully-attended growth; non-zero counts surface explicitly.
        """
        label = "Events" if count == 0 else f"Events ({count})"
        self._tabs.setTabText(self._events_tab_index, label)

    # ----- Session Tab -----------------------------------------------------

    def _build_session_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        # === Top half: Config (left) + Sensor Log (right) ===
        top_half = QHBoxLayout()
        top_half.setSpacing(12)

        # --- Config (upper-left) ---
        config_group = QGroupBox("Config")
        config_group.setMaximumWidth(420)
        config_form = QFormLayout(config_group)
        config_form.setContentsMargins(10, 16, 10, 8)
        config_form.setSpacing(6)
        config_form.setFieldGrowthPolicy(
            QFormLayout.FieldGrowthPolicy.ExpandingFieldsGrow
        )

        self.config_interval_spin = QDoubleSpinBox()
        self.config_interval_spin.setRange(1.0, 600.0)
        self.config_interval_spin.setValue(1.0)
        self.config_interval_spin.setSuffix(" s")
        self.config_interval_spin.setDecimals(0)
        self.config_interval_spin.setSingleStep(1.0)
        config_form.addRow("Recording interval:", self.config_interval_spin)

        # Heartbeat interval — how often we save a RHEED frame regardless
        # of detector flags. Default 5 s per the May 21 joint decision
        # (was env-var-only before Jun 23). Range 1 s (dense) to 600 s
        # (10 min, the old default). Spans the typical use cases.
        # Applied at session START — changing mid-session has no effect.
        self.config_heartbeat_interval_spin = QDoubleSpinBox()
        self.config_heartbeat_interval_spin.setRange(1.0, 600.0)
        self.config_heartbeat_interval_spin.setValue(5.0)
        self.config_heartbeat_interval_spin.setSuffix(" s")
        self.config_heartbeat_interval_spin.setDecimals(0)
        self.config_heartbeat_interval_spin.setSingleStep(1.0)
        self.config_heartbeat_interval_spin.setToolTip(
            "How often to save a RHEED heartbeat frame. Lower = denser "
            "data (more disk + memory); 5 s default per May 21 joint "
            "group meeting."
        )
        config_form.addRow(
            "RHEED heartbeat interval:", self.config_heartbeat_interval_spin,
        )

        save_row = QHBoxLayout()
        self.config_save_path = QLineEdit()
        default_save = _default_save_path()
        self.config_save_path.setPlaceholderText(default_save)
        self.config_save_path.setText(default_save)
        save_row.addWidget(self.config_save_path)
        browse_btn = QPushButton("Browse")
        browse_btn.setFixedWidth(70)
        browse_btn.setStyleSheet(
            "QPushButton { font-size: 11px; padding: 4px 8px; }"
        )
        browse_btn.clicked.connect(self._on_config_browse)
        save_row.addWidget(browse_btn)
        config_form.addRow("Save folder:", save_row)

        self.config_prefix = QLineEdit("growth")
        config_form.addRow("Filename prefix:", self.config_prefix)

        self.config_camera_mode = QComboBox()
        self.config_camera_mode.addItems(["dummy", "screengrab", "direct"])
        config_form.addRow("Camera mode:", self.config_camera_mode)

        self.config_pyrometer_mode = QComboBox()
        self.config_pyrometer_mode.addItems(["dummy", "exactus", "modbus", "screengrab"])
        config_form.addRow("Pyrometer mode:", self.config_pyrometer_mode)

        self.config_exactus_port = QLineEdit("COM4")
        config_form.addRow("Exactus port:", self.config_exactus_port)

        self.config_exactus_baud = QComboBox()
        self.config_exactus_baud.addItems(
            ["9600", "19200", "38400", "57600", "115200"]
        )
        self.config_exactus_baud.setCurrentText("115200")
        config_form.addRow("Exactus baud:", self.config_exactus_baud)

        self.config_mistral_mode = QComboBox()
        # "jsonrpc" — direct-read of the Scandes JSON-RPC 2.0 backend at
        # http://10.0.42.231:9000/api (discovered Jun 23 2026). Multi-client
        # safe alongside a running MistralGui. See drivers/mistral_jsonrpc.py
        # and scripts/test_mistral_jsonrpc_discovery.py. Selecting this mode
        # before running the discovery probe on Bulbasaur produces a
        # connected driver with all-None V/I readings (read_config empty)
        # — no crashes, just no data until set_read_config populates methods.
        self.config_mistral_mode.addItems(["dummy", "screengrab", "jsonrpc"])
        self.config_mistral_mode.setCurrentText("screengrab")
        config_form.addRow("MISTRAL mode:", self.config_mistral_mode)

        self.config_evap_mode = QComboBox()
        # "elog" — direct-read of EvapControl's own .elo binary log
        # (drivers/elog.py). No OCR, no window positioning. See
        # drivers/evap_control.py:ElogReader. Default stays "screengrab"
        # for backward compatibility until elog has live-lab validation.
        self.config_evap_mode.addItems(["dummy", "elog", "screengrab"])
        self.config_evap_mode.setCurrentText("screengrab")
        config_form.addRow("Evap Control mode:", self.config_evap_mode)

        # Enable/disable the live classifier. Off = classifier worker
        # never starts, main-tab sliders stay at zero and status shows
        # "Classifier disabled". Useful when the classifier is misbehaving
        # (e.g. Bulbasaur without torch) or when a grower explicitly
        # doesn't want it — the rest of the app runs unchanged.
        self.config_classifier_enabled = QCheckBox()
        self.config_classifier_enabled.setChecked(True)
        config_form.addRow("Live classifier:", self.config_classifier_enabled)

        top_half.addWidget(config_group)

        # --- Sensor Log (upper-right) ---
        sensor_container = QVBoxLayout()
        sensor_container.setSpacing(4)
        sensor_label = QLabel("Sensor Log \u2014 Interval Reads")
        sensor_label.setStyleSheet("font-weight: bold; font-size: 13px;")
        sensor_container.addWidget(sensor_label)

        self.sensor_log_table = QTableWidget(0, 5)
        self.sensor_log_table.setHorizontalHeaderLabels(
            ["Time", "Temp (\u2103)", "V (V)", "I (A)", "P (mbar)"]
        )
        s_header = self.sensor_log_table.horizontalHeader()
        s_header.setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.sensor_log_table.setEditTriggers(
            QAbstractItemView.EditTrigger.NoEditTriggers
        )
        self.sensor_log_table.setSelectionBehavior(
            QAbstractItemView.SelectionBehavior.SelectRows
        )
        self.sensor_log_table.verticalHeader().setVisible(False)
        sensor_container.addWidget(self.sensor_log_table)

        top_half.addLayout(sensor_container, 1)

        layout.addLayout(top_half)

        # === Bottom half: Growth Notes (full width) ===
        notes_label = QLabel("Growth Notes \u2014 Grower Commits")
        notes_label.setStyleSheet("font-weight: bold; font-size: 13px;")
        layout.addWidget(notes_label)

        self.growth_notes_table = QTableWidget(0, 5)
        self.growth_notes_table.setHorizontalHeaderLabels(
            ["Time", "Temp (\u2103)", "V (V)", "I (A)", "Note"]
        )
        n_header = self.growth_notes_table.horizontalHeader()
        # Columns 0-3: fixed width, evenly sharing ~50% of space
        for col in range(4):
            n_header.setSectionResizeMode(
                col, QHeaderView.ResizeMode.Interactive
            )
            self.growth_notes_table.setColumnWidth(col, 100)
        # Column 4 (Note): stretches to fill remaining ~50%
        n_header.setSectionResizeMode(4, QHeaderView.ResizeMode.Stretch)
        self.growth_notes_table.setEditTriggers(
            QAbstractItemView.EditTrigger.NoEditTriggers
        )
        self.growth_notes_table.setSelectionBehavior(
            QAbstractItemView.SelectionBehavior.SelectRows
        )
        self.growth_notes_table.verticalHeader().setVisible(False)
        layout.addWidget(self.growth_notes_table, 1)

        # Export button row
        btn_row = QHBoxLayout()
        btn_row.addStretch()
        self.export_btn = QPushButton("Export Growth Log")
        self.export_btn.setStyleSheet(
            "QPushButton { background-color: #2563eb; color: white; "
            "font-size: 14px; padding: 10px 20px; }"
        )
        self.export_btn.clicked.connect(lambda: self.export_requested.emit())
        btn_row.addWidget(self.export_btn)
        layout.addLayout(btn_row)

        self._tabs.addTab(tab, "Session")

    # ----- State machine ---------------------------------------------------

    def _apply_state(self):
        s = self._state
        if s == "idle":
            self.arm_btn.setText("ARM")
            self.arm_btn.setStyleSheet(BTN_ARM)
            self.arm_btn.setEnabled(True)
            self.start_btn.setEnabled(False)
            self.stop_btn.setEnabled(False)
            self.commit_btn.setEnabled(False)
            self.sample_id_input.setEnabled(True)
            self.grower_input.setEnabled(True)
        elif s == "armed":
            self.arm_btn.setText("DISARM")
            self.arm_btn.setStyleSheet(BTN_DISARM)
            self.arm_btn.setEnabled(True)
            self.start_btn.setEnabled(True)
            self.start_btn.setStyleSheet(BTN_START)
            self.stop_btn.setEnabled(False)
            self.commit_btn.setEnabled(False)
            self.sample_id_input.setEnabled(True)
            self.grower_input.setEnabled(True)
        elif s == "running":
            self.arm_btn.setEnabled(False)
            self.start_btn.setEnabled(False)
            self.stop_btn.setEnabled(True)
            self.stop_btn.setStyleSheet(BTN_STOP)
            self.commit_btn.setEnabled(True)
            self.sample_id_input.setEnabled(False)
            self.grower_input.setEnabled(False)

    def set_state(self, new_state: str):
        self._state = new_state
        self._apply_state()

    def _on_arm_clicked(self):
        if self._state == "idle":
            self.arm_requested.emit()
        elif self._state == "armed":
            self.disarm_requested.emit()

    def _on_start_clicked(self):
        if self._state == "armed":
            self._start_time = datetime.now()
            self._elapsed_timer.start()
            self.start_requested.emit()

    def _on_stop_clicked(self):
        if self._state == "running":
            self._elapsed_timer.stop()
            self.stop_requested.emit()

    # ----- Elapsed timer ---------------------------------------------------

    def _tick_elapsed(self):
        if self._start_time is None:
            return
        delta = datetime.now() - self._start_time
        total_s = delta.total_seconds()
        h = int(total_s // 3600)
        m = int((total_s % 3600) // 60)
        s = total_s % 60
        self.elapsed_display.value.setText(f"{h:02d}:{m:02d}:{s:05.2f}")

    def get_elapsed_seconds(self) -> float:
        if self._start_time is None:
            return 0.0
        return (datetime.now() - self._start_time).total_seconds()

    # ----- State update methods (called by GrowthApp fan-out) -------------

    def update_psu_state(self, state: PowerSupplyState):
        self._latest_psu = state
        if state.connected:
            self.voltage_display.set_value(state.voltage_measured)
            self.current_display.set_value(state.current_measured)
        else:
            self.voltage_display.value.setText("---")
            self.current_display.value.setText("---")

    def update_pyrometer_state(self, state: PyrometerState):
        self._latest_pyro = state
        if state.connected:
            self.temp_display.value.setText(f"{state.temperature:.0f} \u2103")
        else:
            self.temp_display.value.setText("---")

    def update_mistral_state(self, state: MistralState):
        self._latest_mistral = state
        if state.connected and state.v_actual is not None:
            self.voltage_display.set_value(state.v_actual)
        else:
            self.voltage_display.value.setText("---")
        if state.connected and state.i_actual is not None:
            self.current_display.set_value(state.i_actual)
        else:
            self.current_display.value.setText("---")

    def update_evap_state(self, state: EvapControlState):
        self._latest_evap = state
        p = state.chamber_pressure_mbar
        if state.connected and p is not None:
            self.pressure_display.value.setText(f"{p:.2e} mbar")
        else:
            self.pressure_display.value.setText("---")

    def update_camera_state(self, state: CameraState):
        self._latest_camera = state
        if state.frame is not None:
            self._current_frame = state.frame
            self._display_frame(state.frame)

    # Value-label style presets. Kept as constants so update_classifier_state
    # doesn't allocate style strings per emission (5-slider hot path at 2 Hz).
    _RECON_VAL_STYLE_NORMAL = "font-size: 11px; color: #0d9488;"
    _RECON_VAL_STYLE_ARGMAX = (
        "font-size: 13px; color: #14b8a6; font-weight: bold;"
    )
    _RECON_VAL_STYLE_PLACEHOLDER = "font-size: 11px; color: #666;"
    _RECON_STATUS_STYLE_INFO = "font-size: 10px; color: #888;"
    _RECON_STATUS_STYLE_WARN = "font-size: 10px; color: #d97706;"  # amber
    _RECON_STATUS_STYLE_ERROR = "font-size: 10px; color: #c33;"    # red
    # Correction-mode styles: rose (#e11d48) to distinguish grower-driven
    # sliders from classifier-driven at a glance. Same color also on the
    # ✎ Correct button checked state so the whole section reads "in
    # correction" instantly.
    _RECON_STATUS_STYLE_CORRECTION = (
        "font-size: 10px; color: #e11d48; font-weight: bold;"
    )
    _RECON_VAL_STYLE_CORRECTION = (
        "font-size: 11px; color: #e11d48; font-weight: bold;"
    )
    _SLIDER_STYLE_LIVE = (
        "QSlider::groove:vertical { background: #333; width: 6px; }"
        "QSlider::handle:vertical { background: #0d9488; height: 12px; "
        "margin: 0 -4px; border-radius: 6px; }"
        "QSlider::sub-page:vertical { background: #555; }"
        "QSlider::add-page:vertical { background: #0d9488; }"
    )
    _SLIDER_STYLE_CORRECTION = (
        "QSlider::groove:vertical { background: #3f0a1a; width: 6px; }"
        "QSlider::handle:vertical { background: #e11d48; height: 14px; "
        "margin: 0 -5px; border-radius: 7px; }"
        "QSlider::sub-page:vertical { background: #555; }"
        "QSlider::add-page:vertical { background: #e11d48; }"
    )
    _ARGMAX_HIGHLIGHT_THRESHOLD = 30  # only highlight when clearly above uniform-20

    def update_classifier_state(self, state: ClassifierState):
        """Route ClassifierState to the recon sliders + status label.

        Five display modes, chosen in this order:

        1. **loading** — bridge still initializing. Status: "Loading
           classifier…"; sliders left at whatever they were.
        2. **error** — bridge load or classify failed. Status: red
           "⚠ <error>"; sliders left unchanged.
        3. **warming up** — worker is ready but no non-OOD frame has
           arrived yet, so ``smoothed_percent`` is just the ready-time
           uniform-20 placeholder. Value labels show "—" (not "20%")
           so growers don't misread the placeholder as a real
           prediction. Status: "Waiting for frames…" or
           "Warming up — quality X.XX; no confident data yet".
        4. **OOD** — at least one confident classification has occurred,
           but the current frame is out-of-distribution. Sliders remain
           frozen at the last confident smoothed value (the worker
           enforces the freeze). Status: amber
           "OOD — quality X.XX | Nms | showing last confident".
        5. **normal** — confident classification, sliders reflect the
           freshly-EMA'd smoothed_percent. Status includes raw_sum,
           quality, and inference time for transparency. If any class
           clearly leads (> 30%), its value label is highlighted in
           bold + brighter teal for at-a-glance readability.

        Every non-error state also sets a hover-tooltip on the status
        label that includes the model version and explains what the
        current status means — useful for growers who don't speak ML
        fluently and for anyone debugging on Bulbasaur.

        Sliders are read-only (Option A) so ``blockSignals`` isn't
        strictly required, but it's cheap insurance if we ever re-enable
        interactivity for a grower-override mode.

        Correction-mode short-circuit: when self._correction_active is
        True, the grower is currently driving the sliders and the
        classifier's live output must NOT overwrite them. We still cache
        the state (so _on_commit can pair grower vs classifier in the
        log entry) and keep the status label showing the correction-mode
        message set by _on_correction_toggled. All slider/value-label
        rendering paths below are skipped.
        """
        # Cache first — needed for _on_commit's classifier_recon_* fields
        # even when correction mode has suppressed slider updates.
        self._latest_classifier = state

        if self._correction_active:
            return

        model_line = f"Model: {state.model_version}\n" if state.model_version else ""

        if state.loading:
            self._recon_status_label.setText("Loading classifier…")
            self._recon_status_label.setStyleSheet(self._RECON_STATUS_STYLE_INFO)
            self._recon_status_label.setToolTip(
                "Loading the classifier model. First arm of a session "
                "takes 1-2 seconds; subsequent arms in the same session "
                "reuse the loaded bridge."
            )
            return
        if state.error:
            self._recon_status_label.setText(f"⚠ {state.error}")
            self._recon_status_label.setStyleSheet(self._RECON_STATUS_STYLE_ERROR)
            self._recon_status_label.setToolTip(
                f"{state.error}\n\n"
                "Common causes: torch not installed on this machine, "
                "best_model.pth missing, AI_for_quantum repo not cloned. "
                "Uncheck 'Live classifier' in the config panel to disarm "
                "and re-arm without the classifier."
            )
            return

        smoothed = state.smoothed_percent

        # Warming-up state: no confident classification has ever arrived,
        # so the smoothed_percent values are placeholders — mark them "—"
        # in the value labels so growers don't misread them.
        if not state.has_confident_data:
            if smoothed:
                for name, slider in self._recon_sliders.items():
                    slider.blockSignals(True)
                    slider.setValue(int(smoothed.get(name, 0)))
                    slider.blockSignals(False)
                    self._recon_value_labels[name].setText("—")
                    self._recon_value_labels[name].setStyleSheet(
                        self._RECON_VAL_STYLE_PLACEHOLDER,
                    )
            if state.last_frame_number < 0:
                self._recon_status_label.setText("Waiting for frames…")
            else:
                self._recon_status_label.setText(
                    f"Warming up — quality {state.quality:.2f}; "
                    f"no confident data yet"
                )
            self._recon_status_label.setStyleSheet(self._RECON_STATUS_STYLE_INFO)
            self._recon_status_label.setToolTip(
                f"{model_line}"
                "Waiting for a frame the model recognizes with "
                f"confidence (quality ≥ {self._OOD_TOOLTIP_THRESHOLD}). "
                "Sliders show a neutral placeholder until then — the "
                "'—' means 'no data yet', not 'model predicts 20%'."
            )
            return

        # Confident-data state: sliders reflect real (EMA-smoothed) values.
        # Compute argmax to decide whether to highlight a leading class.
        argmax_label = (
            max(smoothed.items(), key=lambda kv: kv[1])[0] if smoothed else None
        )
        max_v = smoothed.get(argmax_label, 0) if argmax_label else 0
        highlight_winner = max_v > self._ARGMAX_HIGHLIGHT_THRESHOLD

        for name, slider in self._recon_sliders.items():
            v = int(smoothed.get(name, 0))
            slider.blockSignals(True)
            slider.setValue(v)
            slider.blockSignals(False)
            val_lbl = self._recon_value_labels[name]
            val_lbl.setText(f"{v}%")
            if highlight_winner and name == argmax_label:
                val_lbl.setStyleSheet(self._RECON_VAL_STYLE_ARGMAX)
            else:
                val_lbl.setStyleSheet(self._RECON_VAL_STYLE_NORMAL)

        # Enrich status line with quality + inference time for transparency
        if state.is_ood:
            self._recon_status_label.setText(
                f"OOD — quality {state.quality:.2f} | "
                f"{state.inference_ms:.0f} ms | showing last confident"
            )
            self._recon_status_label.setStyleSheet(self._RECON_STATUS_STYLE_WARN)
            self._recon_status_label.setToolTip(
                f"{model_line}"
                "Out-of-distribution: the model saw a frame unlike "
                "anything in its training set (quality below the "
                f"{self._OOD_TOOLTIP_THRESHOLD} threshold). Sliders "
                "are frozen at the last confident prediction until a "
                "recognizable frame arrives. This is expected during "
                "reconstruction transitions and beam-block events."
            )
        else:
            self._recon_status_label.setText(
                f"Sum: {state.raw_sum:.2f} | quality {state.quality:.2f} | "
                f"{state.inference_ms:.0f} ms"
            )
            self._recon_status_label.setStyleSheet(self._RECON_STATUS_STYLE_INFO)
            self._recon_status_label.setToolTip(
                f"{model_line}"
                "Sum: total of raw model scores before normalization "
                "(around 1.0 means the model is behaving; far from 1.0 "
                "means the Equalizer recipe is doing heavy scaling).\n"
                "Quality: the model's confidence in this specific frame.\n"
                "Ms: time to classify this frame."
            )

    # Documented for the tooltip messages so they stay in sync with
    # ClassifierWorker.OOD_QUALITY_THRESHOLD; hard-coded because the
    # worker class-level constant isn't cheap to reach from a Qt slot
    # hot-path. Update both if the threshold moves.
    _OOD_TOOLTIP_THRESHOLD = 0.3

    def set_classifier_disabled(self):
        """Show explicit "disabled" state — invoked from GrowthApp._on_arm
        when the user unchecks "Live classifier" in the config panel.

        Sliders zero out, value labels reset, status label makes the
        disabled state explicit so growers don't wonder why the
        classifier isn't reporting.

        Also disables the ✎ Correct button: correcting a classifier that
        isn't running would produce a "grower belief with no classifier
        pair" log entry, which pollutes Yuxin's #1 active-comparisons
        signal (the whole point of the pair is having both sides).
        """
        # Force correction off first — otherwise the setEnabled(False)
        # loop below re-locks sliders that the correction toggle would
        # otherwise have left enabled.
        if self._correction_active:
            self.correction_btn.setChecked(False)
            self._on_correction_toggled(False)
        self.correction_btn.setEnabled(False)
        for name, slider in self._recon_sliders.items():
            slider.blockSignals(True)
            slider.setValue(0)
            slider.blockSignals(False)
            self._recon_value_labels[name].setText("0%")
            self._recon_value_labels[name].setStyleSheet(
                self._RECON_VAL_STYLE_PLACEHOLDER,
            )
        self._recon_status_label.setText(
            "Classifier disabled — re-arm with the config toggle checked"
        )
        self._recon_status_label.setStyleSheet(self._RECON_STATUS_STYLE_INFO)
        self._recon_status_label.setToolTip(
            "The 'Live classifier' checkbox in the config panel is "
            "unchecked, so the classifier worker never started for "
            "this session. Everything else (camera, pyrometer, MISTRAL, "
            "auto-capture) runs as normal."
        )

    # ----- RHEED frame display --------------------------------------------

    def _display_frame(self, frame: np.ndarray):
        h, w = frame.shape[:2]
        # Ensure contiguous memory layout and convert to bytes for PyQt6
        frame = np.ascontiguousarray(frame)
        data = frame.tobytes()
        if frame.ndim == 2:
            qimg = QImage(data, w, h, w, QImage.Format.Format_Grayscale8)
        else:
            qimg = QImage(data, w, h, 3 * w, QImage.Format.Format_RGB888)
        pixmap = QPixmap.fromImage(qimg).scaled(
            self.rheed_image_label.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        self.rheed_image_label.setPixmap(pixmap)

    # ----- Grower correction (deliverable #5) -----------------------------

    def _on_correction_toggled(self, checked: bool):
        """Handler for the ✎ Correct toggle.

        On (checked=True):
            - Unlock sliders (setEnabled True) so the grower can drag.
            - Immediately normalize existing slider values to sum-100 so
              the correction state starts from a clean total, even when
              the classifier's smoothed_percent rounded to 99 or 101.
            - Swap slider handle color to rose (#e11d48) so it's obvious
              at a glance which mode is active.
            - Replace the status label with the correction-mode message.

        Off (checked=False):
            - Lock sliders (setEnabled False), restore teal handle color.
            - Re-render the last classifier state (if any) so the sliders
              resume tracking the live model. Idle fallback if the
              classifier never emitted.

        Sync the button state defensively so callers who invoke this
        directly (tests, reset paths, auto-lock in _on_commit) leave the
        UI in a self-consistent state — clicked(bool) from a real button
        click already matches, so setChecked here is a cheap no-op for
        that path.
        """
        self._correction_active = checked
        if self.correction_btn.isChecked() != checked:
            self.correction_btn.setChecked(checked)
        for slider in self._recon_sliders.values():
            slider.setEnabled(checked)

        if checked:
            self._normalize_sliders_to_100()
            for slider in self._recon_sliders.values():
                slider.setStyleSheet(self._SLIDER_STYLE_CORRECTION)
            for name, slider in self._recon_sliders.items():
                self._recon_value_labels[name].setText(f"{slider.value()}%")
                self._recon_value_labels[name].setStyleSheet(
                    self._RECON_VAL_STYLE_CORRECTION,
                )
            self._recon_status_label.setText(
                "✎ Correction mode — drag sliders (sum stays at 100%)"
            )
            self._recon_status_label.setStyleSheet(
                self._RECON_STATUS_STYLE_CORRECTION,
            )
            self._recon_status_label.setToolTip(
                "Grower correction is active. Drag any slider — the other "
                "four adjust proportionally to keep the total at 100%. The "
                "next LOG ENTRY captures both your belief and the "
                "classifier's, then auto-locks this toggle so the next "
                "entry is a fresh decision.\n\n"
                "Uncheck to resume classifier-driven display without "
                "logging a correction."
            )
        else:
            for slider in self._recon_sliders.values():
                slider.setStyleSheet(self._SLIDER_STYLE_LIVE)
            if self._latest_classifier is not None:
                self.update_classifier_state(self._latest_classifier)
            else:
                for name, slider in self._recon_sliders.items():
                    slider.blockSignals(True)
                    slider.setValue(0)
                    slider.blockSignals(False)
                    self._recon_value_labels[name].setText("0%")
                    self._recon_value_labels[name].setStyleSheet(
                        self._RECON_VAL_STYLE_NORMAL,
                    )
                self._recon_status_label.setText("Classifier idle")
                self._recon_status_label.setStyleSheet(
                    self._RECON_STATUS_STYLE_INFO,
                )
                self._recon_status_label.setToolTip("")

    def _normalize_sliders_to_100(self):
        """Force the 5 sliders to sum to 100 via proportional scaling.

        Called on correction-mode entry so the first draggable state
        starts from a clean total, even when the classifier's
        smoothed_percent sums to 99 or 101 due to integer rounding, or
        when the sliders are all at 0 (fresh session, classifier idle).

        Edge cases:
            - Total already 100 → no-op.
            - Total is 0 → uniform distribution (20 each with residual on
              the first slider — this is what "no signal, all classes
              equally likely" means).
            - Non-integer proportional result → assign residual to the
              largest slider so the integer sum lands exactly on 100.

        Uses blockSignals so this normalization pass doesn't trigger
        the grower-slider handler (which would try to re-normalize
        recursively).
        """
        values = {n: s.value() for n, s in self._recon_sliders.items()}
        total = sum(values.values())
        if total == 100:
            return

        n_sliders = len(values)
        if total == 0:
            base = 100 // n_sliders
            residual = 100 - base * n_sliders
            new_values = {
                name: base + (1 if i < residual else 0)
                for i, name in enumerate(values)
            }
        else:
            new_values = {
                n: int(round(v * 100 / total)) for n, v in values.items()
            }
            diff = 100 - sum(new_values.values())
            if diff != 0:
                largest = max(new_values, key=new_values.get)
                new_values[largest] += diff

        for name, v in new_values.items():
            slider = self._recon_sliders[name]
            slider.blockSignals(True)
            slider.setValue(max(0, min(100, v)))
            slider.blockSignals(False)

    def _on_grower_slider_changed(self, changed_name: str, new_value: int):
        """Grower dragged a slider — proportionally adjust the others so
        the total returns to 100 (Pattern A). No-op when correction is
        off, no-op when we're already inside a programmatic adjustment
        (recursion guard).

        Algorithm:
            1. The 4 other sliders' new sum must equal 100 - new_value.
            2. If those 4 currently sum to 0, distribute the target sum
               uniformly (with integer residual on the first sliders).
            3. Otherwise, scale each by (its_current / current_sum) *
               target — integer-rounded — then push any rounding
               residual (±1 or ±2) onto the currently-largest one so
               the integer total lands exactly on 100.

        The _adjusting guard is the primary recursion defense: it
        rejects the re-entry that happens when our own setValue()
        below fires *this* handler again for a sibling slider. The
        blockSignals() calls are belt-and-suspenders — cheap enough
        and make the intent explicit at the call site.
        """
        if not self._correction_active or self._adjusting:
            return

        self._adjusting = True
        try:
            others = [
                (n, s) for n, s in self._recon_sliders.items()
                if n != changed_name
            ]
            others_target_sum = 100 - new_value
            current_others_sum = sum(s.value() for _, s in others)

            if current_others_sum == 0:
                # No signal to preserve → uniform distribute.
                n_others = len(others)
                base = others_target_sum // n_others
                residual = others_target_sum - base * n_others
                new_values = {
                    name: base + (1 if i < residual else 0)
                    for i, (name, _) in enumerate(others)
                }
            else:
                # Preserve relative shape, scale to hit target sum.
                new_values = {
                    name: int(round(s.value() * others_target_sum
                                    / current_others_sum))
                    for name, s in others
                }
                diff = others_target_sum - sum(new_values.values())
                if diff != 0:
                    largest = max(new_values, key=new_values.get)
                    new_values[largest] += diff

            for name, v in new_values.items():
                slider = self._recon_sliders[name]
                slider.blockSignals(True)
                slider.setValue(max(0, min(100, v)))
                slider.blockSignals(False)

            # Refresh all value labels so what the grower sees matches
            # what will land in the CSV. Uses the correction style so
            # the row keeps its "grower is driving" visual state.
            for name, slider in self._recon_sliders.items():
                self._recon_value_labels[name].setText(f"{slider.value()}%")
                self._recon_value_labels[name].setStyleSheet(
                    self._RECON_VAL_STYLE_CORRECTION,
                )
        finally:
            self._adjusting = False

    # ----- LOG ENTRY handler ----------------------------------------------

    def _on_commit(self):
        if not self.commit_btn.isEnabled():
            return

        now = datetime.now()
        temp_str = (
            f"{self._latest_pyro.temperature:.1f}"
            if self._latest_pyro and self._latest_pyro.connected else ""
        )
        voltage_str = (
            f"{self._latest_psu.voltage_measured:.3f}"
            if self._latest_psu and self._latest_psu.connected else ""
        )
        current_str = (
            f"{self._latest_psu.current_measured:.3f}"
            if self._latest_psu and self._latest_psu.connected else ""
        )

        entry = {
            "timestamp": now.isoformat(),
            "time_display": now.strftime("%H:%M"),
            "sample_id": self.sample_id_input.text(),
            "grower": self.grower_input.text(),
            "elapsed_s": f"{self.get_elapsed_seconds():.2f}",
            "pyrometer_temp_C": temp_str,
            "voltage_V": voltage_str,
            "current_A": current_str,
            "note": self.log_note_input.toPlainText().strip(),
        }

        # Capture reconstruction estimates from sliders. When correction
        # is off, these mirror the classifier's smoothed_percent (sliders
        # are read-only and driven by update_classifier_state). When
        # correction is on, these are the grower's belief — Pattern A
        # keeps their sum at 100.
        for name, slider in self._recon_sliders.items():
            entry[f"recon_{name}"] = str(slider.value())

        # Snapshot the classifier's live prediction alongside the
        # grower's slider values so every log entry becomes a paired
        # (grower, classifier) datum for Yuxin's #1 active-comparisons
        # signal (see yuxin_deliverables_jul06.md). Blank when the
        # classifier never emitted (disabled for this session, or not
        # yet loaded).
        #
        # classifier_status column disambiguates why classifier_recon_*
        # might be blank or 0. Set even when _latest_classifier is None
        # so downstream analysis can distinguish DISABLED from OK-with-
        # 0-confidence (see growth_logger.py COMMIT_FIELDS comment).
        if self._latest_classifier is not None:
            smoothed = self._latest_classifier.smoothed_percent
            for name in self._recon_sliders:
                entry[f"classifier_recon_{name}"] = str(
                    smoothed.get(name, 0)
                )
            entry["grower_corrected"] = (
                "True" if self._correction_active else "False"
            )
            # Lifecycle branch — errors and loading are checked before
            # "OK" so a state with e.g. both loading=True and error=""
            # doesn't get called OK prematurely. Order matches
            # ClassifierState's documented lifecycle transitions.
            if self._latest_classifier.error:
                entry["classifier_status"] = "ERROR"
            elif self._latest_classifier.loading:
                entry["classifier_status"] = "LOADING"
            else:
                entry["classifier_status"] = "OK"
        else:
            for name in self._recon_sliders:
                entry[f"classifier_recon_{name}"] = ""
            entry["grower_corrected"] = ""
            entry["classifier_status"] = "DISABLED"

        # Add row to Growth Notes table
        self._add_growth_note_row(entry)

        # Clear the note input for next entry
        self.log_note_input.clear()

        # Auto-lock correction after every LOG ENTRY so the next entry
        # is a fresh decision — prevents stale grower values from
        # bleeding across log entries when the growth has moved on. If
        # correction was off, this is a no-op (setChecked(False) on an
        # unchecked button doesn't fire clicked).
        if self._correction_active:
            self.correction_btn.setChecked(False)
            self._on_correction_toggled(False)

        # Sliders are classifier-driven (read-only) — no reset needed.
        # The next classifier emission will keep them in sync with the
        # live model output.

        self.commit_requested.emit(entry)

    def _add_growth_note_row(self, entry: dict):
        """Add a row to the Growth Notes table."""
        row = self.growth_notes_table.rowCount()
        self.growth_notes_table.insertRow(row)
        self.growth_notes_table.setItem(
            row, 0, QTableWidgetItem(entry.get("time_display", ""))
        )
        self.growth_notes_table.setItem(
            row, 1, QTableWidgetItem(
                entry.get("pyrometer_temp_C", "") or "---"
            )
        )
        self.growth_notes_table.setItem(
            row, 2, QTableWidgetItem(
                entry.get("voltage_V", "") or "---"
            )
        )
        self.growth_notes_table.setItem(
            row, 3, QTableWidgetItem(
                entry.get("current_A", "") or "---"
            )
        )
        self.growth_notes_table.setItem(
            row, 4, QTableWidgetItem(entry.get("note", ""))
        )
        self.growth_notes_table.scrollToBottom()

    def clear_session_tables(self):
        """Reset Sensor Log + Growth Notes table UIs for a new session.

        The CSV files are correctly per-session (start_session opens fresh
        ones), but these QTableWidgets accumulate rows visually across
        sessions because nothing in the original wiring resets them.
        Mirrors the setRowCount(0) call EventsTab.attach_session uses.
        """
        self.sensor_log_table.setRowCount(0)
        self.growth_notes_table.setRowCount(0)

    # ----- Sensor Log display ---------------------------------------------

    def add_sensor_log_row(self, time_str: str, temp: Optional[float],
                           voltage: Optional[float] = None,
                           current: Optional[float] = None,
                           pressure: Optional[float] = None):
        """Add a row to the Sensor Log table (newest at top)."""
        table = self.sensor_log_table

        if table.rowCount() >= MAX_SENSOR_DISPLAY_ROWS:
            table.removeRow(table.rowCount() - 1)

        table.insertRow(0)
        table.setItem(0, 0, QTableWidgetItem(time_str))
        table.setItem(
            0, 1, QTableWidgetItem(
                f"{temp:.1f}" if temp is not None else "---"
            )
        )
        table.setItem(
            0, 2, QTableWidgetItem(
                f"{voltage:.3f}" if voltage is not None else "---"
            )
        )
        table.setItem(
            0, 3, QTableWidgetItem(
                f"{current:.3f}" if current is not None else "---"
            )
        )
        table.setItem(
            0, 4, QTableWidgetItem(
                f"{pressure:.2e}" if pressure is not None else "---"
            )
        )

    # ----- Config helpers -------------------------------------------------

    def _on_config_browse(self):
        folder = QFileDialog.getExistingDirectory(self, "Select Save Folder")
        if folder:
            self.config_save_path.setText(folder)

    # ----- Accessors for GrowthApp ----------------------------------------

    def get_current_frame(self) -> Optional[np.ndarray]:
        return self._current_frame

    def set_auto_capture_status(self, text: str):
        """Update the auto-capture footer label. Called by GrowthApp on
        each evaluated frame and on session start/stop."""
        self.auto_capture_label.setText(text)

    def show_auto_capture_event(
        self,
        event_idx: int,
        score: float,
        buffer_dir: str,
        countdown_s: int = 10,
    ):
        """Display the non-modal auto-capture banner for a flagged event."""
        self.auto_capture_banner.show_event(
            event_idx, score, buffer_dir, countdown_s,
        )

    def _on_auto_capture_decision_internal(
        self, event_idx: int, buffer_dir: str, state: str,
    ):
        """Re-emit the banner's decision signal at the GrowthMonitor level
        so GrowthApp can route the state update without knowing about the
        banner internals."""
        self.auto_capture_decision.emit(event_idx, buffer_dir, state)

    def set_auto_capture_pause_enabled(self, enabled: bool):
        """Enable/disable the pause toggle. Called by GrowthApp at session
        start (enable) and stop (disable + reset to unpaused)."""
        self.pause_auto_capture_btn.setEnabled(enabled)
        if not enabled and self.pause_auto_capture_btn.isChecked():
            # Force the toggle back to "Pause" so a re-armed session starts
            # in the running state without the button stuck in "Resume".
            self.pause_auto_capture_btn.setChecked(False)
            self.pause_auto_capture_btn.setText("Pause Auto-Capture")

    def _on_pause_auto_capture_clicked(self):
        """Toggle handler — flips label text and emits the request signal."""
        paused = self.pause_auto_capture_btn.isChecked()
        self.pause_auto_capture_btn.setText(
            "Resume Auto-Capture" if paused else "Pause Auto-Capture"
        )
        self.auto_capture_pause_toggled.emit(paused)

    def get_session_metadata(self) -> dict:
        """Return session metadata for growth log export."""
        return {
            "date": datetime.now().strftime("%Y-%m-%d"),
            "grower": self.grower_input.text(),
            "sample_id": self.sample_id_input.text(),
        }

    # ----- Reset -----------------------------------------------------------

    def reset_displays(self):
        """Clear live data displays back to defaults (preserves tables)."""
        self.elapsed_display.value.setText("00:00:00.00")
        self.temp_display.value.setText("---")
        self.voltage_display.value.setText("---")
        self.current_display.value.setText("---")
        self.pressure_display.value.setText("---")
        self.rheed_image_label.clear()
        self.auto_capture_label.setText("Auto-capture: idle")
        # Force grower correction off before wiping slider state so the
        # next session starts on a clean classifier-driven display, and
        # so the button visual state matches the internal state (a stuck
        # ✓ ✎ Correct button on a fresh session would be misleading).
        if self._correction_active:
            self.correction_btn.setChecked(False)
            self._on_correction_toggled(False)
        self.correction_btn.setEnabled(True)
        # Classifier sliders back to zero + idle status; the next arm will
        # trigger a fresh "Loading classifier…" cycle. Also reset the
        # value-label styles in case the previous session left an argmax
        # highlight or placeholder style behind.
        for name, slider in self._recon_sliders.items():
            slider.blockSignals(True)
            slider.setValue(0)
            slider.blockSignals(False)
            slider.setStyleSheet(self._SLIDER_STYLE_LIVE)
            self._recon_value_labels[name].setText("0%")
            self._recon_value_labels[name].setStyleSheet(
                self._RECON_VAL_STYLE_NORMAL,
            )
        self._recon_status_label.setText("Classifier idle")
        self._recon_status_label.setStyleSheet(self._RECON_STATUS_STYLE_INFO)
        self._start_time = None
        self._current_frame = None
        self._latest_psu = None
        self._latest_pyro = None
        self._latest_camera = None
        self._latest_mistral = None
        self._latest_evap = None
        self._latest_classifier = None
