"""
MBE Growth Monitor — OMBE Growth Log Assistant.

Provides live RHEED + pyrometer display, timestamped operations logging,
and growth log export for OMBE growths.

Tab layout:
  Monitor  — value displays, live RHEED image, note entry, LOG ENTRY button
  Session  — config (upper-left), sensor log (upper-right),
             growth notes (bottom half, full width), export
"""

from datetime import datetime
from typing import Optional

import numpy as np
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QPushButton, QLineEdit, QTextEdit, QSizePolicy, QFrame,
    QDoubleSpinBox, QComboBox, QFormLayout, QFileDialog,
    QTabWidget, QTableWidget, QTableWidgetItem, QHeaderView,
    QAbstractItemView, QGroupBox,
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QImage, QPixmap, QFont, QShortcut, QKeySequence

from gui.state import PowerSupplyState, CameraState, PyrometerState
from gui.widgets import ValueDisplay


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

class GrowthMonitor(QWidget):
    """Main growth monitoring widget — OMBE growth log assistant."""

    arm_requested = pyqtSignal()
    disarm_requested = pyqtSignal()
    start_requested = pyqtSignal()
    stop_requested = pyqtSignal()
    commit_requested = pyqtSignal(dict)
    export_requested = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet(DARK_STYLESHEET)

        self._state = "idle"  # idle | armed | running
        self._start_time: Optional[datetime] = None
        self._latest_psu: Optional[PowerSupplyState] = None
        self._latest_pyro: Optional[PyrometerState] = None
        self._latest_camera: Optional[CameraState] = None
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

        # Value displays at top of monitor tab
        vals = QHBoxLayout()
        self.elapsed_display = ValueDisplay("Elapsed Time", "", 0)
        self.elapsed_display.value.setText("00:00:00.00")
        self.temp_display = ValueDisplay("Temperature", "\u2103", 0)
        self.voltage_display = ValueDisplay("Voltage", "V", 2)
        self.current_display = ValueDisplay("Current", "A", 3)
        for d in (self.elapsed_display, self.temp_display,
                  self.voltage_display, self.current_display):
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

        # --- Right: Note input + LOG ENTRY ---
        right = QVBoxLayout()
        right.setSpacing(10)

        # Note input — prominent, expands to fill available space
        note_label = QLabel("Log Entry")
        note_label.setStyleSheet("font-size: 14px; font-weight: bold;")
        right.addWidget(note_label)

        self.log_note_input = QTextEdit()
        self.log_note_input.setPlaceholderText("What's happening now?")
        self.log_note_input.setStyleSheet(
            "QTextEdit { font-size: 15px; padding: 8px; }"
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

        self._tabs.addTab(tab, "Monitor")

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

        save_row = QHBoxLayout()
        self.config_save_path = QLineEdit()
        self.config_save_path.setPlaceholderText("logs/growths")
        self.config_save_path.setText("logs/growths")
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
        self.config_pyrometer_mode.addItems(["dummy", "modbus", "screengrab"])
        config_form.addRow("Pyrometer mode:", self.config_pyrometer_mode)

        top_half.addWidget(config_group)

        # --- Sensor Log (upper-right) ---
        sensor_container = QVBoxLayout()
        sensor_container.setSpacing(4)
        sensor_label = QLabel("Sensor Log \u2014 Interval Reads")
        sensor_label.setStyleSheet("font-weight: bold; font-size: 13px;")
        sensor_container.addWidget(sensor_label)

        self.sensor_log_table = QTableWidget(0, 4)
        self.sensor_log_table.setHorizontalHeaderLabels(
            ["Time", "Temp (\u2103)", "V (V)", "I (A)"]
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

    def update_camera_state(self, state: CameraState):
        self._latest_camera = state
        if state.frame is not None:
            self._current_frame = state.frame
            self._display_frame(state.frame)

    # ----- RHEED frame display --------------------------------------------

    def _display_frame(self, frame: np.ndarray):
        h, w = frame.shape[:2]
        if frame.ndim == 2:
            qimg = QImage(frame.data, w, h, w, QImage.Format.Format_Grayscale8)
        else:
            qimg = QImage(frame.data, w, h, 3 * w, QImage.Format.Format_RGB888)
        pixmap = QPixmap.fromImage(qimg).scaled(
            self.rheed_image_label.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        self.rheed_image_label.setPixmap(pixmap)

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

        # Add row to Growth Notes table
        self._add_growth_note_row(entry)

        # Clear the note input for next entry
        self.log_note_input.clear()

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

    # ----- Sensor Log display ---------------------------------------------

    def add_sensor_log_row(self, time_str: str, temp: Optional[float],
                           voltage: Optional[float] = None,
                           current: Optional[float] = None):
        """Add a row to the Sensor Log table (newest at top)."""
        table = self.sensor_log_table

        # Cap oldest rows (at bottom) to prevent UI lag during long sessions
        if table.rowCount() >= MAX_SENSOR_DISPLAY_ROWS:
            table.removeRow(table.rowCount() - 1)

        # Insert at row 0 so newest entry is always at top.
        # This also avoids resetting the user's scroll position —
        # their view shifts down by one row rather than jumping.
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

    # ----- Config helpers -------------------------------------------------

    def _on_config_browse(self):
        folder = QFileDialog.getExistingDirectory(self, "Select Save Folder")
        if folder:
            self.config_save_path.setText(folder)

    # ----- Accessors for GrowthApp ----------------------------------------

    def get_current_frame(self) -> Optional[np.ndarray]:
        return self._current_frame

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
        self.rheed_image_label.clear()
        self._start_time = None
        self._current_frame = None
        self._latest_psu = None
        self._latest_pyro = None
        self._latest_camera = None
