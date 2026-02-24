"""
Config tab — data recording, save settings, and hardware polling intervals.
"""

import os
from datetime import datetime

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QGroupBox, QDoubleSpinBox, QLineEdit, QComboBox, QCheckBox,
    QFileDialog, QFormLayout,
)
from PyQt6.QtCore import QTimer

from gui.action_logger import ActionLogger


class ConfigTab(QWidget):
    """Centralised configuration: recording controls, export settings, poll intervals."""

    def __init__(self, action_logger: ActionLogger, parent=None):
        super().__init__(parent)
        self.action_logger = action_logger

        self._rec_timer = QTimer(self)
        self._rec_timer.timeout.connect(self._on_record_tick)
        self._rec_count = 0

        self._build_ui()

    # ------------------------------------------------------------------
    # Public properties read by MainWindow
    # ------------------------------------------------------------------

    @property
    def psu_poll_interval(self) -> float:
        return self.psu_poll_spin.value()

    @property
    def tc_poll_interval(self) -> float:
        return self.tc_poll_spin.value()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(12)

        layout.addWidget(self._build_recording_group())
        layout.addWidget(self._build_export_group())
        layout.addWidget(self._build_polling_group())
        layout.addStretch()

    def _build_recording_group(self) -> QGroupBox:
        group = QGroupBox("Data Recording")
        form = QFormLayout(group)

        self.interval_spin = QDoubleSpinBox()
        self.interval_spin.setRange(0.1, 3600.0)
        self.interval_spin.setValue(1.0)
        self.interval_spin.setSuffix(" s")
        self.interval_spin.setDecimals(1)
        self.interval_spin.setSingleStep(0.5)
        form.addRow("Interval:", self.interval_spin)

        rec_row = QHBoxLayout()
        self.record_btn = QPushButton("Start Recording")
        self.record_btn.setCheckable(True)
        self.record_btn.clicked.connect(self._on_record_toggled)
        rec_row.addWidget(self.record_btn)

        self.rec_status = QLabel("")
        rec_row.addWidget(self.rec_status)
        rec_row.addStretch()
        form.addRow("", rec_row)

        return group

    def _build_export_group(self) -> QGroupBox:
        group = QGroupBox("Data Export")
        form = QFormLayout(group)

        path_row = QHBoxLayout()
        self.save_path_edit = QLineEdit()
        self.save_path_edit.setPlaceholderText("Select folder…")
        path_row.addWidget(self.save_path_edit)
        browse_btn = QPushButton("Browse…")
        browse_btn.clicked.connect(self._on_browse)
        path_row.addWidget(browse_btn)
        form.addRow("Save folder:", path_row)

        self.prefix_edit = QLineEdit("data_log")
        form.addRow("Filename prefix:", self.prefix_edit)

        self.format_combo = QComboBox()
        self.format_combo.addItems(["CSV"])
        form.addRow("Format:", self.format_combo)

        self.auto_export_check = QCheckBox("Auto-export when recording stops")
        form.addRow("", self.auto_export_check)

        return group

    def _build_polling_group(self) -> QGroupBox:
        group = QGroupBox("Hardware Polling")
        form = QFormLayout(group)

        self.psu_poll_spin = QDoubleSpinBox()
        self.psu_poll_spin.setRange(0.1, 10.0)
        self.psu_poll_spin.setValue(0.5)
        self.psu_poll_spin.setSingleStep(0.1)
        self.psu_poll_spin.setDecimals(1)
        self.psu_poll_spin.setSuffix(" s")
        form.addRow("PSU poll interval:", self.psu_poll_spin)

        self.tc_poll_spin = QDoubleSpinBox()
        self.tc_poll_spin.setRange(0.1, 10.0)
        self.tc_poll_spin.setValue(0.5)
        self.tc_poll_spin.setSingleStep(0.1)
        self.tc_poll_spin.setDecimals(1)
        self.tc_poll_spin.setSuffix(" s")
        form.addRow("Thermocouple poll interval:", self.tc_poll_spin)

        note = QLabel("Changes take effect on next connect.")
        note.setStyleSheet("color: gray; font-style: italic;")
        form.addRow("", note)

        return group

    # ------------------------------------------------------------------
    # Slots
    # ------------------------------------------------------------------

    def _on_record_toggled(self, checked: bool):
        if checked:
            interval_ms = int(self.interval_spin.value() * 1000)
            self._rec_count = 0
            self._rec_timer.start(interval_ms)
            self.record_btn.setText("Stop Recording")
            self.record_btn.setStyleSheet("background-color: #d32f2f; color: white;")
            self.interval_spin.setEnabled(False)
            self.action_logger.log(
                "Recording", "Started", f"Interval: {self.interval_spin.value():.1f}s"
            )
        else:
            self._rec_timer.stop()
            self.record_btn.setText("Start Recording")
            self.record_btn.setStyleSheet("")
            self.interval_spin.setEnabled(True)
            self.rec_status.setText("")
            self.action_logger.log(
                "Recording", "Stopped", f"{self._rec_count} measurements taken"
            )
            if self.auto_export_check.isChecked():
                self._auto_export()

    def _on_record_tick(self):
        self._rec_count += 1
        self.action_logger.log("Measurement", "Auto-Record", f"#{self._rec_count}")
        self.rec_status.setText(f"#{self._rec_count}")

    def _on_browse(self):
        folder = QFileDialog.getExistingDirectory(self, "Select Save Folder")
        if folder:
            self.save_path_edit.setText(folder)

    def _auto_export(self):
        folder = self.save_path_edit.text().strip()
        if not folder:
            return
        prefix = self.prefix_edit.text().strip() or "data_log"
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{prefix}_{timestamp}.csv"
        filepath = os.path.join(folder, filename)
        try:
            self.action_logger.export_csv(filepath)
        except Exception as e:
            self.action_logger.log("Export", "Error", str(e))
