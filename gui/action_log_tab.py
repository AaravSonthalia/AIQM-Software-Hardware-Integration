"""
Action log viewer tab widget.
"""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QTableWidget, QTableWidgetItem, QHeaderView, QFileDialog,
    QMessageBox, QDoubleSpinBox,
)
from PyQt6.QtCore import Qt, QTimer

from gui.state import ActionLogEntry
from gui.action_logger import ActionLogger


class ActionLogTab(QWidget):
    """Timestamped table of user actions and instrument readings."""

    COLUMNS = [
        "Timestamp", "Category", "Action", "Details",
        "PSU V", "PSU I", "PSU P", "Output", "Temp (C)",
    ]

    def __init__(self, action_logger: ActionLogger, parent=None):
        super().__init__(parent)
        self.action_logger = action_logger
        self._build_ui()
        self._connect_signals()

    def _build_ui(self):
        layout = QVBoxLayout(self)

        # Recording toolbar
        rec_bar = QHBoxLayout()

        rec_bar.addWidget(QLabel("Interval:"))
        self.interval_spin = QDoubleSpinBox()
        self.interval_spin.setRange(0.1, 3600.0)
        self.interval_spin.setValue(1.0)
        self.interval_spin.setSuffix(" s")
        self.interval_spin.setDecimals(1)
        self.interval_spin.setSingleStep(0.5)
        rec_bar.addWidget(self.interval_spin)

        self.record_btn = QPushButton("Start Recording")
        self.record_btn.setCheckable(True)
        self.record_btn.clicked.connect(self._on_record_toggled)
        rec_bar.addWidget(self.record_btn)

        self.rec_status = QLabel("")
        rec_bar.addWidget(self.rec_status)

        rec_bar.addStretch()
        layout.addLayout(rec_bar)

        # Recording timer
        self._rec_timer = QTimer(self)
        self._rec_timer.timeout.connect(self._on_record_tick)
        self._rec_count = 0

        # Table
        self.table = QTableWidget(0, len(self.COLUMNS))
        self.table.setHorizontalHeaderLabels(self.COLUMNS)
        self.table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.table.setAlternatingRowColors(True)
        self.table.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.ResizeToContents
        )
        self.table.horizontalHeader().setStretchLastSection(True)
        layout.addWidget(self.table)

        # Bottom bar
        bottom = QHBoxLayout()

        self.count_label = QLabel("0 entries")
        bottom.addWidget(self.count_label)

        bottom.addStretch()

        self.export_btn = QPushButton("Export CSV")
        self.export_btn.clicked.connect(self._on_export)
        bottom.addWidget(self.export_btn)

        self.clear_btn = QPushButton("Clear Log")
        self.clear_btn.clicked.connect(self._on_clear)
        bottom.addWidget(self.clear_btn)

        layout.addLayout(bottom)

    def _connect_signals(self):
        self.action_logger.entry_added.connect(self._on_entry_added)
        self.action_logger.cleared.connect(self._on_cleared)

    def _on_entry_added(self, entry: ActionLogEntry):
        """Append a new row to the table."""
        row = self.table.rowCount()
        self.table.insertRow(row)

        values = [
            entry.timestamp.strftime("%H:%M:%S.%f")[:-3],
            entry.category,
            entry.action,
            entry.details,
            f"{entry.psu_voltage:.3f}" if entry.psu_voltage is not None else "",
            f"{entry.psu_current:.3f}" if entry.psu_current is not None else "",
            f"{entry.psu_power:.3f}" if entry.psu_power is not None else "",
            "ON" if entry.psu_output else ("OFF" if entry.psu_output is not None else ""),
            f"{entry.temperature:.2f}" if entry.temperature is not None else "",
        ]

        for col, text in enumerate(values):
            item = QTableWidgetItem(text)
            item.setFlags(item.flags() & ~Qt.ItemFlag.ItemIsEditable)
            self.table.setItem(row, col, item)

        self.table.scrollToBottom()
        self.count_label.setText(f"{self.action_logger.count()} entries")

    def _on_cleared(self):
        """Clear the table."""
        self.table.setRowCount(0)
        self.count_label.setText("0 entries")

    def _on_record_toggled(self, checked: bool):
        if checked:
            interval_ms = int(self.interval_spin.value() * 1000)
            self._rec_count = 0
            self._rec_timer.start(interval_ms)
            self.record_btn.setText("Stop Recording")
            self.record_btn.setStyleSheet("background-color: #d32f2f; color: white;")
            self.interval_spin.setEnabled(False)
            self.action_logger.log("Recording", "Started", f"Interval: {self.interval_spin.value():.1f}s")
        else:
            self._rec_timer.stop()
            self.record_btn.setText("Start Recording")
            self.record_btn.setStyleSheet("")
            self.interval_spin.setEnabled(True)
            self.rec_status.setText("")
            self.action_logger.log("Recording", "Stopped", f"{self._rec_count} measurements taken")

    def _on_record_tick(self):
        self._rec_count += 1
        self.action_logger.log("Measurement", "Auto-Record", f"#{self._rec_count}")
        self.rec_status.setText(f"#{self._rec_count}")

    def _on_export(self):
        filepath, _ = QFileDialog.getSaveFileName(
            self, "Export Action Log", "action_log.csv", "CSV Files (*.csv)"
        )
        if filepath:
            try:
                self.action_logger.export_csv(filepath)
                QMessageBox.information(
                    self, "Export Complete", f"Exported {self.action_logger.count()} entries to:\n{filepath}"
                )
            except Exception as e:
                QMessageBox.critical(self, "Export Error", str(e))

    def _on_clear(self):
        reply = QMessageBox.question(
            self, "Clear Log",
            f"Clear all {self.action_logger.count()} log entries?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
        )
        if reply == QMessageBox.StandardButton.Yes:
            self.action_logger.clear()
