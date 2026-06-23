"""
Central action logging service for the hardware control GUI.
"""

import csv
from datetime import datetime
from typing import Optional

from PyQt6.QtCore import QObject, pyqtSignal

from gui.state import PowerSupplyState, TemperatureState, ActionLogEntry


MAX_ENTRIES = 10_000


class ActionLogger(QObject):
    """Central logging service that records user actions with instrument state snapshots."""

    entry_added = pyqtSignal(ActionLogEntry)
    cleared = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._entries: list[ActionLogEntry] = []
        self._latest_psu: Optional[PowerSupplyState] = None
        self._latest_temp: Optional[TemperatureState] = None

    @property
    def entries(self) -> list[ActionLogEntry]:
        return self._entries

    def update_psu_state(self, state: PowerSupplyState):
        """Update the latest PSU state snapshot (called from worker signal)."""
        self._latest_psu = state

    def update_temp_state(self, state: TemperatureState):
        """Update the latest temperature state snapshot (called from worker signal)."""
        self._latest_temp = state

    def log(self, category: str, action: str, details: str = ""):
        """Log an action with the current instrument state attached."""
        entry = ActionLogEntry(
            timestamp=datetime.now(),
            category=category,
            action=action,
            details=details,
        )

        # Attach PSU snapshot if available
        if self._latest_psu and self._latest_psu.connected:
            entry.psu_voltage = self._latest_psu.voltage_measured
            entry.psu_current = self._latest_psu.current_measured
            entry.psu_power = self._latest_psu.power_measured
            entry.psu_output = self._latest_psu.output_enabled

        # Attach temperature snapshot if available
        if self._latest_temp and self._latest_temp.connected:
            entry.temperature = self._latest_temp.temperature

        self._entries.append(entry)

        # Safety cap
        if len(self._entries) > MAX_ENTRIES:
            self._entries = self._entries[-MAX_ENTRIES:]

        self.entry_added.emit(entry)

    def export_csv(self, filepath: str):
        """Write all entries to a CSV file."""
        with open(filepath, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "Timestamp", "Category", "Action", "Details",
                "PSU Voltage (V)", "PSU Current (A)", "PSU Power (W)",
                "PSU Output", "Temperature (C)",
            ])
            for entry in self._entries:
                writer.writerow([
                    entry.timestamp.isoformat(),
                    entry.category,
                    entry.action,
                    entry.details,
                    f"{entry.psu_voltage:.3f}" if entry.psu_voltage is not None else "",
                    f"{entry.psu_current:.3f}" if entry.psu_current is not None else "",
                    f"{entry.psu_power:.3f}" if entry.psu_power is not None else "",
                    "ON" if entry.psu_output else ("OFF" if entry.psu_output is not None else ""),
                    f"{entry.temperature:.2f}" if entry.temperature is not None else "",
                ])

    def clear(self):
        """Clear all log entries."""
        self._entries.clear()
        self.cleared.emit()

    def count(self) -> int:
        return len(self._entries)
