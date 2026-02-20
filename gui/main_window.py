"""
Main window — orchestrates tabs, workers, and signal fan-out.
"""

from typing import Optional

from PyQt6.QtWidgets import QMainWindow, QWidget, QVBoxLayout, QTabWidget, QMessageBox
from PyQt6.QtCore import pyqtSlot

from owon_power_supply import find_owon_supplies

from gui.state import PowerSupplyState, TemperatureState
from gui.workers import PowerSupplyWorker, ThermocoupleWorker
from gui.action_logger import ActionLogger
from gui.power_supply_tab import PowerSupplyTab
from gui.temperature_tab import TemperatureTab
from gui.dashboard_tab import DashboardTab
from gui.visuals_tab import VisualsTab
from gui.action_log_tab import ActionLogTab


class MainWindow(QMainWindow):
    """Main application window — orchestrator role."""

    def __init__(self, resource: Optional[str] = None):
        super().__init__()

        self.setWindowTitle("Hardware Control Dashboard")
        self.setMinimumSize(900, 650)

        self.resource = resource
        self.psu_worker: Optional[PowerSupplyWorker] = None
        self.thermo_worker: Optional[ThermocoupleWorker] = None

        # Central logging service
        self.action_logger = ActionLogger(self)

        # Build UI
        self._setup_ui()
        self._connect_tab_signals()

        self.action_logger.log("System", "Application Started", "")

        # Auto-connect if resource provided
        if self.resource:
            self._connect_to_psu()

    def _setup_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)

        self.tabs = QTabWidget()

        self.psu_tab = PowerSupplyTab(self.action_logger)
        self.temp_tab = TemperatureTab(self.action_logger)
        self.dashboard_tab = DashboardTab()
        self.visuals_tab = VisualsTab()
        self.log_tab = ActionLogTab(self.action_logger)

        self.tabs.addTab(self.psu_tab, "Power Supply")
        self.tabs.addTab(self.temp_tab, "Temperature")
        self.tabs.addTab(self.dashboard_tab, "Dashboard")
        self.tabs.addTab(self.visuals_tab, "Visuals")
        self.tabs.addTab(self.log_tab, "Action Log")

        main_layout.addWidget(self.tabs)
        self.statusBar().showMessage("Ready")

    def _connect_tab_signals(self):
        # Power Supply tab signals
        self.psu_tab.connect_requested.connect(self._connect_to_psu)
        self.psu_tab.disconnect_requested.connect(self._disconnect_from_psu)
        self.psu_tab.command_requested.connect(self._on_psu_command)

        # Temperature tab signals
        self.temp_tab.connect_requested.connect(self._connect_thermocouple)
        self.temp_tab.disconnect_requested.connect(self._disconnect_thermocouple)

    # --- PSU worker lifecycle ---

    @pyqtSlot()
    def _connect_to_psu(self):
        if self.psu_worker and self.psu_worker.isRunning():
            return

        if not self.resource:
            try:
                supplies = find_owon_supplies()
            except Exception:
                supplies = []
            if not supplies:
                QMessageBox.warning(self, "Not Found", "No OWON power supply detected.")
                return
            self.resource, idn = supplies[0]
            self.statusBar().showMessage(f"Found: {idn}")

        self.psu_worker = PowerSupplyWorker(self.resource)
        self.psu_worker.state_updated.connect(self._on_psu_state)
        self.psu_worker.start()
        self.psu_tab.status_label.setText("Connecting...")

    @pyqtSlot()
    def _disconnect_from_psu(self):
        if self.psu_worker:
            self.psu_worker.stop()
            self.psu_worker.wait()
            self.psu_worker = None

        self.resource = None
        self.psu_tab.on_disconnected()

    @pyqtSlot(str, tuple)
    def _on_psu_command(self, cmd: str, args: tuple):
        if self.psu_worker:
            self.psu_worker.queue_command(cmd, *args)
            if cmd == "emergency_stop":
                self.statusBar().showMessage("!!! EMERGENCY STOP ACTIVATED !!!")

    # --- Thermocouple worker lifecycle ---

    @pyqtSlot()
    def _connect_thermocouple(self):
        if self.thermo_worker and self.thermo_worker.isRunning():
            return

        self.thermo_worker = ThermocoupleWorker()
        self.thermo_worker.state_updated.connect(self._on_temp_state)
        self.thermo_worker.start()
        self.temp_tab.status_label.setText("Connecting...")

    @pyqtSlot()
    def _disconnect_thermocouple(self):
        if self.thermo_worker:
            self.thermo_worker.stop()
            self.thermo_worker.wait()
            self.thermo_worker = None

        self.temp_tab.on_disconnected()

    # --- Signal fan-out ---

    @pyqtSlot(PowerSupplyState)
    def _on_psu_state(self, state: PowerSupplyState):
        """Fan out PSU state to all consumers."""
        self.psu_tab.update_state(state)
        self.dashboard_tab.update_psu_state(state)
        self.visuals_tab.update_psu_state(state)
        self.action_logger.update_psu_state(state)

    @pyqtSlot(TemperatureState)
    def _on_temp_state(self, state: TemperatureState):
        """Fan out temperature state to all consumers."""
        self.temp_tab.update_state(state)
        self.dashboard_tab.update_temp_state(state)
        self.visuals_tab.update_temp_state(state)
        self.action_logger.update_temp_state(state)

    # --- Shutdown ---

    def closeEvent(self, event):
        self.action_logger.log("System", "Application Closing", "")
        self._disconnect_from_psu()
        self._disconnect_thermocouple()
        event.accept()
