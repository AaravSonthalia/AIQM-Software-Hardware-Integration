"""Tests for EvapControlWorker mode routing + Direct-read tab UI display.

Worker: verifies mode="screengrab"|"elog"|"dummy" routes to the correct
driver class in ``_create_driver``. This is the contract that decides
whether elog direct-read even gets a chance to connect — a bug here
silently reverts to screengrab or dummy without any visible signal.

Direct-read tab: verifies the Jul 9 2026 UI addition — new tab shows
all 10 .elo-sourced fields, plasma section auto-hides when the plasma
source is off (all-None state from ``ElogReader``'s NaN-filter), and
``reset_displays`` clears everything back to ``"---"`` on disarm.

Runs headless via ``QT_QPA_PLATFORM=offscreen``.
Run: ``python scripts/test_evap_control_worker.py``, or under pytest.
"""
from __future__ import annotations

import os
import sys
import unittest
from pathlib import Path

# Must set BEFORE any Qt import — headless CI/local runs on Mac + Linux.
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from PyQt6.QtWidgets import QApplication  # noqa: E402

_app = QApplication.instance() or QApplication(sys.argv)

from drivers.evap_control import (  # noqa: E402
    DummyEvapControl, ElogReader, EvapControl,
)
from gui.growth_monitor import GrowthMonitor  # noqa: E402
from gui.state import EvapControlState  # noqa: E402
from gui.workers import EvapControlWorker  # noqa: E402


def _make_evap_state(
    connected: bool = True,
    mode: str = "elog",
    chamber_pressure_mbar: float | None = 3.2e-9,
    substrate_temp_pv_C: float | None = 720.5,
    substrate_temp_setpoint_C: float | None = 725.0,
    cell_HTEC2_pv_C: float | None = 425.0,
    cell_Y_pv_C: float | None = 632.1,
    cell_Sr_pv_C: float | None = 550.3,
    cell_Eu_pv_C: float | None = 580.7,
    cell_Er_pv_C: float | None = 610.2,
    plasma_dc_bias_V: float | None = None,
    plasma_forward_W: float | None = None,
    plasma_reflected_W: float | None = None,
) -> EvapControlState:
    """Build an EvapControlState for tests. Defaults model a live
    elog-mode session with the plasma source OFF (all plasma_* None —
    matches ``ElogReader``'s NaN-filter behavior when plasma is off)."""
    return EvapControlState(
        mode=mode,
        connected=connected,
        error="",
        chamber_pressure_mbar=chamber_pressure_mbar,
        substrate_temp_pv_C=substrate_temp_pv_C,
        substrate_temp_setpoint_C=substrate_temp_setpoint_C,
        cell_HTEC2_pv_C=cell_HTEC2_pv_C,
        cell_Y_pv_C=cell_Y_pv_C,
        cell_Sr_pv_C=cell_Sr_pv_C,
        cell_Eu_pv_C=cell_Eu_pv_C,
        cell_Er_pv_C=cell_Er_pv_C,
        plasma_dc_bias_V=plasma_dc_bias_V,
        plasma_forward_W=plasma_forward_W,
        plasma_reflected_W=plasma_reflected_W,
    )


def _all_direct_read_displays(monitor: GrowthMonitor):
    """The 10 direct-read ValueDisplay widgets on the Direct-read tab.

    Kept as a helper so a schema addition only requires updating this
    list plus ``_build_direct_read_tab`` — tests that iterate over "all
    direct-read displays" automatically pick up new fields.
    """
    return (
        monitor.substrate_pv_display,
        monitor.substrate_sp_display,
        monitor.cell_HTEC2_display,
        monitor.cell_Y_display,
        monitor.cell_Sr_display,
        monitor.cell_Eu_display,
        monitor.cell_Er_display,
        monitor.plasma_dc_display,
        monitor.plasma_fwd_display,
        monitor.plasma_rfl_display,
    )


# ---------------------------------------------------------------------------
# Worker: mode routing
# ---------------------------------------------------------------------------

class WorkerModeRoutingTests(unittest.TestCase):
    """``_create_driver`` returns the right driver class per mode.

    Guarding this is the whole point of the mode flag — if elog silently
    reverted to screengrab, growers wouldn't see any error, they'd just
    see a session without direct-read data. Locking the contract with a
    test.
    """

    def test_screengrab_mode_returns_evap_control(self):
        worker = EvapControlWorker(mode="screengrab")
        driver = worker._create_driver()
        self.assertIsInstance(driver, EvapControl)

    def test_elog_mode_returns_elog_reader(self):
        worker = EvapControlWorker(mode="elog")
        driver = worker._create_driver()
        self.assertIsInstance(driver, ElogReader)

    def test_dummy_mode_returns_dummy_evap_control(self):
        worker = EvapControlWorker(mode="dummy")
        driver = worker._create_driver()
        self.assertIsInstance(driver, DummyEvapControl)

    def test_unknown_mode_falls_back_to_dummy(self):
        # Any unrecognized string hits the else branch in _create_driver
        # and returns DummyEvapControl. Documents the current
        # default-safe fallback — a typo'd mode string yields fake
        # data instead of a crash. Worth being explicit; if we ever
        # tighten this to raise ValueError, this test breaks first
        # and forces a call-site audit.
        worker = EvapControlWorker(mode="eloge")  # plausible typo
        driver = worker._create_driver()
        self.assertIsInstance(driver, DummyEvapControl)


# ---------------------------------------------------------------------------
# Config tab: mode combobox contents + default
# ---------------------------------------------------------------------------

class ConfigModeOptionsTests(unittest.TestCase):
    """Locks the option set and default of the Evap Control mode selector
    in the Config tab. The set of available modes drives which drivers
    can be exercised; the default determines what most sessions actually
    use.
    """

    def setUp(self):
        self.monitor = GrowthMonitor()

    def tearDown(self):
        self.monitor.deleteLater()

    def test_config_evap_mode_offers_three_options(self):
        options = {
            self.monitor.config_evap_mode.itemText(i)
            for i in range(self.monitor.config_evap_mode.count())
        }
        self.assertEqual(options, {"dummy", "elog", "screengrab"})

    def test_config_evap_mode_default_is_screengrab(self):
        # Current default as of Jul 9 2026 — grower has to opt into
        # elog. If the P3 default-mode discussion decides to flip
        # this, update this test alongside growth_monitor.py:745.
        self.assertEqual(
            self.monitor.config_evap_mode.currentText(),
            "screengrab",
        )


# ---------------------------------------------------------------------------
# Direct-read tab: UI display of the 10 elog-direct fields
# ---------------------------------------------------------------------------

class DirectReadTabTests(unittest.TestCase):
    """Jul 9 2026 UI addition. Verifies the tab exists, displays state
    correctly, and ``reset_displays`` clears it. Each of the 10
    elog-direct fields has its own ``ValueDisplay``; plasma section is
    hidden when the plasma source is off (all-None state) and shown when
    any plasma field is populated.
    """

    def setUp(self):
        self.monitor = GrowthMonitor()

    def tearDown(self):
        self.monitor.deleteLater()

    def test_direct_read_tab_registered(self):
        tab_labels = [
            self.monitor._tabs.tabText(i)
            for i in range(self.monitor._tabs.count())
        ]
        self.assertIn("Direct-read", tab_labels)

    def test_all_displays_start_at_dashes(self):
        # ValueDisplay's default value text is "---" — verifying this
        # so we know each widget exists and the tab construction
        # didn't crash somewhere partway through.
        for d in _all_direct_read_displays(self.monitor):
            self.assertEqual(d.value.text(), "---")

    def test_plasma_section_hidden_at_construction(self):
        # ``_build_direct_read_tab`` calls ``plasma_group.setVisible(False)``
        # so a fresh monitor doesn't show the plasma UI until an actual
        # populated state arrives.
        self.assertTrue(self.monitor.plasma_group.isHidden())

    def test_populated_state_populates_every_display(self):
        state = _make_evap_state(
            plasma_dc_bias_V=45.2,
            plasma_forward_W=250.5,
            plasma_reflected_W=3.1,
        )
        self.monitor.update_evap_state(state)

        # Substrate (1 decimal place, °C unit — matches ValueDisplay
        # constructor args in _build_direct_read_tab).
        self.assertEqual(
            self.monitor.substrate_pv_display.value.text(), "720.5 °C",
        )
        self.assertEqual(
            self.monitor.substrate_sp_display.value.text(), "725.0 °C",
        )
        # Cells
        self.assertEqual(
            self.monitor.cell_HTEC2_display.value.text(), "425.0 °C",
        )
        self.assertEqual(
            self.monitor.cell_Y_display.value.text(), "632.1 °C",
        )
        self.assertEqual(
            self.monitor.cell_Sr_display.value.text(), "550.3 °C",
        )
        self.assertEqual(
            self.monitor.cell_Eu_display.value.text(), "580.7 °C",
        )
        self.assertEqual(
            self.monitor.cell_Er_display.value.text(), "610.2 °C",
        )
        # Plasma
        self.assertEqual(
            self.monitor.plasma_dc_display.value.text(), "45.2 V",
        )
        self.assertEqual(
            self.monitor.plasma_fwd_display.value.text(), "250.5 W",
        )
        self.assertEqual(
            self.monitor.plasma_rfl_display.value.text(), "3.1 W",
        )

    def test_plasma_section_hidden_when_all_plasma_none(self):
        # Default _make_evap_state has all plasma fields None (models
        # an off plasma source that ElogReader NaN-filters). The
        # section should stay hidden.
        state = _make_evap_state()  # plasma_* all None
        self.monitor.update_evap_state(state)
        self.assertTrue(self.monitor.plasma_group.isHidden())

    def test_plasma_section_shown_when_any_plasma_populated(self):
        # One populated plasma field is enough to reveal the section
        # (the ``any`` clause in update_evap_state's visibility check).
        state = _make_evap_state(plasma_dc_bias_V=45.0)
        self.monitor.update_evap_state(state)
        self.assertFalse(self.monitor.plasma_group.isHidden())

    def test_disconnected_state_clears_all_direct_read_displays(self):
        # Populate first, then disconnect (worker's error path
        # produces connected=False + error text). Every direct-read
        # display should return to "---" — mirrors the pressure
        # display's connection-conditional behavior on the Monitor tab.
        self.monitor.update_evap_state(_make_evap_state())
        self.assertEqual(
            self.monitor.substrate_pv_display.value.text(), "720.5 °C",
        )
        disconnected = _make_evap_state(connected=False)
        self.monitor.update_evap_state(disconnected)
        for d in _all_direct_read_displays(self.monitor):
            self.assertEqual(d.value.text(), "---")

    def test_screengrab_mode_populates_pressure_only(self):
        # In screengrab mode, EvapControl.read() returns only
        # chamber_pressure_mbar. The worker sets all elog-direct
        # fields via .get() which returns None for missing keys, so
        # they stay at their None default. UI should show pressure
        # on the Monitor tab but "---" on all Direct-read fields.
        screengrab_state = EvapControlState(
            mode="screengrab",
            connected=True,
            error="",
            chamber_pressure_mbar=1e-9,
            # All elog-direct fields left at their None defaults.
        )
        self.monitor.update_evap_state(screengrab_state)
        self.assertEqual(
            self.monitor.pressure_display.value.text(), "1.00e-09 mbar",
        )
        for d in _all_direct_read_displays(self.monitor):
            self.assertEqual(d.value.text(), "---")
        self.assertTrue(self.monitor.plasma_group.isHidden())

    def test_reset_displays_clears_direct_read_and_hides_plasma(self):
        # End-of-session: reset_displays runs. All fields back to "---",
        # plasma section re-hidden regardless of last-populated state.
        plasma_on = _make_evap_state(
            plasma_dc_bias_V=45.0, plasma_forward_W=250.0,
            plasma_reflected_W=3.0,
        )
        self.monitor.update_evap_state(plasma_on)
        self.assertEqual(
            self.monitor.substrate_pv_display.value.text(), "720.5 °C",
        )
        self.assertFalse(self.monitor.plasma_group.isHidden())

        self.monitor.reset_displays()

        for d in _all_direct_read_displays(self.monitor):
            self.assertEqual(d.value.text(), "---")
        self.assertTrue(self.monitor.plasma_group.isHidden())


if __name__ == "__main__":
    unittest.main(verbosity=2)
