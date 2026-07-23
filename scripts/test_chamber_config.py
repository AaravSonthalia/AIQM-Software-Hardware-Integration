#!/usr/bin/env python3
"""Headless tests for the chamber config system (drivers/config.py).

No GUI or hardware required. Verifies get_active_config() env-var dispatch,
field values for both chamber presets, and cell_display list structure.
"""
from __future__ import annotations
import os
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from drivers.config import (
    OXIDE_MBE, CHALCOGENIDE_MBE, SYSTEMS, get_active_config,
)


class TestGetActiveConfig(unittest.TestCase):

    def _with_chamber(self, value):
        """Context manager: set AIQM_CHAMBER env var and restore after."""
        import contextlib
        @contextlib.contextmanager
        def _ctx():
            old = os.environ.get("AIQM_CHAMBER")
            os.environ["AIQM_CHAMBER"] = value
            try:
                yield
            finally:
                if old is None:
                    os.environ.pop("AIQM_CHAMBER", None)
                else:
                    os.environ["AIQM_CHAMBER"] = old
        return _ctx()

    def test_default_is_ombe(self):
        os.environ.pop("AIQM_CHAMBER", None)
        cfg = get_active_config()
        self.assertEqual(cfg.chamber_id, "ombe")

    def test_ombe_explicit(self):
        with self._with_chamber("ombe"):
            cfg = get_active_config()
        self.assertEqual(cfg.chamber_id, "ombe")

    def test_chmbe_explicit(self):
        with self._with_chamber("chmbe"):
            cfg = get_active_config()
        self.assertEqual(cfg.chamber_id, "chmbe")

    def test_legacy_oxide_alias(self):
        with self._with_chamber("oxide"):
            cfg = get_active_config()
        self.assertEqual(cfg.chamber_id, "ombe")

    def test_legacy_chalcogenide_alias(self):
        with self._with_chamber("chalcogenide"):
            cfg = get_active_config()
        self.assertEqual(cfg.chamber_id, "chmbe")

    def test_unknown_falls_back_to_ombe(self):
        with self._with_chamber("nonexistent"):
            cfg = get_active_config()
        self.assertEqual(cfg.chamber_id, "ombe")

    def test_case_insensitive(self):
        with self._with_chamber("CHMBE"):
            cfg = get_active_config()
        self.assertEqual(cfg.chamber_id, "chmbe")


class TestOmbConfig(unittest.TestCase):

    def test_mistral_mode_default(self):
        self.assertEqual(OXIDE_MBE.mistral_mode_default, "screengrab")

    def test_evap_mode_default(self):
        self.assertEqual(OXIDE_MBE.evap_mode_default, "elog")

    def test_five_cells(self):
        self.assertEqual(len(OXIDE_MBE.cell_display), 5)

    def test_all_ombe_cells_have_state_field(self):
        for cell in OXIDE_MBE.cell_display:
            self.assertIsNotNone(cell["state_field"],
                                 f"O-MBE cell {cell['label']} missing state_field")

    def test_cell_labels_not_empty(self):
        for cell in OXIDE_MBE.cell_display:
            self.assertTrue(cell["label"])


class TestChMbeConfig(unittest.TestCase):

    def test_mistral_mode_default(self):
        self.assertEqual(CHALCOGENIDE_MBE.mistral_mode_default, "ads")

    def test_evap_mode_default(self):
        self.assertEqual(CHALCOGENIDE_MBE.evap_mode_default, "screengrab")

    def test_seven_cells(self):
        self.assertEqual(len(CHALCOGENIDE_MBE.cell_display), 7)

    def test_chmbe_cells_have_no_state_field(self):
        for cell in CHALCOGENIDE_MBE.cell_display:
            self.assertIsNone(cell["state_field"],
                              f"Ch-MBE cell {cell['label']} should have state_field=None")

    def test_evap_log_dir_set(self):
        self.assertTrue(CHALCOGENIDE_MBE.evap_log_dir)

    def test_cell_labels_not_empty(self):
        for cell in CHALCOGENIDE_MBE.cell_display:
            self.assertTrue(cell["label"])


class TestSystems(unittest.TestCase):

    def test_all_keys_present(self):
        for key in ("ombe", "oxide", "chmbe", "chalcogenide"):
            self.assertIn(key, SYSTEMS)

    def test_no_mutation_between_instances(self):
        # cell_display uses field(default_factory=...) so instances are independent
        OXIDE_MBE.cell_display.append({"label": "TEST", "state_field": None})
        # The default_factory config should not be affected (SYSTEMS uses pre-built instances)
        self.assertNotIn(
            {"label": "TEST", "state_field": None},
            CHALCOGENIDE_MBE.cell_display,
        )
        OXIDE_MBE.cell_display.pop()  # restore


if __name__ == "__main__":
    print("Running chamber config tests...\n")
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(unittest.TestLoader().discover(
        start_dir=str(Path(__file__).parent),
        pattern="test_chamber_config.py",
    ))
    sys.exit(0 if result.wasSuccessful() else 1)
