"""Unit tests for GrowthLogger — focused on save_frame's Jul 8 2026 contract
change (LOG ENTRY saves unconditionally, returns quality metadata).

Small test surface; no Qt required (GrowthLogger is a plain Python class).
Run: ``python scripts/test_growth_logger.py`` or under pytest.
"""
from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from gui.growth_logger import GrowthLogger  # noqa: E402


def _good_frame() -> np.ndarray:
    """A frame with mean intensity comfortably above the quality gate's
    3.0 threshold — passes cleanly."""
    return (np.random.default_rng(42).integers(80, 180, (200, 300, 3))
            .astype(np.uint8))


def _dark_frame() -> np.ndarray:
    """A frame with mean intensity ~1 — well below the 3.0 threshold,
    should be flagged but still saved per Jul 8 decision."""
    return (np.random.default_rng(0).integers(0, 3, (200, 300, 3))
            .astype(np.uint8))


class SaveFrameTests(unittest.TestCase):
    """Tests the Jul 8 2026 contract change: LOG ENTRY saves the frame
    unconditionally when available, quality gate result is captured as
    metadata rather than a save-blocker."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.logger = GrowthLogger(base_dir=self.tmp.name)
        self.logger.start_session("TEST_SAVE_FRAME")

    def tearDown(self):
        self.logger.end_session()
        self.tmp.cleanup()

    def test_good_frame_saves_and_reports_quality_pass_true(self):
        path, quality_pass = self.logger.save_frame(_good_frame(), "160955")
        self.assertNotEqual(path, "")
        self.assertTrue(Path(path).exists())
        self.assertEqual(quality_pass, True)

    def test_dark_frame_still_saves_and_reports_quality_pass_false(self):
        # Critical contract test: grower explicit intent > automated
        # quality gate. Frame gets saved despite failing the gate.
        path, quality_pass = self.logger.save_frame(_dark_frame(), "160955")
        self.assertNotEqual(path, "", "dark frame should still be saved")
        self.assertTrue(Path(path).exists())
        self.assertEqual(quality_pass, False)

    def test_no_session_returns_empty_path_and_none_quality(self):
        logger = GrowthLogger(base_dir=self.tmp.name)
        # No start_session — session_dir is None.
        path, quality_pass = logger.save_frame(_good_frame(), "160955")
        self.assertEqual(path, "")
        self.assertIsNone(quality_pass)

    def test_frame_counter_advances_regardless_of_quality(self):
        # Every LOG ENTRY should get a distinct frame path even when
        # the previous frame was flagged. Confirms that the counter
        # advances on save success, not on quality pass.
        p1, _ = self.logger.save_frame(_dark_frame(), "160955")
        p2, _ = self.logger.save_frame(_good_frame(), "160956")
        p3, _ = self.logger.save_frame(_dark_frame(), "160957")
        self.assertNotEqual(p1, p2)
        self.assertNotEqual(p2, p3)
        # Filenames should be entry_001, entry_002, entry_003 in order.
        self.assertIn("entry_001", p1)
        self.assertIn("entry_002", p2)
        self.assertIn("entry_003", p3)


class CommitFieldsTests(unittest.TestCase):
    """Schema-level tests for the two Jul 8 additions."""

    def test_classifier_status_and_frame_quality_pass_are_present(self):
        # These two columns landed Jul 8 — verify they're in the schema
        # so downstream code can rely on them.
        self.assertIn("classifier_status", GrowthLogger.COMMIT_FIELDS)
        self.assertIn("frame_quality_pass", GrowthLogger.COMMIT_FIELDS)
        self.assertIn("psu_source", GrowthLogger.COMMIT_FIELDS)

    def test_field_order_stable(self):
        # frame_quality_pass should follow frame_path (both are per-frame
        # metadata). Grouping matters for CSV readability.
        fields = GrowthLogger.COMMIT_FIELDS
        i_path = fields.index("frame_path")
        i_qual = fields.index("frame_quality_pass")
        self.assertGreater(i_qual, i_path)


if __name__ == "__main__":
    unittest.main(verbosity=2)
