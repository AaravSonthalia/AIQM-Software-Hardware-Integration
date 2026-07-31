"""Unit tests for GrowthLogger — focused on save_frame's Jul 8 2026 contract
change (LOG ENTRY saves unconditionally, returns quality metadata) plus the
Jul 10 2026 grower-marked events schema (record_manual_event).

Small test surface; no Qt required (GrowthLogger is a plain Python class).
Run: ``python scripts/test_growth_logger.py`` or under pytest.
"""
from __future__ import annotations

import csv
import json
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from gui.growth_logger import (  # noqa: E402
    GrowthLogger,
    RHEED_VIEW_EVENT_ALIGNMENT_CONFIRMED,
    RHEED_VIEW_EVENT_HISTORY_RESET,
    RHEED_VIEW_EVENT_HISTORY_READY,
    RHEED_VIEW_EVENT_QC_PASS,
    RHEED_VIEW_EVENT_QC_REJECT,
    RHEED_VIEW_EVENT_REALIGN_END,
    RHEED_VIEW_EVENT_REALIGN_START,
    RHEED_VIEW_EVENT_SESSION_START,
    RHEED_VIEW_EVENT_TYPES,
)


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
        # Jul 9 switch: entry frames are .bmp, not .png.
        self.assertTrue(
            path.endswith(".bmp"),
            f"expected .bmp extension, got {path}",
        )

    def test_dark_frame_still_saves_and_reports_quality_pass_false(self):
        # Critical contract test: grower explicit intent > automated
        # quality gate. Frame gets saved despite failing the gate.
        path, quality_pass = self.logger.save_frame(_dark_frame(), "160955")
        self.assertNotEqual(path, "", "dark frame should still be saved")
        self.assertTrue(Path(path).exists())
        self.assertEqual(quality_pass, False)
        self.assertTrue(path.endswith(".bmp"))

    def test_saved_bmp_is_valid_bmp_format(self):
        # PIL should be able to read the file back as a BMP.
        path, _ = self.logger.save_frame(_good_frame(), "160955")
        from PIL import Image
        with Image.open(path) as img:
            self.assertEqual(
                img.format, "BMP",
                f"expected BMP format, PIL reports {img.format}",
            )

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
        self.assertIn(
            "classifier_prediction_actionable",
            GrowthLogger.COMMIT_FIELDS,
        )
        self.assertIn(
            "classifier_visual_history_generation",
            GrowthLogger.COMMIT_FIELDS,
        )
        self.assertIn("frame_quality_pass", GrowthLogger.COMMIT_FIELDS)
        self.assertIn("psu_source", GrowthLogger.COMMIT_FIELDS)

    def test_field_order_stable(self):
        # frame_quality_pass should follow frame_path (both are per-frame
        # metadata). Grouping matters for CSV readability.
        fields = GrowthLogger.COMMIT_FIELDS
        i_path = fields.index("frame_path")
        i_qual = fields.index("frame_quality_pass")
        self.assertGreater(i_qual, i_path)


class CaptureProvenanceTests(unittest.TestCase):
    """RHEED image rows preserve the capture timestamp and source atomically."""

    def test_heartbeat_writes_capture_metadata(self):
        with tempfile.TemporaryDirectory() as tmp:
            logger = GrowthLogger(base_dir=tmp)
            logger.start_session("CAPTURE_META")
            metadata = {
                "capture_backend": "wgc",
                "captured_at_utc": "2026-07-27T12:00:00.123Z",
                "capture_sequence": 42,
                "frame_age_ms": 12.3456,
                "source_hwnd": 9001,
            }
            logger.log_heartbeat(
                elapsed_s=1.0,
                frame_path="frames/heartbeat_001.bmp",
                capture_metadata=metadata,
            )
            csv_path = logger.session_dir / "heartbeat_log.csv"
            logger.end_session()
            with open(csv_path, newline="") as stream:
                row = next(csv.DictReader(stream))
            self.assertEqual(row["capture_backend"], "wgc")
            self.assertEqual(
                row["captured_at_utc"], "2026-07-27T12:00:00.123Z",
            )
            self.assertEqual(row["capture_sequence"], "42")
            self.assertEqual(row["frame_age_ms"], "12.346")
            self.assertEqual(row["source_hwnd"], "9001")

    def test_auto_capture_buffer_writes_per_frame_manifest(self):
        with tempfile.TemporaryDirectory() as tmp:
            logger = GrowthLogger(base_dir=tmp)
            logger.start_session("BUFFER_META")
            metadata = [
                {
                    "capture_backend": "wgc",
                    "captured_at_utc": f"2026-07-27T12:00:0{i}.000Z",
                    "capture_sequence": 100 + i,
                    "frame_age_ms": 10.0 + i,
                    "source_hwnd": 9001,
                }
                for i in range(2)
            ]
            count, rel_dir = logger.save_auto_capture_buffer(
                event_idx=1,
                frames=[_good_frame(), _good_frame()],
                capture_metadata=metadata,
            )
            event_dir = logger.session_dir / rel_dir
            logger.end_session()

            self.assertEqual(count, 2)
            with open(event_dir / "capture_manifest.csv", newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 2)
            self.assertEqual(
                [row["capture_sequence"] for row in rows],
                ["100", "101"],
            )
            for row in rows:
                self.assertTrue((event_dir / row["frame_path"]).exists())
                self.assertEqual(row["capture_backend"], "wgc")
                self.assertEqual(row["source_hwnd"], "9001")


class UpdateEventLabelPartialUpdateTests(unittest.TestCase):
    """Jul 15 2026 additions: change_from and change_to are UI-populated
    via events_tab dropdowns. update_event_label's None-preserving
    contract must not clobber sibling columns when only one of these
    kwargs is passed. Guards against a subtle bug where the two new
    columns' UI-populated status accidentally makes them "always
    written" rather than "written when explicitly passed".
    """

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.logger = GrowthLogger(base_dir=self.tmp.name)
        self.logger.start_session("TEST_UPDATE_LABEL")

    def tearDown(self):
        try:
            self.logger.end_session()
        except Exception:
            pass
        self.tmp.cleanup()

    def _read_row_for_event(self, event_idx: int) -> dict:
        csv_path = self.logger.session_dir / "events_labels.csv"
        with open(csv_path) as f:
            for row in csv.DictReader(f):
                if row.get("event_idx") == str(event_idx):
                    return row
        return {}

    def test_update_event_label_change_from_only_preserves_other_fields(self):
        # Set primary + notes, then update change_from alone. Expect the
        # earlier fields to survive the partial update.
        self.logger.update_event_label(
            event_idx=1,
            primary_reconstruction="c(6x2)",
            notes="test note",
        )
        self.logger.update_event_label(event_idx=1, change_from="1x1")

        row = self._read_row_for_event(1)
        self.assertEqual(row["primary_reconstruction"], "c(6x2)")
        self.assertEqual(row["notes"], "test note")
        self.assertEqual(row["change_from"], "1x1")
        # change_to still blank — not touched by either call.
        self.assertEqual(row["change_to"], "")

    def test_update_event_label_change_to_only_preserves_other_fields(self):
        # Mirror of above for change_to.
        self.logger.update_event_label(
            event_idx=2,
            primary_reconstruction="rt13xrt13",
            notes="another note",
        )
        self.logger.update_event_label(event_idx=2, change_to="HTR")

        row = self._read_row_for_event(2)
        self.assertEqual(row["primary_reconstruction"], "rt13xrt13")
        self.assertEqual(row["notes"], "another note")
        self.assertEqual(row["change_to"], "HTR")
        self.assertEqual(row["change_from"], "")

    def test_update_change_from_and_to_together(self):
        # Common case: grower sets both dropdowns as consecutive
        # dropdown activations. Two separate update calls, one per
        # dropdown, must both land.
        self.logger.update_event_label(event_idx=3, change_from="1x1")
        self.logger.update_event_label(event_idx=3, change_to="Twinned (2x1)")

        row = self._read_row_for_event(3)
        self.assertEqual(row["change_from"], "1x1")
        self.assertEqual(row["change_to"], "Twinned (2x1)")


class ManualEventSchemaTests(unittest.TestCase):
    """Schema-level tests for MANUAL_EVENT_FIELDS (Jul 10 2026 addition).

    The scrubber timeline (workstream #3) will read manual_events.csv, so
    every field name is load-bearing. These tests lock the contract.
    """

    def test_schema_contains_expected_columns(self):
        expected = {
            "timestamp", "elapsed_s", "event_idx",
            "pyrometer_temp_C",
            "voltage_V", "current_A", "psu_source",
            "frame_path", "note",
            "capture_backend", "captured_at_utc", "capture_sequence",
            "frame_age_ms", "source_hwnd",
        }
        self.assertEqual(set(GrowthLogger.MANUAL_EVENT_FIELDS), expected)

    def test_timestamp_first_event_idx_third(self):
        # Ordering convention: timestamp is column 1 (mirrors other logs),
        # event_idx is column 3 (after elapsed_s). Downstream sort/plot
        # code relies on this ordering.
        self.assertEqual(GrowthLogger.MANUAL_EVENT_FIELDS[0], "timestamp")
        self.assertEqual(GrowthLogger.MANUAL_EVENT_FIELDS[1], "elapsed_s")
        self.assertEqual(GrowthLogger.MANUAL_EVENT_FIELDS[2], "event_idx")


class ManualEventLifecycleTests(unittest.TestCase):
    """File-lifecycle tests: manual_events.csv opens on start_session,
    closes on end_session, and has the correct header."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.logger = GrowthLogger(base_dir=self.tmp.name)

    def tearDown(self):
        try:
            self.logger.end_session()
        except Exception:
            pass
        self.tmp.cleanup()

    def test_manual_events_csv_created_on_start_session(self):
        self.logger.start_session("TEST_MANUAL_EVENTS")
        csv_path = self.logger.session_dir / "manual_events.csv"
        self.assertTrue(csv_path.exists())

    def test_manual_events_csv_has_correct_header(self):
        self.logger.start_session("TEST_HEADER")
        csv_path = self.logger.session_dir / "manual_events.csv"
        with open(csv_path, "r", newline="") as f:
            header = next(csv.reader(f))
        self.assertEqual(header, GrowthLogger.MANUAL_EVENT_FIELDS)

    def test_counter_resets_on_new_session(self):
        # Log an event, end the session, start a new one — counter should
        # be 0 again so the next event is idx=1.
        self.logger.start_session("TEST_A")
        idx_a = self.logger.record_manual_event(elapsed_s=1.0)
        self.assertEqual(idx_a, 1)
        self.logger.end_session()

        self.logger.start_session("TEST_B")
        idx_b = self.logger.record_manual_event(elapsed_s=2.0)
        # Fresh session → fresh counter starting at 1, not 2.
        self.assertEqual(idx_b, 1)

    def test_file_closed_on_end_session(self):
        self.logger.start_session("TEST_CLOSE")
        self.logger.end_session()
        # Writer refs cleared so a stray record_manual_event no-ops safely.
        self.assertIsNone(self.logger._manual_event_writer)

    def test_no_session_returns_zero(self):
        # No start_session — record must silently no-op with idx 0.
        idx = self.logger.record_manual_event(elapsed_s=10.0)
        self.assertEqual(idx, 0)


def _dummy_frame() -> np.ndarray:
    """A modest-intensity 100x100x3 frame that PIL / cv2 can encode as BMP."""
    return (np.random.default_rng(7).integers(40, 200, (100, 100, 3))
            .astype(np.uint8))


class ManualEventRecordTests(unittest.TestCase):
    """End-to-end tests for record_manual_event's row + frame behavior."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.logger = GrowthLogger(base_dir=self.tmp.name)
        self.logger.start_session("TEST_RECORD")
        self.csv_path = self.logger.session_dir / "manual_events.csv"

    def tearDown(self):
        try:
            self.logger.end_session()
        except Exception:
            pass
        self.tmp.cleanup()

    def _read_rows(self) -> list[dict]:
        with open(self.csv_path, "r", newline="") as f:
            return list(csv.DictReader(f))

    def test_bare_click_writes_one_row_with_index_1(self):
        # Simulates a grower click with no note, no frame, no PSU state.
        idx = self.logger.record_manual_event(elapsed_s=42.0)
        self.assertEqual(idx, 1)

        rows = self._read_rows()
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["event_idx"], "1")
        self.assertEqual(rows[0]["elapsed_s"], "42.00")
        # Optional fields blank when None is passed.
        self.assertEqual(rows[0]["pyrometer_temp_C"], "")
        self.assertEqual(rows[0]["voltage_V"], "")
        self.assertEqual(rows[0]["current_A"], "")
        self.assertEqual(rows[0]["psu_source"], "none")
        self.assertEqual(rows[0]["frame_path"], "")
        self.assertEqual(rows[0]["note"], "")

    def test_full_payload_populates_all_columns(self):
        idx = self.logger.record_manual_event(
            elapsed_s=123.45,
            pyro_temp=650.7,
            voltage_V=12.345,
            current_A=0.678,
            psu_source="mistral",
            note="rt13 blooming",
        )
        self.assertEqual(idx, 1)
        rows = self._read_rows()
        self.assertEqual(rows[0]["elapsed_s"], "123.45")
        self.assertEqual(rows[0]["pyrometer_temp_C"], "650.7")
        self.assertEqual(rows[0]["voltage_V"], "12.345")
        self.assertEqual(rows[0]["current_A"], "0.678")
        self.assertEqual(rows[0]["psu_source"], "mistral")
        self.assertEqual(rows[0]["note"], "rt13 blooming")

    def test_counter_increments_monotonically_across_calls(self):
        # Multi-click safety: consecutive fast clicks all land as
        # distinct events with monotonically increasing indices.
        idxs = [
            self.logger.record_manual_event(elapsed_s=float(i))
            for i in range(5)
        ]
        self.assertEqual(idxs, [1, 2, 3, 4, 5])
        rows = self._read_rows()
        self.assertEqual(len(rows), 5)
        # Row order matches call order (append semantics).
        self.assertEqual(
            [r["event_idx"] for r in rows],
            ["1", "2", "3", "4", "5"],
        )

    def test_frame_is_saved_when_provided(self):
        # A supplied frame should be written to frames/ as
        # manual_event_NNN_HHMMSS.bmp; the row's frame_path points to it.
        frame = _dummy_frame()
        idx = self.logger.record_manual_event(elapsed_s=5.0, frame=frame)
        self.assertEqual(idx, 1)

        rows = self._read_rows()
        frame_path = rows[0]["frame_path"]
        self.assertNotEqual(frame_path, "")
        self.assertTrue(Path(frame_path).exists())
        self.assertTrue(
            Path(frame_path).name.startswith("manual_event_001_"),
            f"expected manual_event_001_ prefix, got {Path(frame_path).name}",
        )
        self.assertTrue(frame_path.endswith(".bmp"))

    def test_frame_path_empty_when_no_frame_passed(self):
        # A bare click still records the event but leaves frame_path
        # blank so the scrubber can render "mark-only" tick marks.
        idx = self.logger.record_manual_event(elapsed_s=5.0, frame=None)
        self.assertEqual(idx, 1)
        rows = self._read_rows()
        self.assertEqual(rows[0]["frame_path"], "")

    def test_frame_files_are_distinct_across_events(self):
        # Frame filename includes the event_idx so consecutive marks
        # don't collide even at the same HH:MM:SS second.
        frame = _dummy_frame()
        self.logger.record_manual_event(elapsed_s=1.0, frame=frame)
        self.logger.record_manual_event(elapsed_s=1.1, frame=frame)
        rows = self._read_rows()
        p1 = Path(rows[0]["frame_path"]).name
        p2 = Path(rows[1]["frame_path"]).name
        self.assertNotEqual(p1, p2)
        self.assertIn("manual_event_001_", p1)
        self.assertIn("manual_event_002_", p2)

    def test_saved_bmp_is_valid_bmp_format(self):
        frame = _dummy_frame()
        self.logger.record_manual_event(elapsed_s=1.0, frame=frame)
        rows = self._read_rows()
        from PIL import Image
        with Image.open(rows[0]["frame_path"]) as img:
            self.assertEqual(img.format, "BMP")


class RheedViewEventSchemaTests(unittest.TestCase):
    """The structured acquisition/QC log has a stable, separate schema."""

    def test_schema_is_exact_and_manual_schema_is_unchanged(self):
        expected = [
            "timestamp", "elapsed_s", "event_idx", "event_type",
            "realignment_id",
            "previous_view_segment_id", "view_segment_id",
            "visual_history_generation",
            "gun_aligned",
            "history_frame_count", "history_required", "history_ready",
            "qc_reject", "qc_reason", "labeler", "confidence",
            "prediction_actionable",
            "frame_role", "frame_path", "note",
            "capture_backend", "captured_at_utc", "capture_sequence",
            "frame_age_ms", "source_hwnd",
        ]
        self.assertEqual(GrowthLogger.RHEED_VIEW_EVENT_FIELDS, expected)
        self.assertNotIn("event_type", GrowthLogger.MANUAL_EVENT_FIELDS)
        self.assertNotIn("qc_reject", GrowthLogger.MANUAL_EVENT_FIELDS)

    def test_event_type_vocabulary_is_explicit(self):
        self.assertEqual(
            RHEED_VIEW_EVENT_TYPES,
            {
                RHEED_VIEW_EVENT_SESSION_START,
                RHEED_VIEW_EVENT_ALIGNMENT_CONFIRMED,
                RHEED_VIEW_EVENT_REALIGN_START,
                RHEED_VIEW_EVENT_REALIGN_END,
                RHEED_VIEW_EVENT_HISTORY_RESET,
                RHEED_VIEW_EVENT_HISTORY_READY,
                RHEED_VIEW_EVENT_QC_REJECT,
                RHEED_VIEW_EVENT_QC_PASS,
            },
        )


class RheedViewEventLifecycleTests(unittest.TestCase):
    """File creation, counter reset, metadata count, and close behavior."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.logger = GrowthLogger(base_dir=self.tmp.name)

    def tearDown(self):
        try:
            self.logger.end_session()
        except Exception:
            pass
        self.tmp.cleanup()

    def test_file_is_created_with_flushed_header(self):
        self.logger.start_session("RHEED_VIEW_HEADER")
        path = self.logger.session_dir / "rheed_view_events.csv"
        self.assertTrue(path.exists())
        with open(path, newline="") as stream:
            self.assertEqual(
                next(csv.reader(stream)),
                GrowthLogger.RHEED_VIEW_EVENT_FIELDS,
            )

    def test_counter_is_monotonic_and_resets_for_new_session(self):
        self.logger.start_session("RHEED_VIEW_A")
        self.assertEqual(
            self.logger.record_rheed_view_event(
                RHEED_VIEW_EVENT_SESSION_START, 0.0,
            ),
            1,
        )
        self.assertEqual(
            self.logger.record_rheed_view_event(
                RHEED_VIEW_EVENT_REALIGN_START, 1.0,
            ),
            2,
        )
        self.logger.end_session()

        self.logger.start_session("RHEED_VIEW_B")
        self.assertEqual(
            self.logger.record_rheed_view_event(
                RHEED_VIEW_EVENT_SESSION_START, 0.0,
            ),
            1,
        )

    def test_valid_event_without_session_returns_zero(self):
        self.assertEqual(
            self.logger.record_rheed_view_event(
                RHEED_VIEW_EVENT_QC_REJECT, 1.0,
            ),
            0,
        )

    def test_end_session_closes_and_clears_writer(self):
        self.logger.start_session("RHEED_VIEW_CLOSE")
        event_file = self.logger._rheed_view_event_file
        self.logger.end_session()
        self.assertTrue(event_file.closed)
        self.assertIsNone(self.logger._rheed_view_event_file)
        self.assertIsNone(self.logger._rheed_view_event_writer)

    def test_session_metadata_contains_event_count(self):
        self.logger.start_session("RHEED_VIEW_METADATA")
        for event_type in (
            RHEED_VIEW_EVENT_SESSION_START,
            RHEED_VIEW_EVENT_REALIGN_START,
            RHEED_VIEW_EVENT_REALIGN_END,
        ):
            self.logger.record_rheed_view_event(event_type, 1.0)
        self.logger.save_session_metadata({})
        metadata_path = self.logger.session_dir / "session_metadata.json"
        with open(metadata_path) as stream:
            metadata = json.load(stream)
        self.assertEqual(metadata["rheed_view_event_count"], 3)


class RheedViewEventRecordTests(unittest.TestCase):
    """Rows preserve state, frame evidence, and camera provenance."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.logger = GrowthLogger(base_dir=self.tmp.name)
        self.logger.start_session("RHEED_VIEW_RECORD")
        self.csv_path = self.logger.session_dir / "rheed_view_events.csv"

    def tearDown(self):
        try:
            self.logger.end_session()
        except Exception:
            pass
        self.tmp.cleanup()

    def _read_rows(self) -> list[dict]:
        with open(self.csv_path, newline="") as stream:
            return list(csv.DictReader(stream))

    def test_full_state_snapshot_and_capture_provenance_are_written(self):
        state = {
            "view_segment_id": 4,
            "gun_aligned": True,
            "history_frame_count": 0,
            "history_required": 32,
            "history_ready": False,
            "qc_reject": False,
            "qc_reason": "",
            "prediction_actionable": False,
        }
        idx = self.logger.record_rheed_view_event(
            RHEED_VIEW_EVENT_REALIGN_END,
            123.456,
            state_snapshot=state,
            realignment_id=7,
            previous_view_segment_id=3,
            frame_role="post_realign",
            note="new gun direction locked",
            capture_metadata={
                "capture_backend": "wgc",
                "captured_at_utc": "2026-07-29T20:00:00.123Z",
                "capture_sequence": 99,
                "frame_age_ms": 4.5678,
                "source_hwnd": 4321,
            },
        )
        self.assertEqual(idx, 1)
        row = self._read_rows()[0]
        self.assertEqual(row["elapsed_s"], "123.46")
        self.assertEqual(row["event_type"], RHEED_VIEW_EVENT_REALIGN_END)
        self.assertEqual(row["realignment_id"], "7")
        self.assertEqual(row["previous_view_segment_id"], "3")
        self.assertEqual(row["view_segment_id"], "4")
        self.assertEqual(row["gun_aligned"], "True")
        self.assertEqual(row["history_frame_count"], "0")
        self.assertEqual(row["history_required"], "32")
        self.assertEqual(row["history_ready"], "False")
        self.assertEqual(row["qc_reject"], "False")
        self.assertEqual(row["prediction_actionable"], "False")
        self.assertEqual(row["frame_role"], "post_realign")
        self.assertEqual(row["note"], "new gun direction locked")
        self.assertEqual(row["capture_backend"], "wgc")
        self.assertEqual(
            row["captured_at_utc"], "2026-07-29T20:00:00.123Z",
        )
        self.assertEqual(row["capture_sequence"], "99")
        self.assertEqual(row["frame_age_ms"], "4.568")
        self.assertEqual(row["source_hwnd"], "4321")

    def test_ids_may_come_from_state_snapshot(self):
        self.logger.record_rheed_view_event(
            RHEED_VIEW_EVENT_REALIGN_START,
            5.0,
            state_snapshot={
                "realignment_id": 8,
                "previous_view_segment_id": 4,
                "view_segment_id": None,
                "gun_aligned": False,
                "qc_reject": True,
                "qc_reason": "gun_realigning",
            },
        )
        row = self._read_rows()[0]
        self.assertEqual(row["realignment_id"], "8")
        self.assertEqual(row["previous_view_segment_id"], "4")
        self.assertEqual(row["view_segment_id"], "")
        self.assertEqual(row["gun_aligned"], "False")
        self.assertEqual(row["qc_reject"], "True")
        self.assertEqual(row["qc_reason"], "gun_realigning")

    def test_event_frame_is_saved_and_bound_to_row(self):
        idx = self.logger.record_rheed_view_event(
            RHEED_VIEW_EVENT_QC_REJECT,
            3.0,
            frame=_dummy_frame(),
            frame_role="qc_reject",
        )
        self.assertEqual(idx, 1)
        row = self._read_rows()[0]
        path = Path(row["frame_path"])
        self.assertTrue(path.exists())
        self.assertIn(
            "rheed_view_event_001_qc_reject_", path.name,
        )
        self.assertEqual(path.suffix.lower(), ".bmp")
        from PIL import Image
        with Image.open(path) as image:
            self.assertEqual(image.format, "BMP")

    def test_invalid_event_type_raises_without_mutating_counter_or_file(self):
        with self.assertRaisesRegex(ValueError, "Unsupported RHEED view"):
            self.logger.record_rheed_view_event("free_form_typo", 1.0)
        self.assertEqual(self.logger._rheed_view_event_counter, 0)
        self.assertEqual(self._read_rows(), [])

    def test_non_mapping_state_snapshot_is_rejected(self):
        with self.assertRaisesRegex(TypeError, "must be a mapping"):
            self.logger.record_rheed_view_event(
                RHEED_VIEW_EVENT_HISTORY_READY,
                1.0,
                state_snapshot=["not", "a", "mapping"],
            )
        self.assertEqual(self.logger._rheed_view_event_counter, 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
