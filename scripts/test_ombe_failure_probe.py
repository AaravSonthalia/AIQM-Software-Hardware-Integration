"""Platform-independent tests for the O-MBE failure recorder analysis."""

from __future__ import annotations

import hashlib
import json
import sys
import tempfile
import unittest
from argparse import Namespace
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import scripts.ombe_failure_probe as probe_module
from scripts.ombe_failure_probe import (
    ObservationTracker,
    annotate_observation,
    data_age,
    driver_identity,
    mark_event,
    measurement_signature,
    measurement_status,
    prepare_output_dir,
    run_probe,
    state_payload,
    summarize,
    validate_source_mode,
)


@dataclass
class _State:
    temperature: float = 500.0
    temperature_std: float = 1.0
    temperature_n: int = 5
    emissivity: float = 0.9
    connected: bool = True
    error: str = ""


def _pyrometer_row(index: int, t: float, *, error: str = ""):
    return {
        "emission_index": index,
        "observed_at_utc": f"2026-07-27T00:00:{int(t):02d}.000Z",
        "observed_monotonic_s": t,
        "observed_monotonic_ns": round(t * 1_000_000_000),
        "elapsed_s": t,
        "state": {
            "temperature": 500.0,
            "temperature_std": 1.0,
            "temperature_n": 5,
            "emissivity": 0.9,
            "connected": True,
            "error": error,
        },
    }


class SerializationTests(unittest.TestCase):
    def test_dataclass_serialization(self):
        payload = state_payload(_State())
        self.assertEqual(payload["temperature"], 500.0)
        self.assertTrue(payload["connected"])

    def test_signature_uses_all_source_measurements(self):
        payload = state_payload(_State())
        self.assertEqual(
            measurement_signature("pyrometer", payload),
            (500.0, 1.0, 5, 0.9),
        )

    def test_camera_signature_prefers_capture_sequence(self):
        first = {
            "frame_number": 100,
            "capture_sequence": 42,
        }
        second = {
            "frame_number": 101,
            "capture_sequence": 42,
        }
        self.assertEqual(
            measurement_signature("camera", first),
            measurement_signature("camera", second),
        )

    def test_capture_timestamp_produces_age(self):
        age_ms, source = data_age(
            {"captured_at_utc": "2026-07-27T00:00:00.000Z"},
            "2026-07-27T00:00:00.500Z",
            0,
        )
        self.assertEqual(age_ms, 500.0)
        self.assertEqual(source, "captured_at_utc")

    def test_monotonic_receipt_timestamp_produces_age(self):
        age_ms, source = data_age(
            {"received_monotonic_ns": 1_000_000_000},
            "",
            1_500_000_000,
        )
        self.assertEqual(age_ms, 500.0)
        self.assertEqual(source, "received_monotonic_ns")


class ModeValidationTests(unittest.TestCase):
    def test_valid_source_mode_is_accepted(self):
        validate_source_mode("evap", "elog")
        validate_source_mode("camera", "screengrab")

    def test_typo_and_dummy_modes_are_rejected(self):
        with self.assertRaises(ValueError):
            validate_source_mode("mistral", "screenrab")
        with self.assertRaises(ValueError):
            validate_source_mode("camera", "dummy")

    def test_driver_identity_reports_actual_class_and_backend(self):
        class ScreenGrabCamera:
            _backend = "wgc"
            _capture_session = None

        class Worker:
            _camera = ScreenGrabCamera()

        identity = driver_identity(Worker(), "camera")
        self.assertEqual(identity["driver_class"], "ScreenGrabCamera")
        self.assertEqual(identity["driver_backend"], "wgc")


class ValidityTests(unittest.TestCase):
    def test_connected_all_none_ocr_state_is_invalid(self):
        payload = {
            "v_set": None,
            "v_actual": None,
            "i_set": None,
            "i_actual": None,
            "connected": True,
            "error": "",
        }
        valid, reason, all_none = measurement_status("mistral", payload)
        self.assertFalse(valid)
        self.assertTrue(all_none)
        self.assertEqual(reason, "all_measurements_none")

    def test_annotation_records_value_change_and_unchanged_age(self):
        tracker = ObservationTracker()
        first = annotate_observation(_pyrometer_row(0, 0.0), "pyrometer", tracker)
        second = annotate_observation(_pyrometer_row(1, 4.0), "pyrometer", tracker)
        self.assertIsNone(first["value_changed"])
        self.assertFalse(second["value_changed"])
        self.assertEqual(second["unchanged_for_s"], 4.0)
        self.assertTrue(second["measurement_valid"])


class SummaryTests(unittest.TestCase):
    def test_stable_value_without_failure_marker_is_not_stale(self):
        rows = [_pyrometer_row(i, float(i)) for i in range(8)]
        summary = summarize(rows, "pyrometer", baseline_duration_s=4.0)
        self.assertEqual(summary["stale_candidate_count"], 0)
        self.assertEqual(summary["stale_threshold_s"], 3.0)
        self.assertEqual(
            summary["baseline_interval_s"]["source"],
            "healthy_pre_failure_window",
        )

    def test_marker_aligned_retained_valid_value_is_flagged(self):
        rows = [_pyrometer_row(i, float(i)) for i in range(7)]
        markers = [{
            "kind": "failure-start",
            "label": "stop-update",
            "marked_monotonic_s": 2.1,
            "marked_at_utc": "2026-07-27T00:00:02.100Z",
        }]
        summary = summarize(
            rows,
            "pyrometer",
            markers=markers,
            baseline_duration_s=10.0,
        )
        self.assertEqual(summary["baseline_interval_s"]["p95"], 1.0)
        self.assertEqual(summary["stale_candidate_emission_indices"], [6])
        self.assertEqual(summary["failure_marker_count"], 1)

    def test_failure_intervals_do_not_inflate_baseline_p95(self):
        rows = [
            _pyrometer_row(0, 0.0),
            _pyrometer_row(1, 1.0),
            _pyrometer_row(2, 10.0),
            _pyrometer_row(3, 20.0),
        ]
        markers = [{
            "kind": "failure-start",
            "label": "close-window",
            "marked_monotonic_s": 1.5,
        }]
        summary = summarize(
            rows,
            "pyrometer",
            markers=markers,
            baseline_duration_s=30.0,
        )
        self.assertEqual(summary["baseline_interval_s"]["samples"], 1)
        self.assertEqual(summary["baseline_interval_s"]["p95"], 1.0)
        self.assertEqual(summary["stale_threshold_s"], 3.0)

    def test_supplied_baseline_p95_controls_threshold(self):
        rows = [_pyrometer_row(i, float(i)) for i in range(3)]
        summary = summarize(rows, "pyrometer", baseline_p95_s=4.0)
        self.assertEqual(summary["baseline_interval_s"]["source"], "supplied")
        self.assertEqual(summary["stale_threshold_s"], 8.0)

    def test_all_none_failure_has_detection_latency_without_worker_error(self):
        rows = [{
            "emission_index": 0,
            "observed_at_utc": "2026-07-27T00:00:00.000Z",
            "observed_monotonic_s": 0.0,
            "elapsed_s": 0.0,
            "state": {
                "v_set": 1.0,
                "v_actual": 1.0,
                "i_set": 1.0,
                "i_actual": 1.0,
                "connected": True,
                "error": "",
            },
        }, {
            "emission_index": 1,
            "observed_at_utc": "2026-07-27T00:00:01.000Z",
            "observed_monotonic_s": 1.0,
            "elapsed_s": 1.0,
            "state": {
                "v_set": None,
                "v_actual": None,
                "i_set": None,
                "i_actual": None,
                "connected": True,
                "error": "",
            },
        }]
        markers = [{
            "kind": "failure-start",
            "label": "cover",
            "marked_monotonic_s": 0.5,
        }]
        summary = summarize(rows, "mistral", markers=markers)
        self.assertEqual(summary["connected_but_invalid_emissions"], 1)
        self.assertEqual(summary["all_measurements_none_emissions"], 1)
        self.assertEqual(
            summary["failure_episodes"][0]["invalid_detection_latency_s"],
            0.5,
        )


class EvidenceTests(unittest.TestCase):
    def test_output_directory_is_collision_safe(self):
        with tempfile.TemporaryDirectory() as tmp:
            target = Path(tmp) / "run"
            prepare_output_dir(target)
            with self.assertRaises(FileExistsError):
                prepare_output_dir(target)

    def test_marker_is_hashed_after_completed_run(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            (directory / "run_info.json").write_text("{}", encoding="utf-8")
            (directory / "states.jsonl").write_text("", encoding="utf-8")
            (directory / "summary.json").write_text("{}", encoding="utf-8")
            args = type("Args", (), {
                "output_dir": directory,
                "kind": "failure-start",
                "label": "cover",
                "note": "manual action",
            })()
            self.assertEqual(mark_event(args), 0)
            marker_path = directory / "markers.jsonl"
            marker = json.loads(marker_path.read_text(encoding="utf-8"))
            self.assertEqual(marker["kind"], "failure-start")
            manifest = json.loads(
                (directory / "sha256_manifest.json").read_text(
                    encoding="utf-8",
                )
            )
            expected = hashlib.sha256(marker_path.read_bytes()).hexdigest()
            self.assertEqual(manifest["markers.jsonl"], expected)

    def test_marker_rejects_unrelated_directory(self):
        with tempfile.TemporaryDirectory() as tmp:
            args = type("Args", (), {
                "output_dir": Path(tmp),
                "kind": "action",
                "label": "cover",
                "note": "",
            })()
            with self.assertRaises(FileNotFoundError):
                mark_event(args)


class RunLifecycleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PyQt6.QtCore import QCoreApplication

        cls._app = QCoreApplication.instance() or QCoreApplication([])

    @staticmethod
    def _args(output_dir: Path) -> Namespace:
        return Namespace(
            source="mistral",
            mode="screengrab",
            duration_s=0.01,
            poll_interval=1.0,
            baseline_duration_s=1.0,
            baseline_p95_s=None,
            software_root=Path.cwd(),
            expected_capture_backend=None,
            output_dir=output_dir,
        )

    def test_stop_timeout_is_nonzero_and_evidence_is_finalized(self):
        from PyQt6.QtCore import QObject, QTimer, pyqtSignal

        class MistralGui:
            pass

        class FakeWorker(QObject):
            state_updated = pyqtSignal(object)
            finished = pyqtSignal()

            def __init__(self):
                super().__init__()
                self._driver = MistralGui()

            def start(self):
                state = SimpleNamespace(
                    v_set=1.0,
                    v_actual=1.0,
                    i_set=1.0,
                    i_actual=1.0,
                    connected=True,
                    error="",
                    mode="screengrab",
                )
                QTimer.singleShot(0, lambda: self.state_updated.emit(state))

            def stop(self):
                pass

            def wait(self, _timeout_ms):
                return False

            def isRunning(self):
                return True

        with tempfile.TemporaryDirectory() as tmp:
            output_dir = Path(tmp) / "run"
            with patch.object(
                probe_module,
                "_worker_factory",
                return_value=FakeWorker(),
            ):
                result = run_probe(self._args(output_dir))
            self.assertEqual(result, 2)
            summary = json.loads(
                (output_dir / "summary.json").read_text(encoding="utf-8")
            )
            self.assertTrue(summary["stop_timed_out"])
            self.assertIn(
                "worker did not stop within 5 seconds",
                summary["fatal_errors"],
            )
            self.assertEqual(
                len(
                    (output_dir / "states.jsonl")
                    .read_text(encoding="utf-8")
                    .splitlines()
                ),
                1,
            )
            self.assertTrue((output_dir / "sha256_manifest.json").is_file())

    def test_unexpected_driver_fallback_fails_closed(self):
        from PyQt6.QtCore import QObject, QTimer, pyqtSignal

        class DummyMistralGui:
            pass

        class FakeWorker(QObject):
            state_updated = pyqtSignal(object)
            finished = pyqtSignal()

            def __init__(self):
                super().__init__()
                self._driver = DummyMistralGui()
                self._running = True

            def start(self):
                state = SimpleNamespace(
                    v_set=1.0,
                    v_actual=1.0,
                    i_set=1.0,
                    i_actual=1.0,
                    connected=True,
                    error="",
                    mode="screengrab",
                )
                QTimer.singleShot(0, lambda: self.state_updated.emit(state))

            def stop(self):
                self._running = False

            def wait(self, _timeout_ms):
                return True

            def isRunning(self):
                return self._running

        with tempfile.TemporaryDirectory() as tmp:
            output_dir = Path(tmp) / "run"
            with patch.object(
                probe_module,
                "_worker_factory",
                return_value=FakeWorker(),
            ):
                result = run_probe(self._args(output_dir))
            self.assertEqual(result, 2)
            summary = json.loads(
                (output_dir / "summary.json").read_text(encoding="utf-8")
            )
            self.assertTrue(
                any(
                    "refusing dummy/fallback validation" in message
                    for message in summary["fatal_errors"]
                )
            )


if __name__ == "__main__":
    unittest.main(verbosity=2)
