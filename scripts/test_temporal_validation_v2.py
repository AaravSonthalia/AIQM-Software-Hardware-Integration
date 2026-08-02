"""Deterministic tests for GUI temporal validation v2."""

from __future__ import annotations

import csv
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from drivers.evap_control import EvapControl  # noqa: E402
from drivers.mistral import MistralGui  # noqa: E402
from gui.growth_logger import GrowthLogger  # noqa: E402
from gui.state import CameraState, MistralState, PyrometerState  # noqa: E402
from gui.temporal_observability import (  # noqa: E402
    snapshot_state,
    synchronization_summary,
)
from gui.workers import (  # noqa: E402
    _emission_snapshot,
    _mark_read_failed,
    _mark_sample_received,
)
from scripts.ombe_timing_probe import (  # noqa: E402
    summarize_operator_actions,
    summarize_trace,
)
from scripts.validate_temporal_session import validate  # noqa: E402


class TimingMathTests(unittest.TestCase):
    def test_known_delays_use_only_monotonic_clock(self):
        for delay_ms in (0, 100, 500, 1500):
            state = MistralState(
                connected=True,
                valid=True,
                sample_sequence=4,
                received_at_utc="2099-01-01T00:00:00+00:00",
                received_monotonic_ns=10_000_000_000,
            )
            snapshot = snapshot_state(
                "mistral", state,
                10_000_000_000 + delay_ms * 1_000_000,
            )
            self.assertEqual(snapshot.age_ms, float(delay_ms))

    def test_sync_span_and_structural_validity(self):
        states = {}
        for index, source in enumerate((
            "rheed", "pyrometer", "mistral", "evap",
        )):
            state = MistralState(
                connected=True, valid=True, sample_sequence=1,
                received_at_utc="2026-08-02T12:00:00+00:00",
                received_monotonic_ns=1_000_000_000 + index * 500_000,
            )
            states[source] = snapshot_state(
                source, state, 2_000_000_000,
            )
        summary = synchronization_summary(states)
        self.assertTrue(summary["sync_complete"])
        self.assertTrue(summary["sync_valid"])
        self.assertEqual(summary["sync_span_ms"], 1.5)

        states["evap"] = snapshot_state("evap", MistralState())
        summary = synchronization_summary(states)
        self.assertFalse(summary["sync_complete"])
        self.assertFalse(summary["sync_valid"])

    def test_failure_does_not_advance_or_refresh_sample(self):
        state = PyrometerState(sample_sequence=7, valid=True)
        with (
            patch("gui.workers.time.perf_counter_ns", return_value=5_000_000),
            patch("gui.workers._utc_iso_now", return_value="t"),
        ):
            _mark_sample_received(state, 4_000_000)
        previous_received = state.received_monotonic_ns
        self.assertEqual(state.sample_sequence, 8)
        _mark_read_failed(state, "cable disconnected")
        self.assertEqual(state.sample_sequence, 8)
        self.assertEqual(state.received_monotonic_ns, previous_received)
        self.assertFalse(state.valid)

    def test_emission_returns_an_independent_state(self):
        state = MistralState(sample_sequence=2)
        with patch("gui.workers.time.perf_counter_ns", return_value=99):
            emitted = _emission_snapshot(state)
        state.sample_sequence = 3
        self.assertEqual(emitted.sample_sequence, 2)
        self.assertEqual(emitted.worker_emitted_monotonic_ns, 99)


class OcrCaptureTimingTests(unittest.TestCase):
    def test_mistral_capture_time_is_before_ocr_completion(self):
        driver = MistralGui()
        driver._connected = True
        driver._hwnd = 1
        frame = np.zeros((1039, 1793, 3), dtype=np.uint8)
        with (
            patch("drivers.mistral.capture_window", return_value=frame),
            patch("drivers.mistral.ocr_crop", return_value="Set 1 Actual 2"),
            patch("drivers.mistral.time.perf_counter_ns", return_value=123),
        ):
            driver.read()
        self.assertEqual(driver.last_capture_monotonic_ns, 123)
        self.assertIsNotNone(driver.last_capture_at_utc)

    def test_evap_capture_time_survives_ocr_parse_failure(self):
        driver = EvapControl()
        driver._connected = True
        driver._hwnd = 1
        frame = np.zeros((567, 1497, 3), dtype=np.uint8)
        with (
            patch("drivers.evap_control.capture_window", return_value=frame),
            patch("drivers.evap_control.ocr_crop", return_value="garbage"),
            patch("drivers.evap_control.time.perf_counter_ns", return_value=456),
        ):
            values = driver.read()
        self.assertIsNone(values["chamber_pressure_mbar"])
        self.assertEqual(driver.last_capture_monotonic_ns, 456)


class TraceAndDurabilityTests(unittest.TestCase):
    def test_logger_writes_parseable_trace_and_sync_columns(self):
        with tempfile.TemporaryDirectory() as tmp:
            logger = GrowthLogger(base_dir=tmp)
            logger.start_session("TEMPORAL")
            session = logger.session_dir
            logger.log_temporal_event(
                "state_received", "rheed",
                timing={"sequence": 5, "valid": True},
            )
            logger.log_sensors(
                500.0, 1.0,
                snapshot_at_utc="2026-08-02T12:00:00+00:00",
                snapshot_monotonic_ns=123,
                sync_span_ms=25.0,
                sync_complete=True,
                sync_valid=True,
                rheed_timing={
                    "sequence": 8,
                    "received_at_utc": "2026-08-02T12:00:00+00:00",
                    "valid": True,
                    "mode": "screengrab",
                },
            )
            logger.end_session()
            trace = [
                json.loads(line)
                for line in (session / "temporal_trace.jsonl").read_text().splitlines()
            ]
            self.assertEqual(trace[0]["event"], "session_start")
            self.assertEqual(trace[-1]["event"], "session_end")
            with (session / "sensor_log.csv").open(newline="") as stream:
                row = next(csv.DictReader(stream))
            self.assertEqual(row["sync_span_ms"], "25.000")
            self.assertEqual(row["rheed_sample_sequence"], "8")
            self.assertEqual(row["rheed_valid"], "True")

    def test_trace_nearest_neighbor_and_fault_latency(self):
        records = [
            {
                "event": "state_received", "source": source,
                "recorded_monotonic_ns": recorded,
                "timing": {
                    "received_monotonic_ns": received,
                    "valid": valid,
                    "error": error,
                },
            }
            for source, recorded, received, valid, error in (
                ("rheed", 1_000, 1_000, True, ""),
                ("mistral", 1_100, 1_100, True, ""),
                ("mistral", 2_500, 1_100, False, "closed"),
                ("mistral", 4_000, 4_000, True, ""),
            )
        ]
        trace = summarize_trace(records, 0)
        self.assertEqual(
            trace["nearest_neighbor_to_rheed"]["mistral"]
            ["absolute_delta_ms"]["p50"],
            0.0001,
        )
        actions = [
            {"label": "close", "source": "mistral", "phase": "before",
             "recorded_monotonic_ns": 2_000},
            {"label": "reopen", "source": "mistral", "phase": "after",
             "recorded_monotonic_ns": 3_000},
        ]
        result = summarize_operator_actions(records, actions, 0)
        self.assertEqual(result["transitions"][0]["transition"]["latency_ms"], 0.0005)
        self.assertEqual(result["transitions"][1]["transition"]["latency_ms"], 0.001)

    def test_validator_rejects_partial_jsonl_after_forced_exit(self):
        with tempfile.TemporaryDirectory() as tmp:
            session = Path(tmp)
            (session / "frames").mkdir()
            (session / "sensor_log.csv").write_text(
                "timestamp,rheed_valid\n", encoding="utf-8",
            )
            (session / "heartbeat_log.csv").write_text(
                "frame_path,capture_sequence,capture_backend,source_hwnd,captured_at_utc\n",
                encoding="utf-8",
            )
            (session / "temporal_trace.jsonl").write_text(
                '{"event":"session_start"}\n{"event":', encoding="utf-8",
            )
            result = validate(session)
        self.assertFalse(result["passed"])
        self.assertTrue(any("temporal_trace" in error for error in result["errors"]))


if __name__ == "__main__":
    unittest.main()
