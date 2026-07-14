"""Unit tests for scripts/growth_profile_explorer.py (Day 4 Jul 16 2026).

Locks the six-CSV SessionArtifacts reader contract + the temperature
chart rendering path. Uses synthetic session dirs per test — no
dependency on archived sessions or the AI-for-quantum repo (that
comes at Day 7 with batch_classify_session).

The reader tests import only stdlib + the module under test (no
matplotlib), so they cover the data layer even if matplotlib is
absent. Chart tests are the only ones that pull matplotlib in.

Run:
    QT_QPA_PLATFORM=offscreen MPLBACKEND=Agg \\
        python scripts/test_growth_profile_explorer.py
"""
from __future__ import annotations

import csv
import os
import sys
import tempfile
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

os.environ.setdefault("MPLBACKEND", "Agg")

from scripts.growth_profile_explorer import (  # noqa: E402
    EVENT_MARKER_COLORS,
    SessionArtifacts,
    render_temperature_chart,
)


def _write_csv(
    path: Path, fieldnames: list[str], rows: list[dict],
) -> None:
    """Helper: write a CSV with header + given rows. Creates parents."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for r in rows:
            w.writerow(r)


def _synth_sensor_rows(n: int) -> list[dict]:
    """Build n sensor rows: elapsed_s=15*i, temp=500+i, std=1.5.

    Matches the shape of a real ~5-minute session (5s cadence × 60 =
    12 rows/min; 15s stride here just keeps synthetic data compact
    without changing any downstream logic).
    """
    return [
        {
            "timestamp": f"2026-07-16T10:00:{i:02d}",
            "elapsed_s": str(15.0 * i),
            "pyrometer_temp_C": str(500.0 + i),
            "pyrometer_temp_std_C": "1.5",
            "pyrometer_temp_n": "5",
        }
        for i in range(n)
    ]


class SessionArtifactsReaderTests(unittest.TestCase):
    """Seven tests locking the 6-CSV reader behavior."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.session_dir = (
            Path(self.tmp.name) / "growth_TEST_20260716_100000"
        )
        self.session_dir.mkdir()

    def tearDown(self):
        self.tmp.cleanup()

    def test_full_read_from_valid_session(self):
        # All 6 CSVs present with at least one row each. Locks that
        # the reader hits every field on SessionArtifacts and none of
        # the writes get shadowed by field defaults.
        _write_csv(
            self.session_dir / "sensor_log.csv",
            ["timestamp", "elapsed_s", "pyrometer_temp_C",
             "pyrometer_temp_std_C", "pyrometer_temp_n"],
            _synth_sensor_rows(5),
        )
        _write_csv(
            self.session_dir / "commit_log.csv",
            ["timestamp", "elapsed_s", "sample_id", "note"],
            [{"timestamp": "T", "elapsed_s": "30",
              "sample_id": "S", "note": "n"}],
        )
        _write_csv(
            self.session_dir / "auto_capture_events.csv",
            ["timestamp", "elapsed_s", "event_idx", "change_score"],
            [{"timestamp": "T", "elapsed_s": "45",
              "event_idx": "1", "change_score": "0.9"}],
        )
        _write_csv(
            self.session_dir / "manual_events.csv",
            ["timestamp", "elapsed_s", "event_idx",
             "pyrometer_temp_C"],
            [{"timestamp": "T", "elapsed_s": "60",
              "event_idx": "1", "pyrometer_temp_C": "505"}],
        )
        _write_csv(
            self.session_dir / "heartbeat_log.csv",
            ["timestamp", "elapsed_s", "heartbeat_idx",
             "pyrometer_temp_C", "frame_path"],
            [{"timestamp": "T", "elapsed_s": "75",
              "heartbeat_idx": "1", "pyrometer_temp_C": "506",
              "frame_path": "/x"}],
        )
        _write_csv(
            self.session_dir / "events_labels.csv",
            ["event_idx", "primary_reconstruction", "notes"],
            [{"event_idx": "1",
              "primary_reconstruction": "1x1", "notes": ""}],
        )

        artifacts = SessionArtifacts.from_session_dir(self.session_dir)

        self.assertEqual(len(artifacts.sensor_rows), 5)
        self.assertEqual(len(artifacts.commit_rows), 1)
        self.assertEqual(len(artifacts.auto_capture_rows), 1)
        self.assertEqual(len(artifacts.manual_event_rows), 1)
        self.assertEqual(len(artifacts.heartbeat_rows), 1)
        self.assertEqual(len(artifacts.event_label_rows), 1)

    def test_missing_optional_csv_yields_empty_list(self):
        # Only sensor_log.csv exists. Others → empty lists (not
        # exceptions). Locks the pre-schema-session tolerance:
        # analyses of Apr 2026 sessions (predating manual_events)
        # must open cleanly with the newer reader.
        _write_csv(
            self.session_dir / "sensor_log.csv",
            ["elapsed_s", "pyrometer_temp_C"],
            [{"elapsed_s": "10", "pyrometer_temp_C": "500"}],
        )
        artifacts = SessionArtifacts.from_session_dir(self.session_dir)
        self.assertEqual(len(artifacts.sensor_rows), 1)
        self.assertEqual(artifacts.commit_rows, [])
        self.assertEqual(artifacts.auto_capture_rows, [])
        self.assertEqual(artifacts.manual_event_rows, [])
        self.assertEqual(artifacts.heartbeat_rows, [])
        self.assertEqual(artifacts.event_label_rows, [])

    def test_bad_row_skipped_in_temperature_series(self):
        # sensor row with non-numeric elapsed_s must not crash the
        # reader; instead the offending row is silently dropped from
        # the typed series (mirrors plot_temperature.py policy). Row
        # with blank temp is also dropped. Only the first row
        # survives, but the raw sensor_rows still contains all 3.
        _write_csv(
            self.session_dir / "sensor_log.csv",
            ["elapsed_s", "pyrometer_temp_C"],
            [
                {"elapsed_s": "10", "pyrometer_temp_C": "500"},
                {"elapsed_s": "not_a_number",
                 "pyrometer_temp_C": "501"},
                {"elapsed_s": "20", "pyrometer_temp_C": ""},
            ],
        )
        artifacts = SessionArtifacts.from_session_dir(self.session_dir)
        self.assertEqual(len(artifacts.sensor_rows), 3)
        elapsed, temps = artifacts.temperature_series()
        self.assertEqual(elapsed, [10.0])
        self.assertEqual(temps, [500.0])

    def test_events_labels_join_by_event_idx(self):
        # Load auto_capture + events_labels; verify callers can
        # correlate them via event_idx. Also verifies
        # label_for_event(idx) helper — the join path Day 5's
        # grower-vs-classifier scatter will use.
        _write_csv(
            self.session_dir / "sensor_log.csv",
            ["elapsed_s", "pyrometer_temp_C"],
            [{"elapsed_s": "10", "pyrometer_temp_C": "500"}],
        )
        _write_csv(
            self.session_dir / "auto_capture_events.csv",
            ["elapsed_s", "event_idx", "change_score"],
            [
                {"elapsed_s": "30", "event_idx": "1",
                 "change_score": "0.5"},
                {"elapsed_s": "60", "event_idx": "2",
                 "change_score": "0.7"},
            ],
        )
        _write_csv(
            self.session_dir / "events_labels.csv",
            ["event_idx", "primary_reconstruction", "notes"],
            [{"event_idx": "1",
              "primary_reconstruction": "Twinned (2x1)",
              "notes": "keep"}],
        )
        artifacts = SessionArtifacts.from_session_dir(self.session_dir)

        label = artifacts.label_for_event(1)
        self.assertIsNotNone(label)
        self.assertEqual(label["primary_reconstruction"], "Twinned (2x1)")
        # Event 2 was never labeled — join returns None, no exception.
        self.assertIsNone(artifacts.label_for_event(2))

    def test_elapsed_s_coerced_to_float_in_series(self):
        # temperature_series returns floats, not strings, even though
        # csv.DictReader always yields strings. Locks the typed
        # accessor contract — callers can do arithmetic without
        # per-row conversion.
        _write_csv(
            self.session_dir / "sensor_log.csv",
            ["elapsed_s", "pyrometer_temp_C"],
            [{"elapsed_s": "15.5", "pyrometer_temp_C": "500.7"}],
        )
        artifacts = SessionArtifacts.from_session_dir(self.session_dir)
        elapsed, temps = artifacts.temperature_series()
        self.assertIsInstance(elapsed[0], float)
        self.assertIsInstance(temps[0], float)
        self.assertAlmostEqual(elapsed[0], 15.5)
        self.assertAlmostEqual(temps[0], 500.7)

    def test_missing_session_dir_raises_fnf(self):
        # Non-existent path → FileNotFoundError. Guard against silent
        # empty-charts-on-typo (a user typing the wrong session name
        # should learn quickly, not get a blank PNG).
        with self.assertRaises(FileNotFoundError):
            SessionArtifacts.from_session_dir(
                Path("/tmp/does-not-exist-12345"),
            )

    def test_synthetic_10_row_round_trip(self):
        # Write 10 rows, read 10 rows back — end-to-end reader sanity
        # with the same shape a real session would produce.
        _write_csv(
            self.session_dir / "sensor_log.csv",
            ["timestamp", "elapsed_s", "pyrometer_temp_C",
             "pyrometer_temp_std_C", "pyrometer_temp_n"],
            _synth_sensor_rows(10),
        )
        artifacts = SessionArtifacts.from_session_dir(self.session_dir)
        self.assertEqual(len(artifacts.sensor_rows), 10)
        elapsed, temps = artifacts.temperature_series()
        self.assertEqual(len(elapsed), 10)
        self.assertEqual(elapsed[0], 0.0)
        self.assertEqual(elapsed[-1], 135.0)  # 15 * 9


class TemperatureChartTests(unittest.TestCase):
    """Three tests locking chart rendering behavior."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.session_dir = (
            Path(self.tmp.name) / "growth_TEST_20260716_100000"
        )
        self.session_dir.mkdir()
        self.out_dir = Path(self.tmp.name) / "out"
        self.out_dir.mkdir()

    def tearDown(self):
        self.tmp.cleanup()

    def test_png_written_when_temp_data_present(self):
        # Minimal session with 5 sensor rows → chart PNG lands on
        # disk. File size sanity-checked against a "definitely
        # non-empty PNG" floor (matplotlib empty savefig is ~200
        # bytes; a real chart is 10-50KB).
        _write_csv(
            self.session_dir / "sensor_log.csv",
            ["elapsed_s", "pyrometer_temp_C"],
            [{"elapsed_s": str(15.0 * i),
              "pyrometer_temp_C": str(500 + i)}
             for i in range(5)],
        )
        artifacts = SessionArtifacts.from_session_dir(self.session_dir)
        out = self.out_dir / "temp.png"
        result = render_temperature_chart(artifacts, out, dpi=100)
        self.assertEqual(result, out)
        self.assertTrue(out.exists())
        self.assertGreater(out.stat().st_size, 500)

    def test_overlays_include_commit_and_manual_markers(self):
        # With commit and manual event rows present, event_overlays()
        # returns both sources and render succeeds. We can't easily
        # pixel-inspect the PNG for vertical-line colors without a
        # visual regression harness, so we assert on the accessor +
        # the render code path executing cleanly (dedup legend logic
        # + color lookup would raise if the source names diverged
        # from EVENT_MARKER_COLORS).
        _write_csv(
            self.session_dir / "sensor_log.csv",
            ["elapsed_s", "pyrometer_temp_C"],
            [{"elapsed_s": str(15.0 * i),
              "pyrometer_temp_C": str(500 + i)}
             for i in range(5)],
        )
        _write_csv(
            self.session_dir / "commit_log.csv",
            ["elapsed_s", "sample_id", "note"],
            [{"elapsed_s": "30", "sample_id": "S", "note": "n"}],
        )
        _write_csv(
            self.session_dir / "manual_events.csv",
            ["elapsed_s", "event_idx"],
            [{"elapsed_s": "45", "event_idx": "1"}],
        )
        artifacts = SessionArtifacts.from_session_dir(self.session_dir)

        overlays = artifacts.event_overlays()
        sources = {source for _, source in overlays}
        self.assertIn("commit", sources)
        self.assertIn("manual", sources)
        # And both sources have a color mapping — guard against a
        # future rename of the source labels without updating the
        # color dict.
        for source in sources:
            self.assertIn(source, EVENT_MARKER_COLORS)

        out = self.out_dir / "temp_annot.png"
        result = render_temperature_chart(artifacts, out, dpi=100)
        self.assertEqual(result, out)
        self.assertTrue(out.exists())

    def test_none_returned_when_temperature_empty(self):
        # Empty sensor CSV → temperature_series returns [] → chart
        # returns None and writes no file. Guard against a "silent
        # PNG with an empty chart" failure mode — caller should be
        # able to skip the file entirely.
        _write_csv(
            self.session_dir / "sensor_log.csv",
            ["elapsed_s", "pyrometer_temp_C"],
            [],
        )
        artifacts = SessionArtifacts.from_session_dir(self.session_dir)
        out = self.out_dir / "temp.png"
        result = render_temperature_chart(artifacts, out, dpi=100)
        self.assertIsNone(result)
        self.assertFalse(out.exists())


if __name__ == "__main__":
    unittest.main(verbosity=2)
