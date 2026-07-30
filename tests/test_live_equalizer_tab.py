"""Unit tests for the Live Equalizer tab + record_live_label logger method.

Two layers:
  - ``LiveEqualizerTab`` — Qt widget. Tests instantiate + drive it via
    the public methods (``update_camera_frame``, ``update_classifier_state``,
    ``set_save_enabled``, ``reset_for_new_session``) and verify slider
    mechanics + signal emission.
  - ``GrowthLogger.record_live_label`` — pure Python (+ optional PIL).
    Round-trip tests against a synthetic session directory.

Run:
    QT_QPA_PLATFORM=offscreen python scripts/test_live_equalizer_tab.py
"""
from __future__ import annotations

import csv
import os
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

# QApplication precedes any QWidget creation across the test module.
from PyQt6.QtWidgets import QApplication  # noqa: E402
_app = QApplication.instance() or QApplication(sys.argv)  # noqa: F841

from gui.growth_logger import GrowthLogger  # noqa: E402
from gui.live_equalizer_tab import (  # noqa: E402
    CLASS_LABELS,
    CLASSIFIER_LABEL_MAP,
    LiveEqualizerTab,
)
from gui.state import ClassifierState  # noqa: E402


def _rgb_frame(color: int = 128) -> np.ndarray:
    """Solid-color RGB frame that PIL can decode via fromarray()."""
    return np.full((64, 96, 3), color, dtype=np.uint8)


# --- LiveEqualizerTab tests -------------------------------------------------


class TabConstructionTests(unittest.TestCase):
    """Tab constructs cleanly + starts in the expected state."""

    def setUp(self):
        self.tab = LiveEqualizerTab()

    def tearDown(self):
        self.tab.deleteLater()

    def test_five_sliders_registered(self):
        self.assertEqual(len(self.tab._sliders), 5)
        self.assertEqual(set(self.tab._sliders.keys()), set(CLASS_LABELS))

    def test_five_classifier_value_labels_registered(self):
        self.assertEqual(len(self.tab._classifier_value_labels), 5)
        self.assertEqual(
            set(self.tab._classifier_value_labels.keys()), set(CLASS_LABELS),
        )

    def test_initial_slider_state_is_uniform(self):
        # _reset_to_uniform runs in __init__ → each slider at 20 (1/5).
        for slider in self.tab._sliders.values():
            self.assertEqual(slider.value(), 20)

    def test_save_button_starts_disabled(self):
        # Session isn't running at construction; Save must be off.
        self.assertFalse(self.tab._save_btn.isEnabled())

    def test_classifier_labels_start_at_placeholder(self):
        for label in self.tab._classifier_value_labels.values():
            self.assertEqual(label.text(), "—%")


class SaveEnabledTests(unittest.TestCase):
    """Save button gating via set_save_enabled."""

    def setUp(self):
        self.tab = LiveEqualizerTab()

    def tearDown(self):
        self.tab.deleteLater()

    def test_set_save_enabled_true_enables_button(self):
        self.tab.set_save_enabled(True)
        self.assertTrue(self.tab._save_btn.isEnabled())

    def test_set_save_enabled_false_disables_button(self):
        self.tab.set_save_enabled(True)
        self.tab.set_save_enabled(False)
        self.assertFalse(self.tab._save_btn.isEnabled())


class ClassifierStateRoutingTests(unittest.TestCase):
    """update_classifier_state populates the 5 classifier % labels."""

    def setUp(self):
        self.tab = LiveEqualizerTab()

    def tearDown(self):
        self.tab.deleteLater()

    def test_none_state_shows_dashes(self):
        self.tab.update_classifier_state(None)
        for lbl in self.tab._classifier_value_labels.values():
            self.assertEqual(lbl.text(), "—%")

    def test_smoothed_percent_populates_labels(self):
        # ClassifierState.smoothed_percent keys use Monitor-tab spellings;
        # tab translates via CLASSIFIER_LABEL_MAP.
        smoothed = {
            "1x1":            60,
            "Twinned (2x1)":  20,
            "c(6x2)":         10,
            "rt13xrt13":       5,
            "HTR":             5,
        }
        state = ClassifierState(
            ready=True, smoothed_percent=smoothed,
        )
        self.tab.update_classifier_state(state)
        self.assertEqual(self.tab._classifier_value_labels["1x1"].text(), "60%")
        self.assertEqual(
            self.tab._classifier_value_labels["Tw(2x1)"].text(), "20%",
        )
        self.assertEqual(
            self.tab._classifier_value_labels["c(6x2)"].text(), "10%",
        )
        self.assertEqual(self.tab._classifier_value_labels["RT13"].text(), "5%")
        self.assertEqual(self.tab._classifier_value_labels["HTR"].text(), "5%")

    def test_empty_smoothed_percent_shows_dashes(self):
        # Ready state with no percentages yet — same fallback as None.
        state = ClassifierState(ready=True, smoothed_percent={})
        self.tab.update_classifier_state(state)
        for lbl in self.tab._classifier_value_labels.values():
            self.assertEqual(lbl.text(), "—%")


class CameraFrameRoutingTests(unittest.TestCase):
    """update_camera_frame caches the frame + updates the Selected pane."""

    def setUp(self):
        self.tab = LiveEqualizerTab()

    def tearDown(self):
        self.tab.deleteLater()

    def test_frame_cached_on_update(self):
        frame = _rgb_frame(color=180)
        self.tab.update_camera_frame(frame)
        cached = self.tab.get_current_full_frame()
        self.assertIsNotNone(cached)
        self.assertTrue(np.array_equal(cached, frame))

    def test_none_frame_does_not_crash(self):
        # Camera hot-path can occasionally pass None; must silently no-op.
        self.tab.update_camera_frame(None)
        self.assertIsNone(self.tab.get_current_full_frame())

    def test_current_target_downsampled_to_process_wh(self):
        from scripts.equalizer_ui import PROCESS_WH
        frame = _rgb_frame(color=100)
        self.tab.update_camera_frame(frame)
        target = self.tab._current_target
        self.assertIsNotNone(target)
        # PROCESS_WH is (W, H); numpy shape is (H, W).
        self.assertEqual(target.shape, (PROCESS_WH[1], PROCESS_WH[0]))


class SliderMechanicsTests(unittest.TestCase):
    """Slider values, weight round-trip, reset."""

    def setUp(self):
        self.tab = LiveEqualizerTab()

    def tearDown(self):
        self.tab.deleteLater()

    def test_current_weights_are_fractions(self):
        # Uniform state → each weight ≈ 0.20.
        w = self.tab._current_weights()
        self.assertEqual(set(w.keys()), set(CLASS_LABELS))
        for v in w.values():
            self.assertAlmostEqual(v, 0.20, places=2)

    def test_set_weights_moves_all_sliders_atomically(self):
        target = {
            "1x1":     0.60,
            "Tw(2x1)": 0.20,
            "c(6x2)":  0.10,
            "RT13":    0.05,
            "HTR":     0.05,
        }
        self.tab._set_weights(target)
        actual = self.tab._current_weights()
        for label, expected in target.items():
            self.assertAlmostEqual(actual[label], expected, places=2)

    def test_reset_to_uniform_puts_each_at_20(self):
        self.tab._set_weights({
            "1x1": 1.0, "Tw(2x1)": 0.0, "c(6x2)": 0.0,
            "RT13": 0.0, "HTR": 0.0,
        })
        self.tab._reset_to_uniform()
        for label, slider in self.tab._sliders.items():
            self.assertEqual(slider.value(), 20)


class SaveSignalTests(unittest.TestCase):
    """Save button emits the live_label_save_requested signal with weights."""

    def setUp(self):
        self.tab = LiveEqualizerTab()

    def tearDown(self):
        self.tab.deleteLater()

    def test_save_click_emits_signal_with_current_weights(self):
        # Set a known mixture and click Save.
        self.tab._set_weights({
            "1x1":     0.50,
            "Tw(2x1)": 0.30,
            "c(6x2)":  0.10,
            "RT13":    0.05,
            "HTR":     0.05,
        })
        self.tab.set_save_enabled(True)

        captured: list[dict] = []
        self.tab.live_label_save_requested.connect(captured.append)
        self.tab._save_btn.click()

        self.assertEqual(len(captured), 1)
        weights = captured[0]
        self.assertAlmostEqual(weights["1x1"], 0.50, places=2)
        self.assertAlmostEqual(weights["Tw(2x1)"], 0.30, places=2)


class ResetForNewSessionTests(unittest.TestCase):
    """reset_for_new_session wipes frame + classifier state, keeps sliders."""

    def setUp(self):
        self.tab = LiveEqualizerTab()

    def tearDown(self):
        self.tab.deleteLater()

    def test_reset_clears_current_frame(self):
        self.tab.update_camera_frame(_rgb_frame())
        self.assertIsNotNone(self.tab.get_current_full_frame())
        self.tab.reset_for_new_session()
        self.assertIsNone(self.tab.get_current_full_frame())

    def test_reset_clears_classifier_labels(self):
        self.tab.update_classifier_state(ClassifierState(
            ready=True, smoothed_percent={"1x1": 80},
        ))
        self.assertEqual(
            self.tab._classifier_value_labels["1x1"].text(), "80%",
        )
        self.tab.reset_for_new_session()
        self.assertEqual(self.tab._classifier_value_labels["1x1"].text(), "—%")

    def test_reset_returns_sliders_to_uniform(self):
        self.tab._set_weights({
            "1x1": 1.0, "Tw(2x1)": 0.0, "c(6x2)": 0.0,
            "RT13": 0.0, "HTR": 0.0,
        })
        self.tab.reset_for_new_session()
        for slider in self.tab._sliders.values():
            self.assertEqual(slider.value(), 20)


class PauseButtonTests(unittest.TestCase):
    """Freeze frame / Resume live toggle behavior (Jul 14 2026 addition).

    Locks the three-part invariant: (1) button label describes the NEXT
    action, not the current state; (2) paused = camera frames dropped
    while cached frame preserved; (3) session reset restores unpaused
    default so the next armed session doesn't start frozen.
    """

    def setUp(self):
        self.tab = LiveEqualizerTab()

    def tearDown(self):
        self.tab.deleteLater()

    def test_initial_state_is_unpaused_with_freeze_label(self):
        # Fresh tab: not paused, button says what pressing it will do.
        self.assertFalse(self.tab._paused)
        self.assertFalse(self.tab._pause_btn.isChecked())
        self.assertEqual(self.tab._pause_btn.text(), "Freeze frame")

    def test_toggle_on_pauses_and_flips_label(self):
        self.tab._pause_btn.setChecked(True)
        self.assertTrue(self.tab._paused)
        # Label now describes the NEXT action (resume), not the current
        # state (paused).
        self.assertEqual(self.tab._pause_btn.text(), "Resume live")

    def test_toggle_off_resumes_and_flips_label_back(self):
        self.tab._pause_btn.setChecked(True)
        self.tab._pause_btn.setChecked(False)
        self.assertFalse(self.tab._paused)
        self.assertEqual(self.tab._pause_btn.text(), "Freeze frame")

    def test_paused_update_camera_frame_is_ignored(self):
        # Seed with a first frame so we have a known "frozen" state to
        # compare against.
        seed_frame = _rgb_frame(color=90)
        self.tab.update_camera_frame(seed_frame)
        cached_at_pause = self.tab.get_current_full_frame()
        self.assertIsNotNone(cached_at_pause)

        # Pause, then try to push a new frame.
        self.tab._pause_btn.setChecked(True)
        new_frame = _rgb_frame(color=200)
        self.tab.update_camera_frame(new_frame)

        # Cache still points at the pre-pause frame — new frame ignored.
        cached_after = self.tab.get_current_full_frame()
        self.assertTrue(np.array_equal(cached_after, seed_frame))
        self.assertFalse(np.array_equal(cached_after, new_frame))

    def test_paused_frame_still_saveable(self):
        # Frozen frame must still be reachable via get_current_full_frame
        # so the Save path (GrowthApp -> record_live_label) snapshots the
        # frame the grower was actually looking at.
        frame = _rgb_frame(color=120)
        self.tab.update_camera_frame(frame)
        self.tab._pause_btn.setChecked(True)
        # get_current_full_frame is what growth_app._on_live_label_save
        # reads.
        self.assertTrue(
            np.array_equal(self.tab.get_current_full_frame(), frame)
        )

    def test_resume_lets_new_frames_land(self):
        # After unpausing, the next update_camera_frame call updates the
        # cache. Guards against a bug where _paused stays True after the
        # button uncheck (state-machine leak).
        self.tab.update_camera_frame(_rgb_frame(color=50))
        self.tab._pause_btn.setChecked(True)
        # New frame while paused: ignored.
        self.tab.update_camera_frame(_rgb_frame(color=200))
        # Unpause + push a distinctive frame.
        self.tab._pause_btn.setChecked(False)
        new_frame = _rgb_frame(color=175)
        self.tab.update_camera_frame(new_frame)
        self.assertTrue(
            np.array_equal(self.tab.get_current_full_frame(), new_frame)
        )

    def test_reset_for_new_session_clears_pause(self):
        # Set up a paused state, then reset → button unchecked, label
        # back to Freeze frame, _paused False.
        self.tab.update_camera_frame(_rgb_frame(color=80))
        self.tab._pause_btn.setChecked(True)
        self.assertTrue(self.tab._paused)

        self.tab.reset_for_new_session()

        self.assertFalse(self.tab._paused)
        self.assertFalse(self.tab._pause_btn.isChecked())
        self.assertEqual(self.tab._pause_btn.text(), "Freeze frame")


class TabRegistrationTests(unittest.TestCase):
    """Live Equalizer is mounted at the expected tab index in GrowthMonitor."""

    def setUp(self):
        from gui.growth_monitor import GrowthMonitor
        self.monitor = GrowthMonitor()

    def tearDown(self):
        self.monitor.deleteLater()

    def test_tab_registered_between_scrubber_and_session(self):
        labels = [
            self.monitor._tabs.tabText(i)
            for i in range(self.monitor._tabs.count())
        ]
        self.assertIn("Live Equalizer", labels)
        idx = labels.index("Live Equalizer")
        self.assertEqual(labels[idx - 1], "Scrubber")
        self.assertEqual(labels[idx + 1], "Session")

    def test_live_equalizer_tab_attribute_exposed(self):
        self.assertTrue(hasattr(self.monitor, "live_equalizer_tab"))


# --- GrowthLogger.record_live_label tests -----------------------------------


class LiveLabelSchemaTests(unittest.TestCase):
    """LIVE_LABEL_FIELDS shape locked so downstream readers can rely on it."""

    def test_schema_contains_expected_columns(self):
        expected = {
            "timestamp", "elapsed_s", "label_idx",
            "recon_1x1", "recon_tw", "recon_c6x2",
            "recon_rt13", "recon_HTR",
            "pyrometer_temp_C",
            "voltage_V", "current_A", "psu_source",
            "frame_path",
        }
        self.assertEqual(set(GrowthLogger.LIVE_LABEL_FIELDS), expected)

    def test_label_idx_is_third_column(self):
        # Ordering matches manual_events + auto_capture_events pattern
        # (timestamp, elapsed_s, idx, ...) for cross-file consistency.
        self.assertEqual(GrowthLogger.LIVE_LABEL_FIELDS[0], "timestamp")
        self.assertEqual(GrowthLogger.LIVE_LABEL_FIELDS[1], "elapsed_s")
        self.assertEqual(GrowthLogger.LIVE_LABEL_FIELDS[2], "label_idx")


class LiveLabelLifecycleTests(unittest.TestCase):
    """File open on start_session, close on end_session, counter reset."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.logger = GrowthLogger(base_dir=self.tmp.name)

    def tearDown(self):
        try:
            self.logger.end_session()
        except Exception:
            pass
        self.tmp.cleanup()

    def test_live_labels_csv_created_on_start_session(self):
        self.logger.start_session("TEST_LIVE_A")
        self.assertTrue(
            (self.logger.session_dir / "live_labels.csv").exists()
        )

    def test_header_matches_schema(self):
        self.logger.start_session("TEST_LIVE_HEADER")
        with open(self.logger.session_dir / "live_labels.csv") as f:
            header = next(csv.reader(f))
        self.assertEqual(header, GrowthLogger.LIVE_LABEL_FIELDS)

    def test_counter_resets_across_sessions(self):
        self.logger.start_session("TEST_LIVE_C1")
        idx1 = self.logger.record_live_label(elapsed_s=1.0, weights={"1x1": 1.0})
        self.logger.end_session()
        self.logger.start_session("TEST_LIVE_C2")
        idx2 = self.logger.record_live_label(elapsed_s=2.0, weights={"1x1": 1.0})
        self.assertEqual(idx1, 1)
        self.assertEqual(idx2, 1)

    def test_no_session_returns_zero(self):
        self.assertEqual(
            self.logger.record_live_label(elapsed_s=1.0, weights={}), 0,
        )


class LiveLabelRecordTests(unittest.TestCase):
    """End-to-end row writes + optional frame snapshot."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.logger = GrowthLogger(base_dir=self.tmp.name)
        self.logger.start_session("TEST_LIVE_RECORD")
        self.csv_path = self.logger.session_dir / "live_labels.csv"

    def tearDown(self):
        try:
            self.logger.end_session()
        except Exception:
            pass
        self.tmp.cleanup()

    def _rows(self) -> list[dict]:
        with open(self.csv_path) as f:
            return list(csv.DictReader(f))

    def test_bare_call_writes_row(self):
        idx = self.logger.record_live_label(
            elapsed_s=42.0, weights={"1x1": 1.0},
        )
        self.assertEqual(idx, 1)
        rows = self._rows()
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["elapsed_s"], "42.00")
        self.assertEqual(rows[0]["label_idx"], "1")
        self.assertEqual(rows[0]["recon_1x1"], "1.0000")
        # Sliders the caller didn't set → default 0.0.
        self.assertEqual(rows[0]["recon_HTR"], "0.0000")

    def test_full_payload_populates_all_columns(self):
        weights = {
            "1x1":     0.60,
            "Tw(2x1)": 0.20,
            "c(6x2)":  0.10,
            "RT13":    0.05,
            "HTR":     0.05,
        }
        idx = self.logger.record_live_label(
            elapsed_s=123.45,
            weights=weights,
            pyro_temp=640.7,
            voltage_V=12.345,
            current_A=0.678,
            psu_source="mistral",
        )
        self.assertEqual(idx, 1)
        rows = self._rows()
        self.assertEqual(rows[0]["recon_1x1"], "0.6000")
        self.assertEqual(rows[0]["recon_tw"], "0.2000")
        self.assertEqual(rows[0]["recon_c6x2"], "0.1000")
        self.assertEqual(rows[0]["recon_rt13"], "0.0500")
        self.assertEqual(rows[0]["recon_HTR"], "0.0500")
        self.assertEqual(rows[0]["pyrometer_temp_C"], "640.7")
        self.assertEqual(rows[0]["voltage_V"], "12.345")
        self.assertEqual(rows[0]["current_A"], "0.678")
        self.assertEqual(rows[0]["psu_source"], "mistral")

    def test_counter_monotonically_increments(self):
        idxs = [
            self.logger.record_live_label(
                elapsed_s=float(i), weights={"1x1": 1.0},
            )
            for i in range(4)
        ]
        self.assertEqual(idxs, [1, 2, 3, 4])

    def test_frame_saved_when_provided(self):
        frame = _rgb_frame(color=140)
        idx = self.logger.record_live_label(
            elapsed_s=5.0, weights={"1x1": 0.5}, frame=frame,
        )
        rows = self._rows()
        frame_path = rows[0]["frame_path"]
        self.assertNotEqual(frame_path, "")
        self.assertTrue(Path(frame_path).exists())
        self.assertTrue(
            Path(frame_path).name.startswith(f"live_label_{idx:03d}_"),
        )
        self.assertTrue(frame_path.endswith(".bmp"))


if __name__ == "__main__":
    unittest.main(verbosity=2)
