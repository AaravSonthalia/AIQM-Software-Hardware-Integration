"""Mac-side test suite for the grower-correction UI (deliverable #5).

Exercises the ``✎ Correct`` toggle, Pattern A proportional adjustment,
sum-to-100 normalization, correction-mode guard on
``update_classifier_state``, and the enriched ``_on_commit`` payload
(paired classifier + grower fields, ``grower_corrected`` flag,
auto-lock after LOG ENTRY).

Runs headless via ``QT_QPA_PLATFORM=offscreen`` so no display required.
Instantiates a real ``GrowthMonitor`` widget so the tests exercise the
same signal/slot wiring that ships. ClassifierState dataclasses are
built directly — no worker, no torch, no model file.

Run: ``python scripts/test_grower_correction.py`` or under pytest.
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

from gui.growth_monitor import GrowthMonitor  # noqa: E402
from gui.recon_labels import RECON_LABELS  # noqa: E402
from gui.state import ClassifierState  # noqa: E402


def _make_classifier_state(
    smoothed: dict[str, int] | None = None,
    quality: float = 0.9,
    is_ood: bool = False,
    has_confident_data: bool = True,
    ready: bool = True,
    loading: bool = False,
    error: str = "",
    model_version: str = "test-model (2026-07-06)",
) -> ClassifierState:
    """Build a ClassifierState with sane defaults for tests.

    smoothed defaults to a lopsided distribution (60% Twinned + tail)
    so it's obvious when correction *does* vs *doesn't* touch sliders.
    """
    if smoothed is None:
        smoothed = {
            "1x1": 10,
            "Twinned (2x1)": 60,
            "c(6x2)": 20,
            "rt13xrt13": 5,
            "HTR": 5,
        }
    return ClassifierState(
        loading=loading,
        ready=ready,
        error=error,
        last_frame_number=42,
        raw_scores={n: v / 100.0 for n, v in smoothed.items()},
        normalized_percent=smoothed,
        smoothed_percent=smoothed,
        raw_sum=1.0,
        quality=quality,
        is_bad=False,
        bad_confidence=0.0,
        is_ood=is_ood,
        has_confident_data=has_confident_data,
        inference_ms=13.0,
        model_version=model_version,
    )


def _slider_values(monitor: GrowthMonitor) -> dict[str, int]:
    return {n: s.value() for n, s in monitor._recon_sliders.items()}


# ---------------------------------------------------------------------------
# _normalize_sliders_to_100
# ---------------------------------------------------------------------------

class NormalizationTests(unittest.TestCase):

    def setUp(self):
        self.monitor = GrowthMonitor()

    def tearDown(self):
        self.monitor.deleteLater()

    def _set_sliders(self, values: dict[str, int]):
        for name, v in values.items():
            slider = self.monitor._recon_sliders[name]
            slider.blockSignals(True)
            slider.setValue(v)
            slider.blockSignals(False)

    def test_already_at_100_no_change(self):
        target = {"1x1": 20, "Twinned (2x1)": 20, "c(6x2)": 20,
                  "rt13xrt13": 20, "HTR": 20}
        self._set_sliders(target)
        self.monitor._normalize_sliders_to_100()
        self.assertEqual(_slider_values(self.monitor), target)

    def test_all_zero_distributes_uniformly(self):
        self._set_sliders({n: 0 for n in RECON_LABELS})
        self.monitor._normalize_sliders_to_100()
        result = _slider_values(self.monitor)
        self.assertEqual(sum(result.values()), 100)
        # 100/5 = 20 each, no residual
        for v in result.values():
            self.assertEqual(v, 20)

    def test_sum_below_100_scales_up_preserving_ratio(self):
        # 10+30+15+2+3 = 60 → scale to 100 preserving ratios.
        self._set_sliders({"1x1": 10, "Twinned (2x1)": 30, "c(6x2)": 15,
                           "rt13xrt13": 2, "HTR": 3})
        self.monitor._normalize_sliders_to_100()
        result = _slider_values(self.monitor)
        self.assertEqual(sum(result.values()), 100)
        # Argmax preserved.
        self.assertEqual(max(result, key=result.get), "Twinned (2x1)")

    def test_sum_above_100_scales_down_preserving_ratio(self):
        # 30+60+40+20+10 = 160 → scale to 100.
        self._set_sliders({"1x1": 30, "Twinned (2x1)": 60, "c(6x2)": 40,
                           "rt13xrt13": 20, "HTR": 10})
        self.monitor._normalize_sliders_to_100()
        result = _slider_values(self.monitor)
        self.assertEqual(sum(result.values()), 100)
        self.assertEqual(max(result, key=result.get), "Twinned (2x1)")

    def test_rounding_residual_assigned_to_largest(self):
        # A distribution that rounds off-by-one after proportional scaling.
        # 11+22+33+11+22 = 99 → scale to 100. round(each * 100/99):
        #   11.11→11, 22.22→22, 33.33→33, 11.11→11, 22.22→22 = 99.
        # Residual +1 goes to the largest (c(6x2)=33).
        self._set_sliders({"1x1": 11, "Twinned (2x1)": 22, "c(6x2)": 33,
                           "rt13xrt13": 11, "HTR": 22})
        self.monitor._normalize_sliders_to_100()
        result = _slider_values(self.monitor)
        self.assertEqual(sum(result.values()), 100)
        # c(6x2) started largest and stays largest — residual landed on it.
        self.assertEqual(max(result, key=result.get), "c(6x2)")


# ---------------------------------------------------------------------------
# _on_grower_slider_changed — Pattern A proportional adjustment
# ---------------------------------------------------------------------------

class ProportionalAdjustmentTests(unittest.TestCase):

    def setUp(self):
        self.monitor = GrowthMonitor()

    def tearDown(self):
        self.monitor.deleteLater()

    def _prime_correction(self, values: dict[str, int]):
        """Turn on correction and set explicit starting values.

        Bypasses the toggle-on normalization by writing values directly
        under blockSignals — needed to test starting states that don't
        sum to 100 yet."""
        for name, v in values.items():
            slider = self.monitor._recon_sliders[name]
            slider.blockSignals(True)
            slider.setValue(v)
            slider.blockSignals(False)
        self.monitor._correction_active = True
        for s in self.monitor._recon_sliders.values():
            s.setEnabled(True)

    def test_no_op_when_correction_off(self):
        self._prime_correction({"1x1": 30, "Twinned (2x1)": 30, "c(6x2)": 20,
                                "rt13xrt13": 10, "HTR": 10})
        self.monitor._correction_active = False  # override the prime
        before = _slider_values(self.monitor)
        # Simulate a valueChanged handler call — should be a no-op.
        self.monitor._on_grower_slider_changed("1x1", 90)
        self.assertEqual(_slider_values(self.monitor), before)

    def test_no_op_during_reentrant_adjustment(self):
        self._prime_correction({"1x1": 20, "Twinned (2x1)": 20, "c(6x2)": 20,
                                "rt13xrt13": 20, "HTR": 20})
        self.monitor._adjusting = True  # simulate mid-adjustment
        before = _slider_values(self.monitor)
        self.monitor._on_grower_slider_changed("1x1", 90)
        self.assertEqual(_slider_values(self.monitor), before)
        self.monitor._adjusting = False  # cleanup

    def test_drag_up_reduces_others_proportionally(self):
        # Uniform starting state (sum=100).
        self._prime_correction({"1x1": 20, "Twinned (2x1)": 20, "c(6x2)": 20,
                                "rt13xrt13": 20, "HTR": 20})
        # Grower drags 1x1 up to 40. Others must sum to 60, each was 20 of
        # a 80 total → each * 60 / 80 = 15.
        self.monitor._recon_sliders["1x1"].blockSignals(True)
        self.monitor._recon_sliders["1x1"].setValue(40)
        self.monitor._recon_sliders["1x1"].blockSignals(False)
        self.monitor._on_grower_slider_changed("1x1", 40)
        result = _slider_values(self.monitor)
        self.assertEqual(sum(result.values()), 100)
        self.assertEqual(result["1x1"], 40)
        for other in ("Twinned (2x1)", "c(6x2)", "rt13xrt13", "HTR"):
            self.assertEqual(result[other], 15)

    def test_drag_to_100_zeros_others(self):
        self._prime_correction({"1x1": 20, "Twinned (2x1)": 20, "c(6x2)": 20,
                                "rt13xrt13": 20, "HTR": 20})
        self.monitor._recon_sliders["1x1"].blockSignals(True)
        self.monitor._recon_sliders["1x1"].setValue(100)
        self.monitor._recon_sliders["1x1"].blockSignals(False)
        self.monitor._on_grower_slider_changed("1x1", 100)
        result = _slider_values(self.monitor)
        self.assertEqual(sum(result.values()), 100)
        self.assertEqual(result["1x1"], 100)
        for other in ("Twinned (2x1)", "c(6x2)", "rt13xrt13", "HTR"):
            self.assertEqual(result[other], 0)

    def test_drag_down_when_others_zero_uniform_distributes(self):
        # All in one class. Grower drags it down.
        self._prime_correction({"1x1": 100, "Twinned (2x1)": 0, "c(6x2)": 0,
                                "rt13xrt13": 0, "HTR": 0})
        self.monitor._recon_sliders["1x1"].blockSignals(True)
        self.monitor._recon_sliders["1x1"].setValue(20)
        self.monitor._recon_sliders["1x1"].blockSignals(False)
        self.monitor._on_grower_slider_changed("1x1", 20)
        result = _slider_values(self.monitor)
        self.assertEqual(sum(result.values()), 100)
        self.assertEqual(result["1x1"], 20)
        # 80 distributed over 4 → 20 each, no residual.
        for other in ("Twinned (2x1)", "c(6x2)", "rt13xrt13", "HTR"):
            self.assertEqual(result[other], 20)

    def test_rounding_residual_lands_on_largest_other(self):
        # Others 13, 13, 13, 13 = 52. Grower drags changed to 51.
        # Others must sum to 49. Each: 13 * 49/52 = 12.25 → round to 12.
        # 12*4 = 48, residual +1 goes to the largest — since all are tied,
        # dict ordering picks Twinned (2x1) (first "other" in dict order).
        self._prime_correction({"1x1": 48, "Twinned (2x1)": 13, "c(6x2)": 13,
                                "rt13xrt13": 13, "HTR": 13})
        self.monitor._recon_sliders["1x1"].blockSignals(True)
        self.monitor._recon_sliders["1x1"].setValue(51)
        self.monitor._recon_sliders["1x1"].blockSignals(False)
        self.monitor._on_grower_slider_changed("1x1", 51)
        result = _slider_values(self.monitor)
        self.assertEqual(sum(result.values()), 100)
        self.assertEqual(result["1x1"], 51)
        # Exactly one of the others carries the +1 residual.
        others = [result[k] for k in
                  ("Twinned (2x1)", "c(6x2)", "rt13xrt13", "HTR")]
        self.assertEqual(sorted(others), [12, 12, 12, 13])

    def test_reentrancy_guard_prevents_stack_overflow(self):
        # This test proves the guard prevents infinite recursion when
        # our own setValue() re-triggers the connected valueChanged
        # slot on sibling sliders. Without the guard, the test hits
        # Python's recursion limit.
        self._prime_correction({"1x1": 20, "Twinned (2x1)": 20, "c(6x2)": 20,
                                "rt13xrt13": 20, "HTR": 20})
        # Real valueChanged signal path — call setValue directly and
        # let Qt fire the connected slot.
        self.monitor._recon_sliders["1x1"].setValue(40)
        result = _slider_values(self.monitor)
        self.assertEqual(sum(result.values()), 100)
        self.assertEqual(result["1x1"], 40)


# ---------------------------------------------------------------------------
# _on_correction_toggled
# ---------------------------------------------------------------------------

class ToggleTests(unittest.TestCase):

    def setUp(self):
        self.monitor = GrowthMonitor()

    def tearDown(self):
        self.monitor.deleteLater()

    def test_toggle_on_enables_sliders(self):
        self.assertFalse(
            self.monitor._recon_sliders["1x1"].isEnabled()
        )
        self.monitor.correction_btn.setChecked(True)
        self.monitor._on_correction_toggled(True)
        for slider in self.monitor._recon_sliders.values():
            self.assertTrue(slider.isEnabled())
        self.assertTrue(self.monitor._correction_active)

    def test_toggle_off_disables_sliders(self):
        self.monitor._on_correction_toggled(True)
        self.monitor._on_correction_toggled(False)
        for slider in self.monitor._recon_sliders.values():
            self.assertFalse(slider.isEnabled())
        self.assertFalse(self.monitor._correction_active)

    def test_toggle_on_normalizes_to_100(self):
        # Start with a non-100 sum (mirrors real classifier drift).
        for name, v in [("1x1", 25), ("Twinned (2x1)", 30), ("c(6x2)", 25),
                        ("rt13xrt13", 10), ("HTR", 9)]:
            self.monitor._recon_sliders[name].blockSignals(True)
            self.monitor._recon_sliders[name].setValue(v)
            self.monitor._recon_sliders[name].blockSignals(False)
        self.assertEqual(sum(_slider_values(self.monitor).values()), 99)
        self.monitor._on_correction_toggled(True)
        self.assertEqual(sum(_slider_values(self.monitor).values()), 100)

    def test_toggle_off_restores_classifier_state(self):
        # Cache a classifier state, then toggle on → drag → toggle off.
        # Toggle-off must re-render the cached state (not the grower drag).
        state = _make_classifier_state()
        self.monitor.update_classifier_state(state)
        expected_after_off = dict(state.smoothed_percent)

        self.monitor._on_correction_toggled(True)
        # Simulate a drag.
        self.monitor._recon_sliders["1x1"].setValue(80)
        # Toggle off — classifier state should be re-rendered.
        self.monitor._on_correction_toggled(False)
        self.assertEqual(_slider_values(self.monitor), expected_after_off)

    def test_toggle_off_with_no_classifier_falls_back_to_idle(self):
        # No classifier state ever received; toggle on then off should
        # leave sliders at 0 with idle status.
        self.monitor._on_correction_toggled(True)
        self.monitor._on_correction_toggled(False)
        self.assertEqual(
            _slider_values(self.monitor),
            {n: 0 for n in RECON_LABELS},
        )
        self.assertIn("idle", self.monitor._recon_status_label.text().lower())


# ---------------------------------------------------------------------------
# update_classifier_state guard
# ---------------------------------------------------------------------------

class UpdateClassifierStateGuardTests(unittest.TestCase):

    def setUp(self):
        self.monitor = GrowthMonitor()

    def tearDown(self):
        self.monitor.deleteLater()

    def test_state_cached_even_during_correction(self):
        self.monitor._on_correction_toggled(True)
        state = _make_classifier_state()
        self.monitor.update_classifier_state(state)
        self.assertIs(self.monitor._latest_classifier, state)

    def test_sliders_not_updated_during_correction(self):
        # Grower drags to a specific configuration.
        self.monitor._on_correction_toggled(True)
        self.monitor._recon_sliders["1x1"].setValue(70)
        grower_values = _slider_values(self.monitor)

        # New classifier state arrives — sliders must not move.
        different_state = _make_classifier_state(smoothed={
            "1x1": 5, "Twinned (2x1)": 5, "c(6x2)": 5,
            "rt13xrt13": 5, "HTR": 80,
        })
        self.monitor.update_classifier_state(different_state)
        self.assertEqual(_slider_values(self.monitor), grower_values)

    def test_status_label_not_overwritten_during_correction(self):
        # Toggle on sets the correction-mode message; a classifier update
        # arriving while correction is active must NOT stomp it.
        self.monitor._on_correction_toggled(True)
        correction_msg = self.monitor._recon_status_label.text()

        state = _make_classifier_state()
        self.monitor.update_classifier_state(state)
        self.assertEqual(
            self.monitor._recon_status_label.text(), correction_msg,
        )


# ---------------------------------------------------------------------------
# _on_commit enrichment + auto-lock
# ---------------------------------------------------------------------------

class CommitTests(unittest.TestCase):

    def setUp(self):
        self.monitor = GrowthMonitor()
        # Signal capture — commit_requested emits the entry dict.
        self.captured: list[dict] = []
        self.monitor.commit_requested.connect(self.captured.append)
        # Simulate a running session so _on_commit doesn't bail on the
        # commit_btn.isEnabled() early-return guard.
        self.monitor.commit_btn.setEnabled(True)

    def tearDown(self):
        self.monitor.deleteLater()

    def test_commit_correction_off_pairs_slider_with_classifier(self):
        state = _make_classifier_state(smoothed={
            "1x1": 10, "Twinned (2x1)": 60, "c(6x2)": 20,
            "rt13xrt13": 5, "HTR": 5,
        })
        self.monitor.update_classifier_state(state)

        self.monitor._on_commit()
        entry = self.captured[-1]

        # Sliders mirror classifier (correction off) → recon_* ==
        # classifier_recon_*.
        for name in RECON_LABELS:
            self.assertEqual(
                entry[f"recon_{name}"],
                entry[f"classifier_recon_{name}"],
                f"mismatch on {name}",
            )
        self.assertEqual(entry["grower_corrected"], "False")

    def test_commit_correction_on_captures_both(self):
        # Classifier says Twinned; grower disagrees and says 1x1.
        classifier_smoothed = {
            "1x1": 5, "Twinned (2x1)": 60, "c(6x2)": 20,
            "rt13xrt13": 10, "HTR": 5,
        }
        state = _make_classifier_state(smoothed=classifier_smoothed)
        self.monitor.update_classifier_state(state)

        # Enter correction, grower argues for 1x1.
        self.monitor._on_correction_toggled(True)
        self.monitor._recon_sliders["1x1"].setValue(80)

        self.monitor._on_commit()
        entry = self.captured[-1]

        # Grower recon = slider values (sum 100).
        grower_total = sum(
            int(entry[f"recon_{n}"]) for n in RECON_LABELS
        )
        self.assertEqual(grower_total, 100)
        self.assertEqual(int(entry["recon_1x1"]), 80)

        # Classifier recon = the snapshot (unchanged by correction).
        for name, v in classifier_smoothed.items():
            self.assertEqual(int(entry[f"classifier_recon_{name}"]), v)

        self.assertEqual(entry["grower_corrected"], "True")

    def test_commit_without_classifier_state_leaves_columns_empty(self):
        # No classifier state ever received (classifier disabled for
        # session, or hasn't emitted yet).
        self.monitor._on_commit()
        entry = self.captured[-1]
        for name in RECON_LABELS:
            self.assertEqual(entry[f"classifier_recon_{name}"], "")
        self.assertEqual(entry["grower_corrected"], "")

    def test_commit_auto_locks_correction(self):
        state = _make_classifier_state()
        self.monitor.update_classifier_state(state)
        self.monitor._on_correction_toggled(True)
        self.assertTrue(self.monitor._correction_active)
        self.assertTrue(self.monitor.correction_btn.isChecked())

        self.monitor._on_commit()

        # Auto-lock: toggle off, button unchecked, sliders resume tracking.
        self.assertFalse(self.monitor._correction_active)
        self.assertFalse(self.monitor.correction_btn.isChecked())
        for slider in self.monitor._recon_sliders.values():
            self.assertFalse(slider.isEnabled())


# ---------------------------------------------------------------------------
# reset paths
# ---------------------------------------------------------------------------

class ResetPathTests(unittest.TestCase):

    def setUp(self):
        self.monitor = GrowthMonitor()

    def tearDown(self):
        self.monitor.deleteLater()

    def test_reset_displays_locks_correction(self):
        self.monitor._on_correction_toggled(True)
        self.assertTrue(self.monitor._correction_active)
        self.monitor.reset_displays()
        self.assertFalse(self.monitor._correction_active)
        self.assertFalse(self.monitor.correction_btn.isChecked())
        # Button is still enabled after reset — it's a fresh session,
        # correction should be available on the next arm.
        self.assertTrue(self.monitor.correction_btn.isEnabled())

    def test_reset_displays_clears_latest_classifier(self):
        self.monitor.update_classifier_state(_make_classifier_state())
        self.assertIsNotNone(self.monitor._latest_classifier)
        self.monitor.reset_displays()
        self.assertIsNone(self.monitor._latest_classifier)

    def test_set_classifier_disabled_locks_and_disables_button(self):
        self.monitor._on_correction_toggled(True)
        self.monitor.set_classifier_disabled()
        # Correction forced off.
        self.assertFalse(self.monitor._correction_active)
        # Button disabled — correcting a disabled classifier makes no
        # sense (no classifier state to pair with).
        self.assertFalse(self.monitor.correction_btn.isEnabled())


# ---------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    unittest.main(verbosity=2)
