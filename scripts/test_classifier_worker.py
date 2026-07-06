"""Mac-side smoke test for gui.workers.ClassifierWorker.

No torch, no model file, no reference images — uses ``FakeBridge`` to
exercise the run loop, mutex-protected frame handoff, EMA smoothing,
OOD gate, and consecutive-failure tracking in about 1s wall-clock.

Runs via ``python scripts/test_classifier_worker.py`` or under pytest.

Design notes:
    - Signal delivery uses ``Qt.ConnectionType.DirectConnection`` so slots
      run in the emitter's thread — no receiver event loop required, no
      ``processEvents`` gymnastics.
    - Snapshots of ``ClassifierState`` are taken via ``copy.copy`` at
      emit time (the worker reuses one state object across iterations,
      but replaces its dict fields with fresh objects — shallow copy is
      safe here).
    - Per-instance overrides of ``POLL_INTERVAL_S`` and
      ``MAX_CONSECUTIVE_FAILS`` avoid class-level mutation leaking
      between tests. Python's attribute lookup checks the instance
      before the class, so ``self.POLL_INTERVAL_S`` picks up the
      override.
"""
from __future__ import annotations

import copy
import sys
import time
import unittest
from pathlib import Path
from typing import Callable

import numpy as np

# Ensure repo root is on sys.path so gui/ imports resolve when this script
# is executed directly (mirrors the pattern used by test_vimba_camera.py etc).
REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

# QCoreApplication is enough for signal + thread machinery (no widgets
# needed). Create once; safe to reuse across tests.
from PyQt6.QtCore import QCoreApplication, Qt  # noqa: E402

_app = QCoreApplication.instance() or QCoreApplication(sys.argv)

from gui.recon_labels import RECON_LABELS  # noqa: E402
from gui.state import CameraState, ClassifierState  # noqa: E402
from gui.workers import ClassifierWorker  # noqa: E402


# ---------------------------------------------------------------------------
# Test doubles
# ---------------------------------------------------------------------------

class FakeBridge:
    """In-memory replacement for ``ClassifierBridge`` — deterministic, no torch.

    Parameters
    ----------
    scores_seq
        List of ``{label: win_rate}`` dicts. One returned per ``classify``
        call; cycles when exhausted. Defaults to uniform 0.2 across
        ``RECON_LABELS``.
    quality
        Either a single float (returned for every call) or a list of
        floats indexed by call number (last value repeats if the list
        runs out).
    always_raise
        If True, every ``classify`` call raises ``RuntimeError`` — for
        exercising the consecutive-failure counter.
    """

    def __init__(
        self,
        scores_seq: list | None = None,
        quality: float | list = 1.0,
        always_raise: bool = False,
        is_bad: bool = False,
        bad_confidence: float = 0.0,
    ):
        self.scores_seq = scores_seq or [{lbl: 0.2 for lbl in RECON_LABELS}]
        self.quality = quality
        self.always_raise = always_raise
        self.is_bad = is_bad
        self.bad_confidence = bad_confidence

        self.calls = 0
        self.frames_seen: list[np.ndarray] = []

    def classify(self, frame: np.ndarray) -> dict:
        self.calls += 1
        self.frames_seen.append(frame)

        if self.always_raise:
            raise RuntimeError("fake bridge always raises")

        scores = self.scores_seq[(self.calls - 1) % len(self.scores_seq)]

        if isinstance(self.quality, list):
            q = self.quality[min(self.calls - 1, len(self.quality) - 1)]
        else:
            q = float(self.quality)

        predicted = max(scores.items(), key=lambda kv: kv[1])[0]

        return {
            "predicted_class": predicted,
            "classification_scores": scores,
            "is_bad": self.is_bad,
            "bad_confidence": self.bad_confidence,
            "quality": q,
        }


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _wait_for(predicate: Callable[[], bool], timeout: float = 2.0) -> bool:
    """Poll ``predicate`` until it returns True or ``timeout`` elapses.

    Returns True if the predicate fired before timeout, False otherwise.
    Tests should assert on the return value to fail fast on stall.
    """
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(0.005)
    return False


def _snapshot(state: ClassifierState) -> ClassifierState:
    """Shallow copy of a ClassifierState at emit time.

    Safe because the worker replaces dict fields (raw_scores,
    normalized_percent, smoothed_percent) with fresh objects on each
    emission — it never mutates them in place. See module docstring.
    """
    return copy.copy(state)


def _fake_frame() -> np.ndarray:
    """Any 3-channel uint8 array will do; the fake bridge doesn't inspect it."""
    return np.zeros((5, 5, 3), dtype=np.uint8)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class NormalizeTests(unittest.TestCase):
    """Pure-logic tests for the Equalizer-recipe normalization."""

    def test_normalize_preserves_ordering(self):
        scores = {
            "1x1": 0.5, "Twinned (2x1)": 0.3, "c(6x2)": 0.1,
            "rt13xrt13": 0.05, "HTR": 0.05,
        }
        pct, raw_sum = ClassifierWorker._normalize(scores)
        self.assertAlmostEqual(raw_sum, 1.0)
        self.assertEqual(sum(pct.values()), 100)
        # Argmax preserved
        self.assertGreater(pct["1x1"], pct["Twinned (2x1)"])
        self.assertGreater(pct["Twinned (2x1)"], pct["c(6x2)"])

    def test_normalize_clips_negatives(self):
        neg = {
            "1x1": 0.5, "Twinned (2x1)": -0.2, "c(6x2)": 0.3,
            "rt13xrt13": -0.1, "HTR": 0.4,
        }
        pct, raw_sum = ClassifierWorker._normalize(neg)
        # raw_sum reports the actual pre-clip total — transparency signal
        self.assertAlmostEqual(raw_sum, 0.9)
        self.assertEqual(pct["Twinned (2x1)"], 0)
        self.assertEqual(pct["rt13xrt13"], 0)
        # Positive mass renormalizes to ~100 (rounding tolerance)
        self.assertLessEqual(abs(sum(pct.values()) - 100), 1)

    def test_normalize_uniform_fallback_on_all_zero(self):
        zeros = {lbl: 0.0 for lbl in RECON_LABELS}
        pct, raw_sum = ClassifierWorker._normalize(zeros)
        self.assertEqual(raw_sum, 0.0)
        self.assertTrue(all(v == 20 for v in pct.values()))

    def test_normalize_uniform_fallback_on_all_negative(self):
        allneg = {lbl: -0.5 for lbl in RECON_LABELS}
        pct, raw_sum = ClassifierWorker._normalize(allneg)
        # Transparency: raw_sum is negative so UI can warn something's off
        self.assertLess(raw_sum, 0)
        self.assertTrue(all(v == 20 for v in pct.values()))

    def test_normalize_missing_keys_treated_as_zero(self):
        partial = {"1x1": 1.0}  # only one class present
        pct, _ = ClassifierWorker._normalize(partial)
        self.assertEqual(pct["1x1"], 100)
        for k in RECON_LABELS:
            if k != "1x1":
                self.assertEqual(pct[k], 0)


class SlotTests(unittest.TestCase):
    """Tests for the mutex-protected ``on_rheed_state`` slot."""

    def test_slot_stores_latest_frame(self):
        w = ClassifierWorker(ai_repo_root="/nowhere")
        frame = _fake_frame()
        w.on_rheed_state(CameraState(frame=frame, frame_number=7))
        self.assertIs(w._latest_frame, frame)
        self.assertEqual(w._latest_frame_number, 7)

    def test_slot_ignores_none_frame(self):
        w = ClassifierWorker(ai_repo_root="/nowhere")
        w.on_rheed_state(CameraState(frame=None, frame_number=1))
        self.assertIsNone(w._latest_frame)
        self.assertEqual(w._latest_frame_number, -1)

    def test_slot_overwrites_stale_frame(self):
        """Drop-old semantics: newer frame always overwrites."""
        w = ClassifierWorker(ai_repo_root="/nowhere")
        a = np.zeros((5, 5, 3), dtype=np.uint8)
        b = np.ones((5, 5, 3), dtype=np.uint8)
        w.on_rheed_state(CameraState(frame=a, frame_number=1))
        w.on_rheed_state(CameraState(frame=b, frame_number=2))
        self.assertIs(w._latest_frame, b)
        self.assertEqual(w._latest_frame_number, 2)


class RunLoopTests(unittest.TestCase):
    """Tests exercising ``run()`` — real QThread, real signal delivery."""

    def _make_worker(self, bridge: FakeBridge) -> tuple[ClassifierWorker, list]:
        """Build a worker with FakeBridge and a per-instance fast poll interval.

        Returns (worker, states) where states is a list appended to on
        every ``state_updated`` emission (snapshotted).
        """
        w = ClassifierWorker(ai_repo_root="/nowhere")
        w.POLL_INTERVAL_S = 0.01  # instance override — doesn't leak to class
        w._create_bridge = lambda: bridge

        states: list[ClassifierState] = []
        w.state_updated.connect(
            lambda s: states.append(_snapshot(s)),
            Qt.ConnectionType.DirectConnection,  # sync delivery in emitter thread
        )
        return w, states

    def _feed_frame(self, w: ClassifierWorker, frame_number: int) -> None:
        w.on_rheed_state(CameraState(frame=_fake_frame(), frame_number=frame_number))

    def _stop_and_wait(self, w: ClassifierWorker) -> None:
        w.stop()
        self.assertTrue(w.wait(2000), "worker did not exit within 2s of stop()")

    def test_startup_emits_loading_then_ready(self):
        w, states = self._make_worker(FakeBridge())

        w.start()
        try:
            self.assertTrue(_wait_for(lambda: len(states) >= 2))
            self.assertTrue(states[0].loading)
            self.assertFalse(states[0].ready)
            self.assertFalse(states[1].loading)
            self.assertTrue(states[1].ready)
            self.assertEqual(states[1].error, "")
            # Uniform placeholder at ready-time
            self.assertEqual(sum(states[1].smoothed_percent.values()), 100)
            self.assertTrue(all(v == 20 for v in states[1].smoothed_percent.values()))
        finally:
            self._stop_and_wait(w)

    def test_startup_emits_error_on_bridge_failure(self):
        w = ClassifierWorker(ai_repo_root="/nowhere")
        w.POLL_INTERVAL_S = 0.01

        def bad_factory():
            raise RuntimeError("cannot find model at /nowhere/best_model.pth")

        w._create_bridge = bad_factory
        states: list[ClassifierState] = []
        w.state_updated.connect(
            lambda s: states.append(_snapshot(s)),
            Qt.ConnectionType.DirectConnection,
        )

        w.start()
        # Thread ends on its own after emitting the error state
        self.assertTrue(w.wait(2000))
        self.assertGreaterEqual(len(states), 2)
        self.assertFalse(states[-1].loading)
        self.assertFalse(states[-1].ready)
        self.assertIn("cannot find model", states[-1].error)

    def test_first_frame_gets_classified(self):
        scores = {"1x1": 1.0, "Twinned (2x1)": 0.0, "c(6x2)": 0.0,
                  "rt13xrt13": 0.0, "HTR": 0.0}
        bridge = FakeBridge(scores_seq=[scores], quality=0.9)
        w, states = self._make_worker(bridge)

        w.start()
        try:
            self.assertTrue(_wait_for(lambda: len(states) >= 2))  # loading + ready
            self._feed_frame(w, 1)
            self.assertTrue(_wait_for(lambda: bridge.calls >= 1))
            self.assertTrue(_wait_for(
                lambda: any(s.last_frame_number == 1 for s in states)
            ))

            post = [s for s in states if s.last_frame_number == 1][-1]
            self.assertEqual(bridge.calls, 1)
            self.assertAlmostEqual(post.raw_sum, 1.0)
            # Normalized should peak at 1x1 (was the only positive score)
            self.assertEqual(post.normalized_percent["1x1"], 100)
        finally:
            self._stop_and_wait(w)

    def test_frame_dedup(self):
        """Feeding the same frame_number twice → classify called only once."""
        bridge = FakeBridge()
        w, states = self._make_worker(bridge)

        w.start()
        try:
            self.assertTrue(_wait_for(lambda: len(states) >= 2))
            self._feed_frame(w, 42)
            self.assertTrue(_wait_for(lambda: bridge.calls >= 1))

            # Same frame_number → no new classify
            self._feed_frame(w, 42)
            time.sleep(0.1)  # give the loop several iterations to observe
            self.assertEqual(bridge.calls, 1)

            # New frame_number → new classify
            self._feed_frame(w, 43)
            self.assertTrue(_wait_for(lambda: bridge.calls >= 2))
            self.assertEqual(bridge.calls, 2)
        finally:
            self._stop_and_wait(w)

    def test_ema_smoothing_progresses(self):
        """Sustained high-confidence 100% for 1x1 → smoothed ramps toward 100."""
        scores = {"1x1": 1.0, "Twinned (2x1)": 0.0, "c(6x2)": 0.0,
                  "rt13xrt13": 0.0, "HTR": 0.0}
        bridge = FakeBridge(scores_seq=[scores], quality=0.9)
        w, states = self._make_worker(bridge)

        w.start()
        try:
            self.assertTrue(_wait_for(lambda: len(states) >= 2))

            progression: list[int] = []
            for i in range(1, 16):
                self._feed_frame(w, i)
                self.assertTrue(_wait_for(
                    lambda i=i: any(s.last_frame_number == i for s in states)
                ))
                latest_i = [s for s in states if s.last_frame_number == i][-1]
                progression.append(latest_i.smoothed_percent["1x1"])
        finally:
            self._stop_and_wait(w)

        # First inference at alpha=0.2 blending 20 (uniform) with 100 → 36
        self.assertGreater(progression[0], 20)
        self.assertLess(progression[0], 100)
        # Monotonic increase toward 100
        self.assertGreater(progression[-1], progression[0])
        # After ~15 cycles: 20 + 80*(1 - 0.8^15) ≈ 96
        self.assertGreater(progression[-1], 80)

    def test_ood_freezes_ema(self):
        """Low-quality frames trigger is_ood and DON'T advance the EMA."""
        good = {"1x1": 1.0, "Twinned (2x1)": 0.0, "c(6x2)": 0.0,
                "rt13xrt13": 0.0, "HTR": 0.0}
        # Diametrically opposite scores — if EMA were advancing, smoothed
        # would swing toward Twinned. Test asserts it does NOT.
        bad_shape = {"1x1": 0.0, "Twinned (2x1)": 1.0, "c(6x2)": 0.0,
                     "rt13xrt13": 0.0, "HTR": 0.0}
        bridge = FakeBridge(
            scores_seq=[good, good, good, bad_shape, bad_shape, bad_shape],
            # First 3 quality=0.9 (non-OOD), next 3 quality=0.1 (below 0.3 threshold)
            quality=[0.9, 0.9, 0.9, 0.1, 0.1, 0.1],
        )
        w, states = self._make_worker(bridge)

        w.start()
        try:
            self.assertTrue(_wait_for(lambda: len(states) >= 2))
            for i in range(1, 7):
                self._feed_frame(w, i)
                self.assertTrue(_wait_for(
                    lambda i=i: any(s.last_frame_number == i for s in states)
                ))
        finally:
            self._stop_and_wait(w)

        # Verify is_ood flag flipped at frame 4
        for i in range(1, 4):
            frame_i = [s for s in states if s.last_frame_number == i][-1]
            self.assertFalse(frame_i.is_ood, f"frame {i} should NOT be OOD")
        for i in range(4, 7):
            frame_i = [s for s in states if s.last_frame_number == i][-1]
            self.assertTrue(frame_i.is_ood, f"frame {i} SHOULD be OOD")

        # Smoothed_percent frozen during OOD — equals the last non-OOD emission
        last_non_ood = [s for s in states if s.last_frame_number == 3][-1]
        for i in range(4, 7):
            ood = [s for s in states if s.last_frame_number == i][-1]
            self.assertEqual(
                ood.smoothed_percent, last_non_ood.smoothed_percent,
                f"OOD frame {i} should have frozen smoothed_percent",
            )

    def test_consecutive_failures_emit_error(self):
        """After MAX_CONSECUTIVE_FAILS raises, an error state is emitted."""
        bridge = FakeBridge(always_raise=True)
        w, states = self._make_worker(bridge)
        w.MAX_CONSECUTIVE_FAILS = 3  # instance override — faster than default 5

        w.start()
        try:
            self.assertTrue(_wait_for(lambda: len(states) >= 2))  # loading + ready
            for i in range(1, 6):
                self._feed_frame(w, i)
                time.sleep(0.03)  # give run loop time to see each frame
            self.assertTrue(_wait_for(lambda: any(s.error for s in states)))
        finally:
            self._stop_and_wait(w)

        error_states = [s for s in states if s.error]
        self.assertGreater(len(error_states), 0)
        self.assertIn("Classifier failed", error_states[-1].error)

    def test_stop_exits_run_loop(self):
        """Clean stop → thread exits within the poll interval + slack.

        Waits for the ready-state emission (not just isRunning()) before
        stopping: the codebase-wide worker pattern sets ``self.running =
        True`` at the top of ``run()``, so an early ``stop()`` racing
        with that assignment can leave ``running`` stuck at True. Waiting
        for a real emission guarantees ``run()`` is past that line.
        """
        bridge = FakeBridge()
        w, states = self._make_worker(bridge)

        w.start()
        try:
            self.assertTrue(_wait_for(lambda: len(states) >= 2))  # past the race
            self.assertTrue(w.isRunning())
        finally:
            w.stop()
            self.assertTrue(w.wait(2000))
            self.assertFalse(w.isRunning())


if __name__ == "__main__":
    unittest.main(verbosity=2)
