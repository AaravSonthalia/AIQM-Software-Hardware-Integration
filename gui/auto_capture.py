"""
Auto-capture engine for MBE Growth Monitor.

Watches incoming RHEED frames for significant changes and emits a signal
when the change score exceeds a configurable threshold (with debounce and
cooldown).  Designed around a pluggable ChangeDetector ABC so higher-tier
strategies (SimCLR embedding distance, Classifier2 output change) can drop
in without refactoring.
"""
from __future__ import annotations

import time
from abc import ABC, abstractmethod

import numpy as np
from PyQt6.QtCore import QObject, pyqtSignal


# ---------------------------------------------------------------------------
# ChangeDetector ABC + implementations
# ---------------------------------------------------------------------------

class ChangeDetector(ABC):
    """Base class for frame change detection strategies."""

    @abstractmethod
    def reset(self) -> None: ...

    @abstractmethod
    def compute_score(self, frame: np.ndarray) -> float:
        """Return a 0-1 score indicating how different this frame is from reference."""
        ...


class IntensityChangeDetector(ChangeDetector):
    """Tier 1: Global mean pixel intensity comparison."""

    def __init__(self):
        self._reference_intensity: float | None = None

    def reset(self) -> None:
        self._reference_intensity = None

    def compute_score(self, frame: np.ndarray) -> float:
        gray = frame if frame.ndim == 2 else frame.mean(axis=2)
        current = float(gray.mean())
        if self._reference_intensity is None:
            self._reference_intensity = current
            return 0.0
        delta = abs(current - self._reference_intensity) / max(self._reference_intensity, 1e-6)
        self._reference_intensity = current  # rolling reference
        return delta


# Future Tier 2:
# class EmbeddingChangeDetector(ChangeDetector):
#     """Cosine distance in SimCLR embedding space."""
#     def compute_score(self, frame): ...


class ClassificationChangeDetector(ChangeDetector):
    """Tier 3: Triggers when Classifier2 output distribution changes.

    Uses Jensen-Shannon divergence between the current and previous
    classification probability distributions.  Returns 1.0 when the
    argmax label flips; otherwise returns the JS-divergence (0-1).

    Unlike Tiers 1/2, this detector does NOT run the model itself.
    The caller must feed pre-computed classification scores via
    :meth:`set_scores` before calling :meth:`compute_score`.
    """

    def __init__(self):
        self._prev_scores: np.ndarray | None = None
        self._prev_label: int | None = None
        # Latest scores set by the caller (GrowthApp after inference).
        self._current_scores: np.ndarray | None = None

    def reset(self) -> None:
        self._prev_scores = None
        self._prev_label = None
        self._current_scores = None

    def set_scores(self, scores: list[float]) -> None:
        """Provide the latest Classifier2 win-rate scores (len 5)."""
        self._current_scores = np.asarray(scores, dtype=np.float64)

    def compute_score(self, frame: np.ndarray) -> float:  # noqa: ARG002 — frame unused
        """Return change score.  ``frame`` is accepted for ABC compat but unused."""
        if self._current_scores is None:
            return 0.0

        cur = self._current_scores
        cur_label = int(np.argmax(cur))

        if self._prev_scores is None:
            self._prev_scores = cur.copy()
            self._prev_label = cur_label
            return 0.0

        # Hard trigger: argmax label changed → score = 1.0
        if cur_label != self._prev_label:
            self._prev_scores = cur.copy()
            self._prev_label = cur_label
            return 1.0

        # Soft trigger: JS-divergence between distributions.
        score = float(self._js_divergence(self._prev_scores, cur))
        self._prev_scores = cur.copy()
        self._prev_label = cur_label
        return score

    @staticmethod
    def _js_divergence(p: np.ndarray, q: np.ndarray) -> float:
        """Jensen-Shannon divergence (base-2, range 0-1)."""
        # Normalise to probability distributions.
        p = np.clip(p, 1e-12, None)
        q = np.clip(q, 1e-12, None)
        p = p / p.sum()
        q = q / q.sum()
        m = 0.5 * (p + q)
        kl_pm = float(np.sum(p * np.log2(p / m)))
        kl_qm = float(np.sum(q * np.log2(q / m)))
        return 0.5 * (kl_pm + kl_qm)


# ---------------------------------------------------------------------------
# AutoCaptureEngine
# ---------------------------------------------------------------------------

class AutoCaptureEngine(QObject):
    """Evaluates each camera frame and emits *frame_captured* when a
    significant change is detected (after debounce, respecting cooldown)."""

    frame_captured = pyqtSignal(np.ndarray, float)  # (frame, change_score)

    def __init__(
        self,
        threshold: float = 0.20,
        cooldown_s: float = 5.0,
        warmup_frames: int = 30,
        parent=None,
    ):
        super().__init__(parent)
        self._detector: ChangeDetector = IntensityChangeDetector()
        self._threshold = threshold
        self._cooldown_s = cooldown_s
        self._warmup_frames = warmup_frames

        self._frame_count = 0
        self._last_capture_time = 0.0
        self._enabled = False
        self._debounce_count = 0
        self._debounce_required = 3  # consecutive frames above threshold
        self._latest_score = 0.0

    # -- Public API ---------------------------------------------------------

    @property
    def enabled(self) -> bool:
        return self._enabled

    @enabled.setter
    def enabled(self, value: bool) -> None:
        self._enabled = value

    @property
    def threshold(self) -> float:
        return self._threshold

    @threshold.setter
    def threshold(self, value: float) -> None:
        self._threshold = value

    @property
    def latest_score(self) -> float:
        return self._latest_score

    def set_detector(self, detector: ChangeDetector) -> None:
        """Swap in a different detection strategy (Tier 2/3)."""
        self._detector = detector

    def reset(self) -> None:
        """Reset internal counters (call on session start)."""
        self._detector.reset()
        self._frame_count = 0
        self._last_capture_time = 0.0
        self._debounce_count = 0
        self._latest_score = 0.0

    def evaluate(self, frame: np.ndarray) -> None:
        """Called once per camera frame. Emits *frame_captured* if triggered."""
        if not self._enabled:
            return

        self._frame_count += 1
        if self._frame_count <= self._warmup_frames:
            self._detector.compute_score(frame)  # feed baseline
            self._latest_score = 0.0
            return

        score = self._detector.compute_score(frame)
        self._latest_score = score
        now = time.time()

        if score >= self._threshold:
            self._debounce_count += 1
        else:
            self._debounce_count = 0

        if (
            self._debounce_count >= self._debounce_required
            and (now - self._last_capture_time) >= self._cooldown_s
        ):
            self._last_capture_time = now
            self._debounce_count = 0
            self.frame_captured.emit(frame.copy(), score)
