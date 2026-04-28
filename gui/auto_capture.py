"""
Auto-capture engine for MBE Growth Monitor.

Watches incoming RHEED frames for significant changes and emits a signal
when the change score exceeds a configurable threshold (with debounce and
cooldown).  Designed around a pluggable ChangeDetector ABC so higher-tier
strategies (SimCLR embedding distance, Classifier2 output change) can drop
in without refactoring.
"""
from __future__ import annotations

import collections
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
        """Return a score indicating how different this frame is from reference.

        Score range is detector-specific. Tier 1/2 detectors return values on
        the 0-255 absolute pixel-intensity scale; Tier 3 returns 0-1.
        """
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


class PixelDiffChangeDetector(ChangeDetector):
    """Tier 1.5: Mean absolute pixel diff against a FIFO buffer of recent frames.

    Maintains a deque of the last ``buffer_size`` grayscale frames. Each new
    frame is scored as the mean absolute pixel difference between it and the
    *mean* of the buffer. Score is then smoothed by a rolling mean over the
    last ``smooth_window`` raw scores.

    Compared to ``IntensityChangeDetector`` (mean intensity only), this
    catches spatial pattern changes — the actual signal in reconstruction
    transitions — not just global brightness shifts.

    Threshold tuning notes (against Rahim's 2022_02_04 STO trajectory using
    diff-vs-previous-frame): baseline ~0.5, real reconstruction events peak
    at 2.5-9.0. Diff-vs-buffer-mean produces *larger* peaks for the same
    transitions (full delta vs rate of delta), so an in-GUI threshold of
    2.0-2.5 is a reasonable starting point. Re-tune offline against the
    same dataset with the buffer-mean variant before relying on the value.
    """

    def __init__(self, buffer_size: int = 20, smooth_window: int = 3):
        self._buffer_size = buffer_size
        self._smooth_window = smooth_window
        self._buffer: collections.deque[np.ndarray] = collections.deque(
            maxlen=buffer_size,
        )
        # Running sum of buffer contents — O(1) buffer-mean updates rather
        # than O(buffer_size) per frame.
        self._sum: np.ndarray | None = None
        self._recent_scores: collections.deque[float] = collections.deque(
            maxlen=max(1, smooth_window),
        )

    def reset(self) -> None:
        self._buffer.clear()
        self._sum = None
        self._recent_scores.clear()

    def compute_score(self, frame: np.ndarray) -> float:
        gray = self._to_gray(frame)

        # Defensive reset if the camera resolution changed mid-session;
        # the running sum is invalid against a different shape.
        if self._sum is not None and gray.shape != self._sum.shape:
            self.reset()

        if not self._buffer:
            self._buffer.append(gray)
            self._sum = gray.copy()
            self._recent_scores.append(0.0)
            return 0.0

        buffer_mean = self._sum / len(self._buffer)
        raw_score = float(np.mean(np.abs(gray - buffer_mean)))

        # Update running sum: subtract the about-to-be-evicted frame
        # before deque.append silently drops it.
        if len(self._buffer) == self._buffer_size:
            self._sum -= self._buffer[0]
        self._buffer.append(gray)
        self._sum += gray

        self._recent_scores.append(raw_score)
        return float(sum(self._recent_scores) / len(self._recent_scores))

    @staticmethod
    def _to_gray(frame: np.ndarray) -> np.ndarray:
        """Convert frame to float32 grayscale.

        For RGB inputs (kSA false-color screengrabs), takes the green
        channel — RHEED intensity lives there per project convention.
        """
        if frame.ndim == 2:
            return frame.astype(np.float32)
        return frame[:, :, 1].astype(np.float32)


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
        context_buffer_size: int = 20,
        adaptive_sigma: float | None = None,
        adaptive_history: int = 100,
        adaptive_warmup: int = 20,
        adaptive_floor: float = 1.0,
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

        # Pre-event ring buffer of full-resolution RGB frames. Maintained
        # in parallel with the detector's internal grayscale buffer so that
        # when frame_captured fires we can dump the visual context that
        # led up to the trigger. Sized in frames; at ~10 Hz, 20 ≈ 2 s.
        self._context_buffer: collections.deque[np.ndarray] = collections.deque(
            maxlen=context_buffer_size,
        )

        # Adaptive thresholding: when adaptive_sigma is set, the trigger
        # threshold becomes max(adaptive_floor, μ + Nσ) over a rolling
        # window of recent below-threshold scores. Cross-dataset validation
        # on Rahim's STO trajectories showed real events varying by an
        # order of magnitude in raw score, with stable baselines — adaptive
        # generalizes better than a fixed cutoff. Set to None to keep the
        # original fixed-threshold behavior.
        self._adaptive_sigma = adaptive_sigma
        self._adaptive_history = adaptive_history
        self._adaptive_warmup = adaptive_warmup
        self._adaptive_floor = adaptive_floor
        self._baseline_scores: collections.deque[float] = collections.deque(
            maxlen=adaptive_history,
        )

    # -- Public API ---------------------------------------------------------

    @property
    def enabled(self) -> bool:
        return self._enabled

    @enabled.setter
    def enabled(self, value: bool) -> None:
        self._enabled = value

    @property
    def threshold(self) -> float:
        """The fixed-threshold value (used when adaptive is off, or as a
        fallback during the adaptive warmup)."""
        return self._threshold

    @threshold.setter
    def threshold(self, value: float) -> None:
        self._threshold = value

    @property
    def effective_threshold(self) -> float:
        """The threshold actually applied this cycle.

        Adaptive (μ + Nσ over the rolling baseline) when adaptive_sigma is
        configured AND the baseline has filled to at least adaptive_warmup
        samples. Falls back to the fixed threshold during warmup so the
        detector behaves predictably in the first ~30 frames of a session.
        Always clamped to adaptive_floor to prevent runaway sensitivity in
        pathologically quiet sessions.
        """
        if (
            self._adaptive_sigma is None
            or len(self._baseline_scores) < self._adaptive_warmup
        ):
            return self._threshold
        arr = np.asarray(self._baseline_scores, dtype=np.float64)
        return max(
            self._adaptive_floor,
            float(arr.mean() + self._adaptive_sigma * arr.std()),
        )

    @property
    def latest_score(self) -> float:
        return self._latest_score

    def set_detector(self, detector: ChangeDetector) -> None:
        """Swap in a different detection strategy (Tier 2/3)."""
        self._detector = detector

    def get_recent_frames(self) -> list[np.ndarray]:
        """Snapshot of the context buffer (oldest → newest), defensively copied.

        Used by the caller after frame_captured to dump the visual context
        leading up to a flagged event. Returns an empty list before any
        frames have been evaluated.
        """
        return [f.copy() for f in self._context_buffer]

    def reset(self) -> None:
        """Reset internal counters (call on session start)."""
        self._detector.reset()
        self._frame_count = 0
        self._last_capture_time = 0.0
        self._debounce_count = 0
        self._latest_score = 0.0
        self._context_buffer.clear()
        self._baseline_scores.clear()

    def evaluate(self, frame: np.ndarray) -> None:
        """Called once per camera frame. Emits *frame_captured* if triggered."""
        if not self._enabled:
            return

        # Populate the context buffer regardless of warmup state — when a
        # trigger fires shortly after warmup ends, we want pre-event context
        # from the warmup window itself.
        self._context_buffer.append(frame.copy())

        self._frame_count += 1
        if self._frame_count <= self._warmup_frames:
            self._detector.compute_score(frame)  # feed baseline
            self._latest_score = 0.0
            return

        score = self._detector.compute_score(frame)
        self._latest_score = score
        now = time.time()

        threshold = self.effective_threshold
        if score >= threshold:
            self._debounce_count += 1
        else:
            self._debounce_count = 0
            # Only non-flagged scores feed the adaptive baseline — events
            # would pollute the rolling mean and pull the threshold up
            # behind their own peak. The fixed-mode path appends too,
            # which is harmless (the deque is just unused).
            self._baseline_scores.append(score)

        if (
            self._debounce_count >= self._debounce_required
            and (now - self._last_capture_time) >= self._cooldown_s
        ):
            self._last_capture_time = now
            self._debounce_count = 0
            self.frame_captured.emit(frame.copy(), score)
