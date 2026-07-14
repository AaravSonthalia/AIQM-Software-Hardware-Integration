"""
Live Equalizer tab — real-time RHEED labeling as a first-class GUI tab.

Ships workstream #4 from the Jul 10 2026 group-meeting queue. Per PI
direction (Jul 10 2026 same-day pivot): NOT a popup / separate window,
but a full tab in the Growth Monitor alongside Monitor / Direct-read /
Events / Scrubber / Session.

Layout — 2×2 grid:
  ┌─────────────────────────┬─────────────────────────┐
  │ Selected (live RHEED)   │ Constructed (blend)    │
  │ [live camera frame]     │ [reconstruction image] │
  ├─────────────────────────┼─────────────────────────┤
  │ Classifier %:           │ Grower %:              │
  │ 1x1        60%          │ 1x1     [═══◆═══] 60% │
  │ Twinned    20%          │ Twinned [═◆═════] 20% │
  │ c(6x2)     10%          │ c(6x2)  [◆═══════] 10% │
  │ rt13xrt13   5%          │ rt13    [◆═══════]  5% │
  │ HTR         5%          │ HTR     [◆═══════]  5% │
  │                         │ [Auto-fit][Norm][Reset]│
  │                         │ [                Save ]│
  └─────────────────────────┴─────────────────────────┘

Reuses ``scripts/equalizer_ui.py`` helpers verbatim:
  - ``PROCESS_WH`` (128×96) — internal processing resolution for the
    basis + target so pixels are comparable.
  - ``DISPLAY_WH`` (520×390) — upscaled display size in both panes.
  - ``load_class_means(target_wh)`` — the 5-class basis (npz cache or
    training-set means).
  - ``auto_fit(means, target)`` — least-squares fit clipped to [0, 1]
    with normalization.
  - ``array_to_pixmap(arr, display_wh)`` — grayscale → phosphor-green
    QPixmap (matches kSA's BGW ramp so growers see the same visual
    identity as the live RHEED they know).

Data flow:
  1. Camera frame → ``update_camera_frame(np.ndarray)`` (called by
     GrowthApp on ``camera_worker.state_updated``). Downsamples to
     PROCESS_WH grayscale + updates the 'Selected' pane.
  2. Classifier state → ``update_classifier_state(ClassifierState)``.
     Populates the 5 read-only classifier % labels. Mirrors the
     Monitor tab's classifier slider values.
  3. Grower drags sliders → constructed pane rebuilds via
     ``_update_reconstruction`` (weighted sum of basis images).
  4. Grower clicks Save → ``live_label_save_requested`` signal fires
     with the weights dict. GrowthApp handles the sensor snapshot +
     ``GrowthLogger.record_live_label`` call.

Design notes:
  - Class label vocabulary matches ``scripts/equalizer_ui.py``
    (``1x1``, ``Tw(2x1)``, ``c(6x2)``, ``RT13``, ``HTR``) NOT the
    Monitor tab's ``RECON_LABELS`` (``Twinned (2x1)``, ``rt13xrt13``).
    The two label sets are the same 5 reconstructions with different
    spellings; ``CLASSIFIER_LABEL_MAP`` bridges them so the classifier's
    smoothed_percent (keyed by RECON_LABELS) drives the classifier %
    display (keyed by our shorter names).
  - Pause via Freeze frame button (Jul 14 2026). When toggled on,
    ``update_camera_frame`` drops incoming frames so the Selected pane
    holds the last-displayed frame. Grower can then balance sliders +
    Save against a stable image. Label + Auto-fit + Save all operate on
    the frozen frame. Amber (#d97706) match the MARK EVENT identity so
    grower-in-the-loop UI has consistent visual cues across tabs.
  - No auto-save. Saving is explicit; the grower decides when the mix
    matches the target.
"""
from __future__ import annotations

import sys
from pathlib import Path
from typing import Optional

import numpy as np
from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QPixmap
from PyQt6.QtWidgets import (
    QGridLayout, QGroupBox, QHBoxLayout, QLabel, QPushButton, QSlider,
    QSizePolicy, QVBoxLayout, QWidget,
)

from gui.state import ClassifierState


# scripts/ isn't part of the gui/ package — add repo root to sys.path so
# ``scripts.equalizer_ui`` imports resolve regardless of launch path.
# events_tab.py already uses this pattern for the retrospective launcher.
_REPO_ROOT = str(Path(__file__).resolve().parent.parent)
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)


# 5 reconstruction classes. Labels intentionally match the Equalizer's
# vocabulary (``Tw(2x1)`` short form, ``RT13`` uppercase) not the Monitor
# tab's ``RECON_LABELS`` — see docstring. The CLASSIFIER_LABEL_MAP below
# translates when we consume classifier state.
CLASS_LABELS = ["1x1", "Tw(2x1)", "c(6x2)", "RT13", "HTR"]

# Monitor-tab ``RECON_LABELS`` → Equalizer ``CLASS_LABELS``. The classifier's
# ``smoothed_percent`` keys are the Monitor spellings; the tab's classifier
# % display uses the Equalizer spellings so it visually pairs with the
# grower sliders on the right.
CLASSIFIER_LABEL_MAP = {
    "1x1":            "1x1",
    "Twinned (2x1)":  "Tw(2x1)",
    "c(6x2)":         "c(6x2)",
    "rt13xrt13":      "RT13",
    "HTR":            "HTR",
}


class LiveEqualizerTab(QWidget):
    """Live RHEED Equalizer tab. See module docstring for the full design."""

    live_label_save_requested = pyqtSignal(dict)

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._basis: dict[str, np.ndarray] = {}
        self._basis_error: Optional[str] = None
        self._current_target: Optional[np.ndarray] = None
        self._current_full_frame: Optional[np.ndarray] = None
        self._sliders: dict[str, QSlider] = {}
        self._slider_value_labels: dict[str, QLabel] = {}
        self._classifier_value_labels: dict[str, QLabel] = {}
        # Reentrancy guard for the reconstruction update — setting slider
        # values programmatically (e.g. Auto-fit) fires valueChanged for
        # each slider, which recomputes reconstruction each time. Guarding
        # keeps the recompute to once per user action.
        self._adjusting = False
        # Pause state (Jul 14 2026). When True, update_camera_frame drops
        # incoming frames on the floor so the 'Selected' pane holds
        # whatever was last displayed. Grower can then Auto-fit / Save
        # against a stable image. Toggled via _pause_btn's checked state.
        self._paused = False

        self._build_ui()
        self._load_basis()
        self._reset_to_uniform()

    # ----- Basis loading --------------------------------------------------

    def _load_basis(self):
        """Load the 5-class basis via scripts.equalizer_ui.load_class_means.

        Failure modes handled: missing training data + missing cache +
        missing simulated basis. On failure the tab still builds but
        the Constructed pane stays empty + Auto-fit / Save don't do
        anything meaningful. Grower sees the error hint in the
        Constructed pane's placeholder text.
        """
        try:
            from scripts.equalizer_ui import PROCESS_WH, load_class_means
            self._basis = load_class_means(PROCESS_WH)
            if not self._basis:
                self._basis_error = (
                    "Basis images not found — Constructed pane disabled."
                )
                self._constructed_label.setText(self._basis_error)
        except Exception as exc:  # broad — the loader has multiple fallbacks
            self._basis_error = f"Basis load failed: {exc}"
            self._constructed_label.setText(self._basis_error)

    # ----- Layout ---------------------------------------------------------

    def _build_ui(self):
        # Import display size lazily so the module still imports if
        # scripts.equalizer_ui has an error — the tab shows a placeholder
        # instead of failing GUI construction.
        try:
            from scripts.equalizer_ui import DISPLAY_WH
        except Exception:
            DISPLAY_WH = (520, 390)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(8, 8, 8, 8)
        outer.setSpacing(8)

        title = QLabel(
            "Live RHEED Equalizer — drag sliders until the Constructed "
            "blend matches the Selected live frame, then Save."
        )
        title.setStyleSheet("font-size: 12px; color: #aaa;")
        outer.addWidget(title)

        # === Top row: Selected (live) | Constructed (grower blend) ===
        images_row = QHBoxLayout()
        images_row.setSpacing(12)

        # --- Selected (live RHEED) ---
        selected_group = QGroupBox("Selected (live RHEED)")
        selected_layout = QVBoxLayout(selected_group)
        selected_layout.setContentsMargins(8, 12, 8, 8)
        self._selected_label = QLabel("Waiting for live camera stream…")
        self._selected_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._selected_label.setMinimumSize(DISPLAY_WH[0], DISPLAY_WH[1])
        self._selected_label.setStyleSheet(
            "background-color: #111; color: #888; border: 1px solid #333;"
        )
        self._selected_label.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding,
        )
        selected_layout.addWidget(self._selected_label, 1)
        images_row.addWidget(selected_group, 1)

        # --- Constructed (grower blend) ---
        constructed_group = QGroupBox("Constructed (grower blend)")
        constructed_layout = QVBoxLayout(constructed_group)
        constructed_layout.setContentsMargins(8, 12, 8, 8)
        self._constructed_label = QLabel("Uniform mix — drag sliders below.")
        self._constructed_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._constructed_label.setMinimumSize(DISPLAY_WH[0], DISPLAY_WH[1])
        self._constructed_label.setStyleSheet(
            "background-color: #111; color: #888; border: 1px solid #333;"
        )
        self._constructed_label.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding,
        )
        constructed_layout.addWidget(self._constructed_label, 1)
        images_row.addWidget(constructed_group, 1)

        outer.addLayout(images_row, 1)

        # === Bottom row: Classifier % | Grower sliders ===
        bottom_row = QHBoxLayout()
        bottom_row.setSpacing(12)

        # --- Classifier % breakdown ---
        classifier_group = QGroupBox("Classifier % breakdown")
        cls_layout = QGridLayout(classifier_group)
        cls_layout.setContentsMargins(12, 16, 12, 12)
        cls_layout.setHorizontalSpacing(12)
        cls_layout.setVerticalSpacing(6)
        for i, label in enumerate(CLASS_LABELS):
            name_label = QLabel(label)
            name_label.setStyleSheet("font-size: 12px;")
            cls_layout.addWidget(name_label, i, 0)
            value_label = QLabel("—%")
            value_label.setStyleSheet(
                "font-size: 12px; color: #0d9488; font-weight: bold;"
            )
            value_label.setAlignment(Qt.AlignmentFlag.AlignRight)
            cls_layout.addWidget(value_label, i, 1)
            self._classifier_value_labels[label] = value_label
        cls_layout.setColumnStretch(0, 1)
        cls_layout.setRowStretch(len(CLASS_LABELS), 1)  # push rows to top
        bottom_row.addWidget(classifier_group, 1)

        # --- Grower % (sliders + buttons) ---
        grower_group = QGroupBox("Grower % breakdown")
        grower_layout = QGridLayout(grower_group)
        grower_layout.setContentsMargins(12, 16, 12, 12)
        grower_layout.setHorizontalSpacing(10)
        grower_layout.setVerticalSpacing(6)
        grower_layout.setColumnStretch(1, 1)  # slider column expands
        for i, label in enumerate(CLASS_LABELS):
            name_label = QLabel(label)
            name_label.setStyleSheet("font-size: 12px;")
            name_label.setMinimumWidth(70)
            grower_layout.addWidget(name_label, i, 0)

            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(0, 100)
            slider.setTickPosition(QSlider.TickPosition.TicksBelow)
            slider.setTickInterval(10)
            slider.valueChanged.connect(self._on_slider_changed)
            grower_layout.addWidget(slider, i, 1)
            self._sliders[label] = slider

            value_label = QLabel("0%")
            value_label.setStyleSheet(
                "font-size: 12px; color: #d97706; font-weight: bold;"
            )
            value_label.setAlignment(Qt.AlignmentFlag.AlignRight)
            value_label.setMinimumWidth(45)
            grower_layout.addWidget(value_label, i, 2)
            self._slider_value_labels[label] = value_label

        # Action button row spans all columns beneath the sliders.
        btn_row = QHBoxLayout()
        btn_row.setSpacing(6)
        self._auto_fit_btn = QPushButton("Auto-fit")
        self._auto_fit_btn.setToolTip(
            "Least-squares fit the sliders to the current 'Selected' image."
        )
        self._auto_fit_btn.clicked.connect(self._on_auto_fit)
        btn_row.addWidget(self._auto_fit_btn)

        self._normalize_btn = QPushButton("Normalize")
        self._normalize_btn.setToolTip(
            "Rescale the sliders so they sum to 100%."
        )
        self._normalize_btn.clicked.connect(self._on_normalize)
        btn_row.addWidget(self._normalize_btn)

        self._reset_btn = QPushButton("Reset")
        self._reset_btn.setToolTip("Back to uniform 20/20/20/20/20.")
        self._reset_btn.clicked.connect(self._reset_to_uniform)
        btn_row.addWidget(self._reset_btn)

        # Pause button (Jul 14 2026). Checkable so Qt tracks the visual
        # state without us maintaining a duplicate flag. Amber (#d97706)
        # when checked matches the MARK EVENT identity + Manual-events
        # footer counter — a grower looking at any of the three "amber
        # is grower-in-the-loop" surfaces sees the same visual cue.
        # Label flips between "Freeze frame" (pressing will freeze) and
        # "Resume live" (pressing will resume) so the button describes
        # the action, not the current state.
        self._pause_btn = QPushButton("Freeze frame")
        self._pause_btn.setCheckable(True)
        self._pause_btn.setToolTip(
            "Freeze the 'Selected' pane on the current frame so you can "
            "balance sliders without the image changing under you. Auto-"
            "fit + Save still work against the frozen frame."
        )
        self._pause_btn.setStyleSheet(
            "QPushButton { padding: 4px 12px; }"
            "QPushButton:checked { background-color: #d97706; color: white; "
            "font-weight: bold; }"
        )
        self._pause_btn.toggled.connect(self._on_pause_toggled)
        btn_row.addWidget(self._pause_btn)

        btn_row.addStretch(1)

        self._save_btn = QPushButton("Save label")
        self._save_btn.setStyleSheet(
            "QPushButton { background-color: #0d9488; color: white; "
            "font-size: 13px; font-weight: bold; padding: 6px 16px; }"
            "QPushButton:disabled { background-color: #222; color: #666; }"
        )
        self._save_btn.setToolTip(
            "Snapshot the current live frame + save the slider weights to "
            "live_labels.csv. Requires an active session."
        )
        self._save_btn.setEnabled(False)  # Enabled by GrowthApp on session start
        self._save_btn.clicked.connect(self._on_save)
        btn_row.addWidget(self._save_btn)

        grower_layout.addLayout(
            btn_row, len(CLASS_LABELS), 0, 1, 3,
        )
        bottom_row.addWidget(grower_group, 1)

        outer.addLayout(bottom_row, 1)

    # ----- Public methods (called by GrowthApp) ---------------------------

    def update_camera_frame(self, frame: Optional[np.ndarray]):
        """Update the 'Selected' pane with a new live camera frame.

        Frame is expected as ``(H, W, 3)`` uint8 RGB (post-palette-fix
        format from ``drivers/rheed_camera.py``). Handles grayscale
        (2D) input too. Downsamples to PROCESS_WH grayscale for the
        basis-matched processing space and stores the full-res original
        for GrowthApp to snapshot on Save.

        Silently no-ops on decode error (rare — camera may deliver a
        weird frame under stress). The last successful target stays.

        When ``self._paused`` is True (Freeze frame toggled on), the
        incoming frame is dropped on the floor — cache + displayed
        pixmap both preserve whatever was current at pause time. That
        stable state is what Auto-fit / Save see.
        """
        if frame is None:
            return
        if self._paused:
            return
        self._current_full_frame = frame
        try:
            from PIL import Image
            from scripts.equalizer_ui import PROCESS_WH, DISPLAY_WH, array_to_pixmap
            img = Image.fromarray(frame)
            if img.mode != "L":
                img = img.convert("L")
            img = img.resize(PROCESS_WH, Image.LANCZOS)
            self._current_target = np.asarray(img, dtype=np.float32)
            self._selected_label.setPixmap(
                array_to_pixmap(self._current_target, DISPLAY_WH),
            )
        except Exception:
            # Don't raise from the camera hot path.
            return

    def update_classifier_state(self, state: Optional[ClassifierState]):
        """Populate the 5 classifier % labels from ClassifierState.

        Mirrors the Monitor tab's classifier slider read. Uses
        CLASSIFIER_LABEL_MAP to translate the classifier's smoothed_percent
        keys (Monitor spellings) into this tab's short-form class labels.
        """
        if state is None or not state.smoothed_percent:
            for lbl in self._classifier_value_labels.values():
                lbl.setText("—%")
            return
        smoothed = state.smoothed_percent
        for cls_label, tab_label in CLASSIFIER_LABEL_MAP.items():
            pct = smoothed.get(cls_label, 0)
            if tab_label in self._classifier_value_labels:
                self._classifier_value_labels[tab_label].setText(
                    f"{int(pct)}%"
                )

    def get_current_full_frame(self) -> Optional[np.ndarray]:
        """Return the last-received full-resolution camera frame.

        GrowthApp reads this on Save to snapshot the frame into
        live_label_NNN_*.bmp. Downsampling to PROCESS_WH for the
        Equalizer's visualization is intentional; the on-disk snapshot
        preserves the full 656×492 (or whatever the camera delivers)
        so downstream training pipelines get the native-resolution
        data.
        """
        return self._current_full_frame

    def set_save_enabled(self, enabled: bool):
        """Toggle the Save button — GrowthApp calls this on session state
        changes (running vs idle/armed). Auto-fit / Normalize / Reset
        stay always-enabled so the grower can experiment before arming.
        """
        self._save_btn.setEnabled(enabled)

    def reset_for_new_session(self):
        """Wipe transient state so the next session starts fresh.

        Called by GrowthApp on session reset (disarm). Preserves the
        loaded basis + slider defaults; only the live-camera cache,
        classifier % display, and pause state are cleared.
        """
        self._current_target = None
        self._current_full_frame = None
        self._selected_label.clear()
        self._selected_label.setText("Waiting for live camera stream…")
        for lbl in self._classifier_value_labels.values():
            lbl.setText("—%")
        # Clear pause state so the next armed session starts with a
        # streaming Selected pane, not stuck on the previous session's
        # last frame. setChecked triggers _on_pause_toggled which flips
        # _paused + updates the label, keeping all three in sync.
        if self._pause_btn.isChecked():
            self._pause_btn.setChecked(False)
        # Defensive: even if the button was already unchecked (no
        # toggled emission), force _paused False + label back.
        self._paused = False
        self._pause_btn.setText("Freeze frame")
        self._reset_to_uniform()

    def _on_pause_toggled(self, checked: bool):
        """React to the Freeze frame / Resume live toggle.

        ``checked=True`` → frozen; button label switches to "Resume live"
        so the next click's action is legible. ``checked=False`` → live;
        label back to "Freeze frame".
        """
        self._paused = checked
        self._pause_btn.setText("Resume live" if checked else "Freeze frame")

    # ----- Slider mechanics -----------------------------------------------

    def _on_slider_changed(self):
        # Guarded so programmatic setValue in _set_weights / _reset_to_uniform
        # only recomputes the reconstruction ONCE at the end, not 5 times.
        if self._adjusting:
            return
        self._refresh_slider_labels()
        self._update_reconstruction()

    def _refresh_slider_labels(self):
        for label, slider in self._sliders.items():
            self._slider_value_labels[label].setText(f"{slider.value()}%")

    def _current_weights(self) -> dict[str, float]:
        """Slider values as fractions in [0, 1]. Sum is arbitrary — the
        grower may or may not have Normalize'd."""
        return {
            label: slider.value() / 100.0
            for label, slider in self._sliders.items()
        }

    def _set_weights(self, weights: dict[str, float]):
        """Set all 5 sliders from a weight dict, refresh labels + recon
        exactly once via the _adjusting guard."""
        self._adjusting = True
        try:
            for label, slider in self._sliders.items():
                w = weights.get(label, 0.0)
                slider.setValue(int(round(w * 100)))
        finally:
            self._adjusting = False
        self._refresh_slider_labels()
        self._update_reconstruction()

    def _reset_to_uniform(self):
        uniform = 1.0 / len(CLASS_LABELS)
        self._set_weights({label: uniform for label in CLASS_LABELS})

    def _update_reconstruction(self):
        """Rebuild the Constructed pane from the current slider mixture.

        No-op if the basis failed to load (Constructed pane keeps its
        error placeholder). Blends only the classes present in the
        basis; grower sliders for missing classes contribute 0.
        """
        if not self._basis:
            return
        try:
            from scripts.equalizer_ui import DISPLAY_WH, array_to_pixmap
        except Exception:
            return

        weights = self._current_weights()
        shape = next(iter(self._basis.values())).shape
        recon = np.zeros(shape, dtype=np.float32)
        for label, w in weights.items():
            basis_img = self._basis.get(label)
            if basis_img is not None:
                recon += float(w) * basis_img
        pixmap = array_to_pixmap(recon, DISPLAY_WH)
        self._constructed_label.setPixmap(pixmap)

    # ----- Button handlers ------------------------------------------------

    def _on_auto_fit(self):
        """Least-squares fit the sliders onto the current 'Selected' image."""
        if self._current_target is None or not self._basis:
            return
        try:
            from scripts.equalizer_ui import auto_fit
        except Exception:
            return
        weights = auto_fit(self._basis, self._current_target)
        self._set_weights(weights)

    def _on_normalize(self):
        """Rescale sliders so they sum to 1.0 (100%)."""
        weights = self._current_weights()
        s = sum(weights.values())
        if s <= 0:
            return
        self._set_weights({k: v / s for k, v in weights.items()})

    def _on_save(self):
        """Emit the save signal with the current slider weights.

        GrowthApp handles the sensor snapshot + logger call — the tab
        stays independent of the growth-log file lifecycle.
        """
        weights = self._current_weights()
        self.live_label_save_requested.emit(weights)
