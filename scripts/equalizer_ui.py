#!/usr/bin/env python3
"""RHEED Equalizer — interactive labeling tool (PyQt6 standalone).

Per the May 8 group meeting and the May 14 design conversation: sliders
correspond to **reconstruction types** (1x1, Tw(2x1), c(6x2), RT13, HTR),
not abstract SVD components. The basis is the per-class **mean image**
from Classifier2's STO_ideal_* training set. The grower drags sliders
until the live "your blend" reconstruction matches the target image; the
slider values *are* the label.

This sidesteps the "decompose this image into 50% 1x1, 25% rt13, 25% HTR
in your head" cognitive problem flagged at the May 8 meeting. The grower
adjusts and sees the result, instead of estimating mixtures by inspection.

Math:
    basis     = stack(mean_image[class] for class in 5 classes)
    weights   = (w_1x1, w_Tw, w_c6x2, w_RT13, w_HTR)
    reconstr. = sum(w_i * basis_i)

Auto-fit:
    Least-squares onto the basis, clipped to non-negative, normalized to
    sum=1. Gives the grower a sensible starting point to refine from.

Saved labels go to:
    ~/Downloads/equalizer_labels.csv
    columns: timestamp, image_path, 1x1, Tw(2x1), c(6x2), RT13, HTR

Run (Mac, AIQM .venv):
    cd /Users/aj/AIQM-Software-Hardware-Integration
    ./.venv/bin/python scripts/equalizer_ui.py
"""
from __future__ import annotations

import csv
import datetime
import sys
from pathlib import Path

import numpy as np
from PIL import Image
from PyQt6.QtCore import Qt, QSize
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtWidgets import (
    QApplication, QFileDialog, QGridLayout, QGroupBox, QHBoxLayout,
    QLabel, QMainWindow, QMessageBox, QPushButton, QSlider, QStatusBar,
    QVBoxLayout, QWidget,
)


# Training set provides the basis. Mirrors the prototype's paths.
CLASSIFIER2_DATA_ROOT = Path(
    "/Users/aj/test-claude/projects/ai-for-quantum/src/data"
)
CLASS_DIRS: dict[str, str] = {
    "1x1":     "STO_ideal_1x1",
    "Tw(2x1)": "STO_ideal_Twinned2x1",
    "c(6x2)":  "STO_ideal_c6x2",
    "RT13":    "STO_ideal_RT13",
    "HTR":     "STO_ideal_HTR",
}
CLASS_LABELS = list(CLASS_DIRS.keys())
IMAGE_EXTS = {".png", ".bmp", ".jpg", ".jpeg", ".tif", ".tiff"}

# Internal processing resolution — matches the SVD prototype so basis
# vectors are pixel-comparable. Display upscales for visibility.
PROCESS_WH = (128, 96)
DISPLAY_WH = (640, 480)
LABELS_CSV = Path.home() / "Downloads" / "equalizer_labels.csv"

# Sliders are integer 0-100 internally; UI weight is value/100.
SLIDER_MIN = 0
SLIDER_MAX = 100


# ----- Math helpers ------------------------------------------------------


def load_grayscale(path: Path, target_wh: tuple[int, int]) -> np.ndarray:
    """Load image, convert to grayscale, resize to target_wh (W, H)."""
    img = Image.open(path).convert("L").resize(target_wh, Image.LANCZOS)
    return np.asarray(img, dtype=np.float32)


def load_class_means(target_wh: tuple[int, int]) -> dict[str, np.ndarray]:
    """Compute the per-class mean image for each STO reconstruction class."""
    means: dict[str, np.ndarray] = {}
    for label, subdir in CLASS_DIRS.items():
        class_dir = CLASSIFIER2_DATA_ROOT / subdir
        if not class_dir.is_dir():
            continue
        rows: list[np.ndarray] = []
        for p in sorted(class_dir.iterdir()):
            if p.suffix.lower() not in IMAGE_EXTS:
                continue
            rows.append(load_grayscale(p, target_wh))
        if rows:
            means[label] = np.stack(rows).mean(axis=0)
    return means


def reconstruct(
    means: dict[str, np.ndarray],
    weights: dict[str, float],
) -> np.ndarray:
    """Weighted sum of class means. Returns float32 clipped to [0, 255]."""
    out = np.zeros_like(next(iter(means.values())))
    for label, w in weights.items():
        if label in means:
            out += w * means[label]
    return np.clip(out, 0, 255)


def auto_fit(
    means: dict[str, np.ndarray], target: np.ndarray,
) -> dict[str, float]:
    """Least-squares fit onto the class-mean basis.

    Clips negative weights to 0 (negative mixture has no physical meaning)
    and normalizes so the weights sum to 1. Falls back to uniform if the
    fit collapses to all-zeros (e.g., black target image).
    """
    basis = np.stack(
        [means[label].flatten() for label in CLASS_LABELS if label in means],
        axis=1,
    )
    t = target.flatten()
    w, *_ = np.linalg.lstsq(basis, t, rcond=None)
    w = np.clip(w, 0, None)
    s = w.sum()
    if s <= 0:
        w = np.full(len(CLASS_LABELS), 1.0 / len(CLASS_LABELS))
    else:
        w = w / s
    labels_in_means = [label for label in CLASS_LABELS if label in means]
    return dict(zip(labels_in_means, w))


def array_to_pixmap(
    arr: np.ndarray, display_wh: tuple[int, int],
) -> QPixmap:
    """Convert (H, W) float32 in [0, 255] to upscaled Grayscale8 QPixmap."""
    arr_u8 = np.clip(arr, 0, 255).astype(np.uint8)
    # PyQt6 requires contiguous bytes — np arrays usually are, but be safe.
    arr_u8 = np.ascontiguousarray(arr_u8)
    h, w = arr_u8.shape
    qimg = QImage(
        arr_u8.tobytes(), w, h, w, QImage.Format.Format_Grayscale8,
    )
    pixmap = QPixmap.fromImage(qimg)
    return pixmap.scaled(
        display_wh[0], display_wh[1],
        Qt.AspectRatioMode.IgnoreAspectRatio,
        Qt.TransformationMode.SmoothTransformation,
    )


# ----- UI ----------------------------------------------------------------


class EqualizerWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("RHEED Equalizer")
        self.resize(1500, 800)

        self.means = load_class_means(PROCESS_WH)
        if not self.means:
            QMessageBox.critical(
                self, "Training data missing",
                f"Couldn't load per-class means from\n{CLASSIFIER2_DATA_ROOT}\n"
                "Make sure the Classifier2 STO_ideal_* dirs exist.",
            )
            sys.exit(1)
        missing = set(CLASS_LABELS) - set(self.means)
        if missing:
            QMessageBox.warning(
                self, "Some classes missing",
                f"Missing class means: {missing}.\n"
                "Sliders for those classes will appear but do nothing.",
            )

        self.target: np.ndarray | None = None
        self.target_path: Path | None = None

        self._build_ui()
        self._reset_to_uniform()

    # ----- Layout -----

    def _build_ui(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        # Top bar — buttons
        top = QHBoxLayout()
        for text, slot in [
            ("Open image…",    self.action_open),
            ("Auto-fit",       self.action_autofit),
            ("Normalize",      self.action_normalize),
            ("Reset (uniform)", self.action_reset),
            ("Save label",     self.action_save),
        ]:
            btn = QPushButton(text)
            btn.clicked.connect(slot)
            top.addWidget(btn)
        top.addStretch()
        root.addLayout(top)

        # Middle — target image, reconstruction, sliders
        middle = QHBoxLayout()

        target_group = QGroupBox("Target")
        target_layout = QVBoxLayout(target_group)
        self.target_label = QLabel("No image loaded")
        self.target_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.target_label.setMinimumSize(QSize(*DISPLAY_WH))
        self.target_label.setStyleSheet("background-color: #222; color: #888;")
        target_layout.addWidget(self.target_label)
        middle.addWidget(target_group)

        recon_group = QGroupBox("Your blend")
        recon_layout = QVBoxLayout(recon_group)
        self.recon_label = QLabel()
        self.recon_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.recon_label.setMinimumSize(QSize(*DISPLAY_WH))
        recon_layout.addWidget(self.recon_label)
        middle.addWidget(recon_group)

        sliders_group = QGroupBox("Mixture")
        sliders_layout = QGridLayout(sliders_group)
        sliders_layout.setColumnStretch(1, 1)
        self.sliders: dict[str, QSlider] = {}
        self.value_labels: dict[str, QLabel] = {}
        for i, label in enumerate(CLASS_LABELS):
            name_label = QLabel(label)
            name_label.setMinimumWidth(80)
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(SLIDER_MIN, SLIDER_MAX)
            slider.valueChanged.connect(self._on_slider_changed)
            value_label = QLabel("0.00")
            value_label.setMinimumWidth(50)
            value_label.setAlignment(
                Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
            )
            sliders_layout.addWidget(name_label, i, 0)
            sliders_layout.addWidget(slider, i, 1)
            sliders_layout.addWidget(value_label, i, 2)
            self.sliders[label] = slider
            self.value_labels[label] = value_label
        sliders_group.setMaximumWidth(400)
        middle.addWidget(sliders_group)

        root.addLayout(middle)

        # Status bar
        self.statusbar = QStatusBar()
        self.setStatusBar(self.statusbar)
        self.status_sum = QLabel()
        self.status_err = QLabel()
        self.statusbar.addPermanentWidget(self.status_sum)
        self.statusbar.addPermanentWidget(self.status_err)

    # ----- Button actions -----

    def action_open(self) -> None:
        exts = " ".join(f"*{e}" for e in IMAGE_EXTS)
        path_str, _ = QFileDialog.getOpenFileName(
            self, "Open RHEED image",
            str(Path.home() / "Downloads"),
            f"Images ({exts})",
        )
        if not path_str:
            return
        try:
            self.target = load_grayscale(Path(path_str), PROCESS_WH)
            self.target_path = Path(path_str)
            self.target_label.setPixmap(
                array_to_pixmap(self.target, DISPLAY_WH),
            )
            self.statusbar.showMessage(
                f"Loaded {self.target_path.name}", 4000,
            )
            self._update_reconstruction()
        except Exception as e:
            QMessageBox.critical(self, "Load error", str(e))

    def action_autofit(self) -> None:
        if self.target is None:
            self.statusbar.showMessage("Load a target image first", 3000)
            return
        weights = auto_fit(self.means, self.target)
        self._set_weights(weights)

    def action_normalize(self) -> None:
        weights = self._current_weights()
        s = sum(weights.values())
        if s <= 0:
            return
        normalized = {k: v / s for k, v in weights.items()}
        self._set_weights(normalized)

    def action_reset(self) -> None:
        self._reset_to_uniform()

    def action_save(self) -> None:
        if self.target is None or self.target_path is None:
            self.statusbar.showMessage("Load a target image first", 3000)
            return
        weights = self._current_weights()
        LABELS_CSV.parent.mkdir(parents=True, exist_ok=True)
        write_header = not LABELS_CSV.exists()
        with open(LABELS_CSV, "a", newline="") as f:
            writer = csv.writer(f)
            if write_header:
                writer.writerow(["timestamp", "image_path"] + CLASS_LABELS)
            writer.writerow(
                [
                    datetime.datetime.now().isoformat(),
                    str(self.target_path),
                ]
                + [f"{weights.get(label, 0.0):.4f}" for label in CLASS_LABELS]
            )
        self.statusbar.showMessage(f"Saved label → {LABELS_CSV}", 5000)

    # ----- Slider state plumbing -----

    def _reset_to_uniform(self) -> None:
        w = 1.0 / len(CLASS_LABELS)
        self._set_weights({label: w for label in CLASS_LABELS})

    def _set_weights(self, weights: dict[str, float]) -> None:
        """Set sliders without re-firing valueChanged per slider; one update at end."""
        for label, w in weights.items():
            if label in self.sliders:
                slider = self.sliders[label]
                slider.blockSignals(True)
                slider.setValue(int(round(w * SLIDER_MAX)))
                slider.blockSignals(False)
        self._on_slider_changed()

    def _current_weights(self) -> dict[str, float]:
        return {
            label: slider.value() / SLIDER_MAX
            for label, slider in self.sliders.items()
        }

    def _on_slider_changed(self, *_: object) -> None:
        weights = self._current_weights()
        for label, w in weights.items():
            self.value_labels[label].setText(f"{w:.2f}")
        self._update_reconstruction()

    def _update_reconstruction(self) -> None:
        weights = self._current_weights()
        recon = reconstruct(self.means, weights)
        self.recon_label.setPixmap(array_to_pixmap(recon, DISPLAY_WH))
        total = sum(weights.values())
        normalized = "normalized" if abs(total - 1.0) < 0.01 else "un-normalized"
        self.status_sum.setText(f"Sum: {total:.2f} ({normalized})  ")
        if self.target is not None:
            err = float(np.mean(np.abs(recon - self.target)))
            self.status_err.setText(f"Err: {err:.2f}")
        else:
            self.status_err.setText("")


def main() -> None:
    app = QApplication(sys.argv)
    window = EqualizerWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
