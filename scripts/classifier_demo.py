#!/usr/bin/env python3
"""Classifier demo — show Classifier2 reconstruction breakdown on a single image.

Standalone PyQt6 tool for demoing the live classifier to growers and at group
meetings. Opens an image, runs Classifier2 via ClassifierBridge, displays the
predicted-class probability breakdown as bars.

Requires torch + the AI_for_quantum repo set up — runs on the Mac dev
environment, NOT on Bulbasaur (no torch installed).

Usage (Mac):
    cd /Users/aj/AIQM-Software-Hardware-Integration
    ./.venv/bin/python scripts/classifier_demo.py
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from PIL import Image
from PyQt6.QtCore import Qt, QSize
from PyQt6.QtGui import QPixmap
from PyQt6.QtWidgets import (
    QApplication, QFileDialog, QGroupBox, QHBoxLayout, QLabel,
    QMainWindow, QMessageBox, QProgressBar, QPushButton, QStatusBar,
    QVBoxLayout, QWidget,
)

sys.path.insert(0, str(Path(__file__).parent.parent))
from gui.classifier_bridge import ClassifierBridge, GUI_LABELS

# Mac dev path to the AI_for_quantum repo.
AI_REPO_ROOT = Path("/Users/aj/test-claude/projects/ai-for-quantum")
DISPLAY_WH = (640, 480)
IMAGE_EXTS = "*.png *.bmp *.jpg *.jpeg *.tif *.tiff"


class ClassifierDemoWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Classifier Demo — RHEED Reconstruction Breakdown")
        self.resize(1200, 720)

        # Show loading state before the heavy classifier import.
        loading = QLabel("Loading Classifier2 model…  (5-10 s)")
        loading.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setCentralWidget(loading)
        QApplication.processEvents()

        try:
            self.classifier = ClassifierBridge(AI_REPO_ROOT)
        except Exception as e:
            QMessageBox.critical(
                self, "Classifier load error",
                f"Failed to load classifier from\n{AI_REPO_ROOT}\n\n{e}",
            )
            sys.exit(1)

        self._build_ui()

    def _build_ui(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        top = QHBoxLayout()
        open_btn = QPushButton("Open RHEED image…")
        open_btn.clicked.connect(self.action_open)
        top.addWidget(open_btn)
        top.addStretch()
        root.addLayout(top)

        middle = QHBoxLayout()

        image_group = QGroupBox("Input image")
        image_layout = QVBoxLayout(image_group)
        self.image_label = QLabel("No image loaded")
        self.image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.image_label.setMinimumSize(QSize(*DISPLAY_WH))
        self.image_label.setStyleSheet("background-color: #222; color: #888;")
        image_layout.addWidget(self.image_label)
        middle.addWidget(image_group)

        bars_group = QGroupBox("Classifier breakdown")
        bars_layout = QVBoxLayout(bars_group)
        self.bars: dict[str, QProgressBar] = {}
        self.value_labels: dict[str, QLabel] = {}
        for label in GUI_LABELS:
            row = QHBoxLayout()
            name = QLabel(label)
            name.setMinimumWidth(80)
            bar = QProgressBar()
            bar.setRange(0, 100)
            bar.setValue(0)
            bar.setTextVisible(False)
            value = QLabel("--")
            value.setMinimumWidth(60)
            value.setAlignment(
                Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
            )
            row.addWidget(name)
            row.addWidget(bar, stretch=1)
            row.addWidget(value)
            bars_layout.addLayout(row)
            self.bars[label] = bar
            self.value_labels[label] = value

        bars_layout.addStretch()
        self.predicted_label = QLabel("<i>No prediction yet</i>")
        self.predicted_label.setWordWrap(True)
        bars_layout.addWidget(self.predicted_label)
        self.quality_label = QLabel("")
        bars_layout.addWidget(self.quality_label)
        middle.addWidget(bars_group, stretch=1)

        root.addLayout(middle)

        self.statusbar = QStatusBar()
        self.setStatusBar(self.statusbar)

    def action_open(self) -> None:
        path_str, _ = QFileDialog.getOpenFileName(
            self, "Open RHEED image",
            str(Path.home() / "Downloads"),
            f"Images ({IMAGE_EXTS})",
        )
        if not path_str:
            return
        path = Path(path_str)
        try:
            pixmap = QPixmap(str(path)).scaled(
                DISPLAY_WH[0], DISPLAY_WH[1],
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation,
            )
            self.image_label.setPixmap(pixmap)
            self.statusbar.showMessage("Classifying…")
            QApplication.processEvents()

            arr = np.asarray(Image.open(path).convert("RGB"))
            result = self.classifier.classify(arr)
            self._display_result(result)
            self.statusbar.showMessage(f"Classified {path.name}", 4000)
        except Exception as e:
            QMessageBox.critical(self, "Classification error", str(e))
            self.statusbar.clearMessage()

    def _display_result(self, result: dict) -> None:
        scores = result.get("classification_scores", {})
        for label in GUI_LABELS:
            v = scores.get(label, 0.0)
            self.bars[label].setValue(int(round(v * 100)))
            self.value_labels[label].setText(f"{v * 100:.1f}%")

        predicted = result.get("predicted_class", "?")
        is_bad = result.get("is_bad", False)
        bad_conf = result.get("bad_confidence", 0.0)

        if is_bad:
            self.predicted_label.setText(
                f"<b>Predicted:</b> {predicted}  "
                f"<span style='color: #c33;'>(flagged low-quality, "
                f"{bad_conf:.1%} confidence)</span>"
            )
        else:
            self.predicted_label.setText(f"<b>Predicted:</b> {predicted}")

        quality = result.get("quality")
        if quality is not None:
            self.quality_label.setText(f"<b>Quality:</b> {quality:.2f}")
        else:
            self.quality_label.setText("")


def main() -> None:
    app = QApplication(sys.argv)
    window = ClassifierDemoWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
