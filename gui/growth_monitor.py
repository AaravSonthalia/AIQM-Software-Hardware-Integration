"""
MBE Growth Monitor — single-screen dark-themed dashboard widget.

Displays live RHEED image, pyrometer temperature, PSU voltage/current,
elapsed timer, AI + human classification, and a COMMIT button for
user-annotated snapshots.
"""

from datetime import datetime
from typing import Optional

import numpy as np
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel,
    QPushButton, QLineEdit, QTextEdit, QSizePolicy, QFrame,
    QSlider, QCheckBox, QDoubleSpinBox, QGroupBox, QComboBox,
    QFormLayout, QFileDialog,
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QImage, QPixmap, QPainter, QFont, QColor, QShortcut, QKeySequence

from gui.state import PowerSupplyState, CameraState, PyrometerState
from gui.widgets import ValueDisplay


# ---------------------------------------------------------------------------
# Stylesheet
# ---------------------------------------------------------------------------

DARK_STYLESHEET = """
QWidget {
    background-color: #111;
    color: #ddd;
    font-size: 13px;
}
QLineEdit, QTextEdit {
    background-color: #fff;
    color: #111;
    border: 1px solid #555;
    padding: 4px;
}
QLabel {
    background-color: transparent;
}
QPushButton {
    border: 1px solid #555;
    padding: 8px 16px;
    font-weight: bold;
    background-color: #333;
    color: #ddd;
}
QPushButton:disabled {
    background-color: #222;
    color: #666;
    border-color: #333;
}
"""

BTN_ARM = "QPushButton { background-color: #2563eb; color: white; }"
BTN_DISARM = "QPushButton { background-color: #7c3aed; color: white; }"
BTN_START = "QPushButton { background-color: #16a34a; color: white; }"
BTN_STOP = "QPushButton { background-color: #dc2626; color: white; }"
BTN_TEAL = "QPushButton { background-color: #0d9488; color: white; }"

# Reconstruction labels shared by sliders, confidence bars, and AI output.
# Aligned to Classifier2's RECONSTRUCTION_TYPES output order.
RECON_LABELS = ["(1x1)", "Tw(2x1)", "c(6x2)", "rt13", "HTR"]


# ---------------------------------------------------------------------------
# ConfidenceBarWidget
# ---------------------------------------------------------------------------

class ConfidenceBarWidget(QWidget):
    """Horizontal bar showing colored segments with percentage labels."""

    def __init__(self, n_segments: int = 5, parent=None):
        super().__init__(parent)
        self.setFixedHeight(30)
        self._n = n_segments
        self._values = [0.0] * n_segments
        self._labels = [""] * n_segments

    def set_confidences(self, values: list[float]):
        self._values = (values + [0.0] * self._n)[: self._n]
        self.update()

    def set_labels(self, names: list[str]):
        self._labels = (names + [""] * self._n)[: self._n]
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        total = sum(self._values) or 1.0

        x = 0.0
        font = QFont("Monospace", 9)
        painter.setFont(font)

        for i, val in enumerate(self._values):
            frac = val / total
            seg_w = frac * w
            if seg_w < 1:
                continue
            # Draw segment
            shade = max(40, min(255, int(80 + 175 * (val / 100.0))))
            painter.setBrush(QColor(shade, 100, 100))
            painter.setPen(Qt.PenStyle.NoPen)
            painter.drawRect(int(x), 0, int(seg_w), h)

            # Draw percentage text
            pct_text = f"{val:.0f}%"
            painter.setPen(QColor("white"))
            painter.drawText(
                int(x), 0, int(seg_w), h,
                Qt.AlignmentFlag.AlignCenter, pct_text,
            )
            x += seg_w
        painter.end()


# ---------------------------------------------------------------------------
# GrowthMonitor
# ---------------------------------------------------------------------------

class GrowthMonitor(QWidget):
    """Main growth monitoring widget — single-screen dark-themed dashboard."""

    arm_requested = pyqtSignal()
    disarm_requested = pyqtSignal()
    start_requested = pyqtSignal()
    stop_requested = pyqtSignal()
    commit_requested = pyqtSignal(dict)
    psu_command_requested = pyqtSignal(str, tuple)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet(DARK_STYLESHEET)

        self._state = "idle"  # idle | armed | running
        self._start_time: Optional[datetime] = None
        self._latest_psu: Optional[PowerSupplyState] = None
        self._latest_pyro: Optional[PyrometerState] = None
        self._latest_camera: Optional[CameraState] = None
        self._current_frame: Optional[np.ndarray] = None

        self._build_ui()
        self._apply_state()

        # Elapsed timer — 10 ms tick
        self._elapsed_timer = QTimer(self)
        self._elapsed_timer.setInterval(10)
        self._elapsed_timer.timeout.connect(self._tick_elapsed)

    # ----- Layout ----------------------------------------------------------

    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(12, 8, 12, 8)
        root.setSpacing(8)

        # Row 1: Sample ID + buttons
        row1 = QHBoxLayout()
        row1.addWidget(QLabel("Sample ID:"))
        self.sample_id_input = QLineEdit()
        self.sample_id_input.setPlaceholderText("Enter sample ID...")
        self.sample_id_input.setFixedWidth(250)
        row1.addWidget(self.sample_id_input)
        row1.addStretch()

        self.arm_btn = QPushButton("ARM")
        self.start_btn = QPushButton("START")
        self.stop_btn = QPushButton("STOP")
        for btn in (self.arm_btn, self.start_btn, self.stop_btn):
            btn.setFixedWidth(100)
        row1.addWidget(self.arm_btn)
        row1.addWidget(self.start_btn)
        row1.addWidget(self.stop_btn)
        root.addLayout(row1)

        # Separator
        sep1 = QFrame()
        sep1.setFrameShape(QFrame.Shape.HLine)
        sep1.setStyleSheet("color: #555;")
        root.addWidget(sep1)

        # Row 2: Value displays
        row2 = QHBoxLayout()
        self.elapsed_display = ValueDisplay("Elapsed Time", "", 0)
        self.elapsed_display.value.setText("00:00:00.00")
        self.temp_display = ValueDisplay("Temperature", "C", 0)
        self.voltage_display = ValueDisplay("Voltage", "V", 2)
        self.current_display = ValueDisplay("Current", "A", 3)
        for d in (self.elapsed_display, self.temp_display, self.voltage_display, self.current_display):
            d.setStyleSheet("QFrame { border: 1px solid #555; } QLabel { background: transparent; }")
            row2.addWidget(d)
        root.addLayout(row2)

        # Separator
        sep2 = QFrame()
        sep2.setFrameShape(QFrame.Shape.HLine)
        sep2.setStyleSheet("color: #555;")
        root.addWidget(sep2)

        # Row 3: Main content — left (RHEED + classification) | right (instructions + commit)
        row3 = QHBoxLayout()
        row3.setSpacing(12)

        # --- Left panel: AI class, RHEED image, Human class ---
        left = QVBoxLayout()
        left.setSpacing(6)

        # AI classification (disabled for v1 — visible but grayed out)
        ai_row = QHBoxLayout()
        ai_lbl = QLabel("AI:")
        ai_lbl.setStyleSheet("color: #555;")
        ai_row.addWidget(ai_lbl)
        self.ai_class_label = QLabel("\u2014")
        self.ai_class_label.setStyleSheet("color: #555; font-style: italic;")
        self.ai_class_label.setEnabled(False)
        ai_row.addWidget(self.ai_class_label, 1)

        self.ai_bad_indicator = QLabel("BAD")
        self.ai_bad_indicator.setVisible(False)
        ai_row.addWidget(self.ai_bad_indicator)

        self.ai_quality_label = QLabel("")
        self.ai_quality_label.setStyleSheet("color: #555; font-size: 11px;")
        self.ai_quality_label.setEnabled(False)
        ai_row.addWidget(self.ai_quality_label)

        left.addLayout(ai_row)

        self.ai_confidence = ConfidenceBarWidget(n_segments=len(RECON_LABELS))
        self.ai_confidence.set_labels(RECON_LABELS)
        self.ai_confidence.set_confidences([0] * len(RECON_LABELS))
        self.ai_confidence.setEnabled(False)
        self.ai_confidence.setStyleSheet("background-color: #1a1a1a;")
        left.addWidget(self.ai_confidence)

        # RHEED image
        self.rheed_image_label = QLabel()
        self.rheed_image_label.setMinimumSize(400, 300)
        self.rheed_image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.rheed_image_label.setStyleSheet("background-color: #000; border: 1px solid #555;")
        self.rheed_image_label.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding,
        )
        left.addWidget(self.rheed_image_label, 1)

        # Human classification — checkboxes (multiple can be checked for transitional states)
        human_group = QGroupBox("Human Classification")
        human_group.setStyleSheet(
            "QGroupBox { border: 1px solid #555; padding-top: 14px; margin-top: 4px; }"
            "QGroupBox::title { color: #ddd; subcontrol-position: top left; padding: 2px 6px; }"
        )
        human_hbox = QHBoxLayout(human_group)
        human_hbox.setSpacing(12)
        human_hbox.setContentsMargins(6, 4, 6, 4)

        self._human_checkboxes: dict[str, QCheckBox] = {}
        for label_name in RECON_LABELS:
            cb = QCheckBox(label_name)
            cb.setStyleSheet("font-family: monospace; font-size: 12px;")
            human_hbox.addWidget(cb)
            self._human_checkboxes[label_name] = cb

        left.addWidget(human_group)

        # Auto-capture controls (disabled for v1)
        ac_row = QHBoxLayout()
        ac_row.setSpacing(8)

        self.auto_capture_checkbox = QCheckBox("Auto-Capture (coming soon)")
        self.auto_capture_checkbox.setEnabled(False)
        self.auto_capture_checkbox.setStyleSheet("color: #555;")
        ac_row.addWidget(self.auto_capture_checkbox)

        thresh_lbl = QLabel("Threshold:")
        thresh_lbl.setStyleSheet("color: #555;")
        ac_row.addWidget(thresh_lbl)
        self.auto_capture_threshold = QDoubleSpinBox()
        self.auto_capture_threshold.setRange(0.01, 1.00)
        self.auto_capture_threshold.setSingleStep(0.05)
        self.auto_capture_threshold.setValue(0.20)
        self.auto_capture_threshold.setFixedWidth(70)
        self.auto_capture_threshold.setEnabled(False)
        ac_row.addWidget(self.auto_capture_threshold)

        self.auto_capture_delta_label = QLabel("delta: ---")
        self.auto_capture_delta_label.setStyleSheet("color: #555; font-family: monospace; font-size: 11px;")
        ac_row.addWidget(self.auto_capture_delta_label)
        ac_row.addStretch()

        left.addLayout(ac_row)

        row3.addLayout(left, 2)

        # --- Right panel: Instructions + buttons ---
        right = QVBoxLayout()
        right.setSpacing(8)

        ai_instr_lbl = QLabel("AI Instructions")
        ai_instr_lbl.setStyleSheet("color: #555;")
        right.addWidget(ai_instr_lbl)
        self.ai_instructions_box = QTextEdit()
        self.ai_instructions_box.setReadOnly(True)
        self.ai_instructions_box.setEnabled(False)
        self.ai_instructions_box.setPlaceholderText("AI instructions (coming soon)")
        self.ai_instructions_box.setMaximumHeight(120)
        self.ai_instructions_box.setStyleSheet("background-color: #1a1a1a; color: #555; border: 1px solid #333;")
        right.addWidget(self.ai_instructions_box)

        right.addWidget(QLabel("Human Instructions"))
        self.human_instructions_box = QTextEdit()
        self.human_instructions_box.setPlaceholderText("Enter growth notes...")
        self.human_instructions_box.setMaximumHeight(120)
        right.addWidget(self.human_instructions_box)

        right.addStretch()

        # Bottom buttons
        btn_row = QHBoxLayout()
        self.commit_btn = QPushButton("SAVE OBSERVATION  (Ctrl+S)")
        self.commit_btn.setStyleSheet(
            "QPushButton { background-color: #0d9488; color: white; font-size: 16px; font-weight: bold; }"
            "QPushButton:disabled { background-color: #222; color: #666; }"
        )
        self.commit_btn.setFixedHeight(54)
        btn_row.addWidget(self.commit_btn)
        right.addLayout(btn_row)

        # Keyboard shortcut for save
        self._save_shortcut = QShortcut(QKeySequence("Ctrl+S"), self)
        self._save_shortcut.activated.connect(self._on_commit)

        row3.addLayout(right, 1)
        root.addLayout(row3, 1)

        # --- Config panel (collapsible) ---
        self._config_toggle = QPushButton("▶ Config")
        self._config_toggle.setStyleSheet(
            "QPushButton { text-align: left; border: none; color: #aaa; font-size: 12px; padding: 2px; }"
        )
        self._config_toggle.setCheckable(True)
        self._config_toggle.clicked.connect(self._toggle_config)
        root.addWidget(self._config_toggle)

        self._config_panel = QWidget()
        self._config_panel.setVisible(False)
        config_form = QFormLayout(self._config_panel)
        config_form.setContentsMargins(12, 4, 12, 4)
        config_form.setSpacing(6)

        self.config_interval_spin = QDoubleSpinBox()
        self.config_interval_spin.setRange(0.1, 60.0)
        self.config_interval_spin.setValue(1.0)
        self.config_interval_spin.setSuffix(" s")
        self.config_interval_spin.setDecimals(1)
        self.config_interval_spin.setSingleStep(0.5)
        config_form.addRow("Recording interval:", self.config_interval_spin)

        save_row = QHBoxLayout()
        self.config_save_path = QLineEdit()
        self.config_save_path.setPlaceholderText("logs/growths")
        self.config_save_path.setText("logs/growths")
        save_row.addWidget(self.config_save_path)
        browse_btn = QPushButton("Browse...")
        browse_btn.setFixedWidth(80)
        browse_btn.clicked.connect(self._on_config_browse)
        save_row.addWidget(browse_btn)
        config_form.addRow("Save folder:", save_row)

        self.config_prefix = QLineEdit("growth")
        config_form.addRow("Filename prefix:", self.config_prefix)

        self.config_camera_mode = QComboBox()
        self.config_camera_mode.addItems(["dummy", "screengrab", "direct"])
        config_form.addRow("Camera mode:", self.config_camera_mode)

        self.config_pyrometer_mode = QComboBox()
        self.config_pyrometer_mode.addItems(["dummy", "modbus", "screengrab"])
        config_form.addRow("Pyrometer mode:", self.config_pyrometer_mode)

        self.config_psu_resource = QLineEdit()
        self.config_psu_resource.setPlaceholderText("e.g. ASRL/dev/cu.usbserial-120::INSTR")
        config_form.addRow("PSU resource:", self.config_psu_resource)

        root.addWidget(self._config_panel)

        # --- Connect button signals ---
        self.arm_btn.clicked.connect(self._on_arm_clicked)
        self.start_btn.clicked.connect(self._on_start_clicked)
        self.stop_btn.clicked.connect(self._on_stop_clicked)
        self.commit_btn.clicked.connect(self._on_commit)

    # ----- State machine ---------------------------------------------------

    def _apply_state(self):
        """Update button enable/disable and text based on current state."""
        s = self._state
        if s == "idle":
            self.arm_btn.setText("ARM")
            self.arm_btn.setStyleSheet(BTN_ARM)
            self.arm_btn.setEnabled(True)
            self.start_btn.setEnabled(False)
            self.stop_btn.setEnabled(False)
            self.commit_btn.setEnabled(False)
            self.sample_id_input.setEnabled(True)
        elif s == "armed":
            self.arm_btn.setText("DISARM")
            self.arm_btn.setStyleSheet(BTN_DISARM)
            self.arm_btn.setEnabled(True)
            self.start_btn.setEnabled(True)
            self.start_btn.setStyleSheet(BTN_START)
            self.stop_btn.setEnabled(False)
            self.commit_btn.setEnabled(False)
            self.sample_id_input.setEnabled(True)
        elif s == "running":
            self.arm_btn.setEnabled(False)
            self.start_btn.setEnabled(False)
            self.stop_btn.setEnabled(True)
            self.stop_btn.setStyleSheet(BTN_STOP)
            self.commit_btn.setEnabled(True)
            self.sample_id_input.setEnabled(False)

    def set_state(self, new_state: str):
        """Transition to a new state (called by GrowthApp)."""
        self._state = new_state
        self._apply_state()

    def _on_arm_clicked(self):
        if self._state == "idle":
            self.arm_requested.emit()
        elif self._state in ("armed",):
            self.disarm_requested.emit()

    def _on_start_clicked(self):
        if self._state == "armed":
            self._start_time = datetime.now()
            self._elapsed_timer.start()
            self.start_requested.emit()

    def _on_stop_clicked(self):
        if self._state == "running":
            self._elapsed_timer.stop()
            self.stop_requested.emit()

    # ----- Elapsed timer ---------------------------------------------------

    def _tick_elapsed(self):
        if self._start_time is None:
            return
        delta = datetime.now() - self._start_time
        total_s = delta.total_seconds()
        h = int(total_s // 3600)
        m = int((total_s % 3600) // 60)
        s = total_s % 60
        self.elapsed_display.value.setText(f"{h:02d}:{m:02d}:{s:05.2f}")

    def get_elapsed_seconds(self) -> float:
        if self._start_time is None:
            return 0.0
        return (datetime.now() - self._start_time).total_seconds()

    # ----- State update methods (called by GrowthApp fan-out) -------------

    def update_psu_state(self, state: PowerSupplyState):
        self._latest_psu = state
        if state.connected:
            self.voltage_display.set_value(state.voltage_measured)
            self.current_display.set_value(state.current_measured)
        else:
            self.voltage_display.value.setText("---")
            self.current_display.value.setText("---")

    def update_pyrometer_state(self, state: PyrometerState):
        self._latest_pyro = state
        if state.connected:
            self.temp_display.value.setText(f"{state.temperature:.0f} C")
        else:
            self.temp_display.value.setText("---")

    def update_camera_state(self, state: CameraState):
        self._latest_camera = state
        if state.frame is not None:
            self._current_frame = state.frame
            self._display_frame(state.frame)

    # ----- RHEED frame display --------------------------------------------

    def _display_frame(self, frame: np.ndarray):
        h, w = frame.shape[:2]
        if frame.ndim == 2:
            qimg = QImage(frame.data, w, h, w, QImage.Format.Format_Grayscale8)
        else:
            qimg = QImage(frame.data, w, h, 3 * w, QImage.Format.Format_RGB888)
        pixmap = QPixmap.fromImage(qimg).scaled(
            self.rheed_image_label.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        self.rheed_image_label.setPixmap(pixmap)

    # ----- Human classification (checkboxes) ---------------------------------

    def get_human_classification(self) -> dict[str, bool]:
        """Return current checkbox states as a dict of booleans."""
        return {name: self._human_checkboxes[name].isChecked() for name in RECON_LABELS}

    # ----- Config panel helpers -----------------------------------------------

    def _toggle_config(self, checked: bool):
        self._config_panel.setVisible(checked)
        self._config_toggle.setText("▼ Config" if checked else "▶ Config")

    def _on_config_browse(self):
        folder = QFileDialog.getExistingDirectory(self, "Select Save Folder")
        if folder:
            self.config_save_path.setText(folder)

    # ----- AI classification display ---------------------------------------

    def update_ai_classification(self, result: dict):
        """Update AI classification displays from ClassifierBridge output.

        *result* keys: predicted_class, classification_scores (dict label→float),
        is_bad, bad_confidence, quality.
        """
        self.ai_class_label.setText(result.get("predicted_class", ""))
        self.ai_class_label.setStyleSheet("color: #4ade80; font-weight: bold;")

        # Confidence bar — map scores dict to ordered list
        scores = result.get("classification_scores", {})
        vals = [scores.get(lbl, 0.0) * 100 for lbl in RECON_LABELS]
        self.ai_confidence.set_confidences(vals)

        # Bad indicator
        is_bad = result.get("is_bad", False)
        self.ai_bad_indicator.setVisible(is_bad)

        # Quality score
        quality = result.get("quality")
        if quality is not None:
            self.ai_quality_label.setText(f"Q:{quality:.0%}")
        else:
            self.ai_quality_label.setText("")

    # ----- COMMIT handler -------------------------------------------------

    def _on_commit(self):
        if not self.commit_btn.isEnabled():
            return
        import json
        entry = {
            "timestamp": datetime.now().isoformat(),
            "sample_id": self.sample_id_input.text(),
            "elapsed_s": f"{self.get_elapsed_seconds():.2f}",
            "pyrometer_temp_C": (
                f"{self._latest_pyro.temperature:.1f}"
                if self._latest_pyro and self._latest_pyro.connected else ""
            ),
            "psu_voltage_V": (
                f"{self._latest_psu.voltage_measured:.3f}"
                if self._latest_psu and self._latest_psu.connected else ""
            ),
            "psu_current_A": (
                f"{self._latest_psu.current_measured:.3f}"
                if self._latest_psu and self._latest_psu.connected else ""
            ),
            "ai_classification": self.ai_class_label.text(),
            "human_classification": json.dumps(self.get_human_classification()),
            "ai_instructions": self.ai_instructions_box.toPlainText(),
            "human_instructions": self.human_instructions_box.toPlainText(),
        }
        self.commit_requested.emit(entry)

    # ----- Frame save (standalone, outside commit) ------------------------

    def get_current_frame(self) -> Optional[np.ndarray]:
        return self._current_frame

    # ----- Reset -----------------------------------------------------------

    def reset_displays(self):
        """Clear all displays back to defaults."""
        self.elapsed_display.value.setText("00:00:00.00")
        self.temp_display.value.setText("---")
        self.voltage_display.value.setText("---")
        self.current_display.value.setText("---")
        self.rheed_image_label.clear()
        self.ai_class_label.setText("\u2014")
        self.ai_class_label.setStyleSheet("color: #555; font-style: italic;")
        self.ai_bad_indicator.setVisible(False)
        self.ai_quality_label.setText("")
        self.ai_confidence.set_confidences([0] * len(RECON_LABELS))
        # Reset human checkboxes
        for name in RECON_LABELS:
            self._human_checkboxes[name].setChecked(False)
        self.auto_capture_delta_label.setText("delta: ---")
        self._start_time = None
        self._current_frame = None
        self._latest_psu = None
        self._latest_pyro = None
        self._latest_camera = None
