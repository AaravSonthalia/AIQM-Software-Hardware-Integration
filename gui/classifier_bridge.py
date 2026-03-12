"""
Bridge between the Classifier2 ML model and the Growth Monitor GUI.

Loads the trained BradleyTerryModel once at startup, pre-computes ideal/bad
reference scores, and exposes a lightweight ``classify(frame)`` method for
real-time inference on RHEED camera frames.

Requires:
    - AI_for_quantum repo on sys.path (or CLASSIFIER2_ROOT configured)
    - ``Classifier2/artifacts/best_model.pth`` (trained model)
    - ``Data/STO_ideal_*/`` directories (reference images)
"""
from __future__ import annotations

import sys
from pathlib import Path
from typing import Optional

import numpy as np

# Label mapping: GUI display labels → Classifier2 internal labels.
# Classifier2 uses: ['(1 x 1)', 'Twinned(2 x 1)', 'c(6 x 2)', '(√13 x √13)', 'HTR']
# GUI uses shorter versions for slider space.
GUI_LABELS = ["(1x1)", "Tw(2x1)", "c(6x2)", "rt13", "HTR"]
C2_LABELS = ["(1 x 1)", "Twinned(2 x 1)", "c(6 x 2)", "(√13 x √13)", "HTR"]
_C2_TO_GUI = dict(zip(C2_LABELS, GUI_LABELS))


class ClassifierBridge:
    """Loads Classifier2 and runs inference on raw RHEED frames.

    Parameters
    ----------
    ai_repo_root : str | Path
        Path to the AI_for_quantum repository root.
    model_path : str | Path | None
        Override path to ``best_model.pth``.  Defaults to
        ``<ai_repo_root>/Classifier2/artifacts/best_model.pth``.
    bad_threshold : float
        Confidence threshold for "bad image" detection (default 0.6).
    """

    def __init__(
        self,
        ai_repo_root: str | Path,
        model_path: Optional[str | Path] = None,
        bad_threshold: float = 0.6,
    ):
        self._repo = Path(ai_repo_root)
        self._bad_threshold = bad_threshold

        # Add Classifier2 to sys.path so its modules are importable.
        c2_dir = str(self._repo / "Classifier2")
        if c2_dir not in sys.path:
            sys.path.insert(0, c2_dir)

        import torch
        from evaluate import load_model, load_all_ideal_scores, load_bad_image_scores

        self._torch = torch

        # Resolve model path.
        if model_path is None:
            model_path = self._repo / "Classifier2" / "artifacts" / "best_model.pth"

        self._model, self._device = load_model(str(model_path))

        # Pre-compute reference scores (one-time cost).
        self._ideal_scores = load_all_ideal_scores(self._model, self._device)
        self._bad_scores = load_bad_image_scores(self._model, self._device)

        # Import the classify function.
        from evaluate import classify_winrate as _cw, preprocess_image
        self._classify_winrate = _cw
        self._preprocess = preprocess_image

    # ------------------------------------------------------------------

    def classify(self, frame: np.ndarray) -> dict:
        """Classify a single RHEED frame (H x W x 3 uint8 numpy array).

        Returns a dict with keys:
            predicted_class  – GUI-friendly label (from GUI_LABELS)
            classification_scores – {gui_label: float} win-rates (0-1)
            is_bad           – bool
            bad_confidence   – float
            quality          – float (0-1)
        """
        # Save frame to a temporary file (classify_winrate expects a path).
        import tempfile, os
        tmp = tempfile.NamedTemporaryFile(suffix=".png", delete=False)
        try:
            from PIL import Image
            Image.fromarray(frame).save(tmp.name)
            tmp.close()

            raw = self._classify_winrate(
                self._model,
                tmp.name,
                self._device,
                self._ideal_scores,
                bad_scores=self._bad_scores,
                bad_threshold=self._bad_threshold,
            )
        finally:
            os.unlink(tmp.name)

        # Translate Classifier2 labels → GUI labels.
        c2_scores = raw.get("classification_scores", {})
        gui_scores = {_C2_TO_GUI.get(k, k): v for k, v in c2_scores.items()}

        predicted = raw.get("predicted_class", "")
        gui_predicted = _C2_TO_GUI.get(predicted, predicted)

        return {
            "predicted_class": gui_predicted,
            "classification_scores": gui_scores,
            "is_bad": raw.get("is_bad", False),
            "bad_confidence": raw.get("bad_confidence", 0.0),
            "quality": raw.get("quality"),
        }
