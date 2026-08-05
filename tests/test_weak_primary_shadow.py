"""Offline contract tests for the λ=0.1 GUI shadow integration."""
from __future__ import annotations

import os
import sys
import tempfile
import unittest
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt6.QtWidgets import QApplication

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from gui.growth_monitor import GrowthMonitor
from gui.state import WeakPrimaryShadowState
from gui.weak_primary_shadow import (
    WeakPrimaryShadowBridge,
    discover_lambda_point_one_checkpoints,
)

_app = QApplication.instance() or QApplication(sys.argv)


class CollectionDiscoveryTests(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        for fold in range(4):
            for pair in ("2022-02-04", "2022-02-06", "2022-04-11"):
                for seed in (17, 29, 43):
                    directory = self.root / (
                        f"fold_{fold}__pair_{pair}__seed_{seed}__lambda_0.1"
                    )
                    directory.mkdir()
                    (directory / "model.pth").touch()

    def tearDown(self) -> None:
        self.tmp.cleanup()

    def test_requires_all_36_cells_without_selecting_one_checkpoint(self) -> None:
        paths = discover_lambda_point_one_checkpoints(self.root)
        self.assertEqual(len(paths), 36)
        paths[0].unlink()
        with self.assertRaisesRegex(ValueError, "exactly 36"):
            discover_lambda_point_one_checkpoints(self.root)

    def test_checkpoint_contract_rejects_deployable_claim(self) -> None:
        payload = {
            "checkpoint_family": "weak_four_class_primary_v1",
            "execution_scope": "weak_shadow_only",
            "deployment_eligible": True,
            "lambda_pair": 0.1,
            "classes": ["twinned_2x1", "c_6x2", "rt13", "htr"],
            "one_by_one_class": False,
            "none_or_weak_class": False,
            "embedding_dim": 512,
            "temperature": 1.0,
            "model_state_dict": {},
            "data_manifest_sha256": "a" * 64,
            "embedding_cache": {},
        }
        with self.assertRaisesRegex(ValueError, "shadow contract"):
            WeakPrimaryShadowBridge._validate_checkpoint(
                payload, Path("model.pth")
            )


class ShadowDisplayTests(unittest.TestCase):
    def setUp(self) -> None:
        self.monitor = GrowthMonitor()

    def tearDown(self) -> None:
        self.monitor.close()

    def test_display_is_explicitly_shadow_and_does_not_touch_sliders(self) -> None:
        before = {
            name: slider.value()
            for name, slider in self.monitor._recon_sliders.items()
        }
        state = WeakPrimaryShadowState(
            loading=False,
            ready=True,
            last_frame_number=7,
            checkpoint_count=36,
            ensemble_id="weak-primary-lambda0.1-cv36-test",
            conditional_probabilities={
                "Twinned (2x1)": 0.1,
                "c(6x2)": 0.2,
                "rt13xrt13": 0.3,
                "HTR": 0.4,
            },
            checkpoint_disagreement=0.05,
            inference_ms=12.0,
            actionable=False,
        )
        self.monitor._classifier_output_exposure_recorded = True
        self.monitor.update_weak_primary_shadow_state(state)
        text = self.monitor._weak_primary_shadow_label.text()
        self.assertIn("SHADOW ONLY", text)
        self.assertIn("conditional four-class", text)
        self.assertIn("HTR 40.0%", text)
        self.assertEqual(
            before,
            {
                name: slider.value()
                for name, slider in self.monitor._recon_sliders.items()
            },
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
