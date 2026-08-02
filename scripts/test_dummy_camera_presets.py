"""Tests for repository-relative experimental DummyCamera presets."""
from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import numpy as np
from PIL import Image

from drivers.rheed_camera import DummyCamera
from gui.workers import RheedCameraWorker


class DummyCameraPresetTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.data_root = Path(self.temporary.name)
        values = {"1x1": 60, "c6x2": 130, "Twinned2x1": 210}
        for name, value in values.items():
            directory = self.data_root / f"STO_ideal_{name}"
            directory.mkdir()
            image = np.full((24, 32), value, dtype=np.uint8)
            Image.fromarray(image).save(directory / "frame_001.bmp")
        rt13_dir = self.data_root / "STO_ideal_RT13"
        rt13_dir.mkdir()
        rt13 = np.zeros((24, 32), dtype=np.uint8)
        rt13[5:19, 13:19] = 180
        Image.fromarray(rt13).save(rt13_dir / "RT13_20.png")

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def test_each_preset_loads_a_stable_experimental_frame(self) -> None:
        means = {}
        for preset in DummyCamera.PRESETS:
            camera = DummyCamera(
                width=64,
                height=48,
                preset=preset,
                data_root=self.data_root,
            )
            camera.connect()
            first = camera.read_frame()
            second = camera.read_frame()
            self.assertEqual(first.shape, (48, 64, 3))
            self.assertEqual(first.dtype, np.uint8)
            np.testing.assert_array_equal(first, second)
            self.assertIsNotNone(camera.source_path)
            means[preset] = float(first[:, :, 1].mean())
        self.assertEqual(len(set(means.values())), len(means))

    def test_default_data_root_is_workspace_relative(self) -> None:
        camera = DummyCamera()
        expected = (
            Path(__file__).resolve().parents[2]
            / "RHEEDClassify"
            / "data"
        )
        self.assertEqual(camera._data_root, expected)

    def test_worker_forwards_dummy_preset(self) -> None:
        for mode in DummyCamera.PRESETS:
            camera = RheedCameraWorker(mode=mode)._create_camera()
            self.assertIsInstance(camera, DummyCamera)
            self.assertEqual(camera.preset, mode)

    def test_rt13_demo_uses_named_source_and_fixed_rotation(self) -> None:
        camera = DummyCamera(
            width=64,
            height=48,
            preset="dummy_rt13_tilted",
            data_root=self.data_root,
        )
        camera.connect()
        frame = camera.read_frame()
        self.assertEqual(camera.source_path.name, "RT13_20.png")
        self.assertEqual(
            DummyCamera.ROTATION_DEGREES["dummy_rt13_tilted"],
            15.0,
        )
        self.assertGreater(int(frame[:, :, 1].max()), 0)


if __name__ == "__main__":
    unittest.main()
