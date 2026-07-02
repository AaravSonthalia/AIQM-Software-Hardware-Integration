#!/usr/bin/env python3
"""End-to-end test of VmbCamera against an in-process vmbpy mock.

Validates the streaming-callback pattern, connect/disconnect lifecycle,
palette conversion (the historically-buggy (0,I,0) → correct (I,I,I)
fix), and error propagation — all Mac-side, without needing the Allied
Vision Vimba X SDK or a real camera.

Approach: replace ``vmbpy`` in ``sys.modules`` with a fake module whose
``VmbSystem``/camera classes accept the SDK calls VmbCamera makes,
produce mock frames on demand, and record which methods were called
so tests can assert on the driver's behavior.

Usage:
    PYTHONPATH=. python scripts/test_vimba_camera.py

Exits 0 on success; raises AssertionError with a diagnostic on failure.

Built Jul 2 2026 as the pre-lab safety net for the direct-camera path.
If this passes on Mac, VmbCamera's threading + palette + error paths
work; anything that fails at Bulbasaur is then a real-SDK problem, not
a driver-logic problem.
"""

from __future__ import annotations

import sys
import threading
import time
import types
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent))


# ---------------------------------------------------------------------------
# Fake vmbpy — minimal shape that VmbCamera exercises
# ---------------------------------------------------------------------------

class FakeFrame:
    """Fake vmbpy Frame — mimics ``as_numpy_ndarray()`` returning a 2D view."""

    def __init__(self, img: np.ndarray):
        self._img = img

    def as_numpy_ndarray(self) -> np.ndarray:
        return self._img


class FakeSettable:
    """Camera setting proxy — captures which values were set for assertions."""

    def __init__(self):
        self.value: object = None

    def set(self, value) -> None:  # noqa: A003
        self.value = value


class FakeTriggerSoftware:
    """Represents cam.TriggerSoftware — exposes a run() that produces a frame.

    Each ``run()`` call either produces a frame (via the frame handler),
    raises the queued ``raise_on_run`` exception once, or does nothing if
    the camera is in a "silent" mode.
    """

    def __init__(self, camera: "FakeCamera"):
        self._camera = camera

    def run(self) -> None:
        self._camera.trigger_run_count += 1
        exc = self._camera.raise_on_run.pop(0) if self._camera.raise_on_run else None
        if exc is not None:
            raise exc
        if self._camera.silent:
            return
        # Produce one mock frame and pass it through the registered handler.
        # The frame's shape matches G-033B: 656x492 uint16 (12-bit values
        # stored in the low bits — but we use raw uint16 for simplicity).
        img = np.full(
            self._camera.frame_shape, self._camera.frame_value, dtype=np.uint16,
        )
        if self._camera.handler is not None:
            self._camera.handler(self._camera, None, FakeFrame(img))


class FakeCamera:
    """Fake vmbpy Camera — context-manager entry + streaming lifecycle."""

    def __init__(self):
        self.TriggerSource = FakeSettable()
        self.TriggerSelector = FakeSettable()
        self.TriggerMode = FakeSettable()
        self.AcquisitionMode = FakeSettable()
        self.TriggerSoftware = FakeTriggerSoftware(self)

        self.handler = None
        self.start_streaming_calls = 0
        self.stop_streaming_calls = 0
        self.queued_frames: list = []
        self.trigger_run_count = 0
        self.raise_on_run: list[Exception] = []

        # Frame shape + fill value — tests can override before triggering.
        self.frame_shape = (492, 656)
        self.frame_value = 2048  # mid-12-bit value

        # Modes: silent → run() doesn't dispatch to handler
        self.silent = False

        # Context-manager: __enter__ returns self, __exit__ is a no-op
        self.entered = False
        self.exited = False

    def __enter__(self) -> "FakeCamera":
        self.entered = True
        return self

    def __exit__(self, *exc_info) -> None:
        self.exited = True

    def start_streaming(self, handler) -> None:
        self.start_streaming_calls += 1
        self.handler = handler

    def stop_streaming(self) -> None:
        self.stop_streaming_calls += 1
        self.handler = None

    def queue_frame(self, frame) -> None:
        self.queued_frames.append(frame)


class FakeVmbSystem:
    """Fake vmbpy VmbSystem — get_instance() returns a context manager."""

    _cameras: list[FakeCamera] = []
    _raise_on_get_all_cameras: Exception | None = None

    @classmethod
    def get_instance(cls) -> "FakeVmbSystem":
        return cls()

    def __enter__(self) -> "FakeVmbSystem":
        return self

    def __exit__(self, *exc_info) -> None:
        pass

    def get_all_cameras(self) -> list[FakeCamera]:
        if FakeVmbSystem._raise_on_get_all_cameras is not None:
            raise FakeVmbSystem._raise_on_get_all_cameras
        return list(FakeVmbSystem._cameras)


def install_fake_vmbpy(
    cameras: list[FakeCamera] | None = None,
) -> FakeCamera | None:
    """Install a fake vmbpy module with the given cameras.

    Returns the first camera if any exist (for the common one-camera happy
    path), or None if the caller explicitly installed an empty list (to
    exercise the "no cameras found" error path).
    """
    if cameras is None:
        cameras = [FakeCamera()]
    FakeVmbSystem._cameras = cameras
    FakeVmbSystem._raise_on_get_all_cameras = None
    fake = types.SimpleNamespace(VmbSystem=FakeVmbSystem)
    sys.modules["vmbpy"] = fake
    return cameras[0] if cameras else None


def uninstall_fake_vmbpy() -> None:
    sys.modules.pop("vmbpy", None)
    FakeVmbSystem._cameras = []
    FakeVmbSystem._raise_on_get_all_cameras = None


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def test_palette_intensity_in_all_channels() -> None:
    """_to_rgb_uint8 writes intensity into R, G, AND B so PIL L equals intensity."""
    # Pure static test — doesn't need the fake SDK.
    from drivers.rheed_camera import VmbCamera
    cam = VmbCamera()
    mono = np.array([[0, 128, 255]], dtype=np.uint8)
    rgb = cam._to_rgb_uint8(mono)
    assert rgb.shape == (1, 3, 3), f"expected (1, 3, 3), got {rgb.shape}"
    # All three channels equal the input
    assert (rgb[:, :, 0] == mono).all(), f"R != intensity: {rgb[:, :, 0]}"
    assert (rgb[:, :, 1] == mono).all(), f"G != intensity: {rgb[:, :, 1]}"
    assert (rgb[:, :, 2] == mono).all(), f"B != intensity: {rgb[:, :, 2]}"
    # PIL L conversion: 0.299R + 0.587G + 0.114B — should equal intensity
    L = 0.299 * rgb[:, :, 0] + 0.587 * rgb[:, :, 1] + 0.114 * rgb[:, :, 2]
    assert (np.round(L).astype(np.uint8) == mono).all(), (
        f"L != intensity: L={L.round().astype(int).tolist()} vs {mono.tolist()}"
    )


def test_normalization_fixed_denominator() -> None:
    """12-bit uint16 input normalizes with /(2^12 - 1), not per-frame max."""
    from drivers.rheed_camera import VmbCamera
    cam = VmbCamera(bit_depth=12)
    # Two frames with different peak values but the same raw pixel value at [0, 0]
    frame_low = np.array([[2048]], dtype=np.uint16)
    frame_high = np.array([[[2048, 4095]]], dtype=np.uint16).reshape(1, 2)
    rgb_low = cam._to_rgb_uint8(frame_low)
    rgb_high = cam._to_rgb_uint8(frame_high)
    # The [0, 0] pixel MUST have the same value in both frames because the
    # denominator is fixed (per-frame max would make them differ).
    assert rgb_low[0, 0, 0] == rgb_high[0, 0, 0], (
        f"per-frame max normalization leaked: {rgb_low[0, 0, 0]} vs "
        f"{rgb_high[0, 0, 0]} — normalization is not fixed"
    )
    # Value is (2048/4095 * 255) → 127.506... → truncates to 127 via
    # astype(np.uint8). If we ever change to rounding this becomes 128 —
    # both are acceptable, but pin the current behavior explicitly.
    assert rgb_low[0, 0, 0] == 127, f"expected 127 (trunc), got {rgb_low[0, 0, 0]}"


def test_connect_then_read_frame() -> None:
    """After connect() + one trigger cycle, read_frame() returns a valid RGB frame."""
    try:
        fake_cam = install_fake_vmbpy()
        from drivers.rheed_camera import VmbCamera
        cam = VmbCamera(trigger_hz=100.0)  # fast trigger so a frame arrives quickly
        cam.connect()
        assert cam.connected
        # Give the stream loop a moment to trigger a few times.
        for _ in range(50):
            if fake_cam.start_streaming_calls > 0 and fake_cam.trigger_run_count > 0:
                break
            time.sleep(0.01)
        # Wait briefly for the frame handler to process at least one frame.
        for _ in range(50):
            with cam._frame_lock:
                if cam._latest_frame is not None:
                    break
            time.sleep(0.01)
        rgb = cam.read_frame()
        assert rgb.shape == (492, 656, 3), f"unexpected shape: {rgb.shape}"
        assert rgb.dtype == np.uint8, f"unexpected dtype: {rgb.dtype}"
        # Verify trigger config was actually set
        assert fake_cam.TriggerSource.value == "Software"
        assert fake_cam.TriggerMode.value == "On"
        assert fake_cam.AcquisitionMode.value == "Continuous"
        cam.disconnect()
        assert not cam.connected
        assert fake_cam.stop_streaming_calls >= 1, "stop_streaming was not called"
    finally:
        uninstall_fake_vmbpy()


def test_connect_no_cameras_raises() -> None:
    """No cameras found → RuntimeError with a clear diagnostic."""
    try:
        install_fake_vmbpy(cameras=[])
        from drivers.rheed_camera import VmbCamera
        cam = VmbCamera()
        try:
            cam.connect()
        except RuntimeError as exc:
            assert "no allied vision cameras found" in str(exc).lower(), (
                f"unexpected message: {exc}"
            )
            return
        raise AssertionError("expected RuntimeError, none raised")
    finally:
        uninstall_fake_vmbpy()


def test_connect_import_error_raises_early() -> None:
    """If vmbpy isn't importable, connect() raises ImportError before spawning thread."""
    # Ensure vmbpy is NOT in sys.modules.
    uninstall_fake_vmbpy()
    # Also block re-import in case the real vmbpy is present.
    sys.modules["vmbpy"] = None  # type: ignore
    try:
        from drivers.rheed_camera import VmbCamera
        cam = VmbCamera()
        try:
            cam.connect()
        except ImportError as exc:
            assert "vmbpy" in str(exc).lower(), f"unexpected message: {exc}"
            # Stream thread should NOT have been started
            assert cam._stream_thread is None, "thread should not have started"
            return
        raise AssertionError("expected ImportError, none raised")
    finally:
        uninstall_fake_vmbpy()


def test_read_frame_before_any_frame_raises_specific_error() -> None:
    """FrameNotYetAvailableError distinguishes warmup from real failure."""
    try:
        fake_cam = install_fake_vmbpy()
        fake_cam.silent = True  # trigger runs but no frame arrives
        from drivers.rheed_camera import VmbCamera, FrameNotYetAvailableError
        cam = VmbCamera(trigger_hz=100.0)
        cam.connect()
        try:
            cam.read_frame()
        except FrameNotYetAvailableError as exc:
            assert "no frame" in str(exc).lower(), f"unexpected message: {exc}"
        finally:
            cam.disconnect()
    finally:
        uninstall_fake_vmbpy()


def test_read_frame_before_connect_generic_runtime_error() -> None:
    """read_frame() before connect() raises a generic RuntimeError, NOT FrameNotYet."""
    try:
        install_fake_vmbpy()
        from drivers.rheed_camera import VmbCamera, FrameNotYetAvailableError
        cam = VmbCamera()
        try:
            cam.read_frame()
        except FrameNotYetAvailableError:
            raise AssertionError(
                "not-connected should raise generic RuntimeError, not "
                "FrameNotYetAvailableError — those are different failure modes"
            )
        except RuntimeError as exc:
            assert "not connected" in str(exc).lower(), (
                f"unexpected message: {exc}"
            )
    finally:
        uninstall_fake_vmbpy()


def test_stream_error_propagates_to_read_frame() -> None:
    """A stream-thread crash surfaces in read_frame() with the underlying cause."""
    try:
        install_fake_vmbpy()
        # Force get_all_cameras to raise → the stream loop's outer try
        # catches it, sets _stream_error, and unblocks connect via
        # _ready_event.set() in the finally block.
        FakeVmbSystem._raise_on_get_all_cameras = RuntimeError(
            "SDK ate the socket"
        )
        from drivers.rheed_camera import VmbCamera
        cam = VmbCamera()
        try:
            cam.connect()
        except RuntimeError as exc:
            # connect() itself raises because _stream_error is set before
            # returning — verify the original message survives.
            assert "SDK ate the socket" in str(exc), (
                f"expected chained error message, got: {exc}"
            )
            assert not cam.connected
            return
        raise AssertionError("expected RuntimeError from connect, none raised")
    finally:
        uninstall_fake_vmbpy()


def test_disconnect_idempotent_when_never_connected() -> None:
    """disconnect() on a never-connected driver is a no-op, not a raise."""
    from drivers.rheed_camera import VmbCamera
    cam = VmbCamera()
    cam.disconnect()  # should not raise
    assert not cam.connected


def test_reconnect_after_disconnect() -> None:
    """A fresh connect() cycle uses new event primitives — no stale state."""
    try:
        fake_cam = install_fake_vmbpy()
        from drivers.rheed_camera import VmbCamera
        cam = VmbCamera(trigger_hz=100.0)
        # First cycle
        cam.connect()
        assert cam.connected
        cam.disconnect()
        assert not cam.connected
        # Second cycle — the fake camera got __exit__'d, install a fresh one.
        fake_cam2 = install_fake_vmbpy()
        cam.connect()
        assert cam.connected
        cam.disconnect()
        # Both cycles start_streaming called
        assert fake_cam2.start_streaming_calls >= 1
    finally:
        uninstall_fake_vmbpy()


def test_bad_frame_recorded_but_stream_survives() -> None:
    """A handler exception on one frame is recorded but doesn't kill the loop."""
    try:
        fake_cam = install_fake_vmbpy()
        from drivers.rheed_camera import VmbCamera
        cam = VmbCamera(trigger_hz=100.0)
        cam.connect()

        # Wait for stream to produce a good frame.
        for _ in range(100):
            with cam._frame_lock:
                if cam._latest_frame is not None:
                    break
            time.sleep(0.01)

        # Sanity: we DID get a frame
        assert cam._latest_frame is not None, "no baseline frame produced"

        # Now switch the fake camera to produce garbage that breaks the
        # handler's numpy path — the handler must survive and continue.
        class BadFrame:
            def as_numpy_ndarray(self):
                raise ValueError("simulated bad frame")

        # Manually invoke the handler with a bad frame — represents the
        # SDK giving us a corrupted frame.
        cam._frame_handler(fake_cam, None, BadFrame())

        # The handler should have recorded the error, not raised out.
        with cam._error_lock:
            err = cam._last_frame_error
        assert err is not None and "simulated bad frame" in err, (
            f"expected recorded frame error, got: {err!r}"
        )
        # And the stream should still be alive — connected is still True
        assert cam.connected, "stream died on a single bad frame"
        cam.disconnect()
    finally:
        uninstall_fake_vmbpy()


def test_trigger_backoff_after_consecutive_fails() -> None:
    """MAX_CONSECUTIVE_TRIGGER_FAILS raises → backoff, then loop continues."""
    try:
        fake_cam = install_fake_vmbpy()
        from drivers.rheed_camera import VmbCamera
        # Force fake camera to raise on the first few TriggerSoftware.run() calls
        fake_cam.raise_on_run = [
            RuntimeError(f"transient {i}") for i in range(
                VmbCamera.MAX_CONSECUTIVE_TRIGGER_FAILS + 1
            )
        ]
        cam = VmbCamera(trigger_hz=100.0)
        # Reduce backoff for the test — actual constant is 2s, too slow
        cam.TRIGGER_BACKOFF_S = 0.05
        cam.connect()

        # Give the loop enough time to hit the failures + backoff + retry
        # (5 fails × ~10ms period + 50ms backoff = ~100ms, plus post-backoff runs)
        time.sleep(0.4)

        # Verify last_frame_error was recorded from a failed trigger
        with cam._error_lock:
            err = cam._last_frame_error
        assert err is not None and "TriggerSoftware.run" in err, (
            f"expected trigger fail recorded, got: {err!r}"
        )
        # Should still be connected — loop backed off, didn't die
        assert cam.connected
        cam.disconnect()
    finally:
        uninstall_fake_vmbpy()


# ---------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------

TESTS = [
    test_palette_intensity_in_all_channels,
    test_normalization_fixed_denominator,
    test_connect_then_read_frame,
    test_connect_no_cameras_raises,
    test_connect_import_error_raises_early,
    test_read_frame_before_any_frame_raises_specific_error,
    test_read_frame_before_connect_generic_runtime_error,
    test_stream_error_propagates_to_read_frame,
    test_disconnect_idempotent_when_never_connected,
    test_reconnect_after_disconnect,
    test_bad_frame_recorded_but_stream_survives,
    test_trigger_backoff_after_consecutive_fails,
]


def main() -> int:
    print(f"VmbCamera smoke test — {len(TESTS)} cases")
    print()
    failures: list[tuple[str, BaseException]] = []
    for t in TESTS:
        name = t.__name__
        try:
            t()
        except BaseException as exc:  # noqa: BLE001
            failures.append((name, exc))
            print(f"  ✗ {name}: {type(exc).__name__}: {exc}")
        else:
            print(f"  ✓ {name}")

    print()
    if failures:
        print(f"FAIL — {len(failures)}/{len(TESTS)} tests failed")
        return 1
    print(f"PASS — {len(TESTS)}/{len(TESTS)} tests passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
