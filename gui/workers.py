"""
Background worker threads for instrument communication.
"""

import time
from typing import Optional

import numpy as np
from PyQt6.QtCore import QMutex, QThread, pyqtSignal
import pyvisa

from gui.state import (
    CameraState, ClassifierState, EvapControlState, MistralState, PowerSupplyState,
    PyrometerState, TemperatureState,
)
from owon_power_supply import OWONPowerSupply


class PowerSupplyWorker(QThread):
    """Background thread for power supply communication."""

    state_updated = pyqtSignal(PowerSupplyState)

    def __init__(self, resource: str, poll_interval: float = 0.5):
        super().__init__()
        self.resource = resource
        self.poll_interval = poll_interval
        self.psu: Optional[OWONPowerSupply] = None
        self.running = False
        self._command_queue = []
        self._consecutive_failures = 0
        self._poll_counter = 0

    def run(self):
        """Main worker loop."""
        self.running = True
        state = PowerSupplyState()

        # Connect
        try:
            self.psu = OWONPowerSupply(self.resource)
            self.psu.connect()
            state.connected = True
        except Exception as e:
            state.connected = False
            state.error = str(e)
            self.state_updated.emit(state)
            return

        # Main polling loop
        while self.running:
            try:
                # Process any queued commands
                while self._command_queue:
                    cmd, args = self._command_queue.pop(0)
                    self._execute_command(cmd, args)

                # Read high-rate telemetry each cycle
                v, i, p = self.psu.measure_all()
                state.voltage_measured = v
                state.current_measured = i
                state.power_measured = p
                state.output_enabled = self.psu.get_output_state()

                # Read slower-moving values less frequently to reduce bus load
                if self._poll_counter % 5 == 0:
                    state.voltage_setpoint = self.psu.get_voltage_setpoint()
                    state.current_setpoint = self.psu.get_current_setpoint()
                    state.ovp_limit = self.psu.get_ovp()
                    state.ocp_limit = self.psu.get_ocp()

                self._poll_counter += 1
                state.connected = True
                state.error = ""
                self._consecutive_failures = 0

            except Exception as e:
                state.error = str(e)
                self._consecutive_failures += 1

                # If repeated VISA timeouts happen, force reconnect path
                if self._is_timeout_error(e) and self._consecutive_failures >= 3:
                    state.connected = False
                    if self._reconnect():
                        state.connected = True
                        state.error = ""
                        self._consecutive_failures = 0
                        self._poll_counter = 0

            self.state_updated.emit(state)
            time.sleep(self.poll_interval)

        # Cleanup
        if self.psu:
            try:
                self.psu.disconnect()
            except Exception:
                pass

    def _is_timeout_error(self, exc: Exception) -> bool:
        """Return True if exception is a VISA timeout."""
        if isinstance(exc, pyvisa.errors.VisaIOError):
            return exc.error_code == pyvisa.constants.VI_ERROR_TMO
        return "VI_ERROR_TMO" in str(exc)

    def _reconnect(self) -> bool:
        """Attempt to re-establish PSU connection after repeated failures."""
        try:
            if self.psu:
                try:
                    self.psu.disconnect()
                except Exception:
                    pass
            self.psu = OWONPowerSupply(self.resource)
            self.psu.connect()
            return True
        except Exception:
            return False

    def _execute_command(self, cmd: str, args: tuple):
        """Execute a command on the power supply. Raises on failure so the
        caller can surface the error through state.error."""
        if not self.psu:
            return

        if cmd == "set_voltage":
            self.psu.set_voltage(args[0])
        elif cmd == "set_current":
            self.psu.set_current(args[0])
        elif cmd == "output_on":
            self.psu.output_on()
        elif cmd == "output_off":
            self.psu.output_off()
        elif cmd == "set_ovp":
            self.psu.set_ovp(args[0])
        elif cmd == "set_ocp":
            self.psu.set_ocp(args[0])
        elif cmd == "emergency_stop":
            self.psu.output_off()
            self.psu.set_voltage(0)
            self.psu.set_current(0)

    def queue_command(self, cmd: str, *args):
        """Queue a command for execution."""
        self._command_queue.append((cmd, args))

    def stop(self):
        """Stop the worker thread."""
        self.running = False


class ThermocoupleWorker(QThread):
    """Background thread for Dracal TMC100k thermocouple communication."""

    state_updated = pyqtSignal(TemperatureState)

    def __init__(self, port: Optional[str] = None, poll_interval: float = 0.5):
        super().__init__()
        self.port = port
        self.poll_interval = poll_interval
        self.running = False
        self._sensor = None

    def run(self):
        """Main worker loop."""
        self.running = True
        state = TemperatureState()

        # Import here to avoid hard dependency at module level
        try:
            from dracal_tmc100k import DracalTMC100k, find_dracal_sensors
        except ImportError:
            state.error = "dracal_tmc100k module not available"
            self.state_updated.emit(state)
            return

        # Auto-detect port if not specified
        port = self.port
        if not port:
            try:
                sensors = find_dracal_sensors()
                if sensors:
                    port = sensors[0][0]
                    state.device_info = sensors[0][1]
            except Exception:
                pass

        if not port:
            state.error = "No Dracal TMC100k sensor detected."
            self.state_updated.emit(state)
            return

        # Connect
        try:
            self._sensor = DracalTMC100k(port=port)
            self._sensor.connect()
            state.connected = True

            # Get device info
            try:
                info = self._sensor.get_info()
                state.product_id = info.get("product_id", "")
                state.serial_number = info.get("serial_number", "")
                state.device_info = f"{state.product_id} {state.serial_number}".strip()
            except Exception:
                state.device_info = port

            self.state_updated.emit(state)
        except Exception as e:
            state.connected = False
            state.error = str(e)
            self.state_updated.emit(state)
            return

        # Main polling loop
        while self.running:
            try:
                tc_temp = self._sensor.read_temperature()
                state.temperature = tc_temp

                try:
                    state.cold_junction = self._sensor.read_cold_junction()
                except RuntimeError:
                    pass

                state.connected = True
                state.error = ""
                self.state_updated.emit(state)

            except Exception as e:
                state.error = str(e)
                self.state_updated.emit(state)

            time.sleep(self.poll_interval)

        # Cleanup
        if self._sensor:
            try:
                self._sensor.disconnect()
            except Exception:
                pass

    def stop(self):
        """Stop the thermocouple worker thread."""
        self.running = False


class RheedCameraWorker(QThread):
    """Background thread for RHEED camera frame acquisition."""

    state_updated = pyqtSignal(CameraState)

    def __init__(self, mode: str = "dummy", poll_interval: float = 1.0):
        super().__init__()
        self.mode = mode
        self.poll_interval = poll_interval
        self.running = False
        self._camera = None

    def run(self):
        """Main worker loop — connect camera and emit frames."""
        self.running = True
        state = CameraState(mode=self.mode)

        # Create camera driver based on mode
        try:
            self._camera = self._create_camera()
            self._camera.connect()
            state.connected = True
        except Exception as e:
            state.connected = False
            state.error = str(e)
            self.state_updated.emit(state)
            return

        frame_count = 0
        fps_start = time.time()
        fps_frame_count = 0

        while self.running:
            try:
                frame = self._camera.read_frame()
                frame_count += 1
                fps_frame_count += 1

                # Compute FPS over a rolling 1-second window
                elapsed = time.time() - fps_start
                if elapsed >= 1.0:
                    state.fps = fps_frame_count / elapsed
                    fps_start = time.time()
                    fps_frame_count = 0

                state.frame = frame
                state.frame_number = frame_count
                state.height, state.width = frame.shape[:2]
                state.connected = True
                state.error = ""

            except Exception as e:
                state.error = str(e)
                state.frame = None

            self.state_updated.emit(state)
            time.sleep(self.poll_interval)

        # Cleanup
        if self._camera is not None:
            try:
                self._camera.disconnect()
            except Exception:
                pass

    def _create_camera(self):
        """Factory method — import and instantiate camera driver."""
        if self.mode == "direct":
            from drivers.rheed_camera import VmbCamera
            return VmbCamera()
        elif self.mode == "screengrab":
            from drivers.rheed_camera import ScreenGrabCamera
            return ScreenGrabCamera()
        else:
            from drivers.rheed_camera import DummyCamera
            return DummyCamera()

    def stop(self):
        """Stop the camera worker thread."""
        self.running = False


class PyrometerWorker(QThread):
    """Background thread for pyrometer temperature polling."""

    state_updated = pyqtSignal(PyrometerState)

    def __init__(
        self,
        mode: str = "dummy",
        poll_interval: float = 0.5,
        samples_per_poll: int = 5,
        port: str = "COM4",
        baudrate: int = 115200,
    ):
        super().__init__()
        self.mode = mode
        self.poll_interval = poll_interval
        # Number of rapid sub-readings to average per poll cycle. 5 ≈ 0.5 s
        # at the Exactus default rate (~10 reads/s); for screengrab mode it
        # samples whatever jitter the GUI exposes between refreshes.
        self.samples_per_poll = max(1, int(samples_per_poll))
        self.port = port
        self.baudrate = baudrate
        self.running = False
        self._sensor = None

    def run(self):
        """Main worker loop — connect pyrometer and emit temperatures."""
        self.running = True
        state = PyrometerState(mode=self.mode)

        try:
            self._sensor = self._create_sensor()
            self._sensor.connect()
            state.connected = True

            # Get device info if available
            if hasattr(self._sensor, "get_info"):
                try:
                    info = self._sensor.get_info()
                    parts = []
                    if "name" in info:
                        parts.append(info["name"])
                    if "serial" in info:
                        parts.append(f"S/N: {info['serial']}")
                    state.device_info = " | ".join(parts) if parts else self.mode
                except Exception:
                    state.device_info = self.mode
            else:
                state.device_info = self.mode

            self.state_updated.emit(state)

        except Exception as e:
            state.connected = False
            state.error = str(e)
            self.state_updated.emit(state)
            return

        while self.running:
            # Take N rapid sub-readings within each poll cycle so the
            # emitted state carries a mean ± std rather than a single
            # noisy point estimate. Polybot-inspired statistical
            # consistency without a second analysis pass.
            readings: list[float] = []
            for _ in range(self.samples_per_poll):
                try:
                    readings.append(self._sensor.read_temperature())
                except Exception as e:
                    state.error = str(e)
                    break
            if readings:
                arr = np.asarray(readings, dtype=float)
                state.temperature = float(arr.mean())
                state.temperature_std = float(arr.std(ddof=0)) if arr.size > 1 else 0.0
                state.temperature_n = int(arr.size)
                state.connected = True
                state.error = ""

            # Emissivity is a slow-moving config value — single read is fine.
            if hasattr(self._sensor, "read_emissivity"):
                try:
                    state.emissivity = self._sensor.read_emissivity()
                except Exception:
                    pass

            self.state_updated.emit(state)
            time.sleep(self.poll_interval)

        # Cleanup
        if self._sensor is not None:
            try:
                self._sensor.disconnect()
            except Exception:
                pass

    def _create_sensor(self):
        """Factory method — import and instantiate pyrometer driver."""
        if self.mode == "modbus":
            from drivers.pyrometer import ModbusPyrometer
            return ModbusPyrometer()
        elif self.mode == "screengrab":
            from drivers.pyrometer import ScreenGrabPyrometer
            return ScreenGrabPyrometer()
        elif self.mode == "exactus":
            from drivers.pyrometer import ExactusSerialPyrometer
            return ExactusSerialPyrometer(port=self.port, baudrate=self.baudrate)
        else:
            from drivers.pyrometer import DummyPyrometer
            return DummyPyrometer()

    def stop(self):
        """Stop the pyrometer worker thread."""
        self.running = False


class MistralWorker(QThread):
    """Background thread for MistralGui V/I OCR polling.

    Uses lazy connect — if MISTRAL isn't running when the worker starts,
    the loop keeps trying each cycle until the window appears. Same
    behavior if MISTRAL is closed mid-session.
    """

    state_updated = pyqtSignal(MistralState)

    def __init__(self, mode: str = "screengrab", poll_interval: float = 1.0):
        super().__init__()
        self.mode = mode
        self.poll_interval = poll_interval
        self.running = False
        self._driver = None

    def run(self):
        self.running = True
        state = MistralState(mode=self.mode)
        self._driver = self._create_driver()

        while self.running:
            if not self._driver.connected:
                try:
                    self._driver.connect()
                    state.connected = True
                    state.error = ""
                except Exception as e:
                    state.connected = False
                    state.error = str(e)
                    self.state_updated.emit(state)
                    time.sleep(self.poll_interval)
                    continue

            try:
                vals = self._driver.read()
                state.v_set = vals.get("v_set")
                state.v_actual = vals.get("v_actual")
                state.i_set = vals.get("i_set")
                state.i_actual = vals.get("i_actual")
                state.connected = True
                state.error = ""
            except Exception as e:
                state.error = str(e)

            self.state_updated.emit(state)
            time.sleep(self.poll_interval)

        if self._driver is not None:
            try:
                self._driver.disconnect()
            except Exception:
                pass

    def _create_driver(self):
        if self.mode == "screengrab":
            from drivers.mistral import MistralGui
            return MistralGui()
        elif self.mode == "jsonrpc":
            # Direct-read via the Jun 23 2026 discovered backend at
            # http://10.0.42.231:9000/api (see docs/mistral_jsonrpc_discovery.md
            # and drivers/mistral_jsonrpc.py). Multi-client safe at the HTTP
            # layer — the client won't disrupt a live MistralGui session.
            #
            # Read-config is NOT populated yet: until the discovery probes
            # identify the actual V/I method names, read() returns the
            # 4-key all-None dict. The worker still emits state (connected
            # true, values None) so the GUI shows a working driver with
            # no readings — same shape as before, no crashes.
            #
            # After discovery: populate MistralJsonRpcClient.set_read_config
            # here (or add a DEFAULT_READ_CONFIG class attribute to the
            # client) with the discovered method names, then this driver
            # replaces MistralGui as the primary path.
            from drivers.mistral_jsonrpc import MistralJsonRpcClient
            return MistralJsonRpcClient()
        else:
            from drivers.mistral import DummyMistralGui
            return DummyMistralGui()

    def stop(self):
        self.running = False


class EvapControlWorker(QThread):
    """Background thread for Evap Control chamber pressure OCR polling."""

    state_updated = pyqtSignal(EvapControlState)

    def __init__(self, mode: str = "screengrab", poll_interval: float = 1.0):
        super().__init__()
        self.mode = mode
        self.poll_interval = poll_interval
        self.running = False
        self._driver = None

    def run(self):
        self.running = True
        state = EvapControlState(mode=self.mode)
        self._driver = self._create_driver()

        while self.running:
            if not self._driver.connected:
                try:
                    self._driver.connect()
                    state.connected = True
                    state.error = ""
                except Exception as e:
                    state.connected = False
                    state.error = str(e)
                    self.state_updated.emit(state)
                    time.sleep(self.poll_interval)
                    continue

            try:
                vals = self._driver.read()
                # Pressure: populated by both screengrab and elog modes.
                state.chamber_pressure_mbar = vals.get("chamber_pressure_mbar")
                # Substrate + cells + plasma: populated by elog mode only;
                # screengrab returns None for these keys. The same field
                # set is fanned out either way — downstream consumers see
                # None for variables not present in the active mode.
                state.substrate_temp_pv_C = vals.get("substrate_temp_pv_C")
                state.substrate_temp_setpoint_C = vals.get("substrate_temp_setpoint_C")
                state.cell_HTEC2_pv_C = vals.get("cell_HTEC2_pv_C")
                state.cell_Y_pv_C = vals.get("cell_Y_pv_C")
                state.cell_Sr_pv_C = vals.get("cell_Sr_pv_C")
                state.cell_Eu_pv_C = vals.get("cell_Eu_pv_C")
                state.cell_Er_pv_C = vals.get("cell_Er_pv_C")
                state.plasma_dc_bias_V = vals.get("plasma_dc_bias_V")
                state.plasma_forward_W = vals.get("plasma_forward_W")
                state.plasma_reflected_W = vals.get("plasma_reflected_W")
                state.connected = True
                state.error = ""
            except Exception as e:
                state.error = str(e)

            self.state_updated.emit(state)
            time.sleep(self.poll_interval)

        if self._driver is not None:
            try:
                self._driver.disconnect()
            except Exception:
                pass

    def _create_driver(self):
        if self.mode == "screengrab":
            from drivers.evap_control import EvapControl
            return EvapControl()
        elif self.mode == "elog":
            from drivers.evap_control import ElogReader
            return ElogReader()
        else:
            from drivers.evap_control import DummyEvapControl
            return DummyEvapControl()

    def stop(self):
        self.running = False


class ClassifierWorker(QThread):
    """Runs Classifier2 inference off the UI thread at a bounded rate.

    Consumes RHEED frames via :py:meth:`on_rheed_state` — connect this
    slot to ``RheedCameraWorker.state_updated``. Publishes
    ``ClassifierState`` with EMA-smoothed percentages normalized via the
    "Equalizer recipe" (clip negatives → divide by sum → uniform
    fallback), matching ``scripts/equalizer_ui.py:auto_fit``.

    Threading contract:
        - :py:meth:`on_rheed_state` runs in the SENDER's thread (the RHEED
          worker's thread). It writes the latest frame under a QMutex —
          no blocking work, just an attribute swap. Drop-old semantics:
          stale unclassified frames are silently overwritten by newer
          ones, preventing queue growth when the classifier is slower
          than the camera.
        - :py:meth:`run` executes in this worker's own thread. It reads
          the latest frame under the mutex, classifies, normalizes,
          updates the EMA, and emits ``state_updated`` at
          ``POLL_INTERVAL_S`` cadence.

    OOD handling:
        When the classifier's ``quality`` drops below
        ``OOD_QUALITY_THRESHOLD``, ``ClassifierState.is_ood`` is set True
        and the EMA is NOT advanced — the smoothed percentages freeze at
        their last confident values. The UI is expected to grey out the
        sliders while ``is_ood``.

    Failure handling:
        - Startup: bridge load failure emits ``error`` and returns
          (thread ends; recreate the worker to retry).
        - Runtime: classify failures are counted; after
          ``MAX_CONSECUTIVE_FAILS`` in a row the worker emits an error
          state and resets the counter. The loop continues so a transient
          issue can recover on its own.
    """

    state_updated = pyqtSignal(ClassifierState)

    # Class-level knobs — instance-override in tests via monkey-patching.
    POLL_INTERVAL_S = 0.5           # 2 Hz classification cadence
    EMA_ALPHA = 0.2                 # ~5 s time constant at 2 Hz
    OOD_QUALITY_THRESHOLD = 0.3     # below this = freeze EMA + set is_ood
    MAX_CONSECUTIVE_FAILS = 5       # symmetric with Jul-2 driver-hardening pattern

    def __init__(self, ai_repo_root, model_path=None):
        super().__init__()
        self.ai_repo_root = ai_repo_root
        self.model_path = model_path
        self.running = False

        # Frame handoff — mutex-protected latest frame (drop-old, not FIFO).
        self._frame_mutex = QMutex()
        self._latest_frame: Optional[np.ndarray] = None
        self._latest_frame_number = -1

        # Per-cycle state carried across iterations.
        self._smoothed: dict[str, float] = {}   # float internal for EMA math
        self._consecutive_failures = 0
        self._last_classified_frame_number = -1

    # ---- Slot: runs in the sender's thread; mutex-protected write ----
    def on_rheed_state(self, camera_state: CameraState) -> None:
        """Store the latest frame + number under mutex. Runs in sender's thread.

        Fast attribute swap; no I/O, no inference. Silently overwrites any
        prior unclassified frame (drop-old semantics).
        """
        if camera_state.frame is None:
            return
        self._frame_mutex.lock()
        try:
            self._latest_frame = camera_state.frame
            self._latest_frame_number = camera_state.frame_number
        finally:
            self._frame_mutex.unlock()

    def _create_bridge(self):
        """Factory for the ClassifierBridge. Override in tests via monkey-patch."""
        from gui.classifier_bridge import ClassifierBridge
        return ClassifierBridge(self.ai_repo_root, self.model_path)

    def run(self) -> None:
        """Main loop — load bridge once, then classify at POLL_INTERVAL_S."""
        from gui.recon_labels import RECON_LABELS

        self.running = True

        # Emit initial loading state so the UI can show "Loading classifier…"
        state = ClassifierState()  # loading=True, ready=False, error=""
        self.state_updated.emit(state)

        # Load bridge — blocking, ~1-2 s. This is precisely why we're on our
        # own thread: the UI stays responsive during model load.
        try:
            bridge = self._create_bridge()
        except Exception as e:
            state.loading = False
            state.ready = False
            state.error = f"Failed to load classifier: {e}"
            self.state_updated.emit(state)
            return

        # Ready — initialize EMA at uniform, emit uniform placeholder.
        # First real inference will EMA-blend into this baseline, so the
        # sliders visibly "settle" onto the model's answer rather than
        # snapping — feels more natural to growers.
        self._smoothed = {lbl: 20.0 for lbl in RECON_LABELS}
        uniform = {lbl: 20 for lbl in RECON_LABELS}
        state.loading = False
        state.ready = True
        state.error = ""
        state.normalized_percent = uniform.copy()
        state.smoothed_percent = uniform.copy()
        self.state_updated.emit(state)

        # Main polling loop
        while self.running:
            # Snapshot the latest frame under mutex
            self._frame_mutex.lock()
            try:
                frame = self._latest_frame
                frame_number = self._latest_frame_number
            finally:
                self._frame_mutex.unlock()

            # Skip if no frame yet, or same frame we already classified
            if frame is None or frame_number == self._last_classified_frame_number:
                time.sleep(self.POLL_INTERVAL_S)
                continue

            # Classify — the heavy work happens here in this worker's thread
            t0 = time.time()
            try:
                result = bridge.classify(frame)
                inference_ms = (time.time() - t0) * 1000.0
                self._consecutive_failures = 0
            except Exception as e:
                self._consecutive_failures += 1
                if self._consecutive_failures >= self.MAX_CONSECUTIVE_FAILS:
                    state.error = (
                        f"Classifier failed {self._consecutive_failures}x: {e}"
                    )
                    self.state_updated.emit(state)
                    self._consecutive_failures = 0  # reset after emitting; retry
                time.sleep(self.POLL_INTERVAL_S)
                continue

            self._last_classified_frame_number = frame_number

            # Normalize via Equalizer recipe (clip → sum → divide → uniform fallback)
            scores = result.get("classification_scores", {}) or {}
            normalized, raw_sum = self._normalize(scores)

            # OOD gate: freeze the EMA when the model isn't confident
            quality = float(result.get("quality") or 0.0)
            is_ood = quality < self.OOD_QUALITY_THRESHOLD

            if not is_ood:
                for lbl in RECON_LABELS:
                    new = float(normalized.get(lbl, 0))
                    self._smoothed[lbl] = (
                        self.EMA_ALPHA * new
                        + (1.0 - self.EMA_ALPHA) * self._smoothed.get(lbl, 0.0)
                    )

            # Emit populated state
            state.error = ""
            state.last_frame_number = frame_number
            state.raw_scores = dict(scores)
            state.normalized_percent = normalized
            state.smoothed_percent = {
                lbl: int(round(v)) for lbl, v in self._smoothed.items()
            }
            state.raw_sum = raw_sum
            state.quality = quality
            state.is_bad = bool(result.get("is_bad", False))
            state.bad_confidence = float(result.get("bad_confidence", 0.0))
            state.is_ood = is_ood
            state.inference_ms = inference_ms
            self.state_updated.emit(state)

            time.sleep(self.POLL_INTERVAL_S)

    def stop(self) -> None:
        """Signal the run loop to exit at its next iteration."""
        self.running = False

    @staticmethod
    def _normalize(scores: dict) -> tuple[dict, float]:
        """Equalizer recipe: clip → sum → divide → uniform fallback.

        Returns ``(percentages, raw_sum)`` where:
            - ``percentages`` — dict[label, int] summing to 100 across
              ``RECON_LABELS``.
            - ``raw_sum`` — sum of the raw scores BEFORE clip, for the
              "Sum: X.XX" transparency label on the UI. Tells the grower
              how much scaling the display is doing.

        Uniform fallback triggers when the positive mass is zero (all
        scores <= 0), producing 20/20/20/20/20 as a neutral display.
        """
        from gui.recon_labels import RECON_LABELS
        vals = np.array(
            [float(scores.get(lbl, 0.0)) for lbl in RECON_LABELS],
            dtype=np.float64,
        )
        raw_sum = float(vals.sum())  # BEFORE clip — for transparency
        vals = np.clip(vals, 0.0, None)
        s = vals.sum()
        if s <= 0:
            vals = np.full(len(RECON_LABELS), 1.0 / len(RECON_LABELS))
        else:
            vals = vals / s
        return (
            {lbl: int(round(100 * v)) for lbl, v in zip(RECON_LABELS, vals)},
            raw_sum,
        )
