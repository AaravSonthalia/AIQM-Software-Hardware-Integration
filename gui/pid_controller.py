"""
PID temperature controller — event-driven, lives on the main thread.

Receives TemperatureState readings via on_temp_state() called from
MainWindow's signal fan-out. Issues voltage commands to PowerSupplyWorker
via queue_command(). No serial ports opened here.

Safety:
- Hard cutoff at HARD_CUTOFF_C (300 °C): triggers emergency stop + FAULT
- Slew-rate limiting on voltage output
- Three gain bands with smooth interpolation (matching temp_pid.py pattern)
- PSU/TC disconnect detection while running → FAULT
"""

import math
import time
from dataclasses import dataclass, field
from typing import Optional

from PyQt6.QtCore import QObject, pyqtSignal

from gui.state import PowerSupplyState, TemperatureState
from gui.action_logger import ActionLogger


HARD_CUTOFF_C: float = 300.0  # absolute system ceiling — Config tab cannot exceed this


# ------------------------------------------------------------------
# Configuration dataclasses
# ------------------------------------------------------------------

@dataclass
class GainBand:
    kp: float
    ki: float
    kd: float


@dataclass
class PIDConfig:
    target_c: float
    hold_s: float               # seconds (0 = run until manually stopped)
    margin_c: float             # ±margin for "in setpoint" logic
    max_voltage: float          # V
    current_limit_a: float      # A
    slew_rate_v_per_s: float    # max output change rate
    bands: list                 # [GainBand × 3] — low / mid / high
    threshold_t1: float         # °C — Band 1 → Band 2 crossover
    threshold_t2: float         # °C — Band 2 → Band 3 crossover
    interp_band_c: float        # °C — width of smooth transition zone
    hard_cutoff_c: float = 150.0  # °C — emergency stop if temperature reaches this


# ------------------------------------------------------------------
# Live run state (emitted as signal payload)
# ------------------------------------------------------------------

@dataclass
class PIDRunState:
    controller_state: str = "IDLE"   # IDLE / ARMED / RUNNING / COMPLETE / FAULT / STOPPED
    setpoint_c: float = 0.0
    measured_c: float = 0.0
    error_c: float = 0.0
    output_v: float = 0.0
    hold_elapsed_s: float = 0.0
    hold_total_s: float = 0.0
    in_margin: bool = False
    elapsed_s: float = 0.0
    active_band: int = 0             # 0 / 1 / 2
    fault_message: str = ""
    psu_connected: bool = False
    tc_connected: bool = False


# ------------------------------------------------------------------
# PID algorithm (extracted from temperature_pid_control.py)
# ------------------------------------------------------------------

class _RobustPID:
    """Incremental PID with anti-windup clamp and filtered derivative."""

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        output_min: float,
        output_max: float,
        integral_min: float = -200.0,
        integral_max: float = 200.0,
        derivative_alpha: float = 0.2,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_min = integral_min
        self.integral_max = integral_max
        self.derivative_alpha = max(0.0, min(1.0, derivative_alpha))
        self._integral = 0.0
        self._last_error: Optional[float] = None
        self._last_derivative = 0.0

    def reset(self):
        self._integral = 0.0
        self._last_error = None
        self._last_derivative = 0.0

    def update(self, setpoint: float, measured: float, dt: float) -> float:
        dt = max(dt, 1e-3)
        error = setpoint - measured

        self._integral = max(
            self.integral_min,
            min(self.integral_max, self._integral + error * dt),
        )

        raw_d = (error - self._last_error) / dt if self._last_error is not None else 0.0
        self._last_derivative = (
            self.derivative_alpha * raw_d
            + (1.0 - self.derivative_alpha) * self._last_derivative
        )
        self._last_error = error

        out = self.kp * error + self.ki * self._integral + self.kd * self._last_derivative
        return max(self.output_min, min(self.output_max, out))


def _lerp(a: float, b: float, r: float) -> float:
    return a + (b - a) * max(0.0, min(1.0, r))


# ------------------------------------------------------------------
# Controller
# ------------------------------------------------------------------

class PIDController(QObject):
    """
    Event-driven PID controller for temperature regulation.

    Lifecycle:
        arm(config)  →  start()  →  [RUNNING]  →  stop() / complete / fault
        reset()  →  back to IDLE
    """

    pid_state_updated = pyqtSignal(object)   # payload: PIDRunState

    # Minimum seconds between "Measurement" action log entries (avoids flooding)
    _LOG_INTERVAL_S = 5.0

    def __init__(self, action_logger: ActionLogger, parent=None):
        super().__init__(parent)
        self.action_logger = action_logger

        self._psu_worker = None
        self._config: Optional[PIDConfig] = None
        self._pid: Optional[_RobustPID] = None

        self._run_state = PIDRunState()
        self._commanded_v: float = 0.0
        self._run_start: Optional[float] = None
        self._last_tick: Optional[float] = None
        self._hold_elapsed: float = 0.0
        self._last_log_time: float = 0.0

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def set_psu_worker(self, worker):
        """Called by MainWindow when a PSU worker is created or torn down."""
        self._psu_worker = worker

    def arm(self, config: PIDConfig):
        """Validate config and transition to ARMED."""
        if self._run_state.controller_state == "RUNNING":
            return
        effective_cutoff = min(config.hard_cutoff_c, HARD_CUTOFF_C)
        if config.target_c >= effective_cutoff:
            self._fault(
                f"Target {config.target_c:.1f} °C is at or above the "
                f"configured hard cutoff ({effective_cutoff:.1f} °C)"
            )
            return
        if config.threshold_t1 >= config.threshold_t2:
            self._fault("Gain schedule threshold T1 must be less than T2")
            return

        self._config = config
        self._pid = _RobustPID(
            kp=config.bands[0].kp,
            ki=config.bands[0].ki,
            kd=config.bands[0].kd,
            output_min=0.0,
            output_max=config.max_voltage,
        )

        self._run_state.controller_state = "ARMED"
        self._run_state.setpoint_c = config.target_c
        self._run_state.hold_total_s = config.hold_s
        self._run_state.hold_elapsed_s = 0.0
        self._run_state.elapsed_s = 0.0
        self._run_state.fault_message = ""

        self.action_logger.log(
            "PID", "Armed",
            f"Target={config.target_c:.1f}°C  Hold={config.hold_s/60:.1f}min  "
            f"MaxV={config.max_voltage:.1f}V  I-limit={config.current_limit_a:.3f}A"
        )
        self.pid_state_updated.emit(self._run_state)

    def start(self):
        """Begin the control loop. Must be ARMED first."""
        if self._run_state.controller_state != "ARMED":
            return
        if not self._psu_worker:
            return

        self._commanded_v = 0.0
        self._hold_elapsed = 0.0
        self._run_start = time.time()
        self._last_tick = self._run_start
        self._last_log_time = 0.0
        self._pid.reset()

        self._psu_worker.queue_command("set_current", self._config.current_limit_a)
        self._psu_worker.queue_command("set_voltage", 0.0)
        self._psu_worker.queue_command("output_on")

        self._run_state.controller_state = "RUNNING"
        self._run_state.hold_elapsed_s = 0.0
        self._run_state.elapsed_s = 0.0

        self.action_logger.log(
            "PID", "Started",
            f"Target={self._config.target_c:.1f}°C"
        )
        self.pid_state_updated.emit(self._run_state)

    def stop(self):
        """Graceful stop: zero output, log, transition to STOPPED."""
        if self._run_state.controller_state not in ("RUNNING", "ARMED"):
            return
        self._safe_shutdown(emergency=False)
        self._run_state.controller_state = "STOPPED"
        self.action_logger.log(
            "PID", "Stopped",
            f"elapsed={self._run_state.elapsed_s:.1f}s  "
            f"T={self._run_state.measured_c:.2f}°C"
        )
        self.pid_state_updated.emit(self._run_state)

    def emergency_stop(self):
        """Immediate shutdown: emergency_stop command to PSU, log, STOPPED."""
        was_running = self._run_state.controller_state in ("RUNNING", "ARMED")
        self._safe_shutdown(emergency=True)
        if self._run_state.controller_state != "FAULT":
            self._run_state.controller_state = "STOPPED"
        if was_running:
            self.action_logger.log(
                "PID", "Emergency Stop",
                f"T={self._run_state.measured_c:.2f}°C  Output zeroed"
            )
        self.pid_state_updated.emit(self._run_state)

    def reset(self):
        """Return to IDLE from any terminal state."""
        if self._run_state.controller_state in ("RUNNING", "ARMED"):
            self.stop()
        self._run_state = PIDRunState(
            psu_connected=self._run_state.psu_connected,
            tc_connected=self._run_state.tc_connected,
        )
        self._config = None
        self._pid = None
        self._commanded_v = 0.0
        self.pid_state_updated.emit(self._run_state)

    # ------------------------------------------------------------------
    # Slots called from MainWindow fan-out
    # ------------------------------------------------------------------

    def on_psu_state(self, state: PowerSupplyState):
        self._run_state.psu_connected = state.connected
        if not state.connected and self._run_state.controller_state == "RUNNING":
            self._fault("PSU connection lost during run")

    def on_temp_state(self, state: TemperatureState):
        self._run_state.tc_connected = state.connected

        if not state.connected and self._run_state.controller_state == "RUNNING":
            self._fault("Thermocouple connection lost during run")
            return

        if self._run_state.controller_state != "RUNNING":
            return

        now = time.time()
        dt = max(now - self._last_tick, 1e-3)
        self._last_tick = now
        temp_c = state.temperature

        # Hard cutoff — use the configured value, capped at the system ceiling
        effective_cutoff = min(self._config.hard_cutoff_c, HARD_CUTOFF_C)
        if temp_c >= effective_cutoff:
            self._fault(
                f"Hard cutoff reached: {temp_c:.2f} °C ≥ {effective_cutoff:.1f} °C",
                emergency=True,
            )
            return

        # Gain schedule
        active_band = self._apply_gain_schedule(temp_c)

        # PID step
        desired_v = self._pid.update(
            setpoint=self._config.target_c,
            measured=temp_c,
            dt=dt,
        )

        # Slew-rate clamp
        max_delta = self._config.slew_rate_v_per_s * dt
        delta = desired_v - self._commanded_v
        if abs(delta) > max_delta:
            desired_v = self._commanded_v + math.copysign(max_delta, delta)
        self._commanded_v = max(0.0, min(self._config.max_voltage, desired_v))

        if self._psu_worker:
            self._psu_worker.queue_command("set_voltage", self._commanded_v)

        # Hold logic
        error_c = self._config.target_c - temp_c
        in_margin = abs(error_c) <= self._config.margin_c
        if in_margin:
            self._hold_elapsed += dt
        else:
            self._hold_elapsed = 0.0

        elapsed = now - self._run_start

        # Update state
        self._run_state.measured_c = temp_c
        self._run_state.error_c = error_c
        self._run_state.output_v = self._commanded_v
        self._run_state.hold_elapsed_s = self._hold_elapsed
        self._run_state.in_margin = in_margin
        self._run_state.elapsed_s = elapsed
        self._run_state.active_band = active_band

        # Throttled action log for measurements
        if now - self._last_log_time >= self._LOG_INTERVAL_S:
            self.action_logger.log(
                "PID", "Measurement",
                f"T={temp_c:.2f}°C  SP={self._config.target_c:.1f}°C  "
                f"err={error_c:+.2f}°C  V={self._commanded_v:.3f}V  "
                f"hold={self._hold_elapsed:.1f}/{self._config.hold_s:.1f}s  "
                f"band={active_band + 1}"
            )
            self._last_log_time = now

        # Hold complete?
        if self._config.hold_s > 0 and self._hold_elapsed >= self._config.hold_s:
            self._safe_shutdown(emergency=False)
            self._run_state.controller_state = "COMPLETE"
            self.action_logger.log(
                "PID", "Complete",
                f"Hold of {self._config.hold_s / 60:.1f} min achieved at "
                f"{temp_c:.2f} °C"
            )

        self.pid_state_updated.emit(self._run_state)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _apply_gain_schedule(self, temp_c: float) -> int:
        """
        Update PID gains by interpolating across three temperature bands.
        Returns the primary active band index (0, 1, or 2).
        Mirrors the logic in temp_pid.py update_pid() / temperature_pid_control.py.
        """
        cfg = self._config
        t1, t2 = cfg.threshold_t1, cfg.threshold_t2
        half = cfg.interp_band_c / 2.0
        b = cfg.bands

        if temp_c < t1 - half:
            kp, ki, kd, idx = b[0].kp, b[0].ki, b[0].kd, 0
        elif temp_c < t1 + half:
            r = (temp_c - (t1 - half)) / max(cfg.interp_band_c, 1e-6)
            kp = _lerp(b[0].kp, b[1].kp, r)
            ki = _lerp(b[0].ki, b[1].ki, r)
            kd = _lerp(b[0].kd, b[1].kd, r)
            idx = 0
        elif temp_c < t2 - half:
            kp, ki, kd, idx = b[1].kp, b[1].ki, b[1].kd, 1
        elif temp_c < t2 + half:
            r = (temp_c - (t2 - half)) / max(cfg.interp_band_c, 1e-6)
            kp = _lerp(b[1].kp, b[2].kp, r)
            ki = _lerp(b[1].ki, b[2].ki, r)
            kd = _lerp(b[1].kd, b[2].kd, r)
            idx = 1
        else:
            kp, ki, kd, idx = b[2].kp, b[2].ki, b[2].kd, 2

        self._pid.kp, self._pid.ki, self._pid.kd = kp, ki, kd
        return idx

    def _safe_shutdown(self, emergency: bool = False):
        """Zero voltage and cut output on the PSU."""
        if self._psu_worker:
            if emergency:
                self._psu_worker.queue_command("emergency_stop")
            else:
                self._psu_worker.queue_command("set_voltage", 0.0)
                self._psu_worker.queue_command("output_off")
        self._commanded_v = 0.0
        self._run_state.output_v = 0.0

    def _fault(self, message: str, emergency: bool = False):
        """Shut down output, log fault, transition to FAULT state."""
        self._safe_shutdown(emergency=emergency)
        self._run_state.controller_state = "FAULT"
        self._run_state.fault_message = message
        self.action_logger.log("PID", "FAULT", message)
        self.pid_state_updated.emit(self._run_state)
