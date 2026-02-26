"""
Safety-focused PID temperature control loop.

Uses:
- OWON power supply as actuator (voltage output)
- Dracal thermocouple as sensor

Key safety behavior:
- Explicit controller states (arming, running, hold, complete, fault)
- Hard over-temperature cutoff
- Sensor plausibility checks
- Sensor read timeout watchdog
- Time-to-target timeout
- Output voltage slew-rate limiting
- Structured CSV logging for post-run analysis

Requirements:
    pip install pyvisa pyvisa-py pyserial

Example:
    python temperature_pid_control.py --target 100 --hold-minutes 10
"""

from __future__ import annotations

import argparse
import csv
import math
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional

from dracal_thermocouple_reader import DracalTMC100k, auto_detect_port
from owon_power_supply import OWONPowerSupply, find_owon_supplies


class ControlState:
    ARMED = "ARMED"
    RUNNING = "RUNNING"
    COMPLETE = "COMPLETE"
    FAULT = "FAULT"
    STOPPED = "STOPPED"


@dataclass
class SafetyConfig:
    target_c: float
    margin_c: float
    hold_seconds: float
    max_voltage: float
    current_limit_a: float
    sample_time_s: float
    max_temp_hard_c: float
    min_temp_valid_c: float
    max_temp_valid_c: float
    max_temp_rate_c_per_min: float
    sensor_timeout_s: float
    target_reach_timeout_s: float
    max_voltage_step_v_per_s: float
    high_temp_threshold_c: float
    high_temp_voltage_step_v_per_s: float
    stability_window_s: float
    stability_error_c: float
    memory_length: int


@dataclass
class LoopSnapshot:
    timestamp: str
    elapsed_s: float
    state: str
    measured_c: float
    setpoint_c: float
    error_c: float
    output_v: float
    hold_elapsed_s: float
    in_margin: bool
    message: str


class RobustPID:
    """PID with anti-windup clamp and filtered derivative."""

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

    def reset(self) -> None:
        self._integral = 0.0
        self._last_error = None
        self._last_derivative = 0.0

    def update(self, setpoint: float, measured: float, dt: float) -> float:
        if dt <= 0:
            dt = 1e-3

        error = setpoint - measured

        # Integrator with clamp anti-windup.
        self._integral += error * dt
        self._integral = max(self.integral_min, min(self.integral_max, self._integral))

        raw_derivative = 0.0
        if self._last_error is not None:
            raw_derivative = (error - self._last_error) / dt

        # Exponential smoothing on derivative term to suppress noise spikes.
        self._last_derivative = (
            self.derivative_alpha * raw_derivative
            + (1.0 - self.derivative_alpha) * self._last_derivative
        )

        output = (
            self.kp * error
            + self.ki * self._integral
            + self.kd * self._last_derivative
        )

        self._last_error = error
        return max(self.output_min, min(self.output_max, output))


class CsvRunLogger:
    """CSV logger for loop snapshots and events."""

    def __init__(self, path: Path):
        self.path = path
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.path.open("w", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow(
            [
                "timestamp",
                "elapsed_s",
                "state",
                "measured_c",
                "setpoint_c",
                "error_c",
                "output_v",
                "hold_elapsed_s",
                "in_margin",
                "message",
            ]
        )

    def write(self, row: LoopSnapshot) -> None:
        self._writer.writerow(
            [
                row.timestamp,
                f"{row.elapsed_s:.3f}",
                row.state,
                f"{row.measured_c:.5f}",
                f"{row.setpoint_c:.5f}",
                f"{row.error_c:.5f}",
                f"{row.output_v:.5f}",
                f"{row.hold_elapsed_s:.3f}",
                int(row.in_margin),
                row.message,
            ]
        )
        self._file.flush()

    def close(self) -> None:
        self._file.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Safety-focused PID temperature controller")
    parser.add_argument("--target", type=float, required=True, help="Target temperature in C")
    parser.add_argument("--hold-minutes", type=float, default=5.0, help="Hold time in minutes")
    parser.add_argument("--margin", type=float, default=1.0, help="Allowed error margin in C")

    parser.add_argument("--kp", type=float, default=1.0, help="PID Kp")
    parser.add_argument("--ki", type=float, default=0.02, help="PID Ki")
    parser.add_argument("--kd", type=float, default=0.0, help="PID Kd")
    parser.add_argument("--derivative-alpha", type=float, default=0.2, help="Derivative filter alpha [0..1]")

    parser.add_argument("--sample-time", type=float, default=1.0, help="Loop period in seconds")
    parser.add_argument("--max-voltage", type=float, default=24.0, help="Absolute max output voltage")
    parser.add_argument("--current-limit", type=float, default=1.0, help="Current limit in A")
    parser.add_argument(
        "--max-voltage-step",
        type=float,
        default=1.0,
        help="Max output voltage change rate in V/s",
    )

    parser.add_argument("--max-temp-hard", type=float, default=75.0, help="Hard over-temp cutoff in C")
    parser.add_argument("--min-temp-valid", type=float, default=-50.0, help="Minimum valid sensor value")
    parser.add_argument("--max-temp-valid", type=float, default=400.0, help="Maximum valid sensor value")
    parser.add_argument(
        "--max-temp-rate",
        type=float,
        default=60.0,
        help="Max physically plausible rate in C/min",
    )
    parser.add_argument(
        "--sensor-timeout",
        type=float,
        default=5.0,
        help="Fault if no valid sensor sample within this many seconds",
    )
    parser.add_argument(
        "--target-timeout-minutes",
        type=float,
        default=30.0,
        help="Fault if setpoint margin is not reached in this many minutes",
    )
    parser.add_argument("--connect-retries", type=int, default=5, help="Connection retries per device")
    parser.add_argument("--connect-delay", type=float, default=2.0, help="Delay between connection retries (s)")

    parser.add_argument(
        "--stability-window",
        type=float,
        default=300.0,
        help="Window (s) used by is_at_setpoint stability logic",
    )
    parser.add_argument(
        "--stability-error",
        type=float,
        default=0.15,
        help="Allowed fluctuation around setpoint for stability logic",
    )
    parser.add_argument(
        "--memory-length",
        type=int,
        default=800,
        help="Temperature history length for stability logic",
    )
    parser.add_argument(
        "--high-temp-threshold",
        type=float,
        default=120.0,
        help="Above this temperature, auto-reduce output slew rate",
    )
    parser.add_argument(
        "--high-temp-max-voltage-step",
        type=float,
        default=0.5,
        help="Reduced output slew rate (V/s) when above high-temp threshold",
    )

    parser.add_argument(
        "--enable-gain-scheduling",
        action="store_true",
        help="Enable temperature-dependent PID gains",
    )
    parser.add_argument("--pid-threshold-1", type=float, default=60.0, help="Gain schedule threshold 1 (C) — Band 1/2 boundary")
    parser.add_argument("--pid-threshold-2", type=float, default=110.0, help="Gain schedule threshold 2 (C) — Band 2/3 boundary")
    parser.add_argument("--pid-ramp-band", type=float, default=10.0, help="Gain interpolation band width (C)")
    parser.add_argument("--p-low", type=float, default=1.0, help="Gain schedule low-temp P (Band 1: T < threshold-1)")
    parser.add_argument("--i-low", type=float, default=0.02, help="Gain schedule low-temp I")
    parser.add_argument("--d-low", type=float, default=0.0, help="Gain schedule low-temp D")
    parser.add_argument("--p-mid", type=float, default=1.0, help="Gain schedule mid-temp P (Band 2: threshold-1 to threshold-2)")
    parser.add_argument("--i-mid", type=float, default=0.02, help="Gain schedule mid-temp I")
    parser.add_argument("--d-mid", type=float, default=0.0, help="Gain schedule mid-temp D")
    parser.add_argument("--p-high", type=float, default=1.0, help="Gain schedule high-temp P (Band 3: T > threshold-2)")
    parser.add_argument("--i-high", type=float, default=0.02, help="Gain schedule high-temp I")
    parser.add_argument("--d-high", type=float, default=0.0, help="Gain schedule high-temp D")

    parser.add_argument("--owon-port", help="OWON serial port path (ex: /dev/tty.usbserial-XXXX)")
    parser.add_argument("--dracal-port", help="Dracal serial port path (ex: /dev/tty.usbmodemXXXX)")

    parser.add_argument(
        "--log-file",
        help="Optional CSV log file path (default: logs/pid_run_<timestamp>.csv)",
    )
    parser.add_argument(
        "--no-prompt",
        action="store_true",
        help="Skip interactive arming prompt",
    )

    return parser.parse_args()


def resolve_owon_resource(port: str | None) -> str:
    if port:
        return f"ASRL{port}::INSTR"

    supplies = find_owon_supplies()
    if not supplies:
        raise RuntimeError("No OWON power supply found. Provide --owon-port explicitly.")
    resource, idn = supplies[0]
    print(f"Using OWON: {idn.strip()} ({resource})")
    return resource


def resolve_dracal_port(port: str | None) -> str:
    if port:
        return port
    detected = auto_detect_port()
    if not detected:
        raise RuntimeError("No Dracal port found. Provide --dracal-port explicitly.")
    print(f"Using Dracal port: {detected}")
    return detected


def clamp_voltage_step(prev_v: float, desired_v: float, dt: float, max_step_v_per_s: float) -> float:
    if dt <= 0:
        return prev_v
    max_delta = max_step_v_per_s * dt
    delta = desired_v - prev_v
    if delta > max_delta:
        return prev_v + max_delta
    if delta < -max_delta:
        return prev_v - max_delta
    return desired_v


def connect_with_retry(name: str, connect_fn, retries: int, delay_s: float) -> None:
    last_error: Optional[Exception] = None
    for attempt in range(1, max(1, retries) + 1):
        try:
            connect_fn()
            return
        except Exception as exc:
            last_error = exc
            if attempt < retries:
                print(f"{name} connect attempt {attempt}/{retries} failed: {exc}. Retrying...")
                time.sleep(max(0.1, delay_s))
    raise RuntimeError(f"{name} connection failed after {retries} attempts: {last_error}")


def maybe_adjust_for_high_temp(temp_c: float, cfg: SafetyConfig) -> float:
    if temp_c > cfg.high_temp_threshold_c:
        return cfg.high_temp_voltage_step_v_per_s
    return cfg.max_voltage_step_v_per_s


def _lerp(a: float, b: float, r: float) -> float:
    return a + (b - a) * r


def scheduled_pid_gains(temp_c: float, args: argparse.Namespace) -> tuple[float, float, float]:
    p_vals = [args.p_low, args.p_mid, args.p_high]
    i_vals = [args.i_low, args.i_mid, args.i_high]
    d_vals = [args.d_low, args.d_mid, args.d_high]
    t1 = args.pid_threshold_1
    t2 = args.pid_threshold_2
    band = max(1e-6, args.pid_ramp_band)

    if temp_c < t1 - 0.5 * band:
        return p_vals[0], i_vals[0], d_vals[0]
    if temp_c < t1 + 0.5 * band:
        r = (temp_c - t1 + 0.5 * band) / band
        return _lerp(p_vals[0], p_vals[1], r), _lerp(i_vals[0], i_vals[1], r), _lerp(d_vals[0], d_vals[1], r)
    if temp_c < t2 - 0.5 * band:
        return p_vals[1], i_vals[1], d_vals[1]
    if temp_c < t2 + 0.5 * band:
        r = (temp_c - t2 + 0.5 * band) / band
        return _lerp(p_vals[1], p_vals[2], r), _lerp(i_vals[1], i_vals[2], r), _lerp(d_vals[1], d_vals[2], r)
    return p_vals[2], i_vals[2], d_vals[2]


def is_at_setpoint(
    history: deque[tuple[float, float]],
    setpoint_c: float,
    stability_window_s: float,
    stability_error_c: float,
    now_s: float,
) -> bool:
    recent = [(temp, ts) for temp, ts in history if now_s - ts < stability_window_s]
    if not recent:
        return False
    all_within = all(abs(temp - setpoint_c) <= stability_error_c for temp, _ in recent)
    old = [
        (temp, ts)
        for temp, ts in history
        if abs(now_s - ts - stability_window_s) <= 10.0
    ]
    old_stable = any(abs(temp - setpoint_c) <= stability_error_c for temp, _ in old)
    return all_within and old_stable


def validate_temperature(temp_c: float, cfg: SafetyConfig) -> Optional[str]:
    if not math.isfinite(temp_c):
        return "Sensor returned non-finite value"
    if temp_c < cfg.min_temp_valid_c or temp_c > cfg.max_temp_valid_c:
        return (
            f"Sensor value out of valid range: {temp_c:.3f} C "
            f"not in [{cfg.min_temp_valid_c:.3f}, {cfg.max_temp_valid_c:.3f}]"
        )
    if temp_c >= cfg.max_temp_hard_c:
        return f"Hard over-temperature limit reached: {temp_c:.3f} C >= {cfg.max_temp_hard_c:.3f} C"
    return None


def arm_prompt(args: argparse.Namespace, cfg: SafetyConfig) -> bool:
    if args.no_prompt:
        return True

    print("\n=== ARMING CHECK ===")
    print(f"Target:          {cfg.target_c:.2f} C")
    print(f"Hold duration:   {cfg.hold_seconds/60.0:.2f} min")
    print(f"Margin:          +/-{cfg.margin_c:.2f} C")
    print(f"Max voltage:     {cfg.max_voltage:.2f} V")
    print(f"Current limit:   {cfg.current_limit_a:.3f} A")
    print(f"Hard cutoff:     {cfg.max_temp_hard_c:.2f} C")
    print("Type ARM to start: ", end="", flush=True)
    return input().strip() == "ARM"


def main() -> int:
    args = parse_args()

    cfg = SafetyConfig(
        target_c=args.target,
        margin_c=max(0.0, args.margin),
        hold_seconds=max(0.0, args.hold_minutes * 60.0),
        max_voltage=max(0.1, args.max_voltage),
        current_limit_a=max(0.01, args.current_limit),
        sample_time_s=max(0.1, args.sample_time),
        max_temp_hard_c=args.max_temp_hard,
        min_temp_valid_c=args.min_temp_valid,
        max_temp_valid_c=args.max_temp_valid,
        max_temp_rate_c_per_min=max(0.1, args.max_temp_rate),
        sensor_timeout_s=max(0.5, args.sensor_timeout),
        target_reach_timeout_s=max(1.0, args.target_timeout_minutes * 60.0),
        max_voltage_step_v_per_s=max(0.01, args.max_voltage_step),
        high_temp_threshold_c=args.high_temp_threshold,
        high_temp_voltage_step_v_per_s=max(0.01, args.high_temp_max_voltage_step),
        stability_window_s=max(1.0, args.stability_window),
        stability_error_c=max(0.0, args.stability_error),
        memory_length=max(10, args.memory_length),
    )

    if cfg.min_temp_valid_c >= cfg.max_temp_valid_c:
        print("Configuration error: min valid temperature must be lower than max valid temperature.")
        return 2
    if cfg.max_temp_hard_c > cfg.max_temp_valid_c:
        print("Configuration error: hard cutoff must be <= max valid temperature.")
        return 2

    try:
        owon_resource = resolve_owon_resource(args.owon_port)
        dracal_port = resolve_dracal_port(args.dracal_port)
    except RuntimeError as exc:
        print(f"Setup error: {exc}")
        return 1

    log_path = Path(args.log_file) if args.log_file else Path("logs") / f"pid_run_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    logger = CsvRunLogger(log_path)

    pid = RobustPID(
        kp=args.kp,
        ki=args.ki,
        kd=args.kd,
        output_min=0.0,
        output_max=cfg.max_voltage,
        derivative_alpha=args.derivative_alpha,
    )

    psu = OWONPowerSupply(owon_resource)
    sensor = DracalTMC100k(port=dracal_port, poll_interval_ms=int(cfg.sample_time_s * 1000))

    state = ControlState.ARMED
    fault_message = ""

    hold_elapsed = 0.0
    reached_margin_once = False
    target_reach_elapsed = 0.0
    temp_history: deque[tuple[float, float]] = deque(maxlen=cfg.memory_length)

    loop_start = time.time()
    last_loop_time = loop_start
    last_sensor_time = loop_start
    last_temp: Optional[float] = None
    commanded_voltage = 0.0

    try:
        if not arm_prompt(args, cfg):
            print("Arming aborted by user.")
            state = ControlState.STOPPED
            return 0

        print("Connecting devices...")
        connect_with_retry("OWON", psu.connect, args.connect_retries, args.connect_delay)
        connect_with_retry("Dracal", sensor.connect, args.connect_retries, args.connect_delay)
        psu.set_current(cfg.current_limit_a)
        psu.set_voltage(0.0)
        psu.output_on()
        state = ControlState.RUNNING

        print(
            f"Started control loop | target={cfg.target_c:.2f} C | hold={cfg.hold_seconds/60.0:.2f} min | "
            f"margin=+/-{cfg.margin_c:.2f} C | log={log_path}"
        )

        while state == ControlState.RUNNING:
            now = time.time()
            dt = max(1e-3, now - last_loop_time)
            last_loop_time = now

            try:
                temp_c = sensor.read_temperature()
                last_sensor_time = now
            except Exception as exc:
                if now - last_sensor_time > cfg.sensor_timeout_s:
                    state = ControlState.FAULT
                    fault_message = f"Sensor timeout: {exc}"
                    break
                # Grace window before declaring fault.
                continue

            validation_error = validate_temperature(temp_c, cfg)
            if validation_error:
                state = ControlState.FAULT
                fault_message = validation_error
                break

            if last_temp is not None:
                rate_c_per_min = abs((temp_c - last_temp) / dt) * 60.0
                if rate_c_per_min > cfg.max_temp_rate_c_per_min:
                    state = ControlState.FAULT
                    fault_message = (
                        f"Temperature rate exceeded plausibility limit: {rate_c_per_min:.2f} C/min "
                        f"> {cfg.max_temp_rate_c_per_min:.2f} C/min"
                    )
                    break
            last_temp = temp_c
            temp_history.append((temp_c, now))

            if args.enable_gain_scheduling:
                pid.kp, pid.ki, pid.kd = scheduled_pid_gains(temp_c, args)

            desired_voltage = pid.update(setpoint=cfg.target_c, measured=temp_c, dt=dt)
            active_step_limit = maybe_adjust_for_high_temp(temp_c, cfg)
            commanded_voltage = clamp_voltage_step(
                prev_v=commanded_voltage,
                desired_v=desired_voltage,
                dt=dt,
                max_step_v_per_s=active_step_limit,
            )
            psu.set_voltage(commanded_voltage)

            error_c = cfg.target_c - temp_c
            in_margin = abs(error_c) <= cfg.margin_c

            if in_margin:
                hold_elapsed += dt
                reached_margin_once = True
            else:
                hold_elapsed = 0.0

            if not reached_margin_once:
                target_reach_elapsed += dt
                if target_reach_elapsed > cfg.target_reach_timeout_s:
                    state = ControlState.FAULT
                    fault_message = (
                        "Target was not reached within timeout: "
                        f"{target_reach_elapsed/60.0:.2f} min > {cfg.target_reach_timeout_s/60.0:.2f} min"
                    )
                    break

            elapsed = now - loop_start
            hold_remaining = max(0.0, cfg.hold_seconds - hold_elapsed)
            stable_window_ok = is_at_setpoint(
                temp_history,
                cfg.target_c,
                cfg.stability_window_s,
                cfg.stability_error_c,
                now,
            )
            message = "RUN_STABLE" if stable_window_ok else "RUN"

            print(
                f"t={elapsed:7.1f}s | T={temp_c:7.3f} C | SP={cfg.target_c:7.3f} C | "
                f"err={error_c:+7.3f} C | V={commanded_voltage:6.3f} V | "
                f"in_margin={'YES' if in_margin else 'NO '} | stable={'YES' if stable_window_ok else 'NO '} | "
                f"hold_rem={hold_remaining:7.1f}s"
            )

            logger.write(
                LoopSnapshot(
                    timestamp=datetime.now().isoformat(),
                    elapsed_s=elapsed,
                    state=state,
                    measured_c=temp_c,
                    setpoint_c=cfg.target_c,
                    error_c=error_c,
                    output_v=commanded_voltage,
                    hold_elapsed_s=hold_elapsed,
                    in_margin=in_margin,
                    message=message,
                )
            )

            if cfg.hold_seconds > 0 and hold_elapsed >= cfg.hold_seconds:
                state = ControlState.COMPLETE
                break

            time.sleep(cfg.sample_time_s)

    except KeyboardInterrupt:
        state = ControlState.STOPPED
        print("\nStopped by user.")
    except Exception as exc:
        state = ControlState.FAULT
        fault_message = f"Unhandled runtime fault: {exc}"
    finally:
        # Always force safe output state.
        try:
            psu.set_voltage(0.0)
            psu.output_off()
            psu.disconnect()
        except Exception:
            pass
        try:
            sensor.disconnect()
        except Exception:
            pass

        final_elapsed = time.time() - loop_start
        final_temp = last_temp if last_temp is not None else float("nan")

        if state == ControlState.FAULT:
            print(f"FAULT: {fault_message}")
            logger.write(
                LoopSnapshot(
                    timestamp=datetime.now().isoformat(),
                    elapsed_s=final_elapsed,
                    state=state,
                    measured_c=final_temp,
                    setpoint_c=cfg.target_c,
                    error_c=0.0 if not math.isfinite(final_temp) else cfg.target_c - final_temp,
                    output_v=0.0,
                    hold_elapsed_s=hold_elapsed,
                    in_margin=False,
                    message=fault_message,
                )
            )
        elif state == ControlState.COMPLETE:
            print("Hold requirement completed. Control run complete.")
            logger.write(
                LoopSnapshot(
                    timestamp=datetime.now().isoformat(),
                    elapsed_s=final_elapsed,
                    state=state,
                    measured_c=final_temp,
                    setpoint_c=cfg.target_c,
                    error_c=0.0 if not math.isfinite(final_temp) else cfg.target_c - final_temp,
                    output_v=0.0,
                    hold_elapsed_s=hold_elapsed,
                    in_margin=True,
                    message="Run complete",
                )
            )

        logger.close()
        print(f"Run log written: {log_path}")

    if state == ControlState.FAULT:
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
