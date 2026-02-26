"""
Open-loop voltage step test for thermal system characterization.

Applies a configurable sequence of voltages to the heater via the OWON
power supply, records voltage/current/power and temperature from the Dracal
thermocouple at each step, and saves everything to a timestamped CSV.

Purpose:
    Use the step-response data to estimate the heater's steady-state thermal
    gain (°C/W) and time constant (τ) across different temperature ranges.
    These measurements inform initial PID gain selection before closed-loop
    runs.  Specifically:

        K  = ΔT_ss / ΔP      [°C/W]  — steady-state gain at each power level
        τ  = time to reach 63.2% of ΔT_ss after a step
        θ  = dead time (lag before temperature begins to respond)

    If K or τ change noticeably between temperature ranges, that supports
    using a multi-band gain schedule in the PID.

Safety:
    A hard temperature cutoff immediately zeroes voltage and cuts PSU output
    if the thermocouple reads ≥ --max-temp.  Ctrl-C triggers the same safe
    shutdown.

Usage:
    python heater_step_test.py
    python heater_step_test.py --steps 2 4 8 12 18 24 --hold-time 120
    python heater_step_test.py --steps 2 4 6 8 10 --hold-time 60 --max-temp 100
    python heater_step_test.py --owon-port /dev/tty.usbserial-XXXX --dracal-port /dev/tty.usbmodemXXXX
"""

from __future__ import annotations

import argparse
import csv
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

from owon_power_supply import OWONPowerSupply, find_owon_supplies
from dracal_tmc100k import DracalTMC100k, find_dracal_sensors


DEFAULT_STEPS_V = [2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 16.0, 20.0, 24.0]
DEFAULT_HOLD_S = 90.0
DEFAULT_SAMPLE_S = 1.0
DEFAULT_MAX_TEMP_C = 75.0
DEFAULT_CURRENT_LIMIT_A = 1.0
DEFAULT_COOLDOWN_S = 60.0


# ------------------------------------------------------------------
# CSV logger
# ------------------------------------------------------------------

class StepTestLogger:
    """Logs all step-test measurements to a single CSV file."""

    COLUMNS = [
        "timestamp",
        "elapsed_s",
        "phase",
        "step_index",
        "setpoint_v",
        "voltage_v",
        "current_a",
        "power_w",
        "temperature_c",
        "cold_junction_c",
    ]

    def __init__(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        self._file = path.open("w", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow(self.COLUMNS)
        self._start = time.time()

    def write(
        self,
        phase: str,
        step_index: int,
        setpoint_v: float,
        voltage_v: float,
        current_a: float,
        power_w: float,
        temperature_c: float,
        cold_junction_c: float,
    ) -> None:
        self._writer.writerow([
            datetime.now().isoformat(),
            f"{time.time() - self._start:.3f}",
            phase,
            step_index,
            f"{setpoint_v:.3f}",
            f"{voltage_v:.4f}",
            f"{current_a:.4f}",
            f"{power_w:.4f}",
            f"{temperature_c:.4f}",
            f"{cold_junction_c:.4f}",
        ])
        self._file.flush()

    def close(self) -> None:
        self._file.close()


# ------------------------------------------------------------------
# Hardware discovery
# ------------------------------------------------------------------

def resolve_owon(port: Optional[str]) -> str:
    if port:
        return f"ASRL{port}::INSTR"
    supplies = find_owon_supplies()
    if not supplies:
        raise RuntimeError(
            "No OWON power supply found. Specify --owon-port explicitly."
        )
    resource, idn = supplies[0]
    print(f"  Auto-detected OWON: {idn} ({resource})")
    return resource


def resolve_dracal(port: Optional[str]) -> str:
    if port:
        return port
    sensors = find_dracal_sensors()
    if not sensors:
        raise RuntimeError(
            "No Dracal sensor found. Specify --dracal-port explicitly."
        )
    detected_port, description = sensors[0]
    print(f"  Auto-detected Dracal: {description} ({detected_port})")
    return detected_port


# ------------------------------------------------------------------
# Safe shutdown
# ------------------------------------------------------------------

def safe_shutdown(psu: OWONPowerSupply, reason: str) -> None:
    print(f"\n[SHUTDOWN] {reason}")
    try:
        psu.set_voltage(0.0)
        psu.output_off()
        psu.set_local()  # return front-panel control to the user
    except Exception as exc:
        print(f"[SHUTDOWN] Warning — PSU command failed: {exc}")


# ------------------------------------------------------------------
# Main test loop
# ------------------------------------------------------------------

def run_step_test(args: argparse.Namespace) -> int:
    steps_v = sorted(args.steps)
    max_temp = args.max_temp
    hold_s = args.hold_time
    sample_s = args.sample_interval
    cooldown_s = args.cooldown_time
    current_limit = args.current_limit

    print("\n=== Heater Open-Loop Step Test ===")
    print(f"Voltage steps : {steps_v} V")
    print(f"Hold per step : {hold_s:.0f} s")
    print(f"Sample rate   : {sample_s:.1f} s")
    print(f"Current limit : {current_limit:.3f} A")
    print(f"Hard cutoff   : {max_temp:.1f} °C")
    print(f"Cooldown      : {cooldown_s:.0f} s")
    print()

    print("Discovering hardware...")
    try:
        owon_resource = resolve_owon(args.owon_port)
        dracal_port = resolve_dracal(args.dracal_port)
    except RuntimeError as exc:
        print(f"[ERROR] {exc}")
        return 1

    log_dir = Path(args.output_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = log_dir / f"step_test_{ts}.csv"

    psu = OWONPowerSupply(owon_resource)
    sensor = DracalTMC100k(
        port=dracal_port,
        poll_interval_ms=max(100, int(sample_s * 1000)),
    )
    logger = StepTestLogger(log_path)
    exit_code = 0

    try:
        print("\nConnecting to OWON power supply...")
        psu.connect()
        print(f"  {psu.identify()}")
        psu.set_remote()
        print("  Remote mode enabled.")

        print("Connecting to Dracal thermocouple...")
        sensor.connect()
        initial_temp = sensor.read_temperature()
        initial_cj = sensor.read_cold_junction()
        print(f"  T = {initial_temp:.2f} °C  |  CJ = {initial_cj:.2f} °C")

        if initial_temp >= max_temp:
            print(
                f"\n[ABORT] Starting temperature {initial_temp:.2f} °C is already "
                f"at or above the hard cutoff ({max_temp:.1f} °C). "
                "Allow the system to cool before running."
            )
            return 1

        # Safe initial state
        psu.set_current(current_limit)
        psu.set_voltage(0.0)
        psu.output_on()
        time.sleep(1.0)  # let PSU finish processing burst of writes before first query

        if not psu.get_output_state():
            raise RuntimeError(
                "PSU output did not turn on after OUTP ON command. "
                "Check front panel for OVP/OCP/OTP fault indicators."
            )

        print(f"\nOutput ON at 0 V.  Starting step sequence...\n")
        print(
            f"  {'Phase':<22} {'t+':>6}  "
            f"{'Set-V':>5}  {'V':>6}  {'I':>6}  {'P':>6}  "
            f"{'T':>7}  {'CJ':>7}"
        )
        print("  " + "-" * 76)

        total = len(steps_v)

        # ---- Voltage step loop ----
        for step_idx, setpoint_v in enumerate(steps_v):
            phase = f"step_{step_idx + 1}_of_{total}"
            print(f"\n  [Step {step_idx + 1}/{total}]  {setpoint_v:.1f} V  —  hold {hold_s:.0f} s")
            psu.set_voltage(setpoint_v)
            time.sleep(0.5)  # let PSU process voltage change before querying

            # On the first step, verify the setpoint was accepted — catches silent
            # command-ignore if remote mode wasn't enabled correctly.
            if step_idx == 0:
                actual_sp = psu.get_voltage_setpoint()
                if abs(actual_sp - setpoint_v) > 0.1:
                    raise RuntimeError(
                        f"Voltage setpoint not accepted: sent {setpoint_v:.3f} V, "
                        f"PSU reports {actual_sp:.3f} V. "
                        "Check remote mode and front-panel fault indicators."
                    )

            step_start = time.time()
            sample_count = 0

            while True:
                loop_t = time.time()
                elapsed_step = loop_t - step_start
                if elapsed_step >= hold_s:
                    break

                v, i, p = psu.measure_all()
                temp = sensor.read_temperature()
                cj = sensor.read_cold_junction()

                logger.write(phase, step_idx, setpoint_v, v, i, p, temp, cj)
                sample_count += 1

                print(
                    f"  {phase:<22} {elapsed_step:6.1f}s  "
                    f"{setpoint_v:5.1f}V  {v:6.3f}V  {i:6.4f}A  {p:6.3f}W  "
                    f"{temp:7.2f}°C  {cj:7.2f}°C"
                )

                # Hard cutoff
                if temp >= max_temp:
                    safe_shutdown(
                        psu,
                        f"Hard cutoff: T={temp:.2f}°C ≥ {max_temp:.1f}°C"
                    )
                    logger.write("FAULT_CUTOFF", step_idx, setpoint_v, v, i, p, temp, cj)
                    exit_code = 2
                    return exit_code

                sleep_time = sample_s - (time.time() - loop_t)
                if sleep_time > 0:
                    time.sleep(sleep_time)

            print(f"  Step {step_idx + 1} complete ({sample_count} samples).")

        # ---- Cooldown ----
        print(f"\n  [Cooldown]  0 V  —  logging for {cooldown_s:.0f} s")
        psu.set_voltage(0.0)
        time.sleep(0.5)
        cooldown_start = time.time()

        while True:
            loop_t = time.time()
            elapsed_cd = loop_t - cooldown_start
            if elapsed_cd >= cooldown_s:
                break

            v, i, p = psu.measure_all()
            temp = sensor.read_temperature()
            cj = sensor.read_cold_junction()

            logger.write("cooldown", total, 0.0, v, i, p, temp, cj)
            print(
                f"  {'cooldown':<22} {elapsed_cd:6.1f}s  "
                f"{'0.0':>5}V  {v:6.3f}V  {i:6.4f}A  {p:6.3f}W  "
                f"{temp:7.2f}°C  {cj:7.2f}°C"
            )

            sleep_time = sample_s - (time.time() - loop_t)
            if sleep_time > 0:
                time.sleep(sleep_time)

        print("\n[DONE] Step test complete.")

    except KeyboardInterrupt:
        print("\n[STOPPED] Interrupted by user.")
        exit_code = 0
    except Exception as exc:
        print(f"\n[FAULT] Unexpected error: {exc}")
        exit_code = 2
    finally:
        safe_shutdown(psu, "End of test — zeroing output and cutting power")
        try:
            sensor.disconnect()
        except Exception:
            pass
        try:
            psu.disconnect()
        except Exception:
            pass
        logger.close()
        print(f"Log written: {log_path}")

    return exit_code


# ------------------------------------------------------------------
# CLI
# ------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Open-loop voltage step test for thermal system characterization. "
            "Applies a sequence of voltages to the heater, holds each for a "
            "configurable time, and logs PSU + thermocouple data to CSV."
        )
    )
    parser.add_argument(
        "--steps",
        type=float,
        nargs="+",
        default=DEFAULT_STEPS_V,
        metavar="V",
        help=(
            f"Voltage levels to apply in ascending order (V). "
            f"Default: {DEFAULT_STEPS_V}"
        ),
    )
    parser.add_argument(
        "--hold-time",
        type=float,
        default=DEFAULT_HOLD_S,
        metavar="S",
        help=f"Seconds to hold at each voltage level. Default: {DEFAULT_HOLD_S}",
    )
    parser.add_argument(
        "--sample-interval",
        type=float,
        default=DEFAULT_SAMPLE_S,
        metavar="S",
        help=f"Seconds between measurement samples. Default: {DEFAULT_SAMPLE_S}",
    )
    parser.add_argument(
        "--max-temp",
        type=float,
        default=DEFAULT_MAX_TEMP_C,
        metavar="C",
        help=(
            f"Hard safety cutoff in °C — zeroes voltage and exits if exceeded. "
            f"Default: {DEFAULT_MAX_TEMP_C}"
        ),
    )
    parser.add_argument(
        "--current-limit",
        type=float,
        default=DEFAULT_CURRENT_LIMIT_A,
        metavar="A",
        help=f"PSU current limit in A. Default: {DEFAULT_CURRENT_LIMIT_A}",
    )
    parser.add_argument(
        "--cooldown-time",
        type=float,
        default=DEFAULT_COOLDOWN_S,
        metavar="S",
        help=f"Seconds to log at 0 V after the last step. Default: {DEFAULT_COOLDOWN_S}",
    )
    parser.add_argument(
        "--owon-port",
        metavar="PORT",
        help="OWON serial port (e.g., /dev/tty.usbserial-XXXX). Auto-detected if omitted.",
    )
    parser.add_argument(
        "--dracal-port",
        metavar="PORT",
        help="Dracal serial port (e.g., /dev/tty.usbmodemXXXX). Auto-detected if omitted.",
    )
    parser.add_argument(
        "--output-dir",
        default="logs",
        metavar="DIR",
        help="Directory to write the CSV log. Default: logs/",
    )
    return parser.parse_args()


if __name__ == "__main__":
    raise SystemExit(run_step_test(parse_args()))
