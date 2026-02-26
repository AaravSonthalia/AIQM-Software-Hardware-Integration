"""
Analyze open-loop step test CSV → estimate K, τ, θ per band → compute
Ziegler–Nichols initial PID gains for use with temperature_pid_control.py.

The step test applies a sequence of voltages and logs the temperature
response.  This script reads that log and extracts three parameters per
temperature band:

  K_V  [°C/V]  — process gain: steady-state temperature change per volt
                  of controller output change.  Used directly in ZN formulas.
  K_th [°C/W]  — thermal resistance: temperature change per watt.
                  Printed for physical insight; not used in gain formulas.
  τ    [s]     — thermal time constant: time to reach 63.2 % of final ΔT.
  θ    [s]     — dead time: delay before temperature starts responding.

These feed Ziegler–Nichols open-loop tuning:
  Kp = 1.2 × τ / (K_V × θ)
  Ki = Kp / (2 × θ)
  Kd = Kp × 0.5 × θ   ← usually left at 0 (derivative amplifies TC noise)

Usage:
    python analyze_step_test.py logs/step_test_20260225_123456.csv
    python analyze_step_test.py logs/step_test_20260225_123456.csv --plot
    python analyze_step_test.py logs/step_test_20260225_123456.csv \\
        --band1-limit 60 --band2-limit 110 --target 50
"""

from __future__ import annotations

import argparse
import csv
import statistics
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


# ---------------------------------------------------------------------------
# Band thresholds (overridable via CLI)
# ---------------------------------------------------------------------------

DEFAULT_BAND1_LIMIT = 60.0    # °C   Band 1: T_ss < this
DEFAULT_BAND2_LIMIT = 110.0   # °C   Band 2: this ≤ T_ss < next  /  Band 3: T_ss ≥ next

# ---------------------------------------------------------------------------
# Analysis tuning constants
# ---------------------------------------------------------------------------

SS_FRACTION = 0.30            # fraction of step data used for steady-state estimate
RISE_THRESHOLD_C = 0.30       # °C rise that signals end of dead time
MIN_DEAD_TIME_S = 0.5         # credibility floor for θ (sensor latency)
MIN_SAMPLES_FOR_SS = 5        # minimum rows needed to estimate steady state
MIN_SAMPLES_FOR_TAU = 12      # minimum rows needed to attempt τ estimation
MIN_DELTA_V = 0.10            # V  — skip K_V if ΔV is this small
MIN_DELTA_T = 0.20            # °C — skip K estimates if ΔT is this small


# ---------------------------------------------------------------------------
# Data model
# ---------------------------------------------------------------------------

@dataclass
class Row:
    elapsed_s: float
    phase: str
    step_index: int
    setpoint_v: float
    voltage_v: float
    power_w: float
    temperature_c: float


@dataclass
class StepAnalysis:
    step_idx: int           # 0-based index
    setpoint_v: float       # commanded voltage for this step

    t_start_c: float        # T at step start (mean of first ~3 samples)
    t_ss_c: float           # estimated T_ss (mean of last SS_FRACTION)

    v_ss_v: float           # mean measured voltage (last SS_FRACTION)
    p_ss_w: float           # mean measured power  (last SS_FRACTION)

    prev_t_ss_c: float      # T_ss of previous step (or ambient if step 0)
    prev_v_ss_v: float      # V_ss of previous step (or 0)
    prev_p_ss_w: float      # P_ss of previous step (or 0)

    theta_s: float          # dead-time estimate
    tau_s: Optional[float]  # time-constant estimate (None if not reached)

    n_samples: int
    band: int               # 1 / 2 / 3
    note: str = ""

    # ---- derived properties ------------------------------------------------

    @property
    def delta_t_c(self) -> float:
        return self.t_ss_c - self.prev_t_ss_c

    @property
    def delta_v_v(self) -> float:
        return self.v_ss_v - self.prev_v_ss_v

    @property
    def delta_p_w(self) -> float:
        return self.p_ss_w - self.prev_p_ss_w

    @property
    def k_v(self) -> Optional[float]:
        """Process gain °C/V  (used in ZN formulas)."""
        if self.delta_v_v < MIN_DELTA_V or self.delta_t_c < MIN_DELTA_T:
            return None
        return self.delta_t_c / self.delta_v_v

    @property
    def k_thermal(self) -> Optional[float]:
        """Thermal resistance °C/W  (physical insight, not used in ZN)."""
        if self.delta_p_w < 0.05 or self.delta_t_c < MIN_DELTA_T:
            return None
        return self.delta_t_c / self.delta_p_w


# ---------------------------------------------------------------------------
# CSV loading
# ---------------------------------------------------------------------------

def load_csv(path: Path) -> list[Row]:
    rows: list[Row] = []
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        for raw in reader:
            phase = raw.get("phase", "").strip()
            try:
                rows.append(Row(
                    elapsed_s=float(raw["elapsed_s"]),
                    phase=phase,
                    step_index=int(raw["step_index"]),
                    setpoint_v=float(raw["setpoint_v"]),
                    voltage_v=float(raw["voltage_v"]),
                    power_w=float(raw["power_w"]),
                    temperature_c=float(raw["temperature_c"]),
                ))
            except (KeyError, ValueError):
                continue
    return rows


def group_step_rows(rows: list[Row]) -> dict[int, list[Row]]:
    groups: dict[int, list[Row]] = {}
    for row in rows:
        if row.phase.startswith("step_"):
            groups.setdefault(row.step_index, []).append(row)
    for key in groups:
        groups[key].sort(key=lambda r: r.elapsed_s)
    return groups


# ---------------------------------------------------------------------------
# Parameter estimation
# ---------------------------------------------------------------------------

def detect_dead_time(step_rows: list[Row], t_start: float) -> float:
    """Return θ in seconds (relative to start of step)."""
    if len(step_rows) < 3:
        return MIN_DEAD_TIME_S

    t0 = step_rows[0].elapsed_s
    for i, row in enumerate(step_rows):
        if row.temperature_c >= t_start + RISE_THRESHOLD_C:
            # Require at least one subsequent sample also above threshold
            for j in range(i + 1, min(i + 3, len(step_rows))):
                if step_rows[j].temperature_c >= t_start + RISE_THRESHOLD_C * 0.7:
                    return max(MIN_DEAD_TIME_S, row.elapsed_s - t0)
    return MIN_DEAD_TIME_S


def estimate_tau(
    step_rows: list[Row],
    t_start: float,
    t_ss: float,
    theta_s: float,
) -> Optional[float]:
    """Return τ in seconds.  None if 63.2 % ΔT not reached within the step."""
    if len(step_rows) < MIN_SAMPLES_FOR_TAU:
        return None

    delta_t = t_ss - t_start
    if delta_t < 0.5:
        return None  # step too small to characterise

    target_t = t_start + 0.632 * delta_t
    t0 = step_rows[0].elapsed_s

    for row in step_rows:
        if row.temperature_c >= target_t:
            tau = (row.elapsed_s - t0) - theta_s
            return max(1.0, tau)

    return None  # 63.2 % point not reached


# ---------------------------------------------------------------------------
# Per-step analysis
# ---------------------------------------------------------------------------

def analyze_steps(
    groups: dict[int, list[Row]],
    band1_limit: float,
    band2_limit: float,
    ambient_c: float,
) -> list[StepAnalysis]:
    results: list[StepAnalysis] = []

    prev_t_ss = ambient_c
    prev_v_ss = 0.0
    prev_p_ss = 0.0
    prev_setpoint_v = 0.0

    for step_idx in sorted(groups.keys()):
        rows = groups[step_idx]
        n = len(rows)
        if n < MIN_SAMPLES_FOR_SS:
            continue

        ss_start = max(0, int(n * (1.0 - SS_FRACTION)))
        ss_rows = rows[ss_start:]

        setpoint_v = rows[0].setpoint_v
        t_start = statistics.mean(r.temperature_c for r in rows[:min(3, n)])
        t_ss = statistics.mean(r.temperature_c for r in ss_rows)
        v_ss = statistics.mean(r.voltage_v for r in ss_rows)
        p_ss = statistics.mean(r.power_w for r in ss_rows)

        theta_s = detect_dead_time(rows, t_start)
        tau_s = estimate_tau(rows, t_start, t_ss, theta_s)

        # Band is determined by the steady-state temperature reached
        if t_ss < band1_limit:
            band = 1
        elif t_ss < band2_limit:
            band = 2
        else:
            band = 3

        notes = []
        if tau_s is None:
            notes.append("τ not reached — hold time too short; try --hold-time 180+")

        results.append(StepAnalysis(
            step_idx=step_idx,
            setpoint_v=setpoint_v,
            t_start_c=t_start,
            t_ss_c=t_ss,
            v_ss_v=v_ss,
            p_ss_w=p_ss,
            prev_t_ss_c=prev_t_ss,
            prev_v_ss_v=prev_v_ss,
            prev_p_ss_w=prev_p_ss,
            theta_s=theta_s,
            tau_s=tau_s,
            n_samples=n,
            band=band,
            note="  ".join(notes),
        ))

        prev_t_ss = t_ss
        prev_v_ss = v_ss
        prev_p_ss = p_ss
        prev_setpoint_v = setpoint_v

    return results


# ---------------------------------------------------------------------------
# Band-level summary and ZN gain computation
# ---------------------------------------------------------------------------

def band_label(band: int, band1_limit: float, band2_limit: float) -> str:
    if band == 1:
        return f"Band 1  (T < {band1_limit:.0f}°C)"
    elif band == 2:
        return f"Band 2  ({band1_limit:.0f}–{band2_limit:.0f}°C)"
    else:
        return f"Band 3  (T ≥ {band2_limit:.0f}°C)"


def compute_band_summary(
    steps: list[StepAnalysis],
    band: int,
    band1_limit: float,
    band2_limit: float,
) -> Optional[dict]:
    band_steps = [s for s in steps if s.band == band]
    if not band_steps:
        return None

    k_v_vals = [s.k_v for s in band_steps if s.k_v is not None]
    k_th_vals = [s.k_thermal for s in band_steps if s.k_thermal is not None]
    tau_vals = [s.tau_s for s in band_steps if s.tau_s is not None]
    theta_vals = [s.theta_s for s in band_steps]

    def _mean_std(vals: list[float]) -> tuple[Optional[float], float]:
        if not vals:
            return None, 0.0
        m = statistics.mean(vals)
        s = statistics.stdev(vals) if len(vals) >= 2 else 0.0
        return m, s

    k_v_mean, k_v_std = _mean_std(k_v_vals)
    k_th_mean, _ = _mean_std(k_th_vals)
    tau_mean, tau_std = _mean_std(tau_vals)
    theta_mean, theta_std = _mean_std(theta_vals)

    kp = ki = kd = None
    if k_v_mean and tau_mean and theta_mean and theta_mean > 0:
        kp = 1.2 * tau_mean / (k_v_mean * theta_mean)
        ki = kp / (2.0 * theta_mean)
        kd = kp * 0.5 * theta_mean

    return {
        "band": band,
        "label": band_label(band, band1_limit, band2_limit),
        "n_steps": len(band_steps),
        "n_k_v": len(k_v_vals),
        "n_tau": len(tau_vals),
        "k_v_mean": k_v_mean,
        "k_v_std": k_v_std,
        "k_th_mean": k_th_mean,
        "tau_mean": tau_mean,
        "tau_std": tau_std,
        "theta_mean": theta_mean,
        "theta_std": theta_std,
        "kp": kp,
        "ki": ki,
        "kd": kd,
    }


# ---------------------------------------------------------------------------
# Report printing
# ---------------------------------------------------------------------------

def _fmt_optional(val: Optional[float], fmt: str, unit: str = "") -> str:
    if val is None:
        return "n/a"
    return f"{val:{fmt}}{unit}"


def print_step_table(steps: list[StepAnalysis]) -> None:
    print("\n=== Per-Step Results ===\n")
    col_w = 76
    print(
        f"  {'#':>3}  {'Set-V':>5}  "
        f"{'T_start':>8}  {'T_ss':>8}  {'ΔT':>7}  "
        f"{'K_V':>7}  {'K_th':>7}  {'θ':>5}  {'τ':>7}  Band"
    )
    print("  " + "-" * col_w)

    for s in steps:
        k_v_str = _fmt_optional(s.k_v, ".2f", " °C/V")
        k_th_str = _fmt_optional(s.k_thermal, ".2f", " °C/W")
        tau_str = _fmt_optional(s.tau_s, ".0f", "s")
        print(
            f"  {s.step_idx + 1:3d}  {s.setpoint_v:5.1f}V  "
            f"{s.t_start_c:7.2f}°C  {s.t_ss_c:7.2f}°C  {s.delta_t_c:+7.2f}°C  "
            f"{k_v_str:>10}  {k_th_str:>10}  {s.theta_s:4.1f}s  {tau_str:>7}  "
            f"{s.band}"
        )
        if s.note:
            print(f"      Note: {s.note}")

    print()


def print_band_summaries(
    summaries: list[Optional[dict]],
    band1_limit: float,
    band2_limit: float,
) -> None:
    print("\n=== Per-Band Summary ===\n")

    for b in summaries:
        if b is None:
            continue

        print(f"  {b['label']}  —  {b['n_steps']} step(s), "
              f"{b['n_k_v']} valid K_V, {b['n_tau']} valid τ")

        if b["k_v_mean"] is not None:
            std_str = f"  ±{b['k_v_std']:.2f}" if b["k_v_std"] > 0 else ""
            print(f"    K_V     = {b['k_v_mean']:.3f} °C/V{std_str}  "
                  f"(process gain for ZN tuning)")
        else:
            print(f"    K_V     = n/a  (ΔV or ΔT too small for this step)")

        if b["k_th_mean"] is not None:
            print(f"    K_th    = {b['k_th_mean']:.3f} °C/W  (thermal resistance, reference)")

        if b["tau_mean"] is not None:
            std_str = f"  ±{b['tau_std']:.1f}s" if b["tau_std"] > 0 else ""
            print(f"    τ       = {b['tau_mean']:.1f}s{std_str}  (thermal time constant)")
        else:
            print(f"    τ       = n/a  — temperature did not reach 63.2% of ΔT")
            print(f"              → Re-run step test with --hold-time 180 or longer")

        std_str = f"  ±{b['theta_std']:.1f}s" if b["theta_std"] > 0 else ""
        print(f"    θ       = {b['theta_mean']:.1f}s{std_str}  (dead time)")

        if b["kp"] is not None:
            t = b["tau_mean"]
            k = b["k_v_mean"]
            th = b["theta_mean"]
            print(f"\n    Ziegler–Nichols open-loop gains (starting point):")
            print(f"    Kp = 1.2 × τ / (K_V × θ)  =  "
                  f"1.2 × {t:.1f} / ({k:.3f} × {th:.1f})  =  {b['kp']:.4f}")
            print(f"    Ki = Kp / (2θ)             =  "
                  f"{b['kp']:.4f} / {2*th:.1f}            =  {b['ki']:.4f}")
            print(f"    Kd = Kp × 0.5θ             =  "
                  f"{b['kp']:.4f} × {0.5*th:.1f}            =  {b['kd']:.4f}")
            print(f"         ↳ Set Kd = 0 initially; only add derivative if stable "
                  f"but too sluggish.")
        else:
            print(f"\n    ZN gains: cannot compute — K_V or τ missing")

        print()


def print_suggested_command(
    summaries: list[Optional[dict]],
    band1_limit: float,
    band2_limit: float,
    target_temp: float,
) -> None:
    # Collect computed gains; fall back to neighbours or safe defaults
    _fallback = (1.0, 0.02, 0.0)
    gains: dict[int, tuple[float, float, float]] = {}

    for b in summaries:
        if b is not None and b["kp"] is not None:
            gains[b["band"]] = (b["kp"], b["ki"], 0.0)  # Kd always 0 in suggestion

    # Propagate to missing bands
    for band in (1, 2, 3):
        if band not in gains:
            if band == 1 and 2 in gains:
                gains[1] = gains[2]
            elif band == 3 and 2 in gains:
                gains[3] = gains[2]
            elif band == 2:
                if 1 in gains:
                    gains[2] = gains[1]
                elif 3 in gains:
                    gains[2] = gains[3]
                else:
                    gains[2] = _fallback
            else:
                gains[band] = _fallback

    p1, i1, d1 = gains.get(1, _fallback)
    p2, i2, d2 = gains.get(2, _fallback)
    p3, i3, d3 = gains.get(3, _fallback)

    print("=" * 72)
    print("=== Suggested temperature_pid_control.py command ===")
    print("=" * 72)
    print()
    print("Adjust --target for each test point (50 / 100 / 130 °C).")
    print("Start with these gains; refine based on CSV response analysis.")
    print()
    print(
        f"python temperature_pid_control.py \\\n"
        f"  --target {target_temp:.0f} \\\n"
        f"  --hold-minutes 5 \\\n"
        f"  --sample-time 1.0 \\\n"
        f"  --margin 1.0 \\\n"
        f"  --max-temp-hard 145 \\\n"
        f"  --max-voltage 24.0 \\\n"
        f"  --current-limit 1.0 \\\n"
        f"  --enable-gain-scheduling \\\n"
        f"  --pid-threshold-1 {band1_limit:.0f} \\\n"
        f"  --pid-threshold-2 {band2_limit:.0f} \\\n"
        f"  --pid-ramp-band 10 \\\n"
        f"  --p-low {p1:.4f} --i-low {i1:.4f} --d-low 0.0 \\\n"
        f"  --p-mid {p2:.4f} --i-mid {i2:.4f} --d-mid 0.0 \\\n"
        f"  --p-high {p3:.4f} --i-high {i3:.4f} --d-high 0.0"
    )
    print()
    print("Tuning tips:")
    print("  - Oscillating?   → halve Kp (and Ki proportionally)")
    print("  - Too sluggish?  → increase Kp by 50%; or reduce Ki")
    print("  - Overshoot?     → reduce Ki (or add small Kd)")
    print("  - Once stable at 50°C, test 100°C then 130°C, adjusting per band.")


# ---------------------------------------------------------------------------
# Optional plot
# ---------------------------------------------------------------------------

def plot_response(rows: list[Row], steps: list[StepAnalysis], csv_path: Path) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("\n[Warning] matplotlib not installed — skipping plot. "
              "Install with: pip install matplotlib")
        return

    # Filter to step + cooldown rows
    plot_rows = [r for r in rows if r.phase.startswith("step_") or r.phase == "cooldown"]
    if not plot_rows:
        print("[Warning] No plottable rows found.")
        return

    elapsed = [r.elapsed_s for r in plot_rows]
    temps = [r.temperature_c for r in plot_rows]
    setvolt = [r.setpoint_v for r in plot_rows]
    power = [r.power_w for r in plot_rows]

    band_colors = {1: "#4e9af1", 2: "#f1a94e", 3: "#e45c3a"}

    fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
    ax_t, ax_v, ax_p = axes

    # Temperature with T_ss markers coloured by band
    ax_t.plot(elapsed, temps, color="#aaaaaa", linewidth=0.8, label="T measured")
    for s in steps:
        g = [r for r in plot_rows if r.step_index == s.step_idx and r.phase.startswith("step_")]
        if not g:
            continue
        t0, t1 = g[0].elapsed_s, g[-1].elapsed_s
        col = band_colors[s.band]
        ax_t.hlines(s.t_ss_c, t0, t1, colors=col, linewidths=1.5,
                    label=f"T_ss step {s.step_idx + 1}" if s.step_idx == 0 else "_")
        ax_t.axvline(t0, color="gray", linewidth=0.4, linestyle="--")

    ax_t.set_ylabel("Temperature (°C)")
    ax_t.set_title(f"Step Test Analysis — {csv_path.name}")
    ax_t.grid(True, alpha=0.25)

    # Patch legend for bands
    import matplotlib.patches as mpatches
    patches = [
        mpatches.Patch(color=band_colors[1], label=f"Band 1 (T < {DEFAULT_BAND1_LIMIT:.0f}°C)"),
        mpatches.Patch(color=band_colors[2], label=f"Band 2 ({DEFAULT_BAND1_LIMIT:.0f}–{DEFAULT_BAND2_LIMIT:.0f}°C)"),
        mpatches.Patch(color=band_colors[3], label=f"Band 3 (T ≥ {DEFAULT_BAND2_LIMIT:.0f}°C)"),
    ]
    ax_t.legend(handles=patches, fontsize=8, loc="upper left")

    # Voltage setpoint
    ax_v.step(elapsed, setvolt, color="#2196F3", linewidth=1.0, where="post")
    ax_v.set_ylabel("Set voltage (V)")
    ax_v.grid(True, alpha=0.25)

    # Power
    ax_p.plot(elapsed, power, color="#FF5722", linewidth=0.8)
    ax_p.set_ylabel("Power (W)")
    ax_p.set_xlabel("Elapsed time (s)")
    ax_p.grid(True, alpha=0.25)

    plt.tight_layout()
    out_path = csv_path.parent / (csv_path.stem + "_analysis.png")
    plt.savefig(out_path, dpi=150)
    print(f"Plot saved: {out_path}")
    plt.show()


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Analyze a heater_step_test.py CSV log. "
            "Estimates K, τ, θ per temperature band and prints "
            "Ziegler–Nichols initial PID gain estimates."
        )
    )
    parser.add_argument(
        "csv_file",
        type=Path,
        help="Path to the step_test_*.csv log file.",
    )
    parser.add_argument(
        "--band1-limit",
        type=float,
        default=DEFAULT_BAND1_LIMIT,
        metavar="C",
        help=f"Upper °C boundary of Band 1 (default: {DEFAULT_BAND1_LIMIT})",
    )
    parser.add_argument(
        "--band2-limit",
        type=float,
        default=DEFAULT_BAND2_LIMIT,
        metavar="C",
        help=f"Upper °C boundary of Band 2 / lower of Band 3 (default: {DEFAULT_BAND2_LIMIT})",
    )
    parser.add_argument(
        "--target",
        type=float,
        default=50.0,
        metavar="C",
        help=(
            "Target temperature for the suggested Phase 2 command (default: 50). "
            "Change to 50, 100, or 130 for each test point."
        ),
    )
    parser.add_argument(
        "--plot",
        action="store_true",
        help="Generate and show a matplotlib step-response plot (requires matplotlib).",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    csv_path: Path = args.csv_file

    if not csv_path.exists():
        print(f"[Error] File not found: {csv_path}")
        return 1

    print(f"\n=== Step Test Analysis: {csv_path.name} ===\n")

    rows = load_csv(csv_path)
    if not rows:
        print("[Error] No rows loaded — check CSV format.")
        return 1

    print(f"Loaded {len(rows)} rows.")

    groups = group_step_rows(rows)
    if not groups:
        print("[Error] No step-phase rows found — was the file written by heater_step_test.py?")
        return 1

    print(f"Found {len(groups)} voltage step(s).")

    # Ambient temperature: mean of first 3 rows of step 0 (or first step)
    first_step_idx = min(groups.keys())
    first_rows = groups[first_step_idx]
    ambient_c = statistics.mean(r.temperature_c for r in first_rows[:min(3, len(first_rows))])
    print(f"Ambient temperature estimate: {ambient_c:.2f} °C")

    band1_limit = args.band1_limit
    band2_limit = args.band2_limit

    steps = analyze_steps(groups, band1_limit, band2_limit, ambient_c)

    if not steps:
        print("[Error] No steps had enough samples for analysis.")
        return 1

    print_step_table(steps)

    summaries = [
        compute_band_summary(steps, band, band1_limit, band2_limit)
        for band in (1, 2, 3)
    ]

    print_band_summaries(summaries, band1_limit, band2_limit)
    print_suggested_command(summaries, band1_limit, band2_limit, args.target)

    if args.plot:
        plot_response(rows, steps, csv_path)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
