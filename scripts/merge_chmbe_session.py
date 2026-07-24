#!/usr/bin/env python3
"""Merge Ch-MBE sensor_log + ads_shadow CSVs, produce a unified CSV and growth profile plot.

sensor_log timestamps are naive local (CDT); ads_shadow timestamps are UTC with tz offset.
Both are normalized to naive CDT before merging.

Usage (Wednesday Jul 22 session):
    python scripts/merge_chmbe_session.py \\
        --sensor "/Users/aj/Downloads/sensor_log (7).csv" \\
        --ads    /Users/aj/Downloads/ads_shadow_20260722_162145.csv \\
        --out    /tmp/session_jul22
"""
from __future__ import annotations
import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker


# CDT = UTC-5; ads_shadow is UTC, sensor_log is naive local (CDT)
_CDT_OFFSET = pd.Timedelta(hours=5)


def load_sensor_log(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path, parse_dates=["timestamp"])
    return df


def load_ads_shadow(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    # Parse as UTC-aware, strip tz, subtract 5h → naive CDT
    df["timestamp"] = (
        pd.to_datetime(df["timestamp"], utc=True).dt.tz_convert(None) - _CDT_OFFSET
    )
    return df


_CELL_COLORS = [
    "#4ade80",  # Cell1 — substrate (green)
    "#38bdf8",  # Cell2 — highlighted (blue)
    "#f87171",  # Cell3
    "#fbbf24",  # Cell4
    "#a78bfa",  # Cell5
    "#fb923c",  # Cell6
    "#e879f9",  # Cell7
]


def _dark_axes(ax: plt.Axes) -> None:
    ax.set_facecolor("#0f1117")
    ax.tick_params(colors="#94a3b8", labelsize=8)
    for spine in ["top", "right"]:
        ax.spines[spine].set_visible(False)
    for spine in ["left", "bottom"]:
        ax.spines[spine].set_color("#334155")


def plot_session(df_sensor: pd.DataFrame, df_ads: pd.DataFrame, out_path: Path) -> None:
    t0 = min(df_sensor["timestamp"].iloc[0], df_ads["timestamp"].iloc[0])
    el_s = (df_sensor["timestamp"] - t0).dt.total_seconds() / 60  # elapsed minutes
    el_a = (df_ads["timestamp"] - t0).dt.total_seconds() / 60

    fig, axes = plt.subplots(
        3, 1, figsize=(13, 9), sharex=True,
        gridspec_kw={"height_ratios": [2.2, 2, 1], "hspace": 0.08},
    )
    fig.patch.set_facecolor("#0f1117")
    for ax in axes:
        _dark_axes(ax)

    # ── Panel 1: Pyrometer ──────────────────────────────────────────────────
    ax0 = axes[0]
    ax0.plot(el_s, df_sensor["pyrometer_temp_C"],
             color="#f97316", linewidth=1.6, label="Pyrometer")
    ax0.axhline(900, color="#334155", linewidth=0.8, linestyle="--")
    ax0.text(el_s.iloc[-1] * 0.99, 912, "900 °C", color="#475569",
             ha="right", fontsize=8)

    peak_idx = df_sensor["pyrometer_temp_C"].idxmax()
    peak_t = el_s.iloc[peak_idx]
    peak_v = df_sensor["pyrometer_temp_C"].iloc[peak_idx]
    ax0.annotate(
        f"Peak {peak_v:.0f} °C",
        xy=(peak_t, peak_v), xytext=(peak_t + 1.5, peak_v - 100),
        color="#fbbf24", fontsize=8.5,
        arrowprops=dict(arrowstyle="->", color="#fbbf24", lw=0.8),
    )

    ax0.set_ylabel("Pyrometer (°C)", color="#94a3b8", fontsize=9)
    ax0.set_ylim(0, 1150)
    ax0.yaxis.set_major_locator(ticker.MultipleLocator(200))
    ax0.legend(loc="upper right", frameon=False, labelcolor="#94a3b8", fontsize=8)
    ax0.set_title(
        "Ch-MBE Growth Session  ·  Jul 22, 2026  ·  "
        "Substrate anneal/flash, no deposition  (all shutters closed)",
        color="#e2e8f0", fontsize=10, pad=10,
    )

    # ── Panel 2: Cell temperatures (ADS) ───────────────────────────────────
    ax1 = axes[1]
    for i in range(1, 8):
        col = f"Cell{i}_T"
        if col not in df_ads.columns:
            continue
        label = f"Cell {i}" + (" (substrate)" if i == 1 else "")
        lw = 2.0 if i == 2 else 1.1
        alpha = 1.0 if i in (1, 2) else 0.65
        ax1.plot(el_a, df_ads[col], color=_CELL_COLORS[i - 1],
                 linewidth=lw, alpha=alpha, label=label)

    ax1.set_ylabel("Cell Temp (°C)", color="#94a3b8", fontsize=9)
    ax1.legend(
        loc="upper right", frameon=False, labelcolor="#94a3b8",
        fontsize=7.5, ncol=2, handlelength=1.5,
    )

    # ── Panel 3: Chamber pressure (sensor_log) ─────────────────────────────
    ax2 = axes[2]
    pressure = df_sensor["chamber_pressure_mbar"].replace(0, np.nan)
    ax2.semilogy(el_s, pressure, color="#38bdf8", linewidth=1.2)
    ax2.set_ylabel("Pressure\n(mbar)", color="#94a3b8", fontsize=9)
    ax2.set_xlabel("Elapsed time (min)", color="#94a3b8", fontsize=9)
    ax2.yaxis.set_major_formatter(ticker.LogFormatterMathtext())

    x_max = max(el_s.iloc[-1], el_a.iloc[-1])
    axes[0].set_xlim(0, x_max * 1.02)

    fig.savefig(out_path, dpi=150, facecolor=fig.get_facecolor(), bbox_inches="tight")
    print(f"  Plot → {out_path}")
    plt.close(fig)


_ADS_CELL_T_COLS = [f"Cell{i}_T" for i in range(1, 8)]
_ADS_SHUTTER_COLS = [f"Cell{i}_ShutOpen" for i in range(1, 8)]


def merge_sessions(df_sensor: pd.DataFrame, df_ads: pd.DataFrame) -> pd.DataFrame:
    """Left-join sensor_log rows to nearest ads_shadow row within 10s."""
    ads_keep = (
        ["timestamp"] + _ADS_CELL_T_COLS + _ADS_SHUTTER_COLS + ["IonGauge1_P"]
    )
    merged = pd.merge_asof(
        df_sensor.sort_values("timestamp"),
        df_ads[ads_keep].sort_values("timestamp"),
        on="timestamp",
        direction="nearest",
        tolerance=pd.Timedelta(seconds=10),
    )

    rename = {f"Cell{i}_T": f"cell{i}_T_C" for i in range(1, 8)}
    rename.update({f"Cell{i}_ShutOpen": f"cell{i}_shutter_open" for i in range(1, 8)})
    rename["IonGauge1_P"] = "ads_ion_gauge_mbar"
    return merged.rename(columns=rename)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sensor", required=True, help="Path to sensor_log CSV")
    parser.add_argument("--ads", required=True, help="Path to ads_shadow CSV")
    parser.add_argument("--out", default="/tmp/session_merged",
                        help="Output prefix (extensions added automatically)")
    args = parser.parse_args()

    sensor_path = Path(args.sensor)
    ads_path = Path(args.ads)
    out_prefix = Path(args.out)
    out_prefix.parent.mkdir(parents=True, exist_ok=True)

    print(f"Loading sensor_log:  {sensor_path.name}")
    df_s = load_sensor_log(sensor_path)
    print(f"  {len(df_s)} rows  |  "
          f"{df_s['timestamp'].iloc[0]} → {df_s['timestamp'].iloc[-1]}")

    print(f"Loading ads_shadow:  {ads_path.name}")
    df_a = load_ads_shadow(ads_path)
    print(f"  {len(df_a)} rows  |  "
          f"{df_a['timestamp'].iloc[0]} → {df_a['timestamp'].iloc[-1]}")

    print("Generating plot...")
    plot_path = out_prefix.with_suffix(".png")
    plot_session(df_s, df_a, plot_path)

    print("Merging to unified CSV...")
    df_merged = merge_sessions(df_s, df_a)
    csv_path = out_prefix.with_suffix(".csv")
    df_merged.to_csv(csv_path, index=False)
    n_matched = df_merged["cell1_T_C"].notna().sum()
    print(f"  {len(df_merged)} rows × {len(df_merged.columns)} columns")
    print(f"  ADS match rate: {n_matched}/{len(df_merged)} sensor_log rows matched")
    print(f"  CSV  → {csv_path}")


if __name__ == "__main__":
    main()
