#!/usr/bin/env python3
"""
Post-session Temperature vs Time plot.

Reads a sensor_log.csv from a Growth Monitor session and generates a
temperature vs elapsed time plot. Useful for reviewing growth profiles
after a session completes.

Usage:
    python scripts/plot_temperature.py logs/growths/<session_dir>/sensor_log.csv
    python scripts/plot_temperature.py logs/growths/<session_dir>/  # auto-finds CSV

Options:
    --output PATH   Save plot to file instead of displaying (png, pdf, svg)
    --title TEXT     Custom plot title (default: derived from session directory)
    --dpi INT        Output resolution (default: 150)
"""

import argparse
import csv
import sys
from datetime import datetime
from pathlib import Path
from typing import Optional


def parse_sensor_log(csv_path: Path) -> tuple[list[float], list[float]]:
    """Parse sensor_log.csv and return (elapsed_seconds, temperatures).

    Skips rows where temperature is missing or unparseable.
    """
    elapsed = []
    temps = []

    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                t = float(row["elapsed_s"])
            except (KeyError, ValueError):
                continue

            temp_str = row.get("pyrometer_temp_C", "").strip()
            if not temp_str:
                continue
            try:
                temp = float(temp_str)
            except ValueError:
                continue

            elapsed.append(t)
            temps.append(temp)

    return elapsed, temps


def find_sensor_log(session_path: Path) -> Path:
    """Given a session directory or CSV file, return the sensor_log.csv path."""
    if session_path.is_file() and session_path.suffix == ".csv":
        return session_path
    if session_path.is_dir():
        candidate = session_path / "sensor_log.csv"
        if candidate.exists():
            return candidate
    raise FileNotFoundError(
        f"Could not find sensor_log.csv at {session_path}"
    )


def derive_title(csv_path: Path) -> str:
    """Derive a plot title from the session directory name."""
    session_dir = csv_path.parent
    name = session_dir.name
    # Session dirs are named: prefix_sampleid_YYYYMMDD_HHMMSS
    parts = name.rsplit("_", 2)
    if len(parts) >= 3:
        prefix_sample = parts[0]
        date_str = parts[1]
        time_str = parts[2]
        try:
            dt = datetime.strptime(f"{date_str}_{time_str}", "%Y%m%d_%H%M%S")
            return f"{prefix_sample} — {dt.strftime('%Y-%m-%d %H:%M')}"
        except ValueError:
            pass
    return f"Growth Session: {name}"


def plot_temperature(
    elapsed: list[float],
    temps: list[float],
    title: str = "Temperature vs Time",
    output_path: Optional[Path] = None,
    dpi: int = 150,
) -> None:
    """Generate and show/save the T vs t plot."""
    import matplotlib.pyplot as plt

    # Convert elapsed seconds to minutes for readability
    elapsed_min = [t / 60.0 for t in elapsed]

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(elapsed_min, temps, color="#0d9488", linewidth=1.2)
    ax.set_xlabel("Elapsed Time (min)", fontsize=12)
    ax.set_ylabel("Temperature (\u00b0C)", fontsize=12)
    ax.set_title(title, fontsize=14)
    ax.grid(True, alpha=0.3)

    # Annotate min/max
    if temps:
        t_min, t_max = min(temps), max(temps)
        ax.axhline(y=t_max, color="#dc2626", linestyle="--", alpha=0.5, linewidth=0.8)
        ax.axhline(y=t_min, color="#2563eb", linestyle="--", alpha=0.5, linewidth=0.8)
        ax.text(
            elapsed_min[-1], t_max, f"  {t_max:.0f}\u00b0C",
            va="bottom", color="#dc2626", fontsize=9,
        )
        ax.text(
            elapsed_min[-1], t_min, f"  {t_min:.0f}\u00b0C",
            va="top", color="#2563eb", fontsize=9,
        )

    fig.tight_layout()

    if output_path:
        fig.savefig(str(output_path), dpi=dpi)
        print(f"Plot saved to {output_path}")
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(
        description="Plot temperature vs time from a Growth Monitor session.",
    )
    parser.add_argument(
        "session_path",
        type=Path,
        help="Path to sensor_log.csv or session directory",
    )
    parser.add_argument(
        "--output", "-o",
        type=Path,
        default=None,
        help="Save plot to file (png, pdf, svg)",
    )
    parser.add_argument("--title", "-t", type=str, default=None)
    parser.add_argument("--dpi", type=int, default=150)
    args = parser.parse_args()

    try:
        csv_path = find_sensor_log(args.session_path)
    except FileNotFoundError as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

    elapsed, temps = parse_sensor_log(csv_path)

    if not temps:
        print("No temperature data found in sensor log.", file=sys.stderr)
        sys.exit(1)

    title = args.title or derive_title(csv_path)
    print(f"Loaded {len(temps)} temperature readings from {csv_path.name}")
    print(f"Duration: {elapsed[-1] / 60:.1f} min | "
          f"T range: {min(temps):.0f} - {max(temps):.0f}\u00b0C")

    plot_temperature(elapsed, temps, title=title,
                     output_path=args.output, dpi=args.dpi)


if __name__ == "__main__":
    main()
