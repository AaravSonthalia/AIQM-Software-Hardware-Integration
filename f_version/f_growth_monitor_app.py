"""Launch the growth monitor GUI or standalone CSV plot saver.

This file is intentionally self-contained so it can be added to the
AIQM-Software-Hardware-Integration repo without changing the existing package
files there.

Usage:
    python f_growth_monitor_app.py
    python f_growth_monitor_app.py --plotter
"""
from __future__ import annotations

import argparse
import csv
import os
import sys
import tempfile
from datetime import datetime
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "matplotlib"))

WORKSPACE_DIR = Path(__file__).resolve().parents[1]
DEFAULT_PLOT_DIR = WORKSPACE_DIR / "plots"
if str(WORKSPACE_DIR) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_DIR))

TIME_FORMATS = [
    "%H:%M",
    "%H:%M:%S",
    "%Y-%m-%d %H:%M:%S",
    "%m/%d/%Y %H:%M",
]

try:
    import matplotlib.dates as mdates
    import matplotlib.pyplot as plt

    HAS_MATPLOTLIB = True
except Exception:
    HAS_MATPLOTLIB = False


def parse_time(value: str):
    text = value.strip()
    for fmt in TIME_FORMATS:
        try:
            return datetime.strptime(text, fmt)
        except ValueError:
            continue
    return text


def looks_like(name: str, needle: str) -> bool:
    return needle.lower() in name.lower()


def format_plot_title(csv_path: str) -> str:
    stem = Path(csv_path).stem.replace("_", " ").strip()
    if stem:
        return f"{stem} Temperature Over Time"
    return "Temperature Over Time"


def resolve_workspace_folder(folder: str | Path) -> Path:
    """Resolve relative output folders inside this repository."""
    path = Path(folder).expanduser()
    if not path.is_absolute():
        path = WORKSPACE_DIR / path
    return path


def xml_escape(value: str) -> str:
    return (
        str(value)
        .replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace('"', "&quot;")
    )


def save_svg_plot(
    x_values,
    y_values,
    x_label,
    y_label,
    title,
    save_path,
    comment_points=None,
):
    width = 1000
    height = 560
    left = 90
    right = 40
    top = 50
    bottom = 90
    plot_w = width - left - right
    plot_h = height - top - bottom

    def x_num(index, value):
        if isinstance(value, datetime):
            return value.timestamp()
        return float(index)

    x_numeric = [x_num(i, value) for i, value in enumerate(x_values)]
    y_numeric = [float(value) for value in y_values]

    xmin = min(x_numeric)
    xmax = max(x_numeric)
    ymin = min(y_numeric)
    ymax = max(y_numeric)

    if xmin == xmax:
        xmax = xmin + 1.0
    if ymin == ymax:
        ymax = ymin + 1.0

    def sx(value):
        return left + ((value - xmin) / (xmax - xmin)) * plot_w

    def sy(value):
        return top + (1.0 - ((value - ymin) / (ymax - ymin))) * plot_h

    points = " ".join(
        f"{sx(x):.2f},{sy(y):.2f}" for x, y in zip(x_numeric, y_numeric)
    )

    y_ticks = []
    for i in range(6):
        yv = ymin + (ymax - ymin) * (i / 5)
        y_ticks.append((yv, sy(yv)))

    if isinstance(x_values[0], datetime):
        labels = [x_values[0], x_values[len(x_values) // 2], x_values[-1]]
        x_tick_items = [
            (sx(labels[0].timestamp()), labels[0].strftime("%H:%M")),
            (sx(labels[1].timestamp()), labels[1].strftime("%H:%M")),
            (sx(labels[2].timestamp()), labels[2].strftime("%H:%M")),
        ]
    else:
        x_tick_items = [
            (left, str(x_values[0])),
            (left + plot_w / 2, str(x_values[len(x_values) // 2])),
            (left + plot_w, str(x_values[-1])),
        ]

    svg = [
        (
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" '
            f'height="{height}" viewBox="0 0 {width} {height}">'
        ),
        '<rect width="100%" height="100%" fill="white"/>',
        (
            f'<text x="{width / 2:.0f}" y="28" text-anchor="middle" '
            f'font-family="Arial" font-size="22">{xml_escape(title)}</text>'
        ),
        (
            f'<line x1="{left}" y1="{top}" x2="{left}" '
            f'y2="{top + plot_h}" stroke="#111" stroke-width="1.5"/>'
        ),
        (
            f'<line x1="{left}" y1="{top + plot_h}" x2="{left + plot_w}" '
            f'y2="{top + plot_h}" stroke="#111" stroke-width="1.5"/>'
        ),
    ]

    for yv, yp in y_ticks:
        svg.append(
            f'<line x1="{left}" y1="{yp:.2f}" x2="{left + plot_w}" '
            f'y2="{yp:.2f}" stroke="#ddd" stroke-width="1"/>'
        )
        svg.append(
            f'<text x="{left - 10}" y="{yp + 4:.2f}" text-anchor="end" '
            f'font-family="Arial" font-size="12">{yv:.2f}</text>'
        )

    for xp, label in x_tick_items:
        svg.append(
            f'<line x1="{xp:.2f}" y1="{top + plot_h}" x2="{xp:.2f}" '
            f'y2="{top + plot_h + 6}" stroke="#111" stroke-width="1"/>'
        )
        svg.append(
            f'<text x="{xp:.2f}" y="{top + plot_h + 24}" '
            f'text-anchor="middle" font-family="Arial" font-size="12">'
            f"{xml_escape(label)}</text>"
        )

    svg.append(
        f'<polyline points="{points}" fill="none" stroke="#1f77b4" '
        f'stroke-width="2"/>'
    )

    for index, (cx, cy, note) in enumerate(comment_points or []):
        px = sx(cx.timestamp() if isinstance(cx, datetime) else cx)
        py = sy(cy)
        x_offset = 18 if index % 2 == 0 else -18
        anchor = "start" if x_offset > 0 else "end"
        safe_note = xml_escape(note)
        svg.append(f'<circle cx="{px:.2f}" cy="{py:.2f}" r="4" fill="#dc2626"/>')
        svg.append(
            f'<line x1="{px:.2f}" y1="{py:.2f}" x2="{px + x_offset:.2f}" '
            f'y2="{py - 22:.2f}" stroke="#c2410c" stroke-width="1"/>'
        )
        svg.append(
            f'<text x="{px + x_offset:.2f}" y="{py - 26:.2f}" '
            f'text-anchor="{anchor}" font-family="Arial" font-size="11" '
            f'fill="#7c2d12">{safe_note}</text>'
        )

    svg.append(
        f'<text x="{width / 2:.0f}" y="{height - 16}" text-anchor="middle" '
        f'font-family="Arial" font-size="14">{xml_escape(x_label)}</text>'
    )
    svg.append(
        f'<text x="22" y="{height / 2:.0f}" text-anchor="middle" '
        f'transform="rotate(-90 22 {height / 2:.0f})" '
        f'font-family="Arial" font-size="14">{xml_escape(y_label)}</text>'
    )
    svg.append("</svg>")

    with open(save_path, "w", encoding="utf-8") as f:
        f.write("\n".join(svg))


def detect_temp_time_columns(headers: list[str]):
    time_col = next((header for header in headers if looks_like(header, "time")), None)
    temp_col = next((header for header in headers if looks_like(header, "temp")), None)
    return time_col, temp_col


def create_temperature_plot(csv_path: str, save_dir: str, file_name: str) -> str:
    if not csv_path or not os.path.exists(csv_path):
        raise ValueError("Please choose a valid CSV file.")
    if not save_dir:
        raise ValueError("Please choose a save folder.")
    if not file_name:
        raise ValueError("Please enter a file name.")

    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        headers = reader.fieldnames or []
        x_col, y_col = detect_temp_time_columns(headers)
        if not x_col or not y_col:
            raise ValueError(
                "Could not find columns containing 'Time' and 'Temp' in the CSV header."
            )

        x_values = []
        y_values = []
        for row in reader:
            x_raw = (row.get(x_col) or "").strip()
            y_raw = (row.get(y_col) or "").strip()
            if not x_raw or not y_raw:
                continue

            try:
                y = float(y_raw)
            except ValueError:
                continue

            x_values.append(parse_time(x_raw))
            y_values.append(y)

    if not x_values or not y_values:
        raise ValueError("No valid rows found for selected columns.")

    output_dir = resolve_workspace_folder(save_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    extension = ".png" if HAS_MATPLOTLIB else ".svg"
    save_path = output_dir / f"{Path(file_name).name}{extension}"
    plot_title = format_plot_title(csv_path)

    if HAS_MATPLOTLIB:
        fig, ax = plt.subplots(figsize=(12, 6))
        fig.patch.set_facecolor("#e6e6e6")
        ax.set_facecolor("#e6e6e6")
        ax.plot(x_values, y_values, linewidth=1.5, color="#1f77b4", label="Temperature")
        ax.set_xlabel("Adjusted Time")
        ax.set_ylabel("Temperature (C)")
        ax.set_title(plot_title)
        ax.grid(True, color="#9b9b9b", alpha=0.8, linewidth=0.8)
        ax.legend(loc="upper left")

        if isinstance(x_values[0], datetime):
            ax.xaxis.set_major_formatter(mdates.DateFormatter("%m-%d %H"))
            ax.xaxis.set_major_locator(mdates.HourLocator(interval=1))
            fig.autofmt_xdate(rotation=0)

        fig.tight_layout()
        fig.savefig(save_path, dpi=160)
        plt.close(fig)
    else:
        save_svg_plot(
            x_values=x_values,
            y_values=y_values,
            x_label="Adjusted Time",
            y_label="Temperature (C)",
            title=plot_title,
            save_path=save_path,
        )

    return str(save_path)


class PlotApp:
    def __init__(self, root):
        import tkinter as tk
        from tkinter import ttk

        self.tk = tk
        self.ttk = ttk
        self.root = root
        self.root.title("Time/Temperature Plot Saver")
        self.root.geometry("720x320")

        self.csv_path = tk.StringVar(value="")
        self.save_dir = tk.StringVar(value=str(DEFAULT_PLOT_DIR))
        self.file_name = tk.StringVar(value="temperature_vs_time")
        self.temp_vs_time_var = tk.BooleanVar(value=True)

        self._build_ui()

    def _build_ui(self):
        frame = self.ttk.Frame(self.root, padding=12)
        frame.pack(fill="both", expand=True)

        self.ttk.Label(frame, text="CSV file").grid(row=0, column=0, sticky="w", pady=(0, 6))
        self.ttk.Entry(frame, textvariable=self.csv_path, width=72).grid(
            row=1, column=0, sticky="ew"
        )
        self.ttk.Button(frame, text="Browse", command=self._browse_csv).grid(
            row=1, column=1, padx=(8, 0)
        )

        self.ttk.Label(frame, text="Plot choices").grid(row=2, column=0, sticky="w", pady=(14, 6))
        self.ttk.Checkbutton(
            frame, text="Temperature vs time", variable=self.temp_vs_time_var
        ).grid(row=3, column=0, sticky="w")

        self.ttk.Label(frame, text="Output file name (without extension)").grid(
            row=4, column=0, sticky="w", pady=(14, 6)
        )
        self.ttk.Entry(frame, textvariable=self.file_name, width=72).grid(
            row=5, column=0, sticky="ew"
        )

        self.ttk.Label(frame, text="Save folder").grid(row=6, column=0, sticky="w", pady=(14, 6))
        self.ttk.Entry(frame, textvariable=self.save_dir, width=72).grid(
            row=7, column=0, sticky="ew"
        )
        self.ttk.Button(frame, text="Browse", command=self._browse_dir).grid(
            row=7, column=1, padx=(8, 0)
        )

        self.ttk.Button(frame, text="Create + Save Plot", command=self._create_plot).grid(
            row=8, column=0, sticky="w", pady=(20, 0)
        )

        frame.columnconfigure(0, weight=1)

    def _browse_csv(self):
        from tkinter import filedialog

        selected = filedialog.askopenfilename(
            title="Choose CSV file",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        )
        if selected:
            self.csv_path.set(selected)

    def _browse_dir(self):
        from tkinter import filedialog

        selected = filedialog.askdirectory(title="Choose save folder")
        if selected:
            self.save_dir.set(selected)

    def _create_plot(self):
        from tkinter import messagebox

        if not self.temp_vs_time_var.get():
            messagebox.showerror("Input error", "Please check 'Temperature vs time' to create a plot.")
            return

        try:
            save_path = create_temperature_plot(
                self.csv_path.get().strip(),
                self.save_dir.get().strip(),
                self.file_name.get().strip(),
            )
        except Exception as exc:
            messagebox.showerror("Plot error", str(exc))
            return

        if HAS_MATPLOTLIB:
            messagebox.showinfo("Saved", f"Plot saved to:\n{save_path}")
        else:
            messagebox.showinfo(
                "Saved",
                f"Matplotlib is not installed, so the plot was saved as SVG:\n{save_path}",
            )


def run_plotter():
    import tkinter as tk

    root = tk.Tk()
    PlotApp(root)
    root.mainloop()


class PlottingExportMixin:
    def _on_export(self):
        """Export growth log and optionally save selected run plots as PNGs."""
        from PyQt6.QtWidgets import QMessageBox

        metadata = self.monitor.get_session_metadata()
        path = self.growth_log.export_growth_log(metadata)
        if path:
            self.statusBar().showMessage(f"Growth log exported: {path}", 5000)
        else:
            self.statusBar().showMessage(
                "No commit entries found; exporting plots from run CSV only.", 5000,
            )

        csv_path = self._resolve_run_csv_path(path)
        if csv_path is None:
            QMessageBox.warning(
                self,
                "Run CSV not found",
                "Growth log exported, but no run CSV was selected.",
            )
            return

        selections = self._show_plot_export_dialog()
        if selections is None:
            return

        saved_paths, missing = self._save_selected_plots(csv_path, selections)
        if saved_paths:
            msg = "Saved plot files:\n" + "\n".join(str(p) for p in saved_paths)
            if not HAS_MATPLOTLIB:
                msg += "\n\nMatplotlib is not installed in this Python environment, so plots were saved as SVG files."
            if missing:
                msg += "\n\nSkipped (missing data):\n" + "\n".join(missing)
            QMessageBox.information(self, "Plot export complete", msg)
            self.statusBar().showMessage(
                f"Saved {len(saved_paths)} plot PNG file(s)", 6000,
            )
            return

        if missing:
            QMessageBox.warning(
                self,
                "No plots saved",
                "Selected plots could not be created due to missing data:\n"
                + "\n".join(missing),
            )
        else:
            QMessageBox.information(
                self,
                "No plots selected",
                "No plot options were selected.",
            )

    def _resolve_run_csv_path(self, export_path: str | None) -> Path | None:
        from PyQt6.QtWidgets import QFileDialog

        export_dir = (
            Path(export_path).parent
            if export_path
            else (
                self.growth_log.session_dir
                if self.growth_log.session_dir is not None
                else Path.cwd()
            )
        )
        candidates = [export_dir / "sensor_log.csv"]
        if self.growth_log.session_dir:
            candidates.append(self.growth_log.session_dir / "sensor_log.csv")

        for candidate in candidates:
            if candidate.exists():
                return candidate

        selected, _ = QFileDialog.getOpenFileName(
            self,
            "Select extracted run CSV",
            str(export_dir),
            "CSV Files (*.csv);;All Files (*)",
        )
        if not selected:
            return None
        return Path(selected)

    def _show_plot_export_dialog(self) -> dict[str, bool] | None:
        from PyQt6.QtWidgets import (
            QCheckBox,
            QDialog,
            QDialogButtonBox,
            QGridLayout,
            QVBoxLayout,
        )

        dialog = QDialog(self)
        dialog.setWindowTitle("Save Plot PNGs")
        layout = QVBoxLayout(dialog)

        temp_cb = QCheckBox("Temperature vs time")
        temp_cb.setChecked(True)
        voltage_cb = QCheckBox("Voltage vs time")
        current_cb = QCheckBox("Current vs time")
        power_cb = QCheckBox("Power vs time")
        dtdt_cb = QCheckBox("dT/dt vs time")
        resistance_cb = QCheckBox("Resistance vs time")

        checks_grid = QGridLayout()
        checks_grid.addWidget(temp_cb, 0, 0)
        checks_grid.addWidget(voltage_cb, 0, 1)
        checks_grid.addWidget(current_cb, 1, 0)
        checks_grid.addWidget(power_cb, 1, 1)
        checks_grid.addWidget(dtdt_cb, 2, 0)
        checks_grid.addWidget(resistance_cb, 2, 1)
        layout.addLayout(checks_grid)

        buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Save
            | QDialogButtonBox.StandardButton.Cancel
        )
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        layout.addWidget(buttons)

        if dialog.exec() != QDialog.DialogCode.Accepted:
            return None

        return {
            "temp_time": temp_cb.isChecked(),
            "voltage_time": voltage_cb.isChecked(),
            "current_time": current_cb.isChecked(),
            "power_time": power_cb.isChecked(),
            "dtdt_time": dtdt_cb.isChecked(),
            "resistance_time": resistance_cb.isChecked(),
        }

    @staticmethod
    def _parse_float(text) -> float | None:
        if text is None:
            return None
        s = str(text).strip()
        if not s:
            return None
        try:
            return float(s)
        except (TypeError, ValueError):
            return None

    @classmethod
    def _get_numeric(cls, row: dict, *keys: str) -> float | None:
        for key in keys:
            if key in row:
                value = cls._parse_float(row.get(key))
                if value is not None:
                    return value
        return None

    def _load_run_series(self, csv_path: Path) -> dict[str, list[float]]:
        data = {
            "temp_t": [],
            "temp_y": [],
            "v_t": [],
            "v_y": [],
            "i_t": [],
            "i_y": [],
            "p_t": [],
            "p_y": [],
            "r_t": [],
            "r_y": [],
            "dtdt_t": [],
            "dtdt_y": [],
        }

        first_ts = None
        with open(csv_path, newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                elapsed = self._get_numeric(
                    row,
                    "elapsed_s",
                    "elapsed",
                    "time_s",
                    "time_sec",
                    "time_seconds",
                )
                if elapsed is None and "timestamp" in row and row["timestamp"]:
                    try:
                        ts = datetime.fromisoformat(row["timestamp"].strip())
                        if first_ts is None:
                            first_ts = ts
                        elapsed = (ts - first_ts).total_seconds()
                    except ValueError:
                        elapsed = None
                if elapsed is None:
                    continue

                temp = self._get_numeric(
                    row,
                    "pyrometer_temp_C",
                    "temperature_C",
                    "temp_C",
                    "temp",
                    "temperature",
                )
                voltage = self._get_numeric(
                    row, "voltage_V", "voltage", "v_actual", "V",
                )
                current = self._get_numeric(
                    row, "current_A", "current", "i_actual", "I",
                )
                power = self._get_numeric(row, "power_W", "power", "P")
                resistance = self._get_numeric(
                    row, "resistance_ohm", "resistance", "R",
                )

                if power is None and voltage is not None and current is not None:
                    power = voltage * current
                if (
                    resistance is None
                    and voltage is not None
                    and current is not None
                    and abs(current) > 1e-12
                ):
                    resistance = voltage / current

                if temp is not None:
                    data["temp_t"].append(elapsed)
                    data["temp_y"].append(temp)
                if voltage is not None:
                    data["v_t"].append(elapsed)
                    data["v_y"].append(voltage)
                if current is not None:
                    data["i_t"].append(elapsed)
                    data["i_y"].append(current)
                if power is not None:
                    data["p_t"].append(elapsed)
                    data["p_y"].append(power)
                if resistance is not None:
                    data["r_t"].append(elapsed)
                    data["r_y"].append(resistance)

        temp_t = data["temp_t"]
        temp_y = data["temp_y"]
        for idx in range(1, len(temp_t)):
            dt = temp_t[idx] - temp_t[idx - 1]
            if dt <= 0:
                continue
            dtemp = temp_y[idx] - temp_y[idx - 1]
            data["dtdt_t"].append(temp_t[idx])
            data["dtdt_y"].append(dtemp / dt)

        return data

    @staticmethod
    def _save_single_plot(
        x: list[float], y: list[float], title: str, y_label: str, save_path: Path,
    ):
        if not HAS_MATPLOTLIB:
            save_svg_plot(
                x_values=x,
                y_values=y,
                x_label="Time (s)",
                y_label=y_label,
                title=title,
                save_path=save_path,
            )
            return

        fig, ax = plt.subplots(figsize=(10, 5))
        ax.plot(x, y, linewidth=1.8)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel(y_label)
        ax.set_title(title)
        ax.grid(True, alpha=0.25)
        fig.tight_layout()
        fig.savefig(save_path, dpi=160)
        plt.close(fig)

    @classmethod
    def _load_comment_points(
        cls,
        csv_path: Path,
        temp_t: list[float],
        temp_y: list[float],
    ) -> list[tuple[float, float, str]]:
        if not temp_t or not temp_y:
            return []

        comment_path = csv_path.parent / "commit_log.csv"
        if not comment_path.exists():
            return []

        points: list[tuple[float, float, str]] = []
        sensor_start_ts = None
        try:
            with open(csv_path, newline="") as sensor_file:
                sensor_reader = csv.DictReader(sensor_file)
                first_row = next(sensor_reader, None)
                if first_row and first_row.get("timestamp"):
                    sensor_start_ts = datetime.fromisoformat(
                        first_row["timestamp"].strip(),
                    )
        except Exception:
            sensor_start_ts = None

        def temp_on_curve(elapsed_s: float) -> float:
            if elapsed_s <= temp_t[0]:
                return temp_y[0]
            if elapsed_s >= temp_t[-1]:
                return temp_y[-1]

            for i in range(1, len(temp_t)):
                x0 = temp_t[i - 1]
                x1 = temp_t[i]
                if elapsed_s > x1:
                    continue
                y0 = temp_y[i - 1]
                y1 = temp_y[i]
                if x1 == x0:
                    return y1
                alpha = (elapsed_s - x0) / (x1 - x0)
                return y0 + alpha * (y1 - y0)
            return temp_y[-1]

        with open(comment_path, newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                note = (row.get("note") or "").strip()
                if not note:
                    continue

                elapsed = cls._parse_float(row.get("elapsed_s"))
                if elapsed is None and sensor_start_ts and row.get("timestamp"):
                    try:
                        commit_ts = datetime.fromisoformat(
                            row["timestamp"].strip(),
                        )
                        elapsed = (commit_ts - sensor_start_ts).total_seconds()
                    except ValueError:
                        elapsed = None

                if elapsed is None:
                    continue
                if elapsed < temp_t[0] or elapsed > temp_t[-1]:
                    continue

                points.append((elapsed, temp_on_curve(elapsed), note))
        return points

    @classmethod
    def _save_temp_plot_with_comments(
        cls,
        data: dict[str, list[float]],
        csv_path: Path,
        save_path: Path,
    ):
        x = data["temp_t"]
        y = data["temp_y"]
        comments = cls._load_comment_points(csv_path, x, y)

        if not HAS_MATPLOTLIB:
            save_svg_plot(
                x_values=x,
                y_values=y,
                x_label="Time (s)",
                y_label="Temperature (C)",
                title="Temperature vs Time (Comments)",
                save_path=save_path,
                comment_points=comments,
            )
            return

        fig, ax = plt.subplots(figsize=(12, 6))
        ax.plot(x, y, linewidth=1.8, color="#1f77b4")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Temperature (C)")
        ax.set_title("Temperature vs Time (Comments)")
        ax.grid(True, alpha=0.25)

        for i, (cx, cy, note) in enumerate(comments):
            x_offset = 28 if i % 2 == 0 else -190
            y_offset = 20 if i % 3 != 0 else -35
            ax.plot(cx, cy, "o", color="#dc2626", markersize=4)
            ax.annotate(
                note,
                xy=(cx, cy),
                xytext=(x_offset, y_offset),
                textcoords="offset points",
                fontsize=8,
                ha="left" if x_offset > 0 else "right",
                va="bottom" if y_offset > 0 else "top",
                bbox={
                    "boxstyle": "round,pad=0.25",
                    "fc": "#fff7ed",
                    "ec": "#c2410c",
                    "alpha": 0.95,
                },
                arrowprops={
                    "arrowstyle": "-",
                    "color": "#c2410c",
                    "lw": 0.9,
                    "shrinkA": 0,
                    "shrinkB": 0,
                },
            )

        fig.tight_layout()
        fig.savefig(save_path, dpi=180)
        plt.close(fig)

    def _save_selected_plots(
        self, csv_path: Path, selections: dict[str, bool],
    ) -> tuple[list[Path], list[str]]:
        data = self._load_run_series(csv_path)
        out_dir = csv_path.parent / "plots"
        out_dir.mkdir(parents=True, exist_ok=True)
        saved: list[Path] = []
        missing: list[str] = []

        plot_specs = [
            ("temp_time", "temp_t", "temp_y", "temperature_vs_time.png", "Temperature vs Time", "Temperature (C)"),
            ("voltage_time", "v_t", "v_y", "voltage_vs_time.png", "Voltage vs Time", "Voltage (V)"),
            ("current_time", "i_t", "i_y", "current_vs_time.png", "Current vs Time", "Current (A)"),
            ("power_time", "p_t", "p_y", "power_vs_time.png", "Power vs Time", "Power (W)"),
            ("dtdt_time", "dtdt_t", "dtdt_y", "dTdt_vs_time.png", "dT/dt vs Time", "dT/dt (C/s)"),
            ("resistance_time", "r_t", "r_y", "resistance_vs_time.png", "Resistance vs Time", "Resistance (Ohm)"),
        ]

        for selection_key, x_key, y_key, filename, title, y_label in plot_specs:
            if not selections.get(selection_key):
                continue
            if data[x_key] and data[y_key]:
                if HAS_MATPLOTLIB:
                    out = out_dir / filename
                else:
                    out = out_dir / filename.replace(".png", ".svg")
                self._save_single_plot(data[x_key], data[y_key], title, y_label, out)
                saved.append(out)
                if selection_key == "temp_time":
                    comments_filename = (
                        "temperature_vs_time_with_comments.png"
                        if HAS_MATPLOTLIB
                        else "temperature_vs_time_with_comments.svg"
                    )
                    out_comments = out_dir / comments_filename
                    self._save_temp_plot_with_comments(data, csv_path, out_comments)
                    saved.append(out_comments)
            else:
                missing.append(title)

        return saved, missing


def configure_qt_plugin_path():
    try:
        import PyQt6
    except Exception:
        return

    qt_plugins = Path(PyQt6.__file__).parent / "Qt6" / "plugins"
    if qt_plugins.exists() and "QT_QPA_PLATFORM_PLUGIN_PATH" not in os.environ:
        os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = str(qt_plugins / "platforms")


def run_monitor():
    configure_qt_plugin_path()

    from PyQt6.QtWidgets import QApplication
    from gui.growth_app import GrowthApp as BaseGrowthApp

    class GrowthAppWithPlotExport(PlottingExportMixin, BaseGrowthApp):
        pass

    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = GrowthAppWithPlotExport()
    window.show()
    sys.exit(app.exec())


def main():
    parser = argparse.ArgumentParser(
        description="Launch the growth monitor GUI or the temperature plot saver."
    )
    parser.add_argument(
        "--plotter",
        action="store_true",
        help="Open the standalone CSV time/temperature plot saver.",
    )
    args = parser.parse_args()

    if args.plotter:
        run_plotter()
    else:
        run_monitor()


if __name__ == "__main__":
    main()
