#!/usr/bin/env python3
"""Growth Profile Explorer — post-hoc session analysis.

Reads a Growth Monitor session directory and renders diagnostic charts
that would otherwise require manual CSV wrangling. Day 4 (Jul 16 2026)
ships the data layer + two core charts:

  - ``temperature_profile_annotated.png`` — T vs t with commit / manual /
    auto-capture event overlays
  - ``pyro_stability.png`` — pyrometer mean ± std over time

Days 5-6 will layer classifier trajectory, score distributions, grower-
vs-classifier agreement scatter, and HTML report assembly on top of the
same ``SessionArtifacts`` reader.

Usage:
    python scripts/growth_profile_explorer.py \\
        logs/growths/<session_dir>/ \\
        --output-dir logs/growths/<session_dir>/analysis/

Consumed CSVs (all optional except ``sensor_log.csv`` — missing files
yield empty row lists so the tool works against pre-Jul-10 sessions
that predate ``manual_events.csv`` / ``events_labels.csv`` /
``live_labels.csv``):

    sensor_log.csv          — GrowthLogger.SENSOR_FIELDS       (always)
    commit_log.csv          — GrowthLogger.COMMIT_FIELDS       (always)
    auto_capture_events.csv — GrowthLogger.AUTO_CAPTURE_FIELDS (always)
    manual_events.csv       — GrowthLogger.MANUAL_EVENT_FIELDS (Jul 10+)
    heartbeat_log.csv       — GrowthLogger.HEARTBEAT_FIELDS    (always)
    events_labels.csv       — GrowthLogger.EVENT_LABEL_FIELDS  (post-label)

See ``gui/growth_logger.py`` for schema definitions.
"""
from __future__ import annotations

import argparse
import csv
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional


# Event marker colors on temperature charts. First three keys mirror
# gui/scrubber_tab.py:_SOURCE_COLORS (heartbeat cyan / manual amber /
# auto grey) so the visual identity carries from the live scrubber into
# post-hoc charts. The fourth key ("commit") is chart-scope only — LOG
# ENTRY events don't need their own scrubber marker but deserve a
# distinct color here. Duplicated (not imported from gui.scrubber_tab)
# to keep this analysis script free of PyQt6 imports; if the scrubber's
# three colors change, update both places.
EVENT_MARKER_COLORS = {
    "heartbeat": "#0891b2",  # cyan — continuous capture
    "manual":    "#d97706",  # amber — MARK EVENT
    "auto":      "#888888",  # grey — auto-capture engine
    "commit":    "#059669",  # emerald — LOG ENTRY (chart-only)
}

# Matches plot_temperature.py's line color for cross-tool visual
# consistency. If both scripts render side-by-side (e.g. Day 6 HTML
# report), pyro traces look the same.
_TRACE_COLOR = "#0d9488"  # teal


def _safe_float(value) -> Optional[float]:
    """Parse a string-like to float; return None on empty/missing/invalid.

    Mirrors ``plot_temperature.py:37-53``'s tolerance for messy CSV rows
    (blank cells, wrong dtypes, ``None`` when a key is absent). Callers
    check the return for ``None`` and skip the row.
    """
    if value is None:
        return None
    s = str(value).strip()
    if not s:
        return None
    try:
        return float(s)
    except ValueError:
        return None


def _read_csv_rows(path: Path) -> list[dict[str, str]]:
    """Read a CSV into a list-of-dicts. Missing file → empty list.

    Values stay as strings (``csv.DictReader`` default). Per-field type
    coercion is caller-side via ``_safe_float`` — keeps this reader
    schema-agnostic so it survives future column additions.
    """
    if not path.exists():
        return []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        return list(reader)


@dataclass
class SessionArtifacts:
    """All CSV data from one Growth Monitor session, decoded once.

    One field per CSV — each is a list of dicts (raw string cells as
    read from disk). Typed accessors on the dataclass coerce columns to
    float when needed. Missing optional CSVs materialize as empty
    lists, so downstream chart renderers can rely on iteration
    protocols regardless of which files the session logged.
    """
    session_dir: Path
    sensor_rows: list[dict[str, str]] = field(default_factory=list)
    commit_rows: list[dict[str, str]] = field(default_factory=list)
    auto_capture_rows: list[dict[str, str]] = field(default_factory=list)
    manual_event_rows: list[dict[str, str]] = field(default_factory=list)
    heartbeat_rows: list[dict[str, str]] = field(default_factory=list)
    event_label_rows: list[dict[str, str]] = field(default_factory=list)

    @classmethod
    def from_session_dir(cls, session_dir: Path) -> "SessionArtifacts":
        """Read all six CSVs from a Growth Monitor session directory.

        Raises ``FileNotFoundError`` only if ``session_dir`` doesn't
        exist or isn't a directory. Individual CSVs missing from a
        valid session dir do NOT raise — they yield empty lists so
        pre-schema sessions still open cleanly.
        """
        session_dir = Path(session_dir)
        if not session_dir.is_dir():
            raise FileNotFoundError(
                f"Session directory not found: {session_dir}"
            )
        return cls(
            session_dir=session_dir,
            sensor_rows=_read_csv_rows(session_dir / "sensor_log.csv"),
            commit_rows=_read_csv_rows(session_dir / "commit_log.csv"),
            auto_capture_rows=_read_csv_rows(
                session_dir / "auto_capture_events.csv"
            ),
            manual_event_rows=_read_csv_rows(
                session_dir / "manual_events.csv"
            ),
            heartbeat_rows=_read_csv_rows(
                session_dir / "heartbeat_log.csv"
            ),
            event_label_rows=_read_csv_rows(
                session_dir / "events_labels.csv"
            ),
        )

    def temperature_series(self) -> tuple[list[float], list[float]]:
        """Return ``(elapsed_s, pyrometer_temp_C)`` from ``sensor_rows``.

        Skips rows where either column is missing or unparseable — same
        policy as ``plot_temperature.py:35-53``. Empty return
        ``([], [])`` means no chartable data (empty session, or all
        rows had bad temp reads).
        """
        elapsed: list[float] = []
        temps: list[float] = []
        for row in self.sensor_rows:
            t = _safe_float(row.get("elapsed_s"))
            temp = _safe_float(row.get("pyrometer_temp_C"))
            if t is None or temp is None:
                continue
            elapsed.append(t)
            temps.append(temp)
        return elapsed, temps

    def temperature_std_series(
        self,
    ) -> tuple[list[float], list[float], list[float]]:
        """Return ``(elapsed_s, mean_T, std_T)`` from ``sensor_rows``.

        Skips rows missing any of the three columns. Pre-Apr-2026
        sessions predate ``pyrometer_temp_std_C`` and return empty
        lists here (chart renderers detect this and skip PNG output).
        """
        elapsed: list[float] = []
        temps: list[float] = []
        stds: list[float] = []
        for row in self.sensor_rows:
            t = _safe_float(row.get("elapsed_s"))
            temp = _safe_float(row.get("pyrometer_temp_C"))
            std = _safe_float(row.get("pyrometer_temp_std_C"))
            if t is None or temp is None or std is None:
                continue
            elapsed.append(t)
            temps.append(temp)
            stds.append(std)
        return elapsed, temps, stds

    def event_overlays(self) -> list[tuple[float, str]]:
        """List of ``(elapsed_s, source)`` for chart vertical-line overlays.

        Sources come from three discrete-event CSVs:

          - ``commit_log.csv``        → ``"commit"``  (LOG ENTRY)
          - ``manual_events.csv``     → ``"manual"``  (MARK EVENT)
          - ``auto_capture_events.csv`` → ``"auto"``  (auto-capture engine)

        Heartbeat rows are intentionally excluded — continuous capture
        would swamp any chart with hundreds of overlays. Heartbeats
        form the background timeline (visible via density of the
        temperature trace itself), not discrete events.

        Result is sorted by ``elapsed_s``.
        """
        overlays: list[tuple[float, str]] = []
        for source, rows in (
            ("commit", self.commit_rows),
            ("manual", self.manual_event_rows),
            ("auto", self.auto_capture_rows),
        ):
            for row in rows:
                t = _safe_float(row.get("elapsed_s"))
                if t is not None:
                    overlays.append((t, source))
        overlays.sort(key=lambda pair: pair[0])
        return overlays

    def label_for_event(self, event_idx: int) -> Optional[dict[str, str]]:
        """Look up the ``events_labels.csv`` row for a given event_idx.

        Returns the row dict (string cells) or ``None`` if that event
        wasn't labeled. Callers use this to correlate an auto-capture
        or manual event with the grower's post-hoc reconstruction
        label — the primary join Day 5's grower-vs-classifier chart
        will need.
        """
        target = str(event_idx)
        for row in self.event_label_rows:
            if row.get("event_idx") == target:
                return row
        return None


def render_temperature_chart(
    artifacts: SessionArtifacts,
    output_path: Path,
    dpi: int = 150,
    title: Optional[str] = None,
) -> Optional[Path]:
    """Render T vs t chart with event overlays.

    Returns ``output_path`` on success, or ``None`` (and writes no
    file) if the session has no chartable temperature data. Callers
    should check the return before advertising the file exists.
    """
    elapsed, temps = artifacts.temperature_series()
    if not temps:
        return None

    # Import matplotlib lazily so ``SessionArtifacts.from_session_dir``
    # stays cheap for callers that only need the data layer (Day 7's
    # batch inference doesn't render anything, and Day 4's tests only
    # need matplotlib for the 3 chart tests, not the 7 reader tests).
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    elapsed_min = [t / 60.0 for t in elapsed]

    fig, ax = plt.subplots(figsize=(11, 5))
    ax.plot(elapsed_min, temps, color=_TRACE_COLOR, linewidth=1.2,
            zorder=3, label="Pyro T")
    ax.set_xlabel("Elapsed time (min)", fontsize=11)
    ax.set_ylabel("Pyrometer T (°C)", fontsize=11)
    ax.set_title(
        title or f"Temperature profile — {artifacts.session_dir.name}",
        fontsize=13,
    )
    ax.grid(True, alpha=0.3)

    # Overlay commit/manual/auto events as vertical lines. Dedup the
    # legend by only attaching a label on the first line per source
    # (matplotlib would render N separate legend entries otherwise).
    seen_sources: set[str] = set()
    for t_s, source in artifacts.event_overlays():
        color = EVENT_MARKER_COLORS.get(source, "#000000")
        label = source if source not in seen_sources else None
        ax.axvline(
            x=t_s / 60.0, color=color, alpha=0.55, linewidth=0.8,
            linestyle="--", label=label, zorder=1,
        )
        seen_sources.add(source)

    ax.legend(loc="best", fontsize=9, framealpha=0.9)
    fig.tight_layout()
    fig.savefig(str(output_path), dpi=dpi)
    plt.close(fig)
    return output_path


def render_pyro_stability_chart(
    artifacts: SessionArtifacts,
    output_path: Path,
    dpi: int = 150,
    title: Optional[str] = None,
) -> Optional[Path]:
    """Render pyrometer mean ± std band over time.

    Uses ``pyrometer_temp_std_C`` from the 5-sample rolling window
    that ``PyrometerWorker`` writes with each reading. Returns
    ``output_path`` on success; returns ``None`` (writes no file)
    when the session has no std data — either a pre-Apr-2026 session
    predating the column, or a session where every row failed the
    rolling window.
    """
    elapsed, temps, stds = artifacts.temperature_std_series()
    if not stds:
        return None

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    elapsed_min = [t / 60.0 for t in elapsed]
    upper = [m + s for m, s in zip(temps, stds)]
    lower = [m - s for m, s in zip(temps, stds)]

    fig, ax = plt.subplots(figsize=(11, 5))
    ax.fill_between(
        elapsed_min, lower, upper, color=_TRACE_COLOR, alpha=0.25,
        label="±1σ (5-sample rolling)",
    )
    ax.plot(elapsed_min, temps, color=_TRACE_COLOR, linewidth=1.2,
            label="Mean T")
    ax.set_xlabel("Elapsed time (min)", fontsize=11)
    ax.set_ylabel("Pyrometer T (°C)", fontsize=11)
    ax.set_title(
        title or f"Pyro stability — {artifacts.session_dir.name}",
        fontsize=13,
    )
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=9, framealpha=0.9)

    fig.tight_layout()
    fig.savefig(str(output_path), dpi=dpi)
    plt.close(fig)
    return output_path


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Render post-hoc analysis charts for a Growth Monitor session."
        ),
    )
    parser.add_argument(
        "session_path", type=Path,
        help="Path to session directory (contains sensor_log.csv, etc.)",
    )
    parser.add_argument(
        "--output-dir", "-o", type=Path, default=None,
        help=(
            "Directory to write PNGs (default: <session_path>/analysis/)"
        ),
    )
    parser.add_argument("--dpi", type=int, default=150)
    args = parser.parse_args()

    try:
        artifacts = SessionArtifacts.from_session_dir(args.session_path)
    except FileNotFoundError as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

    out_dir = args.output_dir or (args.session_path / "analysis")
    out_dir.mkdir(parents=True, exist_ok=True)

    written: list[Path] = []
    p = render_temperature_chart(
        artifacts,
        out_dir / "temperature_profile_annotated.png",
        dpi=args.dpi,
    )
    if p is not None:
        written.append(p)
    p = render_pyro_stability_chart(
        artifacts,
        out_dir / "pyro_stability.png",
        dpi=args.dpi,
    )
    if p is not None:
        written.append(p)

    if not written:
        print(
            f"No chartable data in session {args.session_path.name}",
            file=sys.stderr,
        )
        sys.exit(1)

    print(f"Rendered {len(written)} chart(s) to {out_dir}:")
    for p in written:
        print(f"  {p.name}")


if __name__ == "__main__":
    main()
