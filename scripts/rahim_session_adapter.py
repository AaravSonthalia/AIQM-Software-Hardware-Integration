"""Wrap a Rahim archival dataset into a Growth-Monitor session directory.

Rahim's 2022 datasets ship as a `Substrates/` folder of raw BMP frames named
`RR<yymmdd>A_<TEMP>C_<HHMM>[-##].bmp`. The Scrubber tab expects a session
directory with `heartbeat_log.csv`, `manual_events.csv`, and
`auto_capture_events.csv`. This adapter synthesizes those CSVs so the Scrubber
can play the archived frames back on the timeline.

Design notes:
- READ-ONLY against the source dataset. We never modify Rahim's files.
- The wrapper session dir is written to `/tmp/rahim_wrapped_<dataset>` by
  default (throwaway, safe to nuke).
- `pyrometer_temp_C` is parsed from the filename (`_1012C_` → 1012.0).
- `elapsed_s` is derived from the `HHMM` (+ trailing sub-index) in the filename.
  Sub-indexes (`-01`, `-02`, ...) get treated as sub-minute offsets so frames
  with identical HHMM don't collide on elapsed_s.
- Frames without a parseable temperature are still indexed, with `pyro=""`.
- Emits `auto_capture_events.csv` and `manual_events.csv` as header-only so
  the timeline shows only cyan (heartbeat) markers.

Usage:
    python scripts/rahim_session_adapter.py \\
        /Users/aj/Downloads/rahim_2022_02_06/2022_02_06_renamed

    # Then:
    python scripts/scrubber_demo.py --session /tmp/rahim_wrapped_2022_02_06

Or one-liner via the --launch flag:
    python scripts/rahim_session_adapter.py \\
        /Users/aj/Downloads/rahim_2022_02_06/2022_02_06_renamed --launch
"""
from __future__ import annotations

import argparse
import csv
import json
import re
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime, timedelta
from pathlib import Path
from typing import Optional


# Filename pattern: RR220206A_1012C_1946.bmp  or  RR220206A_165C_1701-02.bmp
# Groups: (sample_id, temp_C, hhmm, sub_index or None)
_FILENAME_RE = re.compile(
    r"^(?P<sample>RR\d{6}[A-Z])_(?P<temp>\d+)C_(?P<hhmm>\d{4})(?:-(?P<sub>\d+))?\.bmp$",
    re.IGNORECASE,
)


@dataclass
class _RahimFrame:
    path: Path
    hhmm: int      # e.g. 1946 = 19:46
    sub_idx: int   # 0 if no sub-index, else 1, 2, ...
    temp_C: Optional[float]

    @property
    def sort_key(self) -> tuple[int, int]:
        return (self.hhmm, self.sub_idx)


def _parse_frames(substrates_dir: Path) -> list[_RahimFrame]:
    """Enumerate BMPs and parse filename metadata. Returns time-sorted list."""
    frames: list[_RahimFrame] = []
    for bmp in substrates_dir.iterdir():
        if bmp.suffix.lower() != ".bmp":
            continue
        m = _FILENAME_RE.match(bmp.name)
        if not m:
            # Unparseable name — skip silently, we'll report count at end.
            continue
        temp_str = m.group("temp")
        temp_val = float(temp_str) if temp_str.isdigit() else None
        frames.append(_RahimFrame(
            path=bmp,
            hhmm=int(m.group("hhmm")),
            sub_idx=int(m.group("sub") or 0),
            temp_C=temp_val,
        ))
    frames.sort(key=lambda f: f.sort_key)
    return frames


def _elapsed_seconds(f: _RahimFrame, baseline_hhmm: int) -> float:
    """Compute elapsed_s from an HHMM baseline. Sub-indexes are ~1s apart."""
    hh, mm = divmod(f.hhmm, 100)
    b_hh, b_mm = divmod(baseline_hhmm, 100)
    # Handle overnight wrap: if current time < baseline, add 24h
    minutes = (hh * 60 + mm) - (b_hh * 60 + b_mm)
    if minutes < 0:
        minutes += 24 * 60
    return minutes * 60.0 + f.sub_idx * 1.0


def _iso_timestamp(dataset_date: str, f: _RahimFrame) -> str:
    """Build an ISO timestamp for the frame from dataset date + HHMM."""
    # dataset_date is like "2022_02_06"
    yyyy, mm, dd = dataset_date.split("_")
    hh, mn = divmod(f.hhmm, 100)
    ts = datetime(int(yyyy), int(mm), int(dd), hh, mn) + timedelta(seconds=f.sub_idx)
    return ts.isoformat(timespec="milliseconds")


def _write_heartbeat_csv(
    wrapped_dir: Path,
    frames: list[_RahimFrame],
    dataset_date: str,
) -> None:
    if not frames:
        return
    # Baseline against the FIRST frame's total elapsed (including sub_idx)
    # so elapsed_s starts at exactly 0.0 for frame 1. Prior version only
    # baselined HHMM which gave sub_idx * 1.0 as the first elapsed value.
    baseline_hhmm = frames[0].hhmm
    baseline_sub = frames[0].sub_idx
    with (wrapped_dir / "heartbeat_log.csv").open("w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["timestamp", "elapsed_s", "heartbeat_idx", "pyrometer_temp_C", "frame_path"])
        for idx, f in enumerate(frames, start=1):
            elapsed = _elapsed_seconds(f, baseline_hhmm) - baseline_sub * 1.0
            w.writerow([
                _iso_timestamp(dataset_date, f),
                f"{elapsed:.2f}",
                idx,
                "" if f.temp_C is None else f"{f.temp_C:.1f}",
                str(f.path.resolve()),  # absolute — Rahim's frames don't move
            ])


def _write_empty_csv(path: Path, columns: list[str]) -> None:
    with path.open("w", newline="") as fh:
        csv.writer(fh).writerow(columns)


def _write_session_metadata(wrapped_dir: Path, dataset_date: str, n_frames: int) -> None:
    meta = {
        "date": dataset_date.replace("_", "-"),
        "grower": "Rahim (archived)",
        "sample_id": f"Rahim_{dataset_date}",
        "session_end": datetime.now().isoformat(timespec="seconds"),
        "total_entries": 0,
        "source": "rahim_session_adapter.py",
        "wrapped_from": None,  # filled by caller
        "frame_count": n_frames,
    }
    (wrapped_dir / "session_metadata.json").write_text(json.dumps(meta, indent=2))


def wrap_dataset(source_dir: Path, output_dir: Optional[Path] = None) -> Path:
    """Wrap a Rahim dataset. Returns the path to the wrapped session dir."""
    # Locate the Substrates/ subdir — Rahim's layout puts frames there.
    substrates = source_dir / "Substrates"
    if not substrates.exists():
        # Some datasets use a differently-named subdir (e.g., 04_11_2022_substrates)
        alt = next(
            (p for p in source_dir.iterdir() if p.is_dir() and "substrate" in p.name.lower()),
            None,
        )
        if alt is None:
            raise FileNotFoundError(
                f"No Substrates/ subdir found in {source_dir}. "
                "Expected raw BMP frames in a subdir with 'substrate' in the name."
            )
        substrates = alt

    frames = _parse_frames(substrates)
    if not frames:
        raise ValueError(
            f"No parseable BMP files found in {substrates}. "
            "Filenames must match RR<yymmdd>A_<TEMP>C_<HHMM>.bmp"
        )

    # Infer dataset date from source dir name — "2022_02_06_renamed" → "2022_02_06"
    src_name = source_dir.name
    date_match = re.search(r"(\d{4}_\d{2}_\d{2})", src_name)
    dataset_date = date_match.group(1) if date_match else "1900_01_01"

    if output_dir is None:
        output_dir = Path("/tmp") / f"rahim_wrapped_{dataset_date}"
    output_dir.mkdir(parents=True, exist_ok=True)

    _write_heartbeat_csv(output_dir, frames, dataset_date)
    _write_empty_csv(
        output_dir / "manual_events.csv",
        ["timestamp", "elapsed_s", "event_idx", "pyrometer_temp_C",
         "voltage_V", "current_A", "psu_source", "frame_path", "note"],
    )
    _write_empty_csv(
        output_dir / "auto_capture_events.csv",
        ["timestamp", "elapsed_s", "event_idx", "change_score",
         "pyrometer_temp_C", "buffer_count", "buffer_dir",
         "event_state", "state_changed_at"],
    )
    _write_session_metadata(output_dir, dataset_date, len(frames))

    # Print summary
    parseable = len(frames)
    all_bmps = sum(1 for p in substrates.iterdir() if p.suffix.lower() == ".bmp")
    skipped = all_bmps - parseable
    temps = [f.temp_C for f in frames if f.temp_C is not None]
    print(f"Wrapped session written to: {output_dir}")
    print(f"  Frames indexed: {parseable} (skipped {skipped} unparseable BMPs)")
    if temps:
        print(f"  Temperature range: {min(temps):.0f}°C to {max(temps):.0f}°C")
    span_s = _elapsed_seconds(frames[-1], frames[0].hhmm)
    print(f"  Span: {span_s/60:.1f} minutes across HHMM values")
    return output_dir


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument(
        "source_dir",
        type=Path,
        help="Path to Rahim dataset dir (contains Substrates/ or similar)",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Where to write the wrapped session (default: /tmp/rahim_wrapped_<date>)",
    )
    parser.add_argument(
        "--launch",
        action="store_true",
        help="After wrapping, launch scripts/scrubber_demo.py against the result",
    )
    args = parser.parse_args()

    if not args.source_dir.exists():
        print(f"ERROR: source directory does not exist: {args.source_dir}", file=sys.stderr)
        return 2

    wrapped = wrap_dataset(args.source_dir, args.output_dir)

    if args.launch:
        demo = Path(__file__).parent / "scrubber_demo.py"
        print(f"\nLaunching scrubber demo against {wrapped} ...")
        return subprocess.call([sys.executable, str(demo), "--session", str(wrapped)])
    else:
        print(f"\nNext step:")
        print(f"  python scripts/scrubber_demo.py --session {wrapped}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
