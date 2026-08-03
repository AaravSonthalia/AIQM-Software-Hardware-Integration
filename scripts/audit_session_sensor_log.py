#!/usr/bin/env python3
"""Audit a growth session's ``sensor_log.csv`` for ADS schema + provenance.

Purpose: on the next lab day, run this against any completed session on
either chamber to confirm the Jul 27 2026 ADS union schema landed and
the per-chamber provenance is consistent.

Checks performed:

1. **Schema**: all 56 ADS union columns exist in the CSV header
   (8 suffixes × 7 cells: T_C, T_set_C, active_setpoint_C, V, I,
   prog_V, prog_A, power_W).
2. **Historical**: `cell{i}_T_C` names preserved verbatim (regression
   guard for the ``eccba75`` schema expansion — downstream analysis
   depends on these names).
3. **Provenance** (only when ``session_metadata.json`` says
   ``mistral_mode == "ads"``):
   - Cells 1..``mistral_ads_cell_count`` must have at least one
     populated ADS value in the session.
   - Cells above ``mistral_ads_cell_count`` must be blank throughout
     (e.g. Bulbasaur sessions with count=6 must have cell7 fully blank).
4. **Per-column report**: population % and first non-blank value for
   every ADS cell column, regardless of mode.

Non-ADS sessions (screengrab / jsonrpc / dummy / no metadata) get the
schema report only — blank cells are expected and not flagged.

Exit code: 0 on pass, 1 on schema/provenance violation.

Usage:
    python scripts/audit_session_sensor_log.py path/to/session_dir
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path
from typing import Optional


# ---------------------------------------------------------------------------
# ADS union schema (matches gui/growth_logger.py SENSOR_FIELDS)
# ---------------------------------------------------------------------------

ADS_CELL_SUFFIXES: tuple[str, ...] = (
    "T_C", "T_set_C", "active_setpoint_C",
    "V", "I", "prog_V", "prog_A", "power_W",
)

CELL_INDICES: tuple[int, ...] = tuple(range(1, 8))  # 7-cell union

ALL_ADS_CELL_COLUMNS: list[str] = [
    f"cell{i}_{sfx}" for i in CELL_INDICES for sfx in ADS_CELL_SUFFIXES
]

HISTORICAL_CELL_T_C_COLUMNS: list[str] = [
    f"cell{i}_T_C" for i in CELL_INDICES
]


# ---------------------------------------------------------------------------
# Data model
# ---------------------------------------------------------------------------

class ColumnStats:
    """Population count + first non-blank value for one column."""

    __slots__ = ("count", "first")

    def __init__(self) -> None:
        self.count: int = 0
        self.first: Optional[str] = None

    def observe(self, val: str) -> None:
        if val == "":
            return
        self.count += 1
        if self.first is None:
            self.first = val


def _collect_stats(
    rows: list[dict[str, str]],
    columns: list[str],
) -> dict[str, ColumnStats]:
    """Single-pass accumulation of per-column stats."""
    stats = {c: ColumnStats() for c in columns}
    for row in rows:
        for col in columns:
            stats[col].observe(row.get(col, ""))
    return stats


# ---------------------------------------------------------------------------
# Metadata loading
# ---------------------------------------------------------------------------

def _load_metadata(session_dir: Path) -> dict:
    """Return session_metadata.json contents, or {} if absent/unparseable."""
    meta_path = session_dir / "session_metadata.json"
    if not meta_path.exists():
        return {}
    try:
        with open(meta_path) as f:
            return json.load(f)
    except (json.JSONDecodeError, OSError) as exc:
        print(f"WARN: could not parse session_metadata.json: {exc}")
        return {}


# ---------------------------------------------------------------------------
# Audit
# ---------------------------------------------------------------------------

def audit_session(session_dir: Path) -> int:
    """Run the full audit; return exit code (0 pass, 1 fail)."""
    csv_path = session_dir / "sensor_log.csv"
    if not csv_path.exists():
        print(f"FAIL: sensor_log.csv not found in {session_dir}")
        return 1

    metadata = _load_metadata(session_dir)
    chamber_id = metadata.get("chamber_id", "unknown")
    mistral_mode = metadata.get("mistral_mode", "unknown")
    ads_cell_count = metadata.get("mistral_ads_cell_count", None)
    ads_netid = metadata.get("mistral_ads_netid", None)

    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        header: list[str] = list(reader.fieldnames or [])
        rows: list[dict[str, str]] = list(reader)

    print("=== Session sensor-log audit ===")
    print(f"Session dir:   {session_dir}")
    print(f"Chamber:       {chamber_id!r}")
    print(f"MISTRAL mode:  {mistral_mode!r}")
    if mistral_mode == "ads":
        print(f"ADS netId:     {ads_netid!r}")
        print(f"ADS cells:     {ads_cell_count!r}")
    print(f"Header cols:   {len(header)}")
    print(f"Data rows:     {len(rows)}")
    print()

    failures: list[str] = []

    # --- 1. Schema check ---------------------------------------------------
    missing_ads = [c for c in ALL_ADS_CELL_COLUMNS if c not in header]
    if missing_ads:
        failures.append(
            f"schema: {len(missing_ads)}/56 ADS cell columns missing"
        )
        print(f"SCHEMA:        [FAIL] {len(missing_ads)}/56 ADS columns missing")
        for col in missing_ads[:8]:
            print(f"                 - {col}")
        if len(missing_ads) > 8:
            print(f"                 ... +{len(missing_ads) - 8} more")
    else:
        print(f"SCHEMA:        [PASS] all 56 ADS cell columns present")

    # --- 2. Historical name preservation ----------------------------------
    missing_hist = [c for c in HISTORICAL_CELL_T_C_COLUMNS if c not in header]
    if missing_hist:
        failures.append(
            f"historical: {missing_hist} missing — analysis code may break"
        )
        print(f"HISTORICAL:    [FAIL] missing: {missing_hist}")
    else:
        print(f"HISTORICAL:    [PASS] cell1_T_C..cell7_T_C names preserved")

    # --- 3. Provenance (ADS mode only) ------------------------------------
    # Only iterate rows for stats if the schema is intact enough to have
    # the columns we're auditing.
    stats = _collect_stats(rows, [c for c in ALL_ADS_CELL_COLUMNS if c in header])

    if mistral_mode == "ads":
        if not isinstance(ads_cell_count, int) or not (1 <= ads_cell_count <= 7):
            failures.append(
                f"provenance: mistral_ads_cell_count invalid or missing "
                f"({ads_cell_count!r})"
            )
            print(f"PROVENANCE:    [FAIL] mistral_ads_cell_count invalid: "
                  f"{ads_cell_count!r}")
        else:
            print(f"PROVENANCE:    (mode=ads, expect cells 1-{ads_cell_count} "
                  f"populated, cells {ads_cell_count+1}-7 blank)")
            for i in CELL_INDICES:
                cell_has_data = any(
                    stats.get(f"cell{i}_{sfx}", ColumnStats()).count > 0
                    for sfx in ADS_CELL_SUFFIXES
                )
                if i <= ads_cell_count:
                    if cell_has_data:
                        print(f"                 [PASS] cell{i} populated")
                    else:
                        failures.append(
                            f"provenance: cell{i} completely blank "
                            f"despite mode=ads and cell_count={ads_cell_count}"
                        )
                        print(f"                 [FAIL] cell{i} blank "
                              f"(expected populated)")
                else:
                    if cell_has_data:
                        failures.append(
                            f"provenance: cell{i} populated but is above "
                            f"mistral_ads_cell_count={ads_cell_count}"
                        )
                        print(f"                 [FAIL] cell{i} populated "
                              f"(expected blank on this chamber)")
                    else:
                        print(f"                 [PASS] cell{i} blank")
    else:
        print(f"PROVENANCE:    skipped (mistral_mode={mistral_mode!r}, "
              f"not 'ads'; blank cells are expected)")

    # --- 4. Per-column summary --------------------------------------------
    print()
    print("PER-COLUMN SUMMARY (ADS cell columns):")
    total_rows = len(rows)
    for col in ALL_ADS_CELL_COLUMNS:
        if col not in header:
            print(f"  {col:<32} (missing from header)")
            continue
        st = stats[col]
        pct = (st.count / total_rows * 100.0) if total_rows else 0.0
        if st.first is not None:
            detail = f"first={st.first}"
        else:
            detail = "all blank"
        print(f"  {col:<32} {pct:>5.1f}%  {detail}")

    # --- Verdict ----------------------------------------------------------
    print()
    if failures:
        print(f"VERDICT: FAIL ({len(failures)} issue(s))")
        for f in failures:
            print(f"  - {f}")
        return 1
    print("VERDICT: PASS")
    return 0


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Audit a growth session's sensor_log.csv for ADS union schema "
            "compliance (Jul 27 2026 / commit eccba75) and per-chamber "
            "MISTRAL provenance."
        ),
    )
    parser.add_argument(
        "session_dir", type=Path,
        help="Path to the session directory containing sensor_log.csv "
             "and (optionally) session_metadata.json",
    )
    args = parser.parse_args(argv)

    if not args.session_dir.exists():
        print(f"FAIL: {args.session_dir} does not exist")
        return 1
    if not args.session_dir.is_dir():
        print(f"FAIL: {args.session_dir} is not a directory")
        return 1

    return audit_session(args.session_dir)


if __name__ == "__main__":
    sys.exit(main())
