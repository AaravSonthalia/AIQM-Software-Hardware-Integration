#!/usr/bin/env python3
"""Unit tests for scripts/audit_session_sensor_log.py.

Covers:
  - Happy paths (Ch-MBE 7-cell, Bulbasaur 6-cell, non-ADS session)
  - Schema violations (missing ADS columns, missing historical T_C names)
  - Provenance violations (blank cell in ADS mode, populated cell above count)
  - Metadata edge cases (absent file, malformed JSON, missing keys)
  - CLI argument handling

No hardware, no GUI, stdlib only. Runs from Mac dev env.
"""
from __future__ import annotations

import csv
import json
import sys
import tempfile
import unittest
from io import StringIO
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

# Import after path setup
from scripts.audit_session_sensor_log import (  # noqa: E402
    ADS_CELL_SUFFIXES,
    ALL_ADS_CELL_COLUMNS,
    HISTORICAL_CELL_T_C_COLUMNS,
    audit_session,
    main,
)


# ---------------------------------------------------------------------------
# Fixtures — synthesize a session directory in a tempdir
# ---------------------------------------------------------------------------

def _write_metadata(session_dir: Path, meta: dict) -> None:
    (session_dir / "session_metadata.json").write_text(json.dumps(meta))


def _write_sensor_csv(
    session_dir: Path,
    populated_cells: list[int],
    include_all_ads_cols: bool = True,
    extra_columns: list[str] | None = None,
    row_count: int = 5,
) -> None:
    """Write a sensor_log.csv with the specified cells populated.

    populated_cells: list of cell indices (1-7) whose ADS fields are non-blank.
    include_all_ads_cols: if False, only cell1_T_C..cell7_T_C are in the header
                          (simulates pre-eccba75 schema).
    """
    cols = ["timestamp", "elapsed_s"]
    if extra_columns:
        cols.extend(extra_columns)
    if include_all_ads_cols:
        cols.extend(ALL_ADS_CELL_COLUMNS)
    else:
        cols.extend(HISTORICAL_CELL_T_C_COLUMNS)  # only 7 columns, not 56

    csv_path = session_dir / "sensor_log.csv"
    with open(csv_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for r in range(row_count):
            row = {c: "" for c in cols}
            row["timestamp"] = f"2026-07-28T12:00:0{r}"
            row["elapsed_s"] = f"{r:.2f}"
            for i in populated_cells:
                for sfx in ADS_CELL_SUFFIXES:
                    col = f"cell{i}_{sfx}"
                    if col in cols:
                        row[col] = f"{100.0 + i + r * 0.01:.3f}"
            w.writerow(row)


# ---------------------------------------------------------------------------
# Happy path tests
# ---------------------------------------------------------------------------

class HappyPathTests(unittest.TestCase):

    def test_chmbe_seven_cell_session_passes(self):
        with tempfile.TemporaryDirectory() as tmp:
            session = Path(tmp)
            _write_metadata(session, {
                "chamber_id": "chmbe",
                "mistral_mode": "ads",
                "mistral_ads_cell_count": 7,
                "mistral_ads_netid": "10.0.42.112.1.1",
            })
            _write_sensor_csv(session, populated_cells=list(range(1, 8)))
            self.assertEqual(audit_session(session), 0)

    def test_bulbasaur_six_cell_session_passes(self):
        with tempfile.TemporaryDirectory() as tmp:
            session = Path(tmp)
            _write_metadata(session, {
                "chamber_id": "ombe",
                "mistral_mode": "ads",
                "mistral_ads_cell_count": 6,
                "mistral_ads_netid": "10.0.42.111.1.1",
            })
            # Cells 1-6 populated, cell 7 blank (Bulbasaur)
            _write_sensor_csv(session, populated_cells=list(range(1, 7)))
            self.assertEqual(audit_session(session), 0)

    def test_non_ads_session_no_provenance_gate(self):
        # Screengrab mode: no cell columns populated — must still pass
        # schema check and not fail on blanks.
        with tempfile.TemporaryDirectory() as tmp:
            session = Path(tmp)
            _write_metadata(session, {
                "chamber_id": "ombe",
                "mistral_mode": "screengrab",
            })
            _write_sensor_csv(session, populated_cells=[])
            self.assertEqual(audit_session(session), 0)

    def test_session_without_metadata_passes_schema_only(self):
        # No session_metadata.json — audit should still run and just skip
        # provenance checks.
        with tempfile.TemporaryDirectory() as tmp:
            session = Path(tmp)
            # No metadata file at all
            _write_sensor_csv(session, populated_cells=[])
            self.assertEqual(audit_session(session), 0)


# ---------------------------------------------------------------------------
# Schema violation tests
# ---------------------------------------------------------------------------

class SchemaViolationTests(unittest.TestCase):

    def test_pre_eccba75_session_fails_schema(self):
        # Session written with old 7-column schema (cell1_T_C..cell7_T_C
        # only) should fail — 49 of 56 ADS columns missing.
        with tempfile.TemporaryDirectory() as tmp:
            session = Path(tmp)
            _write_metadata(session, {
                "chamber_id": "chmbe",
                "mistral_mode": "ads",
                "mistral_ads_cell_count": 7,
            })
            _write_sensor_csv(
                session, populated_cells=list(range(1, 8)),
                include_all_ads_cols=False,
            )
            self.assertEqual(audit_session(session), 1)

    def test_missing_historical_cell_T_C_flagged(self):
        # Craft a CSV that has the new suffixes but somehow lost cell{i}_T_C.
        # Would indicate a rename regression.
        with tempfile.TemporaryDirectory() as tmp:
            session = Path(tmp)
            _write_metadata(session, {"mistral_mode": "screengrab"})
            csv_path = session / "sensor_log.csv"
            # Header has everything EXCEPT cell{i}_T_C
            cols = ["timestamp", "elapsed_s"]
            cols.extend(c for c in ALL_ADS_CELL_COLUMNS if not c.endswith("_T_C"))
            with open(csv_path, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=cols)
                w.writeheader()
            self.assertEqual(audit_session(session), 1)


# ---------------------------------------------------------------------------
# Provenance violation tests
# ---------------------------------------------------------------------------

class ProvenanceViolationTests(unittest.TestCase):

    def test_ads_mode_with_blank_cell_below_count_fails(self):
        # Ch-MBE session claims 7 cells but cell 4 has no data.
        with tempfile.TemporaryDirectory() as tmp:
            session = Path(tmp)
            _write_metadata(session, {
                "chamber_id": "chmbe",
                "mistral_mode": "ads",
                "mistral_ads_cell_count": 7,
            })
            _write_sensor_csv(
                session, populated_cells=[1, 2, 3, 5, 6, 7],  # cell4 missing
            )
            self.assertEqual(audit_session(session), 1)

    def test_ads_mode_with_populated_cell_above_count_fails(self):
        # Bulbasaur session (count=6) but cell7 has data — indicates the
        # driver read a cell that shouldn't exist for this chamber.
        with tempfile.TemporaryDirectory() as tmp:
            session = Path(tmp)
            _write_metadata(session, {
                "chamber_id": "ombe",
                "mistral_mode": "ads",
                "mistral_ads_cell_count": 6,
            })
            _write_sensor_csv(
                session, populated_cells=list(range(1, 8)),  # all 7 populated
            )
            self.assertEqual(audit_session(session), 1)

    def test_ads_mode_with_invalid_cell_count_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            session = Path(tmp)
            _write_metadata(session, {
                "chamber_id": "ombe",
                "mistral_mode": "ads",
                "mistral_ads_cell_count": "not-an-int",
            })
            _write_sensor_csv(session, populated_cells=list(range(1, 7)))
            self.assertEqual(audit_session(session), 1)

    def test_ads_mode_missing_cell_count_fails(self):
        # ads mode but no mistral_ads_cell_count key at all.
        with tempfile.TemporaryDirectory() as tmp:
            session = Path(tmp)
            _write_metadata(session, {
                "chamber_id": "chmbe",
                "mistral_mode": "ads",
                # no mistral_ads_cell_count
            })
            _write_sensor_csv(session, populated_cells=list(range(1, 8)))
            self.assertEqual(audit_session(session), 1)


# ---------------------------------------------------------------------------
# Metadata edge cases
# ---------------------------------------------------------------------------

class MetadataEdgeCaseTests(unittest.TestCase):

    def test_malformed_metadata_json_warns_but_doesnt_crash(self):
        with tempfile.TemporaryDirectory() as tmp:
            session = Path(tmp)
            (session / "session_metadata.json").write_text("{ not valid json")
            _write_sensor_csv(session, populated_cells=[])
            # Should treat as if no metadata → schema pass, no provenance check
            self.assertEqual(audit_session(session), 0)


# ---------------------------------------------------------------------------
# CLI tests
# ---------------------------------------------------------------------------

class CLITests(unittest.TestCase):

    def test_missing_session_dir_returns_1(self):
        self.assertEqual(main(["/nonexistent/path/xyz"]), 1)

    def test_session_dir_is_file_returns_1(self):
        with tempfile.NamedTemporaryFile(suffix=".txt") as tmp:
            self.assertEqual(main([tmp.name]), 1)

    def test_session_dir_without_sensor_log_returns_1(self):
        with tempfile.TemporaryDirectory() as tmp:
            self.assertEqual(main([tmp]), 1)


if __name__ == "__main__":
    print("Running audit_session_sensor_log tests...\n")
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(unittest.TestLoader().discover(
        start_dir=str(Path(__file__).parent),
        pattern="test_audit_session_sensor_log.py",
    ))
    sys.exit(0 if result.wasSuccessful() else 1)
