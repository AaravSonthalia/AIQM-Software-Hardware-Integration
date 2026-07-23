#!/usr/bin/env python3
"""Probe the kSA TEXT_CMD SQL query interface.

Run on Bulbasaur with kSA open and an active acquisition running (the
backing .kdt file must exist — no acquisition = SESSION_REQUIRED error
on every query).

What this discovers:
  1. Which SQL table names actually exist in the .kdt database.
  2. Whether the app query SQL path is enabled (requires "Enable external
     database tool" checkbox in kSA's Database Configuration tab).
  3. The response format for successful queries (column list, row encoding).
  4. Whether schema-discovery commands work (INFORMATION_SCHEMA / sqlite_master).

Usage:
    python scripts/probe_ksa_sql.py
    python scripts/probe_ksa_sql.py --host 127.0.0.1 --port 1800
    python scripts/probe_ksa_sql.py --quiet   # suppress verbose hex dumps

Prerequisites:
  - kSA 400 running on the same machine (or reachable at --host)
  - An active acquisition (Start button pressed, .kdt file created)
  - "Enable external database tool" checked in kSA Options → Database Configuration
"""
from __future__ import annotations

import argparse
import sys
import textwrap
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from drivers.ksa_comm import (  # noqa: E402
    KsaCommClient,
    KsaErrorCategory,
    classify_error,
    err_name,
)

# ---------------------------------------------------------------------------
# Probe catalog
# ---------------------------------------------------------------------------

# Phase 1: schema discovery — try to list all tables in one shot.
# SQL Server Compact and SQL Server both support INFORMATION_SCHEMA.
# SQLite uses sqlite_master. Try all three.
SCHEMA_QUERIES = [
    ("INFORMATION_SCHEMA (SQL Server / CE)",
     "SELECT TABLE_NAME FROM INFORMATION_SCHEMA.TABLES"),
    ("sys.objects (SQL Server full)",
     "SELECT name FROM sys.objects WHERE type='U'"),
    ("sqlite_master (SQLite)",
     "SELECT name FROM sqlite_master WHERE type='table'"),
    ("SELECT 1 sanity",
     "SELECT 1"),
]

# Phase 2: table name candidates for the kSA data stream.
# Ordered by decreasing likelihood based on the column list from
# ksa_400_datacolumns_2026-05-20.txt (Elapsed Time, Peak Intensity, etc.)
# and the ARHEED product naming conventions.
TABLE_CANDIDATES = [
    "DataPoints",     # most common time-series table name
    "Data",           # terse alternative
    "ARHEEDData",     # k-Space product name (ARHEED = advanced RHEED)
    "ARHEED",
    "DataTable",
    "LiveData",
    "RHEEDData",
    "kSAData",
    "MeasurementData",
    "Measurements",
    "Session",        # might be a metadata/header table
    "Channels",       # might be column-definition table
    "Intensities",
    "AnalysisData",
    "AcquisitionData",
    "Results",
]


def _run_query(client: KsaCommClient, sql: str, quiet: bool) -> tuple[int, str]:
    """Send `app query SQL "..."` and return (err, reply)."""
    cmd = f'app query SQL "{sql}"'
    err, reply = client.send_text(cmd)
    if not quiet:
        print(f"  CMD : {cmd}")
        print(f"  ERR : {err} ({err_name(err)})")
        print(f"  REPLY: {reply!r}")
    return err, reply


def run(host: str, port: int, quiet: bool) -> int:
    """Connect, probe, print results. Returns 0 on success."""
    print(f"Connecting to kSA at {host}:{port}...")
    client = KsaCommClient(host=host, port=port, timeout=5.0)
    try:
        client.connect()
    except ConnectionError as exc:
        print(f"ERROR: Connection failed — {exc}")
        print("  Is kSA running? Is the host/port correct?")
        return 1

    print(f"Connected. Protocol version: {client.protocol_version}")
    print()

    # Baseline: app list datacolumns (confirmed working Jul 7 2026)
    print("=== Baseline: app list datacolumns ===")
    err, reply = client.send_text("app list datacolumns")
    print(f"  err={err} ({err_name(err)}), reply length={len(reply)}")
    if err == 0:
        lines = reply.splitlines()
        print(f"  {len(lines)} columns returned (first 3: {lines[:3]})")
    else:
        cat = classify_error(reply)
        print(f"  category: {cat.name}")
        if cat == KsaErrorCategory.SESSION_REQUIRED:
            print("  *** No active acquisition — start one before running SQL probes ***")
            client.disconnect()
            return 1
    print()

    # Phase 1: schema discovery
    print("=== Phase 1: schema discovery ===")
    found_tables: list[str] = []
    for label, sql in SCHEMA_QUERIES:
        print(f"--- {label} ---")
        err, reply = _run_query(client, sql, quiet)
        if err == 0:
            print(f"  *** SUCCESS — schema query worked! Reply: {reply!r}")
            # Try to parse table names from the reply
            tables = [t.strip() for t in reply.splitlines() if t.strip()]
            found_tables.extend(tables)
        else:
            cat = classify_error(reply)
            print(f"  category: {cat.name}")
        print()

    if found_tables:
        print(f"Tables discovered via schema query: {found_tables}")
        print()

    # Phase 2: table name candidates (SELECT TOP 1 *)
    print("=== Phase 2: table name candidates ===")
    working_tables: list[str] = []
    for table in TABLE_CANDIDATES:
        sql = f"SELECT TOP 1 * FROM {table}"
        err, reply = _run_query(client, sql, quiet)
        if err == 0:
            print(f"  *** SUCCESS — table {table!r} exists!")
            working_tables.append(table)
        else:
            cat = classify_error(reply)
            if not quiet:
                print(f"  category: {cat.name}")
            else:
                # In quiet mode, only print failures when they're not the expected
                # SESSION_REQUIRED / NOT_IN_BUILD
                status = "OK" if err == 0 else f"err={err} ({cat.name})"
                print(f"  {table:<24} {status}")
        print()

    # Phase 3: if a working table was found, probe its column list
    if working_tables:
        print(f"=== Phase 3: column probe on {working_tables[0]} ===")
        sql = f"SELECT TOP 1 * FROM {working_tables[0]}"
        err, reply = _run_query(client, sql, quiet=False)
        print()

    # Phase 4: try LIMIT syntax (SQLite) if TOP didn't work on anything
    if not working_tables:
        print("=== Phase 4: retry with LIMIT (SQLite syntax) ===")
        for table in TABLE_CANDIDATES[:4]:
            sql = f"SELECT * FROM {table} LIMIT 1"
            err, reply = _run_query(client, sql, quiet)
            if err == 0:
                print(f"  *** SUCCESS with LIMIT syntax — table {table!r}!")
                working_tables.append(table)
            else:
                cat = classify_error(reply)
                if not quiet:
                    print(f"  category: {cat.name}")
            print()

    client.disconnect()

    print("=" * 60)
    if working_tables:
        print(f"RESULT: Working tables: {working_tables}")
        print("        SQL interface is functional — update task #190.")
    elif found_tables:
        print(f"RESULT: Schema discovered tables: {found_tables}")
        print("        Add them to TABLE_CANDIDATES and re-run.")
    else:
        print("RESULT: No working tables found.")
        print("        Likely causes:")
        print("          1. 'Enable external database tool' unchecked in kSA")
        print("             Options → Database Configuration tab")
        print("          2. .kdt file format is not SQL (e.g. binary custom format)")
        print("          3. app query SQL sub-command not implemented in this build")
        print()
        print("        Next steps:")
        print("          - Check the Database Configuration tab in kSA Options")
        print("          - Try ProcMon on kSA.exe to see what file it opens")
        print("          - Check if .kdt has a SQLite magic header:")
        print("            python -c \"f=open(r'path\\to\\file.kdt','rb');print(f.read(16))\"")
    return 0


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="127.0.0.1", help="kSA host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=1800, help="kSA port (default: 1800)")
    parser.add_argument("--quiet", action="store_true",
                        help="Suppress per-query CMD/ERR/REPLY lines; only print summaries")
    args = parser.parse_args()
    sys.exit(run(args.host, args.port, args.quiet))


if __name__ == "__main__":
    main()
