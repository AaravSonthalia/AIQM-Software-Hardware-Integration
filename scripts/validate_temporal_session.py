#!/usr/bin/env python3
"""Validate durability and temporal invariants of one Growth Monitor session."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
from typing import Optional


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _float(value: object) -> Optional[float]:
    try:
        parsed = float(str(value).strip())
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def _csv_rows(path: Path) -> tuple[list[str], list[dict[str, str]], list[str]]:
    errors: list[str] = []
    if not path.exists():
        return [], [], [f"missing file: {path.name}"]
    with path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        fields = list(reader.fieldnames or [])
        rows = list(reader)
    if any(None in row for row in rows):
        errors.append(f"{path.name}: malformed row with extra columns")
    return fields, rows, errors


def validate(session: Path) -> dict:
    errors: list[str] = []
    warnings: list[str] = []
    manifests: dict[str, str] = {}
    sensor_fields, sensor_rows, sensor_errors = _csv_rows(
        session / "sensor_log.csv",
    )
    errors.extend(sensor_errors)
    for row_index, row in enumerate(sensor_rows, start=2):
        for field in sensor_fields:
            if not (
                field.endswith("_age_ms")
                or field.endswith("_duration_ms")
                or field == "sync_span_ms"
            ):
                continue
            value = _float(row.get(field))
            if value is not None and value < 0:
                errors.append(
                    f"sensor_log.csv:{row_index}: negative {field}={value}",
                )
        for source in ("rheed", "pyrometer", "mistral", "evap"):
            valid = str(row.get(f"{source}_valid", "")).lower() == "true"
            sequence = str(row.get(f"{source}_sample_sequence", "")).strip()
            received = str(row.get(f"{source}_received_at_utc", "")).strip()
            if valid and (not sequence or not received):
                errors.append(
                    f"sensor_log.csv:{row_index}: {source} valid without provenance",
                )

    trace_path = session / "temporal_trace.jsonl"
    trace_count = 0
    if not trace_path.exists():
        errors.append("missing file: temporal_trace.jsonl")
    else:
        with trace_path.open(encoding="utf-8-sig") as stream:
            for line_number, line in enumerate(stream, start=1):
                if not line.strip():
                    continue
                try:
                    record = json.loads(line)
                except json.JSONDecodeError as exc:
                    errors.append(
                        f"temporal_trace.jsonl:{line_number}: {exc.msg}",
                    )
                    continue
                trace_count += 1
                if not isinstance(record, dict) or not record.get("event"):
                    errors.append(
                        f"temporal_trace.jsonl:{line_number}: invalid record",
                    )

    heartbeat_fields, heartbeat_rows, heartbeat_errors = _csv_rows(
        session / "heartbeat_log.csv",
    )
    errors.extend(heartbeat_errors)
    _ = heartbeat_fields
    listed = set()
    for row in heartbeat_rows:
        raw = str(row.get("frame_path", "")).strip()
        if not raw:
            continue
        path = Path(raw)
        listed.add((path if path.is_absolute() else session / path).resolve())
    heartbeat_images = set((session / "frames").glob("heartbeat_*.bmp"))
    orphaned = sorted(
        str(path) for path in heartbeat_images if path.resolve() not in listed
    )
    if orphaned:
        errors.extend(f"orphan heartbeat image: {path}" for path in orphaned)

    sequences: dict[tuple[str, str, str], int] = {}
    for index, row in enumerate(heartbeat_rows, start=2):
        try:
            sequence = int(row.get("capture_sequence", ""))
        except ValueError:
            errors.append(f"heartbeat_log.csv:{index}: invalid capture_sequence")
            continue
        key = (
            row.get("capture_backend", ""), row.get("source_hwnd", ""),
            row.get("captured_at_utc", ""),
        )
        previous = sequences.get(key)
        if previous is not None and previous != sequence:
            errors.append(
                f"heartbeat_log.csv:{index}: capture identity maps to two sequences",
            )
        sequences[key] = sequence

    for name in (
        "sensor_log.csv", "heartbeat_log.csv", "temporal_trace.jsonl",
        "operator_actions.jsonl",
    ):
        path = session / name
        if path.exists():
            manifests[name] = _sha256(path)
    if not sensor_rows:
        warnings.append("sensor_log.csv contains no data rows")
    if trace_count == 0:
        warnings.append("temporal_trace.jsonl contains no events")
    return {
        "schema_version": 1,
        "session": str(session.resolve()),
        "sensor_rows": len(sensor_rows),
        "heartbeat_rows": len(heartbeat_rows),
        "trace_records": trace_count,
        "errors": errors,
        "warnings": warnings,
        "sha256": manifests,
        "passed": not errors,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = validate(args.session)
    rendered = json.dumps(result, indent=2, sort_keys=True)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    print(rendered)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
