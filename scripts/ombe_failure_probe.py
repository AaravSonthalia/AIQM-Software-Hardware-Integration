#!/usr/bin/env python3
"""Read-only O-MBE worker failure/recovery recorder.

The probe never controls an instrument or closes a vendor application. Run it
while an operator covers, minimizes, closes, or restores the selected software
window. A separate ``mark`` invocation timestamps each manual failure and
recovery action in the same run directory.

For WGC validation, run this recorder from its failure-validation checkout and
point ``--software-root`` at the WGC checkout. The evidence then records both
Git commits and the actual worker/driver/backend used.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import platform
import statistics
import subprocess
import sys
import time
from dataclasses import dataclass, fields, is_dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable

RECORDER_ROOT = Path(__file__).resolve().parent.parent
if str(RECORDER_ROOT) not in sys.path:
    sys.path.insert(0, str(RECORDER_ROOT))

VALID_MODES: dict[str, tuple[str, ...]] = {
    "camera": ("screengrab", "screengrab_mss"),
    "pyrometer": ("screengrab", "modbus", "exactus"),
    "mistral": ("screengrab", "jsonrpc"),
    "evap": ("screengrab", "elog"),
}

EXPECTED_DRIVER_CLASSES: dict[tuple[str, str], str] = {
    ("camera", "screengrab"): "ScreenGrabCamera",
    ("camera", "screengrab_mss"): "ScreenGrabCamera",
    ("pyrometer", "screengrab"): "ScreenGrabPyrometer",
    ("pyrometer", "modbus"): "ModbusPyrometer",
    ("pyrometer", "exactus"): "ExactusSerialPyrometer",
    ("mistral", "screengrab"): "MistralGui",
    ("mistral", "jsonrpc"): "MistralJsonRpcClient",
    ("evap", "screengrab"): "EvapControl",
    ("evap", "elog"): "ElogReader",
}

MEASUREMENT_KEYS: dict[str, tuple[str, ...]] = {
    "camera": ("frame_number", "capture_sequence"),
    "pyrometer": (
        "temperature",
        "temperature_std",
        "temperature_n",
        "emissivity",
    ),
    "mistral": ("v_set", "v_actual", "i_set", "i_actual"),
    "evap": (
        "chamber_pressure_mbar",
        "substrate_temp_pv_C",
        "substrate_temp_setpoint_C",
        "cell_HTEC2_pv_C",
        "cell_Y_pv_C",
        "cell_Sr_pv_C",
        "cell_Eu_pv_C",
        "cell_Er_pv_C",
        "plasma_dc_bias_V",
        "plasma_forward_W",
        "plasma_reflected_W",
    ),
}

DRIVER_ATTRS = {
    "camera": "_camera",
    "pyrometer": "_sensor",
    "mistral": "_driver",
    "evap": "_driver",
}


def utc_now() -> str:
    return (
        datetime.now(timezone.utc)
        .isoformat(timespec="milliseconds")
        .replace("+00:00", "Z")
    )


def _parse_utc(value: Any) -> datetime | None:
    if not isinstance(value, str) or not value.strip():
        return None
    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError:
        return None
    if parsed.tzinfo is None:
        parsed = parsed.replace(tzinfo=timezone.utc)
    return parsed.astimezone(timezone.utc)


def _json_value(value: Any) -> Any:
    if value is None or isinstance(value, (str, int, bool)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {str(k): _json_value(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_value(v) for v in value]
    # Camera pixels are intentionally not serialized. Avoid dataclasses.asdict,
    # which would deep-copy a full-resolution numpy frame on every emission.
    if hasattr(value, "shape") and hasattr(value, "dtype"):
        return {
            "shape": list(value.shape),
            "dtype": str(value.dtype),
        }
    return str(value)


def state_payload(state: Any) -> dict[str, Any]:
    """Convert a worker state into stable, JSON-safe diagnostic fields."""
    if is_dataclass(state):
        raw = {field.name: getattr(state, field.name) for field in fields(state)}
    else:
        raw = vars(state)
    return {key: _json_value(value) for key, value in raw.items()}


def measurement_signature(source: str, payload: dict[str, Any]) -> tuple[Any, ...]:
    """Values whose unexpected retention can indicate stale-data behavior."""
    if source == "camera":
        capture_sequence = payload.get("capture_sequence")
        if capture_sequence not in (None, 0):
            return ("capture_sequence", capture_sequence)
        return ("frame_number", payload.get("frame_number"))
    return tuple(payload.get(key) for key in MEASUREMENT_KEYS[source])


def measurement_status(
    source: str,
    payload: dict[str, Any],
) -> tuple[bool, str, bool]:
    """Return ``(valid, reason, all_measurements_none)`` for one emission."""
    values = [payload.get(key) for key in MEASUREMENT_KEYS[source]]
    all_none = all(value is None for value in values)
    if payload.get("connected") is not True:
        return False, "disconnected", all_none
    if bool(payload.get("error")):
        return False, "worker_error", all_none
    if source == "camera" and payload.get("frame") is None:
        return False, "missing_frame", all_none
    if all_none:
        return False, "all_measurements_none", True
    return True, "valid", all_none


def data_age(
    payload: dict[str, Any],
    observed_at_utc: str,
    observed_monotonic_ns: int,
) -> tuple[float | None, str]:
    """Calculate data age from provenance fields when a State exposes them."""
    observed_dt = _parse_utc(observed_at_utc)
    for field_name in ("source_at_utc", "captured_at_utc", "received_at_utc"):
        source_dt = _parse_utc(payload.get(field_name))
        if source_dt is not None and observed_dt is not None:
            return (
                max(0.0, (observed_dt - source_dt).total_seconds() * 1000.0),
                field_name,
            )

    for field_name in ("received_monotonic_ns", "captured_monotonic_ns"):
        value = payload.get(field_name)
        if isinstance(value, (int, float)) and value > 0:
            return (
                max(0.0, (observed_monotonic_ns - int(value)) / 1_000_000.0),
                field_name,
            )

    frame_age = payload.get("frame_age_ms")
    if isinstance(frame_age, (int, float)) and math.isfinite(float(frame_age)):
        return max(0.0, float(frame_age)), "frame_age_ms"
    return None, "unavailable"


@dataclass
class ObservationTracker:
    previous_signature: tuple[Any, ...] | None = None
    last_changed_monotonic_s: float | None = None


def annotate_observation(
    row: dict[str, Any],
    source: str,
    tracker: ObservationTracker,
) -> dict[str, Any]:
    """Attach change, validity, and freshness diagnostics to one row."""
    now_s = float(row["observed_monotonic_s"])
    payload = row["state"]
    signature = measurement_signature(source, payload)
    first = tracker.previous_signature is None
    changed = first or signature != tracker.previous_signature
    if changed:
        tracker.previous_signature = signature
        tracker.last_changed_monotonic_s = now_s
    unchanged_for_s = (
        0.0
        if tracker.last_changed_monotonic_s is None
        else max(0.0, now_s - tracker.last_changed_monotonic_s)
    )
    valid, reason, all_none = measurement_status(source, payload)
    age_ms, age_source = data_age(
        payload,
        str(row.get("observed_at_utc", "")),
        int(row.get("observed_monotonic_ns", round(now_s * 1_000_000_000))),
    )
    row.update({
        "value_changed": None if first else changed,
        "unchanged_for_s": unchanged_for_s,
        "measurement_valid": valid,
        "validity_reason": reason,
        "all_measurements_none": all_none,
        "data_age_ms": age_ms,
        "data_age_source": age_source,
    })
    return row


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    return float(ordered[round((len(ordered) - 1) * fraction)])


def _ensure_annotations(
    rows: list[dict[str, Any]],
    source: str,
) -> None:
    tracker = ObservationTracker()
    first_time = (
        float(rows[0]["observed_monotonic_s"]) if rows else 0.0
    )
    for row in rows:
        row.setdefault(
            "elapsed_s",
            float(row["observed_monotonic_s"]) - first_time,
        )
        if "measurement_valid" not in row:
            annotate_observation(row, source, tracker)


def _baseline_intervals(
    rows: list[dict[str, Any]],
    markers: list[dict[str, Any]],
    baseline_duration_s: float,
) -> list[float]:
    failure_starts = [
        float(marker["marked_monotonic_s"])
        for marker in markers
        if marker.get("kind") == "failure-start"
    ]
    first_failure = min(failure_starts) if failure_starts else math.inf

    intervals: list[float] = []
    for previous, current in zip(rows, rows[1:]):
        pair_is_baseline = (
            float(current.get("elapsed_s", math.inf)) <= baseline_duration_s
            and float(current["observed_monotonic_s"]) < first_failure
            and previous.get("measurement_valid") is True
            and current.get("measurement_valid") is True
            and not bool(previous["state"].get("error"))
            and not bool(current["state"].get("error"))
        )
        if pair_is_baseline:
            intervals.append(
                float(current["observed_monotonic_s"])
                - float(previous["observed_monotonic_s"])
            )
    return intervals


def _failure_episodes(
    rows: list[dict[str, Any]],
    markers: list[dict[str, Any]],
    stale_threshold_s: float,
) -> tuple[list[dict[str, Any]], list[int]]:
    ordered_markers = sorted(
        markers,
        key=lambda marker: float(marker.get("marked_monotonic_s", 0.0)),
    )
    starts = [
        marker for marker in ordered_markers
        if marker.get("kind") == "failure-start"
    ]
    recoveries = [
        marker for marker in ordered_markers
        if marker.get("kind") == "recovery"
    ]

    episodes: list[dict[str, Any]] = []
    stale_indices: list[int] = []
    for start in starts:
        start_s = float(start["marked_monotonic_s"])
        recovery = next(
            (
                marker for marker in recoveries
                if float(marker["marked_monotonic_s"]) > start_s
            ),
            None,
        )
        end_s = (
            float(recovery["marked_monotonic_s"])
            if recovery is not None
            else math.inf
        )
        episode_rows = [
            row for row in rows
            if start_s <= float(row["observed_monotonic_s"]) < end_s
        ]
        first_invalid = next(
            (
                row for row in episode_rows
                if row.get("measurement_valid") is False
            ),
            None,
        )
        stale_rows = [
            row for row in episode_rows
            if row.get("measurement_valid") is True
            and (
                float(row["observed_monotonic_s"]) - start_s
                > stale_threshold_s
            )
            and float(row.get("unchanged_for_s", 0.0)) > stale_threshold_s
        ]
        stale_indices.extend(
            int(row["emission_index"]) for row in stale_rows
        )
        episodes.append({
            "label": start.get("label", ""),
            "note": start.get("note", ""),
            "failure_started_at_utc": start.get("marked_at_utc"),
            "recovery_at_utc": (
                recovery.get("marked_at_utc") if recovery else None
            ),
            "observed_emissions": len(episode_rows),
            "first_invalid_emission_index": (
                int(first_invalid["emission_index"])
                if first_invalid is not None else None
            ),
            "invalid_detection_latency_s": (
                float(first_invalid["observed_monotonic_s"]) - start_s
                if first_invalid is not None else None
            ),
            "stale_valid_emission_indices": [
                int(row["emission_index"]) for row in stale_rows
            ],
        })
    return episodes, sorted(set(stale_indices))


def summarize(
    rows: list[dict[str, Any]],
    source: str,
    *,
    markers: list[dict[str, Any]] | None = None,
    baseline_p95_s: float | None = None,
    baseline_duration_s: float = 30.0,
) -> dict[str, Any]:
    """Summarize validity, emission cadence, and marker-aligned failures."""
    markers = markers or []
    _ensure_annotations(rows, source)
    intervals = [
        float(current["observed_monotonic_s"])
        - float(previous["observed_monotonic_s"])
        for previous, current in zip(rows, rows[1:])
    ]
    measured_baseline = _baseline_intervals(
        rows, markers, baseline_duration_s,
    )
    if baseline_p95_s is None:
        p95 = percentile(measured_baseline, 0.95)
        p95_source = "healthy_pre_failure_window"
    else:
        p95 = float(baseline_p95_s)
        p95_source = "supplied"
    stale_threshold = max(2.0 * p95, 3.0) if p95 is not None else 3.0
    episodes, stale_indices = _failure_episodes(
        rows, markers, stale_threshold,
    )

    return {
        "source": source,
        "emissions": len(rows),
        "connected_emissions": sum(
            row["state"].get("connected") is True for row in rows
        ),
        "error_emissions": sum(
            bool(row["state"].get("error")) for row in rows
        ),
        "valid_emissions": sum(
            row.get("measurement_valid") is True for row in rows
        ),
        "invalid_emissions": sum(
            row.get("measurement_valid") is False for row in rows
        ),
        "connected_but_invalid_emissions": sum(
            row["state"].get("connected") is True
            and row.get("measurement_valid") is False
            for row in rows
        ),
        "all_measurements_none_emissions": sum(
            row.get("all_measurements_none") is True for row in rows
        ),
        "value_change_emissions": sum(
            row.get("value_changed") is True for row in rows
        ),
        "data_age_available_emissions": sum(
            row.get("data_age_ms") is not None for row in rows
        ),
        "interval_s": {
            "median": statistics.median(intervals) if intervals else None,
            "p95": percentile(intervals, 0.95),
            "p99": percentile(intervals, 0.99),
            "max": max(intervals) if intervals else None,
        },
        "baseline_interval_s": {
            "source": p95_source,
            "duration_s": baseline_duration_s,
            "samples": len(measured_baseline),
            "p95": p95,
        },
        "stale_threshold_s": stale_threshold,
        "failure_marker_count": len(episodes),
        "failure_episodes": episodes,
        "stale_candidate_emission_indices": stale_indices,
        "stale_candidate_count": len(stale_indices),
        "stale_interpretation": (
            "Candidates are valid, unchanged values observed only inside an "
            "operator-marked failure window; constant values outside such a "
            "window are not labeled stale."
        ),
    }


def _git_commit(root: Path) -> str:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"],
            cwd=root,
            text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
    except Exception:
        return ""


def _git_dirty(root: Path) -> bool | None:
    try:
        output = subprocess.check_output(
            ["git", "status", "--porcelain"],
            cwd=root,
            text=True,
            stderr=subprocess.DEVNULL,
        )
        return bool(output.strip())
    except Exception:
        return None


def validate_source_mode(source: str, mode: str) -> None:
    allowed = VALID_MODES.get(source, ())
    if mode not in allowed:
        choices = ", ".join(allowed)
        raise ValueError(
            f"mode {mode!r} is invalid for {source}; choose one of: {choices}"
        )


def _configure_software_root(root: Path) -> Path:
    root = root.expanduser().resolve()
    workers_path = root / "gui" / "workers.py"
    if not workers_path.is_file():
        raise ValueError(
            f"--software-root does not contain gui/workers.py: {root}"
        )
    loaded_gui = sys.modules.get("gui")
    loaded_path = Path(getattr(loaded_gui, "__file__", "") or ".").resolve()
    if loaded_gui is not None and root not in loaded_path.parents:
        raise RuntimeError(
            "gui was already imported from a different checkout; run the "
            "probe in a fresh Python process"
        )
    root_text = str(root)
    if root_text in sys.path:
        sys.path.remove(root_text)
    sys.path.insert(0, root_text)
    return root


def _worker_factory(
    source: str,
    mode: str,
    interval: float,
    software_root: Path,
):
    _configure_software_root(software_root)
    from gui.workers import (
        EvapControlWorker,
        MistralWorker,
        PyrometerWorker,
        RheedCameraWorker,
    )

    factories: dict[str, Callable[[], Any]] = {
        "camera": lambda: RheedCameraWorker(
            mode=mode, poll_interval=interval,
        ),
        "pyrometer": lambda: PyrometerWorker(
            mode=mode, poll_interval=interval,
        ),
        "mistral": lambda: MistralWorker(
            mode=mode, poll_interval=interval,
        ),
        "evap": lambda: EvapControlWorker(
            mode=mode, poll_interval=interval,
        ),
    }
    return factories[source]()


def driver_identity(worker: Any, source: str) -> dict[str, Any]:
    """Inspect already-created worker objects without initiating new reads."""
    driver = getattr(worker, DRIVER_ATTRS[source], None)
    identity: dict[str, Any] = {
        "worker_class": type(worker).__name__,
        "driver_class": type(driver).__name__ if driver is not None else None,
        "driver_module": type(driver).__module__ if driver is not None else None,
    }
    if driver is not None:
        backend = getattr(driver, "_backend", None)
        if backend is not None:
            identity["driver_backend"] = str(backend)
        session = getattr(driver, "_capture_session", None)
        if session is not None:
            identity["capture_session_class"] = type(session).__name__
            identity["capture_session_module"] = type(session).__module__
    return identity


def _write_json_atomic(path: Path, value: Any) -> None:
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def _append_jsonl(stream: Any, row: dict[str, Any]) -> None:
    stream.write(json.dumps(row, separators=(",", ":")) + "\n")
    stream.flush()
    os.fsync(stream.fileno())


def prepare_output_dir(path: Path) -> Path:
    """Create a new evidence directory; never overwrite or merge old runs."""
    path = path.expanduser().resolve()
    if path.exists():
        raise FileExistsError(
            f"output directory already exists; choose a new run directory: {path}"
        )
    path.mkdir(parents=True, exist_ok=False)
    return path


def _load_markers(path: Path) -> tuple[list[dict[str, Any]], list[str]]:
    if not path.exists():
        return [], []
    markers: list[dict[str, Any]] = []
    errors: list[str] = []
    with path.open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, start=1):
            try:
                markers.append(json.loads(line))
            except json.JSONDecodeError as exc:
                errors.append(f"markers.jsonl:{line_number}: {exc}")
    return markers, errors


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def write_sha_manifest(output_dir: Path) -> dict[str, str]:
    names = (
        "run_info.json",
        "states.jsonl",
        "markers.jsonl",
        "summary.json",
    )
    manifest = {
        name: _sha256(output_dir / name)
        for name in names
        if (output_dir / name).is_file()
    }
    _write_json_atomic(output_dir / "sha256_manifest.json", manifest)
    return manifest


def run_probe(args: argparse.Namespace) -> int:
    from PyQt6.QtCore import QCoreApplication, QTimer

    validate_source_mode(args.source, args.mode)
    software_root = args.software_root.expanduser().resolve()
    app = QCoreApplication.instance() or QCoreApplication([])
    worker = _worker_factory(
        args.source,
        args.mode,
        args.poll_interval,
        software_root,
    )
    output_dir = prepare_output_dir(args.output_dir)
    rows: list[dict[str, Any]] = []
    tracker = ObservationTracker()
    fatal_errors: list[str] = []
    state_path = output_dir / "states.jsonl"
    started_monotonic = time.monotonic()
    started_monotonic_ns = time.monotonic_ns()
    run_info: dict[str, Any] = {
        "started_at_utc": utc_now(),
        "recorder_git_commit": _git_commit(RECORDER_ROOT),
        "recorder_git_dirty": _git_dirty(RECORDER_ROOT),
        "recorder_root": str(RECORDER_ROOT),
        "software_git_commit": _git_commit(software_root),
        "software_git_dirty": _git_dirty(software_root),
        "software_root": str(software_root),
        "source": args.source,
        "mode": args.mode,
        "expected_capture_backend": args.expected_capture_backend,
        "duration_s_requested": args.duration_s,
        "poll_interval_s": args.poll_interval,
        "baseline_duration_s": args.baseline_duration_s,
        "baseline_p95_s_supplied": args.baseline_p95_s,
        "platform": platform.platform(),
        "python": sys.version,
        "command": " ".join([sys.executable, *sys.argv]),
        "operator_actions_are_manual": True,
        "read_only_probe": True,
        "worker_identity": driver_identity(worker, args.source),
    }
    _write_json_atomic(output_dir / "run_info.json", run_info)

    timer = QTimer()
    timer.setSingleShot(True)
    lifecycle = {
        "finishing": False,
        "finish_reason": "",
        "stop_timed_out": False,
        "worker_finished_unexpectedly": False,
        "backend_verified": args.expected_capture_backend is None,
    }

    def refresh_identity() -> None:
        identity = driver_identity(worker, args.source)
        if identity != run_info.get("worker_identity"):
            run_info["worker_identity"] = identity
            _write_json_atomic(output_dir / "run_info.json", run_info)
        actual_class = identity.get("driver_class")
        expected_class = EXPECTED_DRIVER_CLASSES[(args.source, args.mode)]
        if actual_class is not None and actual_class != expected_class:
            message = (
                f"mode {args.mode!r} created {actual_class}, expected "
                f"{expected_class}; refusing dummy/fallback validation"
            )
            if message not in fatal_errors:
                fatal_errors.append(message)

    def finish(reason: str = "duration") -> None:
        if lifecycle["finishing"]:
            return
        lifecycle["finishing"] = True
        lifecycle["finish_reason"] = reason
        timer.stop()
        worker.stop()
        if not worker.wait(5000):
            lifecycle["stop_timed_out"] = True
            fatal_errors.append("worker did not stop within 5 seconds")
        app.quit()

    def on_state(state: Any) -> None:
        try:
            observed_at = utc_now()
            observed_s = time.monotonic()
            observed_ns = time.monotonic_ns()
            row = {
                "emission_index": len(rows),
                "observed_at_utc": observed_at,
                "observed_monotonic_s": observed_s,
                "observed_monotonic_ns": observed_ns,
                "elapsed_s": observed_s - started_monotonic,
                "state": state_payload(state),
            }
            annotate_observation(row, args.source, tracker)
            rows.append(row)
            _append_jsonl(states_stream, row)
            refresh_identity()

            if args.expected_capture_backend is not None:
                actual_backend = row["state"].get("capture_backend")
                if actual_backend == args.expected_capture_backend:
                    lifecycle["backend_verified"] = True
                elif actual_backend not in (None, ""):
                    fatal_errors.append(
                        "capture backend mismatch: expected "
                        f"{args.expected_capture_backend!r}, observed "
                        f"{actual_backend!r}"
                    )

            print(
                f"{row['emission_index']:05d} {observed_at} "
                f"connected={row['state'].get('connected')} "
                f"valid={row['measurement_valid']} "
                f"changed={row['value_changed']} "
                f"age_ms={row['data_age_ms']} "
                f"error={row['state'].get('error')!r}",
                flush=True,
            )
            if fatal_errors:
                QTimer.singleShot(0, lambda: finish("validation-error"))
        except Exception as exc:
            fatal_errors.append(f"state recorder failed: {exc!r}")
            QTimer.singleShot(0, lambda: finish("recorder-error"))

    def on_worker_finished() -> None:
        refresh_identity()
        if not lifecycle["finishing"]:
            lifecycle["worker_finished_unexpectedly"] = True
            fatal_errors.append("worker exited before the requested duration")
            app.quit()

    worker.state_updated.connect(on_state)
    worker.finished.connect(on_worker_finished)
    timer.timeout.connect(lambda: finish("duration"))
    timer.start(max(1, round(args.duration_s * 1000)))

    with state_path.open("x", encoding="utf-8", buffering=1) as states_stream:
        try:
            worker.start()
            app.exec()
        except KeyboardInterrupt:
            fatal_errors.append("probe interrupted by operator")
        except Exception as exc:
            fatal_errors.append(f"probe runtime failed: {exc!r}")
        finally:
            if not lifecycle["finishing"] and worker.isRunning():
                finish("finally")
            states_stream.flush()
            os.fsync(states_stream.fileno())

    refresh_identity()
    if (
        args.expected_capture_backend is not None
        and not lifecycle["backend_verified"]
    ):
        fatal_errors.append(
            "expected capture backend was never observed in CameraState"
        )
    if not rows:
        fatal_errors.append("worker emitted no states")

    markers, marker_errors = _load_markers(output_dir / "markers.jsonl")
    fatal_errors.extend(marker_errors)
    summary = summarize(
        rows,
        args.source,
        markers=markers,
        baseline_p95_s=args.baseline_p95_s,
        baseline_duration_s=args.baseline_duration_s,
    )
    finished_at = utc_now()
    summary.update({
        **run_info,
        "finished_at_utc": finished_at,
        "duration_s_observed": (
            time.monotonic_ns() - started_monotonic_ns
        ) / 1_000_000_000.0,
        "finish_reason": lifecycle["finish_reason"],
        "stop_timed_out": lifecycle["stop_timed_out"],
        "worker_finished_unexpectedly": (
            lifecycle["worker_finished_unexpectedly"]
        ),
        "backend_verified": lifecycle["backend_verified"],
        "fatal_errors": fatal_errors,
        "markers_file": str(output_dir / "markers.jsonl"),
    })
    run_info.update({
        "finished_at_utc": finished_at,
        "finish_reason": lifecycle["finish_reason"],
        "worker_identity": driver_identity(worker, args.source),
    })
    _write_json_atomic(output_dir / "run_info.json", run_info)
    _write_json_atomic(output_dir / "summary.json", summary)
    write_sha_manifest(output_dir)
    print(json.dumps(summary, indent=2), flush=True)
    return 2 if fatal_errors else 0


def mark_event(args: argparse.Namespace) -> int:
    output_dir = args.output_dir.expanduser().resolve()
    if not (output_dir / "run_info.json").is_file():
        raise FileNotFoundError(
            "run directory does not contain run_info.json; start the probe "
            f"first and verify --output-dir: {output_dir}"
        )
    path = output_dir / "markers.jsonl"
    row = {
        "marked_at_utc": utc_now(),
        "marked_monotonic_s": time.monotonic(),
        "marked_monotonic_ns": time.monotonic_ns(),
        "kind": args.kind,
        "label": args.label,
        "note": args.note,
    }
    with path.open("a", encoding="utf-8") as stream:
        _append_jsonl(stream, row)
    # A post-run annotation must refresh the manifest. During a live run the
    # finalizer hashes markers together with the completed states file.
    if (output_dir / "summary.json").is_file():
        write_sha_manifest(output_dir)
    print(json.dumps(row))
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)

    run = commands.add_parser("run", help="Record one worker's emitted states.")
    run.add_argument(
        "--source",
        choices=tuple(VALID_MODES),
        required=True,
    )
    run.add_argument(
        "--mode",
        required=True,
        help="Validated against the selected source; dummy is never allowed.",
    )
    run.add_argument("--duration-s", type=float, default=300.0)
    run.add_argument("--poll-interval", type=float, default=1.0)
    run.add_argument(
        "--baseline-duration-s",
        type=float,
        default=30.0,
        help="Healthy pre-failure window used to estimate p95 cadence.",
    )
    run.add_argument(
        "--baseline-p95-s",
        type=float,
        help="Optional healthy p95 cadence measured by the timing branch.",
    )
    run.add_argument(
        "--software-root",
        type=Path,
        default=RECORDER_ROOT,
        help=(
            "Checkout providing gui/workers.py. Point this at the WGC "
            "checkout for the WGC failure case."
        ),
    )
    run.add_argument(
        "--expected-capture-backend",
        choices=("wgc", "mss"),
        help="Camera-only guard; fail unless CameraState reports this backend.",
    )
    tag = datetime.now().strftime("%Y%m%d_%H%M%S")
    run.add_argument(
        "--output-dir",
        type=Path,
        default=Path("logs") / "validation" / f"failure_probe_{tag}",
    )
    run.set_defaults(handler=run_probe)

    mark = commands.add_parser("mark", help="Timestamp an operator action.")
    mark.add_argument("--output-dir", type=Path, required=True)
    mark.add_argument(
        "--kind",
        choices=("action", "failure-start", "recovery"),
        default="action",
    )
    mark.add_argument("--label", required=True)
    mark.add_argument("--note", default="")
    mark.set_defaults(handler=mark_event)
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    if getattr(args, "duration_s", 1.0) <= 0:
        parser.error("--duration-s must be positive")
    if getattr(args, "poll_interval", 1.0) <= 0:
        parser.error("--poll-interval must be positive")
    if getattr(args, "baseline_duration_s", 1.0) <= 0:
        parser.error("--baseline-duration-s must be positive")
    supplied_p95 = getattr(args, "baseline_p95_s", None)
    if supplied_p95 is not None and supplied_p95 <= 0:
        parser.error("--baseline-p95-s must be positive")
    if getattr(args, "expected_capture_backend", None) and (
        getattr(args, "source", None) != "camera"
    ):
        parser.error("--expected-capture-backend is camera-only")
    try:
        if getattr(args, "command", "") == "run":
            validate_source_mode(args.source, args.mode)
        return int(args.handler(args))
    except (FileExistsError, FileNotFoundError, RuntimeError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
