"""Parser for ScientaOmicron EvapControl .elo logfile format.

Reverse-engineered May 15 2026 from a real Bulbasaur log file. The viewer
(``logfile_viewer``) calls them "ELog" files. Both .elo and .elo.zip are
written by EvapControl — the .zip variant is just an ordinary zip wrapping
a single .elo. Files rotate at local midnight; current file is always
``log_<YYYY-MM-DD>_000000.elo`` in
``C:\\_Omicron_Software\\EvapControl\\evap_control_1.2.0.51\\evap_control_1.2.0.51\\log\\``.

Format
------
All multi-byte integers and floats are big-endian.

    [u32 var_count][u32 version=2]
    schema:
        for var in var_count:
            [u32 name_len][utf-8 name][u32 fmt_len][utf-8 printf-style format]
    data:
        for record in (file_size - schema_size) // record_size:
            [f64 labview_timestamp][var_count × f32 values]

The timestamp is seconds since the LabView epoch (1904-01-01 UTC). Records
are emitted at ~1 Hz. Format strings encode units and value type, including
enum-like ``CLOSED:OPEN`` (boolean encoded as 0.0/1.0 float) and
``ERR:OFF:Manual:PID:Atune`` (state encoded as 0/1/2/3/4 float).

Usage
-----
    from drivers.elog import parse_elog, latest_value

    var_names, timestamps, values = parse_elog(path)
    pressure_idx = var_names.index("MBE.Pressure")
    print(values[-1, pressure_idx], "mbar")

    # Or read the live last value cheaply (only reads the tail):
    p = latest_value(path, "MBE.Pressure")
"""
from __future__ import annotations

import datetime
import struct
import zipfile
from pathlib import Path
from typing import IO

import numpy as np

LABVIEW_EPOCH = datetime.datetime(1904, 1, 1, tzinfo=datetime.timezone.utc)
RECORD_TIMESTAMP_SIZE = 8  # f64 BE
VALUE_SIZE = 4  # f32 BE


def _open_elo(path: str | Path) -> IO[bytes]:
    """Return a binary stream for an .elo or .elo.zip file."""
    p = Path(path)
    if p.suffix == ".zip":
        zf = zipfile.ZipFile(p, "r")
        inner = next(n for n in zf.namelist() if n.endswith(".elo"))
        return zf.open(inner, "r")
    return open(p, "rb")


def parse_schema(stream: IO[bytes]) -> tuple[list[str], list[str], int]:
    """Parse the schema header. Returns (names, format_strings, schema_byte_count).

    Leaves the stream positioned at the start of the data section.
    """
    header = stream.read(8)
    n_vars, version = struct.unpack(">II", header)
    if version != 2:
        raise ValueError(f"Unsupported .elo version: {version}")
    names: list[str] = []
    fmts: list[str] = []
    bytes_read = 8
    for _ in range(n_vars):
        nlen = struct.unpack(">I", stream.read(4))[0]
        name = stream.read(nlen).decode("utf-8", errors="replace")
        flen = struct.unpack(">I", stream.read(4))[0]
        fmt = stream.read(flen).decode("utf-8", errors="replace")
        names.append(name)
        fmts.append(fmt)
        bytes_read += 4 + nlen + 4 + flen
    return names, fmts, bytes_read


def parse_elog(
    path: str | Path,
) -> tuple[list[str], np.ndarray, np.ndarray]:
    """Parse a complete .elo (or .elo.zip) file.

    Returns:
        var_names: list of length N_VARS
        timestamps: 1D ndarray of datetime objects, length N_RECORDS
        values: 2D ndarray of float32, shape (N_RECORDS, N_VARS)
    """
    with _open_elo(path) as f:
        names, _fmts, _ = parse_schema(f)
        n_vars = len(names)
        record_size = RECORD_TIMESTAMP_SIZE + n_vars * VALUE_SIZE
        body = f.read()
    n_records = len(body) // record_size
    if n_records * record_size != len(body):
        # Trailing partial record (file being written) — drop it.
        body = body[: n_records * record_size]

    arr = np.frombuffer(body, dtype=np.uint8).reshape(n_records, record_size)
    ts_raw = arr[:, :RECORD_TIMESTAMP_SIZE].copy()
    ts_floats = ts_raw.view(">f8").reshape(n_records)
    timestamps = np.array(
        [LABVIEW_EPOCH + datetime.timedelta(seconds=float(t)) for t in ts_floats],
        dtype=object,
    )
    values_raw = arr[:, RECORD_TIMESTAMP_SIZE:].copy()
    values = values_raw.view(">f4").reshape(n_records, n_vars).astype(np.float32)
    return names, timestamps, values


def latest_value(path: str | Path, var_name: str) -> tuple[datetime.datetime, float]:
    """Return the most recent (timestamp, value) for ``var_name``.

    Cheap: parses the schema and reads only the tail record, not the whole file.
    Useful for live polling without re-reading 20 MB every tick.
    """
    with _open_elo(path) as f:
        names, _fmts, schema_bytes = parse_schema(f)
        n_vars = len(names)
        record_size = RECORD_TIMESTAMP_SIZE + n_vars * VALUE_SIZE
        # Seek to tail; for zip-wrapped files this falls back to streaming.
        try:
            f.seek(-record_size, 2)  # 2 = SEEK_END
            tail = f.read(record_size)
        except (OSError, AttributeError):
            # zip stream — read everything to end and slice the tail.
            blob = f.read()
            tail = blob[-record_size:]
    if var_name not in names:
        raise KeyError(f"variable {var_name!r} not in {len(names)} schema entries")
    idx = names.index(var_name)
    ts_f = struct.unpack(">d", tail[:RECORD_TIMESTAMP_SIZE])[0]
    val_offset = RECORD_TIMESTAMP_SIZE + idx * VALUE_SIZE
    val = struct.unpack(">f", tail[val_offset : val_offset + VALUE_SIZE])[0]
    return LABVIEW_EPOCH + datetime.timedelta(seconds=ts_f), float(val)


def find_current_log(
    log_dir: str | Path = r"C:\_Omicron_Software\EvapControl\evap_control_1.2.0.51\evap_control_1.2.0.51\log",
) -> Path | None:
    """Return the path to today's live .elo file, or None if not found.

    EvapControl rotates at local midnight; the live file is always named
    ``log_<YYYY-MM-DD>_000000.elo`` (uncompressed). Older days are auto-zipped.
    """
    today = datetime.date.today().isoformat()
    candidate = Path(log_dir) / f"log_{today}_000000.elo"
    return candidate if candidate.exists() else None


def latest_record(
    path: str | Path,
    var_names: list[str] | None = None,
) -> tuple[datetime.datetime, dict[str, tuple[float, str]]]:
    """Return the most recent values for multiple variables in one open.

    Parses the schema once and reads only the tail record, then extracts the
    requested vars. Much cheaper than calling ``latest_value`` N times.

    Args:
        path: .elo or .elo.zip file
        var_names: variables to extract; None means all.

    Returns:
        (timestamp, {name: (value, format_string)})
    """
    with _open_elo(path) as f:
        names, fmts, _ = parse_schema(f)
        n_vars = len(names)
        record_size = RECORD_TIMESTAMP_SIZE + n_vars * VALUE_SIZE
        try:
            f.seek(-record_size, 2)
            tail = f.read(record_size)
        except (OSError, AttributeError):
            blob = f.read()
            tail = blob[-record_size:]
    ts_f = struct.unpack(">d", tail[:RECORD_TIMESTAMP_SIZE])[0]
    ts = LABVIEW_EPOCH + datetime.timedelta(seconds=ts_f)
    wanted = var_names if var_names is not None else names
    out: dict[str, tuple[float, str]] = {}
    for name in wanted:
        if name not in names:
            raise KeyError(f"variable {name!r} not in {n_vars} schema entries")
        idx = names.index(name)
        offset = RECORD_TIMESTAMP_SIZE + idx * VALUE_SIZE
        val = struct.unpack(">f", tail[offset : offset + VALUE_SIZE])[0]
        out[name] = (float(val), fmts[idx])
    return ts, out


def format_value(value: float, fmt: str) -> str:
    """Render a value using its EvapControl format string.

    Format strings come in two flavors:
        - printf-style with optional unit suffix: "%.2e mbar", "%.1f W"
        - colon-separated enum: "CLOSED:OPEN", "ERR:OFF:Manual:PID:Atune"
          (value is an integer-valued float used as an index)
    """
    fmt = fmt.strip()
    if "%" not in fmt and ":" in fmt:
        # Enum: pick by integer index
        labels = fmt.split(":")
        idx = int(round(value))
        if 0 <= idx < len(labels):
            return labels[idx]
        return f"?({value:g})"
    # printf-style; split format spec from trailing unit
    parts = fmt.split(None, 1)
    spec = parts[0]
    unit = parts[1] if len(parts) > 1 else ""
    try:
        body = spec % value
    except (TypeError, ValueError):
        body = f"{value:g}"
    return f"{body} {unit}".rstrip()
