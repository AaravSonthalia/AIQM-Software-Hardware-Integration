"""Pyrometer active Exactus query — non-state-changing Report Version probe.

Sends the BASF-documented **Report Version** query (`02 56 56 03`) on
COM4 and interprets the response. Fits between `pyrometer_modbus_discover.py`
(READ-ONLY sweep) and `pyrometer_force_modbus.py` (state-changing write)
in the pyrometer safety ladder — see `docs/lab_command_sheet.md` §4.

Why exist: when Modbus discovery comes back `silent_all_combos`, we need
to know WHICH silence we're seeing before we consider the state-changing
force-Modbus write:

    exactus_responsive     — probe is alive in Exactus mode. NO force
                             needed; probe answers Exactus, just not in
                             Modbus. Switch mode via TemperaSure or the
                             force script if Modbus is required.
    echo_of_sent           — serial line is echoing (consistent with
                             USB-serial local echo). Investigate cable /
                             DIP-switch / terminal config before force.
    query_no_response      — true silence. Force is a candidate IF
                             TemperaSure confirms the probe is alive;
                             otherwise physical inspection first.

════════════════════════════════════════════════════════════════════════
Trust contract:

  * NON-STATE-CHANGING — writes documented query bytes on the wire,
    receives documented responses. This script is NOT read-only; it IS
    non-state-changing. The single command it can send (Report Version,
    0x56) is guarded by an ALLOWED_QUERIES whitelist enforced at the
    top of ``write_query_and_capture``.
  * Never sends the mode-switch (0x4D), start/stop conversions
    (0x31/0x30), calibration (0x4D with parameters), reset, reboot,
    or any command with state-change side effects.
  * TemperaSure MUST be CLOSED while this script runs — COM4 is
    exclusive. TemperaSure should be available to reopen BEFORE / AFTER
    as an alive-check, but not concurrently.

Source of the Report Version spec:
  BASF Exactus User Manual v5.04, Appendix 2, page 51:

    Report Version
    Description:      Request pyrometer version data.
    Command Byte:     0x56
    Format:           STX 0x56 0x56 ETX
    Normal Response:  STX 0x95 <1 byte version> <9 byte PROM code> ETX
    Invalid Response: NAK
    Command Example:  02 56 56 03
    Example Response: 02 95 44 E2 5F 50 2B 10 10 16 73 FF 03
                      (The 95 byte indicates the pyrometer is running in
                       application mode. Version = 4.4.
                       Prom Code = E25F502B10101673FF, factory use only.)

The `56 56` framing follows the general Exactus rule that "the LRC for
single byte commands is simply the command byte repeated" (page 51 note).
════════════════════════════════════════════════════════════════════════

Usage (Bulbasaur):
    python scripts\\pyrometer_active_exactus_query.py --dry-run             # preview bytes
    python scripts\\pyrometer_active_exactus_query.py                       # actual query
    python scripts\\pyrometer_active_exactus_query.py \
        --output-dir C:\\lab_evidence\\2026-07-30\\pyro_active_query

Runs BEFORE ``pyrometer_force_modbus.py`` in the lab flow to save time
when force is unnecessary.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

# Reuse CI-safe environment helpers from pyrometer_physical_debug —
# same pattern as pyrometer_modbus_discover.py + pyrometer_force_modbus.py.
from scripts.pyrometer_physical_debug import (  # noqa: E402
    describe_conflicting_processes,
    list_available_ports,
)


# ---------------------------------------------------------------------------
# Constants — BASF Exactus User Manual v5.04, Appendix 2 page 51
# ---------------------------------------------------------------------------

# Report Version command (single-byte 0x56, LRC = repeated command byte
# per page 51 note). Format: STX 0x56 0x56 ETX = 02 56 56 03.
REPORT_VERSION_BYTES: bytes = bytes([0x02, 0x56, 0x56, 0x03])

# Whitelist of writes this script may send. Any future addition needs:
#   (1) a new byte constant with a manual-page citation
#   (2) a docstring update explaining the state-change classification
#   (3) an updated `test_only_report_version_is_allowed` assertion
# Kept as a tuple of `(name, bytes)` so error messages are clear.
ALLOWED_QUERIES: tuple[tuple[str, bytes], ...] = (
    ("REPORT_VERSION", REPORT_VERSION_BYTES),
)

# Expected response shape (BASF manual page 51):
#   STX 0x95 <1 byte version> <9 byte PROM code> ETX  →  13 bytes total
VERSION_RESPONSE_HEADER: int = 0x95
VERSION_RESPONSE_EXPECTED_LEN: int = 13   # 2 framing + 1 ver + 9 PROM + 1 ETX

DEFAULT_PORT: str = "COM4"
BAUD_RATE: int = 115200
DEFAULT_RESPONSE_WINDOW_S: float = 1.0


# ---------------------------------------------------------------------------
# Verdicts + classifications (observational labels, not causal)
# ---------------------------------------------------------------------------
#
# Codex round-4 refinement: labels stay descriptive, never assertive.
# `echo_of_sent` says what we saw (response == sent bytes); it does NOT
# claim "local echo detected" — that would be over-claiming without
# proving local echo on our hardware. Same rule for every label here.

VERDICT_DRY_RUN = "dry_run"
VERDICT_EXACTUS_RESPONSIVE = "exactus_responsive"
VERDICT_ECHO_OF_SENT = "echo_of_sent"
VERDICT_ECHO_PLUS_EXTRA = "echo_plus_extra"
VERDICT_PARTIAL_ECHO = "partial_echo"
VERDICT_NON_ECHO_RESPONSE = "non_echo_response"
VERDICT_QUERY_NO_RESPONSE = "query_no_response"   # NOT "passive_silent" —
                                                  # we sent bytes; this is
                                                  # active silence, not
                                                  # passive
VERDICT_PORT_ERROR = "port_error"

# Sub-classification for exactus_responsive — the response starts with
# STX 0x95 but may not be exactly 13 bytes. Codex round-4: don't hard-fail
# on length mismatch; record shape.
VERSION_SHAPE_COMPLETE = "complete"       # exactly 13 bytes starting 02 95
VERSION_SHAPE_TRUNCATED = "truncated"     # starts 02 95 but shorter
VERSION_SHAPE_EXTRA_BYTES = "extra_bytes" # starts 02 95 but longer


# ---------------------------------------------------------------------------
# Terminal helpers (mirror sibling scripts' style)
# ---------------------------------------------------------------------------

def status(label: str, ok: bool, detail: str = "") -> None:
    tag = "[PASS]" if ok else "[FAIL]"
    line = f"  {tag} {label}"
    if detail:
        line += f" — {detail}"
    print(line)


def info(label: str, detail: str = "") -> None:
    line = f"  [INFO] {label}"
    if detail:
        line += f" — {detail}"
    print(line)


def section(title: str) -> None:
    print()
    print(f"--- {title} ---")


def _hex_pretty(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)


# ---------------------------------------------------------------------------
# Response shape classifier — DUPLICATED from pyrometer_force_modbus.py
# ---------------------------------------------------------------------------
#
# Codex round-4 explicit decision: duplicate, do NOT import. A
# non-state-changing query script importing from a state-changing force
# script creates the wrong mental dependency. If a third script ever
# needs the classifier, extract to `scripts/pyrometer_response_shapes.py`
# then; not before. Duplication cost is ~15 lines.

def classify_response(sent: bytes, received: bytes) -> str:
    """Return a verdict describing the SHAPE of ``received`` vs ``sent``.

    Returns one of the VERDICT_* constants above. Observational, not
    causal — the label describes what appeared on the wire; the SOP
    describes what each label is consistent with.
    """
    if not received:
        return VERDICT_QUERY_NO_RESPONSE
    if received == sent:
        return VERDICT_ECHO_OF_SENT
    # Exactus Report Version response starts with STX 0x95 (page 51).
    # Check this BEFORE the echo_plus_extra branch — an exactus_responsive
    # answer isn't shaped like the echo.
    if len(received) >= 2 and received[0] == 0x02 \
            and received[1] == VERSION_RESPONSE_HEADER:
        return VERDICT_EXACTUS_RESPONSIVE
    if received.startswith(sent):
        return VERDICT_ECHO_PLUS_EXTRA
    if len(received) < len(sent) and sent.startswith(received):
        return VERDICT_PARTIAL_ECHO
    return VERDICT_NON_ECHO_RESPONSE


def classify_version_shape(received: bytes) -> str:
    """When verdict is `exactus_responsive`, describe the length shape.

    Not a verdict — a supplementary field. `complete` means exactly 13
    bytes matching the BASF example. `truncated` / `extra_bytes` cover
    the length mismatches without hard-failing.
    """
    if len(received) == VERSION_RESPONSE_EXPECTED_LEN:
        return VERSION_SHAPE_COMPLETE
    if len(received) < VERSION_RESPONSE_EXPECTED_LEN:
        return VERSION_SHAPE_TRUNCATED
    return VERSION_SHAPE_EXTRA_BYTES


def parse_version_response(received: bytes) -> dict:
    """Best-effort parse of a Report Version response.

    Returns {version_byte, prom_hex, shape}. Fields default to None /
    empty when they can't be extracted (short response, non-Exactus
    shape). Never raises — a query script that can't parse the reply
    still gives the operator useful evidence.
    """
    result = {
        "version_byte": None,
        "prom_hex": "",
        "shape": classify_version_shape(received),
    }
    if len(received) >= 3 and received[0] == 0x02 \
            and received[1] == VERSION_RESPONSE_HEADER:
        result["version_byte"] = received[2]
    if len(received) >= 12 and received[0] == 0x02 \
            and received[1] == VERSION_RESPONSE_HEADER:
        # Bytes 3..11 (9 bytes) are the PROM code.
        result["prom_hex"] = _hex_pretty(bytes(received[3:12]))
    return result


# ---------------------------------------------------------------------------
# Preamble (printed before every non-dry-run)
# ---------------------------------------------------------------------------

PREAMBLE_TEXT: str = (
    "═════════════════════════════════════════════════════════════════════\n"
    " NON-STATE-CHANGING QUERY — pyrometer active Exactus probe\n"
    "═════════════════════════════════════════════════════════════════════\n"
    " This script writes the BASF-documented Report Version query on\n"
    " COM4 and inspects the response. Report Version is documented\n"
    " (Exactus User Manual v5.04, page 51) with `Normal Response: STX\n"
    " 0x95 <1 byte version> <9 byte PROM code> ETX`. The command does\n"
    " NOT change device state.\n"
    "\n"
    " Pre-flight checklist:\n"
    "   1. Is TemperaSure CLOSED right now? (COM4 is exclusive.)\n"
    "   2. Have you run pyrometer_modbus_discover.py first?\n"
    "   3. This script does not require the --i-am-doing-a-write gate\n"
    "      (it does not change probe state), but it DOES put bytes on\n"
    "      the wire. Use --dry-run to preview without opening the port.\n"
    "═════════════════════════════════════════════════════════════════════"
)


def _print_preamble() -> None:
    print(PREAMBLE_TEXT)


# ---------------------------------------------------------------------------
# Write + capture
# ---------------------------------------------------------------------------

def write_query_and_capture(
    port: str,
    payload: bytes,
    response_window_s: float,
) -> dict:
    """Write ``payload`` to ``port`` at 115200 8N1, read for a window.

    SAFETY: ``payload`` MUST be in ALLOWED_QUERIES. This is the ONLY
    write-side guard — the whitelist is the single choke point that
    prevents future edits from adding a state-changing command via
    this function.

    Returns:
        {
          "opened": bool,
          "bytes_written": int,
          "response_hex": str,
          "response_bytes": bytes | None,
          "response_bytes_len": int,
          "error": str,
        }

    Never raises — errors captured into result["error"]. Port always
    closed on the way out.
    """
    allowed_bytes = tuple(b for _, b in ALLOWED_QUERIES)
    if payload not in allowed_bytes:
        allowed_names = ", ".join(n for n, _ in ALLOWED_QUERIES)
        raise RuntimeError(
            f"SAFETY: refusing to send {_hex_pretty(payload)} — not in "
            f"ALLOWED_QUERIES ({allowed_names}). Any new query must be "
            "added to the whitelist explicitly, with a manual-page "
            "citation and a whitelist-membership test."
        )

    result: dict = {
        "opened": False,
        "bytes_written": 0,
        "response_hex": "",
        "response_bytes": None,
        "response_bytes_len": 0,
        "error": "",
    }
    try:
        import serial
    except ImportError:
        result["error"] = "pyserial not installed (pip install pyserial)"
        return result

    ser = None
    try:
        # Short read timeout so the response-window loop is briskly
        # responsive to arriving bytes without oversampling.
        ser = serial.Serial(port, BAUD_RATE, timeout=0.1)
        result["opened"] = True

        n = ser.write(payload)
        try:
            ser.flush()
        except Exception:  # noqa: BLE001 — flush() on some drivers no-ops
            pass
        result["bytes_written"] = int(n) if n is not None else len(payload)

        buffer = bytearray()
        deadline = time.time() + response_window_s
        while time.time() < deadline:
            chunk = ser.read(64)
            if chunk:
                buffer.extend(chunk)
        received = bytes(buffer)
        result["response_hex"] = _hex_pretty(received)
        result["response_bytes"] = received
        result["response_bytes_len"] = len(received)
    except Exception as exc:  # noqa: BLE001 — never let query phase crash caller
        result["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        if ser is not None:
            try:
                ser.close()
            except Exception:  # noqa: BLE001
                pass
    return result


# ---------------------------------------------------------------------------
# Evidence bundle (per-script; own shape)
# ---------------------------------------------------------------------------

def write_evidence_bundle(
    output_dir: Path,
    args_summary: dict,
    environment: dict,
    dry_run: bool,
    write_result: Optional[dict],
    verdict: str,
    version_info: Optional[dict] = None,
) -> dict[str, Path]:
    """Persist the run's structured results for share / paste-back.

    JSON schema is specific to this script:
      timestamp_utc, args, environment, bytes_written_hex,
      bytes_written_len, dry_run, response_hex, response_bytes_len,
      write_error, verdict, version_info (present iff verdict ==
      exactus_responsive; contains {version_byte, prom_hex, shape}).
    """
    output_dir.mkdir(parents=True, exist_ok=True)
    bundle: dict = {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "args": args_summary,
        "environment": environment,
        "bytes_written_hex": _hex_pretty(REPORT_VERSION_BYTES),
        "bytes_written_len": len(REPORT_VERSION_BYTES),
        "dry_run": dry_run,
        "response_hex": (
            write_result.get("response_hex", "") if write_result else ""
        ),
        "response_bytes_len": (
            write_result.get("response_bytes_len", 0) if write_result else 0
        ),
        "write_error": write_result.get("error", "") if write_result else "",
        "verdict": {"state": verdict},
    }
    if version_info is not None:
        bundle["version_info"] = version_info
    result_path = output_dir / "result.json"
    result_path.write_text(json.dumps(bundle, indent=2, default=str))
    return {"result.json": result_path}


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Pyrometer active Exactus query — sends BASF-documented Report "
            "Version query (02 56 56 03), classifies the response. "
            "Non-state-changing, not read-only."
        ),
    )
    parser.add_argument(
        "--port", default=DEFAULT_PORT,
        help=f"Serial port (default: {DEFAULT_PORT})",
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help=(
            "Print the bytes this script WOULD send, open no port. "
            "Useful for previewing intent and CI-testability. Does NOT "
            "change device state either way, but --dry-run guarantees "
            "the port isn't touched at all."
        ),
    )
    parser.add_argument(
        "--response-window-s", type=float, default=DEFAULT_RESPONSE_WINDOW_S,
        help=(
            "Seconds to hold the port open reading after the write "
            f"(default: {DEFAULT_RESPONSE_WINDOW_S}). Long enough to "
            "capture the 13-byte Report Version response even at slow "
            "baud, short enough to keep the script under 2s wall-clock."
        ),
    )
    parser.add_argument(
        "--output-dir", type=Path, default=None,
        help="If set, write result.json for paste-back / archival.",
    )
    args = parser.parse_args(argv)

    print("═══ Pyrometer active Exactus query — non-state-changing ═══")
    print(f"Port:                    {args.port}")
    print(f"Baud:                    {BAUD_RATE} 8N1")
    print(f"Bytes to query:          {_hex_pretty(REPORT_VERSION_BYTES)} "
          f"(BASF manual page 51 — Report Version)")
    print(f"Response window:         {args.response_window_s}s")
    print(f"Mode:                    {'DRY-RUN' if args.dry_run else 'ACTIVE QUERY'}")

    if not args.dry_run:
        section("Preamble")
        _print_preamble()

    # --- Phase 0: Environment inventory -----------------------------------
    section("Phase 0: Environment")
    ports = list_available_ports()
    if ports:
        info(f"{len(ports)} serial port(s) visible to pyserial:")
        target_in_list = False
        for p in ports:
            marker = ""
            if p["device"] == args.port:
                marker = "   ← --port target"
                target_in_list = True
            print(f"    {p['device']:>10}  {p['description']}{marker}")
        if not target_in_list:
            print()
            print(
                f"  [WARN] --port {args.port!r} is NOT in the visible ports "
                "list. Consider one of: "
                f"{sorted({p['device'] for p in ports})}"
            )
    else:
        info("Serial port enumeration unavailable")
    conflicts = describe_conflicting_processes()
    if conflicts:
        print()
        info(
            f"{len(conflicts)} process(es) matching known port-hoggers "
            "(TemperaSure / BASF / MistralGui / kSA):"
        )
        for line in conflicts:
            print(f"    {line}")
    environment_snapshot = {
        "available_ports": ports,
        "conflicting_processes": conflicts,
        "target_port_visible": any(p["device"] == args.port for p in ports),
    }

    # --- Dry-run branch ---------------------------------------------------
    if args.dry_run:
        section("Phase 1: Dry-run")
        info("Bytes that WOULD be sent:", _hex_pretty(REPORT_VERSION_BYTES))
        info("No serial port opened. No writes performed.")
        if args.output_dir:
            written = write_evidence_bundle(
                args.output_dir, vars(args), environment_snapshot,
                dry_run=True, write_result=None, verdict=VERDICT_DRY_RUN,
            )
            print()
            print("Evidence bundle written:")
            for name, path in written.items():
                print(f"  {name}: {path}")
        return 0

    # --- Phase 1: Query + capture -----------------------------------------
    section("Phase 1: Query + capture")
    write_result = write_query_and_capture(
        args.port, REPORT_VERSION_BYTES, args.response_window_s,
    )
    if write_result["error"]:
        status("Open + write", False, write_result["error"])
        verdict = VERDICT_PORT_ERROR
        version_info = None
    else:
        status(
            "Open + write", True,
            f"wrote {write_result['bytes_written']} byte(s) at {BAUD_RATE} 8N1",
        )
        received = write_result.get("response_bytes") or b""
        verdict = classify_response(REPORT_VERSION_BYTES, received)
        version_info = None
        if verdict == VERDICT_EXACTUS_RESPONSIVE:
            version_info = parse_version_response(received)
            info(
                "Response shape:",
                f"{verdict} ({version_info['shape']})",
            )
            if version_info.get("version_byte") is not None:
                info(
                    "Parsed version byte:",
                    f"0x{version_info['version_byte']:02X}",
                )
            if version_info.get("prom_hex"):
                info("PROM code hex:", version_info["prom_hex"])
        elif write_result["response_bytes_len"] > 0:
            info(
                f"Response: {write_result['response_bytes_len']} byte(s)",
                write_result["response_hex"],
            )
            info(f"Classification: {verdict}")
        else:
            info("Response: 0 bytes (query_no_response)")

    section("Verdict")
    print(f"State: {verdict}")
    print(
        "\n"
        "Labels are OBSERVATIONAL — they describe what appeared on the\n"
        "wire, not causation. See docs/lab_command_sheet.md §4.3 for\n"
        "what each verdict is 'consistent with' (never 'proven')."
    )

    # --- Evidence bundle --------------------------------------------------
    if args.output_dir:
        written = write_evidence_bundle(
            args.output_dir, vars(args), environment_snapshot,
            dry_run=False, write_result=write_result, verdict=verdict,
            version_info=version_info,
        )
        print()
        print("Evidence bundle written:")
        for name, path in written.items():
            print(f"  {name}: {path}")

    # Exit codes: 0 for a completed query (regardless of what the response
    # was — success is downstream); 1 for port_error.
    if verdict in (VERDICT_EXACTUS_RESPONSIVE, VERDICT_ECHO_OF_SENT,
                   VERDICT_ECHO_PLUS_EXTRA, VERDICT_PARTIAL_ECHO,
                   VERDICT_NON_ECHO_RESPONSE, VERDICT_QUERY_NO_RESPONSE):
        return 0
    return 1


if __name__ == "__main__":
    sys.exit(main())
