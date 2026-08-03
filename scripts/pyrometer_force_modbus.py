"""Pyrometer force-Modbus — state-changing WRITE (Exactus → Modbus mode switch).

Purpose: send the BASF Exactus→Modbus mode-switch command
(``02 4D 4D 03`` = STX + 'M' + LRC + ETX) to a probe that appears to be
stuck in Exactus mode after a TemperaSure session. After running this
and waiting ~500 ms, the probe returns to Modbus RTU per BASF's spec.

The byte sequence is documented in the BASF Exactus User Manual v5.04
page 51 (Switch to Modbus Mode command). BASF's own recommended recovery
after TemperaSure/special-mode is a physical power-cycle (page 53) —
this write is a documented protocol command AND state-changing, so it
stays behind the ``--i-am-doing-a-write`` gate. See
[[basf-exactus-manual-findings]].

════════════════════════════════════════════════════════════════════════
This is expert-gated. Read before running:

  1. Run ``pyrometer_modbus_discover.py`` FIRST. If it finds identity at
     any (baud, device_id) combo, the probe is already answering Modbus
     and this script is NOT the right tool.
  2. TemperaSure MUST be CLOSED while this script runs — COM4 is
     exclusive. TemperaSure should be available to reopen BEFORE (verify
     probe is alive) and AFTER (verify probe still alive) as an
     alive-check, but not concurrently.
  3. BASF's OWN recommended recovery is Path A (power-cycle the probe).
     Manual page 53: "To return the probe to Modbus mode, disconnect the
     probe from the computer and cycle the power to the probe." That
     needs Jiangang's authorization and physical access to the probe
     box. This script (Path B) is a documented alternative for when
     power-cycle is not immediately available.
  4. Verify success by re-running ``pyrometer_modbus_discover.py``. Bytes
     coming back on the port after this write are NOT success — the BASF
     manual documents ``Response: None (pyrometer switches to Modbus
     Mode immediately)`` for this command. Response bytes matching the
     sent bytes are `echo_of_sent` (consistent with USB-serial local
     echo, not proof of probe response — see
     [[prolific-pl2303-local-echo]]). Success is proven ONLY by
     "discover subsequently reports identity_found_at_default."
════════════════════════════════════════════════════════════════════════

Safety design:
  * The gate ``--i-am-doing-a-write`` is REQUIRED. Absent → exit 2. The
    command is BASF-documented, but it's still state-changing — the
    probe transitions from Exactus mode to Modbus mode as a direct
    result. Gate stays.
  * ``--dry-run`` alone opens NO port, just prints the bytes it WOULD
    send. Useful for CI-testability and for previewing intent.
  * File-name-as-intent: ``pyrometer_force_modbus`` in a Slack paste is
    unambiguous. ``pyrometer_debug --write-something`` would not be.
  * Raw pyserial (not pymodbus): this is a 4-byte fixed-content Exactus
    ASCII command, not Modbus. Using pymodbus here would add
    wrong-protocol scaffolding and hide what's happening.
  * Echo capture: after the write, read the port for
    ``POST_WRITE_ECHO_WINDOW_S`` and record everything received. The
    ``response_hex`` field is diagnostic ONLY — see caveat above.
  * This script does NOT auto-verify. Verification is
    ``pyrometer_modbus_discover.py``'s job and stays in a separate script
    so a bug in one doesn't block running the other.

Usage (Bulbasaur):
    python scripts\\pyrometer_force_modbus.py --dry-run                  # preview bytes
    python scripts\\pyrometer_force_modbus.py --i-am-doing-a-write       # actual write
    python scripts\\pyrometer_force_modbus.py --i-am-doing-a-write \
        --output-dir C:\\lab_evidence\\2026-07-30\\pyro_force_modbus
    python scripts\\pyrometer_force_modbus.py --i-am-doing-a-write \
        --verify                                                          # print next-step
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

# Reuse the CI-safe environment helpers from pyrometer_physical_debug.
# Same rationale as in pyrometer_modbus_discover.py — those two live at
# module-top there without dragging in pyserial/pymodbus.
from scripts.pyrometer_physical_debug import (  # noqa: E402
    describe_conflicting_processes,
    list_available_ports,
)


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# The Exactus ASCII "Switch to Modbus Mode" command:
#   0x02 = STX (start of text)
#   0x4D = 'M' (command byte, per BASF manual v5.04 page 51)
#   0x4D = LRC — "the LRC for single byte commands is simply the command
#          byte repeated" (page 51 note). XOR of {0x4D} = 0x4D.
#   0x03 = ETX (end of text)
#
# Source: BASF Exactus User Manual v5.04 page 51, "Summary of Commands →
# Switch to Modbus Mode". Format: STX 0x4D 0x4D ETX. Response: None
# (pyrometer switches to Modbus Mode immediately). Command Example:
# 02 4D 4D 03. Also observed in Jacques's JacquesPyrometerTesting.ipynb
# cell 3. See [[basf-exactus-manual-findings]] for the full command
# reference.
EXACTUS_TO_MODBUS_BYTES: bytes = bytes([0x02, 0x4D, 0x4D, 0x03])

# Sanity guard: the byte constant is exactly 4 bytes. If a future edit
# accidentally lengthens or truncates it, the write phase asserts this
# invariant before opening the port.
BYTES_EXPECTED_LEN: int = 4

DEFAULT_PORT: str = "COM4"
BAUD_RATE: int = 115200
POST_WRITE_WAIT_S: float = 0.5  # per Jacques's notebook comment

# After the mode-switch write, hold the port open for this long and read
# whatever bytes arrive. Codex round 3: we want to distinguish
# "echo-only" from "silent" from "actual acknowledgment" — but NONE of
# those are treated as success. Verification is
# pyrometer_modbus_discover.py's job. 1 second is enough to catch echo
# bursts + any lingering framing bytes while still keeping the whole
# script under 2 s wall-clock.
POST_WRITE_ECHO_WINDOW_S: float = 1.0

# The next-step command the operator should run to actually verify the
# switch worked. Kept as a template so ``--verify`` prints it and tests
# can assert on the exact text.
NEXT_STEP_COMMAND_TEMPLATE: str = (
    "python scripts\\pyrometer_modbus_discover.py --output-dir "
    "C:\\lab_evidence\\<date>\\pyro_post_force_modbus"
)


# ---------------------------------------------------------------------------
# Verdict states
# ---------------------------------------------------------------------------

VERDICT_DRY_RUN = "dry_run"
VERDICT_WROTE_NO_RESPONSE = "wrote_no_response"
VERDICT_WROTE_WITH_RESPONSE = "wrote_with_response"
VERDICT_PORT_ERROR = "port_error"
VERDICT_SAFETY_DENIED = "safety_denied"


# ---------------------------------------------------------------------------
# Response classification labels (observational, not assertive)
# ---------------------------------------------------------------------------
#
# Codex round-4 refinement: previous "wrote_with_response" was ambiguous —
# it covered echo-of-sent, partial echo, echo + extra bytes, and truly
# novel bytes all under the same label, and the SOP had to inform the
# operator that none of them mean success.
#
# The new response_classification field disambiguates WHAT was seen, still
# without claiming what it MEANS. Labels are observational — describing the
# shape of the response — not asserting a root cause. Success continues to
# come from re-running pyrometer_modbus_discover.py; none of these
# classifications by themselves prove the mode switch worked.
#
# Rationale for keeping "consistent with USB-serial echo" out of the label:
# we've observed the pattern in Jacques's notebook but have not proven
# local echo on the O-MBE/Ch-MBE hardware. Labels stay descriptive; the
# SOP explains what echo-of-sent is consistent with. See
# [[prolific-pl2303-local-echo]].
RESPONSE_NONE = "no_response"              # 0 bytes received
RESPONSE_ECHO_OF_SENT = "echo_of_sent"     # response == sent bytes exactly
RESPONSE_ECHO_PLUS_EXTRA = "echo_plus_extra"  # starts with sent bytes + more
RESPONSE_PARTIAL_ECHO = "partial_echo"     # partial prefix match of sent
RESPONSE_NON_ECHO = "non_echo_response"    # neither empty nor echo-shaped


def classify_response(sent: bytes, received: bytes) -> str:
    """Classify a post-write response by shape only. Observational.

    NEITHER label proves anything about probe state or line behavior —
    they only describe what appeared on the wire. Downstream interpretation
    lives in the SOP; success still requires pyrometer_modbus_discover.py
    to report `identity_found_at_default`.
    """
    if not received:
        return RESPONSE_NONE
    if received == sent:
        return RESPONSE_ECHO_OF_SENT
    if received.startswith(sent):
        return RESPONSE_ECHO_PLUS_EXTRA
    # Partial-echo: received is a proper prefix of sent (shorter, matches),
    # OR received's leading bytes match some prefix of sent. The common
    # case is a truncated echo where a few bytes made it through.
    if len(received) < len(sent) and sent.startswith(received):
        return RESPONSE_PARTIAL_ECHO
    return RESPONSE_NON_ECHO


# ---------------------------------------------------------------------------
# Terminal helpers (mirror pyrometer_physical_debug's style)
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
    """Space-separated uppercase hex — e.g. bytes([0x02, 0x4D]) → '02 4D'."""
    return " ".join(f"{x:02X}" for x in b)


# ---------------------------------------------------------------------------
# Preamble — printed prominently before every real-write run
# ---------------------------------------------------------------------------

PREAMBLE_TEXT: str = (
    "═════════════════════════════════════════════════════════════════════\n"
    " STATE-CHANGING WRITE — pyrometer mode-switch (Exactus → Modbus)\n"
    "═════════════════════════════════════════════════════════════════════\n"
    " This script writes the BASF-documented mode-switch command\n"
    " (Exactus User Manual v5.04, page 51). BASF's OWN recommended\n"
    " recovery from TemperaSure/special-mode is a physical power-cycle\n"
    " (manual page 53) — this script is Path B, a documented alternative\n"
    " when power-cycle is not immediately available.\n"
    "\n"
    " Pre-flight checklist:\n"
    "   1. Have you run pyrometer_modbus_discover.py first?\n"
    "   2. Is TemperaSure CLOSED right now? (COM4 is exclusive.)\n"
    "   3. Is TemperaSure AVAILABLE to reopen after this script exits\n"
    "      so you can verify the probe is still alive?\n"
    "   4. Do you have authorization for Path A (power-cycle) as\n"
    "      fallback if Path B doesn't produce the desired result?\n"
    "\n"
    " Bytes coming back on the port after this write are NOT success.\n"
    " The BASF spec says 'Response: None (pyrometer switches to Modbus\n"
    " Mode immediately)'. Response bytes matching what we sent are an\n"
    " `echo_of_sent` observation — consistent with USB-serial local\n"
    " echo, not proof of probe response. Success is proven ONLY by\n"
    " 'discover subsequently reports identity_found_at_default'.\n"
    "═════════════════════════════════════════════════════════════════════"
)


# ---------------------------------------------------------------------------
# Write phase — raw pyserial, not pymodbus
# ---------------------------------------------------------------------------

def write_and_capture_echo(
    port: str,
    payload: bytes,
    post_write_wait_s: float,
    echo_window_s: float,
) -> dict:
    """Write ``payload`` to ``port`` at 115200 8N1, then read for echo.

    Returns:
        {
          "opened": bool,
          "bytes_written": int,
          "response_hex": str,     # empty string if nothing arrived
          "response_bytes_len": int,
          "error": str,            # empty string on success
        }

    Never raises — errors are captured into result["error"] so the
    caller can still write an evidence bundle. Port is always closed
    on the way out (try/finally).

    Assertion: len(payload) == BYTES_EXPECTED_LEN. If a future edit
    changes the byte constant, this catches it before opening the port.
    """
    assert len(payload) == BYTES_EXPECTED_LEN, (
        f"payload len {len(payload)} != expected {BYTES_EXPECTED_LEN}. "
        "The Exactus→Modbus command is fixed 4 bytes (STX + 'MM' + ETX)."
    )
    result: dict = {
        "opened": False,
        "bytes_written": 0,
        "response_hex": "",
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
        # Timeout controls how long ``ser.read(N)`` blocks waiting for
        # bytes. 0.1 s is short enough that we loop briskly in the echo
        # window while still batching multi-byte arrivals.
        ser = serial.Serial(port, BAUD_RATE, timeout=0.1)
        result["opened"] = True

        n = ser.write(payload)
        try:
            ser.flush()
        except Exception:  # noqa: BLE001 — flush() on some drivers no-ops
            pass
        result["bytes_written"] = int(n) if n is not None else len(payload)

        # Post-write pause per Jacques's notebook comment. Matches the
        # notebook's own timing so we don't accidentally miss a slow echo.
        time.sleep(post_write_wait_s)

        # Echo capture window — accumulate whatever bytes arrive.
        buffer = bytearray()
        deadline = time.time() + echo_window_s
        while time.time() < deadline:
            chunk = ser.read(64)
            if chunk:
                buffer.extend(chunk)
        result["response_hex"] = _hex_pretty(bytes(buffer))
        result["response_bytes_len"] = len(buffer)
    except Exception as exc:  # noqa: BLE001 — never let write phase crash caller
        result["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        # Port MUST be closed even if write / read raised. Otherwise the
        # port stays wedged and the operator can't run discover next.
        if ser is not None:
            try:
                ser.close()
            except Exception:  # noqa: BLE001
                pass
    return result


# ---------------------------------------------------------------------------
# Evidence bundle (per-script; do NOT reuse pyrometer_physical_debug's)
# ---------------------------------------------------------------------------

def write_evidence_bundle(
    output_dir: Path,
    args_summary: dict,
    environment: dict,
    dry_run: bool,
    write_result: Optional[dict],
    state: str,
    response_classification: str = "",
) -> dict[str, Path]:
    """Persist the run's structured results for share / paste-back.

    JSON schema is specific to this script — deliberately different
    from pyrometer_physical_debug's and pyrometer_modbus_discover's
    schemas. See plan P3.

    ``response_classification`` (Codex round-4) is one of the
    ``RESPONSE_*`` labels — describes the SHAPE of what came back on
    the wire without claiming causation. See ``classify_response``.
    Empty string on dry-run / safety-denied / port-error runs where no
    write reached the wire.
    """
    output_dir.mkdir(parents=True, exist_ok=True)
    bundle: dict = {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "args": args_summary,
        "environment": environment,
        "bytes_written_hex": _hex_pretty(EXACTUS_TO_MODBUS_BYTES),
        "bytes_written_len": BYTES_EXPECTED_LEN,
        "dry_run": dry_run,
        "response_hex": (
            write_result.get("response_hex", "") if write_result else ""
        ),
        "response_bytes_len": (
            write_result.get("response_bytes_len", 0) if write_result else 0
        ),
        "response_classification": response_classification,
        "write_error": write_result.get("error", "") if write_result else "",
        "verdict": {"state": state},
    }
    result_path = output_dir / "result.json"
    result_path.write_text(json.dumps(bundle, indent=2, default=str))
    return {"result.json": result_path}


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _print_preamble() -> None:
    """Print the safety preamble to stdout. Isolated so tests can suppress."""
    print(PREAMBLE_TEXT)


def _next_step_hint() -> str:
    """The command the operator should run to actually verify the switch."""
    return NEXT_STEP_COMMAND_TEMPLATE


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Pyrometer force-Modbus — EXPERIMENTAL WRITE of the Exactus→"
            "Modbus mode-switch command. Requires --i-am-doing-a-write. "
            "Run pyrometer_modbus_discover.py first to rule out "
            "configuration issues."
        ),
    )
    parser.add_argument(
        "--port", default=DEFAULT_PORT,
        help=f"Serial port (default: {DEFAULT_PORT})",
    )
    parser.add_argument(
        "--i-am-doing-a-write", action="store_true",
        dest="i_am_doing_a_write",
        help=(
            "Explicitly acknowledge that this is a WRITE script. "
            "REQUIRED unless --dry-run is set. Absent both flags → exit 2."
        ),
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help=(
            "Print the bytes this script WOULD send and exit 0 without "
            "opening the serial port. Useful for previewing intent and "
            "for CI-testability."
        ),
    )
    parser.add_argument(
        "--output-dir", type=Path, default=None,
        help="If set, write result.json for paste-back / archival.",
    )
    parser.add_argument(
        "--verify", action="store_true",
        help=(
            "After the run, print the exact next-step command the "
            "operator should run to verify the switch worked "
            "(pyrometer_modbus_discover.py). Does NOT auto-invoke — "
            "keeps scripts independent."
        ),
    )
    args = parser.parse_args(argv)

    # --- Safety gate ------------------------------------------------------
    # Exactly one of --dry-run OR --i-am-doing-a-write must be set.
    # Neither → exit 2, safety denied. Both → treat --dry-run as
    # higher-precedence (don't touch the port even if the operator ack'd
    # the write) so previewing is always safe.
    if not (args.dry_run or args.i_am_doing_a_write):
        print(
            "\n"
            "SAFETY DENIED: no write-authorization flag set.\n"
            "\n"
            "This script writes an EXPERIMENTAL command to the pyrometer.\n"
            "You must set EXACTLY ONE of:\n"
            "  --dry-run                — preview the bytes, open no port\n"
            "  --i-am-doing-a-write     — actually send the write\n"
            "\n"
            f"Bytes that would be written: {_hex_pretty(EXACTUS_TO_MODBUS_BYTES)}\n"
            "See --help for the full expert-gated safety context.\n",
            file=sys.stderr,
        )
        if args.output_dir:
            write_evidence_bundle(
                args.output_dir, vars(args), environment={},
                dry_run=False, write_result=None,
                state=VERDICT_SAFETY_DENIED,
            )
        return 2

    print("═══ Pyrometer force-Modbus — EXPERIMENTAL WRITE ═══")
    print(f"Port:                    {args.port}")
    print(f"Baud:                    {BAUD_RATE} 8N1")
    print(f"Bytes to write:          {_hex_pretty(EXACTUS_TO_MODBUS_BYTES)} "
          f"(len={BYTES_EXPECTED_LEN})")
    print(f"Post-write wait:         {POST_WRITE_WAIT_S}s")
    print(f"Echo capture window:     {POST_WRITE_ECHO_WINDOW_S}s")
    print(f"Mode:                    {'DRY-RUN' if args.dry_run else 'ACTUAL WRITE'}")

    if not args.dry_run:
        section("Safety preamble")
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
        print()
        info(
            "TemperaSure or other COM4-holder is running. This script CANNOT "
            "share the port. Close TemperaSure before continuing."
        )
    environment_snapshot = {
        "available_ports": ports,
        "conflicting_processes": conflicts,
        "target_port_visible": any(p["device"] == args.port for p in ports),
    }

    # --- Phase 1: Dry-run branch (no port touched) ------------------------
    if args.dry_run:
        section("Phase 1: Dry-run")
        info("Bytes that WOULD be sent:", _hex_pretty(EXACTUS_TO_MODBUS_BYTES))
        info("No serial port opened. No writes performed.")
        state = VERDICT_DRY_RUN
        if args.output_dir:
            written = write_evidence_bundle(
                args.output_dir, vars(args), environment_snapshot,
                dry_run=True, write_result=None, state=state,
            )
            print()
            print("Evidence bundle written:")
            for name, path in written.items():
                print(f"  {name}: {path}")
        if args.verify:
            section("Next step")
            print(_next_step_hint())
        return 0

    # --- Phase 2: Write + echo capture ------------------------------------
    section("Phase 2: Write + echo capture")
    write_result = write_and_capture_echo(
        args.port, EXACTUS_TO_MODBUS_BYTES,
        POST_WRITE_WAIT_S, POST_WRITE_ECHO_WINDOW_S,
    )
    response_classification = ""
    if write_result["error"]:
        status("Open + write", False, write_result["error"])
        state = VERDICT_PORT_ERROR
    else:
        status(
            "Open + write", True,
            f"wrote {write_result['bytes_written']} byte(s) at {BAUD_RATE} 8N1",
        )
        # Reconstruct raw response bytes from the pretty-printed hex.
        # write_and_capture_echo currently returns hex-formatted only;
        # classify by re-parsing the same string. Safe roundtrip because
        # _hex_pretty produces "XX XX XX" with no ambiguity.
        received_bytes = (
            bytes.fromhex(write_result["response_hex"].replace(" ", ""))
            if write_result["response_hex"] else b""
        )
        response_classification = classify_response(
            EXACTUS_TO_MODBUS_BYTES, received_bytes,
        )
        if write_result["response_bytes_len"] > 0:
            info(
                f"Echo capture: {write_result['response_bytes_len']} byte(s) received",
                write_result["response_hex"],
            )
            info(f"Response classification: {response_classification}")
            info(
                "REMINDER: bytes returned are NOT success. `echo_of_sent` "
                "is consistent with USB-serial local echo (see "
                "docs/lab_command_sheet.md 4.3 verdict table). Verify "
                "success via pyrometer_modbus_discover.py."
            )
            state = VERDICT_WROTE_WITH_RESPONSE
        else:
            info(
                "Echo capture: 0 bytes received "
                "(matches BASF Exactus 'no response expected' documentation)"
            )
            state = VERDICT_WROTE_NO_RESPONSE

    section("Verdict")
    print(f"State:                   {state}")
    if response_classification:
        print(f"Response classification: {response_classification}")
    print(
        "\n"
        "This verdict is DIAGNOSTIC ONLY — not a success flag.\n"
        "Response bytes on the wire — even if `echo_of_sent` — do not\n"
        "confirm the mode switch. Success is proven ONLY by re-running\n"
        "pyrometer_modbus_discover.py and getting identity_found_at_default."
    )

    # --- Evidence bundle --------------------------------------------------
    if args.output_dir:
        written = write_evidence_bundle(
            args.output_dir, vars(args), environment_snapshot,
            dry_run=False, write_result=write_result, state=state,
            response_classification=response_classification,
        )
        print()
        print("Evidence bundle written:")
        for name, path in written.items():
            print(f"  {name}: {path}")

    # --- Next-step hint ---------------------------------------------------
    if args.verify:
        section("Next step (run this to verify)")
        print(_next_step_hint())

    # Exit 0 for any completed write (with or without response) or dry-run.
    # Only port_error / safety_denied return non-zero — matches the pattern
    # that "success != script exit code," it just means "the write phase
    # ran without exploding." Real success comes from discover.
    return 0 if state in (
        VERDICT_DRY_RUN, VERDICT_WROTE_NO_RESPONSE, VERDICT_WROTE_WITH_RESPONSE,
    ) else 1


if __name__ == "__main__":
    sys.exit(main())
