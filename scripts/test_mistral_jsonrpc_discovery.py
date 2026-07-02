#!/usr/bin/env python3
"""
MISTRAL PSU JSON-RPC discovery probe — run on Bulbasaur to identify the
API's method catalog.

Background: on June 23 2026 we discovered MistralGui talks to a JSON-RPC
2.0 backend at ``http://10.0.42.231:9000/api`` (Kestrel/.NET, "Scandes
NoName"). Multi-client safety was proven by three probes during a live
MistralGui session (see ``mistral_psu_backend_discovery_jun23.md`` in
memory). This script continues that work by:

  1. Verifying reachability (GET /)
  2. Trying the three standard discovery conventions (``rpc.discover``,
     ``system.listMethods``, ``system.describe``)
  3. If discovery fails, guessing common vendor method names for V/I
     readback so we get *some* signal without needing Wireshark

Multi-client safety: every probe is a single POST with a short timeout;
none of them mutate server state. The Jun 23 evidence already covered
these exact call shapes; adding a few more guessing probes stays within
the same safety envelope. **But watch MistralGui as this runs** — if you
see any UI flicker, disconnect indicator, or setpoint change, stop
immediately (Ctrl-C) and report.

Usage:
    # On Bulbasaur (default host/port match the Jun 23 discovery):
    python scripts/test_mistral_jsonrpc_discovery.py

    # Retarget if the backend moves or you want to probe from elsewhere:
    python scripts/test_mistral_jsonrpc_discovery.py --host 10.0.42.231 --port 9000

    # Skip the method-guessing block if you only want the standard discovery:
    python scripts/test_mistral_jsonrpc_discovery.py --standard-only

Output goes to stdout, plain text, safe to redirect to a file for later
reference.
"""
from __future__ import annotations

import argparse
import datetime
import json
import sys
from pathlib import Path

# Allow running as `python scripts/test_mistral_jsonrpc_discovery.py` from the
# repo root without a package install.
sys.path.insert(0, str(Path(__file__).parent.parent))

from drivers.mistral_jsonrpc import (  # noqa: E402
    ERR_METHOD_NOT_FOUND,
    JsonRpcError,
    JsonRpcTransportError,
    MistralJsonRpcClient,
)


# ---------------------------------------------------------------------------
# Method-guessing catalog (for use when standard discovery is disabled)
# ---------------------------------------------------------------------------
#
# These are common naming conventions for PSU readback methods across the
# industry. We try each once with no params and, if the server responds
# with method-not-found (-32601), we know it doesn't exist; anything else
# is a signal — either the method exists (success), or it exists but
# expects different params (-32602 invalid params).
#
# Multi-client safety: all read-oriented naming; nothing here can plausibly
# be interpreted as a write. But since we don't KNOW what these do on this
# server, we treat them as best-guess-not-guaranteed and rely on the same
# short-timeout + watchful-user protocol.

GUESS_METHOD_NAMES: list[str] = [
    # Camel/snake variants
    "getVoltage", "get_voltage", "GetVoltage",
    "getCurrent", "get_current", "GetCurrent",
    "readVoltage", "read_voltage", "ReadVoltage",
    "readCurrent", "read_current", "ReadCurrent",
    "voltage", "current",
    # Common short names
    "getV", "getI",
    # With obvious prefixes
    "psu.getVoltage", "psu.getCurrent",
    "power.getVoltage", "power.getCurrent",
    "mistral.getVoltage", "mistral.getCurrent",
    # Setpoint variants (safe reads, symmetric with above)
    "getVoltageSetpoint", "getCurrentSetpoint",
    "getSetpoint", "getStatus",
    # Meta / info
    "getInfo", "info", "status", "identify",
]


# ---------------------------------------------------------------------------
# Output helpers
# ---------------------------------------------------------------------------

def _hdr(title: str) -> None:
    print()
    print("=" * 78)
    print(f"  {title}")
    print("=" * 78)


def _kv(key: str, value: object) -> None:
    print(f"  {key:<24} {value}")


def _ts() -> str:
    return datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")


# ---------------------------------------------------------------------------
# Probe stages
# ---------------------------------------------------------------------------

def probe_reachability(client: MistralJsonRpcClient) -> bool:
    """Return True if the server responded at all on GET /."""
    _hdr("Stage 1 — reachability probe")
    _kv("target", client._url)
    _kv("timestamp", _ts())
    try:
        client.connect()
    except RuntimeError as exc:
        _kv("result", "FAILED")
        _kv("error", str(exc))
        print()
        print("  → server not reachable. Possible reasons:")
        print("    - MistralGui not running (backend service is embedded elsewhere?)")
        print("    - Bulbasaur not on the Omicron Lab10 subnet")
        print("    - Backend IP/port changed since Jun 23 2026")
        print("    - Firewall / route change")
        return False
    _kv("result", "OK — server responded 200 on GET /")
    return True


def probe_standard_discovery(client: MistralJsonRpcClient) -> dict:
    """Run the three standard discovery conventions and print results."""
    _hdr("Stage 2 — standard discovery")
    print(
        "  Trying the three common conventions:\n"
        "    rpc.discover       (OpenRPC 1.x spec)\n"
        "    system.listMethods (XML-RPC introspection legacy)\n"
        "    system.describe    (some vendor implementations)\n"
    )
    results = client.try_standard_discovery()
    for method, outcome in results.items():
        print(f"  --- {method} ---")
        if outcome.get("ok"):
            print(f"    ✓ RESULT (JSON-formatted):")
            _print_json_indent(outcome["result"], indent="      ")
        elif "error_code" in outcome:
            code = outcome["error_code"]
            msg = outcome["error_message"]
            interp = (
                " (method not implemented — no discovery via this convention)"
                if code == ERR_METHOD_NOT_FOUND else ""
            )
            print(f"    ✗ JSON-RPC error {code}: {msg}{interp}")
        elif "transport_error" in outcome:
            print(f"    ✗ transport error: {outcome['transport_error']}")
        print()
    return results


def probe_guess_methods(client: MistralJsonRpcClient) -> dict:
    """Try each guess method with no params; classify by response."""
    _hdr("Stage 3 — method-name guessing")
    print(
        "  Discovery didn't give a catalog. Trying common naming conventions\n"
        "  for PSU readback methods. Interpretation of responses:\n"
        "    ✓ success           → method exists AND accepts no-arg call\n"
        "    ! invalid params    → method exists but needs specific args\n"
        "                          (this is the most informative outcome —\n"
        "                          means we found a real method name)\n"
        "    ✗ method not found  → does not exist on this server\n"
        "    ✗ other error       → some other server issue\n"
    )
    findings: dict[str, dict] = {}
    for method in GUESS_METHOD_NAMES:
        try:
            result = client.call(method, params=None)
            findings[method] = {"ok": True, "result": result}
            print(f"  ✓ {method}: SUCCESS — result: {result!r}")
        except JsonRpcError as exc:
            if exc.code == ERR_METHOD_NOT_FOUND:
                # Skip printing — noise. Just record it.
                findings[method] = {
                    "ok": False, "error_code": exc.code, "note": "not_found",
                }
            else:
                findings[method] = {
                    "ok": False, "error_code": exc.code,
                    "error_message": exc.message, "error_data": exc.data,
                }
                print(f"  ! {method}: {exc.code} — {exc.message}")
        except JsonRpcTransportError as exc:
            findings[method] = {"ok": False, "transport_error": str(exc)}
            print(f"  ✗ {method}: transport error — {exc}")

    interesting = {
        m: r for m, r in findings.items()
        if r.get("ok") or r.get("note") != "not_found"
    }
    if not interesting:
        print()
        print("  (No interesting responses. All guessed methods returned "
              "'method not found'. Next step: Wireshark capture of MistralGui "
              "traffic, or email Thomas Sourisseau for API docs.)")
    return findings


# ---------------------------------------------------------------------------
# Pretty-print JSON with a leading indent so it slots into the report
# ---------------------------------------------------------------------------

def _print_json_indent(obj: object, indent: str = "  ") -> None:
    """json.dumps but each line prefixed with ``indent``."""
    try:
        body = json.dumps(obj, indent=2)
    except TypeError:
        # Non-JSON-serializable result (unusual). Fall back to repr.
        body = repr(obj)
    for line in body.splitlines():
        print(indent + line)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Probe the MISTRAL JSON-RPC 2.0 backend for method catalog.",
    )
    p.add_argument(
        "--host", default=MistralJsonRpcClient.DEFAULT_HOST,
        help=f"Backend host (default: {MistralJsonRpcClient.DEFAULT_HOST} "
             f"— per Jun 23 2026 discovery on Bulbasaur).",
    )
    p.add_argument(
        "--port", type=int, default=MistralJsonRpcClient.DEFAULT_PORT,
        help=f"Backend port (default: {MistralJsonRpcClient.DEFAULT_PORT}).",
    )
    p.add_argument(
        "--path", default=MistralJsonRpcClient.DEFAULT_PATH,
        help=f"JSON-RPC endpoint path (default: {MistralJsonRpcClient.DEFAULT_PATH}).",
    )
    p.add_argument(
        "--timeout", type=float, default=3.0,
        help="Per-request timeout in seconds (default: 3.0). Kept short so "
             "any accidental hang doesn't sit against MistralGui's own "
             "session for long.",
    )
    p.add_argument(
        "--standard-only", action="store_true",
        help="Skip the method-guessing stage; run only reachability + "
             "standard discovery. Faster and even less intrusive.",
    )
    return p.parse_args()


def main() -> int:
    args = parse_args()

    print("MISTRAL JSON-RPC 2.0 discovery probe")
    print(f"Started: {_ts()}")
    print(f"Target:  http://{args.host}:{args.port}{args.path}")
    print()
    print("⚠  WATCH MistralGui while this runs.")
    print("   Any UI flicker / disconnect indicator / setpoint change → Ctrl-C.")
    print("   Jun 23 evidence showed no disruption from equivalent probes;")
    print("   this is defense-in-depth.")

    client = MistralJsonRpcClient(
        host=args.host, port=args.port, path=args.path, timeout=args.timeout,
    )

    if not probe_reachability(client):
        return 1

    standard = probe_standard_discovery(client)
    any_standard_ok = any(v.get("ok") for v in standard.values())

    if not any_standard_ok and not args.standard_only:
        probe_guess_methods(client)

    _hdr("Summary")
    _kv("timestamp end", _ts())
    _kv("reachable", "yes")
    _kv(
        "standard discovery",
        "one or more methods returned a catalog"
        if any_standard_ok
        else "no standard convention was implemented",
    )
    if any_standard_ok:
        print()
        print("  → Next: pick the working discovery method, extract the method")
        print("    list, and populate ``MistralJsonRpcClient.set_read_config`` for")
        print("    the four V/I keys. See docs/mistral_jsonrpc_discovery.md.")
    else:
        print()
        print("  → Standard discovery didn't work.")
        print("    Next options in priority order:")
        print("    1. Review the guess-method output above for any ✓ or !")
        print("       responses. Those method names are real; try them with")
        print("       various params to find the right shape.")
        print("    2. Wireshark: capture MistralGui → 10.0.42.231:9000 traffic")
        print("       while performing representative operations.")
        print("    3. Email thomas.sourisseau@scientaomicron.com for API docs.")
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
