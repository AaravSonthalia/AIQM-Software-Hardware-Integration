#!/usr/bin/env python3
"""kSAComm TCP/IP smoke test.

Validates the kSA Communications Interface (per kSAcomm Protocol V4.1
May 2021) on this machine. Connects, handshakes, asks for app version
and system status, optionally probes for kSA 400 Average Intensity if
the acquisition is running.

Prerequisites:
    - kSA application running (e.g., kSA 400) with Chamber Interface
      configured to kSAComm on port 1800
    - kSA application restarted after the config change

Usage:
    python scripts\\test_ksa_comm.py
    python scripts\\test_ksa_comm.py --host 127.0.0.1 --port 1800
"""
from __future__ import annotations

import argparse
import socket
import struct
import sys


# Command codes per protocol spec section 5
CMD_INITIALIZE = 1000
CMD_SET_DATA_FIELDS = 1001
CMD_RUN = 1002
CMD_GET_DATA = 1003
CMD_STOP = 1004
CMD_GET_DATA_SPECIFIC = 1005
CMD_RESTART_GROWTHRATE_FIT = 1006
CMD_OPEN_ACQUIRE = 1007
CMD_CLOSE_ACQUIRE = 1008
CMD_GET_STATUS = 1009
CMD_GET_APP_VERSION = 1010
CMD_TEXT_CMD = 1011

# kSA 400 application data fields (section 10.2.4)
KSA400_FIELDS = {
    1: "Peak Intensity",
    2: "Summed Intensity",
    3: "Average Intensity",
    11: "Centroid Intensity",
    21: "Center Intensity",
    23: "Normalized Intensity",
    80: "Minimum Intensity",
}

# Measurement IDs (section 5.6 list)
MEAS_ID_GENERIC_IMAGE = 100  # kSA 400 / kSAMOS

# Operational status codes (GET_STATUS section 5.10)
OP_STATUS = {0: "Not operational", 1: "Idle", 2: "Acquiring", 3: "Paused"}
RPM_STATUS = {0: "Unstable", 1: "Stable", 2: "Artificial"}

# Error codes (section 9)
ERR_CODES = {
    0: "Success",
    -1: "General error",
    -2: "Unknown command",
    -3: "Invalid parameter",
    -4: "Invalid state",
}


def send_short_string(sock: socket.socket, s: str) -> None:
    """Per protocol: [Len:1][chars:Len-1][null:1]. Len includes null."""
    payload = s.encode("ascii") + b"\x00"
    sock.sendall(struct.pack("B", len(payload)) + payload)


def recv_short_string(sock: socket.socket) -> str:
    length = sock.recv(1)
    if not length:
        raise ConnectionError("Server closed during string read")
    n = length[0]
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("Server closed mid-string")
        buf += chunk
    # Strip trailing null
    return buf.rstrip(b"\x00").decode("ascii", errors="replace")


def send_command(sock: socket.socket, cmd_code: int, cmd_data: bytes = b"") -> None:
    """[CmdCode:2 LE][CmdLength:2 LE][CmdData:CmdLength]"""
    msg = struct.pack("<HH", cmd_code, len(cmd_data)) + cmd_data
    sock.sendall(msg)


def recv_reply(sock: socket.socket) -> tuple[int, int, bytes]:
    """[CmdCode:2 LE][ErrCode:2 LE int16][ReplyLength:2 LE][ReplyData:ReplyLength]"""
    header = b""
    while len(header) < 6:
        chunk = sock.recv(6 - len(header))
        if not chunk:
            raise ConnectionError("Server closed mid-reply-header")
        header += chunk
    cmd_code, err_code, reply_length = struct.unpack("<HhH", header)
    data = b""
    while len(data) < reply_length:
        chunk = sock.recv(reply_length - len(data))
        if not chunk:
            raise ConnectionError("Server closed mid-reply-data")
        data += chunk
    return cmd_code, err_code, data


def err_name(code: int) -> str:
    return ERR_CODES.get(code, f"unknown({code})")


def cmd_get_app_version(sock: socket.socket) -> str | None:
    send_command(sock, CMD_GET_APP_VERSION)
    cmd, err, data = recv_reply(sock)
    if err != 0:
        print(f"  GET_APP_VERSION ERROR: {err_name(err)}")
        return None
    # Reply data: [Length:1 = 32][Version:32 ASCII]
    if not data:
        return ""
    ver_len = data[0]
    ver = data[1 : 1 + ver_len].rstrip(b"\x00").decode("ascii", errors="replace")
    return ver


def cmd_get_status(sock: socket.socket) -> dict | None:
    send_command(sock, CMD_GET_STATUS)
    cmd, err, data = recv_reply(sock)
    if err != 0:
        print(f"  GET_STATUS ERROR: {err_name(err)}")
        return None
    # Per section 5.10: StatusSize(2) StatusVersion(2) OpStatus(2) LastHome(8) RpmStatus(2) Rpm(8)
    if len(data) < 24:
        print(f"  GET_STATUS reply too short: {len(data)} bytes")
        return None
    status_size, status_version, op_status = struct.unpack_from("<HHH", data, 0)
    last_home = struct.unpack_from("<d", data, 6)[0]
    rpm_status = struct.unpack_from("<H", data, 14)[0]
    rpm = struct.unpack_from("<d", data, 16)[0]
    return {
        "status_size": status_size,
        "status_version": status_version,
        "op_status": op_status,
        "op_status_name": OP_STATUS.get(op_status, f"unknown({op_status})"),
        "last_home_pulse": last_home,
        "rpm_status": rpm_status,
        "rpm_status_name": RPM_STATUS.get(rpm_status, f"unknown({rpm_status})"),
        "rpm": rpm,
    }


def cmd_text(sock: socket.socket, text: str, encoding: str = "raw") -> tuple[int, str]:
    """Send a TEXT_CMD (1011) and return (err_code, reply_text).

    Request encoding:
        "raw"    — CmdData = text bytes only (outer CmdLength has the size)
        "short"  — CmdData = [len:1][chars:len-1][null] (handshake convention)
        "u16"    — CmdData = [len:2 LE][chars]
        "u32"    — CmdData = [len:4 LE][chars][null]  ← working encoding on v6

    Reply format is always [len:4 LE][chars][null] regardless of request
    encoding (per May 20 breakthrough, re-confirmed Jul 7). Length includes
    the trailing null. Error replies with server-side text messages (err=-1,
    err=-2) follow the same u32-prefixed shape.
    """
    if encoding == "raw":
        body = text.encode("ascii")
    elif encoding == "short":
        payload = text.encode("ascii") + b"\x00"
        body = struct.pack("B", len(payload)) + payload
    elif encoding == "u16":
        chars = text.encode("ascii")
        body = struct.pack("<H", len(chars)) + chars
    elif encoding == "u32":
        payload = text.encode("ascii") + b"\x00"
        body = struct.pack("<I", len(payload)) + payload
    else:
        raise ValueError(f"unknown encoding {encoding!r}")
    send_command(sock, CMD_TEXT_CMD, body)
    cmd, err, data = recv_reply(sock)
    # Reply strip: skip the 4-byte u32 length prefix, decode the rest,
    # rstrip trailing nulls. Applies uniformly to err=0 and err<0 replies.
    if len(data) >= 4:
        text_out = data[4:].decode("ascii", errors="replace").rstrip("\x00")
    else:
        text_out = ""
    return err, text_out


def probe_text_commands(sock: socket.socket, commands: list[str]) -> None:
    """Send a series of TEXT_CMD probes; print request/reply/error for each.

    Iterates through candidate encodings. On v6, "u32" is the known working
    encoding (per May 20 breakthrough, re-confirmed Jul 7). The other three
    are retained as diagnostic negatives — they should return -2 on kSA v6,
    which itself is informative if we ever debug encoding drift.
    """
    for enc in ("raw", "short", "u16", "u32"):
        print(f"\n--- TEXT_CMD probe (encoding={enc}) ---")
        any_success = False
        for cmd_text_str in commands:
            err, reply = cmd_text(sock, cmd_text_str, encoding=enc)
            preview = reply.replace("\n", " | ")[:200]
            tag = "OK " if err == 0 else f"err {err}"
            print(f"  [{tag:>7}] {cmd_text_str!r:<32}  -> {preview!r}")
            if err == 0:
                any_success = True
        if any_success:
            return  # Found a working encoding; no need to retry
    print("\n(All four encodings failed — including u32 which is known to work")
    print(" on kSA v6. Either the kSA state is unusual (e.g. no acquisition")
    print(" controller for measurement.* commands) or the command vocabulary")
    print(" is different in this build. Try scripts/test_ksa_single.py --enc u32")
    print(" with a single probe to isolate.)")


def cmd_get_ksa400_intensities(sock: socket.socket) -> dict | None:
    """Try to read all 7 kSA 400 intensity fields for all datasets/regions.

    Per section 5.6 example: GET_DATA_SPECIFIC with MeasID=100, SourceID=-1
    (all sources), MarkerCount=-1 (all datasets), FieldCount=7 (all fields).
    Only works if the acquisition is running.
    """
    # Build the command body:
    # MeasCount(2) [MeasID(2) SourceID(2) MarkerCount(2) FieldCount(4) [FieldID(4) IndexCount(2)]*]
    # We request 1 measurement, all sources, all markers, 7 fields, no indices.
    body = struct.pack("<H", 1)  # MeasCount
    body += struct.pack("<hhh", MEAS_ID_GENERIC_IMAGE, -1, -1)  # MeasID, SourceID, MarkerCount=-1
    body += struct.pack("<i", len(KSA400_FIELDS))  # FieldCount
    for field_id in KSA400_FIELDS:
        body += struct.pack("<ih", field_id, 0)  # FieldID, IndexCount=0

    send_command(sock, CMD_GET_DATA_SPECIFIC, body)
    cmd, err, data = recv_reply(sock)
    if err != 0:
        print(f"  GET_DATA_SPECIFIC ERROR: {err_name(err)} ({err})")
        return None
    # Per response spec: System Status (24 bytes) + payload
    if len(data) < 24:
        print(f"  Reply too short for status: {len(data)} bytes")
        return None
    op_status = struct.unpack_from("<H", data, 4)[0]
    print(f"  System status: {OP_STATUS.get(op_status, f'unknown({op_status})')}")
    if op_status != 2:
        print("  (acquisition not running — no intensity data to return)")
        return None
    # If acquisition IS running, parse the rest. Skipping full parse for v1 —
    # just show that we got more bytes than just the status.
    extra = len(data) - 24
    print(f"  {extra} additional bytes of measurement data — full parse TBD")
    return {"extra_bytes": extra}


BINARY_CMD_NAMES = {
    CMD_INITIALIZE: "INITIALIZE",
    CMD_SET_DATA_FIELDS: "SET_DATA_FIELDS",
    CMD_RUN: "RUN",
    CMD_GET_DATA: "GET_DATA",
    CMD_STOP: "STOP",
    CMD_GET_DATA_SPECIFIC: "GET_DATA_SPECIFIC",
    CMD_RESTART_GROWTHRATE_FIT: "RESTART_GROWTHRATE_FIT",
    CMD_OPEN_ACQUIRE: "OPEN_ACQUIRE",
    CMD_CLOSE_ACQUIRE: "CLOSE_ACQUIRE",
    CMD_GET_STATUS: "GET_STATUS",
    CMD_GET_APP_VERSION: "GET_APP_VERSION",
    CMD_TEXT_CMD: "TEXT_CMD",
}


def scan_binary_commands(sock: socket.socket) -> None:
    """Send each known binary command code with an empty body to map v6 coverage.

    Interpretation:
        err  0 → command alive, succeeded with no body
        err -2 → command code not implemented in this build
        err -3 → command code IS implemented but needs proper parameters
        err -4 → command alive but requires acquisition to be running first
        other → check spec section 9

    This is read-only insofar as the commands we're scanning don't take
    side-effecting parameters when sent empty — but if any of INITIALIZE,
    RUN, STOP, OPEN_ACQUIRE, CLOSE_ACQUIRE returns 0 here, we may have
    nudged kSA state. Watch the kSA UI for unexpected mode changes.
    """
    print("\n--- Binary command coverage scan (all empty bodies) ---")
    for code in sorted(BINARY_CMD_NAMES):
        name = BINARY_CMD_NAMES[code]
        try:
            send_command(sock, code, b"")
            cmd, err, data = recv_reply(sock)
            verdict = {
                0: "ALIVE (returned 0)",
                -2: "dead",
                -3: "ALIVE (needs args)",
                -4: "ALIVE (invalid state)",
            }.get(err, f"err {err}")
            print(f"  {code} {name:<24}  err={err:>3}  data={len(data):>3}B  -- {verdict}")
        except (ConnectionError, OSError) as e:
            print(f"  {code} {name:<24}  TRANSPORT ERROR: {e}")
            return


DISCOVERY_PROBES = [
    "?",                    # Generic help
    "help",                 # Help alias
    "version",              # Software version via text
    "status",               # System status via text
    "measurement ?",        # Discover measurement subcommands
    "measurement sources",  # List sources (kSA 400 should report 1)
    "measurement acquire ?",
    "marker ?",             # Markers = ROI definitions
    "dataset ?",            # Datasets = computed regions on markers
    "field ?",              # Field IDs
]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1", help="kSA host (default: localhost)")
    parser.add_argument("--port", type=int, default=1800, help="kSA TCP port (default: 1800)")
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument(
        "--text-cmd",
        help="Send a single ad-hoc TEXT_CMD instead of the discovery batch",
    )
    parser.add_argument(
        "--skip-text",
        action="store_true",
        help="Skip TEXT_CMD discovery (binary smoke test only)",
    )
    parser.add_argument(
        "--scan-binary",
        action="store_true",
        help="Probe all known binary command codes with empty bodies",
    )
    args = parser.parse_args()

    print(f"Connecting to kSA at {args.host}:{args.port}...")
    try:
        sock = socket.create_connection((args.host, args.port), timeout=args.timeout)
    except (ConnectionRefusedError, socket.timeout, OSError) as e:
        print(f"\nFAILED to connect: {e}")
        print("\nMost likely causes:")
        print("  - kSA isn't running")
        print("  - kSA Chamber Interface isn't set to kSAComm (Options → General → Chamber Interface)")
        print("  - kSA wasn't restarted after the config change")
        print("  - Port mismatch (expected 1800; check kSA Options dialog)")
        return 1

    try:
        # --- Handshake ---
        print("\nHandshake:")
        print('  → "ksacomm_client"')
        send_short_string(sock, "ksacomm_client")
        server_str = recv_short_string(sock)
        version_bytes = b""
        while len(version_bytes) < 2:
            chunk = sock.recv(2 - len(version_bytes))
            if not chunk:
                raise ConnectionError("Server closed during version read")
            version_bytes += chunk
        version = struct.unpack("<H", version_bytes)[0]
        print(f"  ← {server_str!r}  (protocol version {version})")
        if server_str != "ksacomm_server":
            print("  ⚠ unexpected server handshake string")
            return 2

        # --- GET_APP_VERSION ---
        print("\nGET_APP_VERSION:")
        ver = cmd_get_app_version(sock)
        if ver is not None:
            print(f"  → {ver}")

        # --- GET_STATUS ---
        print("\nGET_STATUS:")
        status = cmd_get_status(sock)
        if status:
            print(f"  Operational: {status['op_status_name']}")
            print(f"  RPM:         {status['rpm']:.2f} ({status['rpm_status_name']})")

        # --- GET_DATA_SPECIFIC for kSA 400 intensities ---
        print("\nGET_DATA_SPECIFIC (kSA 400 intensities, all fields):")
        cmd_get_ksa400_intensities(sock)

        # --- Binary command coverage scan (optional) ---
        if args.scan_binary:
            scan_binary_commands(sock)

        # --- TEXT_CMD: ad-hoc or discovery batch ---
        if not args.skip_text:
            if args.text_cmd:
                print(f"\nTEXT_CMD (ad-hoc): {args.text_cmd!r}")
                for enc in ("raw", "short", "u16"):
                    err, reply = cmd_text(sock, args.text_cmd, encoding=enc)
                    tag = "OK " if err == 0 else f"err {err}"
                    print(f"  [{enc:>5}] [{tag:>7}]  -> {reply!r}")
            else:
                probe_text_commands(sock, DISCOVERY_PROBES)

        print("\n✅ Smoke test complete — kSAComm interface is alive and responding.")
        return 0
    finally:
        sock.close()


if __name__ == "__main__":
    sys.exit(main())
