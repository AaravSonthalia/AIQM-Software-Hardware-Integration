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


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1", help="kSA host (default: localhost)")
    parser.add_argument("--port", type=int, default=1800, help="kSA TCP port (default: 1800)")
    parser.add_argument("--timeout", type=float, default=5.0)
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

        print("\n✅ Smoke test passed — kSAComm interface is alive and responding.")
        return 0
    finally:
        sock.close()


if __name__ == "__main__":
    sys.exit(main())
