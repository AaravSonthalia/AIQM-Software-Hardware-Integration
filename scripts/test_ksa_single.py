#!/usr/bin/env python3
"""kSAComm — send EXACTLY one command per invocation, then disconnect.

This is the surgical complement to test_ksa_comm.py. Each invocation does:
  1. open TCP connection
  2. send the handshake
  3. send exactly ONE command (binary or TEXT_CMD)
  4. read the reply, print it
  5. disconnect

Pair this with the kSA Command Log dialog (View -> Command Log) — after
each run, copy the latest entries from the log so we can see what kSA
actually parsed from our packet.

Examples:
    # Sanity: handshake only, no command
    python scripts/test_ksa_single.py --handshake-only

    # Binary command, empty body
    python scripts/test_ksa_single.py --binary 1010

    # Binary command, raw-hex body
    python scripts/test_ksa_single.py --binary 1005 --body-hex 01000064FFFFFFFF

    # TEXT_CMD (1011) with various encodings of the same text
    python scripts/test_ksa_single.py --text-cmd "measurement available" --enc raw
    python scripts/test_ksa_single.py --text-cmd "?" --enc short
    python scripts/test_ksa_single.py --text-cmd "version" --enc u16

    # Try a TEXT_CMD via a DIFFERENT command code (in case v6 renumbered it)
    python scripts/test_ksa_single.py --binary 1012 --text-payload "?"
    python scripts/test_ksa_single.py --binary 2011 --text-payload "?"
"""
from __future__ import annotations

import argparse
import socket
import struct
import sys


def send_short_string(sock: socket.socket, s: str) -> None:
    payload = s.encode("ascii") + b"\x00"
    sock.sendall(struct.pack("B", len(payload)) + payload)


def recv_short_string(sock: socket.socket) -> str:
    n = sock.recv(1)[0]
    buf = b""
    while len(buf) < n:
        buf += sock.recv(n - len(buf))
    return buf.rstrip(b"\x00").decode("ascii", errors="replace")


def send_command(sock: socket.socket, cmd_code: int, body: bytes) -> None:
    sock.sendall(struct.pack("<HH", cmd_code, len(body)) + body)


def recv_reply(sock: socket.socket, timeout: float = 5.0) -> tuple[int, int, bytes]:
    sock.settimeout(timeout)
    header = b""
    while len(header) < 6:
        chunk = sock.recv(6 - len(header))
        if not chunk:
            raise ConnectionError("server closed mid-header")
        header += chunk
    cmd_code, err_code, reply_length = struct.unpack("<HhH", header)
    data = b""
    while len(data) < reply_length:
        chunk = sock.recv(reply_length - len(data))
        if not chunk:
            raise ConnectionError("server closed mid-data")
        data += chunk
    return cmd_code, err_code, data


def encode_text_body(text: str, enc: str) -> bytes:
    """Encode a TEXT_CMD body using one of four candidate formats.

    May 20 2026 finding from kSA Command Log: TEXT_CMD expects a 4-byte
    (u32 LE) length prefix for the Char[1] array — i.e. ``enc="u32"``.
    The other three are kept for diagnostics.
    """
    if enc == "raw":
        return text.encode("ascii")
    if enc == "short":
        payload = text.encode("ascii") + b"\x00"
        return struct.pack("B", len(payload)) + payload
    if enc == "u16":
        chars = text.encode("ascii")
        return struct.pack("<H", len(chars)) + chars
    if enc == "u32":
        chars = text.encode("ascii")
        return struct.pack("<I", len(chars)) + chars
    raise ValueError(f"unknown enc {enc!r}")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=1800)
    p.add_argument("--timeout", type=float, default=5.0)
    p.add_argument(
        "--handshake-only",
        action="store_true",
        help="Connect + handshake, then disconnect (no command sent)",
    )
    p.add_argument("--binary", type=int, help="Binary command code to send (1000-9999)")
    p.add_argument(
        "--body-hex",
        default="",
        help="Hex bytes for the binary command body (e.g. 01000064FFFFFFFF)",
    )
    p.add_argument(
        "--text-cmd",
        help="Send TEXT_CMD (1011) with this text payload",
    )
    p.add_argument(
        "--text-payload",
        help="Use --binary to set a custom code AND send text body (probes whether "
        "v6 renumbered TEXT_CMD); use with --enc",
    )
    p.add_argument(
        "--enc",
        default="u32",
        choices=["raw", "short", "u16", "u32"],
        help="TEXT_CMD payload encoding (default u32 — confirmed correct May 20 2026)",
    )
    args = p.parse_args()

    print(f"# Connecting to {args.host}:{args.port}")
    try:
        sock = socket.create_connection((args.host, args.port), timeout=args.timeout)
    except (ConnectionRefusedError, OSError) as e:
        print(f"# CONNECT FAILED: {e}")
        return 1

    try:
        # Handshake (always required)
        send_short_string(sock, "ksacomm_client")
        server_str = recv_short_string(sock)
        version_bytes = b""
        while len(version_bytes) < 2:
            version_bytes += sock.recv(2 - len(version_bytes))
        version = struct.unpack("<H", version_bytes)[0]
        print(f"# Handshake: {server_str!r} v{version}")

        if args.handshake_only:
            print("# (handshake only — no command sent)")
            return 0

        # Build the command
        if args.text_cmd is not None:
            cmd_code = 1011  # TEXT_CMD per V4.1 spec
            body = encode_text_body(args.text_cmd, args.enc)
            print(f"# Sending TEXT_CMD (1011): {args.text_cmd!r} enc={args.enc} body={body.hex()}")
        elif args.binary is not None:
            cmd_code = args.binary
            if args.text_payload is not None:
                body = encode_text_body(args.text_payload, args.enc)
                print(
                    f"# Sending binary code {cmd_code} with text payload "
                    f"{args.text_payload!r} enc={args.enc} body={body.hex()}"
                )
            else:
                body = bytes.fromhex(args.body_hex) if args.body_hex else b""
                print(f"# Sending binary code {cmd_code} body={body.hex() or '(empty)'}")
        else:
            print("# ERROR: specify --binary <code>, --text-cmd <text>, or --handshake-only")
            return 2

        send_command(sock, cmd_code, body)

        try:
            cmd, err, data = recv_reply(sock, timeout=args.timeout)
        except (ConnectionError, OSError, socket.timeout) as e:
            print(f"# REPLY ERROR: {type(e).__name__}: {e}")
            print("# -> kSA likely crashed or refused to respond. Check Command Log.")
            return 3

        print(f"# Reply: cmd_code={cmd} err={err} data_len={len(data)}")
        if data:
            print(f"#   data hex: {data.hex()}")
            try:
                ascii_repr = data.decode("ascii", errors="replace").rstrip("\x00")
                print(f"#   data ascii: {ascii_repr!r}")
            except Exception:
                pass
        return 0
    finally:
        sock.close()


if __name__ == "__main__":
    sys.exit(main())
