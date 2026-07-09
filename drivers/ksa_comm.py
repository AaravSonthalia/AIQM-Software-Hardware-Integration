"""kSAComm TCP/IP client — direct-read protocol for kSA acquisition systems.

Reverse-engineered from kSAcomm Protocol V4.1 (May 2021) with v6-specific
behavior observed on Bulbasaur's kSA 400 install (May 20 2026 breakthrough,
re-confirmed Jul 7 2026). Wire format details in the
``ksa_tcp_ip_breakthrough_may20.md`` memory file.

Typical usage:

    from drivers.ksa_comm import KsaCommClient

    with KsaCommClient(host="127.0.0.1", port=1800) as client:
        version = client.get_app_version()
        status = client.get_status()  # dict: op_status_name, rpm, ...
        err, reply = client.send_text("version")

Error classification (Jul 7 taxonomy) via :func:`classify_error`:

    err, reply = client.send_text("app query SQL \\\"SELECT 1\\\"")
    if err != ERR_OK:
        category = classify_error(reply)
        # -> KsaErrorCategory.SESSION_REQUIRED (start an acquisition)

The scripts under ``scripts/test_ksa_comm.py`` and
``scripts/test_ksa_single.py`` predate this driver and duplicate its
wire-protocol helpers. They still work; a future refactor could
consolidate them onto this client.
"""
from __future__ import annotations

import enum
import socket
import struct
from typing import Optional

# ---------------------------------------------------------------------------
# Wire-protocol constants
# ---------------------------------------------------------------------------

# Handshake fixed strings (short-string format: [Len:1][chars:Len-1][null:1]).
HANDSHAKE_CLIENT_STR = "ksacomm_client"
HANDSHAKE_SERVER_STR = "ksacomm_server"

# Binary command codes per protocol section 5.
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

# Error codes per protocol section 9.
ERR_OK = 0
ERR_GENERAL = -1
ERR_UNKNOWN = -2
ERR_PARAM = -3
ERR_STATE = -4

ERR_NAMES: dict[int, str] = {
    ERR_OK: "Success",
    ERR_GENERAL: "General error",
    ERR_UNKNOWN: "Unknown command",
    ERR_PARAM: "Invalid parameter",
    ERR_STATE: "Invalid state",
}

# Operational status codes (GET_STATUS reply, protocol section 5.10).
OP_STATUS: dict[int, str] = {
    0: "Not operational",
    1: "Idle",
    2: "Acquiring",
    3: "Paused",
}

RPM_STATUS: dict[int, str] = {
    0: "Unstable",
    1: "Stable",
    2: "Artificial",
}

# Default connection endpoint (localhost kSA install, factory port).
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 1800
DEFAULT_TIMEOUT = 5.0


# ---------------------------------------------------------------------------
# Error taxonomy — Jul 7 2026 vocabulary
# ---------------------------------------------------------------------------

class KsaErrorCategory(enum.Enum):
    """5 categories of server-side error, each implying a different fix.

    kSA returns ``err=-1`` (General error) with a text payload naming the
    specific gate. Clients can classify by phrase without hardcoding
    string matches at every call site — see :func:`classify_error`.

    Categories map 1:1 to the Jul 7 2026 taxonomy in
    ``ksa_tcp_ip_breakthrough_may20.md``.
    """

    WRONG_VOCABULARY = "vocabulary"
    """Command not recognized at all (bad root or unhandled sub-command).
    Fix is client-side: correct the command string."""

    NOT_INSTANTIATED = "not_instantiated"
    """Recognized but the runtime module isn't started. Fix: start an
    acquisition in the kSA UI (auto-instantiates measurement / acquire /
    alarm / laser managers)."""

    NOT_IN_BUILD = "not_in_build"
    """Feature not present in this kSA build. Fix: none — the install
    lacks the subsystem (``mfc`` is the observed example on kSA 400)."""

    DIALOG_CLOSED = "dialog_closed"
    """Requires an open GUI dialog. Fix: open the required Options
    dialog in the kSA UI (e.g. Options → Electron Gun Control)."""

    SESSION_REQUIRED = "session_required"
    """Command dispatched but the backing database file is missing.
    Fix: start an acquisition to create the ``.kdt`` file."""

    UNCLASSIFIED = "unclassified"
    """Response text didn't match any known category. New taxonomy
    entry candidate — worth surfacing to the kSA memory file for
    the next update."""


# Ordering matters here — check the specific phrases first so that
# ``manager not available`` and ``dialog not available`` don't collide
# with the bare ``not available`` fallback that means NOT_IN_BUILD.
_ERROR_PHRASES: tuple[tuple[str, KsaErrorCategory], ...] = (
    ("unsupported root command", KsaErrorCategory.WRONG_VOCABULARY),
    ("unhandled command", KsaErrorCategory.WRONG_VOCABULARY),
    ("manager not available", KsaErrorCategory.NOT_INSTANTIATED),
    ("dialog not available", KsaErrorCategory.DIALOG_CLOSED),
    ("controller not found", KsaErrorCategory.NOT_INSTANTIATED),
    ("cannot find the file specified", KsaErrorCategory.SESSION_REQUIRED),
    # Bare "not available" comes last — it's the feature-absent build
    # signal only after the more-specific manager/dialog variants have
    # been ruled out.
    ("not available", KsaErrorCategory.NOT_IN_BUILD),
)


def classify_error(reply_text: str) -> KsaErrorCategory:
    """Classify a server-side error phrase into one of five actionable
    categories, or :attr:`KsaErrorCategory.UNCLASSIFIED` if none match.

    Matching is case-insensitive substring — kSA's phrasing is
    consistent enough per the Jul 7 taxonomy that literal substrings
    are safe. A drift here would surface via the UNCLASSIFIED bucket
    rather than a wrong category.
    """
    text = reply_text.lower()
    for phrase, category in _ERROR_PHRASES:
        if phrase in text:
            return category
    return KsaErrorCategory.UNCLASSIFIED


def err_name(code: int) -> str:
    """Human-readable name for a wire-protocol error code."""
    return ERR_NAMES.get(code, f"unknown({code})")


# ---------------------------------------------------------------------------
# Wire-protocol helpers (module-level; scripts + client both share them)
# ---------------------------------------------------------------------------

def send_short_string(sock: socket.socket, s: str) -> None:
    """Send a short-string frame: ``[Len:1][chars:Len-1][null:1]``.

    ``Len`` includes the trailing null. Used only for the handshake in
    v6, but retained as a public helper for smoke-test scripts that
    exercise the same wire format.
    """
    payload = s.encode("ascii") + b"\x00"
    sock.sendall(struct.pack("B", len(payload)) + payload)


def recv_short_string(sock: socket.socket) -> str:
    """Receive a short-string frame; return the ASCII text without the
    trailing null."""
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
    return buf.rstrip(b"\x00").decode("ascii", errors="replace")


def send_command(
    sock: socket.socket,
    cmd_code: int,
    cmd_data: bytes = b"",
) -> None:
    """Send a binary command frame: ``[CmdCode:2 LE][CmdLength:2 LE][data]``."""
    msg = struct.pack("<HH", cmd_code, len(cmd_data)) + cmd_data
    sock.sendall(msg)


def recv_reply(sock: socket.socket) -> tuple[int, int, bytes]:
    """Receive a binary reply frame; return ``(cmd_code, err_code, data)``.

    Frame: ``[CmdCode:2 LE][ErrCode:2 LE int16][ReplyLen:2 LE][ReplyData]``.
    ``ErrCode`` is signed 16-bit — negative values (e.g. ``-1`` General
    error) carry descriptive text in ``ReplyData``.
    """
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


# ---------------------------------------------------------------------------
# TEXT_CMD encoding + decoding (pure functions, testable without a socket)
# ---------------------------------------------------------------------------

def encode_text_body(text: str, encoding: str = "u32") -> bytes:
    """Encode a TEXT_CMD body per one of four candidate encodings.

    The "u32" encoding is the v6-working format per the May 20 2026
    breakthrough (re-confirmed Jul 7 2026). The other three
    ("raw", "short", "u16") are retained as diagnostic negatives — on
    kSA v6 they return ``err=-2 Unknown command``, and that specific
    error being returned is itself informative if we ever debug
    encoding drift.

    Raises ``ValueError`` for unknown encoding strings.
    """
    if encoding == "raw":
        return text.encode("ascii")
    if encoding == "short":
        payload = text.encode("ascii") + b"\x00"
        return struct.pack("B", len(payload)) + payload
    if encoding == "u16":
        chars = text.encode("ascii")
        return struct.pack("<H", len(chars)) + chars
    if encoding == "u32":
        payload = text.encode("ascii") + b"\x00"
        return struct.pack("<I", len(payload)) + payload
    raise ValueError(f"unknown encoding {encoding!r}")


def decode_text_reply(data: bytes) -> str:
    """Decode a TEXT_CMD reply payload.

    Reply format is ``[u32 length][chars][null]`` regardless of the
    request encoding. Length includes the trailing null. The same shape
    applies to both success (``err=0``) and error replies (``err=-1``,
    ``err=-2``), so error-text extraction goes through this same
    function.

    Returns ``""`` if the reply is too short to hold the u32 prefix
    (defensive; ordinarily kSA either returns a full frame or the
    ``recv_reply`` helper raises).
    """
    if len(data) >= 4:
        return data[4:].decode("ascii", errors="replace").rstrip("\x00")
    return ""


# ---------------------------------------------------------------------------
# Client class
# ---------------------------------------------------------------------------

class KsaCommClient:
    """TCP/IP client for the kSAComm interface (protocol v6 tested).

    Manages the socket + handshake lifecycle. Provides typed methods for
    the binary commands validated on Bulbasaur (``GET_APP_VERSION``,
    ``GET_STATUS``) and the ``TEXT_CMD`` path with u32 encoding.

    Lifecycle::

        client = KsaCommClient()
        client.connect()          # opens socket, does handshake
        try:
            version = client.get_app_version()
            err, reply = client.send_text("version")
        finally:
            client.disconnect()   # closes socket

    Or as a context manager::

        with KsaCommClient() as client:
            version = client.get_app_version()

    The client does no reconnection logic — a dropped socket
    (``ConnectionError`` from ``send_binary``/``send_text``) is the
    caller's responsibility to catch and re-connect. Add reconnection
    at the worker level rather than smuggling it into the client.
    """

    def __init__(
        self,
        host: str = DEFAULT_HOST,
        port: int = DEFAULT_PORT,
        timeout: float = DEFAULT_TIMEOUT,
    ):
        self.host = host
        self.port = port
        self.timeout = timeout
        self._sock: Optional[socket.socket] = None
        self._server_protocol_version: Optional[int] = None
        self._connected = False

    def connect(self) -> None:
        """Open the socket and run the handshake.

        Raises ``ConnectionError`` on a wire-level failure (socket
        error, unexpected server handshake string, short read on the
        version bytes). Sets :attr:`connected` on success.

        Idempotent when already connected — returns immediately.
        """
        if self._connected:
            return
        self._sock = socket.create_connection(
            (self.host, self.port), timeout=self.timeout,
        )
        send_short_string(self._sock, HANDSHAKE_CLIENT_STR)
        server_str = recv_short_string(self._sock)
        if server_str != HANDSHAKE_SERVER_STR:
            self._sock.close()
            self._sock = None
            raise ConnectionError(
                f"Unexpected server handshake: {server_str!r} "
                f"(expected {HANDSHAKE_SERVER_STR!r})"
            )
        # Server follows its handshake string with a 2-byte protocol
        # version. On Bulbasaur's kSA 400 this is 6.
        version_bytes = b""
        while len(version_bytes) < 2:
            chunk = self._sock.recv(2 - len(version_bytes))
            if not chunk:
                raise ConnectionError("Server closed during version read")
            version_bytes += chunk
        self._server_protocol_version = struct.unpack("<H", version_bytes)[0]
        self._connected = True

    def disconnect(self) -> None:
        """Close the socket. Safe to call when not connected."""
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
        self._sock = None
        self._connected = False
        self._server_protocol_version = None

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def protocol_version(self) -> Optional[int]:
        """The server's protocol version received during handshake.
        ``None`` when not connected. Expected value 6 on Bulbasaur's
        kSA 400."""
        return self._server_protocol_version

    def send_binary(
        self,
        cmd_code: int,
        body: bytes = b"",
    ) -> tuple[int, bytes]:
        """Send a binary command; return ``(err_code, reply_data)``.

        Raises ``ConnectionError`` if not connected or on socket
        failure. Non-zero ``err_code`` values are returned as-is — the
        caller decides what to do with them (see :func:`classify_error`
        for text-error classification).
        """
        if not self._connected or self._sock is None:
            raise ConnectionError("Not connected")
        send_command(self._sock, cmd_code, body)
        _cmd, err, data = recv_reply(self._sock)
        return err, data

    def send_text(
        self,
        text: str,
        encoding: str = "u32",
    ) -> tuple[int, str]:
        """Send a ``TEXT_CMD`` (1011); return ``(err_code, reply_text)``.

        Default encoding "u32" is the v6-working format. Reply is
        decoded uniformly regardless of request encoding — the server
        always u32-prefixes text payloads.

        Error replies (``err=-1``, ``err=-2``) also carry text payloads
        that describe the specific gate; feed the ``reply_text`` through
        :func:`classify_error` to route them into one of five actionable
        categories.
        """
        body = encode_text_body(text, encoding)
        err, data = self.send_binary(CMD_TEXT_CMD, body)
        return err, decode_text_reply(data)

    def get_app_version(self) -> Optional[str]:
        """``GET_APP_VERSION`` (1010) — the application version string.

        Returns ``None`` if the command errored. Reply format is
        ``[len:1][chars:len][padding]``.
        """
        err, data = self.send_binary(CMD_GET_APP_VERSION)
        if err != ERR_OK:
            return None
        if not data:
            return ""
        ver_len = data[0]
        return (
            data[1 : 1 + ver_len]
            .rstrip(b"\x00")
            .decode("ascii", errors="replace")
        )

    def get_status(self) -> Optional[dict]:
        """``GET_STATUS`` (1009) — a dict with operational + RPM info.

        Returns ``None`` if the command errored or the reply is
        shorter than the 24-byte status frame (defensive; ordinarily
        kSA either returns a full frame or ``recv_reply`` raises).

        Dict keys: ``status_size``, ``status_version``, ``op_status``,
        ``op_status_name`` (human-readable via :data:`OP_STATUS`),
        ``last_home_pulse``, ``rpm_status``, ``rpm_status_name``
        (via :data:`RPM_STATUS`), ``rpm``.
        """
        err, data = self.send_binary(CMD_GET_STATUS)
        if err != ERR_OK:
            return None
        if len(data) < 24:
            return None
        # Reply format per protocol section 5.10:
        #   StatusSize(2) StatusVersion(2) OpStatus(2)
        #   LastHome(8)   RpmStatus(2)     Rpm(8)
        status_size, status_version, op_status = struct.unpack_from(
            "<HHH", data, 0,
        )
        last_home = struct.unpack_from("<d", data, 6)[0]
        rpm_status = struct.unpack_from("<H", data, 14)[0]
        rpm = struct.unpack_from("<d", data, 16)[0]
        return {
            "status_size": status_size,
            "status_version": status_version,
            "op_status": op_status,
            "op_status_name": OP_STATUS.get(
                op_status, f"unknown({op_status})",
            ),
            "last_home_pulse": last_home,
            "rpm_status": rpm_status,
            "rpm_status_name": RPM_STATUS.get(
                rpm_status, f"unknown({rpm_status})",
            ),
            "rpm": rpm,
        }

    def __enter__(self) -> "KsaCommClient":
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.disconnect()
