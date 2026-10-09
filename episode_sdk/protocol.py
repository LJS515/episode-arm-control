"""Length-prefixed UTF-8 JSON framing used by the Episode 1 TCP API.

Wire format (both directions)::

    +---------------------------+-------------------------------+
    | 8 bytes, unsigned, big    | N bytes UTF-8 JSON payload    |
    | endian payload length N   |                               |
    +---------------------------+-------------------------------+

Evidence
--------
* B (user-supplied description of the current official SDK): 8-byte
  big-endian length + UTF-8 JSON.
* C (public repo ``enpeizhao/episode_robot`` @ 9ede794, 2025-02-20): the
  same 8-byte big-endian header, but the payload was ``pickle``. This module
  never uses pickle — un-pickling network data allows arbitrary code
  execution.

Hardening relative to the historical implementation
---------------------------------------------------
* ``recv_exact`` loops until exactly ``n`` bytes arrived. The historical code
  called ``sock.recv(8)`` once for the header, which may legally return
  fewer than 8 bytes on a stream socket.
* A maximum frame size is enforced *before* allocating / reading the body.
* All socket operations obey an absolute deadline, so a silent peer cannot
  block the caller forever.
* JSON is encoded with ``allow_nan=False`` and decoded with NaN/Infinity
  rejected, because ``NaN`` is not valid JSON and is meaningless as a joint
  target.
"""

from __future__ import annotations

import json
import math
import socket
import struct
import time
from typing import Any, Optional

from .exceptions import (
    ConnectionClosedError,
    FrameTooLargeError,
    MalformedPayloadError,
    ReceiveTimeoutError,
    SendTimeoutError,
    TruncatedFrameError,
)

HEADER_STRUCT = struct.Struct(">Q")
HEADER_SIZE = HEADER_STRUCT.size  # 8
#: Default upper bound for one frame. A joint-angle reply is < 200 bytes; a
#: 4x4 matrix < 1 KiB. 1 MiB leaves ample room while bounding memory use.
DEFAULT_MAX_MESSAGE_SIZE = 1024 * 1024
PROTOCOL_VERSION = "1.0"


def _reject_constant(name: str) -> Any:
    raise MalformedPayloadError(f"non-finite JSON constant {name!r} is not allowed")


def encode_payload(obj: Any) -> bytes:
    """Serialise ``obj`` to compact UTF-8 JSON. NaN/Inf raise ValueError."""
    try:
        text = json.dumps(obj, ensure_ascii=False, allow_nan=False, separators=(",", ":"))
    except ValueError as exc:  # NaN / Inf
        raise MalformedPayloadError(f"payload contains non-finite number: {exc}") from exc
    except TypeError as exc:
        raise MalformedPayloadError(f"payload is not JSON-serialisable: {exc}") from exc
    return text.encode("utf-8")


def decode_payload(data: bytes) -> Any:
    """Parse a UTF-8 JSON payload, rejecting NaN/Infinity and bad UTF-8."""
    try:
        text = data.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise MalformedPayloadError(f"payload is not valid UTF-8: {exc}") from exc
    try:
        return json.loads(text, parse_constant=_reject_constant)
    except MalformedPayloadError:
        raise
    except json.JSONDecodeError as exc:
        raise MalformedPayloadError(f"payload is not valid JSON: {exc.msg} at pos {exc.pos}") from exc


def encode_frame(obj: Any, max_size: int = DEFAULT_MAX_MESSAGE_SIZE) -> bytes:
    payload = encode_payload(obj)
    if len(payload) > max_size:
        raise FrameTooLargeError(
            "outgoing payload exceeds limit", details={"size": len(payload), "max": max_size}
        )
    return HEADER_STRUCT.pack(len(payload)) + payload


def parse_header(header: bytes, max_size: int = DEFAULT_MAX_MESSAGE_SIZE) -> int:
    if len(header) != HEADER_SIZE:
        raise TruncatedFrameError("incomplete length header", details={"got": len(header)})
    (length,) = HEADER_STRUCT.unpack(header)
    if length > max_size:
        raise FrameTooLargeError(
            "incoming frame exceeds limit", details={"size": length, "max": max_size}
        )
    return length


# --------------------------------------------------------------------------- deadline helpers
class Deadline:
    """Absolute monotonic deadline. ``None`` timeout means wait forever."""

    __slots__ = ("_expires",)

    def __init__(self, timeout: Optional[float]):
        self._expires = None if timeout is None else time.monotonic() + max(0.0, timeout)

    def remaining(self) -> Optional[float]:
        if self._expires is None:
            return None
        return self._expires - time.monotonic()

    def expired(self) -> bool:
        rem = self.remaining()
        return rem is not None and rem <= 0.0

    @staticmethod
    def min(a: "Deadline", b: "Deadline") -> "Deadline":
        ra, rb = a.remaining(), b.remaining()
        if ra is None:
            return b
        if rb is None:
            return a
        return a if ra <= rb else b


def _apply_timeout(sock: socket.socket, deadline: Deadline, exc_type: type, what: str) -> None:
    rem = deadline.remaining()
    if rem is None:
        sock.settimeout(None)
        return
    if rem <= 0:
        raise exc_type(f"{what} deadline expired")
    sock.settimeout(rem)


def recv_exact(sock: socket.socket, n: int, deadline: Optional[Deadline] = None) -> bytes:
    """Read exactly ``n`` bytes or raise.

    * Peer closes before any byte  -> ConnectionClosedError
    * Peer closes mid-way          -> TruncatedFrameError
    * Deadline expires             -> ReceiveTimeoutError
    * Any other socket OSError     -> ConnectionClosedError (e.g. EBADF when another
      thread closed the socket). Raw OSError never escapes the SDK.
    """
    deadline = deadline or Deadline(None)
    buf = bytearray()
    while len(buf) < n:
        try:
            _apply_timeout(sock, deadline, ReceiveTimeoutError, "receive")
            chunk = sock.recv(n - len(buf))
        except socket.timeout as exc:
            raise ReceiveTimeoutError(
                "timed out waiting for data", details={"expected": n, "received": len(buf)}
            ) from exc
        except OSError as exc:  # reset, aborted, broken pipe, EBADF, ENOTCONN, ...
            raise ConnectionClosedError(f"connection lost while receiving: {exc}") from exc
        if not chunk:
            if not buf:
                raise ConnectionClosedError("peer closed the connection")
            raise TruncatedFrameError(
                "peer closed mid-frame", details={"expected": n, "received": len(buf)}
            )
        buf.extend(chunk)
    return bytes(buf)


def send_all(sock: socket.socket, data: bytes, deadline: Optional[Deadline] = None) -> None:
    deadline = deadline or Deadline(None)
    view = memoryview(data)
    while view:
        try:
            _apply_timeout(sock, deadline, SendTimeoutError, "send")
            sent = sock.send(view)
        except socket.timeout as exc:
            raise SendTimeoutError("timed out sending data") from exc
        except OSError as exc:  # reset, aborted, broken pipe, EBADF, ...
            raise ConnectionClosedError(f"connection lost while sending: {exc}") from exc
        if sent == 0:
            raise ConnectionClosedError("socket send returned 0")
        view = view[sent:]


def read_frame_bytes(
    sock: socket.socket, max_size: int = DEFAULT_MAX_MESSAGE_SIZE, deadline: Optional[Deadline] = None
) -> bytes:
    header = recv_exact(sock, HEADER_SIZE, deadline)
    length = parse_header(header, max_size)
    if length == 0:
        return b""
    try:
        return recv_exact(sock, length, deadline)
    except ConnectionClosedError as exc:
        # header arrived, body did not -> that is a truncated frame
        raise TruncatedFrameError("peer closed before payload", details={"expected": length}) from exc


def read_message(
    sock: socket.socket, max_size: int = DEFAULT_MAX_MESSAGE_SIZE, deadline: Optional[Deadline] = None
) -> Any:
    payload = read_frame_bytes(sock, max_size, deadline)
    if not payload:
        raise MalformedPayloadError("empty payload")
    return decode_payload(payload)


def write_message(
    sock: socket.socket, obj: Any, max_size: int = DEFAULT_MAX_MESSAGE_SIZE, deadline: Optional[Deadline] = None
) -> int:
    frame = encode_frame(obj, max_size)
    send_all(sock, frame, deadline)
    return len(frame)


def contains_non_finite(obj: Any) -> bool:
    """Recursively check for NaN/Inf floats (used by validation and tests)."""
    if isinstance(obj, float):
        return not math.isfinite(obj)
    if isinstance(obj, dict):
        return any(contains_non_finite(v) for v in obj.values())
    if isinstance(obj, (list, tuple)):
        return any(contains_non_finite(v) for v in obj)
    return False
