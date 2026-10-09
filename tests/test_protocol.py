"""Framing / codec tests. Covers: header fragmentation (1), JSON fragmentation (2),
truncation (3), invalid JSON (4), oversize (5), receive timeout, peer close (8), NaN/Inf (10)."""

import socket
import struct
import threading
import time

import pytest

from episode_sdk import protocol
from episode_sdk.exceptions import (
    ConnectionClosedError,
    FrameTooLargeError,
    MalformedPayloadError,
    ReceiveTimeoutError,
    TruncatedFrameError,
)


@pytest.fixture
def pair():
    a, b = socket.socketpair()
    yield a, b
    a.close()
    b.close()


def send_in_pieces(sock, data, sizes, pause=0.01):
    def worker():
        i = 0
        for s in sizes:
            sock.sendall(data[i : i + s])
            i += s
            time.sleep(pause)
        if i < len(data):
            sock.sendall(data[i:])

    th = threading.Thread(target=worker)
    th.start()
    return th


def test_roundtrip_encode_decode():
    obj = {"action": "angle_mode", "params": [[1.5, 2, 3, 4, 5, 6], 0.5], "中文": "ok"}
    frame = protocol.encode_frame(obj)
    (n,) = struct.unpack(">Q", frame[:8])
    assert n == len(frame) - 8
    assert protocol.decode_payload(frame[8:]) == obj


def test_header_is_8_byte_big_endian():
    frame = protocol.encode_frame({"a": 1})
    assert frame[:8] == len(b'{"a":1}').to_bytes(8, "big")


def test_header_fragmented_byte_by_byte(pair):
    a, b = pair
    frame = protocol.encode_frame({"action": "get_motor_angles", "params": None})
    th = send_in_pieces(a, frame, [1] * 8)
    msg = protocol.read_message(b, deadline=protocol.Deadline(2.0))
    th.join()
    assert msg["action"] == "get_motor_angles"


def test_json_payload_fragmented(pair):
    a, b = pair
    frame = protocol.encode_frame({"x": list(range(200))})
    th = send_in_pieces(a, frame, [8, 3, 5, 7, 11, 13, 50])
    msg = protocol.read_message(b, deadline=protocol.Deadline(2.0))
    th.join()
    assert msg["x"][-1] == 199


def test_recv_exact_returns_exact_bytes(pair):
    a, b = pair
    a.sendall(b"abcdefghij")
    assert protocol.recv_exact(b, 4) == b"abcd"
    assert protocol.recv_exact(b, 6) == b"efghij"


def test_truncated_header(pair):
    a, b = pair
    a.sendall(b"\x00\x00\x00")
    a.shutdown(socket.SHUT_WR)
    with pytest.raises(TruncatedFrameError):
        protocol.read_message(b, deadline=protocol.Deadline(1.0))


def test_truncated_payload(pair):
    a, b = pair
    frame = protocol.encode_frame({"k": "v" * 50})
    a.sendall(frame[:20])
    a.shutdown(socket.SHUT_WR)
    with pytest.raises(TruncatedFrameError):
        protocol.read_message(b, deadline=protocol.Deadline(1.0))


def test_header_then_close_is_truncation(pair):
    a, b = pair
    a.sendall(struct.pack(">Q", 10))
    a.shutdown(socket.SHUT_WR)
    with pytest.raises(TruncatedFrameError):
        protocol.read_message(b, deadline=protocol.Deadline(1.0))


def test_peer_closed_before_anything(pair):
    a, b = pair
    a.shutdown(socket.SHUT_WR)
    with pytest.raises(ConnectionClosedError):
        protocol.read_message(b, deadline=protocol.Deadline(1.0))


def test_invalid_json(pair):
    a, b = pair
    payload = b"{bad json"
    a.sendall(struct.pack(">Q", len(payload)) + payload)
    with pytest.raises(MalformedPayloadError):
        protocol.read_message(b, deadline=protocol.Deadline(1.0))


def test_invalid_utf8(pair):
    a, b = pair
    payload = b"\xff\xfe\xfd"
    a.sendall(struct.pack(">Q", len(payload)) + payload)
    with pytest.raises(MalformedPayloadError):
        protocol.read_message(b, deadline=protocol.Deadline(1.0))


def test_empty_payload_rejected(pair):
    a, b = pair
    a.sendall(struct.pack(">Q", 0))
    with pytest.raises(MalformedPayloadError):
        protocol.read_message(b, deadline=protocol.Deadline(1.0))


def test_oversize_incoming_frame_rejected_before_reading(pair):
    a, b = pair
    a.sendall(struct.pack(">Q", 1 << 40))
    with pytest.raises(FrameTooLargeError):
        protocol.read_message(b, max_size=1024, deadline=protocol.Deadline(1.0))


def test_oversize_outgoing_frame_rejected():
    with pytest.raises(FrameTooLargeError):
        protocol.encode_frame({"blob": "x" * 2000}, max_size=1000)


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
def test_non_finite_not_encodable(bad):
    with pytest.raises(MalformedPayloadError):
        protocol.encode_payload({"params": [[bad, 0, 0, 0, 0, 0], 1.0]})


@pytest.mark.parametrize("token", [b"NaN", b"Infinity", b"-Infinity"])
def test_non_finite_not_decodable(token):
    with pytest.raises(MalformedPayloadError):
        protocol.decode_payload(b"[1, " + token + b"]")


def test_receive_timeout(pair):
    _a, b = pair
    t0 = time.monotonic()
    with pytest.raises(ReceiveTimeoutError):
        protocol.read_message(b, deadline=protocol.Deadline(0.2))
    assert time.monotonic() - t0 < 1.0


def test_deadline_spans_multiple_chunks(pair):
    """A slow drip must not reset the timeout on every chunk."""
    a, b = pair
    frame = protocol.encode_frame({"x": "y" * 40})
    th = send_in_pieces(a, frame, [1] * 30, pause=0.05)  # ~1.5 s total
    with pytest.raises(ReceiveTimeoutError):
        protocol.read_message(b, deadline=protocol.Deadline(0.3))
    th.join()


def test_socket_closed_by_other_thread_maps_to_sdk_error():
    """Regression (found on Linux, ~1 in 7 runs): a socket closed by another thread
    while recv() blocks raised a raw OSError (EBADF) that bypassed the SDK errors."""
    a, b = socket.socketpair()
    try:
        b.close()
        with pytest.raises(ConnectionClosedError):
            protocol.recv_exact(b, 8, protocol.Deadline(1.0))
        with pytest.raises(ConnectionClosedError):
            protocol.send_all(b, b"x" * 8, protocol.Deadline(1.0))
    finally:
        a.close()


def test_concurrent_close_during_blocking_recv():
    a, b = socket.socketpair()
    errors = []

    def reader():
        try:
            protocol.recv_exact(b, 8, protocol.Deadline(5.0))
        except Exception as exc:  # noqa: BLE001
            errors.append(exc)

    th = threading.Thread(target=reader)
    th.start()
    time.sleep(0.2)
    try:
        b.shutdown(socket.SHUT_RDWR)
    except OSError:
        pass
    b.close()
    th.join(3)
    a.close()
    assert errors and isinstance(errors[0], (ConnectionClosedError, TruncatedFrameError))


def test_contains_non_finite():
    assert protocol.contains_non_finite({"a": [1, [2, float("nan")]]})
    assert not protocol.contains_non_finite({"a": [1, [2, 3.0]]})
