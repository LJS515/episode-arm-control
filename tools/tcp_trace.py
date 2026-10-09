#!/usr/bin/env python3
"""Transparent TCP proxy that logs length-prefixed frames as JSON lines.

Use it between any client (official demo, our SDK) and the official upper
computer to *observe* the real protocol without modifying either side::

    python tools/tcp_trace.py --listen 127.0.0.1:12346 --target 127.0.0.1:12345 --out trace.jsonl
    # then point the client at port 12346

Each frame is logged with direction, monotonic timestamp, length and — if
the payload is UTF-8 JSON — the decoded object. Non-JSON payloads (e.g. the
historical pickle protocol) are logged as length + first bytes in hex and are
NEVER deserialised.

Output is raw observation data; label any derived numbers with the
environment they came from (Mock / real hardware).
"""

from __future__ import annotations

import argparse
import json
import socket
import struct
import sys
import threading
import time
from pathlib import Path
from typing import IO, Optional

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

MAX_FRAME = 16 * 1024 * 1024


class FrameSniffer:
    """Incremental frame parser for one direction of a byte stream."""

    def __init__(self, direction: str, sink, conn_id: int):
        self.direction = direction
        self.sink = sink
        self.conn_id = conn_id
        self.buf = bytearray()
        self.desync = False

    def feed(self, data: bytes) -> None:
        if self.desync:
            return
        self.buf.extend(data)
        while len(self.buf) >= 8:
            (n,) = struct.unpack(">Q", self.buf[:8])
            if n > MAX_FRAME:
                self.sink({"conn": self.conn_id, "dir": self.direction, "event": "desync", "announced_len": n})
                self.desync = True
                return
            if len(self.buf) < 8 + n:
                return
            payload = bytes(self.buf[8 : 8 + n])
            del self.buf[: 8 + n]
            rec = {"conn": self.conn_id, "dir": self.direction, "len": n}
            try:
                rec["json"] = json.loads(payload.decode("utf-8"))
            except (UnicodeDecodeError, ValueError):
                rec["non_json_hex_prefix"] = payload[:32].hex()
            self.sink(rec)


class Tracer:
    def __init__(self, out: IO[str]):
        self.out = out
        self.lock = threading.Lock()
        self.t0 = time.monotonic()

    def __call__(self, rec: dict) -> None:
        rec = {"t": round(time.monotonic() - self.t0, 6), **rec}
        with self.lock:
            self.out.write(json.dumps(rec, ensure_ascii=False) + "\n")
            self.out.flush()


def pump(src: socket.socket, dst: socket.socket, sniffer: FrameSniffer) -> None:
    try:
        while True:
            data = src.recv(65536)
            if not data:
                break
            sniffer.feed(data)
            dst.sendall(data)
    except OSError:
        pass
    finally:
        for s in (src, dst):
            try:
                s.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass


def serve(listen: tuple, target: tuple, tracer: Tracer, stop: Optional[threading.Event] = None) -> None:
    srv = socket.socket()
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(listen)
    srv.listen(8)
    srv.settimeout(0.5)
    cid = 0
    while stop is None or not stop.is_set():
        try:
            c, _ = srv.accept()
        except socket.timeout:
            continue
        cid += 1
        try:
            u = socket.create_connection(target, timeout=3)
        except OSError as exc:
            tracer({"conn": cid, "event": "upstream_connect_failed", "error": str(exc)})
            c.close()
            continue
        tracer({"conn": cid, "event": "open"})
        threading.Thread(target=pump, args=(c, u, FrameSniffer("c2s", tracer, cid)), daemon=True).start()
        threading.Thread(target=pump, args=(u, c, FrameSniffer("s2c", tracer, cid)), daemon=True).start()
    srv.close()


def _addr(s: str) -> tuple:
    host, port = s.rsplit(":", 1)
    return host, int(port)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--listen", default="127.0.0.1:12346")
    ap.add_argument("--target", default="127.0.0.1:12345")
    ap.add_argument("--out", default="-")
    args = ap.parse_args()
    out = sys.stdout if args.out == "-" else open(args.out, "a", encoding="utf-8")
    print(f"tracing {args.listen} -> {args.target}", file=sys.stderr)
    try:
        serve(_addr(args.listen), _addr(args.target), Tracer(out))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
