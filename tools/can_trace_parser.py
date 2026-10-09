#!/usr/bin/env python3
"""Parse CAN captures and print per-identifier statistics.

STATUS: tool implemented and tested on synthetic input only. No capture of
the real Episode 1 bus exists yet (pending hardware).

Supported inputs
----------------
* SocketCAN ``candump -l`` / ``candump -L`` lines::

      (1700000000.123456) can0 141#A100000000000000

* PEAK PCAN-View ``.trc`` v1.x / v2.x message lines, e.g.::

      1)      1.2  Rx         0141  8  A1 00 00 00 00 00 00 00       (v1.1)
      1       1.200 DT     0141 Rx 8  A1 00 00 00 00 00 00 00        (v2.x)

What it does NOT do: it does not decode the joint drivers' payloads. The
driver protocol (vendor firmware V5.1.5D per official forum, evidence B) is
not documented in our sources, and IDs/bit rate must be *observed*, never
assumed. The output answers "which IDs appear, how often, with what period
and jitter, in which order after a known TCP command" — the raw material for
docs/07_can_investigation.md.
"""

from __future__ import annotations

import argparse
import re
import statistics
import sys
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional

CANDUMP_RE = re.compile(r"^\((?P<t>\d+\.\d+)\)\s+(?P<if>\S+)\s+(?P<id>[0-9A-Fa-f]+)#(?P<data>[0-9A-Fa-fR]*)")
TRC_V1_RE = re.compile(r"^\s*\d+\)\s+(?P<t>[\d.]+)\s+(?P<dir>Rx|Tx)\s+(?P<id>[0-9A-Fa-f]+)\s+(?P<dlc>\d+)\s+(?P<data>(?:[0-9A-Fa-f]{2}\s*)*)")
TRC_V2_RE = re.compile(r"^\s*\d+\s+(?P<t>[\d.]+)\s+(?:DT|FD)\s+(?P<id>[0-9A-Fa-f]+)\s+(?P<dir>Rx|Tx)\s+(?P<dlc>\d+)\s+(?P<data>(?:[0-9A-Fa-f]{2}\s*)*)")


@dataclass
class CanFrame:
    t: float  # seconds
    can_id: int
    data: bytes
    direction: str = "?"
    extended: bool = False


@dataclass
class IdStats:
    can_id: int
    count: int = 0
    dlcs: set = field(default_factory=set)
    times: List[float] = field(default_factory=list)
    first_payload: Optional[bytes] = None
    directions: set = field(default_factory=set)


def parse_line(line: str) -> Optional[CanFrame]:
    line = line.rstrip()
    if not line or line.startswith(";"):
        return None
    m = CANDUMP_RE.match(line)
    if m:
        data = m.group("data")
        payload = b"" if data.startswith("R") else bytes.fromhex(data)
        ident = m.group("id")
        return CanFrame(float(m.group("t")), int(ident, 16), payload, "?", len(ident) > 3)
    for rx, scale in ((TRC_V2_RE, 1e-3), (TRC_V1_RE, 1e-3)):
        m = rx.match(line)
        if m:
            ident = m.group("id")
            data = bytes.fromhex(m.group("data").replace(" ", ""))
            return CanFrame(float(m.group("t")) * scale, int(ident, 16), data, m.group("dir"), len(ident) > 4)
    return None


def parse(lines: Iterable[str]) -> List[CanFrame]:
    return [f for f in (parse_line(l) for l in lines) if f is not None]


def stats(frames: List[CanFrame]) -> Dict[int, IdStats]:
    out: Dict[int, IdStats] = {}
    for f in frames:
        s = out.setdefault(f.can_id, IdStats(f.can_id))
        s.count += 1
        s.dlcs.add(len(f.data))
        s.times.append(f.t)
        s.directions.add(f.direction)
        if s.first_payload is None:
            s.first_payload = f.data
    return out


def bus_load_estimate(frames: List[CanFrame], bitrate: Optional[int]) -> Optional[float]:
    """Rough load estimate (standard frames, no stuffing). Needs the *measured* bit rate."""
    if not frames or not bitrate:
        return None
    span = frames[-1].t - frames[0].t
    if span <= 0:
        return None
    bits = sum(47 + 8 * len(f.data) for f in frames)
    return bits / (span * bitrate)


def report(frames: List[CanFrame], bitrate: Optional[int] = None) -> str:
    st = stats(frames)
    lines = ["| ID (hex) | count | DLC | dir | mean period ms | jitter (stdev) ms | first payload |",
             "|---|---|---|---|---|---|---|"]
    for cid in sorted(st):
        s = st[cid]
        periods = [(b - a) * 1000 for a, b in zip(s.times, s.times[1:])]
        mean = f"{statistics.mean(periods):.3f}" if periods else "-"
        jit = f"{statistics.pstdev(periods):.3f}" if len(periods) > 1 else "-"
        lines.append(f"| 0x{cid:X} | {s.count} | {sorted(s.dlcs)} | {''.join(sorted(s.directions))} | {mean} | {jit} | "
                     f"{s.first_payload.hex(' ') if s.first_payload else ''} |")
    load = bus_load_estimate(frames, bitrate)
    lines.append("")
    lines.append(f"frames: {len(frames)}, ids: {len(st)}, "
                 f"bus load: {'TBD (bit rate not measured)' if load is None else f'{load*100:.1f}% (estimate)'}")
    return "\n".join(lines)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file", help="candump log or PCAN .trc file ('-' for stdin)")
    ap.add_argument("--bitrate", type=int, default=None, help="MEASURED bus bit rate, if known")
    args = ap.parse_args()
    fh = sys.stdin if args.file == "-" else open(args.file, encoding="utf-8", errors="replace")
    print(report(parse(fh), args.bitrate))


if __name__ == "__main__":
    main()
