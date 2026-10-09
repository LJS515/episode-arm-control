#!/usr/bin/env python3
"""Measure request/response latency of the TCP API.

    # against the built-in mock (Synthetic / Mock Data)
    python tools/latency_analyzer.py --mock -n 500
    # against a real upper computer (only with the robot in a safe state;
    # get_motor_angles is read-only)
    python tools/latency_analyzer.py --host 192.168.1.10 --port 12345 -n 500 --env "real hardware, <date>"

Only ``get_motor_angles`` (read-only) is used. Output always states the
environment; numbers from ``--mock`` say nothing about the real robot.
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
import time
from pathlib import Path
from typing import Dict, List

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from episode_sdk import EpisodeClient  # noqa: E402
from episode_sdk.exceptions import EpisodeError  # noqa: E402
from episode_sdk.models import ConnectionPolicy  # noqa: E402


def percentile(sorted_vals: List[float], p: float) -> float:
    if not sorted_vals:
        return float("nan")
    k = (len(sorted_vals) - 1) * p
    lo, hi = int(k), min(int(k) + 1, len(sorted_vals) - 1)
    return sorted_vals[lo] + (sorted_vals[hi] - sorted_vals[lo]) * (k - lo)


def summarize(samples_ms: List[float], errors: int, env: str, policy: str) -> Dict:
    s = sorted(samples_ms)
    return {
        "environment": env,
        "connection_policy": policy,
        "n_ok": len(s),
        "n_error": errors,
        "min_ms": round(s[0], 3) if s else None,
        "p50_ms": round(percentile(s, 0.50), 3) if s else None,
        "p95_ms": round(percentile(s, 0.95), 3) if s else None,
        "p99_ms": round(percentile(s, 0.99), 3) if s else None,
        "max_ms": round(s[-1], 3) if s else None,
        "mean_ms": round(statistics.mean(s), 3) if s else None,
        "stdev_ms": round(statistics.pstdev(s), 3) if len(s) > 1 else None,
    }


def measure(client: EpisodeClient, n: int, interval_s: float) -> (List[float], int):
    out, err = [], 0
    for _ in range(n):
        t0 = time.perf_counter()
        try:
            client.get_motor_angles()
            out.append((time.perf_counter() - t0) * 1000.0)
        except EpisodeError:
            err += 1
        if interval_s:
            time.sleep(interval_s)
    return out, err


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=12345)
    ap.add_argument("--mock", action="store_true", help="start an in-process mock server")
    ap.add_argument("-n", type=int, default=200)
    ap.add_argument("--interval", type=float, default=0.0)
    ap.add_argument("--policy", choices=["per_command", "persistent"], default="per_command")
    ap.add_argument("--env", default=None, help="free-text environment label (required for real hardware)")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()

    srv = None
    if args.mock:
        from episode_sdk.mock_server import MockEpisodeServer

        srv = MockEpisodeServer().start()
        args.host, args.port = srv.host, srv.port
        env = "Synthetic / Mock Data (localhost mock server)"
    else:
        if not args.env:
            ap.error("--env is required when measuring a real system")
        env = args.env
    try:
        with EpisodeClient(args.host, args.port, connection_policy=ConnectionPolicy(args.policy)) as c:
            samples, errors = measure(c, args.n, args.interval)
    finally:
        if srv:
            srv.stop()
    res = summarize(samples, errors, env, args.policy)
    if args.json:
        print(json.dumps(res, indent=2, ensure_ascii=False))
    else:
        for k, v in res.items():
            print(f"{k:18s} {v}")


if __name__ == "__main__":
    main()
