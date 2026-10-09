# Mock TCP latency (localhost)

> **Synthetic / Mock Data.** Measured with `tools/latency_analyzer.py --mock -n 300`
> on 2026-10-08, Windows 11 / Python 3.12.8, client and Mock Server in the same
> process over 127.0.0.1. The Mock answers `get_motor_angles` from memory — no
> upper computer, no CAN, no drivers. **These numbers say nothing about the real
> robot.** Real values: TBD (待真机测量).

| connection policy | n | p50 ms | p95 ms | p99 ms | max ms |
|---|---|---|---|---|---|
| per_command (new TCP connection per request, like the historical official client) | 300 | 1.00 | 15.91 | 16.30 | 24.63 |
| persistent (one connection) | 300 | 0.15 | 0.26 | 0.31 | 0.48 |

## What can (cautiously) be learned from a Mock measurement

* It isolates the **client-side transport overhead**: connection setup per
  request dominates on this machine. The ≈ 15.6 ms cluster in the per-command
  run is consistent with the default Windows timer granularity affecting the
  connect/accept path (hypothesis, evidence D — not investigated further).
* It is a **baseline** for the hardware measurement: real RTT − mock RTT ≈ time
  spent in the upper computer + CAN round trip (only if measured on the same host).

## Hardware measurement plan (TBD)

1. Same tool, `--env "real hardware, <date>, upper computer vX, driver fw V5.1.5D"`.
2. Both policies, 1000 samples, robot idle and during a slow joint move.
3. In parallel, record CAN with PCAN-View → `tools/can_trace_parser.py` to split
   the RTT into TCP part and CAN part.
