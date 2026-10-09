#!/usr/bin/env python3
"""Analyse a desired/actual trajectory CSV.

Input columns: ``t, desired_1..6, actual_1..6`` (as produced by
``TrajectoryMonitor.to_rows()`` or a hardware logger). Output: per-joint
RMS / max tracking error, final error and settle time.

Results are only as real as the input. Always state the source of the CSV
(Mock, simulation, or real hardware with date and conditions).
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from typing import Dict, List, Optional


def load(path: str) -> List[Dict[str, float]]:
    with open(path, newline="", encoding="utf-8") as fh:
        rows = []
        for r in csv.DictReader(fh):
            rows.append({k: (float(v) if v not in ("", "None", None) else math.nan) for k, v in r.items()})
        return rows


def analyse(rows: List[Dict[str, float]], goal: Optional[List[float]] = None, tol: float = 1.0) -> Dict:
    if not rows:
        raise ValueError("empty trajectory")
    out: Dict = {"samples": len(rows), "duration_s": rows[-1]["t"] - rows[0]["t"], "joints": {}}
    for j in range(1, 7):
        d = [r[f"desired_{j}"] for r in rows]
        a = [r[f"actual_{j}"] for r in rows]
        e = [x - y for x, y in zip(d, a) if not (math.isnan(x) or math.isnan(y))]
        g = goal[j - 1] if goal else d[-1]
        settle = None
        for i in range(len(rows)):
            if all(not math.isnan(a[k]) and abs(g - a[k]) <= tol for k in range(i, len(rows))):
                settle = rows[i]["t"] - rows[0]["t"]
                break
        out["joints"][f"J{j}"] = {
            "rms_tracking_error_deg": round(math.sqrt(sum(x * x for x in e) / len(e)), 4) if e else None,
            "max_tracking_error_deg": round(max(abs(x) for x in e), 4) if e else None,
            "final_error_deg": None if math.isnan(a[-1]) else round(g - a[-1], 4),
            "settle_time_s": None if settle is None else round(settle, 4),
        }
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("csv")
    ap.add_argument("--tol", type=float, default=1.0)
    ap.add_argument("--source", required=True, help="e.g. 'Synthetic / Mock Data' or 'hardware 2026-xx-xx'")
    args = ap.parse_args()
    res = analyse(load(args.csv), tol=args.tol)
    res["source"] = args.source
    json.dump(res, sys.stdout, indent=2, ensure_ascii=False)
    print()


if __name__ == "__main__":
    main()
