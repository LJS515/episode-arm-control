#!/usr/bin/env python3
"""Run the fault-injection scenario matrix on the Mock and write a Markdown report.

    python tools/generate_mock_report.py --out reports/mock_fault_injection_report.md

Every number in the report is **Synthetic / Mock Data**: it shows how the
*software* reacts to an injected fault, not how the real robot behaves.
The simulation runs on a FakeClock, so results are deterministic.
"""

from __future__ import annotations

import argparse
import datetime as _dt
import platform
import sys
from pathlib import Path
from typing import Callable, Dict, List

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from episode_hardware.clock import FakeClock  # noqa: E402
from episode_hardware.joint_mapping import VERTICAL_MOTOR_DEG  # noqa: E402
from episode_hardware.mock_adapter import MockHardwareAdapter, SimulatedArm  # noqa: E402
from episode_hardware.robot_controller import ControllerConfig, SafeRobotController  # noqa: E402
from episode_hardware.state_machine import RobotState  # noqa: E402
from episode_sdk.exceptions import EpisodeError  # noqa: E402

TARGET = [200, 100, 90, 50, 120, 60]
DT = 0.05


def fresh():
    clk = FakeClock()
    arm = SimulatedArm(clock=clk, initial_motor_deg=VERTICAL_MOTOR_DEG, homing_duration_s=1.0)
    c = SafeRobotController(MockHardwareAdapter(arm), ControllerConfig(), clock=clk)
    c.configure()
    c.poll_once()
    return c, arm, clk


def ready():
    c, arm, clk = fresh()
    c.home()
    step(c, clk, 100, lambda: c.state is RobotState.READY)
    return c, arm, clk


def step(c, clk, n, until=None) -> float:
    t0 = clk()
    for _ in range(n):
        clk.advance(DT)
        c.poll_once()
        if until and until():
            break
    return clk() - t0


def outcome(c, t=None, err=None, detect_s=None) -> Dict:
    return {
        "state": c.state.value,
        "ticket": None if t is None else t.status.value,
        "faults": ",".join(f.code for f in c.supervisor.active_faults) or "-",
        "rejected": "-" if err is None else type(err).__name__,
        "detect_s": "-" if detect_s is None else f"{detect_s:.2f}",
    }


def sc_not_homed():
    c, arm, clk = fresh()
    try:
        c.move_joints(TARGET, 0.5)
    except EpisodeError as e:
        return outcome(c, err=e)
    return outcome(c)


def sc_nominal():
    c, arm, clk = ready()
    t = c.move_joints(TARGET, 0.5)
    d = step(c, clk, 400, lambda: t.done)
    o = outcome(c, t, detect_s=d)
    o["note"] = f"max final error {c.results[t.command_id].max_final_error_deg:.3f} deg (mock)"
    return o


def sc_concurrent():
    c, arm, clk = ready()
    c.move_joints(TARGET, 0.5)
    try:
        c.move_joints([190, 90, 83, 30, 110, 30], 0.5)
    except EpisodeError as e:
        return outcome(c, err=e)
    return outcome(c)


def sc_cancel():
    c, arm, clk = ready()
    t = c.move_joints(TARGET, 0.2)
    step(c, clk, 5)
    c.cancel(t.command_id)
    d = step(c, clk, 100, lambda: c.state is RobotState.READY)
    o = outcome(c, t, detect_s=d)
    o["note"] = f"J1 stopped at {arm.true_positions()[0]:.2f} (target {TARGET[0]})"
    return o


def sc_estop():
    c, arm, clk = ready()
    t = c.move_joints(TARGET, 0.2)
    step(c, clk, 5)
    c.emergency_stop("test")
    o = outcome(c, t)
    try:
        c.move_joints(TARGET, 0.2)
    except EpisodeError as e:
        o["rejected"] = type(e).__name__ + " (after ESTOP)"
    return o


def sc_read_fail():
    c, arm, clk = ready()
    arm.faults.fail_next_reads = 3
    d = step(c, clk, 10, lambda: c.state is RobotState.FAULT)
    return outcome(c, detect_s=d)


def sc_stale_motion():
    c, arm, clk = ready()
    t = c.move_joints(TARGET, 0.2)
    arm.faults.fail_all_reads = True
    d = step(c, clk, 40, lambda: c.state is RobotState.FAULT)
    return outcome(c, t, detect_s=d)


def sc_none_joint():
    c, arm, clk = ready()
    arm.faults.none_joints = {3}
    c.poll_once()
    try:
        c.move_joints(TARGET, 0.2)
    except EpisodeError as e:
        return outcome(c, err=e)
    return outcome(c)


def sc_belt_slip():
    c, arm, clk = ready()
    arm.faults.final_offset_deg = {5: 4.0}
    t = c.move_joints(TARGET, 0.5)
    d = step(c, clk, 400, lambda: t.done)
    return outcome(c, t, detect_s=d)


def sc_stuck():
    c, arm, clk = ready()
    arm.faults.stuck_joints = {1}
    arm.faults.slowdown_factor = 50
    t = c.move_joints(TARGET, 0.5)
    d = step(c, clk, 800, lambda: t.done)
    return outcome(c, t, detect_s=d)


def sc_limit():
    c, arm, clk = ready()
    try:
        c.move_joints([400, 90, 83, 30, 110, 30], 0.5)
    except EpisodeError as e:
        return outcome(c, err=e)
    return outcome(c)


def sc_speed():
    c, arm, clk = ready()
    try:
        c.move_joints(TARGET, 2.0)
    except EpisodeError as e:
        return outcome(c, err=e)
    return outcome(c)


SCENARIOS: List = [
    ("F01", "motion before homing", sc_not_homed),
    ("F02", "nominal joint move", sc_nominal),
    ("F03", "second motion while moving", sc_concurrent),
    ("F04", "cancel during motion", sc_cancel),
    ("F05", "software ESTOP during motion", sc_estop),
    ("F06", "3 consecutive read failures (READY)", sc_read_fail),
    ("F07", "feedback lost during motion", sc_stale_motion),
    ("F08", "joint 3 reports None (FAQ tid=1932 pattern)", sc_none_joint),
    ("F09", "J5 ends 4 deg short (belt slip, FAQ tid=1996 pattern)", sc_belt_slip),
    ("F10", "J1 stuck + slow robot", sc_stuck),
    ("F11", "target beyond joint limit", sc_limit),
    ("F12", "speed_ratio = 2.0", sc_speed),
]


def render(results: List) -> str:
    now = _dt.datetime.now().strftime("%Y-%m-%d %H:%M")
    lines = [
        "# Mock Fault-Injection Report",
        "",
        "> **Synthetic / Mock Data.** Generated by `tools/generate_mock_report.py` on "
        f"{now} ({platform.system()} / Python {platform.python_version()}). The simulated arm runs on a "
        "FakeClock with linear joint interpolation; timings are in *simulated* seconds at a 20 Hz poll and "
        "describe the software's reaction, not the real robot.",
        "",
        "| ID | Scenario | Final state | Ticket | Latched faults | Rejected with | Detect/settle (sim s) | Note |",
        "|---|---|---|---|---|---|---|---|",
    ]
    for sid, name, o in results:
        lines.append(f"| {sid} | {name} | {o['state']} | {o['ticket'] or '-'} | {o['faults']} | {o['rejected']} | "
                     f"{o['detect_s']} | {o.get('note', '')} |")
    lines += ["", "Real-hardware equivalents of every row: **TBD (待真机测量)**."]
    return "\n".join(lines) + "\n"


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", default="-")
    args = ap.parse_args()
    import logging

    logging.disable(logging.CRITICAL)
    results = [(sid, name, fn()) for sid, name, fn in SCENARIOS]
    text = render(results)
    if args.out == "-":
        print(text)
    else:
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.out).write_text(text, encoding="utf-8")
        print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
