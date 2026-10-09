#!/usr/bin/env python3
"""End-to-end Mock demo: TCP mock server -> SDK -> safety layer -> fault handling.

Everything printed here is Synthetic / Mock Data.
"""

import logging
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from episode_hardware.config_loader import load_client, load_controller_config  # noqa: E402
from episode_hardware.robot_controller import SafeRobotController  # noqa: E402
from episode_hardware.state_machine import RobotState  # noqa: E402
from episode_hardware.vendor_adapter import VendorTcpAdapter  # noqa: E402
from episode_sdk.exceptions import EpisodeError  # noqa: E402
from episode_sdk.models import ClientTimeouts  # noqa: E402
from episode_sdk.mock_server import MockEpisodeServer  # noqa: E402


def say(msg: str) -> None:
    print(f"[Synthetic / Mock Data] {msg}")


def wait_state(ctl, state, timeout=3.0) -> bool:
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if ctl.state is state:
            return True
        time.sleep(0.02)
    return False


def main() -> int:
    logging.basicConfig(level=logging.CRITICAL)
    srv = MockEpisodeServer().start()
    client = load_client(host=srv.host, port=srv.port)
    # The poller's read timeout must be shorter than the staleness threshold,
    # otherwise one hung read hides the outage (see docs/10_concurrency_analysis.md).
    client.timeouts = ClientTimeouts(connect_s=1.0, send_s=0.5, receive_s=0.3, command_total_s=0.5)
    ctl = SafeRobotController(VendorTcpAdapter(client), load_controller_config())
    ok = True
    try:
        ctl.configure()
        ctl.poll_once()
        say(f"0) state after configure: {ctl.state.value}")
        try:
            ctl.move_joints([190, 90, 83, 30, 110, 30], 0.3)
            ok = False
        except EpisodeError as e:
            say(f"1) motion before homing rejected: {e.code}")
        ctl.declare_homed("demo")
        ctl.start()
        t = ctl.move_joints([195, 95, 80, 40, 100, 40], 1.0)
        t.wait(10)
        res = ctl.results.get(t.command_id)
        say(f"2) joint move: {t.status.value}, max final error "
            f"{res.max_final_error_deg if res else 'n/a'} deg")
        ok &= t.status.value == "SUCCEEDED"
        t = ctl.move_joints([180, 90, 83, 30, 110, 30], 0.5)
        try:
            ctl.move_joints([200, 90, 83, 30, 110, 30], 0.5)
            ok = False
        except EpisodeError as e:
            say(f"3) concurrent motion rejected: {e.code}")
        t.wait(10)
        try:
            ctl.move_joints([400, 90, 83, 30, 110, 30], 0.5)
            ok = False
        except EpisodeError as e:
            say(f"4) out-of-range target rejected: {e.code}")
        srv.faults.drop_next_replies = 1000  # feedback stops arriving
        reached = wait_state(ctl, RobotState.FAULT, timeout=20)
        say(f"5) feedback loss -> state {ctl.state.value}, faults {[f.code for f in ctl.supervisor.active_faults]}")
        ok &= reached
        srv.faults.drop_next_replies = 0
        time.sleep(0.8)
        ctl.clear_fault("demo-operator")
        say(f"6) fault cleared by operator -> {ctl.state.value} (must re-home or re-declare before moving)")
        ctl.emergency_stop("demo")
        say(f"7) software ESTOP -> {ctl.state.value}, mock drive estopped={srv.arm.estopped}")
        ok &= srv.arm.estopped
    finally:
        ctl.shutdown()
        srv.stop()
    say("DEMO PASSED" if ok else "DEMO FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
