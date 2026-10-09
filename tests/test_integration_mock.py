"""End-to-end tests of SafeRobotController on the Mock (Synthetic / Mock Data).

Covers the fault-injection matrix: not homed (13), concurrent motion (14),
action cancel (15), soft stop (16), estop (17), stale (18), read failures (19),
trajectory timeout (20), final error (21) — plus a TCP round trip through
VendorTcpAdapter + MockEpisodeServer."""

import time

import pytest

from conftest import run
from episode_hardware.command_arbiter import BusyPolicy, TicketStatus
from episode_hardware.mock_adapter import MockHardwareAdapter, SimulatedArm
from episode_hardware.robot_controller import ControllerConfig, SafeRobotController
from episode_hardware.state_machine import RobotState
from episode_hardware.vendor_adapter import VendorTcpAdapter
from episode_sdk import EpisodeClient
from episode_sdk.exceptions import (
    CommandRejectedError,
    EStopActiveError,
    FaultLatchedError,
    NoSolutionError,
    NotHomedError,
    StaleStateError,
    StateRejectedError,
)
from episode_sdk.models import ClientTimeouts, Pose

TARGET = [200, 100, 90, 50, 120, 60]


def wait_done(c, clk, ticket, steps=400):
    run(c, clk, steps=steps, until=lambda: ticket.done)
    return ticket.status


def test_motion_rejected_before_homing(controller):
    assert controller.state is RobotState.INACTIVE
    with pytest.raises(NotHomedError):
        controller.move_joints(TARGET, 0.5)


def test_home_then_move_succeeds(ready_controller, clock):
    c = ready_controller
    t = c.move_joints(TARGET, 0.5)
    assert c.state is RobotState.MOVING
    assert wait_done(c, clock, t) is TicketStatus.SUCCEEDED
    assert c.state is RobotState.READY
    assert c.results[t.command_id].max_final_error_deg < 1.0


def test_concurrent_motion_rejected(ready_controller, clock):
    c = ready_controller
    c.move_joints(TARGET, 0.5)
    with pytest.raises((CommandRejectedError, StateRejectedError)):
        c.move_joints([190, 90, 83, 30, 110, 30], 0.5)


def test_queue_policy_runs_second_after_first(arm, clock):
    c = SafeRobotController(MockHardwareAdapter(arm), ControllerConfig(busy_policy=BusyPolicy.QUEUE), clock=clock)
    c.configure()
    c.poll_once()
    c.home()
    run(c, clock, until=lambda: c.state is RobotState.READY)
    t1 = c.move_joints(TARGET, 0.5)
    # the second request is checked against READY by the supervisor, so queueing
    # happens at the arbiter level when submitted from another caller path:
    t2 = c.arbiter.submit("move_joints", {"target": [190, 90, 83, 30, 110, 30], "speed_ratio": 0.5})
    assert t2.status is TicketStatus.PENDING
    assert wait_done(c, clock, t1) is TicketStatus.SUCCEEDED
    assert wait_done(c, clock, t2) is TicketStatus.SUCCEEDED
    assert arm.true_positions()[0] == pytest.approx(190, abs=0.5)


def test_cancel_stops_physically(ready_controller, arm, clock):
    c = ready_controller
    t = c.move_joints(TARGET, 0.2)
    run(c, clock, steps=5)
    assert c.cancel(t.command_id)
    assert t.status is TicketStatus.CANCELLED
    pos_at_cancel = arm.true_positions()
    run(c, clock, steps=40)
    assert arm.true_positions() == pytest.approx(pos_at_cancel)  # did not continue to target
    assert abs(arm.true_positions()[0] - TARGET[0]) > 1.0
    assert c.state is RobotState.READY  # STOPPING -> STOP_DONE


def test_soft_stop(ready_controller, clock):
    c = ready_controller
    c.move_joints(TARGET, 0.2)
    run(c, clock, steps=3)
    c.request_stop("operator")
    assert c.state is RobotState.STOPPING
    run(c, clock, until=lambda: c.state is RobotState.READY)
    assert c.state is RobotState.READY


def test_stop_escalates_to_estop_when_soft_stop_unsupported(ready_controller, clock, monkeypatch):
    c = ready_controller
    from episode_hardware.interfaces import HardwareCapabilities

    monkeypatch.setattr(MockHardwareAdapter, "capabilities",
                        property(lambda self: HardwareCapabilities(False, False, False, True, "no-soft-stop mock")))
    c.move_joints(TARGET, 0.2)
    c.request_stop("operator")
    assert c.state is RobotState.ESTOP


def test_emergency_stop_during_motion(ready_controller, arm, clock):
    c = ready_controller
    t = c.move_joints(TARGET, 0.2)
    run(c, clock, steps=3)
    c.emergency_stop("button")
    assert c.state is RobotState.ESTOP and arm.estopped
    assert t.status is TicketStatus.ABORTED_ESTOP
    with pytest.raises(EStopActiveError):
        c.move_joints(TARGET, 0.2)
    c.release_estop("alice")
    assert c.state is RobotState.INACTIVE and not c.sm.homed  # must re-home
    with pytest.raises(NotHomedError):
        c.move_joints(TARGET, 0.2)


def test_estop_works_even_if_reads_hang(ready_controller, arm, clock):
    c = ready_controller
    c.hardware_lock.acquire()  # simulate a stuck read holding the bus lock
    try:
        c.emergency_stop("button")
    finally:
        c.hardware_lock.release()
    assert arm.estopped


def test_stale_feedback_during_motion_faults(ready_controller, arm, clock):
    c = ready_controller
    t = c.move_joints(TARGET, 0.2)
    arm.faults.fail_all_reads = True
    run(c, clock, steps=20)
    assert c.state is RobotState.FAULT
    codes = {f.code for f in c.supervisor.active_faults}
    assert codes & {"READ_FAILURES", "STALE_DURING_MOTION"}
    # the command is ended and the arm is physically stopped (mock supports soft stop)
    assert t.status is TicketStatus.FAILED and "fault" in t.reason
    p = arm.true_positions()
    run(c, clock, steps=20)
    assert arm.true_positions() == pytest.approx(p)
    assert abs(p[0] - TARGET[0]) > 1.0


def test_fault_during_motion_escalates_to_estop_without_soft_stop(ready_controller, arm, clock, monkeypatch):
    from episode_hardware.interfaces import HardwareCapabilities

    c = ready_controller
    t = c.move_joints(TARGET, 0.2)
    monkeypatch.setattr(MockHardwareAdapter, "capabilities",
                        property(lambda self: HardwareCapabilities(False, False, False, True, "no-soft-stop mock")))
    arm.faults.fail_all_reads = True
    run(c, clock, steps=20)
    assert c.state is RobotState.ESTOP and arm.estopped
    assert t.done


def test_consecutive_read_failures_fault_in_ready(ready_controller, arm, clock):
    c = ready_controller
    arm.faults.fail_next_reads = 3
    run(c, clock, steps=3)
    assert c.state is RobotState.FAULT
    assert c.supervisor.active_faults[0].code == "READ_FAILURES"
    with pytest.raises(FaultLatchedError):
        c.move_joints(TARGET, 0.2)
    run(c, clock, steps=2)  # reads healthy again
    c.clear_fault("alice")
    assert c.state is RobotState.INACTIVE


def test_transient_single_read_failure_tolerated(ready_controller, arm, clock):
    c = ready_controller
    arm.faults.fail_next_reads = 1
    run(c, clock, steps=3)
    assert c.state is RobotState.READY


def test_none_joint_blocks_motion(ready_controller, arm, clock):
    c = ready_controller
    arm.faults.none_joints = {2}
    c.poll_once()
    with pytest.raises(StaleStateError):
        c.move_joints(TARGET, 0.2)


def test_stuck_joint_times_out(ready_controller, arm, clock):
    c = ready_controller
    arm.faults.stuck_joints = {1}
    arm.faults.slowdown_factor = 50.0  # other joints crawl -> never stationary in time
    t = c.move_joints(TARGET, 0.5)
    assert wait_done(c, clock, t, steps=800) is TicketStatus.FAILED
    assert c.state is RobotState.FAULT
    assert c.supervisor.active_faults[0].code in {"MOTION_TIMEOUT", "FINAL_ERROR_EXCEEDED"}


def test_belt_slip_final_error(ready_controller, arm, clock):
    c = ready_controller
    arm.faults.final_offset_deg = {5: 4.0}  # J5 ends 4 deg off (cf. official FAQ tid=1996)
    t = c.move_joints(TARGET, 0.5)
    assert wait_done(c, clock, t) is TicketStatus.FAILED
    assert c.supervisor.active_faults[0].code == "FINAL_ERROR_EXCEEDED"


def test_arbiter_timeout_latches_fault(ready_controller, arm, clock):
    c = ready_controller
    arm.faults.slowdown_factor = 100.0
    t = c.move_joints(TARGET, 0.5, timeout_s=0.4)  # shorter than the estimate
    run(c, clock, steps=40)
    assert t.status is TicketStatus.TIMED_OUT
    assert c.state is RobotState.FAULT


def test_homing_timeout(controller, arm, clock):
    c = controller
    arm.faults.stuck_joints = {1, 2, 3, 4, 5, 6}
    arm._pos = [0.0] * 6
    c.home()
    run(c, clock, steps=1400, dt=0.05)
    assert c.state is RobotState.FAULT
    assert not c.sm.homed


def test_declare_homed(controller):
    controller.declare_homed("alice")
    assert controller.state is RobotState.READY


def test_pose_move_and_no_solution(ready_controller, arm, clock):
    c = ready_controller
    t = c.move_pose(Pose(250, 50, 380))
    assert wait_done(c, clock, t) is TicketStatus.SUCCEEDED
    arm.faults.ik_fail = True
    with pytest.raises(NoSolutionError):
        c.move_pose(Pose(250, 50, 380))
    assert c.state is RobotState.READY


def test_diagnostics(ready_controller):
    names = {d["name"] for d in ready_controller.diagnostics()}
    assert {"episode/feedback", "episode/state", "episode/arbiter"} <= names


# ----------------------------------------------------------------------------- over TCP
def test_vendor_adapter_over_tcp_mock(legacy_server):
    client = EpisodeClient(legacy_server.host, legacy_server.port,
                           timeouts=ClientTimeouts(1.0, 1.0, 1.0, 2.0))
    c = SafeRobotController(VendorTcpAdapter(client), ControllerConfig(poll_rate_hz=50))
    c.configure()
    c.poll_once()
    c.declare_homed("test")  # the mock server starts homed, like the official server without --resume
    c.start()
    try:
        t = c.move_joints([185, 92, 83, 30, 110, 30], 1.0)
        assert t.wait(5.0)
        assert t.status is TicketStatus.SUCCEEDED
        # soft stop unsupported on the vendor adapter -> stop escalates to ESTOP
        t2 = c.move_joints([200, 92, 83, 30, 110, 30], 0.1)
        time.sleep(0.1)
        c.cancel(t2.command_id)
        assert c.state is RobotState.ESTOP and legacy_server.arm.estopped
        # the official API can release the E-stop in software (params 0, doc tid=1217)
        assert c.release_estop("operator") is RobotState.INACTIVE
        assert not legacy_server.arm.estopped
        assert legacy_server.requests[-1] == {"action": "emergency_stop", "params": 0} or any(
            r == {"action": "emergency_stop", "params": 0} for r in legacy_server.requests)
    finally:
        c.shutdown()
