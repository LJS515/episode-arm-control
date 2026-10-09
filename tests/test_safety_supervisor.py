"""Safety supervisor. Covers: not homed (13), limits (11), speed (12),
stale (18), consecutive read failures (19), timeout (20), final error (21)."""

import pytest

from episode_hardware.clock import FakeClock
from episode_hardware.safety_supervisor import SafetyConfig, SafetySupervisor
from episode_hardware.state_cache import JointStateCache
from episode_hardware.state_machine import Event, RobotState, RobotStateMachine
from episode_hardware.trajectory_monitor import MonitorResult, MonitorStatus
from episode_sdk.exceptions import (
    EStopActiveError,
    FaultLatchedError,
    JointLimitError,
    NotHomedError,
    SpeedRatioError,
    StaleStateError,
    StateRejectedError,
)

P = [180, 90, 83, 30, 110, 30]


def make(**cfg):
    clk = FakeClock()
    sm = RobotStateMachine(clock=clk)
    cache = JointStateCache(stale_after_s=0.5, clock=clk)
    sup = SafetySupervisor(SafetyConfig(**cfg), sm, cache, clock=clk)
    return sup, sm, cache, clk


def ready(**cfg):
    sup, sm, cache, clk = make(**cfg)
    sm.dispatch(Event.CONFIGURE)
    sm.dispatch(Event.START_HOMING)
    sm.dispatch(Event.HOMING_DONE)
    cache.update_success(P)
    return sup, sm, cache, clk


def test_accepts_valid_motion():
    sup, *_ = ready()
    q, s = sup.check_motion_request([181, 90, 83, 30, 110, 30], 0.3)
    assert q[0] == 181 and s == 0.3


def test_not_homed_rejected():
    sup, sm, cache, _ = make()
    sm.dispatch(Event.CONFIGURE)
    cache.update_success(P)
    with pytest.raises(NotHomedError):
        sup.check_motion_request(P, 0.3)


def test_not_ready_rejected():
    sup, sm, *_ = ready()
    sm.dispatch(Event.MOTION_START)
    with pytest.raises(StateRejectedError):
        sup.check_motion_request(P, 0.3)


def test_soft_limits_applied():
    sup, *_ = ready(soft_margin_deg=5.0)
    with pytest.raises(JointLimitError):
        sup.check_motion_request([3, 90, 83, 30, 110, 30], 0.3)


def test_speed_rejected():
    sup, *_ = ready()
    with pytest.raises(SpeedRatioError):
        sup.check_motion_request(P, 1.5)


def test_stale_rejected():
    sup, _, _, clk = ready()
    clk.advance(1.0)
    with pytest.raises(StaleStateError):
        sup.check_motion_request(P, 0.3)


def test_partial_feedback_rejected():
    sup, _, cache, _ = ready()
    cache.update_success([180, None, 83, 30, 110, 30])
    with pytest.raises(StaleStateError):
        sup.check_motion_request(P, 0.3)


def test_max_step():
    sup, *_ = ready(max_joint_step_deg=10.0)
    with pytest.raises(JointLimitError):
        sup.check_motion_request([200, 90, 83, 30, 110, 30], 0.3)


def test_consecutive_read_failures_latch_fault():
    sup, sm, cache, _ = ready(max_consecutive_read_failures=3)
    cache.update_failure("t1")
    cache.update_failure("t2")
    assert sup.evaluate() is None
    cache.update_failure("t3")
    rec = sup.evaluate()
    assert rec.code == "READ_FAILURES" and sm.state is RobotState.FAULT
    with pytest.raises(FaultLatchedError):
        sup.check_motion_request(P, 0.3)


def test_stale_during_motion_latches():
    sup, sm, cache, clk = ready()
    sm.dispatch(Event.MOTION_START)
    clk.advance(0.6)
    assert sup.evaluate().code == "STALE_DURING_MOTION"
    assert sm.state is RobotState.FAULT


def test_stale_in_ready_does_not_fault():
    sup, sm, _, clk = ready()
    clk.advance(5)
    assert sup.evaluate() is None and sm.state is RobotState.READY


def test_position_out_of_range():
    sup, sm, cache, _ = ready()
    cache.update_success([350, 90, 83, 30, 110, 30])
    assert sup.evaluate().code == "POSITION_OUT_OF_RANGE"


@pytest.mark.parametrize("status,code", [
    (MonitorStatus.TIMED_OUT, "MOTION_TIMEOUT"),
    (MonitorStatus.FINAL_ERROR_EXCEEDED, "FINAL_ERROR_EXCEEDED"),
    (MonitorStatus.TRACKING_ERROR, "TRACKING_ERROR"),
])
def test_motion_results_latch(status, code):
    sup, sm, *_ = ready()
    sm.dispatch(Event.MOTION_START)
    res = MonitorResult(status, 1.0, 5.0, [5.0] * 6, 10, 0, "x")
    assert sup.on_motion_result(res).code == code
    assert sm.state is RobotState.FAULT


def test_success_result_no_fault():
    sup, *_ = ready()
    assert sup.on_motion_result(MonitorResult(MonitorStatus.SUCCEEDED, 1, 0, [0] * 6, 1, 0)) is None


def test_fault_clear_requires_conditions():
    sup, sm, cache, clk = ready()
    cache.update_failure("x")
    cache.update_failure("x")
    cache.update_failure("x")
    sup.evaluate()
    assert sm.state is RobotState.FAULT
    with pytest.raises(StateRejectedError):
        sup.clear_fault("alice")  # read failures still present
    cache.update_success(P)
    sup.clear_fault("alice")
    assert sm.state is RobotState.INACTIVE and not sup.active_faults
    assert sup.faults[0].cleared_by == "alice"


def test_duplicate_fault_not_relatched():
    sup, *_ = ready()
    a = sup.latch_fault("X", "1")
    b = sup.latch_fault("X", "2")
    assert a is b and len(sup.active_faults) == 1


def test_estop_rejects_motion():
    sup, sm, *_ = ready()
    sm.dispatch(Event.ESTOP)
    with pytest.raises(EStopActiveError):
        sup.check_motion_request(P, 0.3)
    assert sup.evaluate() is None


def test_config_from_dict():
    cfg = SafetyConfig.from_dict({"soft_margin_deg": 3, "joint_limits_deg": [[0, 10]] * 6, "unknown": 1})
    assert cfg.soft_margin_deg == 3 and cfg.limits.upper == (10.0,) * 6
