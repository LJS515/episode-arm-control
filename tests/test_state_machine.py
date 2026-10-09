import pytest

from episode_hardware.state_machine import (
    STATE_SPECS,
    TRANSITIONS,
    Event,
    RobotState,
    RobotStateMachine,
)
from episode_sdk.exceptions import StateRejectedError

S, E = RobotState, Event


def to_ready():
    sm = RobotStateMachine()
    sm.dispatch(E.CONFIGURE)
    sm.dispatch(E.START_HOMING)
    sm.dispatch(E.HOMING_DONE)
    return sm


def test_happy_path():
    sm = to_ready()
    assert sm.state is S.READY and sm.homed and sm.motion_allowed()
    sm.dispatch(E.MOTION_START)
    assert sm.state is S.MOVING and not sm.motion_allowed()
    sm.dispatch(E.MOTION_DONE)
    sm.dispatch(E.MOTION_START)
    sm.dispatch(E.STOP_REQUEST)
    assert sm.state is S.STOPPING
    sm.dispatch(E.STOP_DONE)
    assert sm.state is S.READY


def test_every_state_has_a_spec_and_specs_match_table():
    assert set(STATE_SPECS) == set(RobotState)
    for st, spec in STATE_SPECS.items():
        assert spec.allowed_events == frozenset(e for (s, e) in TRANSITIONS if s is st)
        assert spec.allowed_events | spec.rejected_events == frozenset(Event)
        assert spec.entry_action and spec.safety_policy and spec.recovery


@pytest.mark.parametrize("state", list(RobotState))
def test_estop_accepted_from_every_state(state):
    assert (state, E.ESTOP) in TRANSITIONS


def test_motion_rejected_outside_ready():
    sm = RobotStateMachine()
    with pytest.raises(StateRejectedError):
        sm.dispatch(E.MOTION_START)
    sm.dispatch(E.CONFIGURE)
    with pytest.raises(StateRejectedError):
        sm.dispatch(E.MOTION_START)
    assert sm.history[-1].accepted is False


def test_activate_requires_homed():
    sm = RobotStateMachine()
    sm.dispatch(E.CONFIGURE)
    with pytest.raises(StateRejectedError):
        sm.dispatch(E.ACTIVATE)
    sm.homed = True
    sm.dispatch(E.ACTIVATE)
    assert sm.state is S.READY


def test_fault_is_latched_and_clears_to_inactive():
    sm = to_ready()
    sm.dispatch(E.MOTION_START)
    sm.dispatch(E.FAULT, "read failures")
    assert sm.state is S.FAULT and sm.fault_reason == "read failures"
    sm.dispatch(E.FAULT, "second")
    assert sm.fault_reason == "read failures"  # first reason kept
    for ev in (E.MOTION_START, E.MOTION_DONE, E.ACTIVATE, E.HOMING_DONE):
        with pytest.raises(StateRejectedError):
            sm.dispatch(ev)
    sm.dispatch(E.CLEAR_FAULT)
    assert sm.state is S.INACTIVE and sm.fault_reason is None


def test_estop_invalidates_homing_and_requires_release():
    sm = to_ready()
    sm.dispatch(E.ESTOP)
    assert sm.state is S.ESTOP and not sm.homed
    with pytest.raises(StateRejectedError):
        sm.dispatch(E.CLEAR_FAULT)
    sm.dispatch(E.RELEASE_ESTOP)
    assert sm.state is S.INACTIVE
    with pytest.raises(StateRejectedError):
        sm.dispatch(E.ACTIVATE)  # must re-home


def test_estop_keeps_homing_when_configured():
    sm = RobotStateMachine(require_rehome_after_estop=False)
    sm.dispatch(E.CONFIGURE)
    sm.dispatch(E.START_HOMING)
    sm.dispatch(E.HOMING_DONE)
    sm.dispatch(E.ESTOP)
    sm.dispatch(E.RELEASE_ESTOP)
    sm.dispatch(E.ACTIVATE)
    assert sm.state is S.READY


def test_homing_failure_goes_to_fault():
    sm = RobotStateMachine()
    sm.dispatch(E.CONFIGURE)
    sm.dispatch(E.START_HOMING)
    assert not sm.homed
    sm.dispatch(E.HOMING_FAILED, "timeout")
    assert sm.state is S.FAULT and not sm.homed


def test_guards_and_hooks_and_listeners():
    sm = RobotStateMachine()
    calls = []
    sm.on_entry(S.INACTIVE, lambda: calls.append("enter_inactive"))
    sm.on_exit(S.UNCONFIGURED, lambda: calls.append("exit_unconfigured"))
    sm.add_listener(lambda a, e, b: calls.append((a.value, e.value, b.value)))
    sm.on_entry(S.INACTIVE, lambda: 1 / 0)  # a broken hook must not break the machine
    sm.dispatch(E.CONFIGURE)
    assert calls == ["exit_unconfigured", "enter_inactive", ("UNCONFIGURED", "CONFIGURE", "INACTIVE")]
    sm.add_guard(E.START_HOMING, lambda s, e: (False, "workspace not clear"))
    with pytest.raises(StateRejectedError) as ei:
        sm.dispatch(E.START_HOMING)
    assert "workspace" in str(ei.value)
    assert not sm.try_dispatch(E.START_HOMING)


def test_history_bounded():
    sm = RobotStateMachine(history_size=5)
    for _ in range(10):
        sm.try_dispatch(E.MOTION_DONE)
    assert len(sm.history) == 5
