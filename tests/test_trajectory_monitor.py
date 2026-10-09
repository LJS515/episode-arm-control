"""Trajectory monitor. Covers: trajectory timeout (20), final error (21)."""

import pytest

from episode_hardware.clock import FakeClock
from episode_hardware.trajectory_monitor import MonitorStatus, TrajectoryMonitor

START = [0.0] * 6
GOAL = [10.0, 0, 0, 0, 0, 0]


def mon(**kw):
    clk = FakeClock()
    kw.setdefault("expected_duration_s", 1.0)
    return TrajectoryMonitor(START, GOAL, clock=clk, **kw), clk


def test_desired_interpolation():
    m, clk = mon()
    assert m.desired(clk())[0] == 0
    assert m.desired(clk() + 0.5)[0] == pytest.approx(5)
    assert m.desired(clk() + 5)[0] == 10


def test_success_requires_settle_time():
    m, clk = mon(settle_time_s=0.3, tolerance_deg=0.5)
    clk.advance(1.0)
    assert m.update([9.8, 0, 0, 0, 0, 0]) is MonitorStatus.SETTLING
    clk.advance(0.2)
    assert m.update([9.9, 0, 0, 0, 0, 0]) is MonitorStatus.SETTLING
    clk.advance(0.15)
    assert m.update([10, 0, 0, 0, 0, 0]) is MonitorStatus.SUCCEEDED
    r = m.result()
    assert r.max_final_error_deg == 0 and r.samples == 3


def test_leaving_tolerance_resets_settle():
    m, clk = mon(settle_time_s=0.3, tolerance_deg=0.5)
    clk.advance(1.0)
    m.update([10, 0, 0, 0, 0, 0])
    clk.advance(0.2)
    assert m.update([8, 0, 0, 0, 0, 0]) is MonitorStatus.RUNNING
    clk.advance(0.2)
    assert m.update([10, 0, 0, 0, 0, 0]) is MonitorStatus.SETTLING


def test_timeout_while_still_moving():
    m, clk = mon(timeout_s=2.0)
    for i in range(30):
        clk.advance(0.1)
        st = m.update([i * 0.2, 0, 0, 0, 0, 0])  # crawling, never stationary
        if st.terminal:
            break
    assert st is MonitorStatus.TIMED_OUT


def test_timeout_without_feedback():
    m, clk = mon(timeout_s=1.0)
    clk.advance(1.5)
    assert m.update([None] * 6) is MonitorStatus.TIMED_OUT
    assert m.result().invalid_samples == 1


def test_final_error_exceeded_when_stopped_short():
    m, clk = mon(tolerance_deg=0.5, settle_time_s=0.3)
    for _ in range(15):
        clk.advance(0.1)
        st = m.update([7.0, 0, 0, 0, 0, 0])  # stopped 3 deg short (e.g. belt slip)
        if st.terminal:
            break
    assert st is MonitorStatus.FINAL_ERROR_EXCEEDED
    assert m.result().max_final_error_deg == pytest.approx(3.0)


def test_stopped_short_before_expected_duration_is_not_failure_yet():
    m, clk = mon(expected_duration_s=5.0, tolerance_deg=0.5, settle_time_s=0.3)
    for _ in range(10):
        clk.advance(0.1)
        st = m.update([3.0, 0, 0, 0, 0, 0])
    assert st is MonitorStatus.RUNNING


def test_tracking_error_optional():
    m, clk = mon(max_tracking_error_deg=2.0)
    clk.advance(0.5)
    assert m.update([0, 0, 0, 0, 0, 0]) is MonitorStatus.TRACKING_ERROR


def test_per_joint_tolerance_and_cancel():
    m, clk = mon(tolerance_deg=[0.1, 1, 1, 1, 1, 1], settle_time_s=0)
    clk.advance(1)
    assert m.update([9.5, 0, 0, 0, 0, 0]) is MonitorStatus.RUNNING
    assert m.cancel() is MonitorStatus.CANCELLED
    assert m.update([10, 0, 0, 0, 0, 0]) is MonitorStatus.CANCELLED  # terminal is sticky


def test_rows_export():
    m, clk = mon()
    clk.advance(0.5)
    m.update([5, 0, 0, 0, 0, 0])
    row = m.to_rows()[0]
    assert row["desired_1"] == pytest.approx(5) and row["actual_1"] == 5 and row["error_1"] == 5


def test_bad_args():
    with pytest.raises(ValueError):
        TrajectoryMonitor([0] * 5, GOAL, expected_duration_s=1)
    with pytest.raises(ValueError):
        TrajectoryMonitor(START, GOAL, expected_duration_s=1, tolerance_deg=[1, 2])
