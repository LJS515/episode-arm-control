"""State cache. Covers: stale data (18), consecutive read failures (19)."""

import threading

import pytest

from episode_hardware.clock import FakeClock
from episode_hardware.state_cache import JointStateCache

P = [180, 90, 83, 30, 110, 30]


def make(stale=0.5):
    clk = FakeClock()
    return JointStateCache(stale_after_s=stale, clock=clk), clk


def test_empty_cache_is_stale_and_invalid():
    c, _ = make()
    s = c.snapshot()
    assert s.stale and not s.valid and s.stamp is None and not s.usable


def test_fresh_sample_usable():
    c, clk = make()
    c.update_success(P)
    s = c.snapshot()
    assert s.usable and s.age_s == 0
    assert s.require_positions() == [float(v) for v in P]


def test_becomes_stale_with_age():
    c, clk = make(0.5)
    c.update_success(P)
    clk.advance(0.49)
    assert not c.snapshot().stale
    clk.advance(0.02)
    s = c.snapshot()
    assert s.stale and not s.usable
    with pytest.raises(ValueError):
        s.require_positions()


def test_failures_counted_and_reset():
    c, clk = make()
    c.update_success(P)
    for i in range(3):
        c.update_failure("timeout")
    s = c.snapshot()
    assert s.consecutive_failures == 3 and s.total_failures == 3 and s.last_error == "timeout"
    c.update_success(P)
    s = c.snapshot()
    assert s.consecutive_failures == 0 and s.total_failures == 3 and s.last_error is None


def test_failures_make_data_stale_over_time():
    c, clk = make(0.5)
    c.update_success(P)
    for _ in range(10):
        clk.advance(0.1)
        c.update_failure("x")
    assert c.snapshot().stale


def test_partial_none_is_invalid_and_counts_as_failure():
    c, _ = make()
    c.update_success([1, None, 3, 4, 5, 6])
    s = c.snapshot()
    assert not s.valid and s.joint_valid[1] is False and s.consecutive_failures == 1
    assert "2" in s.last_error


def test_last_success_time_tracks_full_reads_only():
    c, clk = make()
    c.update_success(P)
    t_ok = clk()
    clk.advance(0.1)
    c.update_success([1, None, 3, 4, 5, 6])
    assert c.snapshot().last_success_time == t_ok


def test_out_of_order_sample_ignored():
    c, clk = make()
    c.update_success(P, stamp=clk())
    c.update_success([0] * 6, stamp=clk() - 1.0)
    assert c.snapshot().positions_deg[0] == 180


def test_velocity_estimate():
    c, clk = make()
    c.update_success(P)
    clk.advance(0.1)
    c.update_success([181] + P[1:])
    assert c.snapshot().velocity_deg_s[0] == pytest.approx(10.0)


def test_invalidate():
    c, _ = make()
    c.update_success(P)
    c.invalidate("estop")
    assert not c.snapshot().usable


def test_thread_safe_snapshots():
    c, clk = make(10)
    stop = threading.Event()
    bad = []

    def writer():
        i = 0
        while not stop.is_set():
            c.update_success([float(i)] * 6)
            i += 1

    def reader():
        for _ in range(2000):
            s = c.snapshot()
            vals = [v for v in s.positions_deg if v is not None]
            if vals and len(set(vals)) != 1:
                bad.append(vals)  # torn read

    w = threading.Thread(target=writer)
    w.start()
    rs = [threading.Thread(target=reader) for _ in range(3)]
    [r.start() for r in rs]
    [r.join() for r in rs]
    stop.set()
    w.join()
    assert not bad


def test_invalid_args():
    with pytest.raises(ValueError):
        JointStateCache(stale_after_s=0)
    c, _ = make()
    with pytest.raises(ValueError):
        c.update_success([1, 2])
