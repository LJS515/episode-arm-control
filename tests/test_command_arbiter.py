"""Command arbiter. Covers: concurrent motion (14), cancellation (15), estop priority (17)."""

import threading

import pytest

from episode_hardware.clock import FakeClock
from episode_hardware.command_arbiter import BusyPolicy, CommandArbiter, TicketStatus
from episode_sdk.exceptions import CommandRejectedError, EStopActiveError


def make(policy=BusyPolicy.REJECT, **kw):
    clk = FakeClock()
    return CommandArbiter(policy=policy, clock=clk, **kw), clk


def test_first_command_becomes_active():
    a, _ = make()
    t = a.submit("move", source="ros")
    assert t.status is TicketStatus.ACTIVE and a.active is t
    assert t.command_id.startswith("cmd-") and t.source == "ros"


def test_reject_policy_rejects_second():
    a, _ = make()
    a.submit("move")
    with pytest.raises(CommandRejectedError):
        a.submit("move")
    assert a.history[-1].status is TicketStatus.REJECTED


def test_complete_frees_owner():
    a, _ = make()
    t = a.submit("move")
    assert a.complete(t.command_id)
    assert t.status is TicketStatus.SUCCEEDED and a.active is None
    assert not a.complete(t.command_id)  # idempotent / unknown


def test_queue_policy_fifo_and_bound():
    a, _ = make(BusyPolicy.QUEUE, max_queue=2)
    t1 = a.submit("m1")
    t2 = a.submit("m2")
    t3 = a.submit("m3")
    assert t2.status is TicketStatus.PENDING and a.queue_length == 2
    with pytest.raises(CommandRejectedError):
        a.submit("m4")
    a.complete(t1.command_id)
    assert a.active is t2 and t2.from_queue
    a.complete(t2.command_id)
    assert a.active is t3


def test_preempt_policy_requires_allowed_source_and_priority():
    a, _ = make(BusyPolicy.PREEMPT)
    t1 = a.submit("m1", source="api", priority=1)
    with pytest.raises(CommandRejectedError):
        a.submit("m2", source="api", priority=5)  # source not allowed to preempt
    with pytest.raises(CommandRejectedError):
        a.submit("m3", source="operator", priority=0)  # lower priority
    t4 = a.submit("m4", source="operator", priority=1)
    assert t1.status is TicketStatus.PREEMPTED and t1.cancel_requested
    assert a.active is t4


def test_cancel_active_and_queued():
    a, _ = make(BusyPolicy.QUEUE)
    t1 = a.submit("m1")
    t2 = a.submit("m2")
    assert a.cancel(t2.command_id)
    assert t2.status is TicketStatus.CANCELLED
    assert a.cancel(t1.command_id)
    assert t1.status is TicketStatus.CANCELLED and t1.cancel_requested and a.active is None
    assert not a.cancel("cmd-999")


def test_emergency_has_highest_priority():
    a, _ = make(BusyPolicy.QUEUE)
    t1 = a.submit("m1")
    t2 = a.submit("m2")
    aborted = a.emergency("button")
    assert {t.command_id for t in aborted} == {t1.command_id, t2.command_id}
    assert all(t.status is TicketStatus.ABORTED_ESTOP for t in (t1, t2))
    with pytest.raises(EStopActiveError):
        a.submit("m3")
    a.clear_emergency()
    assert a.submit("m4").status is TicketStatus.ACTIVE


def test_timeout_of_active_and_queued():
    a, clk = make(BusyPolicy.QUEUE, default_timeout_s=1.0)
    t1 = a.submit("m1")
    t2 = a.submit("m2", timeout_s=0.5)
    clk.advance(0.6)
    a.check_timeouts()
    assert t2.status is TicketStatus.TIMED_OUT
    clk.advance(0.5)
    a.check_timeouts()
    assert t1.status is TicketStatus.TIMED_OUT and t1.cancel_requested


def test_ticket_wait_and_callbacks():
    a, _ = make()
    seen = []
    a.on_finish(lambda t: seen.append(("fin", t.status)))
    a.on_activate(lambda t: seen.append(("act", t.command_id)))
    t = a.submit("m")
    assert not t.wait(0.01)
    a.complete(t.command_id, success=False, reason="x")
    assert t.wait(0.01) and t.status is TicketStatus.FAILED
    assert seen == [("act", t.command_id), ("fin", TicketStatus.FAILED)]


def test_get_and_snapshot():
    a, _ = make()
    t = a.submit("m")
    assert a.get(t.command_id) is t
    assert a.snapshot()["active"] == t.command_id


def test_concurrent_submit_single_owner():
    """20 threads race; exactly one wins under REJECT."""
    a, _ = make()
    winners, rejected = [], []
    barrier = threading.Barrier(20)

    def worker(i):
        barrier.wait()
        try:
            winners.append(a.submit("m", source=f"t{i}"))
        except CommandRejectedError:
            rejected.append(i)

    ths = [threading.Thread(target=worker, args=(i,)) for i in range(20)]
    [t.start() for t in ths]
    [t.join() for t in ths]
    assert len(winners) == 1 and len(rejected) == 19


def test_estop_during_concurrent_submits():
    a, _ = make(BusyPolicy.QUEUE, max_queue=100)
    stop = threading.Event()
    errors = []

    def spam():
        while not stop.is_set():
            try:
                a.submit("m")
            except (CommandRejectedError, EStopActiveError):
                pass
            except Exception as e:  # pragma: no cover
                errors.append(e)

    ths = [threading.Thread(target=spam) for _ in range(4)]
    [t.start() for t in ths]
    a.emergency()
    snap = a.snapshot()
    stop.set()
    [t.join() for t in ths]
    assert not errors
    assert a.estop_active and a.active is None and a.queue_length == 0
    assert snap["estop"]
