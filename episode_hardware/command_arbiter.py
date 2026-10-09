"""Single-owner motion command arbiter.

Problem it solves: in the official ROS2 node and the official TCP server,
several callbacks / client connections can issue motion commands
concurrently and they all end up on one CAN bus (evidence D, from the
historical server spawning one thread per client). Two interleaved
point-to-point moves produce an undefined trajectory.

Policy
------
* At most ONE active motion command ("owner") at a time.
* A new motion command, while one is active, is handled by the configured
  ``BusyPolicy``: REJECT (default), QUEUE (bounded FIFO) or PREEMPT
  (cancel the active one if the new one has >= priority and the source is
  allowed to preempt).
* ``emergency()`` always wins: cancels the active command and the queue
  and blocks every new motion until ``clear_emergency()``.
* Every ticket has an id, source, submit time, timeout and terminal status.
* All public methods are thread-safe.
"""

from __future__ import annotations

import collections
import enum
import itertools
import threading
from dataclasses import dataclass, field
from typing import Any, Callable, Deque, Dict, FrozenSet, List, Optional

from episode_sdk.exceptions import CommandRejectedError, EStopActiveError

from .clock import Clock, monotonic


class BusyPolicy(str, enum.Enum):
    REJECT = "reject"
    QUEUE = "queue"
    PREEMPT = "preempt"


class TicketStatus(str, enum.Enum):
    PENDING = "PENDING"
    ACTIVE = "ACTIVE"
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"
    CANCELLED = "CANCELLED"
    PREEMPTED = "PREEMPTED"
    REJECTED = "REJECTED"
    TIMED_OUT = "TIMED_OUT"
    ABORTED_ESTOP = "ABORTED_ESTOP"

    @property
    def terminal(self) -> bool:
        return self not in (TicketStatus.PENDING, TicketStatus.ACTIVE)


@dataclass
class CommandTicket:
    command_id: str
    source: str
    kind: str
    payload: Any
    priority: int
    submitted_at: float
    timeout_s: float
    status: TicketStatus = TicketStatus.PENDING
    started_at: Optional[float] = None
    finished_at: Optional[float] = None
    reason: str = ""
    _event: threading.Event = field(default_factory=threading.Event, repr=False)
    cancel_requested: bool = False
    #: True when the ticket was activated from the queue (not at submit time)
    from_queue: bool = False

    def wait(self, timeout: Optional[float] = None) -> bool:
        return self._event.wait(timeout)

    @property
    def done(self) -> bool:
        return self.status.terminal


Callback = Callable[[CommandTicket], None]


class CommandArbiter:
    def __init__(
        self,
        *,
        policy: BusyPolicy = BusyPolicy.REJECT,
        max_queue: int = 4,
        preempt_sources: FrozenSet[str] = frozenset({"operator", "safety"}),
        default_timeout_s: float = 30.0,
        clock: Clock = monotonic,
    ) -> None:
        self.policy = BusyPolicy(policy)
        self.max_queue = int(max_queue)
        self.preempt_sources = frozenset(preempt_sources)
        self.default_timeout_s = float(default_timeout_s)
        self.clock = clock
        self._lock = threading.RLock()
        self._active: Optional[CommandTicket] = None
        self._queue: Deque[CommandTicket] = collections.deque()
        self._ids = itertools.count(1)
        self._estop = False
        self.history: List[CommandTicket] = []
        self._on_activate: List[Callback] = []
        self._on_finish: List[Callback] = []

    # ------------------------------------------------------------------ callbacks
    def on_activate(self, cb: Callback) -> None:
        self._on_activate.append(cb)

    def on_finish(self, cb: Callback) -> None:
        self._on_finish.append(cb)

    # ------------------------------------------------------------------ queries
    @property
    def active(self) -> Optional[CommandTicket]:
        with self._lock:
            return self._active

    @property
    def queue_length(self) -> int:
        with self._lock:
            return len(self._queue)

    @property
    def estop_active(self) -> bool:
        with self._lock:
            return self._estop

    def get(self, command_id: str) -> Optional[CommandTicket]:
        with self._lock:
            for t in ([self._active] if self._active else []) + list(self._queue) + self.history:
                if t.command_id == command_id:
                    return t
        return None

    # ------------------------------------------------------------------ submit
    def submit(self, kind: str, payload: Any = None, *, source: str = "api", priority: int = 0,
               timeout_s: Optional[float] = None) -> CommandTicket:
        """Submit a motion command. Raises on rejection; returns the ticket otherwise."""
        activated: List[CommandTicket] = []
        finished: List[CommandTicket] = []
        with self._lock:
            ticket = CommandTicket(
                command_id=f"cmd-{next(self._ids)}", source=source, kind=kind, payload=payload,
                priority=int(priority), submitted_at=self.clock(),
                timeout_s=self.default_timeout_s if timeout_s is None else float(timeout_s),
            )
            if self._estop:
                self._finish_locked(ticket, TicketStatus.REJECTED, "emergency stop active", finished)
                self._fire(activated, finished)
                raise EStopActiveError("motion rejected: emergency stop active", details={"command_id": ticket.command_id})
            self._expire_locked(finished)
            if self._active is None:
                self._activate_locked(ticket, activated)
            elif self.policy is BusyPolicy.PREEMPT and source in self.preempt_sources and priority >= self._active.priority:
                self._active.cancel_requested = True
                self._finish_locked(self._active, TicketStatus.PREEMPTED, f"preempted by {ticket.command_id}", finished)
                self._active = None
                self._activate_locked(ticket, activated)
            elif self.policy is BusyPolicy.QUEUE and len(self._queue) < self.max_queue:
                self._queue.append(ticket)
            else:
                why = "queue full" if self.policy is BusyPolicy.QUEUE else f"busy with {self._active.command_id}"
                self._finish_locked(ticket, TicketStatus.REJECTED, why, finished)
                self._fire(activated, finished)
                raise CommandRejectedError(f"motion rejected: {why}", details={"command_id": ticket.command_id,
                                                                                "active": self._active.command_id})
        self._fire(activated, finished)
        return ticket

    # ------------------------------------------------------------------ completion / cancel
    def complete(self, command_id: str, success: bool = True, reason: str = "") -> bool:
        activated: List[CommandTicket] = []
        finished: List[CommandTicket] = []
        with self._lock:
            t = self._active
            if t is None or t.command_id != command_id:
                return False
            self._finish_locked(t, TicketStatus.SUCCEEDED if success else TicketStatus.FAILED, reason, finished)
            self._active = None
            self._promote_locked(activated)
        self._fire(activated, finished)
        return True

    def cancel(self, command_id: str, reason: str = "cancel requested") -> bool:
        """Cancel a pending or active command.

        NOTE: cancelling the *ticket* does not stop the motors. The owner of
        the active command (SafeRobotController) observes ``cancel_requested``
        / the CANCELLED status and must issue a physical stop.
        """
        activated: List[CommandTicket] = []
        finished: List[CommandTicket] = []
        with self._lock:
            for t in list(self._queue):
                if t.command_id == command_id:
                    self._queue.remove(t)
                    self._finish_locked(t, TicketStatus.CANCELLED, reason, finished)
                    self._fire(activated, finished)
                    return True
            t = self._active
            if t is not None and t.command_id == command_id:
                t.cancel_requested = True
                self._finish_locked(t, TicketStatus.CANCELLED, reason, finished)
                self._active = None
                self._promote_locked(activated)
            else:
                return False
        self._fire(activated, finished)
        return True

    def emergency(self, reason: str = "emergency stop") -> List[CommandTicket]:
        """Highest priority. Never raises, never blocks on the queue."""
        finished: List[CommandTicket] = []
        with self._lock:
            self._estop = True
            if self._active is not None:
                self._active.cancel_requested = True
                self._finish_locked(self._active, TicketStatus.ABORTED_ESTOP, reason, finished)
                self._active = None
            while self._queue:
                self._finish_locked(self._queue.popleft(), TicketStatus.ABORTED_ESTOP, reason, finished)
        self._fire([], finished)
        return finished

    def clear_emergency(self) -> None:
        with self._lock:
            self._estop = False

    def check_timeouts(self) -> List[CommandTicket]:
        activated: List[CommandTicket] = []
        finished: List[CommandTicket] = []
        with self._lock:
            self._expire_locked(finished)
            if self._active is None:
                self._promote_locked(activated)
        self._fire(activated, finished)
        return finished

    # ------------------------------------------------------------------ internals (lock held)
    def _activate_locked(self, t: CommandTicket, activated: List[CommandTicket]) -> None:
        t.status = TicketStatus.ACTIVE
        t.started_at = self.clock()
        self._active = t
        activated.append(t)

    def _promote_locked(self, activated: List[CommandTicket]) -> None:
        if self._active is None and self._queue and not self._estop:
            t = self._queue.popleft()
            t.from_queue = True
            self._activate_locked(t, activated)

    def _finish_locked(self, t: CommandTicket, status: TicketStatus, reason: str, finished: List[CommandTicket]) -> None:
        if t.status.terminal:
            return
        t.status = status
        t.reason = reason
        t.finished_at = self.clock()
        t._event.set()
        self.history.append(t)
        if len(self.history) > 1000:
            del self.history[:100]
        finished.append(t)

    def _expire_locked(self, finished: List[CommandTicket]) -> None:
        now = self.clock()
        if self._active is not None:
            ref = self._active.started_at or self._active.submitted_at
            if now - ref > self._active.timeout_s:
                self._active.cancel_requested = True
                self._finish_locked(self._active, TicketStatus.TIMED_OUT, "command timeout", finished)
                self._active = None
        for t in list(self._queue):
            if now - t.submitted_at > t.timeout_s:
                self._queue.remove(t)
                self._finish_locked(t, TicketStatus.TIMED_OUT, "timed out in queue", finished)

    def _fire(self, activated: List[CommandTicket], finished: List[CommandTicket]) -> None:
        # callbacks run outside the lock to avoid deadlocks with callers
        for t in finished:
            for cb in list(self._on_finish):
                cb(t)
        for t in activated:
            for cb in list(self._on_activate):
                cb(t)

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "policy": self.policy.value,
                "estop": self._estop,
                "active": None if self._active is None else self._active.command_id,
                "queue": [t.command_id for t in self._queue],
            }
