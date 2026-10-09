"""Explicit robot safety state machine.

States::

    UNCONFIGURED -> INACTIVE -> HOMING -> READY <-> MOVING -> STOPPING -> READY
                         any (except UNCONFIGURED) --FAULT--> FAULT
                         any                       --ESTOP--> ESTOP

Rules encoded here (see docs/13_state_machine.md for the rationale):

* ESTOP has the highest priority and is accepted from *every* state.
* FAULT is latched: leaving FAULT requires an explicit ``CLEAR_FAULT``
  whose guard (supplied by the SafetySupervisor) confirms that the cause
  is gone. Clearing never jumps straight to READY — it lands in INACTIVE.
* Leaving ESTOP requires ``RELEASE_ESTOP`` and lands in INACTIVE with the
  homing reference invalidated (configurable), because a motor may have
  been moved by hand or lost steps while de-energised.
* Motion is only accepted in READY.
* Unknown or disallowed events are *rejected* (raise), never ignored
  silently, and every rejection is recorded in the history.
"""

from __future__ import annotations

import enum
import logging
import threading
from dataclasses import dataclass, field
from typing import Callable, Dict, FrozenSet, List, Optional, Tuple

from episode_sdk.exceptions import StateRejectedError

from .clock import Clock, monotonic

log = logging.getLogger("episode_hardware.state_machine")


class RobotState(str, enum.Enum):
    UNCONFIGURED = "UNCONFIGURED"
    INACTIVE = "INACTIVE"
    HOMING = "HOMING"
    READY = "READY"
    MOVING = "MOVING"
    STOPPING = "STOPPING"
    FAULT = "FAULT"
    ESTOP = "ESTOP"


class Event(str, enum.Enum):
    CONFIGURE = "CONFIGURE"
    START_HOMING = "START_HOMING"
    HOMING_DONE = "HOMING_DONE"
    HOMING_FAILED = "HOMING_FAILED"
    ACTIVATE = "ACTIVATE"          # INACTIVE -> READY when already homed
    DEACTIVATE = "DEACTIVATE"      # READY -> INACTIVE
    MOTION_START = "MOTION_START"
    MOTION_DONE = "MOTION_DONE"
    STOP_REQUEST = "STOP_REQUEST"
    STOP_DONE = "STOP_DONE"
    FAULT = "FAULT"
    CLEAR_FAULT = "CLEAR_FAULT"
    ESTOP = "ESTOP"
    RELEASE_ESTOP = "RELEASE_ESTOP"
    SHUTDOWN = "SHUTDOWN"


S, E = RobotState, Event

#: (from_state, event) -> to_state
TRANSITIONS: Dict[Tuple[RobotState, Event], RobotState] = {
    (S.UNCONFIGURED, E.CONFIGURE): S.INACTIVE,
    (S.INACTIVE, E.START_HOMING): S.HOMING,
    (S.INACTIVE, E.ACTIVATE): S.READY,
    (S.INACTIVE, E.SHUTDOWN): S.UNCONFIGURED,
    (S.HOMING, E.HOMING_DONE): S.READY,
    (S.HOMING, E.HOMING_FAILED): S.FAULT,
    (S.HOMING, E.STOP_REQUEST): S.STOPPING,
    (S.READY, E.MOTION_START): S.MOVING,
    (S.READY, E.DEACTIVATE): S.INACTIVE,
    (S.READY, E.START_HOMING): S.HOMING,
    (S.READY, E.STOP_REQUEST): S.READY,  # idempotent stop
    (S.MOVING, E.MOTION_DONE): S.READY,
    (S.MOVING, E.STOP_REQUEST): S.STOPPING,
    (S.STOPPING, E.STOP_DONE): S.READY,
    (S.STOPPING, E.STOP_REQUEST): S.STOPPING,
    (S.FAULT, E.CLEAR_FAULT): S.INACTIVE,
    (S.ESTOP, E.RELEASE_ESTOP): S.INACTIVE,
}
# FAULT from every operational state, ESTOP from every state
for _s in (S.INACTIVE, S.HOMING, S.READY, S.MOVING, S.STOPPING, S.FAULT):
    TRANSITIONS[(_s, E.FAULT)] = S.FAULT
for _s in RobotState:
    TRANSITIONS[(_s, E.ESTOP)] = S.ESTOP


@dataclass(frozen=True)
class StateSpec:
    state: RobotState
    allowed_events: FrozenSet[Event]
    entry_action: str
    exit_action: str
    recovery: str
    safety_policy: str

    @property
    def rejected_events(self) -> FrozenSet[Event]:
        return frozenset(Event) - self.allowed_events


def _allowed(state: RobotState) -> FrozenSet[Event]:
    return frozenset(ev for (st, ev) in TRANSITIONS if st is state)


STATE_SPECS: Dict[RobotState, StateSpec] = {
    S.UNCONFIGURED: StateSpec(S.UNCONFIGURED, _allowed(S.UNCONFIGURED),
        "close hardware connection, clear cache", "load + validate configuration",
        "CONFIGURE with a valid configuration", "no command reaches hardware"),
    S.INACTIVE: StateSpec(S.INACTIVE, _allowed(S.INACTIVE),
        "start feedback polling; motion disabled", "-",
        "START_HOMING, or ACTIVATE if homing reference is still valid",
        "read-only; motion rejected; ESTOP always accepted"),
    S.HOMING: StateSpec(S.HOMING, _allowed(S.HOMING),
        "invalidate homed flag, send official home command, start homing timer", "stop homing timer",
        "HOMING_DONE when feedback settles at the expected post-home posture; timeout -> HOMING_FAILED",
        "no other motion; arm moves by itself — keep workspace clear"),
    S.READY: StateSpec(S.READY, _allowed(S.READY),
        "accept new motion commands", "-", "-",
        "motion allowed only if homed, feedback fresh and target within limits"),
    S.MOVING: StateSpec(S.MOVING, _allowed(S.MOVING),
        "start trajectory monitor (desired/actual/error) and motion timeout", "stop monitor",
        "MOTION_DONE when within tolerance and settled; timeout/final error -> FAULT",
        "exactly one motion owner (arbiter); new motion rejected or preempted per policy"),
    S.STOPPING: StateSpec(S.STOPPING, _allowed(S.STOPPING),
        "issue soft stop (or ESTOP if soft stop unsupported), start stop timer", "-",
        "STOP_DONE when velocity ~ 0 for the settle time; timer expiry -> FAULT",
        "no new motion; ESTOP escalation if stop does not complete"),
    S.FAULT: StateSpec(S.FAULT, _allowed(S.FAULT),
        "latch fault reason, request soft stop, reject motion", "record clearing operator/reason",
        "CLEAR_FAULT only when supervisor guard passes (fresh data, no read failures, operator ack)",
        "latched; never auto-clears; lands in INACTIVE, not READY"),
    S.ESTOP: StateSpec(S.ESTOP, _allowed(S.ESTOP),
        "send emergency_stop (dedicated connection), cancel all commands, invalidate homing", "-",
        "RELEASE_ESTOP after the physical cause is cleared and the operator confirms; re-home",
        "highest priority; software ESTOP is NOT a substitute for a hardware power-cut E-stop"),
}


@dataclass
class TransitionRecord:
    t: float
    from_state: RobotState
    event: Event
    to_state: Optional[RobotState]
    accepted: bool
    reason: str = ""


Guard = Callable[[RobotState, Event], Tuple[bool, str]]
Listener = Callable[[RobotState, Event, RobotState], None]


class RobotStateMachine:
    def __init__(self, *, clock: Clock = monotonic, history_size: int = 500,
                 require_rehome_after_estop: bool = True) -> None:
        self.clock = clock
        self._state = RobotState.UNCONFIGURED
        self._lock = threading.RLock()
        self._guards: Dict[Event, List[Guard]] = {}
        self._listeners: List[Listener] = []
        self._entry_hooks: Dict[RobotState, List[Callable[[], None]]] = {}
        self._exit_hooks: Dict[RobotState, List[Callable[[], None]]] = {}
        self.history: List[TransitionRecord] = []
        self.history_size = history_size
        self.homed = False
        self.fault_reason: Optional[str] = None
        self.require_rehome_after_estop = require_rehome_after_estop

    # ------------------------------------------------------------------ registration
    def add_guard(self, event: Event, guard: Guard) -> None:
        self._guards.setdefault(event, []).append(guard)

    def add_listener(self, listener: Listener) -> None:
        self._listeners.append(listener)

    def on_entry(self, state: RobotState, hook: Callable[[], None]) -> None:
        self._entry_hooks.setdefault(state, []).append(hook)

    def on_exit(self, state: RobotState, hook: Callable[[], None]) -> None:
        self._exit_hooks.setdefault(state, []).append(hook)

    # ------------------------------------------------------------------ queries
    @property
    def state(self) -> RobotState:
        with self._lock:
            return self._state

    def can(self, event: Event) -> bool:
        with self._lock:
            return (self._state, event) in TRANSITIONS

    def motion_allowed(self) -> bool:
        with self._lock:
            return self._state is RobotState.READY and self.homed

    # ------------------------------------------------------------------ core
    def _record(self, rec: TransitionRecord) -> None:
        self.history.append(rec)
        if len(self.history) > self.history_size:
            del self.history[: len(self.history) - self.history_size]

    def dispatch(self, event: Event, reason: str = "") -> RobotState:
        listeners: List[Tuple[RobotState, Event, RobotState]] = []
        with self._lock:
            src = self._state
            dst = TRANSITIONS.get((src, event))
            if dst is None:
                self._record(TransitionRecord(self.clock(), src, event, None, False, f"not allowed in {src.value}"))
                raise StateRejectedError(
                    f"event {event.value} not allowed in state {src.value}",
                    details={"state": src.value, "event": event.value},
                )
            if event is Event.ACTIVATE and not self.homed:
                self._record(TransitionRecord(self.clock(), src, event, None, False, "not homed"))
                raise StateRejectedError("ACTIVATE requires a valid homing reference", details={"state": src.value})
            for guard in self._guards.get(event, []):
                ok, why = guard(src, event)
                if not ok:
                    self._record(TransitionRecord(self.clock(), src, event, None, False, why))
                    raise StateRejectedError(f"guard rejected {event.value}: {why}", details={"state": src.value})

            # bookkeeping that is part of the transition itself
            if event is Event.START_HOMING:
                self.homed = False
            elif event is Event.HOMING_DONE:
                self.homed = True
            elif event in (Event.FAULT, Event.HOMING_FAILED):
                # keep the first reason while latched
                if self.fault_reason is None or src is not RobotState.FAULT:
                    self.fault_reason = reason or event.value
            elif event is Event.CLEAR_FAULT:
                self.fault_reason = None
            elif event is Event.ESTOP and self.require_rehome_after_estop:
                self.homed = False

            if dst is not src:
                for hook in self._exit_hooks.get(src, []):
                    self._safe(hook)
            self._state = dst
            self._record(TransitionRecord(self.clock(), src, event, dst, True, reason))
            if dst is not src:
                for hook in self._entry_hooks.get(dst, []):
                    self._safe(hook)
            listeners = [(src, event, dst)]
        for s, e, d in listeners:
            log.info("state %s --%s--> %s %s", s.value, e.value, d.value, f"({reason})" if reason else "")
            for cb in list(self._listeners):
                self._safe(lambda cb=cb: cb(s, e, d))
        return dst

    def try_dispatch(self, event: Event, reason: str = "") -> bool:
        try:
            self.dispatch(event, reason)
            return True
        except StateRejectedError:
            return False

    @staticmethod
    def _safe(fn: Callable[[], None]) -> None:
        try:
            fn()
        except Exception:  # hooks must never break the state machine
            log.exception("state machine hook failed")
