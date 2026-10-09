"""SafeRobotController — composes the control-layer pieces into one facade.

    caller (ROS2 node / example / test)
          │ move_joints / move_pose / cancel / stop / emergency_stop
          ▼
    ┌───────────────────────────── SafeRobotController ─────────────────────────────┐
    │ SafetySupervisor ── pre-checks ──► CommandArbiter ── single owner ──► hardware │
    │        ▲                                   │                                   │
    │        │ evaluate()                        ▼                                   │
    │ JointStateCache ◄── poll_once() ◄── RobotHardware.read  (hardware_lock)        │
    │        │                                                                       │
    │        └──► TrajectoryMonitor ──► MOTION_DONE / FAULT ──► RobotStateMachine    │
    └────────────────────────────────────────────────────────────────────────────────┘

Concurrency rule: **every** call into ``RobotHardware`` except
``emergency_stop`` goes through ``hardware_lock`` — the software analogue of
"one CAN bus, one transaction at a time". ``emergency_stop`` bypasses the
lock so it can never queue behind a slow read.

``poll_once()`` is the single place where feedback is read and monitors are
advanced. It is driven by an internal thread (``start()``) or called
manually by tests with a FakeClock.
"""

from __future__ import annotations

import logging
import threading
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Sequence

from episode_sdk.exceptions import (
    EpisodeError,
    EStopActiveError,
    NoSolutionError,
    StateRejectedError,
)
from episode_sdk.models import Pose
from episode_sdk.validation import validate_pose

from .clock import Clock, monotonic
from .command_arbiter import BusyPolicy, CommandArbiter, CommandTicket, TicketStatus
from .diagnostics import build_diagnostics
from .interfaces import HardwareError, RobotHardware
from .joint_mapping import VERTICAL_MOTOR_DEG
from .safety_supervisor import SafetyConfig, SafetySupervisor
from .state_cache import JointStateCache, JointStateSnapshot
from .state_machine import Event, RobotState, RobotStateMachine
from .trajectory_monitor import MonitorResult, MonitorStatus, TrajectoryMonitor

log = logging.getLogger("episode_hardware.controller")


@dataclass
class ControllerConfig:
    poll_rate_hz: float = 20.0
    stale_after_s: float = 0.5
    busy_policy: BusyPolicy = BusyPolicy.REJECT
    max_queue: int = 4
    homing_tolerance_deg: float = 2.0
    stationary_eps_deg: float = 0.05
    safety: SafetyConfig = field(default_factory=SafetyConfig)


@dataclass
class _ActiveMotion:
    ticket: CommandTicket
    monitor: Optional[TrajectoryMonitor]
    kind: str
    expected_s: float
    started: float
    #: for pose moves the joint goal is unknown -> completion by stationarity
    stationary_since: Optional[float] = None
    last_pos: Optional[List[float]] = None


class SafeRobotController:
    def __init__(self, hardware: RobotHardware, config: Optional[ControllerConfig] = None, *, clock: Clock = monotonic):
        self.hw = hardware
        self.config = config or ControllerConfig()
        self.clock = clock
        self.hardware_lock = threading.Lock()
        self._ctl = threading.RLock()
        self.cache = JointStateCache(stale_after_s=self.config.stale_after_s, clock=clock)
        self.sm = RobotStateMachine(clock=clock)
        self.supervisor = SafetySupervisor(self.config.safety, self.sm, self.cache, clock=clock)
        self.arbiter = CommandArbiter(policy=self.config.busy_policy, max_queue=self.config.max_queue, clock=clock)
        self._motion: Optional[_ActiveMotion] = None
        self._homing: Optional[TrajectoryMonitor] = None
        self._stop_started: Optional[float] = None
        self._stop_stationary_since: Optional[float] = None
        self._thread: Optional[threading.Thread] = None
        self._running = threading.Event()
        self.results: Dict[str, MonitorResult] = {}
        self._feedback_listeners: List[Callable[[JointStateSnapshot], None]] = []
        self._finish_listeners: List[Callable[[CommandTicket, Optional[MonitorResult]], None]] = []
        self.arbiter.on_finish(self._on_ticket_finished)
        self._promoted: List[CommandTicket] = []
        self.arbiter.on_activate(self._on_ticket_activated)

    # ------------------------------------------------------------------ listeners
    def add_feedback_listener(self, cb: Callable[[JointStateSnapshot], None]) -> None:
        self._feedback_listeners.append(cb)

    def add_finish_listener(self, cb: Callable[[CommandTicket, Optional[MonitorResult]], None]) -> None:
        self._finish_listeners.append(cb)

    # ------------------------------------------------------------------ lifecycle
    def configure(self) -> None:
        with self.hardware_lock:
            self.hw.connect()
        self.sm.dispatch(Event.CONFIGURE, "hardware connected")

    def start(self) -> None:
        if self._thread is not None:
            return
        self._running.set()
        self._thread = threading.Thread(target=self._loop, name="episode-poll", daemon=True)
        self._thread.start()

    def shutdown(self) -> None:
        self._running.clear()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self.sm.state in (RobotState.MOVING, RobotState.STOPPING, RobotState.HOMING):
            self.emergency_stop("shutdown during motion")
        with self.hardware_lock:
            try:
                self.hw.disconnect()
            except HardwareError:
                pass

    def _loop(self) -> None:
        import time

        period = 1.0 / self.config.poll_rate_hz
        while self._running.is_set():
            t0 = time.monotonic()
            try:
                self.poll_once()
            except Exception:  # keep polling; errors are reflected in the cache
                log.exception("poll_once failed")
            time.sleep(max(0.0, period - (time.monotonic() - t0)))

    # ------------------------------------------------------------------ homing
    def home(self) -> None:
        if self.sm.state is RobotState.ESTOP:
            raise EStopActiveError("cannot home during ESTOP")
        self.sm.dispatch(Event.START_HOMING, "operator requested homing")
        try:
            with self.hardware_lock:
                est = self.hw.home()
        except EpisodeError as exc:
            self.sm.try_dispatch(Event.HOMING_FAILED, f"home command failed: {exc.code}")
            raise
        snap = self.cache.snapshot()
        start = [p if p is not None else 0.0 for p in snap.positions_deg]
        with self._ctl:
            self._homing = TrajectoryMonitor(
                start, VERTICAL_MOTOR_DEG, expected_duration_s=est or 0.0,
                tolerance_deg=self.config.homing_tolerance_deg, settle_time_s=self.config.safety.settle_time_s,
                timeout_s=self.config.safety.homing_timeout_s, clock=self.clock)

    def declare_homed(self, operator: str) -> None:
        """Operator asserts the official process already homed the robot.

        The official server homes at start-up unless started with
        ``--resume`` (evidence C). Our layer cannot observe that, so a human
        must declare it. Logged for traceability.
        """
        if self.sm.state is not RobotState.INACTIVE:
            raise StateRejectedError("declare_homed only allowed in INACTIVE")
        log.warning("homing reference declared valid by %s", operator)
        self.sm.homed = True
        self.sm.dispatch(Event.ACTIVATE, f"homed declared by {operator}")

    # ------------------------------------------------------------------ motion
    def move_joints(self, target_motor_deg: Sequence[float], speed_ratio: Optional[float] = None, *,
                    source: str = "api", priority: int = 0, timeout_s: Optional[float] = None,
                    tolerance_deg: Optional[float] = None) -> CommandTicket:
        sr = self.config.safety.speed_ratio_default_cap if speed_ratio is None else speed_ratio
        q, s = self.supervisor.check_motion_request(target_motor_deg, sr)
        snap = self.cache.snapshot()
        start = snap.require_positions()
        ticket = self.arbiter.submit("move_joints", {"target": q, "speed_ratio": s}, source=source,
                                     priority=priority, timeout_s=timeout_s)
        if ticket.status is not TicketStatus.ACTIVE:
            return ticket  # queued (QUEUE policy) — executed when promoted
        return self._execute_joint_ticket(ticket, start, q, s, tolerance_deg)

    def _execute_joint_ticket(self, ticket: CommandTicket, start: List[float], q: List[float], s: float,
                              tolerance_deg: Optional[float]) -> CommandTicket:
        try:
            self.sm.dispatch(Event.MOTION_START, ticket.command_id)
        except StateRejectedError:
            self.arbiter.complete(ticket.command_id, False, "state machine rejected motion start")
            raise
        try:
            with self.hardware_lock:
                est = self.hw.command_joint_positions(q, s)
        except EpisodeError as exc:
            self.arbiter.complete(ticket.command_id, False, f"hardware rejected: {exc.code}")
            self.supervisor.latch_fault("COMMAND_FAILED", str(exc))
            raise
        cfg = self.config.safety
        mon = TrajectoryMonitor(start, q, expected_duration_s=est or 0.0,
                                tolerance_deg=cfg.final_tolerance_deg if tolerance_deg is None else tolerance_deg,
                                settle_time_s=cfg.settle_time_s, timeout_factor=cfg.motion_timeout_factor,
                                timeout_margin_s=cfg.motion_timeout_margin_s,
                                stationary_eps_deg=self.config.stationary_eps_deg, clock=self.clock)
        with self._ctl:
            self._motion = _ActiveMotion(ticket, mon, "move_joints", est or 0.0, self.clock())
        return ticket

    def move_pose(self, pose: Pose, *, linear: bool = False, source: str = "api", priority: int = 0,
                  timeout_s: Optional[float] = None) -> CommandTicket:
        """Cartesian move through the official IK.

        The joint goal is computed inside the official stack and not
        returned, so joint limits cannot be pre-checked and the final-error
        check is replaced by "stationary after the estimated duration".
        """
        validate_pose(pose, max_reach_mm=560.0)
        self.supervisor.check_motion_request(self.cache.snapshot().require_positions(), self.config.safety.speed_ratio_min)
        ticket = self.arbiter.submit("move_pose", {"pose": pose, "linear": linear}, source=source,
                                     priority=priority, timeout_s=timeout_s)
        if ticket.status is not TicketStatus.ACTIVE:
            return ticket
        self.sm.dispatch(Event.MOTION_START, ticket.command_id)
        try:
            with self.hardware_lock:
                est = self.hw.move_pose(pose, linear)
        except NoSolutionError:
            self.arbiter.complete(ticket.command_id, False, "no IK solution")
            self.sm.dispatch(Event.MOTION_DONE, "no IK solution, nothing moved")
            raise
        except EpisodeError as exc:
            self.arbiter.complete(ticket.command_id, False, str(exc))
            self.supervisor.latch_fault("COMMAND_FAILED", str(exc))
            raise
        with self._ctl:
            self._motion = _ActiveMotion(ticket, None, "move_pose", est or 0.0, self.clock())
        return ticket

    # ------------------------------------------------------------------ stop / cancel / estop
    def cancel(self, command_id: str, reason: str = "cancel requested") -> bool:
        """Cancel a command AND physically stop it if it is executing."""
        with self._ctl:
            executing = self._motion is not None and self._motion.ticket.command_id == command_id
        ok = self.arbiter.cancel(command_id, reason)
        if ok and executing:
            self.request_stop(f"cancel {command_id}")
        return ok

    def request_stop(self, reason: str = "stop requested") -> None:
        state = self.sm.state
        if state not in (RobotState.MOVING, RobotState.HOMING):
            return
        if not self.hw.capabilities.soft_stop:
            log.warning("soft stop unsupported by %s -> escalating to ESTOP", self.hw.capabilities.description)
            self.emergency_stop(f"{reason} (soft stop unsupported)")
            return
        self.sm.dispatch(Event.STOP_REQUEST, reason)
        with self.hardware_lock:
            supported = self.hw.soft_stop()
        if not supported:
            self.emergency_stop(f"{reason} (soft stop failed)")
            return
        with self._ctl:
            self._stop_started = self.clock()
            self._stop_stationary_since = None
            if self._motion is not None:
                if self._motion.monitor is not None:
                    self._motion.monitor.cancel(reason)
                if not self._motion.ticket.done:
                    self.arbiter.cancel(self._motion.ticket.command_id, reason)
                self._motion = None
            if self._homing is not None:
                self._homing.cancel(reason)
                self._homing = None

    def emergency_stop(self, reason: str = "emergency stop") -> None:
        """Highest priority path. Hardware first, bookkeeping second."""
        err: Optional[Exception] = None
        try:
            self.hw.emergency_stop()  # NOT under hardware_lock on purpose
        except Exception as exc:  # still enter ESTOP locally
            err = exc
            log.critical("emergency_stop delivery failed: %s — use the hardware E-stop!", exc)
        self.arbiter.emergency(reason)
        self.sm.dispatch(Event.ESTOP, reason)
        with self._ctl:
            if self._motion is not None and self._motion.monitor is not None:
                self._motion.monitor.cancel(reason)
            self._motion = None
            self._homing = None
            self._stop_started = None
        if err is not None:
            raise err

    def release_estop(self, operator: str, *, hardware_reset_confirmed: bool = False) -> RobotState:
        if self.sm.state is not RobotState.ESTOP:
            raise StateRejectedError("not in ESTOP")
        if not self.hw.capabilities.software_estop_release and not hardware_reset_confirmed:
            raise StateRejectedError("hardware reset must be confirmed by the operator")
        with self.hardware_lock:
            self.hw.release_emergency_stop()
        self.arbiter.clear_emergency()
        return self.sm.dispatch(Event.RELEASE_ESTOP, f"released by {operator}")

    def clear_fault(self, operator: str) -> RobotState:
        return self.supervisor.clear_fault(operator)

    # ------------------------------------------------------------------ polling
    def poll_once(self) -> JointStateSnapshot:
        stamp = self.clock()
        try:
            with self.hardware_lock:
                pos = self.hw.read_joint_positions()
            self.cache.update_success(pos, stamp)
        except EpisodeError as exc:
            self.cache.update_failure(f"{exc.code}: {exc.message}")
        snap = self.cache.snapshot()
        self.supervisor.evaluate()
        self._abort_on_fault()
        self.arbiter.check_timeouts()
        self._advance_motion(snap)
        self._advance_homing(snap)
        self._advance_stopping(snap)
        self._run_promoted()
        for cb in list(self._feedback_listeners):
            try:
                cb(snap)
            except Exception:
                log.exception("feedback listener failed")
        return snap

    def _abort_on_fault(self) -> None:
        """FAULT while something is executing: end the command AND stop the arm.

        A latched fault (e.g. feedback lost mid-move) does not by itself stop
        the motors — the drives keep executing the last target. If the backend
        cannot soft-stop, escalate to the emergency stop: with no trustworthy
        feedback we cannot supervise the motion any more.
        """
        if self.sm.state is not RobotState.FAULT:
            return
        with self._ctl:
            m, self._motion = self._motion, None
            h, self._homing = self._homing, None
        if m is None and h is None:
            return
        reason = f"aborted by fault: {self.sm.fault_reason}"
        if m is not None:
            if m.monitor is not None:
                m.monitor.cancel(reason)
            if not m.ticket.done:
                self.arbiter.complete(m.ticket.command_id, False, reason)
        if h is not None:
            h.cancel(reason)
        if self.hw.capabilities.soft_stop:
            try:
                with self.hardware_lock:
                    stopped = self.hw.soft_stop()
            except EpisodeError:
                stopped = False
            if stopped:
                log.error("%s -> soft stop issued", reason)
                return
        log.critical("%s -> escalating to emergency stop", reason)
        try:
            self.emergency_stop(reason)
        except Exception:  # already logged inside emergency_stop
            pass

    def _advance_motion(self, snap: JointStateSnapshot) -> None:
        with self._ctl:
            m = self._motion
        if m is None:
            return
        if m.ticket.done and m.ticket.status is not TicketStatus.ACTIVE:
            # cancelled / timed out / preempted by the arbiter while executing
            with self._ctl:
                self._motion = None
            if m.monitor is not None:
                m.monitor.cancel(m.ticket.status.value)
            if m.ticket.status is TicketStatus.TIMED_OUT:
                self.supervisor.latch_fault("MOTION_TIMEOUT", f"{m.ticket.command_id} exceeded {m.ticket.timeout_s}s")
            elif self.sm.state is RobotState.MOVING:
                self.request_stop(f"{m.ticket.command_id} {m.ticket.status.value}")
            return
        if self.sm.state is not RobotState.MOVING:
            return
        if m.monitor is not None:
            status = m.monitor.update(list(snap.positions_deg) if snap.usable else [None] * 6, snap.stamp)
            if not status.terminal:
                return
            res = m.monitor.result()
            self.results[m.ticket.command_id] = res
            with self._ctl:
                self._motion = None
            if status is MonitorStatus.SUCCEEDED:
                self.arbiter.complete(m.ticket.command_id, True, res.reason)
                self.sm.dispatch(Event.MOTION_DONE, m.ticket.command_id)
            else:
                self.arbiter.complete(m.ticket.command_id, False, res.reason)
                self.supervisor.on_motion_result(res)
            return
        # pose move: stationarity after estimated duration
        now = self.clock()
        if not snap.usable:
            return
        pos = [float(p) for p in snap.positions_deg]  # type: ignore[arg-type]
        if m.last_pos is not None and max(abs(a - b) for a, b in zip(pos, m.last_pos)) <= self.config.stationary_eps_deg:
            m.stationary_since = m.stationary_since or now
        else:
            m.stationary_since = None
        m.last_pos = pos
        elapsed = now - m.started
        timeout = m.expected_s * self.config.safety.motion_timeout_factor + self.config.safety.motion_timeout_margin_s
        if elapsed >= m.expected_s and m.stationary_since is not None and now - m.stationary_since >= self.config.safety.settle_time_s:
            with self._ctl:
                self._motion = None
            self.arbiter.complete(m.ticket.command_id, True, "stationary after estimated duration")
            self.sm.dispatch(Event.MOTION_DONE, m.ticket.command_id)
        elif elapsed > timeout:
            with self._ctl:
                self._motion = None
            self.arbiter.complete(m.ticket.command_id, False, "pose move timeout")
            self.supervisor.latch_fault("MOTION_TIMEOUT", f"pose move not settled in {timeout:.1f}s")

    def _advance_homing(self, snap: JointStateSnapshot) -> None:
        with self._ctl:
            mon = self._homing
        if mon is None or self.sm.state is not RobotState.HOMING:
            return
        status = mon.update(list(snap.positions_deg) if snap.usable else [None] * 6, snap.stamp)
        if status.terminal:
            with self._ctl:
                self._homing = None
            if status is MonitorStatus.SUCCEEDED:
                self.sm.dispatch(Event.HOMING_DONE, "post-home posture reached")
            else:
                self.sm.try_dispatch(Event.HOMING_FAILED, f"homing {status.value}: {mon.reason}")
                self.supervisor.latch_fault("HOMING_FAILED", mon.reason)

    def _advance_stopping(self, snap: JointStateSnapshot) -> None:
        if self.sm.state is not RobotState.STOPPING:
            return
        now = self.clock()
        with self._ctl:
            started = self._stop_started or now
            vel = snap.velocity_deg_s
            still = snap.usable and all(v is not None and abs(v) < 0.5 for v in vel)
            if still:
                self._stop_stationary_since = self._stop_stationary_since or now
            else:
                self._stop_stationary_since = None
            settled = self._stop_stationary_since is not None and now - self._stop_stationary_since >= self.config.safety.settle_time_s
        if settled:
            self.sm.dispatch(Event.STOP_DONE, "velocity ~0")
            with self._ctl:
                self._stop_started = None
        elif now - started > self.config.safety.stop_timeout_s:
            self.emergency_stop("soft stop did not complete in time")

    # ------------------------------------------------------------------ queue promotion
    def _on_ticket_activated(self, ticket: CommandTicket) -> None:
        if ticket.from_queue:
            with self._ctl:
                self._promoted.append(ticket)

    def _run_promoted(self) -> None:
        with self._ctl:
            todo, self._promoted = self._promoted, []
        for t in todo:
            if t.done:
                continue
            if t.kind != "move_joints":
                self.arbiter.complete(t.command_id, False, "only joint moves can be queued")
                continue
            try:
                q, s = self.supervisor.check_motion_request(t.payload["target"], t.payload["speed_ratio"])
                start = self.cache.snapshot().require_positions()
                self._execute_joint_ticket(t, start, q, s, None)
            except (EpisodeError, ValueError) as exc:
                self.arbiter.complete(t.command_id, False, f"queued command rejected at start: {exc}")

    # ------------------------------------------------------------------ misc
    def _on_ticket_finished(self, ticket: CommandTicket) -> None:
        res = self.results.get(ticket.command_id)
        for cb in list(self._finish_listeners):
            try:
                cb(ticket, res)
            except Exception:
                log.exception("finish listener failed")

    def wait(self, ticket: CommandTicket, timeout: Optional[float] = None) -> TicketStatus:
        ticket.wait(timeout)
        return ticket.status

    def diagnostics(self) -> List[Dict[str, Any]]:
        from .diagnostics import to_dict

        return to_dict(build_diagnostics(self.cache.snapshot(), self.sm.state, homed=self.sm.homed,
                                         active_faults=self.supervisor.snapshot()["active_faults"],
                                         arbiter=self.arbiter.snapshot(),
                                         hardware_description=self.hw.capabilities.description))

    @property
    def state(self) -> RobotState:
        return self.sm.state
