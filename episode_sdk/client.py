"""Hardened TCP/JSON client for the Episode 1 official upper computer.

Design notes
------------
* ``legacy_compatible=True`` (default) sends exactly ``{"action", "params"}``
  and interprets the reply as the raw result value — the only shape we know
  the official server accepts. Enhanced fields (``request_id``,
  ``protocol_version``, structured errors) are only sent in ENHANCED mode,
  which our Mock Server implements.
* Every round trip has connect / send / receive timeouts *and* an absolute
  command deadline.
* Commands on one client are serialised by a lock (one request in flight per
  connection). ``emergency_stop`` never waits for that lock: it always uses a
  fresh, dedicated connection so it cannot be queued behind a stuck request.
* A motion reply is an *acknowledgement with an estimated duration*, not a
  completion. This client never ``sleep()``s for the estimated duration the
  way the historical client did; completion must be judged from feedback
  (see ``episode_hardware.trajectory_monitor``).
"""

from __future__ import annotations

import logging
import math
import socket
import threading
import time
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence

from . import protocol
from .exceptions import (
    CommandTimeoutError,
    ConnectionClosedError,
    ConnectionFailedError,
    ConnectTimeoutError,
    EpisodeError,
    MalformedPayloadError,
    NoSolutionError,
    ParameterTypeError,
    ProtocolVersionError,
    ReadFailedError,
    ReceiveTimeoutError,
    RemoteCommandError,
    RequestIdMismatchError,
    TransportError,
)
from .models import (
    ClientTimeouts,
    CommandEnvelope,
    ConnectionPolicy,
    JointLimits,
    MotionAck,
    Pose,
    ProtocolMode,
)
from .retry import RetryPolicy, call_with_retry
from .validation import (
    validate_bool_flag,
    validate_euler_order,
    validate_joint_reply,
    validate_joint_vector,
    validate_number,
    validate_pose,
    validate_servo_angle,
    validate_speed_ratio,
)

log = logging.getLogger("episode_sdk.client")

#: Official action strings. Overridable via ``action_names`` in case a
#: future SDK renames them. Evidence: B (user list) / C (historical source).
DEFAULT_ACTIONS: Dict[str, str] = {
    "emergency_stop": "emergency_stop",
    "home": "home",
    "angle_mode": "angle_mode",
    "move_xyz_rotation": "move_xyz_rotation",
    "move_linear_xyz_rotation": "move_linear_xyz_rotation",
    "dynamic_move": "dynamic_move",
    "get_motor_angles": "get_motor_angles",
    "get_T": "get_T",
    "get_pose": "get_pose",
    "gripper_on": "gripper_on",
    "gripper_off": "gripper_off",
    "servo_gripper": "servo_gripper",
    "set_free_mode": "set_free_mode",
    "robodk_simu": "robodk_simu",
}

#: Actions for which the *historical* server (2025-02, evidence C) could send no
#: reply. The current official doc says set_free_mode returns 0.1 (evidence B);
#: the tolerance is kept for old servers.
LEGACY_MAY_NOT_REPLY = frozenset({"set_free_mode"})


class EpisodeClient:
    def __init__(
        self,
        host: str = "localhost",
        port: int = 12345,
        *,
        legacy_compatible: bool = True,
        connection_policy: ConnectionPolicy = ConnectionPolicy.PER_COMMAND,
        timeouts: ClientTimeouts = ClientTimeouts(),
        max_message_size: int = protocol.DEFAULT_MAX_MESSAGE_SIZE,
        joint_limits: Optional[JointLimits] = JointLimits.episode_default(),
        limit_margin_deg: float = 0.0,
        speed_ratio_range: Sequence[float] = (0.01, 1.0),
        max_reach_mm: Optional[float] = 560.0,
        retry_policy: RetryPolicy = RetryPolicy(),
        action_names: Optional[Mapping[str, str]] = None,
        protocol_version: str = protocol.PROTOCOL_VERSION,
    ) -> None:
        self.host = host
        self.port = int(port)
        self.mode = ProtocolMode.LEGACY if legacy_compatible else ProtocolMode.ENHANCED
        self.connection_policy = ConnectionPolicy(connection_policy)
        self.timeouts = timeouts
        self.max_message_size = int(max_message_size)
        self.joint_limits = joint_limits
        self.limit_margin_deg = float(limit_margin_deg)
        self.speed_ratio_range = (float(speed_ratio_range[0]), float(speed_ratio_range[1]))
        self.max_reach_mm = max_reach_mm
        self.retry_policy = retry_policy
        self.actions = dict(DEFAULT_ACTIONS)
        if action_names:
            self.actions.update(action_names)
        self.protocol_version = protocol_version
        self._lock = threading.RLock()
        self._sock: Optional[socket.socket] = None
        self._closed = False

    # ------------------------------------------------------------------ lifecycle
    def __enter__(self) -> "EpisodeClient":
        if self.connection_policy is ConnectionPolicy.PERSISTENT:
            with self._lock:
                self._ensure_persistent(protocol.Deadline(self.timeouts.command_total_s))
        return self

    def __exit__(self, *exc: Any) -> None:
        self.close()

    def close(self) -> None:
        with self._lock:
            self._drop_socket()
            self._closed = True

    @property
    def address(self) -> tuple:
        return (self.host, self.port)

    # ------------------------------------------------------------------ transport
    def _connect(self, deadline: protocol.Deadline) -> socket.socket:
        rem = deadline.remaining()
        timeout = self.timeouts.connect_s if rem is None else min(self.timeouts.connect_s, max(rem, 0.0))
        if timeout <= 0:
            raise CommandTimeoutError("command deadline expired before connect")
        try:
            sock = socket.create_connection(self.address, timeout=timeout)
        except socket.timeout as exc:
            raise ConnectTimeoutError(f"connect to {self.host}:{self.port} timed out") from exc
        except OSError as exc:
            raise ConnectionFailedError(f"connect to {self.host}:{self.port} failed: {exc}") from exc
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        return sock

    def _ensure_persistent(self, deadline: protocol.Deadline) -> socket.socket:
        if self._sock is None:
            self._sock = self._connect(deadline)
        return self._sock

    def _drop_socket(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    def _roundtrip(self, sock: socket.socket, wire: Any, deadline: protocol.Deadline, action: str) -> Any:
        send_deadline = protocol.Deadline.min(deadline, protocol.Deadline(self.timeouts.send_s))
        protocol.write_message(sock, wire, self.max_message_size, send_deadline)
        recv_deadline = protocol.Deadline.min(deadline, protocol.Deadline(self.timeouts.receive_s))
        try:
            return protocol.read_message(sock, self.max_message_size, recv_deadline)
        except ReceiveTimeoutError:
            # Classify by which deadline was binding, not by re-checking the clock:
            # the socket timeout can fire a few ms before the absolute deadline
            # (Windows timer granularity), which made this flaky (1 in ~20 runs).
            if recv_deadline is deadline or deadline.expired():
                raise CommandTimeoutError(
                    f"{action}: command deadline of {self.timeouts.command_total_s}s exceeded"
                )
            raise

    def _transact(self, env: CommandEnvelope, *, dedicated: bool = False) -> Any:
        wire = env.to_wire(self.mode, self.protocol_version)
        t0 = time.perf_counter()
        deadline = protocol.Deadline(self.timeouts.command_total_s)

        def once() -> Any:
            if dedicated or self.connection_policy is ConnectionPolicy.PER_COMMAND:
                sock = self._connect(deadline)
                try:
                    return self._roundtrip(sock, wire, deadline, env.action)
                finally:
                    sock.close()
            with self._lock:
                sock = self._ensure_persistent(deadline)
                try:
                    return self._roundtrip(sock, wire, deadline, env.action)
                except EpisodeError:
                    # stream state is unknown after any error -> resync by reconnecting
                    self._drop_socket()
                    raise

        try:
            if dedicated:
                raw = call_with_retry(once, self.retry_policy, action=env.action)
            elif self.connection_policy is ConnectionPolicy.PER_COMMAND:
                with self._lock:
                    raw = call_with_retry(once, self.retry_policy, action=env.action)
            else:
                raw = call_with_retry(once, self.retry_policy, action=env.action)
        except ReceiveTimeoutError:
            if self.mode is ProtocolMode.LEGACY and env.action in LEGACY_MAY_NOT_REPLY:
                log.warning("%s: no reply (legacy server may not reply for this action)", env.action)
                return None
            raise
        finally:
            elapsed_ms = (time.perf_counter() - t0) * 1000.0
            log.debug(
                "%s done in %.1f ms", env.action, elapsed_ms,
                extra={"request_id": env.request_id, "action": env.action, "elapsed_ms": round(elapsed_ms, 2)},
            )
        return self._unwrap(env, raw)

    def _unwrap(self, env: CommandEnvelope, raw: Any) -> Any:
        if self.mode is ProtocolMode.LEGACY:
            return raw
        if not isinstance(raw, dict) or "ok" not in raw:
            raise MalformedPayloadError("enhanced reply must be an object with 'ok'", details={"got": raw})
        version = raw.get("protocol_version")
        if version is not None and str(version).split(".")[0] != self.protocol_version.split(".")[0]:
            raise ProtocolVersionError("incompatible protocol major version", details={"server": version})
        rid = raw.get("request_id")
        if rid is not None and rid != env.request_id:
            raise RequestIdMismatchError("reply belongs to another request", details={"sent": env.request_id, "got": rid})
        if not raw["ok"]:
            err = raw.get("error") or {}
            code = err.get("code", "REMOTE_COMMAND_FAILED")
            if code == NoSolutionError.code:
                raise NoSolutionError(err.get("message", "no IK solution"), details=err)
            raise RemoteCommandError(err.get("message", "remote command failed"), details=err)
        return raw.get("result")

    def send_raw(self, action: str, params: Any = None) -> Any:
        """Escape hatch: send an arbitrary action. No validation is applied."""
        return self._transact(CommandEnvelope(action=action, params=params))

    # ------------------------------------------------------------------ helpers
    def _motion_ack(self, action: str, result: Any, request_id: str) -> MotionAck:
        if isinstance(result, (int, float)) and not isinstance(result, bool):
            if result == -1:
                raise NoSolutionError(f"{action}: server reported no solution (-1)")
            if not math.isfinite(float(result)) or result < 0:
                raise MalformedPayloadError(f"{action}: invalid duration {result!r}")
            return MotionAck(action, float(result), raw=result, request_id=request_id)
        if result is None:
            return MotionAck(action, None, raw=None, request_id=request_id)
        if isinstance(result, dict) and "estimated_duration_s" in result:
            return MotionAck(action, float(result["estimated_duration_s"]), raw=result, request_id=request_id)
        raise MalformedPayloadError(f"{action}: unexpected reply {result!r}")

    def _check_joints(self, angles: Iterable[Any]) -> List[float]:
        return validate_joint_vector(list(angles) if not isinstance(angles, (list, tuple)) else angles,
                                     self.joint_limits, margin_deg=self.limit_margin_deg)

    def _check_speed(self, speed_ratio: Any) -> float:
        lo, hi = self.speed_ratio_range
        return validate_speed_ratio(speed_ratio, minimum=lo, maximum=hi)

    # ------------------------------------------------------------------ API
    def emergency_stop(self) -> Any:
        """Software emergency stop: ``{"action": "emergency_stop", "params": 1}``.

        Official behaviour (tid=1217 / tid=1191, evidence B): the arm stops and the
        server ignores further commands until released; returns 0.05. It does
        NOT stop the servo gripper. Uses its own connection; retried.
        This is not a substitute for a hardware E-stop that cuts motor power.
        """
        env = CommandEnvelope(self.actions["emergency_stop"], 1)
        return self._transact(env, dedicated=True)

    def release_emergency_stop(self) -> Any:
        """Release the software E-stop: ``params 0`` (evidence B).

        Callers must only do this after the physical cause is cleared; the
        safety layer additionally requires re-homing / re-declaration.
        """
        env = CommandEnvelope(self.actions["emergency_stop"], 0)
        return self._transact(env, dedicated=True)

    def home(self) -> MotionAck:
        """Historical action (evidence C); not in the current official API list."""
        env = CommandEnvelope(self.actions["home"], None)
        return self._motion_ack(env.action, self._transact(env), env.request_id)

    def angle_mode(self, angles: Sequence[float], speed_ratio: float = 1.0) -> MotionAck:
        """MoveJ in *motor* degrees. Reply: 0 = no motion needed, -1 = no solution,
        >0 = estimated duration in s (evidence B)."""
        q = self._check_joints(angles)
        s = self._check_speed(speed_ratio)
        env = CommandEnvelope(self.actions["angle_mode"], [q, s])
        return self._motion_ack(env.action, self._transact(env), env.request_id)

    def move_xyz_rotation(self, x: float, y: float, z: float,
                          rotation: Sequence[float] = (90.0, 0.0, 180.0), order: str = "zyx",
                          speed_ratio: Optional[float] = 1.0) -> MotionAck:
        """Pose move via the official IK. 8 params incl. speed_ratio (evidence B).

        ``speed_ratio=None`` sends the 7-param form understood by the historical
        server (evidence C), where a trailing speed would be misread as the order.
        """
        pose = validate_pose(Pose(x, y, z, *self._rot(rotation), order=order), max_reach_mm=self.max_reach_mm)
        speed = None if speed_ratio is None else self._check_speed(speed_ratio)
        env = CommandEnvelope(self.actions["move_xyz_rotation"], pose.to_legacy_params(speed))
        return self._motion_ack(env.action, self._transact(env), env.request_id)

    def move_linear_xyz_rotation(self, x: float, y: float, z: float,
                                 rotation: Sequence[float] = (90.0, 0.0, 180.0), order: str = "zyx") -> MotionAck:
        """Straight-line move, 7 params, speed not adjustable; the server replies
        with the total time first and then executes (evidence B)."""
        pose = validate_pose(Pose(x, y, z, *self._rot(rotation), order=order), max_reach_mm=self.max_reach_mm)
        env = CommandEnvelope(self.actions["move_linear_xyz_rotation"], pose.to_legacy_params())
        return self._motion_ack(env.action, self._transact(env), env.request_id)

    #: gear ratios and pulses/rev used by the official SDK demo for dynamic_move (evidence B)
    DYNAMIC_MOVE_GEAR_RATIOS = (25, 20, 25, 10, 4, 1)
    DYNAMIC_MOVE_PULSES_PER_REV = 3200

    def dynamic_move(self, goal_motor_deg: Sequence[float], motors_max_speed: Sequence[float],
                     current_motor_deg: Optional[Sequence[float]] = None) -> MotionAck:
        """Streaming joint target (official SDK demo V1.2.1, evidence B).

        Wire format: ``[{"1": q1, ..., "6": q6}, current_joint_pulses[6], motors_max_speed[6]]``
        with ``pulses = int(3200 * gear_ratio * angle / 360)``; reply fixed 0.05.
        Official prerequisites: driver-board "Response" set to None and the
        upper computer's status refresh disabled. Not in the API reference
        page (tid=1217); unit of ``motors_max_speed`` is not documented (E;
        the demo uses [90, 40, 100, 100, 100, 10] * 0.5).
        """
        q = self._check_joints(goal_motor_deg)
        cur = list(current_motor_deg) if current_motor_deg is not None else self.get_motor_angles()
        if any(c is None for c in cur):
            raise ReadFailedError("dynamic_move needs all 6 current angles")
        cur_f = validate_joint_vector(cur, None, name="current_motor_deg")
        speeds = validate_joint_vector(motors_max_speed, None, name="motors_max_speed")
        if any(s <= 0 for s in speeds):
            raise ParameterTypeError("motors_max_speed must be > 0", details={"got": speeds})
        pulses = [int(self.DYNAMIC_MOVE_PULSES_PER_REV * g * a / 360)
                  for g, a in zip(self.DYNAMIC_MOVE_GEAR_RATIOS, cur_f)]
        goal = {str(i + 1): float(v) for i, v in enumerate(q)}
        env = CommandEnvelope(self.actions["dynamic_move"], [goal, pulses, speeds])
        return self._motion_ack(env.action, self._transact(env), env.request_id)

    def _read(self, action: str, params: Any = None) -> Any:
        """Idempotent read; a ``None`` reply is retried like a transport error."""

        def once() -> Any:
            raw = self._transact(CommandEnvelope(self.actions[action], params))
            if raw is None:
                raise ReadFailedError(f"{action}: server returned None (CAN busy or read failure)")
            return raw

        policy = RetryPolicy(max_attempts=self.retry_policy.max_attempts,
                             initial_backoff_s=self.retry_policy.initial_backoff_s,
                             retry_on=(ReadFailedError,))
        return call_with_retry(once, policy, action=self.actions[action])

    def get_motor_angles(self) -> List[Optional[float]]:
        """Motor-side encoder angles in degrees.

        Whole-reply ``None`` (official doc, B) -> ReadFailedError. Per-entry
        ``None`` (seen in the upper computer UI when a joint has no CAN contact,
        FAQ tid=1932) is kept so the cache can mark that joint invalid.
        """
        return validate_joint_reply(self._read("get_motor_angles"))

    def get_transform(self) -> List[List[float]]:
        """4x4 homogeneous transform of the flange (``get_T``, evidence B)."""
        raw = self._read("get_T")
        if not (isinstance(raw, list) and len(raw) == 4 and all(isinstance(r, list) and len(r) == 4 for r in raw)):
            raise MalformedPayloadError("get_T must return a 4x4 matrix", details={"got": raw})
        return [[validate_number(v, "T") for v in row] for row in raw]

    def get_pose(self, order: str = "zyx") -> List[float]:
        """Flange ``[x, y, z, r1, r2, r3]``; ``order`` is ``"zyx"`` or ``"xyz"`` (evidence B)."""
        raw = self._read("get_pose", validate_euler_order(order))
        if not isinstance(raw, list) or len(raw) < 6:
            raise MalformedPayloadError("get_pose must return >= 6 numbers", details={"got": raw})
        return [validate_number(v, "pose") for v in raw[:6]]

    def gripper_on(self) -> Any:
        return self._transact(CommandEnvelope(self.actions["gripper_on"], None))

    def gripper_off(self) -> Any:
        return self._transact(CommandEnvelope(self.actions["gripper_off"], None))

    def servo_gripper(self, angle: float) -> Any:
        """Integer angle 0..110; larger = closes. Not stopped by emergency_stop (evidence B)."""
        return self._transact(CommandEnvelope(self.actions["servo_gripper"], validate_servo_angle(angle)))

    def set_free_mode(self, enable: bool) -> Any:
        """Enable/disable free (back-drivable) mode for teaching.

        DANGER: with free mode on, the arm may fall under gravity. The
        historical client waited 10 s and printed a warning before enabling;
        here the *caller* is responsible for supporting the arm.
        """
        return self._transact(CommandEnvelope(self.actions["set_free_mode"], validate_bool_flag(enable, "enable")))

    def robodk_simulation(self, enable: bool) -> Any:
        return self._transact(CommandEnvelope(self.actions["robodk_simu"], validate_bool_flag(enable, "enable")))

    # ------------------------------------------------------------------ misc
    @staticmethod
    def _rot(rotation: Sequence[float]) -> List[float]:
        if isinstance(rotation, (str, bytes)) or len(rotation) != 3:
            raise ParameterTypeError("rotation must have 3 elements", details={"got": rotation})
        return list(rotation)

    def ping(self) -> float:
        """Round-trip time of one ``get_motor_angles`` in seconds."""
        t0 = time.perf_counter()
        self.get_motor_angles()
        return time.perf_counter() - t0


__all__ = ["EpisodeClient", "DEFAULT_ACTIONS", "TransportError", "ConnectionClosedError"]
