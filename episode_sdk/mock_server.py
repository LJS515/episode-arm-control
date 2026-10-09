"""Mock of the official TCP/JSON upper computer.

All data served by this module is **Synthetic / Mock Data**.

Behaviour that intentionally mirrors the historical server (evidence C):
* 8-byte big-endian length + payload framing; one thread per client; a
  connection can carry many requests.
* Motion commands reply with an *estimated duration* (seconds) or ``-1``
  for "no solution"; the reply is sent while the motion is still running.
* In LEGACY mode an unknown action produces *no reply at all* (the
  historical server only replied when ``result is not None``).

Additions (ENHANCED mode only): request_id echo, protocol_version,
structured ``{"ok", "result", "error"}`` replies.

Fault injection (``MockFaults``) lets tests exercise short reads,
fragmented headers/payloads, truncated replies, garbage JSON, oversize
headers, slow replies, dropped replies and abrupt closes.

Run standalone::

    python -m episode_sdk.mock_server --port 12345 --mode legacy
"""

from __future__ import annotations

import argparse
import json
import logging
import socket
import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

from . import protocol
from .exceptions import EpisodeError
from .models import ProtocolMode, Pose

log = logging.getLogger("episode_sdk.mock_server")


@dataclass
class MockFaults:
    reply_delay_s: float = 0.0
    #: do not send any reply for the next N requests
    drop_next_replies: int = 0
    #: send only the first K bytes of the reply frame, then close
    truncate_reply_to: Optional[int] = None
    #: send the reply in chunks of this size with a pause between them
    fragment_size: Optional[int] = None
    fragment_pause_s: float = 0.01
    #: replace the payload with invalid JSON
    garbage_payload: bool = False
    #: announce a huge payload length in the header
    oversize_header: bool = False
    #: close the connection right after reading a request
    close_after_request: bool = False
    #: per-joint None in get_motor_angles
    none_joints: List[int] = field(default_factory=list)
    #: next N reads (get_motor_angles / get_T / get_pose) answer JSON null,
    #: as the official server does when the CAN bus is busy (doc tid=1217)
    null_reads: int = 0


class MockEpisodeServer:
    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 0,
        *,
        mode: ProtocolMode = ProtocolMode.LEGACY,
        arm: Any = None,
        homed: bool = True,
        max_message_size: int = protocol.DEFAULT_MAX_MESSAGE_SIZE,
    ) -> None:
        # Lazy import keeps episode_sdk importable without episode_hardware.
        from episode_hardware.joint_mapping import VERTICAL_MOTOR_DEG
        from episode_hardware.mock_adapter import SimulatedArm

        self.mode = ProtocolMode(mode)
        self.arm = arm or SimulatedArm(initial_motor_deg=VERTICAL_MOTOR_DEG, homed=homed, homing_duration_s=0.5)
        self.faults = MockFaults()
        #: emulate the historical server, which sent no reply to set_free_mode
        self.legacy_free_mode_no_reply = False
        self.max_message_size = max_message_size
        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind((host, port))
        self._srv.listen(16)
        self._srv.settimeout(0.2)
        self.host, self.port = self._srv.getsockname()[:2]
        self._running = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._clients: List[socket.socket] = []
        #: one lock models the single shared CAN bus behind the server
        self.bus_lock = threading.Lock()
        self.requests: List[Dict[str, Any]] = []
        self._req_lock = threading.Lock()

    # ------------------------------------------------------------------ lifecycle
    def start(self) -> "MockEpisodeServer":
        self._running.set()
        self._thread = threading.Thread(target=self._accept_loop, name="mock-accept", daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        self._running.clear()
        try:
            self._srv.close()
        except OSError:
            pass
        for c in list(self._clients):
            # shutdown() first wakes up a recv() blocked in a client thread cleanly;
            # a bare close() under it yields EBADF in that thread.
            try:
                c.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                c.close()
            except OSError:
                pass
        if self._thread:
            self._thread.join(timeout=2.0)

    def __enter__(self) -> "MockEpisodeServer":
        return self.start()

    def __exit__(self, *exc: Any) -> None:
        self.stop()

    @property
    def address(self) -> Tuple[str, int]:
        return (self.host, self.port)

    # ------------------------------------------------------------------ networking
    def _accept_loop(self) -> None:
        while self._running.is_set():
            try:
                conn, _ = self._srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            self._clients.append(conn)
            threading.Thread(target=self._client_loop, args=(conn,), daemon=True).start()

    def _client_loop(self, conn: socket.socket) -> None:
        try:
            while self._running.is_set():
                try:
                    msg = protocol.read_message(conn, self.max_message_size, protocol.Deadline(30.0))
                except EpisodeError as exc:
                    log.debug("client loop ends: %s", exc.code)
                    break
                with self._req_lock:
                    self.requests.append(msg)
                if self.faults.close_after_request:
                    break
                reply = self._dispatch(msg)
                if reply is _NO_REPLY:
                    continue
                if reply is _SEND_NULL:
                    reply = None
                if self.faults.drop_next_replies > 0:
                    self.faults.drop_next_replies -= 1
                    continue
                if self.faults.reply_delay_s:
                    time.sleep(self.faults.reply_delay_s)
                try:
                    self._send(conn, reply)
                except OSError:
                    break  # client went away (e.g. it timed out first)
                if self.faults.truncate_reply_to is not None:
                    break
        finally:
            try:
                conn.close()
            except OSError:
                pass
            if conn in self._clients:
                self._clients.remove(conn)

    def _send(self, conn: socket.socket, reply: Any) -> None:
        payload = b"{not json" if self.faults.garbage_payload else protocol.encode_payload(reply)
        header = struct.pack(">Q", (1 << 40) if self.faults.oversize_header else len(payload))
        frame = header + payload
        if self.faults.truncate_reply_to is not None:
            frame = frame[: self.faults.truncate_reply_to]
        if self.faults.fragment_size:
            for i in range(0, len(frame), self.faults.fragment_size):
                conn.sendall(frame[i : i + self.faults.fragment_size])
                time.sleep(self.faults.fragment_pause_s)
        else:
            conn.sendall(frame)

    # ------------------------------------------------------------------ command handling
    def _dispatch(self, msg: Any) -> Any:
        if not isinstance(msg, dict) or "action" not in msg:
            return self._wrap(msg, error=("BAD_REQUEST", "request must be an object with 'action'"))
        action = msg.get("action")
        params = msg.get("params")
        handler = getattr(self, f"_h_{action}", None)
        if handler is None:
            if self.mode is ProtocolMode.LEGACY:
                return _NO_REPLY  # historical behaviour: silence
            return self._wrap(msg, error=("UNKNOWN_ACTION", f"unknown action {action!r}"))
        try:
            with self.bus_lock:
                result = handler(params)
        except _NoSolution:
            if self.mode is ProtocolMode.LEGACY:
                return -1
            return self._wrap(msg, error=("NO_IK_SOLUTION", "no IK solution"))
        except Exception as exc:  # noqa: BLE001 - mock reports any failure
            if self.mode is ProtocolMode.LEGACY:
                return _NO_REPLY
            return self._wrap(msg, error=("REMOTE_COMMAND_FAILED", str(exc)))
        if result is _NULL:
            result = None
            if self.mode is ProtocolMode.LEGACY:
                return _SEND_NULL
        elif self.mode is ProtocolMode.LEGACY and result is None:
            return _NO_REPLY
        return self._wrap(msg, result=result)

    def _wrap(self, msg: Any, *, result: Any = None, error: Optional[Tuple[str, str]] = None) -> Any:
        if self.mode is ProtocolMode.LEGACY:
            return result
        rid = msg.get("request_id") if isinstance(msg, dict) else None
        out: Dict[str, Any] = {"protocol_version": protocol.PROTOCOL_VERSION, "request_id": rid, "ok": error is None}
        if error is None:
            out["result"] = result
        else:
            out["error"] = {"code": error[0], "message": error[1]}
        return out

    # handlers -----------------------------------------------------------
    def _null_read(self) -> bool:
        if self.faults.null_reads > 0:
            self.faults.null_reads -= 1
            return True
        return False

    def _h_get_motor_angles(self, _p: Any) -> Any:
        if self._null_read():
            return _NULL
        pos: List[Optional[float]] = [round(v, 3) for v in self.arm.true_positions()]
        for j in self.faults.none_joints:
            pos[j - 1] = None
        return pos

    def _h_angle_mode(self, p: Any) -> Any:
        angles, speed = p[0], (p[1] if len(p) > 1 else 1.0)
        return round(self.arm.start_move([float(a) for a in angles], float(speed)), 3)

    def _h_dynamic_move(self, p: Any) -> Any:
        # official format (SDK demo V1.2.1): [{"1": q1, ..., "6": q6}, pulses[6], max_speed[6]] -> 0.05
        goal = p[0]
        q = [float(goal[str(i + 1)]) for i in range(6)]
        self.arm.start_move(q, 1.0)
        return 0.05

    def _pose(self, p: Any) -> Pose:
        return Pose(float(p[0]), float(p[1]), float(p[2]), float(p[3]), float(p[4]), float(p[5]), str(p[6]))

    def _h_move_xyz_rotation(self, p: Any) -> Any:
        # official: [x,y,z,r1,r2,r3,order,speed_ratio]; speed clamped to 0..1
        speed = min(1.0, max(0.0, float(p[7]))) if len(p) > 7 else 1.0
        if str(p[6]).lower() not in ("zyx", "xyz"):
            raise _NoSolution()
        sol = self.arm.solve_ik(self._pose(p))
        if sol is None:
            raise _NoSolution()
        return round(self.arm.start_move(sol, max(speed, 0.01)), 3)

    def _h_move_linear_xyz_rotation(self, p: Any) -> Any:
        # Mock: no Cartesian interpolation, joint move to the IK solution.
        return self._h_move_xyz_rotation(list(p[:7]))

    def _h_get_T(self, _p: Any) -> Any:
        from episode_hardware.kinematics import fk_motor_deg

        if self._null_read():
            return _NULL

        return [[round(v, 6) for v in row] for row in fk_motor_deg(self.arm.true_positions())]

    def _h_get_pose(self, p: Any) -> Any:
        import math

        if self._null_read():
            return _NULL
        t = self._h_get_T(None)
        if str(p or "zyx").lower() == "xyz":
            # R = Rx(a) Ry(b) Rz(c)
            b = math.degrees(math.asin(max(-1.0, min(1.0, t[0][2]))))
            a = math.degrees(math.atan2(-t[1][2], t[2][2]))
            c = math.degrees(math.atan2(-t[0][1], t[0][0]))
            return [t[0][3], t[1][3], t[2][3], round(a, 4), round(b, 4), round(c, 4)]
        # zyx euler from rotation matrix: R = Rz(a) Ry(b) Rx(c)
        b = math.degrees(math.asin(max(-1.0, min(1.0, -t[2][0]))))
        a = math.degrees(math.atan2(t[1][0], t[0][0]))
        c = math.degrees(math.atan2(t[2][1], t[2][2]))
        return [t[0][3], t[1][3], t[2][3], round(a, 4), round(b, 4), round(c, 4)]

    def _h_home(self, _p: Any) -> Any:
        return self.arm.home()

    def _h_emergency_stop(self, p: Any) -> Any:
        # official: params 1 = stop, 0 = release; returns 0.05 (doc tid=1217)
        if p == 0:
            self.arm.release_estop()
        else:
            self.arm.emergency_stop()
        return 0.05

    def _h_reset_estop(self, _p: Any) -> Any:  # mock-only helper
        self.arm.release_estop()
        return 0

    def _h_gripper_on(self, _p: Any) -> Any:
        self.arm.gripper_closed = True
        return 0.05

    def _h_gripper_off(self, _p: Any) -> Any:
        self.arm.gripper_closed = False
        return 0.05

    def _h_servo_gripper(self, p: Any) -> Any:
        self.arm.servo_angle = float(min(110, max(0, int(round(float(p))))))  # official clamps 0..110
        return 1

    def _h_set_free_mode(self, p: Any) -> Any:
        self.arm.stop()
        self.arm.free_mode = bool(p)
        if self.legacy_free_mode_no_reply:
            return None  # historical server (C): no reply
        return 0.1  # current official doc (B)

    def _h_robodk_simu(self, p: Any) -> Any:
        return 0.05


class _NoSolution(Exception):
    pass


_NO_REPLY = object()
#: handler result meaning "reply with JSON null" (official read failure)
_NULL = object()
#: dispatch result: send the JSON value null in legacy mode
_SEND_NULL = object()


def main(argv: Optional[List[str]] = None) -> None:
    ap = argparse.ArgumentParser(description="Episode 1 mock TCP server (Synthetic / Mock Data)")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=12345)
    ap.add_argument("--mode", choices=[m.value for m in ProtocolMode], default="legacy")
    args = ap.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    srv = MockEpisodeServer(args.host, args.port, mode=ProtocolMode(args.mode)).start()
    log.info("mock server listening on %s:%d (mode=%s) — Synthetic / Mock Data", srv.host, srv.port, args.mode)
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        srv.stop()


if __name__ == "__main__":
    main()
