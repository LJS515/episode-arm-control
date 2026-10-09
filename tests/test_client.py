"""SDK client against the Mock Server (Synthetic / Mock Data).

Covers: connect timeout (6), response timeout (7), server close (8),
fragmented replies (1,2), truncated reply (3), invalid JSON (4), oversize (5),
plus legacy/enhanced envelopes, retries, and the dedicated E-stop path."""

import socket
import threading
import time

import pytest

from episode_sdk import EpisodeClient
from episode_sdk.exceptions import (
    CommandTimeoutError,
    ConnectionClosedError,
    ConnectionFailedError,
    ConnectTimeoutError,
    FrameTooLargeError,
    JointLimitError,
    MalformedPayloadError,
    NoSolutionError,
    ParameterTypeError,
    ReadFailedError,
    ReceiveTimeoutError,
    RemoteCommandError,
    RequestIdMismatchError,
    TruncatedFrameError,
)
from episode_sdk import protocol
from episode_sdk.models import ClientTimeouts, ConnectionPolicy, ProtocolMode
from episode_sdk.retry import RetryPolicy

FAST = ClientTimeouts(connect_s=1.0, send_s=1.0, receive_s=0.5, command_total_s=1.5)


def client_for(srv, **kw):
    kw.setdefault("timeouts", FAST)
    kw.setdefault("legacy_compatible", srv.mode is ProtocolMode.LEGACY)
    return EpisodeClient(srv.host, srv.port, **kw)


# ----------------------------------------------------------------------------- legacy envelope
def test_legacy_request_shape_is_exactly_action_params(legacy_server):
    c = client_for(legacy_server)
    c.get_motor_angles()
    # official SDK V1.2.1 omits "params" for parameterless commands
    assert legacy_server.requests[-1] == {"action": "get_motor_angles"}


def test_legacy_angle_mode_params_shape(legacy_server):
    c = client_for(legacy_server)
    ack = c.angle_mode([180, 90, 83, 30, 110, 30], 0.5)
    req = legacy_server.requests[-1]
    assert req == {"action": "angle_mode", "params": [[180.0, 90.0, 83.0, 30.0, 110.0, 30.0], 0.5]}
    assert ack.estimated_duration_s == 0.0


def test_get_motor_angles_legacy(legacy_server):
    angles = client_for(legacy_server).get_motor_angles()
    assert angles == pytest.approx([180, 90, 83, 30, 110, 30])


def test_motion_ack_is_estimate_not_completion(legacy_server):
    c = client_for(legacy_server)
    ack = c.angle_mode([200, 90, 83, 30, 110, 30], 1.0)
    assert ack.estimated_duration_s and ack.estimated_duration_s > 0
    # immediately after the ack the robot is still on its way
    assert c.get_motor_angles()[0] < 199.0


def test_none_joint_reported(legacy_server):
    legacy_server.faults.none_joints = [3]
    angles = client_for(legacy_server).get_motor_angles()
    assert angles[2] is None and angles[0] is not None


def test_no_solution_legacy(legacy_server):
    legacy_server.arm.faults.ik_fail = True
    with pytest.raises(NoSolutionError):
        client_for(legacy_server).move_xyz_rotation(250, 0, 300)


def test_move_xyz_rotation_mock_ik(legacy_server):
    ack = client_for(legacy_server).move_xyz_rotation(250, 50, 380, (90, 0, 180))
    assert ack.estimated_duration_s is not None


def test_get_transform_and_pose(legacy_server):
    c = client_for(legacy_server)
    T = c.get_transform()
    assert len(T) == 4 and T[3] == [0, 0, 0, 1]
    pose = c.get_pose()
    assert pose[:3] == pytest.approx([T[0][3], T[1][3], T[2][3]])


def test_gripper_servo_free_mode(legacy_server):
    c = client_for(legacy_server)
    c.gripper_on()
    assert legacy_server.arm.gripper_closed
    c.gripper_off()
    assert not legacy_server.arm.gripper_closed
    c.servo_gripper(45)
    assert legacy_server.arm.servo_angle == 45
    assert legacy_server.requests[-1] == {"action": "servo_gripper", "params": 45}
    # current official server replies 0.1 to set_free_mode (doc tid=1217)
    assert c.set_free_mode(True) == 0.1
    assert legacy_server.arm.free_mode


def test_free_mode_historical_server_without_reply(legacy_server):
    legacy_server.legacy_free_mode_no_reply = True  # 2025-02 public server behaviour
    c = client_for(legacy_server, retry_policy=RetryPolicy.none())
    assert c.set_free_mode(True) is None
    assert legacy_server.arm.free_mode


def test_estop_and_release_params_follow_official_doc(legacy_server):
    c = client_for(legacy_server)
    assert c.emergency_stop() == 0.05
    assert legacy_server.requests[-1] == {"action": "emergency_stop", "params": 1}
    assert legacy_server.arm.estopped
    assert c.release_emergency_stop() == 0.05
    assert legacy_server.requests[-1] == {"action": "emergency_stop", "params": 0}
    assert not legacy_server.arm.estopped


def test_move_xyz_rotation_sends_8_params_with_speed(legacy_server):
    c = client_for(legacy_server)
    c.move_xyz_rotation(250, 50, 380, (90, 0, 180), "zyx", speed_ratio=0.4)
    assert legacy_server.requests[-1]["params"] == [250, 50, 380, 90, 0, 180, "zyx", 0.4]
    c.move_xyz_rotation(250, 50, 380, speed_ratio=None)  # historical 7-param form
    assert len(legacy_server.requests[-1]["params"]) == 7
    c.move_linear_xyz_rotation(250, 50, 380)
    assert len(legacy_server.requests[-1]["params"]) == 7


def test_unsupported_euler_order_rejected_before_send(legacy_server):
    c = client_for(legacy_server)
    n = len(legacy_server.requests)
    with pytest.raises(ParameterTypeError):
        c.move_xyz_rotation(250, 50, 380, order="xzy")
    assert len(legacy_server.requests) == n


def test_get_pose_sends_rotation_order(legacy_server):
    c = client_for(legacy_server)
    p_zyx = c.get_pose()
    assert legacy_server.requests[-1] == {"action": "get_pose", "params": "zyx"}
    p_xyz = c.get_pose("xyz")
    assert legacy_server.requests[-1]["params"] == "xyz"
    assert p_zyx[:3] == pytest.approx(p_xyz[:3])


def test_null_read_reply_is_read_failed_and_retried(legacy_server):
    legacy_server.faults.null_reads = 1
    c = client_for(legacy_server, retry_policy=RetryPolicy(max_attempts=3, initial_backoff_s=0.01))
    assert len(c.get_motor_angles()) == 6  # first answer null, retry succeeds
    legacy_server.faults.null_reads = 5
    c2 = client_for(legacy_server, retry_policy=RetryPolicy.none())
    with pytest.raises(ReadFailedError):
        c2.get_motor_angles()
    with pytest.raises(ReadFailedError):
        c2.get_transform()


def test_dynamic_move_and_home(legacy_server):
    c = client_for(legacy_server)
    ack = c.dynamic_move([181, 90, 83, 30, 110, 30], [45, 20, 50, 50, 50, 5])
    assert ack.estimated_duration_s == pytest.approx(0.05)
    goal, pulses, speeds = legacy_server.requests[-1]["params"]
    assert goal == {"1": 181.0, "2": 90.0, "3": 83.0, "4": 30.0, "5": 110.0, "6": 30.0}
    # pulses = int(3200 * gear * angle / 360), gear = [25, 20, 25, 10, 4, 1] (official demo)
    assert pulses == [int(3200 * g * a / 360) for g, a in zip([25, 20, 25, 10, 4, 1], [180, 90, 83, 30, 110, 30])]
    assert speeds == [45, 20, 50, 50, 50, 5]
    assert c.home().estimated_duration_s == pytest.approx(0.5)


def test_unknown_action_legacy_times_out(legacy_server):
    c = client_for(legacy_server, retry_policy=RetryPolicy.none())
    with pytest.raises((ReceiveTimeoutError, CommandTimeoutError)):
        c.send_raw("does_not_exist")


def test_emergency_stop(legacy_server):
    c = client_for(legacy_server)
    c.emergency_stop()
    assert legacy_server.arm.estopped


# ----------------------------------------------------------------------------- enhanced envelope
def test_enhanced_request_has_id_and_version(enhanced_server):
    c = client_for(enhanced_server)
    c.get_motor_angles()
    req = enhanced_server.requests[-1]
    assert req["protocol_version"] == protocol.PROTOCOL_VERSION
    assert req["request_id"].startswith("req-")


def test_enhanced_unknown_action_structured_error(enhanced_server):
    with pytest.raises(RemoteCommandError) as ei:
        client_for(enhanced_server).send_raw("nope")
    assert ei.value.details["code"] == "UNKNOWN_ACTION"


def test_enhanced_no_solution(enhanced_server):
    enhanced_server.arm.faults.ik_fail = True
    with pytest.raises(NoSolutionError):
        client_for(enhanced_server).move_linear_xyz_rotation(250, 0, 300)


def test_request_id_mismatch_detected():
    srv = socket.socket()
    srv.bind(("127.0.0.1", 0))
    srv.listen(1)
    port = srv.getsockname()[1]

    def serve():
        conn, _ = srv.accept()
        protocol.read_message(conn, deadline=protocol.Deadline(2))
        protocol.write_message(conn, {"protocol_version": "1.0", "request_id": "someone-else", "ok": True, "result": 1})
        conn.close()

    th = threading.Thread(target=serve)
    th.start()
    c = EpisodeClient("127.0.0.1", port, legacy_compatible=False, timeouts=FAST, retry_policy=RetryPolicy.none())
    with pytest.raises(RequestIdMismatchError):
        c.get_motor_angles()
    th.join()
    srv.close()


# ----------------------------------------------------------------------------- validation before send
def test_validation_blocks_send(legacy_server):
    c = client_for(legacy_server)
    n = len(legacy_server.requests)
    with pytest.raises(JointLimitError):
        c.angle_mode([400, 90, 83, 30, 110, 30])
    assert len(legacy_server.requests) == n


# ----------------------------------------------------------------------------- transport faults
def test_connection_refused():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()  # nothing listens here now
    c = EpisodeClient("127.0.0.1", port, timeouts=FAST, retry_policy=RetryPolicy.none())
    # Linux returns RST immediately (ConnectionFailedError). Windows retries the
    # SYN for ~2 s before reporting "refused", so a 1 s connect timeout fires first.
    with pytest.raises((ConnectionFailedError, ConnectTimeoutError)):
        c.get_motor_angles()


def test_connect_timeout(monkeypatch):
    def fake_create_connection(*a, **k):
        raise socket.timeout("simulated")

    monkeypatch.setattr(socket, "create_connection", fake_create_connection)
    c = EpisodeClient("192.0.2.1", 12345, timeouts=FAST, retry_policy=RetryPolicy.none())
    with pytest.raises(ConnectTimeoutError):
        c.get_motor_angles()


def test_response_timeout(legacy_server):
    legacy_server.faults.reply_delay_s = 1.0
    c = client_for(legacy_server, retry_policy=RetryPolicy.none())
    t0 = time.monotonic()
    with pytest.raises((ReceiveTimeoutError, CommandTimeoutError)):
        c.get_motor_angles()
    assert time.monotonic() - t0 < 1.0


def test_command_total_deadline(legacy_server):
    legacy_server.faults.reply_delay_s = 2.0
    c = client_for(legacy_server, timeouts=ClientTimeouts(1.0, 1.0, 5.0, 0.4), retry_policy=RetryPolicy.none())
    t0 = time.monotonic()
    with pytest.raises(CommandTimeoutError):
        c.get_motor_angles()
    assert time.monotonic() - t0 < 1.0


def test_server_closes_connection(legacy_server):
    legacy_server.faults.close_after_request = True
    c = client_for(legacy_server, retry_policy=RetryPolicy.none())
    with pytest.raises(ConnectionClosedError):
        c.get_motor_angles()


def test_truncated_reply(legacy_server):
    legacy_server.faults.truncate_reply_to = 12
    with pytest.raises(TruncatedFrameError):
        client_for(legacy_server, retry_policy=RetryPolicy.none()).get_motor_angles()


def test_garbage_reply(legacy_server):
    legacy_server.faults.garbage_payload = True
    with pytest.raises(MalformedPayloadError):
        client_for(legacy_server).get_motor_angles()


def test_oversize_reply_header(legacy_server):
    legacy_server.faults.oversize_header = True
    with pytest.raises(FrameTooLargeError):
        client_for(legacy_server).get_motor_angles()


def test_fragmented_reply_is_reassembled(legacy_server):
    legacy_server.faults.fragment_size = 3
    legacy_server.faults.fragment_pause_s = 0.002
    assert client_for(legacy_server).get_motor_angles()[1] == pytest.approx(90)


# ----------------------------------------------------------------------------- retry semantics
def test_reads_are_retried(legacy_server):
    legacy_server.faults.drop_next_replies = 1
    c = client_for(legacy_server, retry_policy=RetryPolicy(max_attempts=3, initial_backoff_s=0.01))
    assert c.get_motor_angles() is not None
    assert sum(1 for r in legacy_server.requests if r["action"] == "get_motor_angles") == 2


def test_motion_is_never_retried(legacy_server):
    legacy_server.faults.drop_next_replies = 1
    c = client_for(legacy_server, retry_policy=RetryPolicy(max_attempts=3, initial_backoff_s=0.01))
    with pytest.raises((ReceiveTimeoutError, CommandTimeoutError)):
        c.angle_mode([181, 90, 83, 30, 110, 30])
    assert sum(1 for r in legacy_server.requests if r["action"] == "angle_mode") == 1


# ----------------------------------------------------------------------------- connection policy / estop
def test_persistent_connection_many_requests(legacy_server):
    with client_for(legacy_server, connection_policy=ConnectionPolicy.PERSISTENT) as c:
        for _ in range(20):
            c.get_motor_angles()
    assert len(legacy_server.requests) == 20


def test_persistent_connection_recovers_after_error(legacy_server):
    c = client_for(legacy_server, connection_policy=ConnectionPolicy.PERSISTENT, retry_policy=RetryPolicy.none())
    legacy_server.faults.garbage_payload = True
    with pytest.raises(MalformedPayloadError):
        c.get_motor_angles()
    legacy_server.faults.garbage_payload = False
    assert c.get_motor_angles() is not None
    c.close()


def test_estop_not_blocked_by_hanging_command(legacy_server):
    """A stuck request must not delay the emergency stop."""
    c = client_for(legacy_server, timeouts=ClientTimeouts(1.0, 1.0, 3.0, 3.0), retry_policy=RetryPolicy.none())
    legacy_server.faults.drop_next_replies = 1
    th = threading.Thread(target=lambda: pytest.raises(Exception, c.get_motor_angles))
    th.start()
    time.sleep(0.2)
    t0 = time.monotonic()
    c.emergency_stop()
    assert time.monotonic() - t0 < 0.5
    assert legacy_server.arm.estopped
    th.join()


def test_concurrent_calls_are_serialised(legacy_server):
    c = client_for(legacy_server, connection_policy=ConnectionPolicy.PERSISTENT)
    errors = []

    def worker():
        try:
            for _ in range(10):
                assert len(c.get_motor_angles()) == 6
        except Exception as e:  # pragma: no cover - failure path
            errors.append(e)

    ths = [threading.Thread(target=worker) for _ in range(4)]
    [t.start() for t in ths]
    [t.join() for t in ths]
    c.close()
    assert not errors
