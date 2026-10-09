"""Exercise robot_interface_node logic with *stub* ROS modules.

This does NOT prove the node works under ROS2 Jazzy (no DDS, no executor,
no real message types). It proves the node's own logic — goal validation,
waypoint conversion, execute/cancel flow, services — against the Mock.
Covers: Action cancel (15) at the ROS-adapter level.
"""

import enum
import sys
import threading
import time
import types

import pytest


# ----------------------------------------------------------------------------- stub ROS
class _Msg:
    _defaults = {}

    def __init__(self, **kw):
        for k, v in self._defaults.items():
            setattr(self, k, v() if callable(v) else v)
        for k, v in kw.items():
            setattr(self, k, v)


def _msg(cls_name, **defaults):
    return type(cls_name, (_Msg,), {"_defaults": defaults})


Stamp = _msg("Stamp", sec=0, nanosec=0)
Header = _msg("Header", stamp=Stamp)


class _Param:
    def __init__(self, v):
        self.value = v


class _Pub:
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m)


class _Logger:
    def __init__(self):
        self.lines = []

    def __getattr__(self, level):
        return lambda msg: self.lines.append((level, msg))


class _Now:
    def __init__(self):
        self.nanoseconds = time.monotonic_ns()

    def to_msg(self):
        return Stamp()


class _Clock:
    def now(self):
        return _Now()


class Node:
    def __init__(self, name):
        self._params, self.pubs, self.services, self._logger = {}, {}, {}, _Logger()

    def declare_parameter(self, name, default):
        self._params.setdefault(name, default)

    def get_parameter(self, name):
        return _Param(self._params[name])

    def create_publisher(self, typ, topic, qos):
        self.pubs[topic] = _Pub()
        return self.pubs[topic]

    def create_timer(self, *a, **k):
        return None

    def create_service(self, typ, name, cb, **k):
        self.services[name] = cb

    def create_subscription(self, *a, **k):
        return None

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return _Clock()

    def destroy_node(self):
        return True


class GoalResponse(enum.Enum):
    ACCEPT = 1
    REJECT = 2


class CancelResponse(enum.Enum):
    ACCEPT = 1
    REJECT = 2


class ActionServer:
    def __init__(self, node, typ, name, **kw):
        node.action_server = self
        self.kw = kw


def _install_stubs():
    m = lambda n: sys.modules.setdefault(n, types.ModuleType(n))  # noqa: E731
    rclpy = m("rclpy")
    rclpy.init = lambda args=None: None
    rclpy.try_shutdown = lambda: None
    rclpy.time = types.SimpleNamespace(Time=lambda nanoseconds=0: _Now())
    m("rclpy.node").Node = Node
    act = m("rclpy.action")
    act.ActionServer, act.CancelResponse, act.GoalResponse = ActionServer, CancelResponse, GoalResponse
    cg = m("rclpy.callback_groups")
    cg.MutuallyExclusiveCallbackGroup = cg.ReentrantCallbackGroup = lambda: object()
    m("rclpy.executors").MultiThreadedExecutor = object
    qos = m("rclpy.qos")
    qos.QoSProfile = lambda **kw: kw
    qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=1)
    qos.ReliabilityPolicy = types.SimpleNamespace(RELIABLE=1, BEST_EFFORT=2)
    qos.DurabilityPolicy = types.SimpleNamespace(VOLATILE=1, TRANSIENT_LOCAL=2)

    Point = _msg("JointTrajectoryPoint", positions=list)
    Traj = _msg("JointTrajectory", joint_names=list, points=list)
    Result = _msg("Result", error_code=0, error_string="")
    Result.SUCCESSFUL, Result.INVALID_GOAL, Result.GOAL_TOLERANCE_VIOLATED = 0, -1, -5
    FJT = types.SimpleNamespace(Goal=_msg("Goal", trajectory=Traj), Result=Result,
                                Feedback=_msg("Feedback", header=Header, joint_names=list, desired=Point,
                                              actual=Point, error=Point))
    m("control_msgs")
    m("control_msgs.action").FollowJointTrajectory = FJT
    d = m("diagnostic_msgs.msg")
    d.DiagnosticArray = _msg("DiagnosticArray", header=Header, status=list)
    d.DiagnosticStatus = _msg("DiagnosticStatus", level=b"\x00", name="", message="", hardware_id="", values=list)
    d.KeyValue = _msg("KeyValue", key="", value="")
    m("sensor_msgs.msg").JointState = _msg("JointState", header=Header, name=list, position=list, velocity=list)
    s = m("std_msgs.msg")
    s.Empty, s.Float64MultiArray, s.String = _msg("Empty"), _msg("F64", data=list), _msg("String", data="")
    srv = m("std_srvs.srv")
    srv.Trigger = types.SimpleNamespace(Request=_msg("Req"), Response=_msg("Res", success=False, message=""))
    srv.SetBool = srv.Trigger
    m("trajectory_msgs.msg").JointTrajectoryPoint = Point
    return FJT, Traj, Point


FJT, Traj, Point = _install_stubs()
sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parents[1] / "episode_ros2"))
from episode_ros2.robot_interface_node import RobotInterfaceNode  # noqa: E402

NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


class FakeGoalHandle:
    def __init__(self, goal):
        self.request = goal
        self.is_cancel_requested = False
        self.outcome = None
        self.feedback = []

    def publish_feedback(self, fb):
        self.feedback.append(fb)

    def succeed(self):
        self.outcome = "succeeded"

    def abort(self):
        self.outcome = "aborted"

    def canceled(self):
        self.outcome = "canceled"


@pytest.fixture
def node():
    n = RobotInterfaceNode()
    n.controller.poll_once()
    stop = threading.Event()

    def spin():
        while not stop.is_set():
            n._poll()
            time.sleep(0.02)

    th = threading.Thread(target=spin, daemon=True)
    th.start()
    yield n
    stop.set()
    th.join(1)
    n.destroy_node()


def goal(points, names=NAMES):
    return FJT.Goal(trajectory=Traj(joint_names=list(names), points=[Point(positions=p) for p in points]))


def trigger(n, name):
    return n.services[name](None, sys.modules["std_srvs.srv"].Trigger.Response())


def test_goal_rejected_before_homing(node):
    assert node._on_goal(goal([[0.05, 0, 0, 0, 0, 0]])) is GoalResponse.REJECT


def test_goal_validation(node):
    assert trigger(node, "~/declare_homed").success
    assert node._on_goal(goal([[0.05, 0, 0, 0, 0, 0]], names=NAMES[:5] + ["x"])) is GoalResponse.REJECT
    assert node._on_goal(goal([[float("nan"), 0, 0, 0, 0, 0]])) is GoalResponse.REJECT
    assert node._on_goal(goal([[10.0, 0, 0, 0, 0, 0]])) is GoalResponse.REJECT  # far outside limits
    assert node._on_goal(goal([])) is GoalResponse.REJECT
    assert node._on_goal(goal([[0.05, 0, 0, 0, 0, 0]])) is GoalResponse.ACCEPT


def test_joint_order_is_remapped(node):
    trigger(node, "~/declare_homed")
    rev = list(reversed(NAMES))
    motor = node._to_motor(rev, [0, 0, 0, 0, 0, 0.1])  # 0.1 rad on joint1
    assert motor[0] == pytest.approx(180 + 5.7296, abs=1e-3)


def test_execute_succeeds_with_feedback(node):
    trigger(node, "~/declare_homed")
    gh = FakeGoalHandle(goal([[0.02, 0, 0, 0, 0, 0], [0.1, 0, 0, 0, 0, 0]]))
    res = node._execute(gh)
    assert gh.outcome == "succeeded" and res.error_code == 0
    assert node.pubs["joint_states"].msgs[-1].position[0] == pytest.approx(0.1, abs=0.02)


def test_execute_cancel_stops_motion(node):
    trigger(node, "~/declare_homed")
    gh = FakeGoalHandle(goal([[1.0, 0, 0, 0, 0, 0]]))
    t = threading.Timer(0.3, lambda: setattr(gh, "is_cancel_requested", True))
    t.start()
    node._execute(gh)
    assert gh.outcome == "canceled"
    arm = node.controller.hw.arm
    p0 = arm.true_positions()[0]
    time.sleep(0.3)
    assert arm.true_positions()[0] == pytest.approx(p0, abs=1e-6)  # really stopped
    assert p0 < 180 + 57.0  # did not reach the 1 rad target
    assert len(gh.feedback) >= 1


def test_estop_service_and_release(node):
    trigger(node, "~/declare_homed")
    assert trigger(node, "~/emergency_stop").success
    assert node.controller.state.value == "ESTOP"
    req = types.SimpleNamespace(data=True)
    res = node.services["~/release_estop"](req, sys.modules["std_srvs.srv"].SetBool.Response())
    assert res.success and node.controller.state.value == "INACTIVE"


def test_stale_feedback_not_published(node):
    trigger(node, "~/declare_homed")
    time.sleep(0.1)
    n_before = len(node.pubs["joint_states"].msgs)
    node.controller.hw.arm.faults.fail_all_reads = True
    time.sleep(0.3)
    n_after = len(node.pubs["joint_states"].msgs)
    time.sleep(0.3)
    assert len(node.pubs["joint_states"].msgs) == n_after  # nothing new while failing
    assert n_after >= n_before
