"""ROS2 interface node for the Episode 1 arm (safety-layer front end).

STATUS: run under ROS2 Jazzy (WSL2 Ubuntu 24.04) against the TCP Mock upper
computer; runtime smoke test 17/17 (episode_ros2/test/ros2_runtime_smoke.py,
reports/ros2_runtime_wsl.md). NOT run on the real robot.

Topology
--------
::

    MoveIt / user ──FollowJointTrajectory (action)──┐
                  ──Trigger/SetBool (services)──────┤
                  ──/episode/estop (topic)──────────┤
                                                    ▼
                                   episode_robot_interface (this node)
                                      SafeRobotController (pure Python)
                                         │ hardware_lock (one bus transaction at a time)
                                         ▼
                      VendorTcpAdapter ──TCP/JSON──► official upper computer ──CAN──► drivers
                      (or MockHardwareAdapter)

Design decisions (details: docs/09, docs/10, docs/15)
-----------------------------------------------------
* Motion = Action. A motion lasts seconds, needs feedback (desired / actual /
  error) and must be cancellable. A Service would block the caller and give
  no progress or cancel semantics.
* Services only for short transactions: home request, stop, estop, clear
  fault, gripper, free mode. They return as soon as the request was
  *accepted* — homing progress is visible on /episode/robot_state.
* Callback groups + MultiThreadedExecutor:
    - ``poll_group``   MutuallyExclusive: the feedback timer (never overlaps itself)
    - ``action_group`` Reentrant: goal / cancel / execute callbacks
    - ``srv_group``    MutuallyExclusive: ordinary services
    - ``estop_group``  MutuallyExclusive, *separate*: E-stop service + topic,
      so an E-stop is never queued behind another service callback.
  The executor provides parallelism, NOT safety: the hardware is protected by
  ``SafeRobotController.hardware_lock`` and the single-owner CommandArbiter.
* /joint_states carries ONLY measured feedback (``actual``), converted to
  model radians through the joint mapping. It is not published when the
  feedback is stale or incomplete — publishing old or planned values would
  make RViz/MoveIt believe something false.
* Cancel = physical stop. ``cancel_callback`` accepts; the execute loop then
  calls ``controller.cancel()`` which issues a stop (or ESTOP if the backend
  cannot soft-stop) before reporting the goal as canceled.
"""

from __future__ import annotations

import math
import threading
import time
from typing import List, Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from control_msgs.action import FollowJointTrajectory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, Float64MultiArray, String
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

from episode_hardware.command_arbiter import TicketStatus
from episode_hardware.joint_mapping import JointMapping
from episode_hardware.mock_adapter import MockHardwareAdapter, SimulatedArm
from episode_hardware.joint_mapping import VERTICAL_MOTOR_DEG
from episode_hardware.robot_controller import ControllerConfig, SafeRobotController
from episode_hardware.safety_supervisor import SafetyConfig
from episode_hardware.state_cache import JointStateSnapshot
from episode_hardware.vendor_adapter import VendorTcpAdapter
from episode_sdk import EpisodeClient
from episode_sdk.exceptions import EpisodeError
from episode_sdk.models import ClientTimeouts, JointLimits

JOINT_STATE_QOS = QoSProfile(  # reliable so robot_state_publisher/MoveIt (reliable subs) can connect
    history=HistoryPolicy.KEEP_LAST, depth=10,
    reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
LATCHED_QOS = QoSProfile(  # late joiners get the current robot state
    history=HistoryPolicy.KEEP_LAST, depth=1,
    reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
ESTOP_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE)


class RobotInterfaceNode(Node):
    def __init__(self) -> None:
        super().__init__("episode_robot_interface")
        p = self.declare_parameter
        p("backend", "mock")                       # mock | vendor_tcp
        p("host", "127.0.0.1")
        p("port", 12345)
        p("legacy_compatible", True)
        p("read_timeout_s", 0.3)                   # must be < stale_after_s
        p("joint_names", ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"])
        p("mapping_signs", [1, 1, 1, 1, 1, 1])  # evidence B: official node uses motor - offset
        p("mapping_offsets_deg", [180.0, 90.0, 83.0, 30.0, 110.0, 30.0])
        p("joint_lower_deg", [0.0] * 6)
        p("joint_upper_deg", [340.0, 180.0, 163.0, 335.0, 220.0, 335.0])
        p("poll_rate_hz", 20.0)
        p("stale_after_s", 0.5)
        p("speed_ratio_cap", 0.3)
        p("final_tolerance_deg", 1.0)
        p("waypoint_min_spacing_deg", 5.0)
        p("assume_vendor_homed", False)
        p("action_name", "episode_arm_controller/follow_joint_trajectory")

        gp = self.get_parameter
        self.joint_names: List[str] = list(gp("joint_names").value)
        limits = JointLimits(tuple(gp("joint_lower_deg").value), tuple(gp("joint_upper_deg").value))
        self.mapping = JointMapping("ros_model", tuple(int(s) for s in gp("mapping_signs").value),
                                    tuple(float(o) for o in gp("mapping_offsets_deg").value), limits,
                                    evidence="B")
        self.speed_cap = float(gp("speed_ratio_cap").value)
        self.spacing = float(gp("waypoint_min_spacing_deg").value)

        cfg = ControllerConfig(
            poll_rate_hz=float(gp("poll_rate_hz").value), stale_after_s=float(gp("stale_after_s").value),
            safety=SafetyConfig(limits=limits, speed_ratio_default_cap=self.speed_cap,
                                final_tolerance_deg=float(gp("final_tolerance_deg").value)))
        self.controller = SafeRobotController(self._make_backend(limits), cfg)
        self.controller.configure()
        if gp("assume_vendor_homed").value:
            self.controller.poll_once()
            self.controller.declare_homed("parameter assume_vendor_homed")

        self.poll_group = MutuallyExclusiveCallbackGroup()
        self.action_group = ReentrantCallbackGroup()
        self.srv_group = MutuallyExclusiveCallbackGroup()
        self.estop_group = MutuallyExclusiveCallbackGroup()

        self.js_pub = self.create_publisher(JointState, "joint_states", JOINT_STATE_QOS)
        self.motor_pub = self.create_publisher(Float64MultiArray, "episode/motor_angles", JOINT_STATE_QOS)
        self.state_pub = self.create_publisher(String, "episode/robot_state", LATCHED_QOS)
        self.diag_pub = self.create_publisher(DiagnosticArray, "diagnostics", 10)
        self._last_state: Optional[str] = None
        self._last_published_stamp: Optional[float] = None

        self.create_timer(1.0 / cfg.poll_rate_hz, self._poll, callback_group=self.poll_group)
        self.create_timer(1.0, self._publish_diagnostics, callback_group=self.poll_group)

        self.create_service(Trigger, "~/emergency_stop", self._srv_estop, callback_group=self.estop_group)
        self.create_subscription(Empty, "episode/estop", self._topic_estop, ESTOP_QOS, callback_group=self.estop_group)
        self.create_service(Trigger, "~/home", self._srv_home, callback_group=self.srv_group)
        self.create_service(Trigger, "~/stop", self._srv_stop, callback_group=self.srv_group)
        self.create_service(Trigger, "~/clear_fault", self._srv_clear_fault, callback_group=self.srv_group)
        self.create_service(Trigger, "~/declare_homed", self._srv_declare_homed, callback_group=self.srv_group)
        self.create_service(SetBool, "~/release_estop", self._srv_release_estop, callback_group=self.srv_group)
        self.create_service(SetBool, "~/gripper", self._srv_gripper, callback_group=self.srv_group)
        self.create_service(SetBool, "~/set_free_mode", self._srv_free_mode, callback_group=self.srv_group)

        self._action = ActionServer(
            self, FollowJointTrajectory, str(gp("action_name").value),
            execute_callback=self._execute, goal_callback=self._on_goal, cancel_callback=self._on_cancel,
            callback_group=self.action_group)
        self._goal_lock = threading.Lock()
        self._goal_active = False
        self.get_logger().info(f"episode_robot_interface up, backend={gp('backend').value}, "
                               f"hardware={self.controller.hw.capabilities.description}")

    # ------------------------------------------------------------------ backend
    def _make_backend(self, limits: JointLimits):
        backend = self.get_parameter("backend").value
        if backend == "mock":
            return MockHardwareAdapter(SimulatedArm(initial_motor_deg=VERTICAL_MOTOR_DEG, limits=limits))
        if backend == "vendor_tcp":
            rt = float(self.get_parameter("read_timeout_s").value)
            client = EpisodeClient(self.get_parameter("host").value, int(self.get_parameter("port").value),
                                   legacy_compatible=bool(self.get_parameter("legacy_compatible").value),
                                   timeouts=ClientTimeouts(1.0, 0.5, rt, rt + 0.2), joint_limits=limits)
            return VendorTcpAdapter(client)
        raise ValueError(f"unknown backend {backend!r}")

    # ------------------------------------------------------------------ feedback
    def _poll(self) -> None:
        snap = self.controller.poll_once()
        self._publish_feedback(snap)
        st = self.controller.state.value
        if st != self._last_state:
            self.state_pub.publish(String(data=st))
            self._last_state = st

    def _publish_feedback(self, snap: JointStateSnapshot) -> None:
        if not snap.usable:
            return  # never publish stale / partial data as if it were current
        if snap.stamp == self._last_published_stamp:
            return  # no NEW sample (read failed this cycle) -> do not re-publish the old one
        self._last_published_stamp = snap.stamp
        motor = [float(v) for v in snap.positions_deg]  # type: ignore[arg-type]
        model_rad = self.mapping.motor_to_model_rad(motor)
        js = JointState()
        # stamp = time the sample was taken, not "now"
        age_ns = int((snap.age_s or 0.0) * 1e9)
        js.header.stamp = rclpy.time.Time(nanoseconds=self.get_clock().now().nanoseconds - age_ns).to_msg()
        js.name = self.joint_names
        js.position = [float(v) for v in model_rad]
        vel = snap.velocity_deg_s
        if all(v is not None for v in vel):
            js.velocity = [math.radians(v) for v in self.mapping.model_velocity_from_motor(vel)]  # type: ignore[arg-type]
        self.js_pub.publish(js)
        self.motor_pub.publish(Float64MultiArray(data=motor))

    def _publish_diagnostics(self) -> None:
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        for d in self.controller.diagnostics():
            s = DiagnosticStatus()
            s.level = bytes([{"OK": 0, "WARN": 1, "ERROR": 2, "STALE": 3}[d["level"]]])
            s.name, s.message, s.hardware_id = d["name"], d["message"], "episode1"
            s.values = [KeyValue(key=k, value=v) for k, v in d["values"].items()]
            arr.status.append(s)
        self.diag_pub.publish(arr)

    # ------------------------------------------------------------------ action
    def _on_goal(self, goal: FollowJointTrajectory.Goal) -> GoalResponse:
        traj = goal.trajectory
        if sorted(traj.joint_names) != sorted(self.joint_names) or not traj.points:
            self.get_logger().warn("goal rejected: joint names mismatch or empty trajectory")
            return GoalResponse.REJECT
        with self._goal_lock:
            if self._goal_active:
                self.get_logger().warn("goal rejected: another trajectory is executing (single owner)")
                return GoalResponse.REJECT
        try:
            for pt in traj.points:
                self._to_motor(traj.joint_names, pt.positions)
        except (ValueError, EpisodeError) as exc:
            self.get_logger().warn(f"goal rejected: {exc}")
            return GoalResponse.REJECT
        if not self.controller.sm.motion_allowed():
            self.get_logger().warn(f"goal rejected in state {self.controller.state.value}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT  # the execute loop performs the physical stop

    def _to_motor(self, names: List[str], positions) -> List[float]:
        order = [list(names).index(n) for n in self.joint_names]
        model = [float(positions[i]) for i in order]
        if any(not math.isfinite(v) for v in model):
            raise ValueError("non-finite position")
        motor = self.mapping.model_rad_to_motor(model)
        if not self.mapping.within_motor_limits(motor):
            raise ValueError(f"point outside joint limits: {[round(m, 1) for m in motor]}")
        return motor

    def _waypoints(self, traj) -> List[List[float]]:
        """Subsample to waypoints >= spacing apart; always keep the final point.

        The official API is point-to-point (each command plans its own
        profile), so the trajectory is executed as stop-and-go segments.
        Timing (time_from_start) is NOT honoured — documented limitation.
        """
        pts = [self._to_motor(traj.joint_names, p.positions) for p in traj.points]
        out: List[List[float]] = []
        for q in pts[:-1]:
            ref = out[-1] if out else self.controller.cache.snapshot().require_positions()
            if max(abs(a - b) for a, b in zip(q, ref)) >= self.spacing:
                out.append(q)
        out.append(pts[-1])
        return out

    def _execute(self, goal_handle):
        result = FollowJointTrajectory.Result()
        with self._goal_lock:
            self._goal_active = True
        try:
            waypoints = self._waypoints(goal_handle.request.trajectory)
            for idx, q in enumerate(waypoints):
                try:
                    ticket = self.controller.move_joints(q, self.speed_cap, source="ros_action")
                except EpisodeError as exc:
                    goal_handle.abort()
                    result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                    result.error_string = f"waypoint {idx} rejected: {exc.code}"
                    return result
                while not ticket.done:
                    if goal_handle.is_cancel_requested:
                        self.controller.cancel(ticket.command_id, "action cancel")
                        goal_handle.canceled()
                        result.error_string = "canceled; stop issued"
                        return result
                    goal_handle.publish_feedback(self._feedback(q))
                    time.sleep(0.1)
                if ticket.status is not TicketStatus.SUCCEEDED:
                    goal_handle.abort()
                    result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                    result.error_string = f"waypoint {idx}: {ticket.status.value} {ticket.reason}"
                    return result
            goal_handle.succeed()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return result
        finally:
            with self._goal_lock:
                self._goal_active = False

    def _feedback(self, desired_motor: List[float]) -> FollowJointTrajectory.Feedback:
        fb = FollowJointTrajectory.Feedback()
        fb.joint_names = self.joint_names
        fb.header.stamp = self.get_clock().now().to_msg()
        des = JointTrajectoryPoint(positions=[float(v) for v in self.mapping.motor_to_model_rad(desired_motor)])
        fb.desired = des
        snap = self.controller.cache.snapshot()
        if snap.usable:
            act = [float(v) for v in self.mapping.motor_to_model_rad([float(p) for p in snap.positions_deg])]  # type: ignore[arg-type]
            fb.actual = JointTrajectoryPoint(positions=act)
            fb.error = JointTrajectoryPoint(positions=[d - a for d, a in zip(des.positions, act)])
        return fb

    # ------------------------------------------------------------------ services
    def _trigger(self, response, fn, ok_msg: str):
        try:
            fn()
            response.success, response.message = True, ok_msg
        except EpisodeError as exc:
            response.success, response.message = False, f"{exc.code}: {exc.message}"
        return response

    def _srv_estop(self, _req, res):
        return self._trigger(res, lambda: self.controller.emergency_stop("ros service"), "ESTOP")

    def _topic_estop(self, _msg) -> None:
        try:
            self.controller.emergency_stop("ros topic")
        except EpisodeError as exc:
            self.get_logger().fatal(f"software ESTOP failed ({exc}); USE THE HARDWARE E-STOP")

    def _srv_home(self, _req, res):
        return self._trigger(res, self.controller.home, "homing started; watch episode/robot_state")

    def _srv_stop(self, _req, res):
        return self._trigger(res, lambda: self.controller.request_stop("ros stop service"), "stop requested")

    def _srv_clear_fault(self, _req, res):
        return self._trigger(res, lambda: self.controller.clear_fault("ros operator"), "fault cleared -> INACTIVE")

    def _srv_declare_homed(self, _req, res):
        return self._trigger(res, lambda: self.controller.declare_homed("ros operator"), "declared homed -> READY")

    def _srv_release_estop(self, req, res):
        return self._trigger(res, lambda: self.controller.release_estop("ros operator", hardware_reset_confirmed=req.data),
                             "ESTOP released -> INACTIVE (re-home required)")

    def _srv_gripper(self, req, res):
        def go():
            with self.controller.hardware_lock:
                self.controller.hw.set_gripper(bool(req.data))
        return self._trigger(res, go, "gripper closed" if req.data else "gripper opened")

    def _srv_free_mode(self, req, res):
        if req.data and self.controller.state.value not in ("INACTIVE", "READY"):
            res.success, res.message = False, "free mode only from INACTIVE/READY"
            return res

        def go():
            with self.controller.hardware_lock:
                self.controller.hw.set_free_mode(bool(req.data))
        return self._trigger(res, go, "free mode ON — support the arm!" if req.data else "free mode OFF")

    def destroy_node(self) -> bool:
        self.controller.shutdown()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobotInterfaceNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
