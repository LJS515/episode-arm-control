"""Control / safety layer that sits between callers and the official stack.

Nothing in this package talks to CAN directly; CAN is owned by the
official upper computer (route A) or the official ROS2 control core
(route B).
"""

from .command_arbiter import BusyPolicy, CommandArbiter, CommandTicket, TicketStatus
from .interfaces import HardwareCapabilities, HardwareError, HardwareReadError, RobotHardware
from .joint_mapping import PROFILES, ROS_MODEL, VENDOR_DH, VERTICAL_MOTOR_DEG, JointMapping, get_profile
from .mock_adapter import FaultInjection, MockHardwareAdapter, SimulatedArm
from .robot_controller import ControllerConfig, SafeRobotController
from .safety_supervisor import SafetyConfig, SafetySupervisor
from .state_cache import JointStateCache, JointStateSnapshot
from .state_machine import Event, RobotState, RobotStateMachine, STATE_SPECS
from .trajectory_monitor import MonitorStatus, TrajectoryMonitor
from .vendor_adapter import VendorTcpAdapter

__all__ = [
    "BusyPolicy", "CommandArbiter", "CommandTicket", "TicketStatus",
    "HardwareCapabilities", "HardwareError", "HardwareReadError", "RobotHardware",
    "PROFILES", "ROS_MODEL", "VENDOR_DH", "VERTICAL_MOTOR_DEG", "JointMapping", "get_profile",
    "FaultInjection", "MockHardwareAdapter", "SimulatedArm",
    "ControllerConfig", "SafeRobotController",
    "SafetyConfig", "SafetySupervisor",
    "JointStateCache", "JointStateSnapshot",
    "Event", "RobotState", "RobotStateMachine", "STATE_SPECS",
    "MonitorStatus", "TrajectoryMonitor",
    "VendorTcpAdapter",
]
