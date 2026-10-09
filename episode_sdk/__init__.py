"""Hardened Python SDK for the Episode 1 official TCP/JSON upper computer.

This package is *client-side* work of this project. The server, CAN stack,
joint drivers (FOC) and IK are official/vendor components.
"""

from .client import EpisodeClient
from .exceptions import *  # noqa: F401,F403
from .models import ClientTimeouts, ConnectionPolicy, JointLimits, MotionAck, Pose, ProtocolMode
from .protocol import DEFAULT_MAX_MESSAGE_SIZE, PROTOCOL_VERSION, recv_exact

__version__ = "0.1.0"

__all__ = [
    "EpisodeClient",
    "ClientTimeouts",
    "ConnectionPolicy",
    "JointLimits",
    "MotionAck",
    "Pose",
    "ProtocolMode",
    "DEFAULT_MAX_MESSAGE_SIZE",
    "PROTOCOL_VERSION",
    "recv_exact",
]
