"""Structured exception hierarchy for the Episode 1 SDK and control layer.

Every exception carries a stable machine-readable ``code`` so that callers
(ROS2 nodes, test harnesses, log analysers) can branch on the error kind
without parsing human-readable messages.

Hierarchy::

    EpisodeError
    ├── ProtocolError            framing / payload problems
    │   ├── FrameTooLargeError
    │   ├── TruncatedFrameError
    │   ├── MalformedPayloadError
    │   ├── ProtocolVersionError
    │   └── RequestIdMismatchError
    ├── TransportError           socket-level problems
    │   ├── ConnectionFailedError
    │   ├── ConnectTimeoutError
    │   ├── SendTimeoutError
    │   ├── ReceiveTimeoutError
    │   ├── CommandTimeoutError
    │   └── ConnectionClosedError
    ├── ValidationError          rejected before anything is sent
    │   ├── JointCountError
    │   ├── NonFiniteValueError
    │   ├── JointLimitError
    │   ├── SpeedRatioError
    │   └── ParameterTypeError
    ├── RemoteError              server answered, but with a failure
    │   ├── NoSolutionError
    │   ├── ReadFailedError
    │   └── RemoteCommandError
    ├── SafetyError              rejected by the safety layer
    │   ├── NotHomedError
    │   ├── StateRejectedError
    │   ├── EStopActiveError
    │   ├── FaultLatchedError
    │   └── StaleStateError
    └── ArbitrationError         rejected by the command arbiter
        ├── CommandRejectedError
        ├── CommandPreemptedError
        └── CommandCancelledError
"""

from __future__ import annotations

from typing import Any, Dict, Optional


class EpisodeError(Exception):
    """Base class for all project errors."""

    code: str = "EPISODE_ERROR"
    #: Whether retrying the *same* request may succeed. Motion commands must
    #: never be retried automatically regardless of this flag.
    retryable: bool = False

    def __init__(self, message: str = "", *, details: Optional[Dict[str, Any]] = None):
        super().__init__(message or self.__class__.__name__)
        self.message = message or self.__class__.__name__
        self.details: Dict[str, Any] = dict(details or {})

    def to_dict(self) -> Dict[str, Any]:
        return {
            "code": self.code,
            "type": self.__class__.__name__,
            "message": self.message,
            "retryable": self.retryable,
            "details": self.details,
        }

    def __str__(self) -> str:  # pragma: no cover - trivial
        if self.details:
            return f"[{self.code}] {self.message} {self.details}"
        return f"[{self.code}] {self.message}"


# --------------------------------------------------------------------------- protocol
class ProtocolError(EpisodeError):
    code = "PROTOCOL_ERROR"


class FrameTooLargeError(ProtocolError):
    code = "FRAME_TOO_LARGE"


class TruncatedFrameError(ProtocolError):
    code = "FRAME_TRUNCATED"


class MalformedPayloadError(ProtocolError):
    code = "PAYLOAD_MALFORMED"


class ProtocolVersionError(ProtocolError):
    code = "PROTOCOL_VERSION_MISMATCH"


class RequestIdMismatchError(ProtocolError):
    code = "REQUEST_ID_MISMATCH"


# --------------------------------------------------------------------------- transport
class TransportError(EpisodeError):
    code = "TRANSPORT_ERROR"
    retryable = True


class ConnectionFailedError(TransportError):
    code = "CONNECTION_FAILED"


class ConnectTimeoutError(TransportError):
    code = "CONNECT_TIMEOUT"


class SendTimeoutError(TransportError):
    code = "SEND_TIMEOUT"


class ReceiveTimeoutError(TransportError):
    code = "RECEIVE_TIMEOUT"


class CommandTimeoutError(TransportError):
    code = "COMMAND_TIMEOUT"


class ConnectionClosedError(TransportError):
    code = "CONNECTION_CLOSED"


# --------------------------------------------------------------------------- validation
class ValidationError(EpisodeError):
    code = "VALIDATION_ERROR"


class JointCountError(ValidationError):
    code = "JOINT_COUNT_INVALID"


class NonFiniteValueError(ValidationError):
    code = "NON_FINITE_VALUE"


class JointLimitError(ValidationError):
    code = "JOINT_LIMIT_VIOLATION"


class SpeedRatioError(ValidationError):
    code = "SPEED_RATIO_INVALID"


class ParameterTypeError(ValidationError):
    code = "PARAMETER_TYPE_INVALID"


# --------------------------------------------------------------------------- remote
class RemoteError(EpisodeError):
    code = "REMOTE_ERROR"


class NoSolutionError(RemoteError):
    """The server reported ``-1`` (no IK solution / planning failed)."""

    code = "NO_IK_SOLUTION"


class ReadFailedError(RemoteError):
    """The server answered ``None`` to a read (get_motor_angles / get_T / get_pose).

    Official API doc (forum tid=1217, evidence B): reads can fail when the CAN
    bus is busy, e.g. while other control commands run, and then return None.
    Retryable: the read is idempotent.
    """

    code = "READ_FAILED"
    retryable = True


class RemoteCommandError(RemoteError):
    code = "REMOTE_COMMAND_FAILED"


# --------------------------------------------------------------------------- safety
class SafetyError(EpisodeError):
    code = "SAFETY_REJECTED"


class NotHomedError(SafetyError):
    code = "NOT_HOMED"


class StateRejectedError(SafetyError):
    code = "STATE_REJECTED"


class EStopActiveError(SafetyError):
    code = "ESTOP_ACTIVE"


class FaultLatchedError(SafetyError):
    code = "FAULT_LATCHED"


class StaleStateError(SafetyError):
    code = "STATE_STALE"


# --------------------------------------------------------------------------- arbitration
class ArbitrationError(EpisodeError):
    code = "ARBITRATION_ERROR"


class CommandRejectedError(ArbitrationError):
    code = "COMMAND_REJECTED"


class CommandPreemptedError(ArbitrationError):
    code = "COMMAND_PREEMPTED"


class CommandCancelledError(ArbitrationError):
    code = "COMMAND_CANCELLED"


__all__ = [name for name in globals() if name.endswith("Error")]
