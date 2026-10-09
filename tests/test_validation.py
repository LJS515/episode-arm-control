"""Parameter validation. Covers: wrong array length (9), NaN/Inf (10),
joint limits (11), invalid speed (12)."""

import math

import pytest

from episode_sdk.exceptions import (
    JointCountError,
    JointLimitError,
    NonFiniteValueError,
    ParameterTypeError,
    SpeedRatioError,
)
from episode_sdk.models import JointLimits, Pose
from episode_sdk.validation import (
    validate_bool_flag,
    validate_joint_reply,
    validate_joint_vector,
    validate_pose,
    validate_servo_angle,
    validate_speed_ratio,
)

LIM = JointLimits.episode_default()
OK = [180, 90, 83, 30, 110, 30]


def test_valid_vector_passes():
    assert validate_joint_vector(OK, LIM) == [float(v) for v in OK]


@pytest.mark.parametrize("n", [0, 5, 7])
def test_wrong_length(n):
    with pytest.raises(JointCountError):
        validate_joint_vector([1.0] * n, LIM)


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), -math.inf])
def test_non_finite(bad):
    v = list(OK)
    v[3] = bad
    with pytest.raises(NonFiniteValueError):
        validate_joint_vector(v, LIM)


@pytest.mark.parametrize("bad", ["10", None, True, [1], {}])
def test_wrong_types(bad):
    v = list(OK)
    v[0] = bad
    with pytest.raises(ParameterTypeError):
        validate_joint_vector(v, LIM)


def test_string_is_not_a_sequence_of_numbers():
    with pytest.raises(ParameterTypeError):
        validate_joint_vector("123456", LIM)


@pytest.mark.parametrize("joint,value", [(0, -0.1), (0, 340.1), (1, 181), (2, 164), (3, 336), (4, 221), (5, -5)])
def test_joint_limits(joint, value):
    v = list(OK)
    v[joint] = value
    with pytest.raises(JointLimitError) as ei:
        validate_joint_vector(v, LIM)
    assert ei.value.details["violations"][0]["joint"] == joint + 1


def test_limit_boundaries_inclusive():
    validate_joint_vector([0, 0, 0, 0, 0, 0], LIM)
    validate_joint_vector([340, 180, 163, 335, 220, 335], LIM)


def test_soft_margin():
    with pytest.raises(JointLimitError):
        validate_joint_vector([1, 90, 83, 30, 110, 30], LIM, margin_deg=2.0)


@pytest.mark.parametrize("bad", [0, -0.5, 1.01, 5, float("nan"), "1", True])
def test_speed_ratio_invalid(bad):
    with pytest.raises(SpeedRatioError):
        validate_speed_ratio(bad)


@pytest.mark.parametrize("ok", [0.01, 0.5, 1.0, 1])
def test_speed_ratio_valid(ok):
    assert validate_speed_ratio(ok) == float(ok)


def test_pose_validation():
    validate_pose(Pose(250, 0, 300))
    with pytest.raises(NonFiniteValueError):
        validate_pose(Pose(float("nan"), 0, 0))
    with pytest.raises(ParameterTypeError):
        validate_pose(Pose(250, 0, 300, order="abc"))
    with pytest.raises(JointLimitError):
        validate_pose(Pose(2000, 0, 300), max_reach_mm=560)


def test_servo_and_flags():
    assert validate_servo_angle(90) == 90
    assert validate_servo_angle(45.4) == 45 and isinstance(validate_servo_angle(45.4), int)
    validate_servo_angle(110)
    for bad in (111, 270, -1):  # official range 0..110 (doc tid=1217)
        with pytest.raises(ParameterTypeError):
            validate_servo_angle(bad)
    assert validate_bool_flag(True, "x") == 1
    assert validate_bool_flag(0, "x") == 0
    with pytest.raises(ParameterTypeError):
        validate_bool_flag(2, "x")


def test_joint_reply_keeps_none():
    assert validate_joint_reply([1, None, 3, 4, 5, 6])[1] is None
    with pytest.raises(ParameterTypeError):
        validate_joint_reply([1, 2, 3])
    with pytest.raises(NonFiniteValueError):
        validate_joint_reply([1, 2, 3, 4, 5, float("inf")])


def test_joint_limits_model_validation():
    with pytest.raises(ValueError):
        JointLimits((0,) * 5, (1,) * 6)
    with pytest.raises(ValueError):
        JointLimits((10,) * 6, (5,) * 6)
