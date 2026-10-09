import math

import pytest

from episode_hardware import kinematics
from episode_hardware.joint_mapping import (
    ROS_MODEL,
    VENDOR_DH,
    VENDOR_DH_QLIM_DEG,
    VERTICAL_MOTOR_DEG,
    JointMapping,
    check_dh_qlim_consistency,
    get_profile,
)


def vendor_formula(m):
    # literal copy of EpisodeAPP.calculate_T_based_on_degrees (evidence C)
    return [m[0] - 180, 180 - m[1], 83 - m[2], m[3] - 30, 110 - m[4], m[5] - 30]


def vendor_inverse(q):
    # literal copy of EpisodeAPP.convert_motor_angles (evidence C)
    return [180 + q[0], 180 - q[1], 83 - q[2], 30 + q[3], 110 - q[4], 30 + q[5]]


@pytest.mark.parametrize("m", [VERTICAL_MOTOR_DEG, (0, 0, 0, 0, 0, 0), (340, 180, 163, 335, 220, 335), (12.5, 33, 140, 7, 200, 300)])
def test_vendor_dh_matches_official_formulas(m):
    assert VENDOR_DH.motor_to_model_deg(m) == pytest.approx(vendor_formula(m))
    q = vendor_formula(m)
    assert VENDOR_DH.model_to_motor_deg(q) == pytest.approx(vendor_inverse(q))


@pytest.mark.parametrize("mapping", [VENDOR_DH, ROS_MODEL])
def test_roundtrip(mapping):
    m = [100.0, 45.0, 120.0, 200.0, 50.0, 10.0]
    assert mapping.model_to_motor_deg(mapping.motor_to_model_deg(m)) == pytest.approx(m)
    assert mapping.model_rad_to_motor(mapping.motor_to_model_rad(m)) == pytest.approx(m)


@pytest.mark.parametrize("m", [VERTICAL_MOTOR_DEG, (0, 0, 0, 0, 0, 0), (340, 180, 163, 335, 220, 335), (12.5, 33, 140, 7, 200, 300)])
def test_ros_model_matches_official_node_formula(m):
    # literal copy of publish_joint_states() in the official episode_controller (evidence B)
    offsets = [180, 90, 83, 30, 110, 30]
    official = [math.radians(m[i] - offsets[i]) for i in range(6)]
    assert ROS_MODEL.motor_to_model_rad(m) == pytest.approx(official)


def test_ros_model_limits_equal_official_urdf_limits():
    from episode_hardware.joint_mapping import OFFICIAL_URDF_LIMITS_DEG
    assert ROS_MODEL.model_limits_deg() == pytest.approx([tuple(p) for p in OFFICIAL_URDF_LIMITS_DEG])


def test_vertical_posture_is_ros_zero():
    assert ROS_MODEL.motor_to_model_deg(VERTICAL_MOTOR_DEG) == pytest.approx([0] * 6)


def test_none_passthrough():
    out = VENDOR_DH.motor_to_model_deg([180, None, 83, 30, 110, 30])
    assert out[1] is None and out[0] == 0


def test_velocity_mapping_ignores_offset():
    assert VENDOR_DH.model_velocity_from_motor([1, 1, 1, 1, 1, 1]) == [1, -1, -1, 1, -1, 1]


def test_model_limits_sorted():
    for lo, hi in VENDOR_DH.model_limits_deg():
        assert lo < hi


def test_mapping_matches_current_official_dh_table():
    from episode_hardware.joint_mapping import OFFICIAL_DH_THETA_RANGE_DEG
    assert check_dh_qlim_consistency(VENDOR_DH, OFFICIAL_DH_THETA_RANGE_DEG) == []


def test_qlim_inconsistency_found():
    diffs = check_dh_qlim_consistency(VENDOR_DH, VENDOR_DH_QLIM_DEG)
    assert [d["joint"] for d in diffs] == [1, 4, 6]


def test_from_dict_and_bad_input():
    m = JointMapping.from_dict({"name": "x", "signs": [1] * 6, "offsets_deg": [0] * 6})
    assert m.motor_to_model_deg([1] * 6) == [1] * 6
    with pytest.raises(ValueError):
        JointMapping("bad", (1, 1, 1, 1, 1, 2), (0,) * 6)
    with pytest.raises(ValueError):
        VENDOR_DH.motor_to_model_deg([1, 2, 3])
    with pytest.raises(KeyError):
        get_profile("nope")


def test_fk_vertical_posture_geometry():
    """Synthetic check of the public DH table: upper arm vertical, forearm horizontal."""
    t = kinematics.fk_motor_deg(VERTICAL_MOTOR_DEG)
    assert t[0][3] == pytest.approx(55 + 56 + 192 - 1, abs=2)  # ~302 mm forward
    assert t[2][3] == pytest.approx(166 + 200 + 56, abs=2)  # ~422 mm up


def test_ik_recovers_fk_pose():
    q = [200, 100, 70, 60, 120, 50]
    t = kinematics.fk_motor_deg(q)
    b = math.degrees(math.asin(-t[2][0]))
    a = math.degrees(math.atan2(t[1][0], t[0][0]))
    c = math.degrees(math.atan2(t[2][1], t[2][2]))
    sol = kinematics.ik_motor_deg([t[0][3], t[1][3], t[2][3]], [a, b, c], "zyx", VERTICAL_MOTOR_DEG)
    assert sol is not None
    t2 = kinematics.fk_motor_deg(sol)
    for i in range(3):
        assert t2[i][3] == pytest.approx(t[i][3], abs=1.0)


def test_ik_matches_orientation_not_only_position():
    """Regression: the cross-product orientation error is blind to 180 deg flips."""
    sol = kinematics.ik_motor_deg([250, 50, 380], [90, 0, 180], "zyx", VERTICAL_MOTOR_DEG)
    assert sol is not None
    t = kinematics.fk_motor_deg(sol)
    r = kinematics.euler_to_matrix([90, 0, 180], "zyx")
    for i in range(3):
        assert t[i][3] == pytest.approx([250, 50, 380][i], abs=1.0)
        for j in range(3):
            assert t[i][j] == pytest.approx(r[i][j], abs=1e-2)


def test_rotation_error_detects_180_deg():
    ident = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    flip = kinematics.euler_to_matrix([0, 0, 180], "zyx")
    w = kinematics.rotation_error(ident, flip)
    assert math.sqrt(sum(v * v for v in w)) == pytest.approx(math.pi, abs=1e-3)


def test_ik_unreachable_returns_none():
    assert kinematics.ik_motor_deg([2000, 0, 0], [90, 0, 180], "zyx", VERTICAL_MOTOR_DEG) is None
