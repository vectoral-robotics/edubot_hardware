"""Unit tests for the odometry estimator (pure math, no ROS)."""

import math

import pytest

from edubot_hardware.odometry import OdometryEstimator

R = 0.04
LX = 0.087
LY = 0.1154
TICKS_PER_REV = 4320.0
DT = 0.1


@pytest.fixture
def odom() -> OdometryEstimator:
    return OdometryEstimator(R, LX, LY, TICKS_PER_REV, mecanum_layout="X")


def test_first_update_returns_none_and_sets_baseline(odom: OdometryEstimator):
    assert odom.update((0, 0, 0, 0), DT) is None
    assert odom.get_pose() == (0.0, 0.0, 0.0)


def test_non_positive_dt_is_ignored(odom: OdometryEstimator):
    odom.update((0, 0, 0, 0), DT)  # baseline
    assert odom.update((100, 100, 100, 100), 0.0) is None


def test_one_full_revolution_forward_moves_one_circumference(odom: OdometryEstimator):
    """Equal ticks on all wheels = pure forward motion."""
    odom.update((0, 0, 0, 0), DT)  # baseline
    d = int(TICKS_PER_REV)  # exactly one wheel revolution
    vx, vy, wz = odom.update((d, d, d, d), DT)

    circumference = 2.0 * math.pi * R
    x, y, yaw = odom.get_pose()
    assert x == pytest.approx(circumference, rel=1e-6)
    assert y == pytest.approx(0.0, abs=1e-9)
    assert yaw == pytest.approx(0.0, abs=1e-9)
    assert vx == pytest.approx(circumference / DT, rel=1e-6)
    assert vy == pytest.approx(0.0, abs=1e-9)
    assert wz == pytest.approx(0.0, abs=1e-9)


def test_wheel_angles_integrate_to_two_pi_after_one_rev(odom: OdometryEstimator):
    odom.update((0, 0, 0, 0), DT)
    d = int(TICKS_PER_REV)
    odom.update((d, d, d, d), DT)
    assert odom.get_wheel_angles() == pytest.approx([2 * math.pi] * 4)


def test_pure_rotation_changes_yaw_only(odom: OdometryEstimator):
    """RR/FR forward, RL/FL backward = rotation in place."""
    odom.update((0, 0, 0, 0), DT)
    d = 500
    _, _, wz = odom.update((d, d, -d, -d), DT)

    x, y, yaw = odom.get_pose()
    assert x == pytest.approx(0.0, abs=1e-9)
    assert y == pytest.approx(0.0, abs=1e-9)
    assert yaw != pytest.approx(0.0)
    assert wz != pytest.approx(0.0)


def test_yaw_is_wrapped_to_pi_range(odom: OdometryEstimator):
    odom.update((0, 0, 0, 0), DT)
    # Drive a large rotation across several steps; yaw must stay in [-pi, pi].
    d = 2000
    ticks = 0
    for _ in range(20):
        ticks += d
        odom.update((ticks, ticks, -ticks, -ticks), DT)
        _, _, yaw = odom.get_pose()
        assert -math.pi <= yaw <= math.pi


def test_reset_clears_pose_and_baseline(odom: OdometryEstimator):
    odom.update((0, 0, 0, 0), DT)
    odom.update((100, 100, 100, 100), DT)
    odom.reset(x=1.0, y=2.0, yaw=0.5)

    assert odom.get_pose() == (1.0, 2.0, 0.5)
    assert odom.get_wheel_angles() == [0.0, 0.0, 0.0, 0.0]
    # After reset the next update is treated as a fresh baseline.
    assert odom.update((100, 100, 100, 100), DT) is None
