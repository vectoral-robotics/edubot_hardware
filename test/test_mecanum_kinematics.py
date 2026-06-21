"""Unit tests for the mecanum kinematics (pure math, no ROS)."""

import math

import pytest

from edubot_hardware.mecanum_kinematics import MecanumKinematics

# Realistic EduBot-ish geometry.
R = 0.04
LX = 0.087
LY = 0.1154


@pytest.fixture
def kin_x() -> MecanumKinematics:
    return MecanumKinematics(R, LX, LY, layout="X")


def test_inverse_pure_forward_drives_all_wheels_equally(kin_x: MecanumKinematics):
    w_rr, w_fr, w_rl, w_fl = kin_x.inverse(0.5, 0.0, 0.0)
    expected = 0.5 / R
    assert w_rr == pytest.approx(expected)
    assert w_fr == pytest.approx(expected)
    assert w_rl == pytest.approx(expected)
    assert w_fl == pytest.approx(expected)


def test_inverse_pure_strafe_left_x_layout(kin_x: MecanumKinematics):
    # vy>0 (left): X-layout (s=+1) gives the classic +-+- wheel pattern.
    w_rr, w_fr, w_rl, w_fl = kin_x.inverse(0.0, 0.3, 0.0)
    mag = 0.3 / R
    assert w_rr == pytest.approx(-mag)
    assert w_fr == pytest.approx(+mag)
    assert w_rl == pytest.approx(+mag)
    assert w_fl == pytest.approx(-mag)


def test_inverse_pure_rotation(kin_x: MecanumKinematics):
    w_rr, w_fr, w_rl, w_fl = kin_x.inverse(0.0, 0.0, 1.0)
    mag = (LX + LY) / R
    assert w_rr == pytest.approx(+mag)
    assert w_fr == pytest.approx(+mag)
    assert w_rl == pytest.approx(-mag)
    assert w_fl == pytest.approx(-mag)


@pytest.mark.parametrize(
    ("vx", "vy", "wz"),
    [
        (0.5, 0.0, 0.0),
        (0.0, 0.3, 0.0),
        (0.0, 0.0, 1.2),
        (0.4, -0.2, 0.6),
        (-0.3, 0.15, -0.9),
    ],
)
def test_forward_is_inverse_of_inverse(kin_x: MecanumKinematics, vx, vy, wz):
    """forward(inverse(v)) must recover the original body velocity."""
    wheels = kin_x.inverse(vx, vy, wz)
    out_vx, out_vy, out_wz = kin_x.forward(*wheels)
    assert out_vx == pytest.approx(vx, abs=1e-9)
    assert out_vy == pytest.approx(vy, abs=1e-9)
    assert out_wz == pytest.approx(wz, abs=1e-9)


def test_o_layout_flips_strafe_sign():
    kin_o = MecanumKinematics(R, LX, LY, layout="O")
    _, w_fr_o, _, _ = kin_o.inverse(0.0, 0.3, 0.0)
    kin_x = MecanumKinematics(R, LX, LY, layout="X")
    _, w_fr_x, _, _ = kin_x.inverse(0.0, 0.3, 0.0)
    assert w_fr_o == pytest.approx(-w_fr_x)


def test_layout_is_case_insensitive():
    assert MecanumKinematics(R, LX, LY, layout="x").s == 1.0
    assert MecanumKinematics(R, LX, LY, layout="o").s == -1.0


def test_repr_mentions_layout(kin_x: MecanumKinematics):
    assert "layout='X'" in repr(kin_x)
    assert math.isclose(kin_x.L, LX + LY)
