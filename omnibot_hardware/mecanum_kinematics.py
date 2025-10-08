# omnibot_hardware/mecanum_kinematics.py
"""
Mecanum wheel kinematics helper.

Provides forward and inverse kinematics for a four-wheeled mecanum platform
following ROS conventions:
 - x-axis: forward
 - y-axis: left
 - z-axis: upward (CCW rotation positive)
"""

import math


class MecanumKinematics:
    """
    Encapsulates the forward and inverse kinematics for a mecanum drive.

    Wheel order convention:
        1 = Front Left (FL)
        2 = Rear Left  (RL)
        3 = Rear Right (RR)
        4 = Front Right (FR)

    Layout can be 'X' (rollers form an X-shape) or 'O' (rollers form an O-shape).
    """

    def __init__(self, R: float, Lx: float, Ly: float, layout: str = "X"):
        """
        Args:
            R: wheel radius [m]
            Lx: half the robot length [m]
            Ly: half the robot width [m]
            layout: 'X' or 'O' configuration
        """
        self.R = R
        self.Lx = Lx
        self.Ly = Ly
        self.layout = layout.upper()
        self.s = 1.0 if self.layout == "X" else -1.0
        self.L = self.Lx + self.Ly

    # --------------------------------------------------------------
    # Inverse kinematics: (vx, vy, wz) → (ω_FL, ω_RL, ω_RR, ω_FR)
    # --------------------------------------------------------------
    def inverse(self, vx: float, vy: float, wz: float):
        """
        Compute individual wheel angular velocities (rad/s)
        from body velocities.

        Returns:
            tuple: (w_fl, w_rl, w_rr, w_fr)
        """
        R, L, s = self.R, self.L, self.s

        w_fl = (1.0 / R) * (vx - s * vy - L * wz)  # Front Left
        w_fr = (1.0 / R) * (vx + s * vy + L * wz)  # Front Right
        w_rl = (1.0 / R) * (vx + s * vy - L * wz)  # Rear Left
        w_rr = (1.0 / R) * (vx - s * vy + L * wz)  # Rear Right

        return w_fl, w_rl, w_rr, w_fr

    # --------------------------------------------------------------
    # Forward kinematics: (ω_FL, ω_RL, ω_RR, ω_FR) → (vx, vy, wz)
    # --------------------------------------------------------------
    def forward(self, w_fl: float, w_rl: float, w_rr: float, w_fr: float):
        """
        Compute the body velocity (vx, vy, wz) from wheel angular velocities.

        Returns:
            tuple: (vx, vy, wz)
        """
        R, L, s = self.R, self.L, self.s

        vx = (R / 4.0) * (w_fl + w_fr + w_rl + w_rr)
        vy = (R / 4.0) * (-s * w_fl + s * w_fr + s * w_rl - s * w_rr)
        wz = (R / (4.0 * L)) * (-w_fl + w_fr - w_rl + w_rr)

        return vx, vy, wz

    # --------------------------------------------------------------
    def __repr__(self):
        return (f"<MecanumKinematics R={self.R:.3f} Lx={self.Lx:.3f} Ly={self.Ly:.3f} "
                f"layout='{self.layout}'>")
