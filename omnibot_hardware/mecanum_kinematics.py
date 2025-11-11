# omnibot_hardware/mecanum_kinematics.py
"""
Mecanum wheel kinematics helper for the ESP32-based OmniBot.

Provides forward and inverse kinematics for a four-wheeled mecanum platform
following ROS conventions:
 - x-axis: forward
 - y-axis: left
 - z-axis: upward (CCW rotation positive)

Wheel and protocol order:
    RR (rear right)
    FR (front right)
    RL (rear left)
    FL (front left)

This order matches the ESP32 controller protocol:
    TX: M w_rr w_fr w_rl w_fl
    RX: E seq time t_rr t_fr t_rl t_fl
"""

import math


class MecanumKinematics:
    """
    Encapsulates the forward and inverse kinematics for a mecanum drive.

    Wheel order convention (ESP32 protocol):
        1 = Rear Right (RR)
        2 = Front Right (FR)
        3 = Rear Left  (RL)
        4 = Front Left (FL)

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
    # Inverse kinematics: (vx, vy, wz) → (ω_RR, ω_FR, ω_RL, ω_FL)
    # --------------------------------------------------------------
    def inverse(self, vx: float, vy: float, wz: float):
        """
        Compute individual wheel angular velocities (rad/s)
        from body velocities.

        Returns:
            tuple: (w_rr, w_fr, w_rl, w_fl)
        """
        R, L, s = self.R, self.L, self.s

        w_rr = (1.0 / R) * (vx - s * vy + L * wz)  # Rear Right
        w_fr = (1.0 / R) * (vx + s * vy + L * wz)  # Front Right
        w_rl = (1.0 / R) * (vx + s * vy - L * wz)  # Rear Left
        w_fl = (1.0 / R) * (vx - s * vy - L * wz)  # Front Left

        return w_rr, w_fr, w_rl, w_fl

    # --------------------------------------------------------------
    # Forward kinematics: (ω_RR, ω_FR, ω_RL, ω_FL) → (vx, vy, wz)
    # --------------------------------------------------------------
    def forward(self, w_rr: float, w_fr: float, w_rl: float, w_fl: float):
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

