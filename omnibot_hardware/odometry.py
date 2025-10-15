# omnibot_hardware/odometry.py
"""
Odometry computation for a mecanum-driven robot.

This module estimates robot pose (x, y, yaw) and body velocities (vx, vy, wz)
based on wheel encoder tick counts, and integrates wheel angles for
JointState publishing.
"""

import math
from typing import Optional, Tuple, List


class OdometryEstimator:
    """
    Integrates encoder readings into position and orientation estimates.

    Handles:
      - tick-to-angle conversion
      - pose integration in world frame
      - wheel angle integration
      - velocity estimation (vx, vy, wz)
    """

    def __init__(self,
                 wheel_radius: float,
                 base_length: float,
                 base_width: float,
                 ticks_per_rev: float,
                 mecanum_layout: str = "X"):
        """
        Args:
            wheel_radius: wheel radius [m]
            base_length: half of robot length [m]
            base_width: half of robot width [m]
            ticks_per_rev: encoder ticks per revolution
            mecanum_layout: 'X' or 'O'
        """
        self.R = wheel_radius
        self.Lx = base_length
        self.Ly = base_width
        self.L = self.Lx + self.Ly
        self.ticks_per_rev = ticks_per_rev
        self.s = 1.0 if mecanum_layout.upper() == "X" else -1.0

        # Pose state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Last encoder readings (absolute ticks)
        self._last_ticks = None  # (t_fl, t_rl, t_rr, t_fr)

        # Integrated wheel angles [rad] for joint state publishing
        self._wheel_angles = [0.0, 0.0, 0.0, 0.0]

    # ------------------------------------------------------------------
    def update(self,
               ticks: Tuple[int, int, int, int],
               dt: float) -> Optional[Tuple[float, float, float]]:
        """
        Update odometry from new encoder tick readings.

        Args:
            ticks: tuple (t_fl, t_rl, t_rr, t_fr)
            dt: time delta since last encoder update [s]

        Returns:
            (vx, vy, wz): body-frame velocities [m/s, m/s, rad/s]
            or None if this is the first update.
        """
        if self._last_ticks is None:
            self._last_ticks = ticks
            return None

        if dt <= 0.0:
            return None

        t_fl, t_rl, t_rr, t_fr = ticks
        l_fl, l_rl, l_rr, l_fr = self._last_ticks
        self._last_ticks = ticks

        # Compute tick deltas
        d_fl = t_fl - l_fl
        d_rl = t_rl - l_rl
        d_rr = t_rr - l_rr
        d_fr = t_fr - l_fr

        # Correct encoder polarity (measured for this robot)
        d_rr *= -1
        d_fr *= -1

        # Convert ticks → wheel angular velocity [rad/s]
        two_pi = 2.0 * math.pi
        w_fl = (d_fl / self.ticks_per_rev) * two_pi / dt
        w_rl = (d_rl / self.ticks_per_rev) * two_pi / dt
        w_rr = (d_rr / self.ticks_per_rev) * two_pi / dt
        w_fr = (d_fr / self.ticks_per_rev) * two_pi / dt

        # Forward kinematics (body-frame)
        vx = (self.R / 4.0) * (w_fl + w_fr + w_rl + w_rr)
        vy = (self.R / 4.0) * (-self.s * w_fl + self.s * w_fr + self.s * w_rl - self.s * w_rr)
        wz = (self.R / (4.0 * self.L)) * (-w_fl + w_fr - w_rl + w_rr)

        # Integrate pose in world frame
        cos_y = math.cos(self.yaw)
        sin_y = math.sin(self.yaw)
        self.x += vx * dt * cos_y - vy * dt * sin_y
        self.y += vx * dt * sin_y + vy * dt * cos_y
        self.yaw += wz * dt

        # Normalize yaw to [-pi, pi]
        self.yaw = (self.yaw + math.pi) % (2.0 * math.pi) - math.pi

        # Integrate wheel angles for JointState
        self._wheel_angles[0] += (d_fl / self.ticks_per_rev) * two_pi
        self._wheel_angles[1] += (d_rl / self.ticks_per_rev) * two_pi
        self._wheel_angles[2] += (d_rr / self.ticks_per_rev) * two_pi
        self._wheel_angles[3] += (d_fr / self.ticks_per_rev) * two_pi

        return vx, vy, wz

    # ------------------------------------------------------------------
    def get_pose(self) -> Tuple[float, float, float]:
        """Return current pose (x, y, yaw) in world frame."""
        return self.x, self.y, self.yaw

    # ------------------------------------------------------------------
    def get_wheel_angles(self) -> List[float]:
        """Return integrated wheel angles [rad] for joint state publishing."""
        return self._wheel_angles

    # ------------------------------------------------------------------
    def reset(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0):
        """Reset pose, tick baselines, and wheel angles."""
        self.x, self.y, self.yaw = x, y, yaw
        self._last_ticks = None
        self._wheel_angles = [0.0, 0.0, 0.0, 0.0]

    # ------------------------------------------------------------------
    def __repr__(self):
        return (f"<OdometryEstimator x={self.x:.3f} y={self.y:.3f} "
                f"yaw={math.degrees(self.yaw):.1f}°>")
