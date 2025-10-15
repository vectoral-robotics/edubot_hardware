# omnibot_hardware/simulation_interface.py
"""
Simulation interface for a virtual robot backend.

Matches the raw encoder polarity of the real hardware so that the
OdometryEstimator's correction (RR & FR inverted) yields identical behavior.
Also integrates ticks in floating point and rounds on output to avoid
quantization artifacts at low speeds.
"""

import math
import time
from typing import List, Tuple, Optional


class SimulationInterface:
    """
    Drop-in replacement for SerialBridge that simulates motor control and encoder feedback.
    Emits lines of the form: "E seq timestamp_us t_fl t_rl t_rr t_fr"
    """

    def __init__(
        self,
        ticks_per_rev: int = 4320,
        wheel_radius: float = 0.04,
        logger=None
    ):
        """
        Args:
            ticks_per_rev: simulated encoder resolution [ticks/rev]
            wheel_radius: wheel radius [m] (not strictly needed here)
            logger: optional rclpy logger (passed from HardwareNode)
        """
        self.ticks_per_rev = float(ticks_per_rev)
        self.wheel_radius = float(wheel_radius)
        self.logger = logger

        # current simulated wheel speeds [rad/s]
        self._w_fl = 0.0
        self._w_rl = 0.0
        self._w_rr = 0.0
        self._w_fr = 0.0

        # cumulative encoder ticks (float for sub-tick accumulation)
        self._t_fl_f = 0.0
        self._t_rl_f = 0.0
        self._t_rr_f = 0.0
        self._t_fr_f = 0.0

        # MATCH RAW HARDWARE POLARITY (so odometry can correct RR/FR)
        # FL=+1, RL=+1, RR=-1, FR=-1
        self._enc_sign = (+1.0, +1.0, -1.0, -1.0)

        self._seq = 0
        self._last_update = time.time()

        self._log_info("SimulationInterface initialized (no real hardware)")

    # ------------------------------------------------------------------
    def is_connected(self) -> bool:
        """Always True for simulation."""
        return True

    # ------------------------------------------------------------------
    def send_motor_speeds(self, w_fl: float, w_rl: float, w_rr: float, w_fr: float):
        """Store target wheel angular velocities for simulation [rad/s]."""
        self._w_fl = float(w_fl)
        self._w_rl = float(w_rl)
        self._w_rr = float(w_rr)
        self._w_fr = float(w_fr)

        self._log_debug(
            f"Motor speeds set (rad/s): FL={self._w_fl:.2f}, RL={self._w_rl:.2f}, "
            f"RR={self._w_rr:.2f}, FR={self._w_fr:.2f}"
        )

    # ------------------------------------------------------------------
    def read_lines(self) -> List[str]:
        """
        Simulate encoder feedback lines with timestamps.

        Returns:
            a list with zero or one line:
            "E seq timestamp_us t_fl t_rl t_rr t_fr"
        """
        now = time.time()
        dt = now - self._last_update
        # simulate ~50 Hz output but dynamically handle timing drift
        if dt < 0.02:
            return []

        self._last_update = now
        timestamp_us = int(now * 1e6)  # simulated microsecond timestamp

        # integrate encoder ticks based on wheel speeds
        # Δticks = (ω [rad/s] * dt / 2π) * ticks_per_rev * encoder_sign
        two_pi = 2.0 * math.pi
        d_fl = (self._w_fl * dt / two_pi) * self.ticks_per_rev * self._enc_sign[0]
        d_rl = (self._w_rl * dt / two_pi) * self.ticks_per_rev * self._enc_sign[1]
        d_rr = (self._w_rr * dt / two_pi) * self.ticks_per_rev * self._enc_sign[2]
        d_fr = (self._w_fr * dt / two_pi) * self.ticks_per_rev * self._enc_sign[3]

        self._t_fl_f += d_fl
        self._t_rl_f += d_rl
        self._t_rr_f += d_rr
        self._t_fr_f += d_fr

        self._seq += 1

        # Round only when emitting (reduces quantization artifacts)
        t_fl = int(round(self._t_fl_f))
        t_rl = int(round(self._t_rl_f))
        t_rr = int(round(self._t_rr_f))
        t_fr = int(round(self._t_fr_f))

        line = f"E {self._seq} {timestamp_us} {t_fl} {t_rl} {t_rr} {t_fr}"

        self._log_debug(
            f"Simulated encoder line: seq={self._seq}, ts={timestamp_us}, "
            f"ticks=[{t_fl}, {t_rl}, {t_rr}, {t_fr}]"
        )

        return [line]

    # ------------------------------------------------------------------
    def parse_encoder_line(self, line: str) -> Optional[Tuple[int, int, int, int, int, int]]:
        """
        Parse a simulated encoder line.
        Returns: (seq, timestamp_us, t_fl, t_rl, t_rr, t_fr)
        """
        parts = line.split()
        if len(parts) != 7 or parts[0] != "E":
            return None
        try:
            seq = int(parts[1])
            ts_us = int(parts[2])
            t_fl = int(parts[3])
            t_rl = int(parts[4])
            t_rr = int(parts[5])
            t_fr = int(parts[6])
            return seq, ts_us, t_fl, t_rl, t_rr, t_fr
        except ValueError:
            self._log_warn(f"Failed to parse encoder line: {line}")
            return None

    # ------------------------------------------------------------------
    def close(self):
        """Stop simulation (noop)."""
        self._log_info("SimulationInterface closed.")

    # ------------------------------------------------------------------
    # Internal logging helpers
    # ------------------------------------------------------------------
    def _log_info(self, msg: str):
        if self.logger:
            self.logger.info(msg)
        else:
            print(msg)

    def _log_warn(self, msg: str):
        if self.logger:
            self.logger.warn(msg)
        else:
            print(f"WARNING: {msg}")

    def _log_debug(self, msg: str):
        if self.logger:
            self.logger.debug(msg)
        # don't print debug messages without a logger
