# edubot_hardware/simulation_interface.py
"""
Simulation interface for the ESP32-based EduBot.

Emulates the SerialBridge communication protocol used by the real ESP32
motor controller.

Protocol format:
    TX → ESP32 : "M w_rr w_fr w_rl w_fl"      (wheel angular velocities [rad/s])
    RX ← ESP32 : "E seq timestamp_us t_rr t_fr t_rl t_fl"  (cumulative encoder ticks)

This simulator reproduces the same wheel order, encoder direction,
and timing characteristics as the physical hardware, allowing the
OdometryEstimator and HardwareNode to behave identically in simulation.
"""

import math
import time
from typing import List, Tuple, Optional


class SimulationInterface:
    """
    Drop-in replacement for SerialBridge that simulates motor control
    and encoder feedback following the ESP32 protocol.

    Emits lines of the form:
        "E seq timestamp_us t_rr t_fr t_rl t_fl"
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
            wheel_radius: wheel radius [m]
            logger: optional rclpy logger
        """
        self.ticks_per_rev = float(ticks_per_rev)
        self.wheel_radius = float(wheel_radius)
        self.logger = logger

        # current simulated wheel speeds [rad/s]
        self._w_rr = 0.0
        self._w_fr = 0.0
        self._w_rl = 0.0
        self._w_fl = 0.0

        # cumulative encoder ticks (float for sub-tick accumulation)
        self._t_rr_f = 0.0
        self._t_fr_f = 0.0
        self._t_rl_f = 0.0
        self._t_fl_f = 0.0

        # encoder polarity (matches physical hardware)
        # all positive means forward rotation increases tick count
        self._enc_sign = (+1.0, +1.0, +1.0, +1.0)  # RR, FR, RL, FL

        self._seq = 0
        self._last_update = time.time()

        self._log_info("SimulationInterface initialized (ESP32 mode)")

    # ------------------------------------------------------------------
    def is_connected(self) -> bool:
        """Always True in simulation."""
        return True

    # ------------------------------------------------------------------
    def send_motor_speeds(self, w_rr: float, w_fr: float, w_rl: float, w_fl: float):
        """Store target wheel angular velocities [rad/s] for simulation."""
        self._w_rr = float(w_rr)
        self._w_fr = float(w_fr)
        self._w_rl = float(w_rl)
        self._w_fl = float(w_fl)

        self._log_debug(
            f"Motor speeds set (rad/s): RR={self._w_rr:.2f}, FR={self._w_fr:.2f}, "
            f"RL={self._w_rl:.2f}, FL={self._w_fl:.2f}"
        )

    # ------------------------------------------------------------------
    def read_lines(self) -> List[str]:
        """
        Simulate encoder feedback lines with timestamps.

        Returns:
            A list with zero or one line:
            "E seq timestamp_us t_rr t_fr t_rl t_fl"
        """
        now = time.time()
        dt = now - self._last_update
        if dt < 0.02:  # ~50 Hz output rate
            return []

        self._last_update = now
        timestamp_us = int(now * 1e6)
        two_pi = 2.0 * math.pi

        # Δticks = (ω * dt / 2π) * ticks_per_rev * sign
        d_rr = (self._w_rr * dt / two_pi) * self.ticks_per_rev * self._enc_sign[0]
        d_fr = (self._w_fr * dt / two_pi) * self.ticks_per_rev * self._enc_sign[1]
        d_rl = (self._w_rl * dt / two_pi) * self.ticks_per_rev * self._enc_sign[2]
        d_fl = (self._w_fl * dt / two_pi) * self.ticks_per_rev * self._enc_sign[3]

        self._t_rr_f += d_rr
        self._t_fr_f += d_fr
        self._t_rl_f += d_rl
        self._t_fl_f += d_fl
        self._seq += 1

        # integer rounding only when emitting (reduces quantization noise)
        t_rr = int(round(self._t_rr_f))
        t_fr = int(round(self._t_fr_f))
        t_rl = int(round(self._t_rl_f))
        t_fl = int(round(self._t_fl_f))

        line = f"E {self._seq} {timestamp_us} {t_rr} {t_fr} {t_rl} {t_fl}"

        self._log_debug(
            f"Simulated encoder line: seq={self._seq}, ts={timestamp_us}, "
            f"ticks=[{t_rr}, {t_fr}, {t_rl}, {t_fl}]"
        )

        return [line]

    # ------------------------------------------------------------------
    def parse_encoder_line(self, line: str) -> Optional[Tuple[int, int, int, int, int, int]]:
        """
        Parse a simulated encoder line.

        Returns:
            (seq, timestamp_us, t_rr, t_fr, t_rl, t_fl)
        """
        parts = line.split()
        if len(parts) != 7 or parts[0] != "E":
            return None
        try:
            seq = int(parts[1])
            ts_us = int(parts[2])
            t_rr = int(parts[3])
            t_fr = int(parts[4])
            t_rl = int(parts[5])
            t_fl = int(parts[6])
            return seq, ts_us, t_rr, t_fr, t_rl, t_fl
        except ValueError:
            self._log_warn(f"Failed to parse encoder line: {line}")
            return None

    # ------------------------------------------------------------------
    def close(self):
        """Stop the simulation (no-op)."""
        self._log_info("SimulationInterface closed.")

    # ------------------------------------------------------------------
    # Logging helpers
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
        # no stdout debug fallback

