# omnibot_hardware/serial_interface.py
"""
Serial interface for communicating with the Arduino-based motor controller.

This module handles:
 - Opening and maintaining a serial connection
 - Sending formatted motor speed commands
 - Receiving and parsing encoder data frames
 - Basic fault tolerance and logging hooks
"""

import serial
import threading
import time
from typing import Optional, List, Tuple


class SerialBridge:
    """
    Bidirectional interface to the Arduino controller.

    Expected protocol:
        - TX (to Arduino): "M w_fl w_rl w_rr w_fr\n"
        - RX (from Arduino): "E seq timestamp_us t_fl t_rl t_rr t_fr\n"
    """

    def __init__(self, port: str = "/dev/ttyUSB0", baud: int = 115200, timeout: float = 0.0):
        """
        Args:
            port: Serial device (e.g., /dev/ttyUSB0)
            baud: Baud rate (default 115200)
            timeout: Serial read timeout in seconds (0 = non-blocking)
        """
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self._rxbuf = bytearray()
        self._lock = threading.Lock()

        # Open connection
        try:
            self.ser = serial.Serial(port, baud, timeout=timeout)
            time.sleep(2.0)  # Allow Arduino reset on connect
            print(f"Connected to Arduino on {port} @ {baud} baud")
        except serial.SerialException as e:
            print(f"Serial connection failed: {e}")
            self.ser = None

    # ------------------------------------------------------------------
    def is_connected(self) -> bool:
        """Check if serial connection is valid."""
        return self.ser is not None and self.ser.is_open

    # ------------------------------------------------------------------
    def send_motor_speeds(self, w_fl: float, w_rl: float, w_rr: float, w_fr: float):
        """
        Send motor angular velocities to Arduino.

        Args:
            w_fl, w_rl, w_rr, w_fr: wheel angular velocities [rad/s]
        """
        if not self.is_connected():
            return

        cmd = f"M {w_fl:.2f} {w_rl:.2f} {w_rr:.2f} {w_fr:.2f}\n"
        try:
            with self._lock:
                self.ser.write(cmd.encode("utf-8"))
        except serial.SerialException as e:
            print(f"Serial write error: {e}")

    # ------------------------------------------------------------------
    def read_lines(self) -> List[str]:
        """
        Read all complete lines currently in the serial buffer.

        Returns:
            list of str: each line without newline characters
        """
        if not self.is_connected():
            return []

        lines = []
        try:
            n = self.ser.in_waiting
            if n > 0:
                chunk = self.ser.read(n)
                self._rxbuf.extend(chunk)
        except serial.SerialException as e:
            print(f"Serial read error: {e}")
            return []

        while True:
            idx = self._rxbuf.find(b"\n")
            if idx < 0:
                break
            line = self._rxbuf[:idx].decode("utf-8", errors="ignore").strip()
            self._rxbuf = self._rxbuf[idx + 1 :]
            if line:
                lines.append(line)
        return lines

    # ------------------------------------------------------------------
    def parse_encoder_line(self, line: str) -> Optional[Tuple[int, int, int, int, int, int]]:
        """
        Parse an encoder feedback line of the form:
            "E seq timestamp_us t_fl t_rl t_rr t_fr"

        Returns:
            (seq, timestamp_us, t_fl, t_rl, t_rr, t_fr)
            or None if invalid.
        """
        if not line.startswith("E"):
            return None

        parts = line.split()
        if len(parts) != 7:
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
            return None

    # ------------------------------------------------------------------
    def close(self):
        """Close the serial connection."""
        if self.is_connected():
            self.ser.close()
            print("Serial connection closed.")
