# edubot_hardware/imu_node.py
"""
ROS 2 node for the BNO085 IMU connected to the Raspberry Pi 5 over I2C.

Publishes sensor_msgs/Imu on ``imu/data`` at a configurable rate.
The message contains:
  - orientation      (quaternion, from BNO085 rotation-vector report)
  - angular_velocity (rad/s,      from gyroscope report)
  - linear_acceleration (m/s²,   from linear-acceleration report, gravity removed)

Intended use: feed directly into robot_localization EKF alongside /odom to
produce /odometry/filtered for Nav2.

Wiring (RPi 5):
  BNO085 VIN  → Pin 1  (3.3V)
  BNO085 GND  → Pin 6  (GND)
  BNO085 SDA  → Pin 3  (GPIO2, I2C SDA)
  BNO085 SCL  → Pin 5  (GPIO3, I2C SCL)
  (PS0, PS1 float / GND — defaults to I2C address 0x4B if ADDR pin floats high, 0x4A if tied to GND)

Enable I2C before first use:
  /boot/firmware/config.txt: dtparam=i2c_arm=on  (then reboot)
  verify:  i2cdetect -y 1   → 0x4A should appear

Python dependency (host, not Docker):
  pip3 install --break-system-packages adafruit-circuitpython-bno08x

Parameters (all optional):
  ~i2c_address     int   0x4A  I2C address (0x4A or 0x4B)
  ~frame_id        str   imu_link   frame_id in the published header
  ~publish_hz      float  100.0  target publish rate [Hz]
  ~use_sim         bool  false  if true → publish zeroed messages (no hardware)
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu


# Covariance diagonals for the BNO085 (row-major 3×3, zeros off-diagonal).
# Orientation: ~1° = 0.017 rad → variance ~3e-4 rad²
# Gyroscope:   ~0.005 rad/s noise → variance ~2.5e-5 (rad/s)²
# Linear accel: ~0.05 m/s² noise → variance ~2.5e-3 (m/s²)²
_ORIENT_COV_DIAG = [3e-4, 3e-4, 3e-4]
_GYRO_COV_DIAG = [2.5e-5, 2.5e-5, 2.5e-5]
_ACCEL_COV_DIAG = [2.5e-3, 2.5e-3, 2.5e-3]


def _diag_cov(values: list[float]) -> list[float]:
    """Return a flat 9-element covariance matrix with values on the diagonal."""
    m = [0.0] * 9
    for i, v in enumerate(values):
        m[i * 3 + i] = v
    return m


class ImuNode(Node):
    """ROS 2 node publishing sensor_msgs/Imu from a BNO085 over I2C."""

    def __init__(self):
        super().__init__("imu_node")
        self.get_logger().info("EduBot IMU Node starting up...")

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter("use_sim", False)
        self.declare_parameter("i2c_address", 0x4B)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("publish_hz", 100.0)

        use_sim = bool(self.get_parameter("use_sim").value)
        self._i2c_address = int(self.get_parameter("i2c_address").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        publish_hz = float(self.get_parameter("publish_hz").value)

        # ------------------------------------------------------------------
        # BNO085 driver (real or null)
        # ------------------------------------------------------------------
        self._bno = None
        if use_sim:
            self.get_logger().info("IMU Node: simulation mode — publishing zeroed messages.")
        else:
            self._bno = self._init_bno()

        # ------------------------------------------------------------------
        # Publisher
        # ------------------------------------------------------------------
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._pub = self.create_publisher(Imu, "imu/data", qos)

        # ------------------------------------------------------------------
        # Timer
        # ------------------------------------------------------------------
        self.create_timer(1.0 / publish_hz, self._timer_cb)

        self.get_logger().info(
            f"IMU Node ready: publishing on imu/data @ {publish_hz:.0f} Hz "
            f"(frame_id={self._frame_id}, addr=0x{self._i2c_address:02X})"
        )

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _init_bno(self):
        """Initialise the BNO085 over I2C; returns the driver or None on failure."""
        try:
            import board  # noqa: PLC0415
            import busio  # noqa: PLC0415
            from adafruit_bno08x import (  # noqa: PLC0415
                BNO_REPORT_GYROSCOPE,
                BNO_REPORT_LINEAR_ACCELERATION,
                BNO_REPORT_ROTATION_VECTOR,
            )
            from adafruit_bno08x.i2c import BNO08X_I2C  # noqa: PLC0415
        except ImportError as exc:
            self.get_logger().error(
                f"IMU library unavailable ({exc}). "
                "Install: pip3 install --break-system-packages adafruit-circuitpython-bno08x"
            )
            return None

        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            bno = BNO08X_I2C(i2c, address=self._i2c_address)
            bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            bno.enable_feature(BNO_REPORT_GYROSCOPE)
            bno.enable_feature(BNO_REPORT_LINEAR_ACCELERATION)
            # Keep module-level names accessible for the read loop.
            self._BNO_REPORT_ROTATION_VECTOR = BNO_REPORT_ROTATION_VECTOR
            self._BNO_REPORT_GYROSCOPE = BNO_REPORT_GYROSCOPE
            self._BNO_REPORT_LINEAR_ACCELERATION = BNO_REPORT_LINEAR_ACCELERATION
            self.get_logger().info(
                f"BNO085 found at I2C address 0x{self._i2c_address:02X}."
            )
            return bno
        except Exception as exc:
            self.get_logger().error(
                f"Failed to initialise BNO085 at 0x{self._i2c_address:02X}: {exc}. "
                "Check wiring and that i2c_arm is enabled in /boot/firmware/config.txt."
            )
            return None

    def _timer_cb(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id

        # Pre-fill covariances (same every message — avoids reallocation).
        msg.orientation_covariance = _diag_cov(_ORIENT_COV_DIAG)
        msg.angular_velocity_covariance = _diag_cov(_GYRO_COV_DIAG)
        msg.linear_acceleration_covariance = _diag_cov(_ACCEL_COV_DIAG)

        if self._bno is None:
            # Simulation / hardware unavailable: publish identity quaternion.
            msg.orientation.w = 1.0
            self._pub.publish(msg)
            return

        try:
            quat = self._bno.quaternion  # (i, j, k, real) — note order!
            gyro = self._bno.gyro
            accel = self._bno.linear_acceleration
        except Exception as exc:
            self.get_logger().warn(f"BNO085 read error: {exc}", throttle_duration_sec=5.0)
            return

        if quat is None or gyro is None or accel is None:
            # Sensor not yet ready; skip this tick silently.
            return

        # BNO08x quaternion order: (i, j, k, real) = (x, y, z, w)
        qi, qj, qk, qreal = quat
        msg.orientation.x = qi
        msg.orientation.y = qj
        msg.orientation.z = qk
        msg.orientation.w = qreal

        gx, gy, gz = gyro
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz

        ax, ay, az = accel
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
