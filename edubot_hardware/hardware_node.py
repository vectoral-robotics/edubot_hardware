# edubot_hardware/hardware_node.py
"""
Main ROS2 hardware node for EduBot (ESP32-based).

Bridges ROS messages (/cmd_vel, /odom, /joint_states, TF)
with either a real ESP32-based motor controller (SerialBridge)
or a simulated backend (SimulationInterface).

ESP32 protocol:
 - TX: "M w_rr w_fr w_rl w_fl"  (wheel angular velocities [rad/s])
 - RX: "E seq timestamp_us t_rr t_fr t_rl t_fl"  (cumulative encoder ticks)
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

from .mecanum_kinematics import MecanumKinematics
from .odometry import OdometryEstimator


class HardwareNode(Node):
    """ROS2 Node managing the EduBot ESP32 hardware or simulation backend."""

    def __init__(self):
        super().__init__("hardware_node")
        self.get_logger().info("EduBot ESP32 Hardware Node starting up...")

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter("use_sim", False)
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("wheel_radius", 0.04)
        self.declare_parameter("base_length", 0.087)
        self.declare_parameter("base_width", 0.1154)
        self.declare_parameter("ticks_per_rev", 4320.0)
        self.declare_parameter("cmd_timeout", 0.5)
        self.declare_parameter("mecanum_layout", "X")
        self.declare_parameter("log_commands", True)
        self.declare_parameter("odom_hz", 50.0)
        self.declare_parameter("tf_hz", 30.0)
        # Whether this node broadcasts the odom -> base_link TF itself. Set to
        # false when a sensor-fusion node (robot_localization EKF) owns that
        # transform instead; two publishers for the same TF corrupt the tree.
        self.declare_parameter("publish_tf", True)

        # Odometry covariance diagonals. Values are variances (= error^2).
        # For a mecanum base the sideways (vy) and yaw estimates are less
        # trustworthy than forward (vx) because the passive rollers slip, so
        # they get larger variances. Consumers like Nav2 / robot_localization
        # use these to weight wheel odometry against the lidar; a zero (default)
        # covariance would wrongly claim perfect certainty.
        # Pose diagonal: [x, y, z, roll, pitch, yaw]
        self.declare_parameter("pose_covariance_diagonal", [0.01, 0.05, 1e6, 1e6, 1e6, 0.1])
        # Twist diagonal: [vx, vy, vz, wx, wy, wz]
        self.declare_parameter("twist_covariance_diagonal", [0.002, 0.01, 1e6, 1e6, 1e6, 0.02])
        use_sim = bool(self.get_parameter("use_sim").value)
        port = self.get_parameter("port").value
        baud = self.get_parameter("baud").value
        R = float(self.get_parameter("wheel_radius").value)
        Lx = float(self.get_parameter("base_length").value)
        Ly = float(self.get_parameter("base_width").value)
        ticks_per_rev = float(self.get_parameter("ticks_per_rev").value)
        layout = self.get_parameter("mecanum_layout").value.upper()
        self._odom_hz = float(self.get_parameter("odom_hz").value)
        self._tf_hz = float(self.get_parameter("tf_hz").value)
        self._log_commands = bool(self.get_parameter("log_commands").value)
        self._publish_tf = bool(self.get_parameter("publish_tf").value)

        # Pre-build the full 6x6 (36-element, row-major) covariance matrices
        # once from the configured diagonals; they are constant per run.
        self._pose_covariance = self._diag_to_covariance(
            self.get_parameter("pose_covariance_diagonal").value
        )
        self._twist_covariance = self._diag_to_covariance(
            self.get_parameter("twist_covariance_diagonal").value
        )

        # ------------------------------------------------------------------
        # QoS Profiles
        # ------------------------------------------------------------------
        qos_cmd = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        qos_odom = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_joint = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, depth=10
        )

        # ------------------------------------------------------------------
        # Backend selection (real ↔ simulation)
        # ------------------------------------------------------------------
        if use_sim:
            from .simulation_interface import SimulationInterface

            self.backend = SimulationInterface(ticks_per_rev, R, logger=self.get_logger())
            self.get_logger().info("Running in SIMULATION mode.")
        else:
            from .serial_interface import SerialBridge

            self.backend = SerialBridge(port, baud, logger=self.get_logger())
            self.get_logger().info(f"Connected to ESP32 on {port} @ {baud} baud.")

        # ------------------------------------------------------------------
        # Submodules
        # ------------------------------------------------------------------
        self.kin = MecanumKinematics(R, Lx, Ly, layout)
        self.odom = OdometryEstimator(R, Lx, Ly, ticks_per_rev, layout)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ------------------------------------------------------------------
        # ROS interfaces
        # ------------------------------------------------------------------
        self.cmd_sub = self.create_subscription(Twist, "/cmd_vel", self.cmd_cb, qos_cmd)
        self.joint_pub = self.create_publisher(JointState, "/joint_states", qos_joint)
        self.odom_pub = self.create_publisher(Odometry, "/odom", qos_odom)

        # Internal state
        self._last_cmd = Twist()
        self._last_cmd_time = self.get_clock().now()
        self._cmd_timeout = float(self.get_parameter("cmd_timeout").value)
        self._last_encoder_ts = None
        self._last_tf_stamp = None

        # Timers
        self.create_timer(1.0 / self._odom_hz, self._update_loop)
        # Only broadcast the odom -> base_link TF when no EKF owns it.
        if self._publish_tf:
            self.create_timer(1.0 / self._tf_hz, self._publish_tf_cb)
            self.get_logger().info("Publishing odom -> base_link TF.")
        else:
            self.get_logger().info("publish_tf=false: leaving odom -> base_link TF to the EKF.")

        self.get_logger().info("ESP32 HardwareNode initialized successfully.")

    # ------------------------------------------------------------------
    def cmd_cb(self, msg: Twist):
        """Store the latest /cmd_vel message."""
        self._last_cmd = msg
        self._last_cmd_time = self.get_clock().now()

        if self._log_commands:
            self.get_logger().debug(
                f"/cmd_vel: vx={msg.linear.x:.3f}, vy={msg.linear.y:.3f}, wz={msg.angular.z:.3f}"
            )

    # ------------------------------------------------------------------
    def _update_loop(self):
        """Periodic update: send motor commands, read encoders, update odometry."""
        now = self.get_clock().now()
        dt_cmd = (now - self._last_cmd_time).nanoseconds * 1e-9

        # Stop if command timeout exceeded
        if 0.0 < self._cmd_timeout < dt_cmd:
            vx = vy = wz = 0.0
        else:
            vx = self._last_cmd.linear.x
            vy = self._last_cmd.linear.y
            wz = self._last_cmd.angular.z

        # ---- Inverse kinematics ----
        w_rr, w_fr, w_rl, w_fl = self.kin.inverse(vx, vy, wz)
        self.backend.send_motor_speeds(w_rr, w_fr, w_rl, w_fl)

        if self._log_commands:
            self.get_logger().debug(
                f"Motor cmd: w_rr={w_rr:.2f}, w_fr={w_fr:.2f}, w_rl={w_rl:.2f}, w_fl={w_fl:.2f}"
            )

        # ---- Read encoder data ----
        for line in self.backend.read_lines():
            enc = self.backend.parse_encoder_line(line)
            if not enc:
                continue

            seq, ts_us, t_rr, t_fr, t_rl, t_fl = enc

            dt = None
            if ts_us is not None:
                if self._last_encoder_ts is not None:
                    dt = (ts_us - self._last_encoder_ts) / 1e6
                self._last_encoder_ts = ts_us

            if dt is None or dt <= 0.0 or dt > 0.2:
                self.get_logger().debug(f"Skipping odometry update: invalid dt={dt}")
                continue

            # Pass ticks in correct order (RR, FR, RL, FL)
            vel = self.odom.update((t_rr, t_fr, t_rl, t_fl), dt)
            if vel:
                vx, vy, wz = vel
                self._publish_odometry(vx, vy, wz)

    # ------------------------------------------------------------------
    @staticmethod
    def _diag_to_covariance(diagonal) -> list[float]:
        """
        Expand a 6-element variance diagonal into a 36-element (row-major)
        6x6 covariance matrix as expected by nav_msgs/Odometry.
        """
        diag = [float(v) for v in diagonal]
        if len(diag) != 6:
            raise ValueError(f"covariance diagonal must have 6 values, got {len(diag)}")
        cov = [0.0] * 36
        for i in range(6):
            cov[i * 6 + i] = diag[i]
        return cov

    # ------------------------------------------------------------------
    def _publish_odometry(self, vx: float, vy: float, wz: float):
        """Publish Odometry and JointState messages."""
        x, y, yaw = self.odom.get_pose()
        stamp = self.get_clock().now().to_msg()

        # --- Odometry message ---
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = math.sin(yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(yaw / 2.0)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        odom.pose.covariance = self._pose_covariance
        odom.twist.covariance = self._twist_covariance
        self.odom_pub.publish(odom)

        # --- JointState message ---
        js = JointState()
        js.header.stamp = stamp
        js.name = ["wheel_rr", "wheel_fr", "wheel_rl", "wheel_fl"]
        js.position = self.odom.get_wheel_angles()
        js.velocity = [0.0, 0.0, 0.0, 0.0]
        self.joint_pub.publish(js)

        # Store for TF publisher
        self._last_tf_stamp = (x, y, yaw, stamp)

        if self._log_commands:
            self.get_logger().debug(
                f"Odom update: x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.1f}°, "
                f"vx={vx:.3f}, vy={vy:.3f}, wz={wz:.3f}"
            )

    # ------------------------------------------------------------------
    def _publish_tf_cb(self):
        """Broadcast the odom -> base_link transform (only when publish_tf)."""
        if self._last_tf_stamp is None:
            return

        x, y, yaw, stamp = self._last_tf_stamp
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)
        self.tf_broadcaster.sendTransform(t)

    # ------------------------------------------------------------------
    def destroy_node(self):
        """Ensure backend closed cleanly on shutdown."""
        self.get_logger().info("Shutting down ESP32 HardwareNode...")
        try:
            self.backend.close()
        except Exception as e:
            self.get_logger().warn(f"Error while closing backend: {e}")
        super().destroy_node()


# ----------------------------------------------------------------------
def main(args=None):
    """Main entry point for the ESP32 hardware node."""
    rclpy.init(args=args)
    node = HardwareNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt — stopping EduBot.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
