#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial, time, math

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class HardwareNode(Node):
    def __init__(self):
        super().__init__('hardware_node')

        # ---------------- Parameters ----------------
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('wheel_radius', 0.04)
        self.declare_parameter('base_length', 0.095)   # [m] center ↔ front/back axle
        self.declare_parameter('base_width', 0.1025)   # [m] center ↔ left/right axle
        self.declare_parameter('ticks_per_rev', 4320.0)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value

        # ---------------- Serial ----------------
        self.ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(2.0)
        self.get_logger().info(f"✅ Connected to Arduino on {port} at {baud}")

        # ---------------- ROS interfaces ----------------
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ---------------- State ----------------
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_time = self.get_clock().now()

        # ---------------- Timers ----------------
        self.create_timer(0.05, self.read_serial)  # 20 Hz

    # =====================================================
    # Handle incoming /cmd_vel and compute wheel speeds
    # =====================================================
    def cmd_cb(self, msg: Twist):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        R = self.get_parameter('wheel_radius').value
        Lx = self.get_parameter('base_length').value
        Ly = self.get_parameter('base_width').value

        # ---- Mecanum Inverse Kinematics (theoretical model) ----
        w_fl = (1/R) * (vx - vy - (Lx+Ly)*wz)  # Front-Left
        w_fr = (1/R) * (vx + vy + (Lx+Ly)*wz)  # Front-Right
        w_rl = (1/R) * (vx + vy - (Lx+Ly)*wz)  # Rear-Left
        w_rr = (1/R) * (vx - vy + (Lx+Ly)*wz)  # Rear-Right

        # ---- Physical correction (your wiring/motor orientation) ----
        w_fl *= -1   # Front-Left inverted
        w_rl *= -1   # Rear-Left inverted
        w_fr *=  1   # Front-Right OK
        w_rr *=  1   # Rear-Right OK

        # ---- Order matches Arduino expectation ----
        cmd = f"M {w_fl:.2f} {w_rl:.2f} {w_rr:.2f} {w_fr:.2f}\n"
        self.ser.write(cmd.encode('utf-8'))
        self.get_logger().debug(f"Sent: {cmd.strip()}")

    # =====================================================
    # Read encoder data from Arduino and publish odometry
    # =====================================================
    def read_serial(self):
        line = self.ser.readline().decode('utf-8', errors='ignore').strip()
        if not line.startswith("E"):
            return

        parts = line.split()
        if len(parts) != 5:
            return

        try:
            # Order coming from Arduino: d1=FL, d2=RL, d3=RR, d4=FR
            d_fl, d_rl, d_rr, d_fr = [int(p) for p in parts[1:]]
        except ValueError:
            return

        # Δt
        dt = (self.get_clock().now() - self.last_time).nanoseconds * 1e-9
        self.last_time = self.get_clock().now()

        ticks_per_rev = self.get_parameter('ticks_per_rev').value
        R = self.get_parameter('wheel_radius').value
        Lx = self.get_parameter('base_length').value
        Ly = self.get_parameter('base_width').value

        # ---- Wheel angular velocities (rad/s) ----
        ws_fl = (d_fl/ticks_per_rev) * 2*math.pi / dt
        ws_rl = (d_rl/ticks_per_rev) * 2*math.pi / dt
        ws_rr = (d_rr/ticks_per_rev) * 2*math.pi / dt
        ws_fr = (d_fr/ticks_per_rev) * 2*math.pi / dt

        ws = [ws_fl, ws_fr, ws_rl, ws_rr]

        # ---- Forward Kinematics ----
        vx = (R/4)*(ws_fl + ws_fr + ws_rl + ws_rr)
        vy = (R/4)*(-ws_fl + ws_fr + ws_rl - ws_rr)
        wz = (R/(4*(Lx+Ly)))*(-ws_fl + ws_fr - ws_rl + ws_rr)

        # ---- Odom integration ----
        self.x += vx*dt*math.cos(self.yaw) - vy*dt*math.sin(self.yaw)
        self.y += vx*dt*math.sin(self.yaw) + vy*dt*math.cos(self.yaw)
        self.yaw += wz*dt

        # ---- Joint States ----
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['wheel_fl_joint','wheel_fr_joint','wheel_rl_joint','wheel_rr_joint']
        js.position = [0.0, 0.0, 0.0, 0.0]  # TODO: integrate wheel angles later
        js.velocity = ws
        self.joint_pub.publish(js)

        # ---- Odometry ----
        odom = Odometry()
        odom.header.stamp = js.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw/2)
        odom.pose.pose.orientation.w = math.cos(self.yaw/2)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)

        # ---- TF ----
        t = TransformStamped()
        t.header.stamp = js.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = odom.pose.pose.orientation.z
        t.transform.rotation.w = odom.pose.pose.orientation.w
        self.tf_broadcaster.sendTransform(t)


# =====================================================
# Main
# =====================================================
def main(args=None):
    rclpy.init(args=args)
    node = HardwareNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
