#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial, time, math

from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class HardwareNode(Node):
    def __init__(self):
        super().__init__('hardware_node')

        # ---------------- Parameters ----------------
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('wheel_radius', 0.04)   # R
        self.declare_parameter('base_length', 0.095)   # Lx
        self.declare_parameter('base_width', 0.1025)   # Ly
        self.declare_parameter('ticks_per_rev', 4320.0)
        self.declare_parameter('encoder_dt', 0.02)     # 50 Hz Arduino
        self.declare_parameter('cmd_timeout', 0.5)     # s (0.0 = halten)
        self.declare_parameter('mecanum_layout', 'X')  # 'X' oder 'O'
        self.declare_parameter('log_commands', True)  # Debug-Ausgabe der M-Befehle

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value

        # ---------------- Serial ----------------
        self.ser = serial.Serial(port, baud, timeout=0)  # non-blocking read
        time.sleep(2.0)
        self.get_logger().info(f"✅ Connected to Arduino on {port} @ {baud} baud")
        self._rxbuf = bytearray()

        # ---------------- ROS interfaces ----------------
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ---------------- State ----------------
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # letzter empfangener Twist
        self._last_twist = Twist()
        self._last_cmd_time = self.get_clock().now()
        self._cmd_timeout = float(self.get_parameter('cmd_timeout').value)

        # Encoder-Zustand (kumulativ, Arduino: 1=FL, 2=RL, 3=RR, 4=FR)
        self._last_seq = None
        self._last_t_fl = 0
        self._last_t_rl = 0
        self._last_t_rr = 0
        self._last_t_fr = 0
        self._have_prev_ticks = False

        # ---------------- Timers ----------------
        # Sende-Takt: 50 Hz (entkoppelt von /cmd_vel)
        self._send_hz = 50.0
        self.create_timer(1.0/self._send_hz, self._send_cmd)

        # Serial lesen: 200 Hz
        self.create_timer(0.005, self._read_serial_fast)

        # Diagnose
        self._frames = 0
        self._last_frames_time = self.get_clock().now()
        self.create_timer(1.0, self._diag)

        # Debug Throttle
        self._last_cmd_log = 0.0

    # =====================================================
    # /cmd_vel nur puffern
    # =====================================================
    def cmd_cb(self, msg: Twist):
        self._last_twist = msg
        self._last_cmd_time = self.get_clock().now()

    # =====================================================
    # Zyklisch mit fixer Rate an Arduino senden
    #  -> KEINE Invertierungen auf Befehlseite!
    #  -> Reihenfolge: [FL, RL, RR, FR]
    # =====================================================
    def _send_cmd(self):
        # Timeout-Handling
        if self._cmd_timeout > 0.0:
            dt_cmd = (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9
            if dt_cmd > self._cmd_timeout:
                vx = vy = wz = 0.0
            else:
                vx = self._last_twist.linear.x
                vy = self._last_twist.linear.y
                wz = self._last_twist.angular.z
        else:
            vx = self._last_twist.linear.x
            vy = self._last_twist.linear.y
            wz = self._last_twist.angular.z

        R  = float(self.get_parameter('wheel_radius').value)
        Lx = float(self.get_parameter('base_length').value)
        Ly = float(self.get_parameter('base_width').value)
        L  = (Lx + Ly)

        # Layout-Schalter: s = +1 für "X", s = -1 für "O"
        layout = self.get_parameter('mecanum_layout').value.upper()
        s = 1.0 if layout == 'X' else -1.0

        # ---- Mecanum inverse kinematics (ROS-Konvention: x vor, y links, z CCW) ----
        # Physische Namen:
        #   FL = Front-Left, FR = Front-Right, RL = Rear-Left, RR = Rear-Right
        w_fl = (1.0/R) * (vx - s*vy - L*wz)  # FL
        w_fr = (1.0/R) * (vx + s*vy + L*wz)  # FR
        w_rl = (1.0/R) * (vx + s*vy - L*wz)  # RL
        w_rr = (1.0/R) * (vx - s*vy + L*wz)  # RR

        # ---- Arduino-Reihenfolge ----
        #   Motor 1 = FL,  Motor 2 = RL,  Motor 3 = RR,  Motor 4 = FR
        cmd = f"M {w_fl:.2f} {w_rl:.2f} {w_rr:.2f} {w_fr:.2f}\n"

        # optional: Debug einmal pro ~0.2 s
        if bool(self.get_parameter('log_commands').value):
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - self._last_cmd_log > 0.2:
                self._last_cmd_log = now
                self.get_logger().info(f"M-cmd: FL={w_fl:.2f}, RL={w_rl:.2f}, RR={w_rr:.2f}, FR={w_fr:.2f}")

        try:
            self.ser.write(cmd.encode('utf-8'))
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write error: {e}")

    # =====================================================
    # Seriell lesen
    # =====================================================
    def _read_serial_fast(self):
        try:
            n = self.ser.in_waiting
            if n > 0:
                chunk = self.ser.read(n)
                self._rxbuf.extend(chunk)
        except serial.SerialException as e:
            self.get_logger().error(f"Serial read error: {e}")
            return

        while True:
            idx = self._rxbuf.find(b'\n')
            if idx < 0:
                break
            line = self._rxbuf[:idx].decode('utf-8', errors='ignore').strip()
            self._rxbuf = self._rxbuf[idx+1:]
            self._process_line(line)

    # =====================================================
    # "E seq t_fl t_rl t_rr t_fr"
    #  -> RR & FR Encoder-Polung invertieren (aus Messung)
    # =====================================================
    def _process_line(self, line: str):
        if not line or not line.startswith('E'):
            return

        parts = line.split()
        if len(parts) != 6:
            return

        try:
            seq = int(parts[1])
            t_fl = int(parts[2])
            t_rl = int(parts[3])
            t_rr = int(parts[4])
            t_fr = int(parts[5])
        except ValueError:
            return

        # Frameverlust melden
        if self._last_seq is not None and seq != self._last_seq + 1:
            missed = seq - self._last_seq - 1
            if missed > 0:
                self.get_logger().warn(f"⚠️ Missed {missed} encoder frame(s)")
        self._last_seq = seq

        # erster Satz -> baselines setzen
        if not self._have_prev_ticks:
            self._last_t_fl = t_fl
            self._last_t_rl = t_rl
            self._last_t_rr = t_rr
            self._last_t_fr = t_fr
            self._have_prev_ticks = True
            return

        # Differenzen
        d_fl = t_fl - self._last_t_fl
        d_rl = t_rl - self._last_t_rl
        d_rr = t_rr - self._last_t_rr
        d_fr = t_fr - self._last_t_fr

        self._last_t_fl = t_fl
        self._last_t_rl = t_rl
        self._last_t_rr = t_rr
        self._last_t_fr = t_fr

        self._frames += 1

        # Encoder-Polungskorrektur (gemessen): FR & RR invertieren
        d_rr *= -1
        d_fr *= -1

        # Winkelgeschwindigkeit (rad/s)
        ticks_per_rev = float(self.get_parameter('ticks_per_rev').value)
        R  = float(self.get_parameter('wheel_radius').value)
        Lx = float(self.get_parameter('base_length').value)
        Ly = float(self.get_parameter('base_width').value)
        L  = (Lx + Ly)
        dt = float(self.get_parameter('encoder_dt').value)

        two_pi_over_dt = (2.0 * math.pi) / dt
        ws_fl = (d_fl / ticks_per_rev) * two_pi_over_dt
        ws_rl = (d_rl / ticks_per_rev) * two_pi_over_dt
        ws_rr = (d_rr / ticks_per_rev) * two_pi_over_dt
        ws_fr = (d_fr / ticks_per_rev) * two_pi_over_dt

        # Forward Kinematics (Layout beachten)
        layout = self.get_parameter('mecanum_layout').value.upper()
        s = 1.0 if layout == 'X' else -1.0

        vx = (R / 4.0) * (ws_fl + ws_fr + ws_rl + ws_rr)
        vy = (R / 4.0) * ( -s*ws_fl + s*ws_fr + s*ws_rl - s*ws_rr )
        wz = (R / (4.0 * L)) * ( -ws_fl + ws_fr - ws_rl + ws_rr )

        # Odometrie
        cos_y = math.cos(self.yaw)
        sin_y = math.sin(self.yaw)
        self.x   += vx * dt * cos_y - vy * dt * sin_y
        self.y   += vx * dt * sin_y + vy * dt * cos_y
        self.yaw += wz * dt

        # Joint States (FL, FR, RL, RR)
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['wheel_fl_joint', 'wheel_fr_joint', 'wheel_rl_joint', 'wheel_rr_joint']
        js.position = [0.0, 0.0, 0.0, 0.0]
        js.velocity = [ws_fl, ws_fr, ws_rl, ws_rr]
        self.joint_pub.publish(js)

        # Odometry
        odom = Odometry()
        odom.header.stamp = js.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        odom.twist.twist.linear.x  = vx
        odom.twist.twist.linear.y  = vy
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)

        # TF
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
    # Diagnose
    # =====================================================
    def _diag(self):
        now = self.get_clock().now()
        dt = (now - self._last_frames_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        fps = self._frames / dt
        self.get_logger().debug(f"Encoder FPS ≈ {fps:.1f} (expected 50.0)")
        self._frames = 0
        self._last_frames_time = now


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
