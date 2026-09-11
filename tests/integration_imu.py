"""Run with ROS sourced: PYTHONPATH=. python3 tests/integration_imu.py."""

import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from edubot_hardware.imu_node import ImuNode


class ImuPublishingTest(unittest.TestCase):
    def test_missing_failed_and_working_sensor(self):
        rclpy.init()
        with patch.object(ImuNode, "_init_bno", return_value=None):
            node = ImuNode()
        received = []
        node.create_subscription(Imu, "imu/data", received.append, qos_profile_sensor_data)

        def spin(seconds):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.02)

        try:
            spin(0.5)
            self.assertEqual(received, [], "Absent IMU must not report zero motion")
            node._bno = SimpleNamespace(
                quaternion=(0.0, 0.0, 0.0, 1.0),
                gyro=(0.0, 0.0, 0.6),
                linear_acceleration=(0.0, 0.0, 0.0),
            )
            spin(0.5)
            self.assertGreater(len(received), 0)
            self.assertAlmostEqual(received[-1].angular_velocity.z, 0.6)
            node._bno = SimpleNamespace(quaternion=None, gyro=None, linear_acceleration=None)
            spin(0.1)  # Drain previously queued DDS messages.
            received.clear()
            spin(0.3)
            self.assertEqual(received, [], "Unreadable IMU must not report zero motion")
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
