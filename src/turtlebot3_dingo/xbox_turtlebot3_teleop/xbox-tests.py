import unittest

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy
from xbox_turtlebot3_teleop.xbox_teleop_node import XboxTeleopNode


class TestXboxTeleopNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = XboxTeleopNode()

    def tearDown(self):
        self.node.destroy_node()

    def test_dead_man_switch_disengaged(self):
        # Create a Joy message with dead man's switch (right trigger) not pressed
        joy_msg = Joy()
        joy_msg.axes = [0.0] * 6  # Initialize all axes to 0
        joy_msg.axes[5] = 0.0  # Right trigger not pressed

        # Call the callback
        self.node.joy_callback(joy_msg)

        # Get the last published message
        test_msgs = []
        self.node.publisher_.publish = lambda msg: test_msgs.append(msg)

        # Verify no movement when dead man's switch is disengaged
        self.assertEqual(len(test_msgs), 0)

    def test_forward_movement(self):
        # Create a Joy message for forward movement
        joy_msg = Joy()
        joy_msg.axes = [0.0] * 6
        joy_msg.axes[1] = 1.0  # Full forward on left stick
        joy_msg.axes[5] = -1.0  # Dead man's switch engaged

        # Get the published message
        test_msgs = []
        self.node.publisher_.publish = lambda msg: test_msgs.append(msg)

        # Call the callback
        self.node.joy_callback(joy_msg)

        # Verify forward movement
        self.assertEqual(len(test_msgs), 1)
        self.assertAlmostEqual(test_msgs[0].linear.x, 0.5)  # Max speed is 0.5 m/s
        self.assertAlmostEqual(test_msgs[0].angular.z, 0.0)

    def test_rotation(self):
        # Create a Joy message for rotation
        joy_msg = Joy()
        joy_msg.axes = [0.0] * 6
        joy_msg.axes[2] = 1.0  # Full left on stick
        joy_msg.axes[5] = -1.0  # Dead man's switch engaged

        # Get the published message
        test_msgs = []
        self.node.publisher_.publish = lambda msg: test_msgs.append(msg)

        # Call the callback
        self.node.joy_callback(joy_msg)

        # Verify rotation
        self.assertEqual(len(test_msgs), 1)
        self.assertAlmostEqual(test_msgs[0].linear.x, 0.0)
        self.assertNotEqual(test_msgs[0].angular.z, 0.0)


def main():
    unittest.main()
