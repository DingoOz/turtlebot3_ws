# test/test_xbox_teleop.py
import unittest
from unittest.mock import MagicMock, patch
import pytest
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy

from xbox_turtlebot3_teleop.xbox_teleop_node import XboxTeleopNode

@pytest.fixture(scope="session", autouse=True)
def ros_context():
    print("\nInitializing ROS2 context")
    rclpy.init()
    yield
    print("\nShutting down ROS2 context")
    rclpy.shutdown()

@pytest.fixture
def xbox_node(ros_context):
    print("\nCreating Xbox Teleop Node")
    node = XboxTeleopNode()
    yield node
    print("\nDestroying Xbox Teleop Node")
    node.destroy_node()

class TestXboxTeleopNode:
    def test_initialization(self, xbox_node, capsys):
        """Test that the node initializes with correct default parameters"""
        print("\nTesting node initialization...")
        assert xbox_node.max_linear_speed == 0.5, "Incorrect max linear speed"
        assert xbox_node.max_angular_speed == 1.5, "Incorrect max angular speed"
        assert xbox_node.deadman_trigger_axis == 5, "Incorrect deadman trigger axis"
        print("✓ Node initialized with correct parameters")

    def test_dead_man_switch_disengaged(self, xbox_node):
        """Test that no movement occurs when dead man's switch is disengaged"""
        print("\nTesting deadman switch disengagement...")
        published_msgs = []
        xbox_node.publisher_.publish = MagicMock(side_effect=published_msgs.append)

        joy_msg = Joy()
        joy_msg.axes = [0.0] * 6
        joy_msg.axes[5] = 1.0  # Right trigger not pressed
        
        xbox_node.joy_callback(joy_msg)
        
        assert len(published_msgs) == 1, "Expected one message to be published"
        assert published_msgs[0].linear.x == 0.0, "Expected zero linear velocity"
        assert published_msgs[0].angular.z == 0.0, "Expected zero angular velocity"
        print("✓ Robot correctly stops when deadman switch is disengaged")

    # ... [Keep other test methods, adding print statements similarly]

    @pytest.mark.parametrize("linear_speed,angular_speed", [
        (0.3, 1.0),
        (0.7, 2.0),
        (0.1, 0.5)
    ])
    def test_parameter_changes(self, ros_context, linear_speed, angular_speed):
        """Test that the node responds correctly to different parameter values"""
        print(f"\nTesting with linear_speed={linear_speed}, angular_speed={angular_speed}")
        node = XboxTeleopNode()
        node.max_linear_speed = linear_speed
        node.max_angular_speed = angular_speed

        published_msgs = []
        node.publisher_.publish = MagicMock(side_effect=published_msgs.append)

        joy_msg = Joy()
        joy_msg.axes = [0.0] * 6
        joy_msg.axes[1] = 1.0
        joy_msg.axes[5] = -1.0
        
        node.joy_callback(joy_msg)
        
        assert len(published_msgs) == 1
        assert published_msgs[0].linear.x == pytest.approx(linear_speed)
        print(f"✓ Node correctly handles {linear_speed}m/s speed parameter")
        node.destroy_node()
        
