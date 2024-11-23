# tests/test_xbox_teleop.py
import unittest
from unittest.mock import MagicMock, patch

import pytest
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy

from xbox_turtlebot3_teleop.xbox_teleop_node import XboxTeleopNode

@pytest.fixture
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()

@pytest.fixture
def xbox_node(ros_context):
    node = XboxTeleopNode()
    yield node
    node.destroy_node()

class TestXboxTeleopNode:
    def test_initialization(self, xbox_node):
        """Test that the node initializes with correct default parameters"""
        assert xbox_node.max_linear_speed == 0.5
        assert xbox_node.max_angular_speed == 1.5
        assert xbox_node.deadman_trigger_axis == 5

    def test_dead_man_switch_disengaged(self, xbox_node):
        """Test that no movement occurs when dead man's switch is disengaged"""
        published_msgs = []
        xbox_node.publisher_.publish = MagicMock(side_effect=published_msgs.append)

        joy_msg = Joy()
        joy_msg.axes = [0.0] * 6  # Initialize all axes to 0
        joy_msg.axes[5] = 1.0  # Right trigger not pressed
        
        xbox_node.joy_callback(joy_msg)
        
        assert len(published_msgs) == 1
        assert published_msgs[0].linear.x == 0.0
        assert published_msgs[0].angular.z == 0.0

    def test_forward_movement(self, xbox_node):
        """Test forward movement with dead man's switch engaged"""
        published_msgs = []
        xbox_node.publisher_.publish = MagicMock(side_effect=published_msgs.append)

        joy_msg = Joy()
        joy_msg.axes = [0.0] * 6
        joy_msg.axes[1] = 1.0  # Full forward on left stick
        joy_msg.axes[5] = -1.0  # Dead man's switch engaged
        
        xbox_node.joy_callback(joy_msg)
        
        assert len(published_msgs) == 1
        assert published_msgs[0].linear.x == pytest.approx(0.5)  # Max speed is 0.5 m/s
        assert published_msgs[0].angular.z == pytest.approx(0.0)

    def test_reverse_movement(self, xbox_node):
        """Test reverse movement with dead man's switch engaged"""
        published_msgs = []
        xbox_node.publisher_.publish = MagicMock(side_effect=published_msgs.append)

        joy_msg = Joy()
        joy_msg.axes = [0.0] * 6
        joy_msg.axes[1] = -1.0  # Full reverse on left stick
        joy_msg.axes[5] = -1.0  # Dead man's switch engaged
        
        xbox_node.joy_callback(joy_msg)
        
        assert len(published_msgs) == 1
        assert published_msgs[0].linear.x == pytest.approx(-0.5)  # Max speed is -0.5 m/s
        assert published_msgs[0].angular.z == pytest.approx(0.0)

    def test_rotation(self, xbox_node):
        """Test rotation with dead man's switch engaged"""
        published_msgs = []
        xbox_node.publisher_.publish = MagicMock(side_effect=published_msgs.append)

        joy_msg = Joy()
        joy_msg.axes = [0.0] * 6
        joy_msg.axes[2] = 1.0  # Full left on stick
        joy_msg.axes[5] = -1.0  # Dead man's switch engaged
        
        xbox_node.joy_callback(joy_msg)
        
        assert len(published_msgs) == 1
        assert published_msgs[0].linear.x == pytest.approx(0.0)
        assert abs(published_msgs[0].angular.z) > 0.0

    def test_combined_movement(self, xbox_node):
        """Test combined forward movement and rotation"""
        published_msgs = []
        xbox_node.publisher_.publish = MagicMock(side_effect=published_msgs.append)

        joy_msg = Joy()
        joy_msg.axes = [0.0] * 6
        joy_msg.axes[1] = 1.0  # Full forward
        joy_msg.axes[2] = 1.0  # Full left
        joy_msg.axes[5] = -1.0  # Dead man's switch engaged
        
        xbox_node.joy_callback(joy_msg)
        
        assert len(published_msgs) == 1
        assert published_msgs[0].linear.x > 0.0
        assert published_msgs[0].angular.z != 0.0

    def test_invalid_joy_message(self, xbox_node):
        """Test handling of invalid joy message"""
        published_msgs = []
        xbox_node.publisher_.publish = MagicMock(side_effect=published_msgs.append)

        joy_msg = Joy()
        joy_msg.axes = [0.0] * 3  # Too few axes
        
        xbox_node.joy_callback(joy_msg)
        
        assert len(published_msgs) == 1
        assert published_msgs[0].linear.x == 0.0
        assert published_msgs[0].angular.z == 0.0

    @pytest.mark.parametrize("linear_speed,angular_speed", [
        (0.3, 1.0),
        (0.7, 2.0),
        (0.1, 0.5)
    ])
    def test_parameter_changes(self, ros_context, linear_speed, angular_speed):
        """Test that the node responds correctly to different parameter values"""
        node = XboxTeleopNode()
        node.max_linear_speed = linear_speed
        node.max_angular_speed = angular_speed

        published_msgs = []
        node.publisher_.publish = MagicMock(side_effect=published_msgs.append)

        joy_msg = Joy()
        joy_msg.axes = [0.0] * 6
        joy_msg.axes[1] = 1.0  # Full forward
        joy_msg.axes[5] = -1.0  # Dead man's switch engaged
        
        node.joy_callback(joy_msg)
        
        assert len(published_msgs) == 1
        assert published_msgs[0].linear.x == pytest.approx(linear_speed)
        node.destroy_node()
