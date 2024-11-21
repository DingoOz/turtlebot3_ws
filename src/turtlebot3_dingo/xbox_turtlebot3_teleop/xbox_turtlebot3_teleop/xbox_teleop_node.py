#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from sensor_msgs.msg import Joy


class XboxTeleopNode(Node):
    def __init__(self):
        super().__init__("xbox_teleop_node")

        # Declare parameters with proper descriptors
        self.declare_parameter(
            "max_linear_speed", 0.5, ParameterDescriptor(description="Maximum linear speed in m/s")
        )
        self.declare_parameter(
            "max_angular_speed",
            1.5,
            ParameterDescriptor(description="Maximum angular speed in rad/s"),
        )
        self.declare_parameter(
            "deadman_trigger_axis",
            5,
            ParameterDescriptor(description="Axis number for deadman trigger"),
        )

        # Get parameters
        self.max_linear_speed = self.get_parameter("max_linear_speed").value
        self.max_angular_speed = self.get_parameter("max_angular_speed").value
        self.deadman_trigger_axis = self.get_parameter("deadman_trigger_axis").value

        # Initialize publishers with QoS profile
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 10)

        # Initialize subscribers with QoS profile
        self.subscription = self.create_subscription(Joy, "joy", self.joy_callback, 10)

        # Initialize state variables
        self.last_twist = Twist()
        self.get_logger().info("Xbox teleop node initialized")

    def joy_callback(self, joy_msg: Joy) -> None:
        """Process joystick messages and publish Twist commands."""
        try:
            twist = Twist()

            # Check for valid message
            if len(joy_msg.axes) <= self.deadman_trigger_axis:
                self.get_logger().warning("Invalid joy message: not enough axes")
                return

            # Initialize inputs
            forward_input = 0.0
            sideways_input = 0.0
            direction_reversed = False

            # Check deadman switch (right trigger)
            dead_man_switch = joy_msg.axes[self.deadman_trigger_axis] == -1.0

            if dead_man_switch:
                # Get forward/backward input (left stick Y-axis)
                if len(joy_msg.axes) > 1:
                    forward_input = joy_msg.axes[1]

                # Get left/right input (left stick X-axis)
                if len(joy_msg.axes) > 2:
                    sideways_input = joy_msg.axes[2]

                # Reverse left/right direction when moving forward
                if forward_input >= 0:
                    sideways_input = -sideways_input
                    direction_reversed = True

                # Calculate velocities
                linear_velocity = math.sqrt(forward_input**2 + sideways_input**2)

                # Set linear velocity with direction and speed limit
                twist.linear.x = math.copysign(
                    min(linear_velocity * self.max_linear_speed, self.max_linear_speed),
                    forward_input,
                )

                # Calculate angular velocity
                if linear_velocity != 0:
                    angular_velocity = math.atan2(-sideways_input, abs(forward_input))
                    twist.angular.z = angular_velocity * self.max_angular_speed
                else:
                    twist.angular.z = 0.0

            # Log controller state if debug logging is enabled
            if self.get_logger().get_effective_level() <= 10:  # DEBUG level
                self.log_controller_state(joy_msg, twist, dead_man_switch, direction_reversed)

            # Publish command
            self.publisher_.publish(twist)
            self.last_twist = twist

        except Exception as e:
            self.get_logger().error(f"Error in joy_callback: {str(e)}")
            # Publish zero velocity in case of error
            self.publisher_.publish(Twist())

    def log_controller_state(
        self, joy_msg: Joy, twist: Twist, dead_man_switch: bool, direction_reversed: bool
    ) -> None:
        """Log detailed controller state information."""
        self.get_logger().debug("Xbox Controller Axes:")
        for i, axis in enumerate(joy_msg.axes):
            self.get_logger().debug(f"  Axis {i}: {axis:.2f}")

        self.get_logger().debug("Twist Output:")
        self.get_logger().debug(f"  Linear X: {twist.linear.x:.2f} m/s")
        self.get_logger().debug(f"  Angular Z: {twist.angular.z:.2f} rad/s")
        self.get_logger().debug(
            f'  Dead Man\'s Switch: {"Engaged" if dead_man_switch else "Disengaged"}'
        )
        self.get_logger().debug(f'  Direction Reversed: {"Yes" if direction_reversed else "No"}')
        self.get_logger().debug("------------------------")


def main(args=None):
    rclpy.init(args=args)
    try:
        xbox_teleop_node = XboxTeleopNode()
        rclpy.spin(xbox_teleop_node)
    except Exception as e:
        print(f"Error in main: {str(e)}")
    finally:
        if "xbox_teleop_node" in locals():
            xbox_teleop_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
