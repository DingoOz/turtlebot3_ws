from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "max_linear_speed", default_value="0.5", description="Maximum linear speed in m/s"
            ),
            DeclareLaunchArgument(
                "max_angular_speed",
                default_value="1.5",
                description="Maximum angular speed in rad/s",
            ),
            DeclareLaunchArgument(
                "deadman_trigger_axis",
                default_value="5",
                description="Axis number for deadman trigger",
            ),
            Node(
                package="xbox_turtlebot3_teleop",
                executable="xbox_teleop_node",
                name="xbox_teleop_node",
                parameters=[
                    {
                        "max_linear_speed": LaunchConfiguration("max_linear_speed"),
                        "max_angular_speed": LaunchConfiguration("max_angular_speed"),
                        "deadman_trigger_axis": LaunchConfiguration("deadman_trigger_axis"),
                    }
                ],
                output="screen",
            ),
        ]
    )
