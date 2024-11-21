from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # Print initial message
            ExecuteProcess(
                cmd=["echo", "\n\033[1;32mTurn on your Xbox controller now\033[0m\n"],
                output="screen",
            ),
            # Start joy node
            Node(package="joy", executable="joy_node", name="joy_node", output="screen"),
            # Start Xbox teleop node
            Node(
                package="xbox_turtlebot3_teleop",
                executable="xbox_teleop",
                name="xbox_teleop_node",
                output="screen",
            ),
        ]
    )
