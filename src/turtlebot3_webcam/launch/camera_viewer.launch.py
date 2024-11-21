from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
            SetEnvironmentVariable("ROS_DOMAIN_ID", "30"),
            SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "0"),
            Node(
                package="turtlebot3_webcam",
                executable="camera_viewer",
                name="camera_viewer",
                output="screen",
                emulate_tty=True,
                parameters=[{"use_sim_time": False}],
            ),
        ]
    )
