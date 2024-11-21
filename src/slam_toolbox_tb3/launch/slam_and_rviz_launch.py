import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    slam_toolbox_tb3_dir = get_package_share_directory("slam_toolbox_tb3")

    # Include the SLAM launch file
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([slam_toolbox_tb3_dir, "/launch/slam_toolbox_tb3_launch.py"])
    )

    # Start RViz
    rviz_config_dir = os.path.join(
        get_package_share_directory("turtlebot3_cartographer"), "rviz", "tb3_cartographer.rviz"
    )  # set up Rviz2 based on cartographer
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_dir],
        output="screen",
    )

    return LaunchDescription([slam_launch, rviz_node])
