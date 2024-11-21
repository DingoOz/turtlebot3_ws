from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Include the robot launch file
    # turtlebot3_bringup_dir = get_package_share_directory('turtlebot3_bringup')
    # robot_launch = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource([turtlebot3_bringup_dir, '/launch/dingo.robot.launch.py'])
    # )

    # Include the SLAM Toolbox launch file
    slam_toolbox_tb3_dir = get_package_share_directory("slam_toolbox_tb3")
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([slam_toolbox_tb3_dir, "/launch/slam_toolbox_launch.py"])
    )

    return LaunchDescription(
        [
            # robot_launch,
            slam_launch,
        ]
    )
