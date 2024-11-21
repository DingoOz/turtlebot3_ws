from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Find the path to nav2_bringup and to the parameter file
    bringup_dir = FindPackageShare("nav2_bringup")
    param_file = PathJoinSubstitution(
        [
            FindPackageShare(
                "turtlebot3_navigation2"
            ),  # Replace 'your_package_name' with the actual package containing burger.yaml
            "param",
            "burger.yaml",
        ]
    )

    # Launch Navigation2 bringup
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_dir, "launch", "bringup_launch.py"])
        ),
        launch_arguments={"use_sim_time": "false", "params_file": param_file}.items(),
    )

    return LaunchDescription([nav2_bringup])
