from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Set environment variables
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', '30'),

        # Launch Turtlebot3 robot
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('turtlebot3_bringup'),
                    'launch',
                    'dingo.robot.launch.py'
                ])
            ])
        ),

        # Launch OAK-D Lite
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('oakd_lite_turtlebot3'),
                    'launch',
                    'oakd_lite.launch.py'
                ])
            ])
        ),

        # Launch Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'bringup_launch.py'
                ])
            ]),
            launch_arguments={
                'params_file': PathJoinSubstitution([
                    FindPackageShare('oakd_lite_turtlebot3'),
                    'config',
                    'nav2_params.yaml'
                ]),
                'use_sim_time': 'false'
            }.items()
        )
    ])

