#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Darby Lim

# edited by Dingo July 2024 to support RPLidar a1 instead of HLDS_laser.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():

    # TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    turtlebot3_model = EnvironmentVariable("TURTLEBOT3_MODEL", default_value="burger")

    # set the ROS_DOMAIN_ID
    ros_domain_id = "30"

    LDS_MODEL = os.environ.get("LDS_MODEL", "A1")
    #    LDS_LAUNCH_FILE = '/hlds_laser.launch.py'
    LDS_LAUNCH_FILE = "/hlds_laser.launch.py"

    usb_port = LaunchConfiguration(
        "usb_port", default="/dev/ttyACM0"
    )  # This is for the OpenCR, not lidar
    #    usb_port = LaunchConfiguration('usb_port', default='/dev/ttyUSB0')

    tb3_param_dir = LaunchConfiguration(
        "tb3_param_dir",
        default=os.path.join(
            get_package_share_directory("turtlebot3_bringup"),
            "param",
            #            TURTLEBOT3_MODEL + '.yaml'))
            "burger" + ".yaml",
        ),
    )

    #    if LDS_MODEL == 'LDS-01':
    #        lidar_pkg_dir = LaunchConfiguration(
    #            'lidar_pkg_dir',
    #            default=os.path.join(get_package_share_directory('hls_lfcd_lds_driver'), 'launch'))
    if LDS_MODEL == "A1":
        lidar_pkg_dir = LaunchConfiguration(
            "lidar_pkg_dir",
            default=os.path.join(get_package_share_directory("sllidar_ros2"), "launch"),
        )
        LDS_LAUNCH_FILE = "/sllidar_a1_launch.py"
    elif LDS_MODEL == "LDS-02":
        lidar_pkg_dir = LaunchConfiguration(
            "lidar_pkg_dir",
            default=os.path.join(get_package_share_directory("ld08_driver"), "launch"),
        )
        LDS_LAUNCH_FILE = "/ld08.launch.py"
    else:
        lidar_pkg_dir = LaunchConfiguration(
            "lidar_pkg_dir",
            default=os.path.join(get_package_share_directory("sllidar_ros2"), "launch"),
        )
        LDS_LAUNCH_FILE = "/sllidar_a1_launch.py"
        # lidar_pkg_dir = LaunchConfiguration(
        #    'lidar_pkg_dir',
        #    default=os.path.join(get_package_share_directory('hls_lfcd_lds_driver'), 'launch'))

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    return LaunchDescription(
        [
            # Set TURTLEBOT3_MODEL environment variable if not already set
            SetEnvironmentVariable("TURTLEBOT3_MODEL", turtlebot3_model),
            # Set ROS_DOMAIN_ID to 30
            SetEnvironmentVariable("ROS_DOMAIN_ID", ros_domain_id),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value=use_sim_time,
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                "usb_port", default_value=usb_port, description="Connected USB port with OpenCR"
            ),
            DeclareLaunchArgument(
                "tb3_param_dir",
                default_value=tb3_param_dir,
                description="Full path to turtlebot3 parameter file to load",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [ThisLaunchFileDir(), "/turtlebot3_state_publisher.launch.py"]
                ),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([lidar_pkg_dir, LDS_LAUNCH_FILE]),
                launch_arguments={"port": "/dev/ttyUSB0", "frame_id": "base_scan"}.items(),
            ),
            Node(
                package="turtlebot3_node",
                executable="turtlebot3_ros",
                parameters=[tb3_param_dir],
                arguments=["-i", usb_port],
                output="screen",
            ),
            # Add the WiFi shutdown monitor node
            Node(
                package="wifi_shutdown_monitor",
                executable="wifi_monitor",
                name="wifi_shutdown_monitor",
                output="screen",
            ),
        ]
    )
