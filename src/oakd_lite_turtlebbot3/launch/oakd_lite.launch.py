from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='depthai_ros_driver',
            executable='depthai_ros_driver_node',
            name='oak_d_lite',
            parameters=[{
                'camera_name': 'oak_d_lite',
                'frame_name': 'oak_d_lite_frame',
                'imu_mode': '0',
                'rgb_resolution': '1080p',
                'fps': '30',
                'enable_depth': 'true',
                'enable_rgb': 'true',
                'enable_imu': 'false',
                'enable_pointcloud': 'true',
                'sync_outputs': 'true',
            }],
            output='screen',
        )
    ])

