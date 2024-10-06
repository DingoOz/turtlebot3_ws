from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='depthai_ros_driver',
            executable='camera_node',
            name='oak_d_lite',
            parameters=[{
                'camera_name': 'oak_d_lite',
                'frame_name': 'oak_d_lite_frame',
                'imu_mode': '0',
                'rgb_resolution': '1080p',
                'fps': 30,
                'enable_depth': True,
                'enable_rgb': True,
                'enable_imu': False,
                'enable_pointcloud': True,
                'sync_outputs': True,
            }],
            output='screen',
        )
    ])
