from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wifi_shutdown_monitor',
            executable='wifi_monitor',
            name='wifi_monitor_node',
            output='screen',
        ),
    ])
    
