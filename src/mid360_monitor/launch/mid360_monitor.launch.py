from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('mid360_monitor'),
        'config',
        'mid360_monitor.yaml',
    )

    return LaunchDescription([
        Node(
            package='mid360_monitor',
            executable='mid360_monitor_node',
            name='mid360_monitor_node',
            output='screen',
            parameters=[config_path],
        ),
    ])
