from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    perception = Path(get_package_share_directory('lidar_perception'))
    navigation = Path(get_package_share_directory('lidar_nav2_bringup'))
    return LaunchDescription([
        Node(package='lidar_nav2_bringup', executable='lio_nav_bridge', parameters=[str(navigation / 'config/bridge.yaml')], output='screen'),
        Node(package='lidar_perception', executable='perception_node', name='lidar_perception', parameters=[str(perception / 'config/perception.yaml')], output='screen'),
    ])
