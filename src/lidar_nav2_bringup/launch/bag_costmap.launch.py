from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    p = Path(get_package_share_directory('lidar_nav2_bringup'))
    return LaunchDescription([
        Node(package='nav2_costmap_2d', executable='nav2_costmap_2d', name='local_costmap', namespace='local_costmap', parameters=[str(p / 'config/bag_costmap.yaml')], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_costmap', parameters=[{'use_sim_time': True, 'autostart': False, 'bond_timeout': 0.0, 'node_names': ['local_costmap/local_costmap']}], output='screen'),
    ])
