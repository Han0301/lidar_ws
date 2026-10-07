from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    p = Path(get_package_share_directory('lidar_nav2_bringup'))
    return LaunchDescription([
        Node(package='nav2_planner', executable='planner_server', name='planner_server', parameters=[str(p / 'config/bag_planner.yaml')], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_planner', parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': ['planner_server']}], output='screen'),
    ])
