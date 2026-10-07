from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml
from pathlib import Path

def generate_launch_description():
    p = Path(get_package_share_directory('lidar_nav2_bringup'))
    namespace = LaunchConfiguration('namespace')
    configured = RewrittenYaml(source_file=LaunchConfiguration('params'), root_key=namespace, param_rewrites={}, convert_types=True)
    return LaunchDescription([
        DeclareLaunchArgument('params', default_value=str(p / 'config/bag_planner.yaml')),
        DeclareLaunchArgument('namespace', default_value=''),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', namespace=namespace, parameters=[configured], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_planner', namespace=namespace, parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': ['planner_server']}], output='screen'),
    ])
