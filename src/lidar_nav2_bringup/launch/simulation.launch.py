from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    p = Path(get_package_share_directory('lidar_nav2_bringup'))
    params = LaunchConfiguration('params')
    servers = [('nav2_map_server', 'map_server'), ('nav2_controller', 'controller_server'), ('nav2_planner', 'planner_server'), ('nav2_behaviors', 'behavior_server'), ('nav2_bt_navigator', 'bt_navigator')]
    actions = [DeclareLaunchArgument('map', default_value=str(p / 'maps/validation.yaml')), DeclareLaunchArgument('params', default_value=str(p / 'config/nav2_sim.yaml'))]
    for package, executable in servers:
        settings = [params]
        if executable == 'map_server':
            settings.append({'yaml_filename': LaunchConfiguration('map')})
        actions.append(Node(package=package, executable=executable, name=executable, parameters=settings, output='screen'))
    actions.append(Node(package='nav2_loopback_sim', executable='loopback_simulator', name='loopback_simulator', parameters=[params], output='screen'))
    actions.append(Node(package='tf2_ros', executable='static_transform_publisher', arguments=['--x', '0', '--y', '0', '--z', '0.30', '--frame-id', 'nav_base', '--child-frame-id', 'nav_sensor']))
    actions.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation', parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': [exe for _, exe in servers]}], output='screen'))
    return LaunchDescription(actions)
