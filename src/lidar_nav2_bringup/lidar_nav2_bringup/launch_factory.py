"""Shared node assembly; YAML owns parameters and launch owns process boundaries."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def share(package):
    return Path(get_package_share_directory(package))


def component(package, executable, params, name=None, **kwargs):
    return Node(package=package, executable=executable, name=name,
                parameters=params, output='screen', **kwargs)


def perception_nodes(bridge, perception):
    return [component('lidar_nav2_bringup', 'lio_nav_bridge', bridge, 'lio_nav_bridge'),
            component('lidar_perception', 'perception_node', perception, 'lidar_perception')]


def system(context):
    path = Path(LaunchConfiguration('profile').perform(context)).expanduser()
    cfg = yaml.safe_load(path.read_text())
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in ['mapping', 'planning']:
        raise ValueError('mode must be mapping or planning')
    sim = LaunchConfiguration('source').perform(context) == 'bag'
    if LaunchConfiguration('source').perform(context) not in ['bag', 'sensor']:
        raise ValueError('source must be bag or sensor')
    overrides = {'use_sim_time': sim}
    def file(key, package):
        value = Path(cfg[key]).expanduser()
        return str(value if value.is_absolute() else share(package)/value)
    actions = []
    if LaunchConfiguration('frontend').perform(context).lower() == 'true':
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share('fast_lio')/'launch/mapping.launch.py')),
            launch_arguments={'config_path': str(share('fast_lio')/'config'),
                              'config_file': 'mid360.yaml', 'use_sim_time': str(sim).lower(),
                              'rviz': 'false'}.items()))
    if not sim:
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(share('livox_ros_driver2')/'launch_ROS2/msg_MID360_launch.py'))))
    actions.extend(perception_nodes([file('bridge', 'lidar_nav2_bringup'), overrides],
                                   [file('perception', 'lidar_perception'), overrides]))
    map_yaml = LaunchConfiguration('map').perform(context)
    if mode == 'planning' and map_yaml and not sim:
        raise ValueError('Saved odom map requires verified session alignment; sensor saved-map mode needs a localization backend')
    managed = []
    if mode == 'planning' and map_yaml:
        actions.append(component('nav2_map_server', 'map_server',
            [overrides, {'yaml_filename': str(Path(map_yaml).expanduser()), 'frame_id': 'odom'}], 'map_server'))
        # 地图服务器先独立激活；规划门控等到地图发布后再启动
        actions.append(component('nav2_lifecycle_manager', 'lifecycle_manager',
            [overrides, {'autostart': True, 'bond_timeout': 0.0, 'node_names': ['map_server']}],
            'lifecycle_manager_map'))
    else:
        actions.append(component('lidar_mapping', 'mapping_node',
            [file('mapping', 'lidar_mapping'), overrides,
             {'output_prefix': LaunchConfiguration('output').perform(context)}], 'lidar_mapping'))
    if mode == 'planning':
        actions.append(component('nav2_planner', 'planner_server',
            [file('planner', 'lidar_nav2_bringup'), overrides], 'planner_server'))
        actions.append(component('lidar_nav2_bringup', 'local_costmap_server',
            [file('local_costmap', 'lidar_nav2_bringup'), overrides],
            'local_costmap', namespace='local_costmap'))
        managed = ['planner_server', 'local_costmap/local_costmap']
        actions.append(component('nav2_lifecycle_manager', 'lifecycle_manager',
            [overrides, {'autostart': False, 'bond_timeout': 0.0, 'node_names': managed}],
            'lifecycle_manager_navigation'))
        actions.append(component('lidar_nav2_bringup', 'nav2_startup_gate',
            [file('bringup', 'lidar_nav2_bringup'), overrides,
             {'require_map': True, 'manager_service': '/lifecycle_manager_navigation/manage_nodes'}],
            'nav2_startup_gate'))
    if LaunchConfiguration('rviz').perform(context).lower() == 'true':
        actions.append(component('rviz2', 'rviz2', [overrides], 'rviz_navigation',
                                 arguments=['-d', str(share('lidar_nav2_bringup')/'rviz/session_mapping.rviz')]))
    return actions


def full_system():
    navigation = share('lidar_nav2_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='mapping', description='mapping or planning'),
        DeclareLaunchArgument('source', default_value='bag', description='bag or sensor'),
        DeclareLaunchArgument('profile', default_value=str(navigation/'config/system.yaml')),
        DeclareLaunchArgument('frontend', default_value='true', description='false when FAST-LIO runs externally'),
        DeclareLaunchArgument('map', default_value='', description='saved map in the same odom session only'),
        DeclareLaunchArgument('output', default_value='/tmp/lidar_session_map'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=system),
    ])


def legacy_navigation(context, combined=False):
    navigation = share('lidar_nav2_bringup')
    def value(key):
        return LaunchConfiguration(key).perform(context)
    actions = [component('lidar_nav2_bringup', 'lio_nav_bridge', [value('bridge_params')], 'lio_nav_bridge')]
    if combined:
        actions.append(component('lidar_perception', 'perception_node', [value('perception_params')], 'lidar_perception'))
    actions.extend([
        component('nav2_planner', 'planner_server', [value('planner_params')], 'planner_server'),
        component('nav2_lifecycle_manager', 'lifecycle_manager', [value('bringup_params')], 'lifecycle_manager_planner'),
        component('lidar_nav2_bringup', 'nav2_startup_gate', [value('bringup_params')], 'nav2_startup_gate'),
    ])
    if value('rviz').lower() == 'true':
        view = 'ground_navigation.rviz' if combined else 'navigation.rviz'
        actions.append(component('rviz2', 'rviz2', [value('bringup_params')], 'rviz_navigation',
                                 arguments=['-d', str(navigation/'rviz'/view)]))
    return actions


def navigation_component(combined=False):
    navigation = share('lidar_nav2_bringup')
    defaults = {
        'bridge_params': str(navigation/'config/bridge.yaml'),
        'planner_params': str(navigation/'config'/('bag_planner_ground.yaml' if combined else 'bag_planner.yaml')),
        'bringup_params': str(navigation/'config/bringup.yaml'),
        'rviz': 'true' if combined else 'false',
    }
    if combined:
        defaults['perception_params'] = str(share('lidar_perception')/'config/perception.yaml')
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k,v in defaults.items()]+
                             [OpaqueFunction(function=lambda context: legacy_navigation(context, combined))])
