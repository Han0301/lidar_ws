"""Start perception and ground-evidence planning with one shared RViz view."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# 一体启动感知、桥接和地面证据规划地图，只打开一个 RViz 窗口
def generate_launch_description():
    navigation = Path(get_package_share_directory('lidar_nav2_bringup')) # 导航包的安装目录，提供地面证据配置和组合视图
    perception = Path(get_package_share_directory('lidar_perception')) # 感知包的安装目录，复用其参数与独立入口
    # 默认选择地面证据候选地图，所有处理参数继续由 YAML 管理
    return LaunchDescription([
        DeclareLaunchArgument('perception_params',
                              default_value=str(perception / 'config/perception.yaml')),
        DeclareLaunchArgument('bridge_params',
                              default_value=str(navigation / 'config/bridge.yaml')),
        DeclareLaunchArgument('planner_params',
                              default_value=str(navigation / 'config/bag_planner_ground.yaml')),
        DeclareLaunchArgument('bringup_params',
                              default_value=str(navigation / 'config/bringup.yaml')),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open the ground, obstacle and costmap view'),
        # 隔离子 launch 的参数作用域，避免它的 rviz=false 覆盖顶层窗口开关
        GroupAction([
            IncludeLaunchDescription(
                # 复用感知入口，关闭它的独立窗口
                PythonLaunchDescriptionSource(str(perception / 'launch/perception.launch.py')),
                launch_arguments={
                    'params': LaunchConfiguration('perception_params'),
                    'rviz': 'false',
                }.items()),
        ]),
        # 隔离子 launch 的参数作用域，避免它的 rviz=false 覆盖顶层窗口开关
        GroupAction([
            IncludeLaunchDescription(
                # 复用带数据就绪门控的导航入口，传入地面证据地图配置
                PythonLaunchDescriptionSource(str(navigation / 'launch/nav2.launch.py')),
                launch_arguments={
                    'bridge_params': LaunchConfiguration('bridge_params'),
                    'planner_params': LaunchConfiguration('planner_params'),
                    'bringup_params': LaunchConfiguration('bringup_params'),
                    'rviz': 'false',
                }.items()),
        ]),
        # 统一显示地面、障碍、代价地图和轨迹，避免打开两个重复界面
        Node(package='rviz2', executable='rviz2', name='rviz_navigation',
             parameters=[LaunchConfiguration('bringup_params')],
             arguments=['-d', str(navigation / 'rviz/ground_navigation.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
    ])
