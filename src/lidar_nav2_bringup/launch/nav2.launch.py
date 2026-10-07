"""Start LIO frame adaptation and Nav2 planning on recorded observations."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# 组装 LIO 坐标桥接、规划地图、启动门控及可选导航视图
def generate_launch_description():
    share = Path(get_package_share_directory('lidar_nav2_bringup')) # 导航包的安装资源目录
    bringup = LaunchConfiguration('bringup_params') # 管理器、启动门控和 RViz 共用的 YAML 配置路径
    # 暴露配置文件路径，默认采用原生体素规划地图
    return LaunchDescription([
        DeclareLaunchArgument('bridge_params', default_value=str(share / 'config/bridge.yaml')),
        DeclareLaunchArgument('planner_params', default_value=str(share / 'config/bag_planner.yaml')),
        DeclareLaunchArgument('bringup_params', default_value=str(share / 'config/bringup.yaml')),
        DeclareLaunchArgument('rviz', default_value='false'),
        # 将 LIO 位姿接到导航 TF，建立平面参考及真实传感器原点
        Node(package='lidar_nav2_bringup', executable='lio_nav_bridge',
             name='lio_nav_bridge', parameters=[LaunchConfiguration('bridge_params')],
             output='screen'),
        # 规划服务器内部创建 global_costmap，按指定 YAML 加载地图层
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             parameters=[LaunchConfiguration('planner_params')], output='screen'),
        # 建立生命周期管理服务，关闭立即自动激活，由门控决定启动时刻
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_planner', parameters=[bringup], output='screen'),
        # 等待有效时钟、位姿与 TF 后请求激活，避免录包开始时的时间跳变
        Node(package='lidar_nav2_bringup', executable='nav2_startup_gate',
             name='nav2_startup_gate', parameters=[bringup], output='screen'),
        # 按开关打开代价地图、规划路径和 LIO 轨迹视图
        Node(package='rviz2', executable='rviz2', name='rviz_navigation', parameters=[bringup],
             arguments=['-d', str(share / 'rviz/navigation.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
    ])
