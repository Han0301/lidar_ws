"""Start the perception node; navigation supplies its TF reference."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# 组装感知节点及可选 RViz；调平 TF 由外部导航桥接节点提供
def generate_launch_description():
    share = Path(get_package_share_directory('lidar_perception')) # 感知包的安装资源目录，参数和 RViz 从这里加载
    params = LaunchConfiguration('params') # 用户可替换的参数文件，节点参数仍保存在 YAML 中
    # 声明参数路径和界面开关，再启动实际处理节点
    return LaunchDescription([
        DeclareLaunchArgument('params', default_value=str(share / 'config/perception.yaml')),
        DeclareLaunchArgument('rviz', default_value='false'),
        # 启动独立感知处理，不在这里启动 FAST-LIO2 或导航桥接
        Node(package='lidar_perception', executable='perception_node',
             name='lidar_perception', parameters=[params], output='screen'),
        # 按开关打开地面/障碍点云视图，并读取对应节点的仿真时间参数
        Node(package='rviz2', executable='rviz2', name='rviz_perception', parameters=[params],
             arguments=['-d', str(share / 'rviz/perception.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
    ])
