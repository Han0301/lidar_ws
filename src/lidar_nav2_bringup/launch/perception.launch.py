from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

# 兼容旧入口：只启动桥接和感知，不包含 Nav2 地图或规划器
def generate_launch_description():
    perception = Path(get_package_share_directory('lidar_perception'))      # 感知参数的安装位置
    navigation = Path(get_package_share_directory('lidar_nav2_bringup'))      # 桥接参数的安装位置
    # 两个节点分别读取各自 YAML，FAST-LIO2 仍由外部启动
    return LaunchDescription([
        # 先声明导航坐标桥接节点，感知点云处理依赖它提供的 TF
        Node(package='lidar_nav2_bringup', executable='lio_nav_bridge', parameters=[str(navigation / 'config/bridge.yaml')], output='screen'),
        # 声明感知节点；回调会等待同一扫描时刻 TF 后再处理
        Node(package='lidar_perception', executable='perception_node', name='lidar_perception', parameters=[str(perception / 'config/perception.yaml')], output='screen'),
    ])
