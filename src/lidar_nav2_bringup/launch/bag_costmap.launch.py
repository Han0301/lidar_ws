from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

# 启动独立局部代价地图；生命周期由外部录包评估门控脚本激活
def generate_launch_description():
    p = Path(get_package_share_directory('lidar_nav2_bringup')) # 录包局部地图参数所在的安装目录
    return LaunchDescription([
        # 在 local_costmap 名字空间运行独立地图节点
        Node(package='nav2_costmap_2d', executable='nav2_costmap_2d', name='local_costmap', namespace='local_costmap', parameters=[str(p / 'config/bag_costmap.yaml')], output='screen'),
        # 不立即激活；独立地图没有父服务器 bond，关闭 bond 等待
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_costmap', parameters=[{'use_sim_time': True, 'autostart': False, 'bond_timeout': 0.0, 'node_names': ['local_costmap/local_costmap']}], output='screen'),
    ])
