from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml
from pathlib import Path

# 启动录包规划服务器及生命周期管理器，依赖评估工具在输入就绪后调用
def generate_launch_description():
    p = Path(get_package_share_directory('lidar_nav2_bringup')) # 规划地图参数的安装目录
    namespace = LaunchConfiguration('namespace') # 可选名字空间，用于同时运行多个规划器进行对照
    configured = RewrittenYaml(source_file=LaunchConfiguration('params'), root_key=namespace, param_rewrites={}, convert_types=True) # 在指定名字空间下重写 YAML 根节点，使参数匹配实际节点
    # 声明地图配置和名字空间，再启动规划服务
    return LaunchDescription([
        DeclareLaunchArgument('params', default_value=str(p / 'config/bag_planner.yaml')),
        DeclareLaunchArgument('namespace', default_value=''),
        # 每个规划器拥有独立代价地图，不输出底盘速度
        Node(package='nav2_planner', executable='planner_server', name='planner_server', namespace=namespace, parameters=[configured], output='screen'),
        # 评估工具已等待输入就绪，该入口按标准 bond 管理并自动激活规划器
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_planner', namespace=namespace, parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': ['planner_server']}], output='screen'),
    ])
