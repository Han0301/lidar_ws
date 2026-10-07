from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

# 启动理想运动仿真中的完整 Nav2 规划、控制和行为闭环
def generate_launch_description():
    p = Path(get_package_share_directory('lidar_nav2_bringup')) # 仿真地图与导航参数的安装目录
    params = LaunchConfiguration('params') # 包含各个 Nav2 节点配置的 YAML 文件
    servers = [('nav2_map_server', 'map_server'), ('nav2_controller', 'controller_server'), ('nav2_planner', 'planner_server'), ('nav2_behaviors', 'behavior_server'), ('nav2_bt_navigator', 'bt_navigator')] # 需要生命周期管理的标准导航服务器列表
    actions = [DeclareLaunchArgument('map', default_value=str(p / 'maps/validation.yaml')), DeclareLaunchArgument('params', default_value=str(p / 'config/nav2_sim.yaml'))] # 按顺序收集启动参数声明和实际节点启动动作
    # 为每个标准服务器创建节点，保持配置集中在 YAML 中
    for package, executable in servers:
        settings = [params] # 当前服务器的参数文件列表；地图服务器额外接收地图路径
        # 把用户选择的二维地图文件传给地图服务器
        if executable == 'map_server':
            settings.append({'yaml_filename': LaunchConfiguration('map')})
        actions.append(Node(package=package, executable=executable, name=executable, parameters=settings, output='screen'))
    # 理想仿真器根据速度请求产生运动反馈与传感器数据，不能代表真实底盘
    actions.append(Node(package='nav2_loopback_sim', executable='loopback_simulator', name='loopback_simulator', parameters=[params], output='screen'))
    # 定义仿真传感器相对虚拟底盘的固定高度（m）
    actions.append(Node(package='tf2_ros', executable='static_transform_publisher', arguments=['--x', '0', '--y', '0', '--z', '0.30', '--frame-id', 'nav_base', '--child-frame-id', 'nav_sensor']))
    # 统一配置并激活完整仿真导航服务器
    actions.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation', parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': [exe for _, exe in servers]}], output='screen'))
    # 把全部声明和节点交给 ROS launch 执行
    return LaunchDescription(actions)
