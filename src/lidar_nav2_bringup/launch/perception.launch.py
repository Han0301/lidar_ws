"""Legacy bridge + perception component, with externally managed FAST-LIO."""
from launch import LaunchDescription
from lidar_nav2_bringup.launch_factory import share, perception_nodes


def generate_launch_description():
    return LaunchDescription(perception_nodes(
        [str(share('lidar_nav2_bringup')/'config/bridge.yaml')],
        [str(share('lidar_perception')/'config/perception.yaml')]))
