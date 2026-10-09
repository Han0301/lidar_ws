"""Public entry: one LIO/perception pipeline and either mapping or planning."""
from lidar_nav2_bringup.launch_factory import full_system


def generate_launch_description():
    return full_system()
