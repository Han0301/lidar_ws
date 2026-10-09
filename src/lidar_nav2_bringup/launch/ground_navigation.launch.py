"""Compatibility component entry; parameter defaults and ownership preserved."""
from lidar_nav2_bringup.launch_factory import navigation_component


def generate_launch_description():
    return navigation_component(True)
