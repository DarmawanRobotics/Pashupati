#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Start robot_driver_node."""
    pkg_share = get_package_share_directory('robot_drivers')
    params_yaml = os.path.join(pkg_share, 'config', 'sdk_brigde_param.yaml')

    return LaunchDescription([
        Node(
            package='robot_drivers',
            executable='robot_driver_node',
            name='robot_driver_node',
            output='screen',
            parameters=[params_yaml],
        ),
    ])