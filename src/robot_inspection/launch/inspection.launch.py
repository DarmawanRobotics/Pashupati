#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the VLM anomaly detector."""
    share = get_package_share_directory('robot_inspection')
    params = os.path.join(share, 'config', 'inspection_params.yaml')
    return LaunchDescription(
        [
            DeclareLaunchArgument('params_file', default_value=params),
            Node(
                package='robot_inspection',
                executable='anomaly_detector_node',
                name='anomaly_detector_node',
                output='screen',
                parameters=[LaunchConfiguration('params_file')],
            ),
        ]
    )
