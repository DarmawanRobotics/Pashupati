#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch FAST-LIO mapping with optional path recording."""
    package_dir = get_package_share_directory('robot_mapping')
    params_file = os.path.join(package_dir, 'config', 'mapping_param.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'record',
            default_value='true',
            description='Enable path recording.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='Path to mapping parameter file.',
        ),

        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            name='fastlio_mapping_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),

        Node(
            package='robot_mapping',
            executable='path_recorder_node',
            name='path_recorder_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
            condition=IfCondition(LaunchConfiguration('record')),
        ),
    ])