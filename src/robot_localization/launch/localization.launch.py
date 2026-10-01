#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the localization node."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'tags_config_file',
            default_value='',
            description='JSON file with tag poses in the map frame.',
        ),
        Node(
            package='robot_localization',
            executable='localization_node',
            name='localization_node',
            output='screen',
            parameters=[{'tags_config_file': LaunchConfiguration('tags_config_file')}],
        ),
    ])
