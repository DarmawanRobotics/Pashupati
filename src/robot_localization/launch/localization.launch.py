#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_tag_file = '/home/robot/dev/Pashupati/map/example/tag_config.json'

    return LaunchDescription([
        DeclareLaunchArgument(
            'tag_file',
            default_value=default_tag_file,
            description='Path to the tag configuration file for the localization node.',
        ),
        Node(
            package='robot_localization',
            executable='localization_node',
            name='localization_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
            ],
        ),
    ])