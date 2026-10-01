#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the command-center uplink."""
    share = get_package_share_directory('robot_fleet')
    params = os.path.join(share, 'config', 'fleet_params.yaml')
    robot_ns = LaunchConfiguration('robot_ns')
    return LaunchDescription(
        [
            DeclareLaunchArgument('params_file', default_value=params),
            DeclareLaunchArgument('robot_ns', default_value='l1w'),
            DeclareLaunchArgument('server_url', default_value=''),
            DeclareLaunchArgument('token', default_value=''),
            Node(
                package='robot_fleet',
                executable='fleet_uplink_node',
                name='fleet_uplink_node',
                output='screen',
                parameters=[
                    LaunchConfiguration('params_file'),
                    {
                        'server_url': LaunchConfiguration('server_url'),
                        'token': LaunchConfiguration('token'),
                    },
                ],
                remappings=[
                    ('battery', ['/', robot_ns, '/battery']),
                    ('diagnostics', ['/', robot_ns, '/diagnostics']),
                ],
            ),
        ]
    )
