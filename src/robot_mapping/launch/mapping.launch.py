#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch odometry (robot_odometry), the map->base_link frame bridge and path recording."""
    params_file = os.path.join(
        get_package_share_directory('robot_mapping'), 'config', 'mapping_param.yaml'
    )
    odometry_launch = os.path.join(
        get_package_share_directory('robot_odometry'), 'launch', 'odometry.launch.py'
    )
    localization_launch = os.path.join(
        get_package_share_directory('robot_localization'), 'launch', 'localization.launch.py'
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'record', default_value='true', description='Enable path recording.'
            ),
            DeclareLaunchArgument('params_file', default_value=params_file),
            DeclareLaunchArgument(
                'frame_bridge',
                default_value='true',
                description='Run localization_node so map->base_link exists.',
            ),
            DeclareLaunchArgument(
                'tags_config_file',
                default_value='',
                description='AprilTag poses JSON passed to localization_node.',
            ),
            DeclareLaunchArgument(
                'save_map',
                default_value='true',
                description='Accumulate and save the PCD map. Use false while patrolling: '
                'FAST-LIO keeps every scan in RAM until shutdown.',
            ),
            DeclareLaunchArgument(
                'wait_still',
                default_value='true',
                description='Start FAST-LIO only once the robot stands still.',
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(odometry_launch),
                launch_arguments={
                    'save_map': LaunchConfiguration('save_map'),
                    'wait_still': LaunchConfiguration('wait_still'),
                }.items(),
            ),
            Node(
                package='robot_mapping',
                executable='path_recorder_node',
                name='path_recorder_node',
                output='screen',
                parameters=[LaunchConfiguration('params_file')],
                condition=IfCondition(LaunchConfiguration('record')),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(localization_launch),
                launch_arguments={
                    'tags_config_file': LaunchConfiguration('tags_config_file')
                }.items(),
                condition=IfCondition(LaunchConfiguration('frame_bridge')),
            ),
        ]
    )
