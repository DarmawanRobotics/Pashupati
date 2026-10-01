#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Launch FAST-LIO, the map->base_link frame bridge, and optional path recording."""
    params_file = os.path.join(
        get_package_share_directory('robot_mapping'), 'config', 'mapping_param.yaml'
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
            Node(
                package='fast_lio',
                executable='fastlio_mapping',
                name='fastlio_mapping_node',
                output='screen',
                parameters=[
                    LaunchConfiguration('params_file'),
                    {
                        'pcd_save.pcd_save_en': ParameterValue(
                            LaunchConfiguration('save_map'), value_type=bool
                        )
                    },
                ],
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
