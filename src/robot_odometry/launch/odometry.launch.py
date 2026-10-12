#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Launch FAST-LIO (held until the IMU is still) and the odometry monitor."""
    share = get_package_share_directory('robot_odometry')
    prefix = get_package_prefix('robot_odometry')
    gate = os.path.join(prefix, 'lib', 'robot_odometry', 'wait_still')
    params = LaunchConfiguration('params_file')
    respawn = LaunchConfiguration('respawn')
    fast_lio_params = [
        LaunchConfiguration('fast_lio_params_file'),
        {
            'pcd_save.pcd_save_en': ParameterValue(
                LaunchConfiguration('save_map'), value_type=bool
            )
        },
    ]

    def fast_lio(condition, prefix=None):
        return Node(
            package='fast_lio',
            executable='fastlio_mapping',
            name='fastlio_mapping_node',
            output='screen',
            parameters=fast_lio_params,
            prefix=prefix,
            respawn=respawn,
            respawn_delay=3.0,
            condition=condition,
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'fast_lio_params_file',
                default_value=os.path.join(share, 'config', 'fast_lio.yaml'),
            ),
            DeclareLaunchArgument(
                'params_file', default_value=os.path.join(share, 'config', 'odometry_params.yaml')
            ),
            DeclareLaunchArgument(
                'save_map',
                default_value='false',
                description='Accumulate every scan and save a PCD on shutdown (RAM grows).',
            ),
            DeclareLaunchArgument(
                'wait_still',
                default_value='true',
                description='Hold FAST-LIO (also on respawn) until the IMU is still and level.',
            ),
            DeclareLaunchArgument('respawn', default_value='false'),
            DeclareLaunchArgument(
                'leg_odom_topic', default_value='/l1w/odom', description='"" disables the check'
            ),
            fast_lio(
                IfCondition(LaunchConfiguration('wait_still')),
                prefix=[gate, ' --params-file ', params, ' --'],
            ),
            fast_lio(UnlessCondition(LaunchConfiguration('wait_still'))),
            Node(
                package='robot_odometry',
                executable='odometry_monitor_node',
                name='odometry_monitor_node',
                output='screen',
                parameters=[params, {'leg_odom_topic': LaunchConfiguration('leg_odom_topic')}],
                respawn=respawn,
                respawn_delay=3.0,
            ),
        ]
    )
