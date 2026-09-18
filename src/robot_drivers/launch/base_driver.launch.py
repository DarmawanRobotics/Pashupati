#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch Livox and RealSense nodes."""

    package_dir = get_package_share_directory('robot_drivers')
    config_dir = os.path.join(package_dir, 'config')

    livox_config = os.path.join(config_dir, 'MID360_config.json')
    realsense_params = os.path.join(config_dir, 'realsense_params.yaml')

    livox_params = [
        {'xfer_format': 4},
        {'multi_topic': 0},
        {'data_src': 0},
        {'publish_freq': 10.0},
        {'output_data_type': 0},
        {'frame_id': 'livox_frame'},
        {'lvx_file_path': '/home/livox/livox_test.lvx'},
        {'user_config_path': livox_config},
        {'cmdline_input_bd_code': 'livox0000000001'},
    ]

    return LaunchDescription([
        Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_lidar_publisher',
            output='screen',
            parameters=livox_params,
        ),
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense_camera',
            output='screen',
            parameters=[realsense_params],
        ),
    ])