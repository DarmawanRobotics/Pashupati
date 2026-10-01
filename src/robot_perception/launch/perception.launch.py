#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch lidar sectors, AprilTag detection and, optionally, people detection."""
    default_params = os.path.join(
        get_package_share_directory('robot_perception'), 'config', 'perception_params.yaml'
    )
    params_file = LaunchConfiguration('params_file')
    image = '/camera/camera/color/image_raw'

    return LaunchDescription(
        [
            DeclareLaunchArgument('params_file', default_value=default_params),
            DeclareLaunchArgument('people', default_value='true',
                                  description='Run the person detector (crowd heatmap).'),
            DeclareLaunchArgument('draw_tags', default_value='false',
                                  description='Run apriltag_draw (needs that package).'),
            Node(
                package='robot_perception',
                executable='lidar_sector_node',
                name='lidar_sector_node',
                output='screen',
                parameters=[params_file],
            ),
            Node(
                package='apriltag_ros',
                executable='apriltag_node',
                name='apriltag',
                output='screen',
                parameters=[params_file],
                remappings=[
                    ('image_rect', image),
                    ('camera_info', '/camera/camera/color/camera_info'),
                    ('detections', '/perception/apriltag/detections'),
                ],
            ),
            Node(
                package='robot_perception',
                executable='person_detector_node',
                name='person_detector_node',
                output='screen',
                parameters=[params_file],
                condition=IfCondition(LaunchConfiguration('people')),
            ),
            Node(
                package='apriltag_draw',
                executable='apriltag_draw_node',
                name='apriltag_draw_node',
                output='screen',
                remappings=[
                    ('tags', '/perception/apriltag/detections'),
                    ('image', image),
                    ('image_tags', '/perception/apriltag/image_tags'),
                ],
                condition=IfCondition(LaunchConfiguration('draw_tags')),
            ),
        ]
    )
