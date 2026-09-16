"""Launch the SDK sender and receiver nodes with their shared params file."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Start the sender and receiver nodes."""
    pkg_share = get_package_share_directory('robot_drivers')
    params_yaml = os.path.join(pkg_share, 'config', 'robot_driver_params.yaml')

    sender_node = Node(
        package='robot_drivers',
        executable='sender_node',
        name='robot_driver_sender',
        output='screen',
        parameters=[params_yaml],
    )

    receiver_node = Node(
        package='robot_drivers',
        executable='receiver_node',
        name='robot_driver_receiver',
        output='screen',
        parameters=[params_yaml],
    )

    return LaunchDescription([
        sender_node,
        receiver_node,
    ])
