import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_drivers')

    realsense_params_yaml = os.path.join(pkg_share, 'config', 'realsense_params.yaml')

    realsense_camera_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense_camera',
        output='screen',
        parameters=[
            realsense_params_yaml,
        ],
    )

    return LaunchDescription([
        realsense_camera_node,
    ])