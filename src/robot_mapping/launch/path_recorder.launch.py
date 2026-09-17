import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_mapping')
    default_params = os.path.join(pkg_share, 'config', 'path_record.yaml')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        Node(
            package='robot_mapping',
            executable='path_recorder_node',
            name='path_recorder_node',
            output='screen',
            parameters=[params_file],
        )
    ])