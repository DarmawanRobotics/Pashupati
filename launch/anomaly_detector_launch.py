import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('moondream_anomaly_detector')
    default_params = os.path.join(pkg_share, 'config', 'anomaly_detector_params.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Path to the anomaly detector params YAML')

    node = Node(
        package='moondream_anomaly_detector',
        executable='anomaly_detector_node',
        name='anomaly_detector_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    return LaunchDescription([params_file_arg, node])