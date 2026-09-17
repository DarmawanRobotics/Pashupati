import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_navigation')
    default_params = os.path.join(pkg_share, 'config', 'navigation_params.yaml')
    default_waypoints = os.path.join(pkg_share, 'waypoints', 'example_route.csv')

    params_file = LaunchConfiguration('params_file')
    waypoints_file = LaunchConfiguration('waypoints_file')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('waypoints_file', default_value=default_waypoints),
        Node(
            package='robot_navigation', executable='path_loader_node',
            name='path_loader_node', output='screen',
            parameters=[params_file, {'waypoints_file': waypoints_file}],
        ),
    ])
