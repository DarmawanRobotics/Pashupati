import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_mission')
    default_tree = os.path.join(pkg_share, 'trees', 'patrol.xml')

    tree_file = LaunchConfiguration('tree_file')
    tick_rate = LaunchConfiguration('tick_rate')

    return LaunchDescription([
        DeclareLaunchArgument('tree_file', default_value=default_tree),
        DeclareLaunchArgument('tick_rate', default_value='5.0'),
        Node(
            package='robot_mission',
            executable='mission_node',
            name='mission_node',
            output='screen',
            parameters=[{'tree_file': tree_file, 'tick_rate': tick_rate}],
        ),
    ])
