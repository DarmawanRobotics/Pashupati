import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_rviz_config = os.path.join(
        get_package_share_directory('robot_bringup'), 'rviz', 'robot.rviz'
    )
    rviz_config = LaunchConfiguration('rviz_config')

    return LaunchDescription([
        DeclareLaunchArgument('rviz_config', default_value=default_rviz_config),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
