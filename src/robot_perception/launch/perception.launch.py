import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch perception nodes."""
    pkg_share = get_package_share_directory("robot_perception")
    default_params = os.path.join(pkg_share, "config", "perception_params.yaml")

    params_file = LaunchConfiguration("params_file")
    image_topic = LaunchConfiguration("image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
        ),
        DeclareLaunchArgument(
            "image_topic",
            default_value="/camera/color/image_raw",
        ),
        DeclareLaunchArgument(
            "camera_info_topic",
            default_value="/camera/color/camera_info",
        ),

        Node(
            package="robot_perception",
            executable="lidar_sector_node",
            name="lidar_sector_node",
            output="screen",
            parameters=[params_file],
        ),

        Node(
            package="apriltag_ros",
            executable="apriltag_node",
            name="apriltag",
            output="screen",
            parameters=[params_file],
            remappings=[
                ("image_rect", image_topic),
                ("camera_info", camera_info_topic),
                ("detections", "/perception/apriltag/detections"),
            ],
        ),
    ])