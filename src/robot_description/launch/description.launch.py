import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory("robot_description")
    urdf = os.path.join(
        pkg,
        "urdf",
        "robot.urdf.xacro"
    )
    robot_description = {
        "robot_description": Command([
            "xacro ",
            urdf
        ])
    }
    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),
    ])