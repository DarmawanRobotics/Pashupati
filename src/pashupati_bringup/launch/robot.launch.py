#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def config(package: str, name: str) -> str:
    """Return the installed config file of a package."""
    return os.path.join(get_package_share_directory(package), 'config', name)


def generate_launch_description():
    """Launch the whole robot stack; every Pashupati node respawns if it dies."""
    ns = LaunchConfiguration('robot_ns')
    respawn = {'respawn': True, 'respawn_delay': 3.0, 'output': 'screen'}

    def topic(name):
        return ['/', ns, '/', name]

    def node(package, executable, params, condition=None, remappings=None):
        return Node(
            package=package,
            executable=executable,
            name=executable,
            parameters=params,
            remappings=remappings or [],
            condition=condition,
            **respawn,
        )

    nav = config('robot_navigation', 'navigation_params.yaml')
    perception = config('robot_perception', 'perception_params.yaml')
    mapping = config('robot_mapping', 'mapping_param.yaml')
    bridge = config('robot_bridge', 'bridge_params.yaml')
    on = lambda name: IfCondition(LaunchConfiguration(name))  # noqa: E731

    args = [
        DeclareLaunchArgument(
            'robot_ns', default_value=EnvironmentVariable('ROBOT_NS', default_value='l1w')
        ),
        DeclareLaunchArgument(
            'mode', default_value='patrol', description='patrol | mapping (saves the PCD map)'
        ),
        DeclareLaunchArgument(
            'route', default_value='', description='route CSV; empty: last loaded route'
        ),
        DeclareLaunchArgument('tags', default_value=EnvironmentVariable('TAGS', default_value='')),
        DeclareLaunchArgument(
            'cc_url', default_value=EnvironmentVariable('CC_URL', default_value='')
        ),
        DeclareLaunchArgument(
            'cc_token', default_value=EnvironmentVariable('CC_TOKEN', default_value='')
        ),
        DeclareLaunchArgument(
            'sensors', default_value='true', description='robot driver, Livox and RealSense'
        ),
        DeclareLaunchArgument(
            'people', default_value='true', description='person detection for the crowd map'
        ),
        DeclareLaunchArgument(
            'inspection', default_value='true', description='VLM anomaly detection'
        ),
        DeclareLaunchArgument('health', default_value='true'),
    ]

    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('l1w_bringup'), 'launch', 'bringup.launch.py')
        ),
        launch_arguments={'namespace': ns}.items(),
        condition=on('sensors'),
    )

    save_map = ParameterValue(
        PythonExpression(["'", LaunchConfiguration('mode'), "' == 'mapping'"]), value_type=bool
    )
    image = '/camera/camera/color/image_raw'

    nodes = [
        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            name='fastlio_mapping_node',
            parameters=[mapping, {'pcd_save.pcd_save_en': save_map}],
            **respawn,
        ),
        node('robot_mapping', 'path_recorder_node', [mapping]),
        node(
            'robot_localization',
            'localization_node',
            [
                config('robot_localization', 'localization_params.yaml'),
                {'tags_config_file': LaunchConfiguration('tags')},
            ],
        ),
        node('robot_perception', 'lidar_sector_node', [perception]),
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag',
            parameters=[perception],
            remappings=[
                ('image_rect', image),
                ('camera_info', '/camera/camera/color/camera_info'),
                ('detections', '/perception/apriltag/detections'),
            ],
            **respawn,
        ),
        node('robot_perception', 'person_detector_node', [perception], on('people')),
        node(
            'robot_navigation',
            'path_loader_node',
            [nav, {'waypoints_file': LaunchConfiguration('route')}],
        ),
        node('robot_navigation', 'obstacle_avoidance_node', [nav]),
        node(
            'robot_navigation',
            'path_follower_node',
            [nav],
            remappings=[('cmd_vel', '/bridge/nav_cmd_vel'), ('battery', topic('battery'))],
        ),
        node(
            'robot_bridge',
            'control_mux_node',
            [bridge],
            remappings=[('cmd_vel', topic('cmd_vel'))],
        ),
        node(
            'robot_bridge',
            'teleop_udp_node',
            [bridge],
            remappings=[
                ('cmd_vel', topic('cmd_vel')),
                ('battery', topic('battery')),
                ('diagnostics', topic('diagnostics')),
            ],
        ),
        node(
            'robot_fleet',
            'fleet_uplink_node',
            [
                config('robot_fleet', 'fleet_params.yaml'),
                {
                    'server_url': LaunchConfiguration('cc_url'),
                    'token': LaunchConfiguration('cc_token'),
                },
            ],
            remappings=[('battery', topic('battery')), ('diagnostics', topic('diagnostics'))],
        ),
        node(
            'robot_inspection',
            'anomaly_detector_node',
            [config('robot_inspection', 'inspection_params.yaml')],
            on('inspection'),
        ),
        node(
            'robot_health',
            'health_monitor_node',
            [config('robot_health', 'health_params.yaml')],
            on('health'),
        ),
    ]
    return LaunchDescription([*args, sensors, *nodes])
