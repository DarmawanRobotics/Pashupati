#!/usr/bin/env python3
"""
The whole Pashupati robot in one launch (production: pashupati.service).

    ros2 launch pashupati robot.launch.py                    # patrol
    ros2 launch pashupati robot.launch.py mode:=teach        # keep the map, record a route
    ros2 launch pashupati robot.launch.py odometry:=leg_kilo

Every Pashupati node respawns 3 s after it dies. A site configuration file (by default
/etc/pashupati/site.yaml) is applied on top of the package defaults of every node.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from pashupati.bringup import (
    expected_nodes,
    load_site_config,
    non_empty,
    package_version,
    resolve_mode,
    resolve_namespace,
    resolve_odometry,
    truthy,
)

CAMERA = '/camera/camera/color'


def share(package: str, *path: str) -> str:
    """Return a file in the installed share directory of a package."""
    return os.path.join(get_package_share_directory(package), *path)


def launch_setup(context):
    """Validate the arguments and build every action of the robot."""
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    mode = resolve_mode(arg('mode'))
    odometry = resolve_odometry(arg('odometry'))
    ns = resolve_namespace(arg('robot_ns'))
    site, warnings = load_site_config(arg('site_config'))

    def topic(name: str) -> str:
        return f'/{ns}/{name}'

    started = []  # node names, for the health monitor's liveness check

    def node(package, executable, defaults, overrides=None, remappings=None, **kwargs):
        """Pashupati node: package defaults, then the site file, then non-empty overrides."""
        params = [*defaults, *site]
        if overrides and non_empty(overrides):
            params.append(non_empty(overrides))
        name = kwargs.pop('name', executable)
        started.append(name)
        return Node(
            package=package,
            executable=executable,
            name=name,
            parameters=params,
            remappings=remappings or [],
            respawn=True,
            respawn_delay=3.0,
            output='screen',
            **kwargs,
        )

    nav = share('robot_navigation', 'config', 'navigation_params.yaml')
    perception = share('robot_perception', 'config', 'perception_params.yaml')
    bridge = share('robot_bridge', 'config', 'bridge_params.yaml')
    version = package_version(share('pashupati', 'package.xml'))
    git = os.environ.get('PASHUPATI_GIT', 'unknown')

    actions = [
        LogInfo(
            msg=f'Pashupati {version} ({git}) robot_id={arg("robot_id") or "(fleet default)"} '
            f'robot_ns={ns} mode={mode} odometry={odometry} '
            f'site_config={site[0] if site else "(none)"}'
        ),
        *[LogInfo(msg=f'WARNING {w}') for w in warnings],
    ]
    if truthy(arg('sensors')):
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(share('l1w_bringup', 'launch', 'bringup.launch.py')),
                launch_arguments={'namespace': ns}.items(),
            )
        )
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(share('robot_odometry', 'launch', 'odometry.launch.py')),
            launch_arguments={
                'method': odometry,
                'save_map': str(mode == 'teach'),
                'respawn': 'true',
                'record': 'true',
                'leg_odom_topic': topic('odom'),
                'site_config': site[0] if site else '',
            }.items(),
        )
    )
    actions += [
        node(
            'robot_localization',
            'localization_node',
            [share('robot_localization', 'config', 'localization_params.yaml')],
            {'tags_config_file': arg('tags')},
        ),
        node('robot_perception', 'lidar_sector_node', [perception]),
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag',
            parameters=[perception, *site],
            remappings=[
                ('image_rect', f'{CAMERA}/image_raw'),
                ('camera_info', f'{CAMERA}/camera_info'),
                ('detections', '/perception/apriltag/detections'),
            ],
            respawn=True,
            respawn_delay=3.0,
            output='screen',
        ),
        node('robot_navigation', 'path_loader_node', [nav], {'waypoints_file': arg('route')}),
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
            [share('robot_fleet', 'config', 'fleet_params.yaml')],
            {
                'server_url': arg('cc_url'),
                'token': arg('cc_token'),
                'robot_id': arg('robot_id'),
            },
            remappings=[('battery', topic('battery')), ('diagnostics', topic('diagnostics'))],
        ),
    ]
    if truthy(arg('people')):
        actions.append(node('robot_perception', 'person_detector_node', [perception]))
    if truthy(arg('inspection')):
        actions.append(
            node(
                'robot_inspection',
                'anomaly_detector_node',
                [share('robot_inspection', 'config', 'inspection_params.yaml')],
            )
        )
    if truthy(arg('health')):
        actions.append(
            node(
                'robot_health',
                'health_monitor_node',
                [share('robot_health', 'config', 'health_params.yaml')],
                {'expected_nodes': expected_nodes(started, odometry)},
            )
        )
    return actions


def generate_launch_description():
    """Declare the arguments (most default to the environment of pashupati.service)."""
    env = lambda name, default='': EnvironmentVariable(name, default_value=default)  # noqa: E731
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'robot_id', default_value=env('ROBOT_ID'), description='fleet id; empty: yaml'
            ),
            DeclareLaunchArgument(
                'robot_ns', default_value=env('ROBOT_NS', 'l1w'), description='driver namespace'
            ),
            DeclareLaunchArgument(
                'mode', default_value=env('MODE', 'patrol'), description='patrol | teach'
            ),
            DeclareLaunchArgument(
                'odometry',
                default_value=env('ODOMETRY', 'fast_lio'),
                description='fast_lio | leg_kilo',
            ),
            DeclareLaunchArgument(
                'route', default_value=env('ROUTE'), description='route CSV; empty: last loaded'
            ),
            DeclareLaunchArgument(
                'tags', default_value=env('TAGS'), description='AprilTag map poses (JSON)'
            ),
            DeclareLaunchArgument('cc_url', default_value=env('CC_URL')),
            DeclareLaunchArgument('cc_token', default_value=env('CC_TOKEN')),
            DeclareLaunchArgument(
                'site_config',
                default_value=env('SITE_CONFIG', '/etc/pashupati/site.yaml'),
                description='per-robot parameter overrides; missing file: none',
            ),
            DeclareLaunchArgument(
                'sensors', default_value='true', description='robot driver, Livox and RealSense'
            ),
            DeclareLaunchArgument(
                'people', default_value=env('PEOPLE', 'true'), description='person detection'
            ),
            DeclareLaunchArgument(
                'inspection', default_value=env('INSPECTION', 'true'), description='VLM checks'
            ),
            DeclareLaunchArgument('health', default_value='true'),
            OpaqueFunction(function=launch_setup),
        ]
    )
