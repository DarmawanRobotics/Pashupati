#!/usr/bin/env python3
"""
The whole Pashupati robot in one launch (production: pashupati.service).

    ros2 launch pashupati robot.launch.py                    # patrol
    ros2 launch pashupati robot.launch.py mode:=teach        # keep the map, record a route
    ros2 launch pashupati robot.launch.py odometry:=leg_kilo
    ros2 launch pashupati robot.launch.py robot_bringup:=my_dog_bringup/bringup.launch.py

Works with any quadruped: its driver, URDF and sensors come from a robot package started with
robot_bringup (contract in src/xtras/README.md). Every Pashupati node respawns 3 s after it dies.
A site configuration file (by default /etc/pashupati/site.yaml) is applied on top of the package
defaults of every node. With MediaMTX installed (script/setup.sh install webrtc) the handheld gets
its video over WebRTC; without it, as JPEG over the teleop link.
"""

import os

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
    PackageNotFoundError,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from pashupati import video
from pashupati.bringup import (
    BringupError,
    expected_nodes,
    load_site_config,
    node_parameter,
    non_empty,
    package_version,
    resolve_mode,
    resolve_namespace,
    resolve_odometry,
    resolve_robot_bringup,
    truthy,
)

CAMERA = '/camera/camera/color'
MEDIAMTX_CONFIG = '~/.pashupati/mediamtx.yml'


def share(package: str, *path: str) -> str:
    """Return a file in the installed share directory of a package."""
    return os.path.join(get_package_share_directory(package), *path)


def robot_bringup_action(value: str, ns: str):
    """Include the robot package's launch file with namespace:=<robot_ns>, or None if unset."""
    robot = resolve_robot_bringup(value)
    if robot is None:
        return None
    package, launch_file = robot
    try:
        path = share(package, 'launch', launch_file)
    except PackageNotFoundError as error:
        raise BringupError(
            f'robot_bringup: package {package!r} is not built in this workspace '
            '(add the robot package, see src/xtras/README.md)'
        ) from error
    if not os.path.isfile(path):
        raise BringupError(f'robot_bringup: {path} does not exist')
    return IncludeLaunchDescription(
        AnyLaunchDescriptionSource(path), launch_arguments={'namespace': ns}.items()
    )


def video_server(webrtc: str, binary: str, params_files: list):
    """
    Return (MediaMTX process or None, log line) for the handheld's WebRTC video.

    MediaMTX reads the teleop token from the same parameter files as teleop_udp_node, and starts
    video_stream_node with those files while the handheld watches.
    """
    if not truthy(webrtc):
        return None, 'WebRTC video off (WEBRTC=false): the handheld gets JPEG frames'
    found = video.find_mediamtx(binary)
    if found is None:
        return None, (
            'WebRTC video off: MediaMTX not found (script/setup.sh install webrtc, or MEDIAMTX); '
            'the handheld gets JPEG frames'
        )
    version = video.mediamtx_version(found)
    if version is None or version < video.MIN_VERSION:
        return None, (
            f'WebRTC video off: {found} is {video.version_text(version)}, it needs v1.11 or '
            'newer (script/setup.sh install webrtc)'
        )
    try:
        executable = os.path.join(
            get_package_prefix('pashupati_bridge'), 'lib', 'pashupati_bridge', 'video_stream_node'
        )
    except PackageNotFoundError:
        executable = ''
    if not os.path.isfile(executable):
        return None, 'WebRTC video off: video_stream_node is not built (setup.sh install build)'
    missing = video.gstreamer_missing()
    if missing:
        return None, f'WebRTC video off: {missing} (script/setup.sh install webrtc)'
    token = str(node_parameter(params_files, 'teleop_udp_node', 'token', '') or '')
    command = video.node_command(executable, params_files, video.rtsp_url())
    try:
        path = video.write_config(video.mediamtx_config(command, token, version), MEDIAMTX_CONFIG)
    except OSError as error:
        return None, f'WebRTC video off: cannot write {MEDIAMTX_CONFIG}: {error}'
    process = ExecuteProcess(
        cmd=[found, path], name='mediamtx', output='screen', respawn=True, respawn_delay=3.0
    )
    return process, (
        f'WebRTC video: MediaMTX {video.version_text(version)} ({found}), '
        f'WHEP on port {video.WEBRTC_PORT}, media on UDP {video.ICE_PORT}'
    )


def launch_setup(context):
    """Validate the arguments and build every action of the robot."""
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    mode = resolve_mode(arg('mode'))
    odometry = resolve_odometry(arg('odometry'))
    ns = resolve_namespace(arg('robot_ns'))
    site, warnings = load_site_config(arg('site_config'))
    robot = robot_bringup_action(arg('robot_bringup'), ns)

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

    nav = share('pashupati', 'config', 'navigation_params.yaml')
    perception = share('pashupati', 'config', 'perception_params.yaml')
    bridge = share('pashupati_bridge', 'config', 'bridge_params.yaml')
    version = package_version(share('pashupati', 'package.xml'))
    git = os.environ.get('PASHUPATI_GIT', 'unknown')

    mediamtx, video_note = video_server(arg('webrtc'), arg('mediamtx'), [bridge, *site])
    teleop = {'robot_command_prefix': f'/{ns}/'}
    if mediamtx is not None:
        teleop['video_port'] = video.WEBRTC_PORT  # the handheld asks for WebRTC video here

    actions = [
        LogInfo(
            msg=f'Pashupati {version} ({git}) robot_id={arg("robot_id") or "(fleet default)"} '
            f'robot_ns={ns} robot_bringup={arg("robot_bringup") or "(started elsewhere)"} '
            f'mode={mode} odometry={odometry} site_config={site[0] if site else "(none)"}'
        ),
        *[LogInfo(msg=f'WARNING {w}') for w in warnings],
        LogInfo(msg=video_note),
    ]
    if robot is not None:
        actions.append(robot)
    if mediamtx is not None:
        actions.append(mediamtx)
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                share('dharma_odometry', 'launch', 'odometry.launch.py')
            ),
            launch_arguments={
                'method': odometry,
                'save_map': str(mode == 'teach'),
                'respawn': 'true',
                'record': 'true',
                'leg_odom_topic': topic('odom'),
                'site_config': site[0] if site else '',
                'params_file': share('pashupati', 'config', 'odometry_params.yaml'),
                'fast_lio_params_file': share('pashupati', 'config', 'fast_lio.yaml'),
                'leg_kilo_config_file': share('pashupati', 'config', 'leg_kilo.yaml'),
            }.items(),
        )
    )
    actions += [
        node(
            'dharma_localization',
            'localization_node',
            [share('pashupati', 'config', 'localization_params.yaml')],
            {'tags_config_file': arg('tags')},
        ),
        node('dharma_perception', 'lidar_sector_node', [perception]),
        # the tag detector sees the camera at rate_hz only: localization needs a few poses/s
        node(
            'dharma_perception',
            'image_throttle_node',
            [perception],
            name='apriltag_throttle',
            remappings=[
                ('in/image', f'{CAMERA}/image_raw'),
                ('in/camera_info', f'{CAMERA}/camera_info'),
                ('out/image', '/perception/apriltag/image_rect'),
                ('out/camera_info', '/perception/apriltag/camera_info'),
            ],
        ),
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag',
            parameters=[perception, *site],
            remappings=[
                ('image_rect', '/perception/apriltag/image_rect'),
                ('camera_info', '/perception/apriltag/camera_info'),
                ('detections', '/perception/apriltag/detections'),
            ],
            respawn=True,
            respawn_delay=3.0,
            output='screen',
        ),
        node('dharma_navigation', 'path_loader_node', [nav], {'waypoints_file': arg('route')}),
        node('dharma_navigation', 'obstacle_avoidance_node', [nav]),
        node(
            'dharma_navigation',
            'path_follower_node',
            [nav],
            remappings=[('cmd_vel', '/bridge/nav_cmd_vel'), ('battery', topic('battery'))],
        ),
        node(
            'dharma_control',
            'control_mux_node',
            [bridge],
            remappings=[('cmd_vel', topic('cmd_vel'))],
        ),
        node(
            'pashupati_bridge',
            'teleop_udp_node',
            [bridge],
            teleop,
            remappings=[
                ('cmd_vel', topic('cmd_vel')),
                ('battery', topic('battery')),
                ('diagnostics', topic('diagnostics')),
            ],
        ),
    ]
    if arg('cc_url').strip():
        actions.append(
            node(
                'pashupati_fleet',
                'fleet_uplink_node',
                [share('pashupati_fleet', 'config', 'fleet_params.yaml')],
                {
                    'server_url': arg('cc_url'),
                    'token': arg('cc_token'),
                    'robot_id': arg('robot_id'),
                },
                remappings=[
                    ('battery', topic('battery')),
                    ('diagnostics', topic('diagnostics')),
                ],
            )
        )
    else:
        actions.append(LogInfo(msg='NETRA uplink off: no cc_url (CC_URL)'))
    if truthy(arg('people')):
        actions.append(node('pashupati_inspection', 'person_detector_node', [perception]))
    if truthy(arg('inspection')):
        actions.append(
            node(
                'pashupati_inspection',
                'anomaly_detector_node',
                [share('pashupati_inspection', 'config', 'inspection_params.yaml')],
            )
        )
    if truthy(arg('health')):
        actions.append(
            node(
                'dharma_health',
                'health_monitor_node',
                [share('pashupati', 'config', 'health_params.yaml')],
                {'expected_nodes': expected_nodes(started, odometry), 'robot_ns': ns},
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
                'robot_ns',
                default_value=env('ROBOT_NS', 'robot'),
                description='namespace of the robot driver (/<robot_ns>/cmd_vel, /odom, ...)',
            ),
            DeclareLaunchArgument(
                'robot_bringup',
                default_value=env('ROBOT_BRINGUP'),
                description='robot package <pkg>[/<file>.launch.py]; empty: started elsewhere',
            ),
            DeclareLaunchArgument(
                'mode', default_value=env('MODE', 'patrol'), description='patrol | teach'
            ),
            DeclareLaunchArgument(
                'odometry',
                default_value=env('ODOMETRY', 'fast_lio'),
                description='fast_lio | leg_kilo | external (simulator)',
            ),
            DeclareLaunchArgument(
                'route', default_value=env('ROUTE'), description='route CSV; empty: last loaded'
            ),
            DeclareLaunchArgument(
                'tags', default_value=env('TAGS'), description='AprilTag map poses (JSON)'
            ),
            DeclareLaunchArgument(
                'cc_url', default_value=env('CC_URL'), description='NETRA url; empty: no uplink'
            ),
            DeclareLaunchArgument('cc_token', default_value=env('CC_TOKEN')),
            DeclareLaunchArgument(
                'site_config',
                default_value=env('SITE_CONFIG', '/etc/pashupati/site.yaml'),
                description='per-robot parameter overrides; missing file: none',
            ),
            DeclareLaunchArgument(
                'people', default_value=env('PEOPLE', 'true'), description='person detection'
            ),
            DeclareLaunchArgument(
                'inspection', default_value=env('INSPECTION', 'true'), description='VLM checks'
            ),
            DeclareLaunchArgument('health', default_value='true'),
            DeclareLaunchArgument(
                'webrtc',
                default_value=env('WEBRTC', 'true'),
                description='handheld video over WebRTC (MediaMTX); false: JPEG frames only',
            ),
            DeclareLaunchArgument(
                'mediamtx',
                default_value=env('MEDIAMTX'),
                description=f'MediaMTX binary; empty: {video.INSTALL_DIR}, then PATH',
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
