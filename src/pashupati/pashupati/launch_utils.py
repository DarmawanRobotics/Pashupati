"""Launch-side helpers shared by robot.launch.py and sim.launch.py."""

from __future__ import annotations

from dataclasses import replace
import os

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_prefix,
    get_package_share_directory,
)
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    UnsetLaunchConfiguration,
)
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import EnvironmentVariable, LaunchConfiguration

from dharma_utils.ros.launch import group
from pashupati import bringup, stack

PACKAGE = 'pashupati'
MEDIAMTX_CONFIG = '~/.pashupati/mediamtx.yml'

# name -> (environment variable or None, default, description); pashupati.env sets them
ARGUMENTS = {
    'robot_params': ('ROBOT_PARAMS', '', 'robot profile (robot.*); empty: config/robot.yaml'),
    'params_file': ('SITE_CONFIG', bringup.DEFAULT_SITE_CONFIG, 'site file on top of stack.yaml'),
    'use_sim_time': (None, '', 'true | false; empty: from the parameter files'),
    'log_level': (None, 'info', 'debug, info, warn, error'),
    'robot_ns': ('ROBOT_NS', bringup.DEFAULT_ROBOT_NS, 'driver namespace (/<ns>/cmd_vel, odom)'),
    'robot_driver': ('ROBOT_DRIVER', '', '<package>[/<file>.launch.py]; empty: started elsewhere'),
    'odometry': ('ODOMETRY', 'fast_lio', 'fast_lio | leg_kilo | external'),
    'mode': ('MODE', 'patrol', 'patrol | teach (teach: map `site` from the start)'),
    'site': ('SITE', '', 'site to load (or to map); empty: the last one'),
    'sites_dir': ('SITES_DIR', '~/.pashupati/sites', 'sites: maps, tags, taught routes'),
    'fiducials': (None, 'true', 'camera throttle + tag detector'),
    'fiducials_detector': ('FIDUCIALS', 'apriltag', 'apriltag (apriltag_ros) | aruco'),
    'camera': ('CAMERA', 'driver', 'driver (the robot package) | rtsp (rtsp_camera_node)'),
    'camera_url': ('CAMERA_URL', '', 'rtsp:// url of the camera (camera:=rtsp)'),
    'camera_topics': ('CAMERA_TOPICS', '/camera/camera/color', 'camera topic prefix'),
    'health': (None, 'true', 'start the health monitor'),
    'inspection': ('INSPECTION', 'true', 'VLM inspection (anomaly_detector_node)'),
    'people': ('PEOPLE', 'true', 'person detection (person_detector_node)'),
    'ollama_url': ('OLLAMA_URL', '', 'Ollama server; empty: the inspection default'),
    'webrtc': ('WEBRTC', 'true', 'handheld video over WebRTC (MediaMTX); false: JPEG only'),
    'mediamtx': ('MEDIAMTX', '', 'MediaMTX binary; empty: /opt/pashupati/mediamtx, then PATH'),
    'cc_url': ('CC_URL', '', 'NETRA url; empty: no uplink'),
    'cc_token': ('CC_TOKEN', '', 'NETRA robot token'),
    'robot_id': ('ROBOT_ID', '', 'fleet id; empty: from the parameter files'),
}
SIM_ONLY = ('world', 'scenario', 'rviz')


def share(package: str, *path: str) -> str:
    """Return a file in the installed share directory of a package."""
    try:
        root = get_package_share_directory(package)
    except PackageNotFoundError as error:
        raise bringup.BringupError(
            f'package {package!r} is not built in this workspace '
            '(vcs import src < pashupati.repos, then colcon build)'
        ) from error
    return os.path.join(root, *path)


def declare(names, defaults: dict | None = None, env: bool = True) -> list:
    """Return DeclareLaunchArgument actions for ARGUMENTS (env: default from the environment)."""
    defaults = defaults or {}
    actions = []
    for name in names:
        variable, default, description = ARGUMENTS[name]
        default = defaults.get(name, default)
        if env and variable:
            default = EnvironmentVariable(variable, default_value=default)
        actions.append(DeclareLaunchArgument(name, default_value=default, description=description))
    return actions


def options_of(arg, robot_params: str, sim: bool) -> stack.Options:
    """Return the validated Options of the launch arguments (params_file filled in later)."""
    mode = bringup.resolve_mode(arg('mode'))
    camera, url = bringup.resolve_camera(arg('camera'), arg('camera_url'))
    return stack.Options(
        robot_params=robot_params,
        params_file='',
        robot_ns=bringup.resolve_namespace(arg('robot_ns')),
        odometry='external' if sim else bringup.resolve_odometry(arg('odometry')),
        use_sim_time=arg('use_sim_time').strip(),
        log_level=arg('log_level').strip() or 'info',
        site=bringup.resolve_site(arg('site')),
        sites_dir=os.path.expanduser(arg('sites_dir').strip()),
        start_mapping=mode == 'teach',
        fiducials=bringup.truthy(arg('fiducials')),
        fiducials_detector=bringup.resolve_detector(arg('fiducials_detector')),
        camera=camera,
        camera_url=url,
        camera_topics=bringup.resolve_topics(arg('camera_topics')),
        health=bringup.truthy(arg('health')),
        inspection=bringup.truthy(arg('inspection')),
        people=bringup.truthy(arg('people')),
        ollama_url=arg('ollama_url').strip(),
        cc_url=arg('cc_url').strip(),
        cc_token=arg('cc_token').strip(),
        robot_id=arg('robot_id').strip(),
    )


def resolve(arg, sim: bool = False):
    """Validate the arguments and merge the parameter files; return (Options, data, notes)."""
    robot_params = bringup.existing_file('robot_params', arg('robot_params'))
    robot_params = robot_params or share(PACKAGE, 'config', 'robot.yaml')
    profile = bringup.robot_profile(robot_params)
    options = options_of(arg, robot_params, sim)
    if options.start_mapping and not options.site:
        raise bringup.BringupError('mode:=teach needs site:=<name> (the site to map)')
    site_file, note = bringup.site_config(arg('params_file'))
    files = [share(PACKAGE, 'config', 'stack.yaml')]
    files += [share(PACKAGE, 'config', 'sim.yaml')] if sim else []
    files.append(site_file)
    data = bringup.merge_files(files)
    data = bringup.with_sites_dir(data, stack.SITES_NODES, options.sites_dir)
    data = bringup.with_robot_ns(data, stack.HEALTH_NODE, options.robot_ns)
    unknown = bringup.unknown_sections(data, stack.known_sections())
    if options.health:
        names = stack.node_names(options, sim=sim)
        data = bringup.with_expected_nodes(data, stack.HEALTH_NODE, names)
    options = replace(options, params_file=bringup.write_params(data))
    version = bringup.package_version(share(PACKAGE, 'package.xml'))
    notes = [
        LogInfo(
            msg=f'Pashupati {version} ({os.environ.get("PASHUPATI_GIT", "unknown")}): robot '
            f'{profile.name} ({profile.kinematics}), ns /{options.robot_ns}, odometry '
            f'{options.odometry}, site {options.site or "(the last one)"} in {options.sites_dir}, '
            f'camera {options.camera}, parameters {robot_params}, '
            f'{", ".join(f for f in files if f)}'
        ),
        *([LogInfo(msg=note)] if note else []),
        *[LogInfo(msg=f'WARNING no node is called "{n}" (typo?)') for n in unknown],
    ]
    if not options.cc_url:
        notes.append(LogInfo(msg='NETRA uplink off: no cc_url (CC_URL)'))
    return options, data, notes


def isolated(actions: list):
    """Return actions in their own scope that the bringup's own arguments never reach."""
    hidden = [UnsetLaunchConfiguration(name) for name in (*ARGUMENTS, *SIM_ONLY)]
    return group([*hidden, *actions], scoped=True, forwarding=False)


def include_action(include: stack.Include):
    """Return a package launch in its own scope: only its explicit arguments reach it."""
    source = PythonLaunchDescriptionSource(share(include.package, 'launch', include.launch_file))
    return isolated([IncludeLaunchDescription(source, launch_arguments=include.arguments.items())])


def driver_action(value: str, options: stack.Options):
    """Return the robot driver launch with namespace:=<robot_ns>, or None when unset."""
    driver = bringup.resolve_driver(value)
    if driver is None:
        return None
    package, launch_file = driver
    path = share(package, 'launch', launch_file)
    if not os.path.isfile(path):
        raise bringup.BringupError(f'robot_driver: {path} does not exist')
    args = stack.non_empty({'namespace': options.robot_ns, 'use_sim_time': options.use_sim_time})
    source = AnyLaunchDescriptionSource(path)
    return isolated([IncludeLaunchDescription(source, launch_arguments=args.items())])


def video_executable() -> str:
    """Return the installed video_stream_node, '' when the bridge is not built."""
    try:
        prefix = get_package_prefix('pashupati_bridge')
    except PackageNotFoundError:
        return ''
    path = os.path.join(prefix, 'lib', 'pashupati_bridge', 'video_stream_node')
    return path if os.path.isfile(path) else ''


def performer(context):
    """Return arg(name) -> the evaluated launch argument."""
    return lambda name: LaunchConfiguration(name).perform(context)


def mediamtx_action(path: str, binary: str):
    """Return MediaMTX running a configuration, respawning."""
    return ExecuteProcess(cmd=[binary, path], name='mediamtx', output='screen', respawn=True,
                          respawn_delay=3.0)  # fmt: skip
