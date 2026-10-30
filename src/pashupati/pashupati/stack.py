"""
The composition of the Pashupati robot: the Dharma stack plus the Pashupati packages.

This is the one place to adjust when a package's launch contract changes. Every package launch
accepts params_file, robot_params, use_sim_time and log_level; only non-empty arguments are
passed, so an unset one never hides a value from a parameter file.
"""

from __future__ import annotations

from dataclasses import dataclass, field

HEALTH_NODE = 'health_monitor_node'
TELEOP_NODE = 'teleop_udp_node'
# nodes that read sites_dir: the bringup gives them all the one of the launch
SITES_NODES = (
    'localization_node',
    'site_manager_node',
    'grid_mapper_node',
    'route_recorder_node',
    'path_loader_node',
    TELEOP_NODE,
    'fleet_uplink_node',
)


@dataclass(frozen=True)
class Include:
    """One package launch file with its arguments."""

    package: str
    launch_file: str
    arguments: dict = field(default_factory=dict)


@dataclass(frozen=True)
class Options:
    """The resolved bringup arguments."""

    robot_params: str
    params_file: str
    robot_ns: str = 'robot'
    odometry: str = 'fast_lio'
    use_sim_time: str = ''
    log_level: str = 'info'
    site: str = ''
    sites_dir: str = ''
    start_mapping: bool = False
    fiducials: bool = True
    fiducials_detector: str = 'apriltag'
    camera: str = 'driver'  # driver: the robot package publishes it; rtsp: rtsp_camera_node
    camera_url: str = ''
    camera_topics: str = '/camera/camera/color'
    health: bool = True
    teleop: bool = True
    video_port: str = ''  # MediaMTX WHEP port when it runs
    inspection: bool = True
    people: bool = True
    ollama_url: str = ''
    cc_url: str = ''
    cc_token: str = ''
    robot_id: str = ''

    @property
    def camera_image(self) -> str:
        """The raw image every consumer reads."""
        return f'{self.camera_topics}/image_raw'

    @property
    def camera_info(self) -> str:
        """The camera info every consumer reads."""
        return f'{self.camera_topics}/camera_info'


# component -> (package, launch file, nodes it starts, by node name)
COMPONENTS = {
    'odometry': ('dharma_odometry', 'odometry.launch.py', ('odometry_monitor_node',)),
    'localization': ('dharma_localization', 'localization.launch.py', ('localization_node',)),
    'perception': ('dharma_perception', 'perception.launch.py', ('lidar_sector_node',)),
    'mapping': (
        'dharma_mapping',
        'mapping.launch.py',
        ('grid_mapper_node', 'site_manager_node', 'route_recorder_node'),
    ),
    'navigation': (
        'dharma_navigation',
        'navigation.launch.py',
        (
            'planner_node',
            'path_loader_node',
            'path_follower_node',
            'obstacle_avoidance_node',
            'docking_node',
        ),
    ),
    'control': ('dharma_control', 'control.launch.py', ('control_mux_node',)),
    'health': ('dharma_health', 'health.launch.py', (HEALTH_NODE,)),
    'bridge': ('pashupati_bridge', 'bridge.launch.py', (TELEOP_NODE,)),
    'inspection': ('pashupati_inspection', 'inspection.launch.py', ()),
    'fleet': ('pashupati_fleet', 'fleet.launch.py', ('fleet_uplink_node',)),
}
ESTIMATOR_NODES = {'fast_lio': ('fastlio_mapping_node',)}  # leg_kilo names its own node
FIDUCIAL_NODES = ('apriltag_throttle', 'apriltag')
DETECTOR_NODES = {'apriltag': 'apriltag', 'aruco': 'aruco_detector_node'}
INSPECTION_NODES = {'inspection': 'anomaly_detector_node', 'people': 'person_detector_node'}
CAMERA_NODE = 'rtsp_camera'
SIM = ('dharma_sim', 'sim.launch.py', ('sim_robot_node',))
# configurable but not started as a node of its own here
OTHER_SECTIONS = ('odometry_gate', 'video_stream_node')


def non_empty(arguments: dict) -> dict:
    """Return the arguments that are set (an empty one must never override a file)."""
    return {k: str(v) for k, v in arguments.items() if v is not None and str(v).strip()}


def flag(value: bool) -> str:
    """Return a launch bool."""
    return 'true' if value else 'false'


def common(o: Options) -> dict:
    """Return the arguments every package launch accepts."""
    return {
        'params_file': o.params_file,
        'robot_params': o.robot_params,
        'use_sim_time': o.use_sim_time,
        'log_level': o.log_level,
    }


def component_arguments(o: Options) -> dict:
    """Return the launch arguments of every component (before dropping empty ones)."""
    driver = f'/{o.robot_ns}'
    return {
        'odometry': {
            'method': o.odometry,
            'save_map': flag(o.start_mapping),
            'leg_odom_topic': f'{driver}/odom',
            'site_config': o.params_file,  # the estimator reads it too (FAST-LIO overrides)
        },
        'localization': {'sites_dir': o.sites_dir},
        'perception': {
            'fiducials': flag(o.fiducials),
            'fiducials_detector': o.fiducials_detector,
            'camera_image': o.camera_image,
            'camera_info': o.camera_info,
        },
        'mapping': {
            'sites_dir': o.sites_dir,
            'site': o.site,
            'start_mapping': 'true' if o.start_mapping else '',
        },
        'navigation': {'battery_topic': f'{driver}/battery', 'odom_topic': f'{driver}/odom'},
        'control': {'cmd_vel_topic': f'{driver}/cmd_vel'},  # the mux drives the robot
        'health': {},
        'bridge': {
            'robot_ns': o.robot_ns,
            'teleop': flag(o.teleop),
            'video_port': o.video_port,
            'camera': o.camera,
            'camera_url': o.camera_url,
            'camera_topics': o.camera_topics,
        },
        'inspection': {
            'anomaly': flag(o.inspection),
            'people': flag(o.people),
            'camera_image': o.camera_image,
            'camera_info': o.camera_info,
            'ollama_url': o.ollama_url,
        },
        'fleet': {
            'robot_ns': o.robot_ns,
            'server_url': o.cc_url,
            'token': o.cc_token,
            'robot_id': o.robot_id,
        },
    }


def components(o: Options) -> list[str]:
    """Return the components to start, in launch order."""
    wanted = {
        'health': o.health,
        'bridge': o.teleop or o.camera == 'rtsp',
        'inspection': o.inspection or o.people,
        'fleet': bool(o.cc_url),
    }
    return [name for name in COMPONENTS if wanted.get(name, True)]


def compose(o: Options) -> list[Include]:
    """Return the package launch files of the robot with their arguments."""
    arguments = component_arguments(o)
    out = []
    for name in components(o):
        package, launch_file, _ = COMPONENTS[name]
        out.append(Include(package, launch_file, non_empty({**common(o), **arguments[name]})))
    return out


def sim_include(o: Options, world: str = '', scenario: str = '', rviz: bool = False) -> Include:
    """Return the simulated robot (driver, LiDAR, IMU and odometry in one node)."""
    package, launch_file, _ = SIM
    args = {**common(o), 'world': world, 'scenario': scenario, 'robot_ns': o.robot_ns,
            'site': o.site, 'sites_dir': o.sites_dir, 'rviz': flag(rviz)}  # fmt: skip
    return Include(package, launch_file, non_empty(args))


def node_names(o: Options, sim: bool = False) -> list[str]:
    """Return the fully qualified names of every node the bringup starts."""
    started = components(o)
    names = [n for c in started for n in COMPONENTS[c][2]]
    if not o.teleop and TELEOP_NODE in names:  # the bridge runs for the RTSP camera only
        names.remove(TELEOP_NODE)
    names += ESTIMATOR_NODES.get(o.odometry, ())
    if o.fiducials:
        names += (FIDUCIAL_NODES[0], DETECTOR_NODES[o.fiducials_detector])
    names += [node for key, node in INSPECTION_NODES.items() if getattr(o, key)]
    if o.camera == 'rtsp':
        names.append(CAMERA_NODE)
    if sim:
        names += SIM[2]
    return sorted(f'/{n}' for n in names)


def known_sections() -> list[str]:
    """Return every node name a parameter file of the bringup may configure."""
    names = [n for c in COMPONENTS.values() for n in c[2]]
    names += [n for nodes in ESTIMATOR_NODES.values() for n in nodes]
    names += [*FIDUCIAL_NODES, *DETECTOR_NODES.values(), *INSPECTION_NODES.values()]
    return sorted({*names, CAMERA_NODE, *SIM[2], *OTHER_SECTIONS})
