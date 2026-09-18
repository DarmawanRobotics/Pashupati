import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include(pkg, launch_file, launch_arguments=None, condition=None):
    """Build an IncludeLaunchDescription for pkg's launch/launch_file, with optional passthrough args."""
    path = os.path.join(get_package_share_directory(pkg), 'launch', launch_file)
    source = PythonLaunchDescriptionSource(path)
    kwargs = {}
    if launch_arguments:
        kwargs['launch_arguments'] = launch_arguments.items()
    if condition is not None:
        kwargs['condition'] = condition
    return IncludeLaunchDescription(source, **kwargs)


def generate_launch_description():
    """Bring up the full stack in dependency order: description+drivers first (need real
    init time), then mapping/localization/perception once sensor data is flowing, then
    navigation once perception is up, then mission last since it needs navigation's action
    server. Each stage is a TimerAction delay, not a real readiness check -- if a stage is
    slow to come up on a given boot, later stages may still start before it's truly ready
    and will just retry/wait on their own (TF lookups, action server discovery, etc.)."""
    use_rviz = LaunchConfiguration('use_rviz')
    waypoints_file = LaunchConfiguration('waypoints_file')
    tree_file = LaunchConfiguration('tree_file')

    default_waypoints = '/home/robot/dev/Pashupati/map/example/example_waypoint.csv'
    default_tree = os.path.join(get_package_share_directory('robot_mission'), 'trees', 'patrol.xml')

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('waypoints_file', default_value=default_waypoints),
        DeclareLaunchArgument('tree_file', default_value=default_tree),

        # t=0s: TF tree + hardware drivers. Both take real wall-clock time to
        # actually come up (sensor init, SDK connect), everything else waits.
        include('robot_description', 'description.launch.py'),
        include('robot_drivers', 'driver.launch.py'),
        include('robot_bringup', 'rviz.launch.py', condition=IfCondition(use_rviz)),

        # t=3s: mapping/localization need Livox data + TF flowing.
        TimerAction(period=3.0, actions=[
            include('robot_mapping', 'fast_lio.launch.py'),
            include('robot_mapping', 'path_recorder.launch.py'),
            include('robot_localization', 'localization.launch.py'),
        ]),

        # t=4s: perception needs the same Livox data + base_link->livox_frame TF,
        # plus the (rectified) camera stream from robot_drivers.
        TimerAction(period=4.0, actions=[
            include('robot_perception', 'perception.launch.py'),
        ]),

        # t=6s: navigation consumes perception's SectorScan and localization's map->odom.
        TimerAction(period=6.0, actions=[
            include('robot_navigation', 'obstacle_avoidance.launch.py'),
            include('robot_navigation', 'path_follower.launch.py'),
            include('robot_navigation', 'path_loader.launch.py', {'waypoints_file': waypoints_file}),
        ]),

        # t=7s: mission needs path_follower_node's navigate_route action server up.
        TimerAction(period=7.0, actions=[
            include('robot_mission', 'mission.launch.py', {'tree_file': tree_file}),
        ]),
    ])
