#!/usr/bin/env python3
import math
import threading

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import MarkerArray

from robot_interfaces.msg import AvoidanceCommand, NavigationStatus, StopPointEvent, WaypointPath

from robot_navigation.utils import visualization
from robot_navigation.utils.command_filter import SlewLimiter
from robot_navigation.utils.controllers.registry import create_controller
from robot_navigation.utils.final_approach import FinalApproach
from robot_navigation.utils.geometry import clamp, yaw_from_quaternion
from robot_navigation.utils.path_processing import velocity_profile
from robot_navigation.utils.pose2d import Pose2D
from robot_navigation.utils.speed_regulator import SpeedRegulator


class PathFollowerNode(Node):
    """Patrols a taught route: follows it with a selectable controller, blends reactive avoidance,
    and at each stop point converges onto x/y/yaw, triggers inspection services and dwells."""

    _WARN_STATES = {'NO_PATH', 'TF_UNAVAILABLE', 'EMERGENCY_STOP', 'AVOIDANCE_STALE'}

    def __init__(self):
        """Declare params, set up TF, build the controller, and wire all interfaces."""
        super().__init__('path_follower_node')
        self.declare_params()
        p = self.get_parameter

        self._control_rate = float(p('control_rate').value)
        self._target_linear_velocity = float(p('target_linear_velocity').value)
        self._min_linear_velocity = float(p('min_linear_velocity').value)
        self._max_lateral_accel = float(p('max_lateral_accel').value)
        self._decel_limit = float(p('decel_limit').value)
        self._approach_speed = float(p('approach_speed').value)
        self._max_linear_velocity = float(p('max_linear_velocity').value)
        self._max_angular_velocity = float(p('max_angular_velocity').value)
        self._arc_length = float(p('arc_visualization_length').value)
        self._marker_lifetime = Duration(seconds=float(p('marker_lifetime_sec').value)).to_msg()
        self._avoidance_enabled = bool(p('avoidance_enabled').value)
        self._speed_regulator_enabled = bool(p('speed_regulator_enabled').value)
        self._avoidance_timeout = Duration(seconds=float(p('avoidance_timeout_sec').value))
        self._tf_timeout = Duration(seconds=float(p('tf_timeout_sec').value))
        self._loop_route = bool(p('loop_route').value)
        self._loop_close_distance = float(p('loop_close_distance').value)

        self._approach_radius = float(p('approach_radius').value)
        self._approach_timeout = float(p('approach_timeout_sec').value)
        self._holonomic = bool(p('holonomic').value)
        self._stop_point_skip_margin = int(p('stop_point_skip_margin').value)
        self._approach = FinalApproach(
            holonomic=self._holonomic,
            kp_xy=float(p('approach.kp_xy').value),
            kp_yaw=float(p('approach.kp_yaw').value),
            max_speed=float(p('approach.max_speed').value),
            max_yaw_rate=float(p('approach.max_yaw_rate').value),
            min_speed=float(p('approach.min_speed').value),
            position_tolerance=float(p('approach.position_tolerance').value),
            yaw_tolerance=math.radians(float(p('approach.yaw_tolerance_deg').value)),
        )
        self._inspection_timeout = float(p('inspection_timeout_sec').value)

        self._linear_limiter = SlewLimiter(float(p('linear_accel_limit').value))
        self._angular_limiter = SlewLimiter(float(p('angular_accel_limit').value))
        self._speed_regulator = SpeedRegulator(
            kp=float(p('speed_regulator.kp').value),
            ki=float(p('speed_regulator.ki').value),
            kd=float(p('speed_regulator.kd').value),
            min_scale=float(p('speed_regulator.min_scale').value),
        )

        self._avoidance = AvoidanceCommand(velocity_scale=1.0)
        self._avoidance_time = None
        self._nav_active = False
        self._waypoints: list[tuple[float, float, float, float]] = []
        self._speed_profile: list[float] = []
        self._next_stop_index = None
        self._follower_state = 'FOLLOWING'
        self._approach_start_time = None
        self._approach_index = 0
        self._approach_is_goal = False
        self._dwell_duration = 0.0
        self._dwell_start_time = None
        self._inspection_pending = 0
        self._inspection_results: list[str] = []
        self._lap = 0
        self._last_status = 'NAV_INACTIVE'

        self._controller_name = p('controller').value
        self._controller = create_controller(self._controller_name, self.build_controller_params(self._controller_name))

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)

        self._srv_group = ReentrantCallbackGroup()
        self._loop_group = MutuallyExclusiveCallbackGroup()
        latched = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(WaypointPath, 'navigation/waypoints', self.waypoints_callback, latched,
                                 callback_group=self._loop_group)
        self.create_subscription(AvoidanceCommand, 'navigation/avoidance', self.avoidance_callback, 10,
                                 callback_group=self._loop_group)
        self._cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self._markers_pub = self.create_publisher(MarkerArray, 'navigation/markers', 10)
        self._status_pub = self.create_publisher(NavigationStatus, 'navigation/status', 10)
        self._stop_event_pub = self.create_publisher(StopPointEvent, 'navigation/stop_point_event', 10)

        self._auto_on_client = self.optional_client(p('auto_mode_on_service').value)
        self._auto_off_client = self.optional_client(p('auto_mode_off_service').value)
        self._inspection_clients = [
            self.create_client(Trigger, name, callback_group=self._srv_group)
            for name in p('inspection_services').value if name
        ]
        self.create_service(SetBool, 'navigation/start_nav', self.start_nav_callback,
                            callback_group=self._srv_group)
        self.add_on_set_parameters_callback(self.on_parameters_changed)

        self._last_time = self.get_clock().now()
        self.create_timer(1.0 / self._control_rate, self.control_loop, callback_group=self._loop_group)

    def declare_params(self):
        """Declare every parameter with its default."""
        defaults = {
            'control_rate': 20.0,
            'target_linear_velocity': 0.4,
            'min_linear_velocity': 0.1,
            'max_lateral_accel': 0.3,
            'decel_limit': 0.3,
            'approach_speed': 0.15,
            'max_linear_velocity': 0.6,
            'max_angular_velocity': 1.0,
            'linear_accel_limit': 0.5,
            'angular_accel_limit': 1.5,
            'arc_visualization_length': 2.0,
            'marker_lifetime_sec': 0.5,
            'avoidance_enabled': True,
            'avoidance_timeout_sec': 0.5,
            'tf_timeout_sec': 0.5,
            'loop_route': True,
            'loop_close_distance': 1.0,
            'holonomic': True,
            'approach_radius': 0.5,
            'approach_timeout_sec': 20.0,
            'approach.kp_xy': 1.2,
            'approach.kp_yaw': 1.5,
            'approach.max_speed': 0.2,
            'approach.max_yaw_rate': 0.6,
            'approach.min_speed': 0.04,
            'approach.position_tolerance': 0.05,
            'approach.yaw_tolerance_deg': 3.0,
            'stop_point_skip_margin': 5,
            'auto_mode_on_service': '',
            'auto_mode_off_service': '',
            'inspection_services': [''],
            'inspection_timeout_sec': 10.0,
            'controller': 'pure_pursuit',
            'pure_pursuit.lookahead_distance': 1.0,
            'pure_pursuit.goal_tolerance': 0.3,
            'pid.lookahead_distance': 1.0,
            'pid.goal_tolerance': 0.3,
            'pid.kp': 1.5,
            'pid.ki': 0.0,
            'pid.kd': 0.2,
            'mppi.goal_tolerance': 0.3,
            'mppi.horizon_steps': 15,
            'mppi.dt': 0.1,
            'mppi.num_samples': 200,
            'mppi.angular_std': 1.0,
            'mppi.temperature': 1.0,
            'mppi.window_points': 60,
            'speed_regulator_enabled': False,
            'speed_regulator.kp': 0.3,
            'speed_regulator.ki': 0.0,
            'speed_regulator.kd': 0.0,
            'speed_regulator.min_scale': 0.3,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def optional_client(self, name: str):
        """Create a Trigger client, or None when the service name is empty."""
        return self.create_client(Trigger, name, callback_group=self._srv_group) if name else None

    def build_controller_params(self, name: str) -> dict:
        """Collect the constructor kwargs for a controller name from its declared parameters."""
        p = self.get_parameter
        params = {}
        if name == 'pure_pursuit':
            params['lookahead_distance'] = float(p('pure_pursuit.lookahead_distance').value)
            params['goal_tolerance'] = float(p('pure_pursuit.goal_tolerance').value)
        elif name == 'pid':
            params['lookahead_distance'] = float(p('pid.lookahead_distance').value)
            params['goal_tolerance'] = float(p('pid.goal_tolerance').value)
            params['kp'] = float(p('pid.kp').value)
            params['ki'] = float(p('pid.ki').value)
            params['kd'] = float(p('pid.kd').value)
        elif name == 'mppi':
            params['goal_tolerance'] = float(p('mppi.goal_tolerance').value)
            params['horizon_steps'] = int(p('mppi.horizon_steps').value)
            params['dt'] = float(p('mppi.dt').value)
            params['num_samples'] = int(p('mppi.num_samples').value)
            params['angular_std'] = float(p('mppi.angular_std').value)
            params['temperature'] = float(p('mppi.temperature').value)
            params['window_points'] = int(p('mppi.window_points').value)
            params['max_angular_velocity'] = self._max_angular_velocity
        return params

    def switch_controller(self, name: str):
        """Instantiate the requested controller, carrying over the current path if any."""
        new_controller = create_controller(name, self.build_controller_params(name))
        if self._waypoints:
            new_controller.set_path([(x, y) for x, y, _, _ in self._waypoints])
        self._controller = new_controller
        self._controller_name = name
        self.get_logger().info(f'switched controller to {name}')

    def on_parameters_changed(self, params):
        """Apply runtime changes to avoidance, speed regulator, loop and controller selection."""
        for param in params:
            if param.name == 'avoidance_enabled':
                self._avoidance_enabled = bool(param.value)
            elif param.name == 'speed_regulator_enabled':
                self._speed_regulator_enabled = bool(param.value)
            elif param.name == 'loop_route':
                self._loop_route = bool(param.value)
            elif param.name == 'controller':
                try:
                    self.switch_controller(param.value)
                except ValueError as error:
                    return SetParametersResult(successful=False, reason=str(error))
        return SetParametersResult(successful=True)

    def find_next_stop_index(self, from_index: int):
        """Index of the next waypoint with dwell_sec > 0 at or after from_index, or None."""
        for i in range(from_index, len(self._waypoints)):
            if self._waypoints[i][3] > 0.0:
                return i
        return None

    def restart_route(self, from_start: bool = False):
        """Reset controller progress and the stop-point state machine to the start of the route."""
        self._controller.set_path([(x, y) for x, y, _, _ in self._waypoints], 0 if from_start else None)
        self._speed_regulator.reset()
        self._follower_state = 'FOLLOWING'
        self._next_stop_index = self.find_next_stop_index(0)

    def waypoints_callback(self, msg: WaypointPath):
        """Load a new route; identical republished routes are ignored so progress is kept."""
        waypoints = [(w.x, w.y, w.yaw, w.dwell_sec) for w in msg.waypoints]
        if waypoints == self._waypoints:
            return
        self._waypoints = waypoints
        self._speed_profile = velocity_profile(
            [(w[0], w[1]) for w in waypoints],
            [i for i, w in enumerate(waypoints) if w[3] > 0.0],
            cruise=self._target_linear_velocity,
            min_speed=self._min_linear_velocity,
            max_lateral_accel=self._max_lateral_accel,
            decel=self._decel_limit,
            approach_speed=self._approach_speed,
        )
        self._lap = 0
        self.restart_route()
        stops = sum(1 for w in waypoints if w[3] > 0.0)
        self.get_logger().info(f'route loaded: {len(waypoints)} waypoints, {stops} stop points')

    def avoidance_callback(self, msg: AvoidanceCommand):
        """Cache the latest avoidance signal and its arrival time."""
        self._avoidance = msg
        self._avoidance_time = self.get_clock().now()

    def call_trigger(self, client, timeout: float = 2.0):
        """Call a Trigger service and wait for it (requires the multi-threaded executor)."""
        if client is None:
            return True, 'skipped'
        if not client.wait_for_service(timeout_sec=timeout):
            return False, f'{client.srv_name} not available'
        done = threading.Event()
        future = client.call_async(Trigger.Request())
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout) or future.result() is None:
            return False, f'{client.srv_name} timed out'
        return future.result().success, future.result().message

    def start_nav_callback(self, request, response):
        """Enable or disable navigation, switching the robot's control mode along with it."""
        if request.data:
            ok, message = self.call_trigger(self._auto_on_client)
            if not ok:
                response.success = False
                response.message = f'failed to enable auto mode: {message}'
                self.get_logger().error(response.message)
                return response
            self._nav_active = True
            response.success = True
            response.message = 'navigation started'
            self.get_logger().info(response.message)
            return response

        self._nav_active = False
        self.publish_stop(hard=True)
        ok, message = self.call_trigger(self._auto_off_client)
        if not ok:
            self.get_logger().error(f'failed to switch back to manual mode: {message}')
        response.success = True
        response.message = 'navigation stopped'
        self.get_logger().info(response.message)
        return response

    def current_pose(self):
        """Look up map->base_link; None if unavailable or older than tf_timeout_sec."""
        try:
            t = self._tf_buffer.lookup_transform('map', 'base_link', Time())
        except TransformException as error:
            self.get_logger().warn(f'TF map->base_link failed: {error}', throttle_duration_sec=2.0)
            return None
        stamp = Time.from_msg(t.header.stamp)
        if stamp.nanoseconds > 0 and self.get_clock().now() - stamp > self._tf_timeout:
            self.get_logger().warn('TF map->base_link is stale (odometry stopped?)', throttle_duration_sec=2.0)
            return None
        p = t.transform.translation
        return Pose2D(p.x, p.y, yaw_from_quaternion(t.transform.rotation))

    def avoidance_signal(self):
        """Return (steering_bias, velocity_scale, emergency, stale) from the latest avoidance message."""
        if not self._avoidance_enabled:
            return 0.0, 1.0, False, False
        stale = (self._avoidance_time is None
                 or self.get_clock().now() - self._avoidance_time > self._avoidance_timeout)
        a = self._avoidance
        return a.steering_bias, a.velocity_scale, a.emergency, stale

    def publish_status(self, stamp, status, message):
        """Publish NavigationStatus, logging once per state transition."""
        if status != self._last_status:
            log = self.get_logger().warn if status in self._WARN_STATES else self.get_logger().info
            log(f'[{status}] {message}')
        self._last_status = status
        msg = NavigationStatus()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'
        msg.state = status
        msg.message = message
        self._status_pub.publish(msg)

    def publish_markers(self, stamp, status, message, pose_x, pose_y, debug=None):
        """Publish the status and the controller debug MarkerArray."""
        self.publish_status(stamp, status, message)
        debug = debug or {}
        markers = MarkerArray()
        items = [visualization.status_text_marker(
            'map', stamp, pose_x, pose_y, f'{status}\n{message}', visualization.status_color(status))]
        if debug.get('lookahead_xy') is not None:
            items.append(visualization.lookahead_marker('map', stamp, *debug['lookahead_xy']))
        if debug.get('nearest_xy') is not None:
            items.append(visualization.nearest_point_marker('map', stamp, *debug['nearest_xy']))
        if debug.get('curvature') is not None:
            items.append(visualization.curvature_arc_marker('base_link', stamp, debug['curvature'], self._arc_length))
        if debug.get('rollout_xy') is not None:
            items.append(visualization.rollout_marker('map', stamp, debug['rollout_xy']))
        for m in items:
            m.lifetime = self._marker_lifetime
            markers.markers.append(m)
        self._markers_pub.publish(markers)

    def publish_stop_event(self, event: str, message: str = ''):
        """Publish a StopPointEvent for the current stop point."""
        x, y, yaw_deg, dwell = self._waypoints[self._next_stop_index]
        msg = StopPointEvent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.index = self._next_stop_index
        msg.x, msg.y, msg.yaw_deg, msg.dwell_sec = float(x), float(y), float(yaw_deg), float(dwell)
        msg.event = event
        msg.message = message
        self._stop_event_pub.publish(msg)

    def start_inspection(self):
        """Fire every inspection Trigger service asynchronously."""
        self._inspection_results = []
        self._inspection_pending = 0
        for client in self._inspection_clients:
            if not client.service_is_ready():
                self._inspection_results.append(f'{client.srv_name}: not available')
                continue
            self._inspection_pending += 1
            future = client.call_async(Trigger.Request())
            future.add_done_callback(lambda f, name=client.srv_name: self.inspection_done(name, f))

    def inspection_done(self, name: str, future):
        """Collect one inspection result."""
        result = future.result()
        text = f'{name}: {"ok" if result and result.success else "fail"} {result.message if result else ""}'
        self._inspection_results.append(text.strip())
        self._inspection_pending = max(0, self._inspection_pending - 1)
        if self._inspection_pending == 0 and self._next_stop_index is not None:
            self.publish_stop_event('inspected', ' | '.join(self._inspection_results))

    def run_approaching(self, pose: Pose2D, stamp):
        """Converge exactly onto the target x, y, yaw, then dwell (stop point) or hold (route end)."""
        x, y, yaw_deg, _ = self._waypoints[self._approach_index]
        cmd = self._approach.update(pose, x, y, math.radians(yaw_deg))
        elapsed = (self.get_clock().now() - self._approach_start_time).nanoseconds / 1e9
        if cmd.done or elapsed > self._approach_timeout:
            self.publish_stop(hard=True)
            note = '' if cmd.done else f'approach timeout, error {cmd.position_error:.2f} m'
            if note:
                self.get_logger().warn(note)
            if self._approach_is_goal:
                self._follower_state = 'GOAL_HOLD'
                return
            self._follower_state = 'DWELLING'
            self._dwell_start_time = self.get_clock().now()
            self.publish_stop_event('arrived', note)
            self.start_inspection()
            self.publish_markers(stamp, 'DWELLING', f'stop {self._next_stop_index}, inspecting', pose.x, pose.y)
            return

        _, _, emergency, stale = self.avoidance_signal()
        if emergency or stale:
            self.publish_stop(hard=True)
            self.publish_markers(stamp, 'EMERGENCY_STOP', 'obstacle during final approach', pose.x, pose.y)
            return

        target = 'goal' if self._approach_is_goal else f'stop {self._approach_index}'
        out = Twist()
        out.linear.x = cmd.vx
        out.linear.y = cmd.vy
        out.angular.z = cmd.wz
        self._linear_limiter.reset(cmd.vx)
        self._angular_limiter.reset(cmd.wz)
        self._cmd_vel_pub.publish(out)
        self.publish_markers(
            stamp, 'APPROACHING',
            f'{target}: {cmd.position_error * 100:.1f} cm, '
            f'{math.degrees(cmd.yaw_error):.1f} deg', pose.x, pose.y)

    def run_dwelling(self, pose: Pose2D, stamp):
        """Hold until dwell time passed and inspections finished (or timed out), then move on."""
        self.publish_stop(hard=True)
        elapsed = (self.get_clock().now() - self._dwell_start_time).nanoseconds / 1e9
        waiting = self._inspection_pending > 0 and elapsed < self._dwell_duration + self._inspection_timeout
        if elapsed < self._dwell_duration or waiting:
            remaining = max(0.0, self._dwell_duration - elapsed)
            note = f', {self._inspection_pending} inspection(s) pending' if self._inspection_pending else ''
            self.publish_markers(stamp, 'DWELLING', f'resuming in {remaining:.1f}s{note}', pose.x, pose.y)
            return

        if self._inspection_pending:
            self.publish_stop_event('inspected', 'timeout: ' + ' | '.join(self._inspection_results))
            self._inspection_pending = 0
        self.publish_stop_event('departed')
        self._follower_state = 'FOLLOWING'
        self._next_stop_index = self.find_next_stop_index(self._next_stop_index + 1)

    def check_stop_point(self, pose: Pose2D, stamp) -> bool:
        """Enter APPROACHING near the next stop point, or skip it if progress already passed it."""
        if self._next_stop_index is None:
            return False
        sx, sy, _, sdwell = self._waypoints[self._next_stop_index]
        if math.hypot(sx - pose.x, sy - pose.y) < self._approach_radius:
            self._dwell_duration = sdwell
            self.start_approach(self._next_stop_index, is_goal=False)
            self.publish_markers(stamp, 'APPROACHING', f'stop point {self._next_stop_index}', pose.x, pose.y)
            return True
        if self._controller.progress_index() > self._next_stop_index + self._stop_point_skip_margin:
            self.get_logger().warn(f'stop point {self._next_stop_index} missed, skipping')
            self.publish_stop_event('skipped', 'passed without reaching tolerance')
            self._next_stop_index = self.find_next_stop_index(self._next_stop_index + 1)
        return False

    def start_approach(self, index: int, is_goal: bool):
        """Switch to the final approach toward waypoint index."""
        self._follower_state = 'APPROACHING'
        self._approach_index = index
        self._approach_is_goal = is_goal
        self._approach.reset()
        self._approach_start_time = self.get_clock().now()

    def handle_route_end(self):
        """At the route end: start the next lap on a closed loop, else approach the last point and hold."""
        first_x, first_y = self._waypoints[0][0], self._waypoints[0][1]
        last_x, last_y = self._waypoints[-1][0], self._waypoints[-1][1]
        closed = math.hypot(first_x - last_x, first_y - last_y) < self._loop_close_distance
        if self._loop_route and closed:
            self._lap += 1
            self.get_logger().info(f'lap {self._lap} complete, restarting route')
            self.restart_route(from_start=True)
            return
        self.start_approach(len(self._waypoints) - 1, is_goal=True)

    def control_loop(self):
        """Run one control tick: safety checks, stop-point state machine, then path following."""
        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds / 1e9
        self._last_time = now
        if dt <= 0.0 or dt > 1.0:
            dt = 1.0 / self._control_rate
        stamp = now.to_msg()

        if not self._nav_active:
            self.publish_markers(stamp, 'NAV_INACTIVE', "call 'navigation/start_nav' to begin", 0.0, 0.0)
            return
        if not self._waypoints:
            self.publish_stop(hard=True)
            self.publish_markers(stamp, 'NO_PATH', 'waiting for navigation/waypoints', 0.0, 0.0)
            return

        pose = self.current_pose()
        if pose is None:
            self.publish_stop(hard=True)
            self.publish_markers(stamp, 'TF_UNAVAILABLE', 'waiting for fresh map->base_link', 0.0, 0.0)
            return

        if self._follower_state == 'GOAL_HOLD':
            self.publish_stop(hard=True)
            self.publish_markers(stamp, 'GOAL_REACHED', 'holding position at the route end', pose.x, pose.y)
            return
        if self._follower_state == 'APPROACHING':
            self.run_approaching(pose, stamp)
            return
        if self._follower_state == 'DWELLING':
            self.run_dwelling(pose, stamp)
            return

        steering_bias, velocity_scale, emergency, stale = self.avoidance_signal()
        if stale:
            self.publish_stop(hard=True)
            self.publish_markers(stamp, 'AVOIDANCE_STALE', 'no fresh navigation/avoidance, stopped', pose.x, pose.y)
            return

        if self._controller.is_finished(pose):
            self.handle_route_end()
            return
        if self.check_stop_point(pose, stamp):
            return

        index = min(self._controller.progress_index(), len(self._speed_profile) - 1)
        target_speed = self._speed_profile[index] if self._speed_profile else self._target_linear_velocity
        output = self._controller.update(pose, dt, target_speed)
        target_angular = output.angular + steering_bias
        target_linear = output.linear * velocity_scale
        if self._speed_regulator_enabled:
            target_linear *= self._speed_regulator.scale_for(target_angular, dt)

        cmd = Twist()
        if emergency:
            self.publish_stop(hard=True)
        else:
            cmd.linear.x = self._linear_limiter.step(clamp(target_linear, self._max_linear_velocity), dt)
            cmd.angular.z = self._angular_limiter.step(clamp(target_angular, self._max_angular_velocity), dt)
            self._cmd_vel_pub.publish(cmd)

        status = 'EMERGENCY_STOP' if emergency else 'FOLLOWING'
        message = (
            f'lap {self._lap} | {self._controller_name} | v_scale {velocity_scale:.2f} | '
            f'next stop {self._next_stop_index}'
        )
        self.publish_markers(stamp, status, message, pose.x, pose.y, self._controller.debug_info())

    def publish_stop(self, hard: bool = False):
        """Publish zero velocity; hard skips the slew ramp entirely."""
        if hard:
            self._linear_limiter.reset()
            self._angular_limiter.reset()
            self._cmd_vel_pub.publish(Twist())
            return
        cmd = Twist()
        cmd.linear.x = self._linear_limiter.step(0.0, 1.0 / self._control_rate)
        cmd.angular.z = self._angular_limiter.step(0.0, 1.0 / self._control_rate)
        self._cmd_vel_pub.publish(cmd)


def main(args=None):
    """Spin the path follower node on a multi-threaded executor."""
    rclpy.init(args=args)
    node = PathFollowerNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
