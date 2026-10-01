#!/usr/bin/env python3
import math
import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, Trigger
from visualization_msgs.msg import MarkerArray

from rcl_interfaces.msg import SetParametersResult

from tf2_ros import Buffer, TransformException, TransformListener

from robot_interfaces.msg import AvoidanceCommand, NavigationStatus, WaypointPath

from robot_navigation.utils import visualization
from robot_navigation.utils.controllers.pid_controller import angle_diff
from robot_navigation.utils.controllers.registry import create_controller
from robot_navigation.utils.pose2d import Pose2D
from robot_navigation.utils.speed_regulator import SpeedRegulator


def yaw_from_quaternion(q) -> float:
    """Extract the yaw angle from a geometry_msgs Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class SlewLimiter:
    """Caps how fast a value can change per second, for smooth accel/turn-rate limiting."""

    def __init__(self, max_rate: float):
        self._max_rate = max_rate
        self._value = 0.0

    def step(self, target: float, dt: float) -> float:
        """Move value toward target by at most max_rate * dt and return the new value."""
        max_delta = self._max_rate * dt
        delta = max(-max_delta, min(max_delta, target - self._value))
        self._value += delta
        return self._value

    def reset(self, value: float = 0.0):
        """Jump straight to value, bypassing the rate limit (hard stop)."""
        self._value = value


class PathFollowerNode(Node):
    """Drives the robot along a received waypoint path using a selectable controller, blended
    with avoidance, pausing to align and dwell at inspection stop points. A single SetBool
    service starts/stops it -- the path is a dense, continuous trajectory rather than a
    handful of discrete goals, so a plain always-on control loop is a better fit here than
    a goal/feedback/result action."""

    _WARN_STATES = {'NO_PATH', 'TF_UNAVAILABLE', 'EMERGENCY_STOP', 'AVOIDANCE_STALE'}

    def __init__(self):
        """Declare params, set up TF, build the initial controller, and wire subscriptions/outputs."""
        super().__init__('path_follower_node')
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('target_linear_velocity', 0.4)
        self.declare_parameter('max_linear_velocity', 0.6)
        self.declare_parameter('max_angular_velocity', 1.0)
        self.declare_parameter('linear_accel_limit', 0.5)
        self.declare_parameter('angular_accel_limit', 1.5)
        self.declare_parameter('arc_visualization_length', 2.0)
        self.declare_parameter('marker_lifetime_sec', 0.5)
        self.declare_parameter('avoidance_enabled', True)
        self.declare_parameter('avoidance_timeout_sec', 0.5)
        self.declare_parameter('tf_timeout_sec', 0.5)
        self.declare_parameter('auto_mode_on_service', '')
        self.declare_parameter('auto_mode_off_service', '')

        self.declare_parameter('stop_point_tolerance', 0.3)
        self.declare_parameter('yaw_tolerance_deg', 5.0)
        self.declare_parameter('align_kp', 1.5)

        self.declare_parameter('controller', 'pure_pursuit')
        self.declare_parameter('pure_pursuit.lookahead_distance', 1.0)
        self.declare_parameter('pure_pursuit.goal_tolerance', 0.3)
        self.declare_parameter('pid.lookahead_distance', 1.0)
        self.declare_parameter('pid.goal_tolerance', 0.3)
        self.declare_parameter('pid.kp', 1.5)
        self.declare_parameter('pid.ki', 0.0)
        self.declare_parameter('pid.kd', 0.2)
        self.declare_parameter('mppi.goal_tolerance', 0.3)
        self.declare_parameter('mppi.horizon_steps', 15)
        self.declare_parameter('mppi.dt', 0.1)
        self.declare_parameter('mppi.num_samples', 200)
        self.declare_parameter('mppi.angular_std', 1.0)
        self.declare_parameter('mppi.temperature', 0.05)
        self.declare_parameter('mppi.window_points', 60)

        self.declare_parameter('speed_regulator_enabled', False)
        self.declare_parameter('speed_regulator.kp', 0.3)
        self.declare_parameter('speed_regulator.ki', 0.0)
        self.declare_parameter('speed_regulator.kd', 0.0)
        self.declare_parameter('speed_regulator.min_scale', 0.3)

        self._control_rate = float(self.get_parameter('control_rate').value)
        self._target_linear_velocity = float(self.get_parameter('target_linear_velocity').value)
        self._max_linear_velocity = float(self.get_parameter('max_linear_velocity').value)
        self._max_angular_velocity = float(self.get_parameter('max_angular_velocity').value)
        self._arc_length = float(self.get_parameter('arc_visualization_length').value)
        self._marker_lifetime = Duration(seconds=float(self.get_parameter('marker_lifetime_sec').value)).to_msg()
        self._avoidance_enabled = bool(self.get_parameter('avoidance_enabled').value)
        self._avoidance_timeout = Duration(seconds=float(self.get_parameter('avoidance_timeout_sec').value))
        self._tf_timeout = Duration(seconds=float(self.get_parameter('tf_timeout_sec').value))
        self._speed_regulator_enabled = bool(self.get_parameter('speed_regulator_enabled').value)

        self._stop_point_tolerance = float(self.get_parameter('stop_point_tolerance').value)
        self._yaw_tolerance = math.radians(float(self.get_parameter('yaw_tolerance_deg').value))
        self._align_kp = float(self.get_parameter('align_kp').value)

        self._linear_limiter = SlewLimiter(float(self.get_parameter('linear_accel_limit').value))
        self._angular_limiter = SlewLimiter(float(self.get_parameter('angular_accel_limit').value))
        self._speed_regulator = SpeedRegulator(
            kp=float(self.get_parameter('speed_regulator.kp').value),
            ki=float(self.get_parameter('speed_regulator.ki').value),
            kd=float(self.get_parameter('speed_regulator.kd').value),
            min_scale=float(self.get_parameter('speed_regulator.min_scale').value),
        )

        self._avoidance_steering_bias = 0.0
        self._avoidance_velocity_scale = 1.0
        self._avoidance_emergency = False
        self._avoidance_time = None
        self._have_path = False
        self._nav_active = False
        self._current_path_points: list[tuple[float, float]] = []

        self._waypoints: list[tuple[float, float, float, float]] = []
        self._next_stop_index = None
        self._follower_state = 'FOLLOWING'
        self._align_target_yaw = 0.0
        self._dwell_duration = 0.0
        self._dwell_start_time = None

        self._last_status = 'NAV_INACTIVE'
        self._last_status_message = ''

        self._controller_name = self.get_parameter('controller').value
        self._controller = create_controller(self._controller_name, self.build_controller_params(self._controller_name))

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        latched = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(WaypointPath, 'navigation/waypoints', self.waypoints_callback, latched)
        self.create_subscription(AvoidanceCommand, 'navigation/avoidance', self.avoidance_callback, 10)
        self._cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self._markers_pub = self.create_publisher(MarkerArray, 'navigation/markers', 10)
        self._status_pub = self.create_publisher(NavigationStatus, 'navigation/status', 10)

        self._srv_group = ReentrantCallbackGroup()
        self._auto_on_client = self.optional_client(self.get_parameter('auto_mode_on_service').value)
        self._auto_off_client = self.optional_client(self.get_parameter('auto_mode_off_service').value)
        self.create_service(SetBool, 'navigation/start_nav', self.start_nav_callback,
                            callback_group=self._srv_group)
        self.add_on_set_parameters_callback(self.on_parameters_changed)

        self._last_time = self.get_clock().now()
        self.create_timer(1.0 / self._control_rate, self.control_loop)

    def build_controller_params(self, name: str) -> dict:
        """Collect the constructor kwargs for a controller name from its declared parameters."""
        params = {'target_linear_velocity': self._target_linear_velocity}
        if name == 'pure_pursuit':
            params['lookahead_distance'] = float(self.get_parameter('pure_pursuit.lookahead_distance').value)
            params['goal_tolerance'] = float(self.get_parameter('pure_pursuit.goal_tolerance').value)
        elif name == 'pid':
            params['lookahead_distance'] = float(self.get_parameter('pid.lookahead_distance').value)
            params['goal_tolerance'] = float(self.get_parameter('pid.goal_tolerance').value)
            params['kp'] = float(self.get_parameter('pid.kp').value)
            params['ki'] = float(self.get_parameter('pid.ki').value)
            params['kd'] = float(self.get_parameter('pid.kd').value)
        elif name == 'mppi':
            params['goal_tolerance'] = float(self.get_parameter('mppi.goal_tolerance').value)
            params['horizon_steps'] = int(self.get_parameter('mppi.horizon_steps').value)
            params['dt'] = float(self.get_parameter('mppi.dt').value)
            params['num_samples'] = int(self.get_parameter('mppi.num_samples').value)
            params['angular_std'] = float(self.get_parameter('mppi.angular_std').value)
            params['temperature'] = float(self.get_parameter('mppi.temperature').value)
            params['window_points'] = int(self.get_parameter('mppi.window_points').value)
            params['max_angular_velocity'] = self._max_angular_velocity
        return params

    def switch_controller(self, name: str):
        """Instantiate the requested controller, carrying over the current path if any."""
        new_controller = create_controller(name, self.build_controller_params(name))
        if self._have_path:
            new_controller.set_path(self._current_path_points)
        self._controller = new_controller
        self._controller_name = name
        self.get_logger().info(f'switched controller to {name}')

    def on_parameters_changed(self, params):
        """Apply runtime changes to avoidance_enabled, speed_regulator_enabled, and controller selection."""
        for param in params:
            if param.name == 'avoidance_enabled':
                self._avoidance_enabled = bool(param.value)
            elif param.name == 'speed_regulator_enabled':
                self._speed_regulator_enabled = bool(param.value)
            elif param.name == 'controller':
                try:
                    self.switch_controller(param.value)
                except ValueError as error:
                    return SetParametersResult(successful=False, reason=str(error))
        return SetParametersResult(successful=True)

    def find_next_stop_index(self, from_index: int):
        """Return the index of the next waypoint with dwell_sec > 0 at or after from_index, or None."""
        for i in range(from_index, len(self._waypoints)):
            if self._waypoints[i][3] > 0.0:
                return i
        return None

    def waypoints_callback(self, msg: WaypointPath):
        """Load a new waypoint path; identical republished paths are ignored so progress is kept."""
        waypoints = [(w.x, w.y, w.yaw, w.dwell_sec) for w in msg.waypoints]
        if waypoints == self._waypoints:
            return
        self._waypoints = waypoints
        self._current_path_points = [(x, y) for x, y, _, _ in self._waypoints]
        self._controller.set_path(self._current_path_points)
        self._speed_regulator.reset()
        self._have_path = len(self._waypoints) > 0
        self._follower_state = 'FOLLOWING'
        self._next_stop_index = self.find_next_stop_index(0)

    def avoidance_callback(self, msg: AvoidanceCommand):
        """Cache the latest avoidance signal for blending into the control loop."""
        self._avoidance_steering_bias = msg.steering_bias
        self._avoidance_velocity_scale = msg.velocity_scale
        self._avoidance_emergency = msg.emergency
        self._avoidance_time = self.get_clock().now()

    def optional_client(self, name: str):
        """Create a Trigger client, or None when the service name is empty."""
        return self.create_client(Trigger, name, callback_group=self._srv_group) if name else None

    def call_trigger(self, client, timeout: float = 2.0):
        """Call a Trigger service and wait for it without blocking the executor."""
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
        """Enable or disable navigation, switching the robot control mode along with it."""
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
        """Look up the robot's current map->base_link pose, or None if TF isn't ready."""
        try:
            t = self._tf_buffer.lookup_transform('map', 'base_link', Time())
        except TransformException as error:
            self.get_logger().warn(f'TF lookup map->base_link failed: {error}', throttle_duration_sec=2.0)
            return None
        stamp = Time.from_msg(t.header.stamp)
        if stamp.nanoseconds > 0 and self.get_clock().now() - stamp > self._tf_timeout:
            self.get_logger().warn('TF map->base_link is stale (odometry stopped?)', throttle_duration_sec=2.0)
            return None
        p = t.transform.translation
        return Pose2D(p.x, p.y, yaw_from_quaternion(t.transform.rotation))

    def publish_status(self, stamp, status, message):
        """Publish the current follower state as a NavigationStatus, logging once per
        state transition (not every tick) so the console and the GUI both show clear,
        readable events -- "navigation started/ended", "no tf", "no path", etc -- instead
        of staying silent or spamming at the 20Hz control rate."""
        if status != self._last_status:
            log = self.get_logger().warn if status in self._WARN_STATES else self.get_logger().info
            log(f'[{status}] {message}')

        self._last_status = status
        self._last_status_message = message
        msg = NavigationStatus()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'
        msg.state = status
        msg.message = message
        self._status_pub.publish(msg)

    def publish_markers(self, stamp, status, message, pose_x, pose_y, lookahead_xy=None, nearest_xy=None, curvature=None, rollout_xy=None):
        """Build and publish the path controller debug MarkerArray, and the plain NavigationStatus."""
        self.publish_status(stamp, status, message)
        markers = MarkerArray()
        color = visualization.status_color(status)

        text_marker = visualization.status_text_marker('map', stamp, pose_x, pose_y, f'{status}\n{message}', color)
        text_marker.lifetime = self._marker_lifetime
        markers.markers.append(text_marker)

        if lookahead_xy is not None:
            m = visualization.lookahead_marker('map', stamp, lookahead_xy[0], lookahead_xy[1])
            m.lifetime = self._marker_lifetime
            markers.markers.append(m)

        if nearest_xy is not None:
            m = visualization.nearest_point_marker('map', stamp, nearest_xy[0], nearest_xy[1])
            m.lifetime = self._marker_lifetime
            markers.markers.append(m)

        if curvature is not None:
            m = visualization.curvature_arc_marker('base_link', stamp, curvature, self._arc_length)
            m.lifetime = self._marker_lifetime
            markers.markers.append(m)

        if rollout_xy is not None:
            m = visualization.rollout_marker('map', stamp, rollout_xy)
            m.lifetime = self._marker_lifetime
            markers.markers.append(m)

        self._markers_pub.publish(markers)

    def run_aligning(self, pose: Pose2D, dt: float, stamp):
        """Rotate in place toward the stop point's recorded yaw; transition to DWELLING once aligned."""
        error = angle_diff(self._align_target_yaw, pose.yaw)

        if abs(error) < self._yaw_tolerance:
            self._follower_state = 'DWELLING'
            self._dwell_start_time = self.get_clock().now()
            self.publish_stop()
            self.publish_markers(stamp, 'DWELLING', f'holding for {self._dwell_duration:.1f}s', pose.x, pose.y)
            return

        angular = max(-self._max_angular_velocity, min(self._max_angular_velocity, self._align_kp * error))
        cmd = Twist()
        cmd.angular.z = self._angular_limiter.step(angular, dt)
        self._cmd_vel_pub.publish(cmd)
        self.publish_markers(stamp, 'ALIGNING', f'heading error {math.degrees(error):.1f} deg', pose.x, pose.y)

    def run_dwelling(self, pose: Pose2D, stamp):
        """Hold position until dwell_duration has elapsed, then resume following past this stop point."""
        self.publish_stop()
        elapsed = (self.get_clock().now() - self._dwell_start_time).nanoseconds / 1e9
        remaining = max(0.0, self._dwell_duration - elapsed)
        self.publish_markers(stamp, 'DWELLING', f'resuming in {remaining:.1f}s', pose.x, pose.y)

        if elapsed >= self._dwell_duration:
            self._follower_state = 'FOLLOWING'
            self._next_stop_index = self.find_next_stop_index(self._next_stop_index + 1)

    def control_loop(self):
        """Compute and publish the blended controller + avoidance cmd_vel, or run the stop-point state machine."""
        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds / 1e9
        self._last_time = now
        if dt <= 0.0:
            dt = 1.0 / self._control_rate
        stamp = now.to_msg()

        if not self._nav_active:
            self.publish_markers(stamp, 'NAV_INACTIVE', "call service 'navigation/start_nav' to begin", 0.0, 0.0)
            return

        if not self._have_path:
            self.publish_stop(hard=True)
            self.publish_markers(stamp, 'NO_PATH', 'waiting for a path on navigation/waypoints', 0.0, 0.0)
            return

        pose = self.current_pose()
        if pose is None:
            self.publish_stop(hard=True)
            self.publish_markers(stamp, 'TF_UNAVAILABLE', 'waiting for map->base_link TF', 0.0, 0.0)
            return

        if self._follower_state == 'ALIGNING':
            self.run_aligning(pose, dt, stamp)
            return

        if self._follower_state == 'DWELLING':
            self.run_dwelling(pose, stamp)
            return

        if self._controller.is_finished(pose):
            self.publish_stop()
            self.publish_markers(stamp, 'GOAL_REACHED', 'holding position', pose.x, pose.y)
            return

        if self._next_stop_index is not None:
            sx, sy, syaw_deg, sdwell = self._waypoints[self._next_stop_index]
            if math.hypot(sx - pose.x, sy - pose.y) < self._stop_point_tolerance:
                self._follower_state = 'ALIGNING'
                self._align_target_yaw = math.radians(syaw_deg)
                self._dwell_duration = sdwell
                self.publish_stop()
                self.publish_markers(stamp, 'ALIGNING', f'reached stop point {self._next_stop_index}', pose.x, pose.y)
                return

        if self._avoidance_enabled and (
                self._avoidance_time is None
                or self.get_clock().now() - self._avoidance_time > self._avoidance_timeout):
            self.publish_stop(hard=True)
            self.publish_markers(stamp, 'AVOIDANCE_STALE', 'no fresh navigation/avoidance, stopped', pose.x, pose.y)
            return

        controller_output = self._controller.update(pose, dt)

        steering_bias = self._avoidance_steering_bias if self._avoidance_enabled else 0.0
        velocity_scale = self._avoidance_velocity_scale if self._avoidance_enabled else 1.0
        emergency = self._avoidance_emergency if self._avoidance_enabled else False

        target_angular = controller_output.angular + steering_bias
        target_linear = controller_output.linear * velocity_scale

        if self._speed_regulator_enabled:
            target_linear *= self._speed_regulator.scale_for(target_angular, dt)

        target_linear = max(-self._max_linear_velocity, min(self._max_linear_velocity, target_linear))
        target_angular = max(-self._max_angular_velocity, min(self._max_angular_velocity, target_angular))

        if emergency:
            self.publish_stop(hard=True)
        else:
            cmd = Twist()
            cmd.linear.x = self._linear_limiter.step(target_linear, dt)
            cmd.angular.z = self._angular_limiter.step(target_angular, dt)
            self._cmd_vel_pub.publish(cmd)

        status = 'EMERGENCY_STOP' if emergency else 'FOLLOWING'
        message = (
            f'controller {self._controller_name} | v_scale {velocity_scale:.2f} | '
            f'avoidance {"on" if self._avoidance_enabled else "off"} | '
            f'speed_reg {"on" if self._speed_regulator_enabled else "off"}'
        )
        debug = self._controller.debug_info()
        self.publish_markers(
            stamp, status, message, pose.x, pose.y,
            lookahead_xy=debug.get('lookahead_xy'),
            nearest_xy=debug.get('nearest_xy'),
            curvature=debug.get('curvature'),
            rollout_xy=debug.get('rollout_xy'),
        )

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
    """Spin the path follower node."""
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