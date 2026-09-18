#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
from visualization_msgs.msg import MarkerArray

from rcl_interfaces.msg import SetParametersResult

from tf2_ros import Buffer, TransformListener

from robot_interfaces.action import NavigateRoute
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


class PathFollowerNode(Node):
    """Drives the robot along a received waypoint path using a selectable controller, blended
    with avoidance, pausing to align and dwell at inspection stop points, and exposing the whole
    route as a cancellable NavigateRoute action with feedback (not just a service call)."""

    FAILURE_STATES = {'TF_UNAVAILABLE', 'NO_PATH'}

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

        self.declare_parameter('stop_point_tolerance', 0.3)
        self.declare_parameter('yaw_tolerance_deg', 5.0)
        self.declare_parameter('align_kp', 1.5)

        self.declare_parameter('action_feedback_rate', 5.0)
        self.declare_parameter('failure_state_timeout_sec', 5.0)

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
        self.declare_parameter('mppi.temperature', 1.0)

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
        self._speed_regulator_enabled = bool(self.get_parameter('speed_regulator_enabled').value)

        self._stop_point_tolerance = float(self.get_parameter('stop_point_tolerance').value)
        self._yaw_tolerance = math.radians(float(self.get_parameter('yaw_tolerance_deg').value))
        self._align_kp = float(self.get_parameter('align_kp').value)

        self._action_feedback_rate = float(self.get_parameter('action_feedback_rate').value)
        self._failure_state_timeout = float(self.get_parameter('failure_state_timeout_sec').value)

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
        self._have_path = False
        self._goal_logged = False
        self._nav_active = False
        self._current_path_points: list[tuple[float, float]] = []

        self._waypoints: list[tuple[float, float, float, float]] = []
        self._total_route_distance = 0.0
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

        self.create_subscription(WaypointPath, 'navigation/waypoints', self.waypoints_callback, 10)
        self.create_subscription(AvoidanceCommand, 'navigation/avoidance', self.avoidance_callback, 10)
        self._cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self._markers_pub = self.create_publisher(MarkerArray, 'navigation/markers', 10)
        self._status_pub = self.create_publisher(NavigationStatus, 'navigation/status', 10)

        self._auto_mode_client = self.create_client(SetBool, 'drivers/set_auto_mode')
        self.add_on_set_parameters_callback(self.on_parameters_changed)

        timer_group = MutuallyExclusiveCallbackGroup()
        action_group = ReentrantCallbackGroup()

        self._last_time = self.get_clock().now()
        self.create_timer(1.0 / self._control_rate, self.control_loop, callback_group=timer_group)

        self._action_server = ActionServer(
            self, NavigateRoute, 'navigate_route',
            execute_callback=self.execute_navigate_route,
            goal_callback=self.navigate_goal_callback,
            cancel_callback=self.navigate_cancel_callback,
            callback_group=action_group,
        )

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
        """Load a new waypoint path into the controller and reset the stop-point state machine."""
        self._waypoints = [(w.x, w.y, w.yaw, w.dwell_sec) for w in msg.waypoints]
        self._current_path_points = [(x, y) for x, y, _, _ in self._waypoints]
        self._controller.set_path(self._current_path_points)
        self._speed_regulator.reset()
        self._have_path = len(self._waypoints) > 0
        self._goal_logged = False
        self._follower_state = 'FOLLOWING'
        self._next_stop_index = self.find_next_stop_index(0)

        if len(self._waypoints) >= 2:
            fx, fy, _, _ = self._waypoints[0]
            lx, ly, _, _ = self._waypoints[-1]
            self._total_route_distance = math.hypot(lx - fx, ly - fy)
        else:
            self._total_route_distance = 0.0

    def avoidance_callback(self, msg: AvoidanceCommand):
        """Cache the latest avoidance signal for blending into the control loop."""
        self._avoidance_steering_bias = msg.steering_bias
        self._avoidance_velocity_scale = msg.velocity_scale
        self._avoidance_emergency = msg.emergency

    def call_set_auto_mode(self, enable: bool):
        """Call the driver's set_auto_mode service and return (success, message)."""
        if not self._auto_mode_client.wait_for_service(timeout_sec=2.0):
            return False, 'drivers/set_auto_mode service not available'

        driver_request = SetBool.Request()
        driver_request.data = enable
        future = self._auto_mode_client.call_async(driver_request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if future.result() is None:
            return False, 'drivers/set_auto_mode call timed out'
        return future.result().success, future.result().message

    def compute_progress(self) -> float:
        """Return an approximate 0..1 progress: 1 - remaining/total straight-line distance to the goal."""
        if not self._waypoints or self._total_route_distance <= 1e-6:
            return 0.0
        pose = self.current_pose()
        if pose is None:
            return 0.0
        gx, gy, _, _ = self._waypoints[-1]
        remaining = math.hypot(gx - pose.x, gy - pose.y)
        return max(0.0, min(1.0, 1.0 - remaining / self._total_route_distance))

    def navigate_goal_callback(self, goal_request):
        """Accept a NavigateRoute goal only if navigation isn't already running."""
        if self._nav_active:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def navigate_cancel_callback(self, goal_handle):
        """Always accept a cancel request."""
        return CancelResponse.ACCEPT

    def execute_navigate_route(self, goal_handle):
        """Drive the currently loaded route to completion, reporting feedback and supporting cancellation."""
        success, message = self.call_set_auto_mode(True)
        if not success:
            goal_handle.abort()
            return NavigateRoute.Result(success=False, message=f'failed to enable auto mode: {message}')

        self._nav_active = True
        self._goal_logged = False
        feedback_msg = NavigateRoute.Feedback()
        failure_since = None

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._nav_active = False
                self.publish_stop()
                self.call_set_auto_mode(False)
                goal_handle.canceled()
                return NavigateRoute.Result(success=False, message='navigation canceled')

            feedback_msg.state = self._last_status
            feedback_msg.message = self._last_status_message
            feedback_msg.progress = self.compute_progress()
            goal_handle.publish_feedback(feedback_msg)

            if self._last_status == 'GOAL_REACHED':
                self._nav_active = False
                self.call_set_auto_mode(False)
                goal_handle.succeed()
                return NavigateRoute.Result(success=True, message='goal reached')

            if self._last_status in self.FAILURE_STATES:
                now = self.get_clock().now()
                if failure_since is None:
                    failure_since = now
                elif (now - failure_since).nanoseconds / 1e9 > self._failure_state_timeout:
                    self._nav_active = False
                    self.call_set_auto_mode(False)
                    goal_handle.abort()
                    return NavigateRoute.Result(success=False, message=f'stuck in {self._last_status} too long')
            else:
                failure_since = None

            time.sleep(1.0 / self._action_feedback_rate)

        self._nav_active = False
        return NavigateRoute.Result(success=False, message='node shutting down')

    def current_pose(self):
        """Look up the robot's current map->base_link pose, or None if TF isn't ready."""
        try:
            t = self._tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception as error:
            self.get_logger().warn(f'TF lookup map->base_link failed: {error}', throttle_duration_sec=2.0)
            return None
        p = t.transform.translation
        return Pose2D(p.x, p.y, yaw_from_quaternion(t.transform.rotation))

    def publish_status(self, stamp, status, message):
        """Publish the current follower state as a NavigationStatus, and cache it for the action server."""
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
            self.publish_markers(stamp, 'NAV_INACTIVE', 'send a navigate_route goal to begin', 0.0, 0.0)
            return

        if not self._have_path:
            self.publish_stop()
            self.publish_markers(stamp, 'NO_PATH', 'waiting for a path on navigation/waypoints', 0.0, 0.0)
            return

        pose = self.current_pose()
        if pose is None:
            self.publish_stop()
            self.publish_markers(stamp, 'TF_UNAVAILABLE', 'waiting for map->base_link TF', 0.0, 0.0)
            return

        if self._follower_state == 'ALIGNING':
            self.run_aligning(pose, dt, stamp)
            return

        if self._follower_state == 'DWELLING':
            self.run_dwelling(pose, stamp)
            return

        if self._controller.is_finished(pose):
            if not self._goal_logged:
                self.get_logger().info('Goal reached, holding position.')
                self._goal_logged = True
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

        controller_output = self._controller.update(pose, dt)

        steering_bias = self._avoidance_steering_bias if self._avoidance_enabled else 0.0
        velocity_scale = self._avoidance_velocity_scale if self._avoidance_enabled else 1.0
        emergency = self._avoidance_emergency if self._avoidance_enabled else False

        target_angular = controller_output.angular + steering_bias
        target_linear = controller_output.linear * velocity_scale

        if self._speed_regulator_enabled:
            target_linear *= self._speed_regulator.scale_for(target_angular, dt)

        if emergency:
            target_linear = 0.0

        target_linear = max(-self._max_linear_velocity, min(self._max_linear_velocity, target_linear))
        target_angular = max(-self._max_angular_velocity, min(self._max_angular_velocity, target_angular))

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

    def publish_stop(self):
        """Ramp linear and angular velocity down to zero and publish."""
        cmd = Twist()
        cmd.linear.x = self._linear_limiter.step(0.0, 1.0 / self._control_rate)
        cmd.angular.z = self._angular_limiter.step(0.0, 1.0 / self._control_rate)
        self._cmd_vel_pub.publish(cmd)


def main(args=None):
    """Spin the path follower node on a multi-threaded executor so the action's blocking
    execute_callback never stalls the periodic control_loop timer."""
    rclpy.init(args=args)
    node = PathFollowerNode()
    executor = MultiThreadedExecutor()
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
