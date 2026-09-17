#!/usr/bin/env python3
import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from std_srvs.srv import SetBool
from visualization_msgs.msg import MarkerArray

from rcl_interfaces.msg import SetParametersResult

from tf2_ros import Buffer, TransformListener

from robot_interfaces.msg import AvoidanceCommand

from robot_navigation.utils import visualization
from robot_navigation.utils.pure_pursuit import Pose2D, PurePursuit


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
    """Drives the robot along a received path using pure pursuit, blended with avoidance input."""

    def __init__(self):
        """Declare params, set up TF, and wire the path/avoidance subscriptions and cmd_vel output."""
        super().__init__('path_follower_node')
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('lookahead_distance', 1.0)
        self.declare_parameter('goal_tolerance', 0.3)
        self.declare_parameter('target_linear_velocity', 0.4)
        self.declare_parameter('max_linear_velocity', 0.6)
        self.declare_parameter('max_angular_velocity', 1.0)
        self.declare_parameter('linear_accel_limit', 0.5)
        self.declare_parameter('angular_accel_limit', 1.5)
        self.declare_parameter('arc_visualization_length', 2.0)
        self.declare_parameter('marker_lifetime_sec', 0.5)
        self.declare_parameter('avoidance_enabled', True)

        self._control_rate = float(self.get_parameter('control_rate').value)
        self._target_linear_velocity = float(self.get_parameter('target_linear_velocity').value)
        self._max_linear_velocity = float(self.get_parameter('max_linear_velocity').value)
        self._max_angular_velocity = float(self.get_parameter('max_angular_velocity').value)
        self._arc_length = float(self.get_parameter('arc_visualization_length').value)
        self._marker_lifetime = Duration(seconds=float(self.get_parameter('marker_lifetime_sec').value)).to_msg()
        self._avoidance_enabled = bool(self.get_parameter('avoidance_enabled').value)

        self._pure_pursuit = PurePursuit(
            lookahead_distance=float(self.get_parameter('lookahead_distance').value),
            goal_tolerance=float(self.get_parameter('goal_tolerance').value),
        )
        self._linear_limiter = SlewLimiter(float(self.get_parameter('linear_accel_limit').value))
        self._angular_limiter = SlewLimiter(float(self.get_parameter('angular_accel_limit').value))

        self._avoidance_steering_bias = 0.0
        self._avoidance_velocity_scale = 1.0
        self._avoidance_emergency = False
        self._have_path = False
        self._goal_logged = False
        self._nav_active = False

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(Path, 'navigation/path', self.path_callback, 10)
        self.create_subscription(AvoidanceCommand, 'navigation/avoidance', self.avoidance_callback, 10)
        self._cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self._markers_pub = self.create_publisher(MarkerArray, 'navigation/markers', 10)

        self._auto_mode_client = self.create_client(SetBool, 'drivers/set_auto_mode')
        self.create_service(SetBool, 'navigation/start_nav', self.start_nav_callback)
        self.add_on_set_parameters_callback(self.on_parameters_changed)

        self._last_time = self.get_clock().now()
        self.create_timer(1.0 / self._control_rate, self.control_loop)

    def on_parameters_changed(self, params):
        """Apply a runtime change to avoidance_enabled immediately."""
        for param in params:
            if param.name == 'avoidance_enabled':
                self._avoidance_enabled = bool(param.value)
        return SetParametersResult(successful=True)

    def path_callback(self, msg: Path):
        """Load a new path into the pure pursuit tracker."""
        points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self._pure_pursuit.set_path(points)
        self._have_path = len(points) > 0
        self._goal_logged = False

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

    def start_nav_callback(self, request, response):
        """Enable or disable navigation, gating driver auto mode along with it."""
        if request.data:
            success, message = self.call_set_auto_mode(True)
            if not success:
                self.get_logger().error(f'failed to enable auto mode: {message}')
                response.success = False
                response.message = f'failed to enable auto mode: {message}'
                return response

            self._nav_active = True
            response.success = True
            response.message = 'navigation started'
            return response

        self._nav_active = False
        self.publish_stop()

        success, message = self.call_set_auto_mode(False)
        if not success:
            self.get_logger().error(f'failed to switch back to manual mode: {message}')

        response.success = True
        response.message = 'navigation stopped, switched to manual mode'
        return response

    def current_pose(self):
        """Look up the robot's current map->base_link pose, or None if TF isn't ready."""
        try:
            t = self._tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception as error:
            self.get_logger().warn(f'TF lookup map->base_link failed: {error}', throttle_duration_sec=2.0)
            return None
        p = t.transform.translation
        return Pose2D(p.x, p.y, yaw_from_quaternion(t.transform.rotation))

    def publish_markers(self, stamp, status, message, pose_x, pose_y, lookahead_xy=None, nearest_xy=None, curvature=None):
        """Build and publish the pure pursuit debug MarkerArray for the current control tick."""
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

        self._markers_pub.publish(markers)

    def control_loop(self):
        """Compute and publish the blended pure-pursuit + avoidance cmd_vel at a fixed rate."""
        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds / 1e9
        self._last_time = now
        if dt <= 0.0:
            dt = 1.0 / self._control_rate
        stamp = now.to_msg()

        if not self._nav_active:
            self.publish_markers(stamp, 'NAV_INACTIVE', 'call navigation/start_nav to begin', 0.0, 0.0)
            return

        if not self._have_path:
            self.publish_stop()
            self.publish_markers(stamp, 'NO_PATH', 'waiting for a path on navigation/path', 0.0, 0.0)
            return

        pose = self.current_pose()
        if pose is None:
            self.publish_stop()
            self.publish_markers(stamp, 'TF_UNAVAILABLE', 'waiting for map->base_link TF', 0.0, 0.0)
            return

        if self._pure_pursuit.is_finished(pose):
            if not self._goal_logged:
                self.get_logger().info('Goal reached, holding position.')
                self._goal_logged = True
            self.publish_stop()
            self.publish_markers(stamp, 'GOAL_REACHED', 'holding position', pose.x, pose.y)
            return

        curvature = self._pure_pursuit.update(pose)

        steering_bias = self._avoidance_steering_bias if self._avoidance_enabled else 0.0
        velocity_scale = self._avoidance_velocity_scale if self._avoidance_enabled else 1.0
        emergency = self._avoidance_emergency if self._avoidance_enabled else False

        target_linear = 0.0 if emergency else self._target_linear_velocity * velocity_scale
        target_angular = target_linear * curvature + steering_bias

        target_linear = max(-self._max_linear_velocity, min(self._max_linear_velocity, target_linear))
        target_angular = max(-self._max_angular_velocity, min(self._max_angular_velocity, target_angular))

        cmd = Twist()
        cmd.linear.x = self._linear_limiter.step(target_linear, dt)
        cmd.angular.z = self._angular_limiter.step(target_angular, dt)
        self._cmd_vel_pub.publish(cmd)

        status = 'EMERGENCY_STOP' if emergency else 'FOLLOWING'
        message = (
            f'curvature {curvature:.2f} 1/m | lookahead {self._pure_pursuit.lookahead_distance():.2f} m | '
            f'v_scale {velocity_scale:.2f} | avoidance {"on" if self._avoidance_enabled else "off"}'
        )
        self.publish_markers(
            stamp, status, message, pose.x, pose.y,
            lookahead_xy=self._pure_pursuit.last_lookahead_point(),
            nearest_xy=self._pure_pursuit.nearest_point(),
            curvature=curvature,
        )

    def publish_stop(self):
        """Ramp linear and angular velocity down to zero and publish."""
        cmd = Twist()
        cmd.linear.x = self._linear_limiter.step(0.0, 1.0 / self._control_rate)
        cmd.angular.z = self._angular_limiter.step(0.0, 1.0 / self._control_rate)
        self._cmd_vel_pub.publish(cmd)


def main(args=None):
    """Spin the path follower node."""
    rclpy.init(args=args)
    node = PathFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()