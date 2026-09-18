import math

from robot_navigation.utils.controllers.base_controller import ControllerOutput, PathController
from robot_navigation.utils.pose2d import Pose2D


def angle_diff(a: float, b: float) -> float:
    """Return the signed difference a-b wrapped into (-pi, pi]."""
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


class PidController(PathController):
    """Heading-error PID path follower: steers toward a lookahead point at a constant cruise speed."""

    def __init__(self, target_linear_velocity=0.4, lookahead_distance=1.0, goal_tolerance=0.3, kp=1.5, ki=0.0, kd=0.2):
        self._target_linear_velocity = target_linear_velocity
        self._lookahead_distance = lookahead_distance
        self._goal_tolerance = goal_tolerance
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._path: list[tuple[float, float]] = []
        self._index_nearest = 0
        self._integral = 0.0
        self._last_error = 0.0
        self._last_lookahead = (0.0, 0.0)

    def set_path(self, path_xy):
        """Replace the tracked path and reset the controller's integral/derivative state."""
        self._path = path_xy
        self._index_nearest = 0
        self._integral = 0.0
        self._last_error = 0.0

    def nearest_index(self, pose: Pose2D) -> int:
        """Find the closest path index searching forward only, to keep progress monotonic."""
        best_i = self._index_nearest
        best_d = math.hypot(self._path[best_i][0] - pose.x, self._path[best_i][1] - pose.y)
        search_end = min(len(self._path), self._index_nearest + 200)
        for i in range(self._index_nearest, search_end):
            d = math.hypot(self._path[i][0] - pose.x, self._path[i][1] - pose.y)
            if d < best_d:
                best_d = d
                best_i = i
        return best_i

    def lookahead_point(self, pose: Pose2D, start_index: int) -> tuple[float, float]:
        """Walk forward from start_index to the first point at least lookahead_distance away."""
        for i in range(start_index, len(self._path)):
            d = math.hypot(self._path[i][0] - pose.x, self._path[i][1] - pose.y)
            if d >= self._lookahead_distance:
                return self._path[i]
        return self._path[-1]

    def is_finished(self, pose: Pose2D) -> bool:
        """Return True once the robot is within goal_tolerance of the path's last point."""
        if not self._path:
            return True
        gx, gy = self._path[-1]
        return math.hypot(gx - pose.x, gy - pose.y) < self._goal_tolerance

    def update(self, pose: Pose2D, dt: float) -> ControllerOutput:
        """Run one PID step on the heading error toward the lookahead point."""
        if not self._path:
            return ControllerOutput(0.0, 0.0)

        self._index_nearest = self.nearest_index(pose)
        gx, gy = self.lookahead_point(pose, self._index_nearest)
        self._last_lookahead = (gx, gy)

        desired_heading = math.atan2(gy - pose.y, gx - pose.x)
        error = angle_diff(desired_heading, pose.yaw)

        safe_dt = max(dt, 1e-3)
        self._integral += error * safe_dt
        derivative = (error - self._last_error) / safe_dt
        self._last_error = error

        angular = self._kp * error + self._ki * self._integral + self._kd * derivative
        return ControllerOutput(linear=self._target_linear_velocity, angular=angular)

    def debug_info(self) -> dict:
        """Expose the lookahead point for visualization."""
        return {'lookahead_xy': self._last_lookahead, 'lookahead_distance': self._lookahead_distance}
