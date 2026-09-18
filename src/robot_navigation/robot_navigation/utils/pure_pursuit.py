import math

from robot_navigation.utils.pose2d import Pose2D


class PurePursuit:
    """Tracks a path of x,y points and outputs steering curvature toward a lookahead point."""

    def __init__(self, lookahead_distance: float = 1.0, goal_tolerance: float = 0.3):
        self._lookahead_distance = lookahead_distance
        self._goal_tolerance = goal_tolerance
        self._path: list[tuple[float, float]] = []
        self._index_nearest = 0
        self._last_lookahead = (0.0, 0.0)

    def set_path(self, path_xy: list[tuple[float, float]]):
        """Replace the tracked path and reset the nearest-point search index."""
        self._path = path_xy
        self._index_nearest = 0

    def set_lookahead_distance(self, distance: float):
        """Update the lookahead distance, floored to avoid a degenerate zero-length lookahead."""
        self._lookahead_distance = max(0.05, distance)

    def lookahead_distance(self) -> float:
        """Return the currently configured lookahead distance."""
        return self._lookahead_distance

    def last_lookahead_point(self) -> tuple[float, float]:
        """Return the lookahead point computed by the most recent update() call."""
        return self._last_lookahead

    def nearest_point(self) -> tuple[float, float]:
        """Return the path point at the current nearest-index."""
        if not self._path:
            return 0.0, 0.0
        return self._path[self._index_nearest]

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

    def update(self, pose: Pose2D) -> float:
        """Advance tracking to the current pose and return the target curvature (1/m, +left)."""
        if not self._path:
            return 0.0

        self._index_nearest = self.nearest_index(pose)
        gx, gy = self.lookahead_point(pose, self._index_nearest)
        self._last_lookahead = (gx, gy)

        dx, dy = gx - pose.x, gy - pose.y
        local_x = math.cos(-pose.yaw) * dx - math.sin(-pose.yaw) * dy
        local_y = math.sin(-pose.yaw) * dx + math.cos(-pose.yaw) * dy

        lookahead_sq = local_x * local_x + local_y * local_y
        if lookahead_sq < 1e-6:
            return 0.0
        return 2.0 * local_y / lookahead_sq
