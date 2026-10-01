import math

from robot_navigation.utils.pose2d import Pose2D


def wrap_angle(a: float) -> float:
    """Wrap an angle into (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


class PathProgress:
    """Monotonic progress along a polyline path, robust to overlapping or looping routes."""

    def __init__(self, search_window: int = 200, heading_weight: float = 1.0):
        self._search_window = search_window
        self._heading_weight = heading_weight
        self._path: list[tuple[float, float]] = []
        self._index = 0
        self._initialized = False

    def set_path(self, path_xy: list[tuple[float, float]], start_index=None):
        """Replace the path; without start_index the next update searches the whole path."""
        self._path = list(path_xy)
        self._index = start_index or 0
        self._initialized = start_index is not None

    @property
    def path(self) -> list[tuple[float, float]]:
        """The tracked path."""
        return self._path

    @property
    def index(self) -> int:
        """Current nearest path index."""
        return self._index

    def segment_heading(self, i: int) -> float:
        """Heading of the path segment starting at index i."""
        j = min(i + 1, len(self._path) - 1)
        k = max(j - 1, 0)
        return math.atan2(self._path[j][1] - self._path[k][1], self._path[j][0] - self._path[k][0])

    def score(self, i: int, pose: Pose2D) -> float:
        """Distance to point i plus a penalty for driving against the segment direction."""
        d = math.hypot(self._path[i][0] - pose.x, self._path[i][1] - pose.y)
        heading_err = abs(wrap_angle(self.segment_heading(i) - pose.yaw))
        return d + self._heading_weight * (heading_err / math.pi)

    def update(self, pose: Pose2D) -> int:
        """Advance the nearest index (global search on first call, forward window afterwards)."""
        if not self._path:
            return 0
        if not self._initialized:
            self._index = min(range(len(self._path)), key=lambda i: self.score(i, pose))
            self._initialized = True
            return self._index
        end = min(len(self._path), self._index + self._search_window)
        self._index = min(range(self._index, end), key=lambda i: self.score(i, pose))
        return self._index

    def lookahead_point(self, pose: Pose2D, distance: float) -> tuple[float, float]:
        """First path point at least `distance` away, walking forward from the current index."""
        for i in range(self._index, len(self._path)):
            if math.hypot(self._path[i][0] - pose.x, self._path[i][1] - pose.y) >= distance:
                return self._path[i]
        return self._path[-1]

    def window(self, count: int) -> list[tuple[float, float]]:
        """Up to `count` path points starting at the current index."""
        return self._path[self._index:self._index + count]

    def is_finished(self, pose: Pose2D, tolerance: float, end_margin: int = 3) -> bool:
        """True when progress reached the last points and the robot is within tolerance of the goal."""
        if not self._path:
            return True
        if self._index < len(self._path) - 1 - end_margin:
            return False
        gx, gy = self._path[-1]
        return math.hypot(gx - pose.x, gy - pose.y) < tolerance
