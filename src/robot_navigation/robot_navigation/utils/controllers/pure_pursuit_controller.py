from robot_navigation.utils.controllers.base_controller import ControllerOutput, PathController
from robot_navigation.utils.pose2d import Pose2D
from robot_navigation.utils.pure_pursuit import PurePursuit


class PurePursuitController(PathController):
    """Pure pursuit adapter: curvature-based steering at a constant cruise speed."""

    def __init__(self, target_linear_velocity=0.4, lookahead_distance=1.0, goal_tolerance=0.3):
        self._target_linear_velocity = target_linear_velocity
        self._pure_pursuit = PurePursuit(lookahead_distance=lookahead_distance, goal_tolerance=goal_tolerance)
        self._last_curvature = 0.0

    def set_path(self, path_xy):
        """Replace the tracked path."""
        self._pure_pursuit.set_path(path_xy)

    def update(self, pose: Pose2D, dt: float) -> ControllerOutput:
        """Compute the pure pursuit curvature and scale it by the cruise speed."""
        curvature = self._pure_pursuit.update(pose)
        self._last_curvature = curvature
        return ControllerOutput(linear=self._target_linear_velocity, angular=self._target_linear_velocity * curvature)

    def is_finished(self, pose: Pose2D) -> bool:
        """Delegate to the underlying pure pursuit tracker."""
        return self._pure_pursuit.is_finished(pose)

    def debug_info(self) -> dict:
        """Expose the lookahead point, nearest point, and curvature for visualization."""
        return {
            'lookahead_xy': self._pure_pursuit.last_lookahead_point(),
            'nearest_xy': self._pure_pursuit.nearest_point(),
            'lookahead_distance': self._pure_pursuit.lookahead_distance(),
            'curvature': self._last_curvature,
        }
