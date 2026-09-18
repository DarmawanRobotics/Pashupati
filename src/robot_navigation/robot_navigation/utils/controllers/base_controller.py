from dataclasses import dataclass

from robot_navigation.utils.pose2d import Pose2D


@dataclass
class ControllerOutput:
    """Nominal (pre-avoidance) linear and angular velocity command from a path controller."""
    linear: float = 0.0
    angular: float = 0.0


class PathController:
    """Common interface every path-following controller implements."""

    def set_path(self, path_xy: list[tuple[float, float]]):
        """Replace the tracked path."""
        raise NotImplementedError

    def update(self, pose: Pose2D, dt: float) -> ControllerOutput:
        """Compute the nominal linear/angular command for the current pose."""
        raise NotImplementedError

    def is_finished(self, pose: Pose2D) -> bool:
        """Return True once the robot is close enough to the path's last point."""
        raise NotImplementedError

    def debug_info(self) -> dict:
        """Return controller-specific debug values for visualization (all keys optional)."""
        return {}
