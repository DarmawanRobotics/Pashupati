from dataclasses import dataclass


@dataclass
class AvoidanceResult:
    """Output of one avoidance update: steering bias, velocity scale, and an emergency flag."""
    steering_bias: float = 0.0
    velocity_scale: float = 1.0
    emergency: bool = False


class AvoidanceAlgorithm:
    """Common interface every reactive obstacle-avoidance algorithm implements."""

    def update(self, ranges: list[float], angle_min: float, angle_increment: float, range_max: float) -> AvoidanceResult:
        """Compute a new avoidance result from one sector scan."""
        raise NotImplementedError
