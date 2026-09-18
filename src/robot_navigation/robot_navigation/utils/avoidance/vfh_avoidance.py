import math

from robot_navigation.utils.avoidance.base_avoidance import AvoidanceAlgorithm, AvoidanceResult


class VfhAvoidance(AvoidanceAlgorithm):
    """Simplified Vector Field Histogram: finds the widest safe gap in the sector scan and
    steers toward the gap closest to straight ahead, since no desired heading is given here."""

    def __init__(
        self,
        safe_distance: float = 3.0,
        emergency_distance: float = 0.4,
        emergency_cone_deg: float = 60.0,
        min_gap_width_deg: float = 20.0,
        steering_gain: float = 1.0,
        velocity_gain: float = 1.0,
        smoothing: float = 0.85,
    ):
        self._safe_distance = safe_distance
        self._emergency_distance = emergency_distance
        self._emergency_half_cone = math.radians(emergency_cone_deg) / 2.0
        self._min_gap_width = math.radians(min_gap_width_deg)
        self._steering_gain = steering_gain
        self._velocity_gain = velocity_gain
        self._smoothing = smoothing
        self._filtered = AvoidanceResult()

    def sector_angle(self, index: int, angle_min: float, angle_increment: float) -> float:
        """Return the center angle of a given sector index."""
        return angle_min + (index + 0.5) * angle_increment

    def find_gaps(self, blocked: list[bool]) -> list[tuple[int, int]]:
        """Return (start, end) index ranges of contiguous free (not blocked) sectors."""
        gaps = []
        start = None
        for i, is_blocked in enumerate(blocked):
            if not is_blocked and start is None:
                start = i
            elif is_blocked and start is not None:
                gaps.append((start, i - 1))
                start = None
        if start is not None:
            gaps.append((start, len(blocked) - 1))
        return gaps

    def update(self, ranges: list[float], angle_min: float, angle_increment: float, range_max: float) -> AvoidanceResult:
        """Build the obstacle histogram, pick the best gap, and steer toward its center."""
        n = len(ranges)
        if n == 0:
            return self._filtered

        blocked = []
        emergency = False
        frontal_danger_max = 0.0

        for i, r in enumerate(ranges):
            if r is None or math.isnan(r) or math.isinf(r):
                r = range_max
            theta = self.sector_angle(i, angle_min, angle_increment)
            blocked.append(r < self._safe_distance)

            if r < self._emergency_distance and abs(theta) < self._emergency_half_cone:
                emergency = True
            if abs(theta) < self._emergency_half_cone:
                danger = max(0.0, 1.0 - min(r, self._safe_distance) / self._safe_distance)
                frontal_danger_max = max(frontal_danger_max, danger)

        gaps = self.find_gaps(blocked)
        min_gap_sectors = max(1, int(self._min_gap_width / angle_increment))
        gaps = [g for g in gaps if (g[1] - g[0] + 1) >= min_gap_sectors]

        steering_bias = 0.0
        if gaps:
            best_gap = min(gaps, key=lambda g: abs(self.sector_angle((g[0] + g[1]) // 2, angle_min, angle_increment)))
            gap_center_angle = self.sector_angle((best_gap[0] + best_gap[1]) // 2, angle_min, angle_increment)
            steering_bias = self._steering_gain * gap_center_angle
        elif not emergency:
            emergency = True

        velocity_scale = max(0.0, 1.0 - self._velocity_gain * frontal_danger_max)
        raw = AvoidanceResult(steering_bias, velocity_scale, emergency)

        a = self._smoothing
        self._filtered = AvoidanceResult(
            steering_bias=a * self._filtered.steering_bias + (1 - a) * raw.steering_bias,
            velocity_scale=a * self._filtered.velocity_scale + (1 - a) * raw.velocity_scale,
            emergency=raw.emergency,
        )
        return self._filtered
