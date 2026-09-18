import math

import numpy as np

from robot_navigation.utils.controllers.base_controller import ControllerOutput, PathController
from robot_navigation.utils.pose2d import Pose2D


class MppiController(PathController):
    """Simplified Model Predictive Path Integral controller: samples angular-velocity sequences
    at a fixed cruise speed, rolls them out with a unicycle model, and returns a cost-weighted
    average of the first control (softmax over path-tracking cost, warm-started each tick)."""

    def __init__(
        self,
        target_linear_velocity=0.4,
        goal_tolerance=0.3,
        horizon_steps=15,
        dt=0.1,
        num_samples=200,
        angular_std=1.0,
        temperature=1.0,
        max_angular_velocity=1.0,
    ):
        self._target_linear_velocity = target_linear_velocity
        self._goal_tolerance = goal_tolerance
        self._horizon_steps = horizon_steps
        self._rollout_dt = dt
        self._num_samples = num_samples
        self._angular_std = angular_std
        self._temperature = temperature
        self._max_angular_velocity = max_angular_velocity
        self._path = np.zeros((0, 2))
        self._last_angular = 0.0
        self._last_rollout = None

    def set_path(self, path_xy):
        """Replace the tracked path as a numpy array for vectorized distance queries."""
        self._path = np.array(path_xy) if path_xy else np.zeros((0, 2))

    def is_finished(self, pose: Pose2D) -> bool:
        """Return True once the robot is within goal_tolerance of the path's last point."""
        if self._path.shape[0] == 0:
            return True
        gx, gy = self._path[-1]
        return math.hypot(gx - pose.x, gy - pose.y) < self._goal_tolerance

    def path_cost(self, xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
        """Sum, over the horizon, of each rolled-out point's squared distance to its nearest path point."""
        dx = xs[:, :, None] - self._path[None, None, :, 0]
        dy = ys[:, :, None] - self._path[None, None, :, 1]
        nearest_sq = (dx * dx + dy * dy).min(axis=2)
        return nearest_sq.sum(axis=1)

    def update(self, pose: Pose2D, dt: float) -> ControllerOutput:
        """Sample control sequences, roll them out, and return a cost-weighted-average command."""
        if self._path.shape[0] == 0:
            return ControllerOutput(0.0, 0.0)

        angular_samples = np.random.normal(self._last_angular, self._angular_std, size=(self._num_samples, self._horizon_steps))
        angular_samples = np.clip(angular_samples, -self._max_angular_velocity, self._max_angular_velocity)

        x = np.full(self._num_samples, pose.x)
        y = np.full(self._num_samples, pose.y)
        yaw = np.full(self._num_samples, pose.yaw)
        xs = np.zeros((self._num_samples, self._horizon_steps))
        ys = np.zeros((self._num_samples, self._horizon_steps))

        for step in range(self._horizon_steps):
            yaw = yaw + angular_samples[:, step] * self._rollout_dt
            x = x + self._target_linear_velocity * np.cos(yaw) * self._rollout_dt
            y = y + self._target_linear_velocity * np.sin(yaw) * self._rollout_dt
            xs[:, step] = x
            ys[:, step] = y

        costs = self.path_cost(xs, ys) + 0.01 * np.sum(angular_samples ** 2, axis=1)
        weights = np.exp(-costs / self._temperature)
        weights /= np.sum(weights) + 1e-9

        best_angular_sequence = np.sum(weights[:, None] * angular_samples, axis=0)
        self._last_angular = float(best_angular_sequence[0])
        self._last_rollout = (xs[np.argmax(weights)].tolist(), ys[np.argmax(weights)].tolist())

        return ControllerOutput(linear=self._target_linear_velocity, angular=self._last_angular)

    def debug_info(self) -> dict:
        """Expose the best-scoring rollout for visualization, once one has been computed."""
        if self._last_rollout is None:
            return {}
        xs, ys = self._last_rollout
        return {'rollout_xy': list(zip(xs, ys))}
