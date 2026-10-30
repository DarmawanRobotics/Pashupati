"""Where the robot was: a short history, so a late finding gets the pose of its frame."""

from __future__ import annotations

from collections import deque

HISTORY_SEC = 900.0  # an inspection queued in interval mode can be reported this late
TOLERANCE_SEC = 5.0


class PoseHistory:
    """(time, pose) once per telemetry tick, the last HISTORY_SEC."""

    def __init__(self, keep_sec: float = HISTORY_SEC):
        self.keep = keep_sec
        self.items: deque = deque()

    def add(self, when: float, pose: dict | None):
        """Remember a pose (None: not localized, nothing kept) and forget old ones."""
        if pose is not None:
            self.items.append((when, pose))
        while self.items and when - self.items[0][0] > self.keep:
            self.items.popleft()

    def at(self, when: float, tolerance: float = TOLERANCE_SEC):
        """Return the pose closest in time to when, None when none is that close."""
        return pose_at(self.items, when, tolerance)


def pose_at(history, when: float, tolerance: float = TOLERANCE_SEC):
    """Return the recorded pose closest in time to when, or None when none is that close."""
    best = min(history, key=lambda entry: abs(entry[0] - when), default=None)
    return best[1] if best is not None and abs(best[0] - when) <= tolerance else None
