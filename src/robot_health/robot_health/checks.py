from collections import deque
import glob
import os
import shutil


class RateTracker:
    """Message rate over a sliding window; drops to zero when the topic goes silent."""

    def __init__(self, window_sec: float = 5.0):
        self.window = window_sec
        self._stamps: deque = deque()

    def tick(self, now: float) -> None:
        """Record one message at time now (s)."""
        self._stamps.append(now)
        while self._stamps and now - self._stamps[0] > self.window:
            self._stamps.popleft()

    def rate(self, now: float) -> float:
        """Return messages per second, 0 when nothing arrived within the window."""
        while self._stamps and now - self._stamps[0] > self.window:
            self._stamps.popleft()
        if len(self._stamps) < 2:
            return 0.0
        span = max(self._stamps[-1] - self._stamps[0], 1e-6)
        if now - self._stamps[-1] > max(1.0, 3.0 * span / (len(self._stamps) - 1)):
            return 0.0
        return (len(self._stamps) - 1) / span


def parse_topic_specs(specs: list[str]) -> dict[str, float]:
    """Parse 'topic:min_hz' strings."""
    out = {}
    for spec in specs:
        if not spec:
            continue
        topic, _, hz = spec.rpartition(':')
        try:
            out[topic or spec] = float(hz) if topic else 0.0
        except ValueError:
            out[spec] = 0.0
    return out


def cpu_temperature() -> float | None:
    """Return the hottest thermal zone in degrees C, or None."""
    temps = []
    for path in glob.glob('/sys/class/thermal/thermal_zone*/temp'):
        try:
            with open(path) as f:
                temps.append(int(f.read().strip()) / 1000.0)
        except (OSError, ValueError):
            continue
    return max(temps) if temps else None


def memory_percent() -> float | None:
    """Return used memory in percent from /proc/meminfo, or None."""
    try:
        with open('/proc/meminfo') as f:
            info = {line.split(':')[0]: int(line.split()[1]) for line in f if ':' in line}
        return 100.0 * (1 - info['MemAvailable'] / info['MemTotal'])
    except (OSError, KeyError, ValueError, ZeroDivisionError):
        return None


def disk_free_gb(path: str) -> float | None:
    """Return free space in GB at path, or None."""
    try:
        return shutil.disk_usage(os.path.expanduser(path)).free / 1e9
    except OSError:
        return None
