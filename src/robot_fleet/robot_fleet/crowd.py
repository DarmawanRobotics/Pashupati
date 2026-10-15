import math


class CrowdAggregator:
    """Turns per-frame people positions into per-cell peak counts over fixed time windows."""

    def __init__(self, cell_size: float = 1.0, window_sec: float = 60.0):
        self.cell_size = cell_size
        self.window_sec = window_sec
        self._start = None
        self._cells: dict[tuple[int, int], int] = {}
        self._peak = 0
        self._frames = 0

    def add(self, stamp: float, points: list) -> None:
        """Add one detection frame of (x, y) map positions taken at stamp (s)."""
        if self._start is None:
            self._start = stamp
        frame: dict[tuple[int, int], int] = {}
        for x, y in points:
            key = (math.floor(x / self.cell_size), math.floor(y / self.cell_size))
            frame[key] = frame.get(key, 0) + 1
        for key, count in frame.items():
            self._cells[key] = max(self._cells.get(key, 0), count)
        self._peak = max(self._peak, len(points))
        self._frames += 1

    def flush(self, now: float, force: bool = False):
        """Return the finished window as a dict and start a new one, or None if still open."""
        if self._start is None or (not force and now - self._start < self.window_sec):
            return None
        snapshot = {
            'ts_start': self._start,
            'ts_end': now,
            'cell_size': self.cell_size,
            'cells': [[ix, iy, n] for (ix, iy), n in sorted(self._cells.items())],
            'peak_people': self._peak,
            'frames': self._frames,
        }
        self._start, self._cells, self._peak, self._frames = None, {}, 0, 0
        return snapshot
