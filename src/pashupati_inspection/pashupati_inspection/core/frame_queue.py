from __future__ import annotations

from collections import deque
import contextlib
import json
import os
import threading
import time

import numpy as np

THUMB_SIZE = (32, 24)  # width, height of the picture compared to the last queued one


def thumbnail(img: np.ndarray) -> np.ndarray:
    """Return a small grey copy of a BGR (or grey) image, for frame_change."""
    import cv2

    grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
    return cv2.resize(grey, THUMB_SIZE, interpolation=cv2.INTER_AREA).astype(np.float32)


def frame_change(thumb: np.ndarray, last) -> float:
    """
    Return how much the picture changed: the largest grey-level change of a thumbnail cell.

    The largest, not the mean: a piece of trash appearing changes a few cells a lot and the
    rest not at all. Camera noise averages out in the cells (below 3 levels).
    """
    if last is None or last.shape != thumb.shape:
        return 255.0
    return float(np.max(np.abs(thumb - last)))


class FrameQueue:
    """
    Frames waiting for the model, first in first out, at most `limit` (the oldest goes first).

    Kept in `directory` (JPEG plus a JSON of its metadata per frame) so that frames survive a
    restart; an empty directory keeps them in memory only.
    """

    def __init__(self, directory: str, limit: int):
        self._dir = os.path.expanduser(directory) if directory else ''
        self.limit = max(int(limit), 1)
        self.dropped = 0  # frames pushed out by newer ones because the queue was full
        self._lock = threading.Lock()
        self._items: deque = deque()  # (name, meta, jpeg or None when on disk), oldest first
        self._seq = 0
        if self._dir:
            os.makedirs(self._dir, exist_ok=True)
            self.load()

    def load(self):
        """Take over the frames a previous run left in the directory, oldest first."""
        files: dict = {}
        for entry in os.listdir(self._dir):
            if entry.endswith('.tmp'):  # cut off while writing
                os.remove(os.path.join(self._dir, entry))
                continue
            name, ext = os.path.splitext(entry)
            files.setdefault(name, set()).add(ext)
        for name in sorted(files):
            meta = None
            if files[name] == {'.jpg', '.json'}:
                try:
                    with open(self.path(name, '.json')) as f:
                        meta = json.load(f)
                except (OSError, ValueError):
                    pass
            if not isinstance(meta, dict):
                self.delete(name)
                continue
            self._items.append((name, meta, None))
        while len(self._items) > self.limit:
            self.drop_oldest()

    def path(self, name: str, ext: str) -> str:
        """Return the file of a queued frame."""
        return os.path.join(self._dir, name + ext)

    def delete(self, name: str):
        """Remove a frame's files, ignoring missing ones."""
        for ext in ('.jpg', '.json'):
            with contextlib.suppress(OSError):
                os.remove(self.path(name, ext))

    def drop_oldest(self):
        """Forget the oldest frame (lock held)."""
        name, _, _ = self._items.popleft()
        if self._dir:
            self.delete(name)
        self.dropped += 1

    def put(self, jpeg: bytes, meta: dict) -> str:
        """Queue a frame behind the others; returns its name."""
        with self._lock:
            while len(self._items) >= self.limit:
                self.drop_oldest()
            self._seq += 1
            # names sort in arrival order, also across restarts
            name = f'{time.time_ns():020d}-{self._seq:06d}'
            if self._dir:
                for ext, data in (('.jpg', jpeg), ('.json', json.dumps(meta).encode())):
                    partial = self.path(name, ext + '.tmp')
                    with open(partial, 'wb') as f:
                        f.write(data)
                    os.replace(partial, self.path(name, ext))
                self._items.append((name, meta, None))
            else:
                self._items.append((name, meta, jpeg))
            return name

    def peek(self):
        """Return (name, meta, jpeg) of the oldest frame without removing it, or None."""
        with self._lock:
            while self._items:
                name, meta, jpeg = self._items[0]
                if jpeg is not None:
                    return name, meta, jpeg
                try:
                    with open(self.path(name, '.jpg'), 'rb') as f:
                        return name, meta, f.read()
                except OSError:  # removed under us: skip it
                    self._items.popleft()
                    self.delete(name)
            return None

    def remove(self, name: str):
        """Remove a frame once it has been inspected."""
        with self._lock:
            for item in list(self._items):
                if item[0] == name:
                    self._items.remove(item)
                    break
            if self._dir:
                self.delete(name)

    def __len__(self) -> int:
        """Return the number of waiting frames."""
        with self._lock:
            return len(self._items)
