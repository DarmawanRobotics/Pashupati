"""The inspection queue: first in first out, bounded, kept across restarts."""

import numpy as np
import pytest
from robot_inspection.frame_queue import frame_change, FrameQueue, thumbnail


@pytest.fixture(params=['disk', 'memory'])
def make_queue(request, tmp_path):
    """Return a function making a queue on disk (tmp_path) or in memory."""
    directory = str(tmp_path / 'queue') if request.param == 'disk' else ''
    return lambda limit=5: FrameQueue(directory, limit)


def test_first_in_first_out(make_queue):
    """Frames come out in the order they went in, and only once removed."""
    queue = make_queue()
    for i in range(3):
        queue.put(bytes([i]), {'at': float(i)})
    assert len(queue) == 3
    seen = []
    while (item := queue.peek()) is not None:
        name, meta, jpeg = item
        assert queue.peek()[0] == name  # peeking does not take it
        seen.append((meta['at'], jpeg))
        queue.remove(name)
    assert seen == [(0.0, b'\x00'), (1.0, b'\x01'), (2.0, b'\x02')]
    assert len(queue) == 0


def test_a_full_queue_drops_the_oldest(make_queue):
    """At the limit a new frame pushes out the oldest one."""
    queue = make_queue(limit=2)
    for i in range(4):
        queue.put(bytes([i]), {'at': float(i)})
    assert len(queue) == 2 and queue.dropped == 2
    assert queue.peek()[1] == {'at': 2.0}


def test_frames_survive_a_restart(tmp_path):
    """A new queue on the same directory continues where the old one stopped."""
    directory = str(tmp_path / 'queue')
    first = FrameQueue(directory, 10)
    for i in range(3):
        first.put(bytes([i]), {'at': float(i)})
    first.remove(first.peek()[0])
    (tmp_path / 'queue' / 'half.jpg.tmp').write_bytes(b'x')  # a write cut off by a power loss
    (tmp_path / 'queue' / 'lonely.jpg').write_bytes(b'x')  # its JSON never written
    again = FrameQueue(directory, 10)
    assert len(again) == 2
    assert again.peek()[1:] == ({'at': 1.0}, b'\x01')
    assert sorted(p.name for p in (tmp_path / 'queue').iterdir()) == sorted(
        f'{name}{ext}' for name, _, _ in again._items for ext in ('.jpg', '.json')
    )
    assert len(FrameQueue(directory, 1)) == 1  # a smaller limit keeps the newest


def test_change_between_frames():
    """A still scene is no change; something new in a corner is."""
    floor = np.full((480, 640, 3), 120, np.uint8)
    noisy = np.clip(floor + np.random.default_rng(1).normal(0, 4, floor.shape), 0, 255)
    trash = floor.copy()
    trash[400:440, 600:640] = 20  # 40 x 40 px near a corner
    assert frame_change(thumbnail(noisy.astype(np.uint8)), thumbnail(floor)) < 3.0
    assert frame_change(thumbnail(trash), thumbnail(floor)) > 30.0
    assert frame_change(thumbnail(floor), None) == 255.0
