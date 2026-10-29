from __future__ import annotations

from dataclasses import replace
import time

from fakes import FakeOllama
import numpy as np
from pashupati_inspection.core.config import AnomalyConfig
from pashupati_inspection.core.engine import STOP_POINT, InspectionEngine
from pashupati_inspection.core.frame_queue import FrameQueue
from pashupati_inspection.core.loop import Frame, InspectionLoop
import pytest

from dharma_utils.config import build

pytest.importorskip('cv2')


class Camera:
    """The latest frame the node would hand the loop."""

    def __init__(self):
        self.frame = None

    def show(self, grey: int):
        image = np.full((48, 64, 3), grey, np.uint8)
        self.frame = Frame(image, 1, 2, 'camera', time.time())


def make(tmp_path, **values):
    fake = FakeOllama()
    engine = InspectionEngine(build(AnomalyConfig, values), fake)
    camera, verdicts, logs = Camera(), [], []
    loop = InspectionLoop(
        engine,
        FrameQueue(str(tmp_path / 'queue'), 50),
        lambda: camera.frame,
        lambda v, f, data: verdicts.append((v, f, data)),
        lambda level, message: logs.append((level, message)),
    )
    return loop, camera, fake, verdicts


def test_a_stop_point_inspection_reports_with_the_frame(tmp_path):
    loop, camera, fake, verdicts = make(tmp_path, interval_sec=-1.0)
    loop.step()  # loads the model
    assert loop.engine.state == 'running' and fake.loads == ['qwen3-vl:2b-instruct']
    assert loop.inspect(STOP_POINT, 0.1) == (None, 'no recent camera frame')
    camera.show(128)
    verdict, message = loop.inspect(STOP_POINT, 0.1)
    assert verdict.report and message.startswith('trash')
    (_, frame, data), = verdicts  # fmt: skip
    assert frame.sec == 1 and data[:2] == b'\xff\xd8'  # the evidence JPEG


def test_interval_frames_queue_first_in_first_out_and_still_scenes_are_skipped(tmp_path):
    loop, camera, fake, _ = make(tmp_path, interval_sec=1.0)
    for grey in (40, 120, 200, 200):
        camera.show(grey)
        loop.next_capture = 0.0
        loop.capture()
    assert len(loop.queue) == 3 and loop.unchanged == 1
    loop.step()  # load
    while len(loop.queue):
        loop.step()
    assert len(fake.asked) == 3
    assert loop.engine.reports == 1  # trash once, the cooldown holds the others


def test_switched_off_the_model_is_freed(tmp_path):
    loop, _, fake, _ = make(tmp_path, interval_sec=-1.0)
    loop.step()
    loop.engine.config = replace(loop.engine.config, enabled=False)
    loop.wake.set()
    loop.step()
    assert fake.unloads == ['qwen3-vl:2b-instruct'] and loop.engine.state == 'off'


def test_unreadable_queued_frames_are_dropped(tmp_path):
    loop, _, _, _ = make(tmp_path, interval_sec=1.0)
    loop.queue.put(b'not a jpeg', {'at': 1.0, 'sec': 0, 'nanosec': 0})
    loop.step()  # load
    loop.inspect_queued()
    assert len(loop.queue) == 0
