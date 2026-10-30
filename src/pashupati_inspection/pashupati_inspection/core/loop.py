"""
The inspection worker loop (no ROS).

It keeps the model loaded, inspects back to back or works through the interval queue, and frees
the model when inspection is switched off.
"""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time

from pashupati_inspection.core.engine import PERIODIC, InspectionEngine
from pashupati_inspection.core.frame_queue import FrameQueue, frame_change, thumbnail
from pashupati_inspection.core.images import decode_jpeg, fit_width, jpeg

EVIDENCE_QUALITY = 90
MODEL_QUALITY = 85


@dataclass(frozen=True)
class Frame:
    """A camera frame: BGR image, camera stamp, frame id, when it was taken (time.time)."""

    image: object
    sec: int
    nanosec: int
    frame_id: str
    taken_at: float

    def meta(self) -> dict:
        """Return what the queue keeps of the frame."""
        return {'sec': self.sec, 'nanosec': self.nanosec, 'frame_id': self.frame_id,
                'at': self.taken_at}  # fmt: skip

    @classmethod
    def from_queue(cls, meta: dict, data: bytes) -> Frame | None:
        """Return a queued frame, None when it is unreadable."""
        image = decode_jpeg(data)
        try:
            stamp = (int(meta['sec']), int(meta['nanosec']))
            frame = cls(image, *stamp, str(meta.get('frame_id', '')), float(meta['at']))
        except (KeyError, TypeError, ValueError):
            return None
        return frame if image is not None else None


class InspectionLoop:
    """The worker thread's loop; the node gives it the latest frame and publishes verdicts."""

    def __init__(self, engine: InspectionEngine, queue: FrameQueue, latest, on_verdict, log):
        self.engine = engine
        self.queue = queue
        self.latest = latest  # () -> Frame | None, fresh frames only
        self.on_verdict = on_verdict  # (verdict, frame, evidence jpeg) after a report
        self.log = log  # (level, message)
        self.stop = threading.Event()
        self.wake = threading.Event()  # a live parameter changed: re-check at once
        self.next_capture = 0.0
        self.last_thumb = None
        self.unchanged = 0  # frames not queued: the picture was the same as the last one

    def pause(self, seconds: float):
        """Wait, but wake up at once when a live parameter changes or the node stops."""
        self.wake.wait(seconds)
        self.wake.clear()

    def inspect(self, source: str, wait: float, frame: Frame | None = None, data: bytes = b''):
        """Inspect the latest (or a queued) frame; return (verdict or None, message)."""
        if frame is None:
            frame = self.latest()
            if frame is None:
                self.engine.set_state('error', 'no recent camera frame')
                return None, 'no recent camera frame'
        image = fit_width(frame.image, self.engine.config.max_image_width)
        verdict, message = self.engine.inspect(
            jpeg(image, MODEL_QUALITY), source, frame.taken_at, wait
        )
        if verdict is not None and verdict.report:
            self.on_verdict(verdict, frame, data or jpeg(frame.image, EVIDENCE_QUALITY))
        return verdict, message

    def capture(self) -> bool:
        """Interval mode: queue the latest frame every interval_sec unless nothing changed."""
        config = self.engine.config
        now = time.monotonic()
        if not config.enabled or config.interval_sec <= 0.0 or now < self.next_capture:
            return False
        frame = self.latest()
        if frame is None:
            return False
        self.next_capture = now + config.interval_sec
        thumb = thumbnail(frame.image)
        if config.queue_min_change > 0.0 and (
            frame_change(thumb, self.last_thumb) < config.queue_min_change
        ):
            self.unchanged += 1  # standing still: the same floor again
            return False
        self.last_thumb = thumb
        self.queue.put(jpeg(frame.image, EVIDENCE_QUALITY), frame.meta())
        self.wake.set()
        return True

    def step(self):
        """One round of the worker loop."""
        engine, config = self.engine, self.engine.config
        if not config.enabled:
            if engine.unload():  # off frees the model's memory; on loads it again
                self.log('info', 'model unloaded, inspection off')
            engine.set_state('off')
            self.pause(5.0)
        elif engine.loaded != config.model:
            if not engine.load(config.model) and not self.stop.is_set():
                if engine.state == 'error':
                    self.log('warning', engine.error)
                self.pause(10.0)  # retry, or load another model as soon as one is chosen
        elif len(self.queue):  # interval mode, or frames left from it: oldest first
            self.inspect_queued()
            self.pause(0.05)
        elif config.interval_sec != 0.0:  # capture() fills the queue; < 0: stop points only
            engine.set_state('running')
            self.pause(5.0)
        else:
            verdict, message = self.inspect(PERIODIC, wait=0.5)
            if verdict is None and not self.stop.is_set():
                self.log('warning', message)
                self.pause(1.0)
            self.pause(0.05)  # room for a stop point's check between periodic ones

    def inspect_queued(self):
        """Inspect the oldest queued frame; it stays queued when the model could not take it."""
        item = self.queue.peek()
        if item is None:
            return
        name, meta, data = item
        frame = Frame.from_queue(meta, data)
        if frame is None:
            self.log('warning', f'dropping unreadable queued frame {name}')
            self.queue.remove(name)
            return
        verdict, message = self.inspect(PERIODIC, 0.5, frame, data)
        if verdict is not None:
            self.queue.remove(name)
        elif not self.stop.is_set():
            self.log('warning', f'queued frame waits: {message}')
            self.pause(1.0)

    def run(self):
        """Loop until stop is set."""
        while not self.stop.is_set():
            self.step()
