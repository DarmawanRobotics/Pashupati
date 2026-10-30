"""What person_detector_node reports: its state, the boxes for the handheld, the rate (no ROS)."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import os
import time

NO_CAMERA_SEC = 3.0  # without frames for this long the status says so
AUTO_RATE_GPU_HZ = 5.0  # rate_hz 0: smooth boxes on the GPU
AUTO_RATE_CPU_HZ = 2.0  # rate_hz 0: light on the CPU
MAX_BOXES = 20


def detection_rate(rate_hz: float, backend: str) -> float:
    """Return detections per second: rate_hz, or with 0 by where the model runs."""
    if rate_hz > 0.0:
        return rate_hz
    on_gpu = backend.endswith('cuda') or backend == 'ultralytics'
    return AUTO_RATE_GPU_HZ if on_gpu else AUTO_RATE_CPU_HZ


def box_fractions(boxes: list[tuple], width: int, height: int) -> list[list[float]]:
    """Return [x1, y1, x2, y2, score] as fractions of the image, at most MAX_BOXES."""
    return [
        [round(x1 / width, 3), round(y1 / height, 3), round(x2 / width, 3),
         round(y2 / height, 3), round(score, 2)]
        for x1, y1, x2, y2, score in boxes[:MAX_BOXES]
    ]  # fmt: skip


@dataclass
class PeopleState:
    """The detector's state between frames."""

    enabled: bool = True
    model_path: str = ''
    image_topic: str = ''
    backend: str = ''
    load_error: str = ''
    frame_time: float = -math.inf  # monotonic time of the last camera frame
    result: dict = field(default_factory=dict)  # count, boxes, placed, latency, time

    def state(self, now: float, loaded: bool) -> tuple[str, str]:
        """Return (state, error): off, error, loading or running."""
        if not self.enabled:
            return 'off', ''
        if self.load_error:
            return 'error', self.load_error
        if self.frame_time == -math.inf:
            return 'loading', ''  # the camera has not sent a frame yet
        if now - self.frame_time > NO_CAMERA_SEC:
            return 'error', f'no camera images on {self.image_topic}'
        return ('running', '') if loaded else ('loading', '')

    def detected(self, boxes: list[tuple], width: int, height: int, placed: int, latency: float):
        """Keep the result of one detection."""
        self.result = {
            'count': len(boxes),
            'boxes': box_fractions(boxes, width, height),
            'placed': placed,
            'latency_ms': int(latency * 1000),
            'at': time.time(),
        }

    def status(self, rate_hz: float, loaded: bool, now: float | None = None) -> dict:
        """Return perception/people_status (the handheld shows it, NETRA gets a summary)."""
        state, error = self.state(time.monotonic() if now is None else now, loaded)
        result = self.result
        return {
            'enabled': self.enabled,
            'state': state,
            'error': error,
            'backend': self.backend,
            'model': os.path.basename(self.model_path),
            'count': result.get('count'),
            'boxes': result.get('boxes', []),
            'placed': result.get('placed'),
            'latency_ms': result.get('latency_ms'),
            'at': result.get('at'),
            'rate_hz': round(detection_rate(rate_hz, self.backend), 2),
        }
