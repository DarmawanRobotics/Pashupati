"""
The robot's clock offset to NETRA, measured like NTP over the WebSocket.

The robot sends `time_sync` {t0} (its clock), NETRA answers {t0, t1, t2} (its clock when it
received and when it answered), the robot notes t3 on arrival. offset = ((t1 - t0) + (t2 - t3))
/ 2 is NETRA's clock minus the robot's; the round trip minus NETRA's own time bounds its error.
The sample with the shortest round trip of the last few is the best estimate.
"""

from __future__ import annotations

from collections import deque
import threading

SAMPLES = 8  # the last answers, the shortest round trip wins


class ClockOffset:
    """NETRA's clock minus the robot's, from the last time_sync answers (thread-safe)."""

    def __init__(self, samples: int = SAMPLES):
        self._samples: deque = deque(maxlen=samples)  # (round trip, offset, t3)
        self._lock = threading.Lock()

    def add(self, t0: float, t1: float, t2: float, t3: float) -> bool:
        """Add one exchange; False when its times are impossible (ignored)."""
        rtt = (t3 - t0) - (t2 - t1)
        if t3 < t0 or t2 < t1 or rtt < 0.0:
            return False
        offset = ((t1 - t0) + (t2 - t3)) / 2.0
        with self._lock:
            self._samples.append((rtt, offset, t3))
        return True

    def best(self) -> tuple[float, float] | None:
        """Return (offset s, round trip s) of the shortest round trip, None before any answer."""
        with self._lock:
            if not self._samples:
                return None
            rtt, offset, _ = min(self._samples)
        return offset, rtt

    def report(self, now: float, max_age_sec: float) -> dict | None:
        """Return the telemetry `clock` {offset_ms, rtt_ms, age_sec}, None when none is recent."""
        with self._lock:
            recent = [s for s in self._samples if now - s[2] <= max_age_sec]
        if not recent:
            return None
        rtt, offset, at = min(recent)
        return {
            'offset_ms': round(offset * 1000.0, 1),
            'rtt_ms': round(rtt * 1000.0, 1),
            'age_sec': round(now - at, 1),
        }


def answer(request: dict, received: float, sent: float) -> dict:
    """NETRA's answer to a time_sync request (what the server does, also in FakeNetra)."""
    return {'type': 'time_sync', 't0': request.get('t0'), 't1': received, 't2': sent}
