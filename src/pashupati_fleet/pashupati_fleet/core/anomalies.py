"""Findings waiting for the frame they were found in, then spooled for upload (no ROS)."""

from __future__ import annotations

from collections import deque
import json
import math
import time

from pashupati_fleet.core.anomaly import parse_result

LATE_REPORT_SEC = 3.0  # a finding reported this long after its frame waited in a queue
EVIDENCE_WAIT_SEC = 1.0  # how long a finding waits for its frame
LATEST_FRAME_MAX_AGE_SEC = 3.0  # the fallback frame must be this fresh
EVIDENCE_KEPT = 10


def stamp_key(sec: int, nanosec: int) -> str:
    """Return the key of a camera stamp, as the detector writes evidence_stamp."""
    return f'{sec}.{nanosec:09d}'


class Anomalies:
    """Pending findings matched with evidence frames; ready ones go to the spool."""

    def __init__(self, min_confidence: float):
        self.min_confidence = min_confidence
        self.evidence: deque = deque(maxlen=EVIDENCE_KEPT)  # (stamp key, jpeg)
        self.latest = (0.0, b'')  # (time, jpeg) of the camera, the fallback
        self.pending: list = []  # (deadline, meta, evidence key)

    def finding(self, text: str, context: dict, pose, stop_index, poses, now: float) -> bool:
        """Queue a detector result; False when it is no finding or below the confidence."""
        result = parse_result(text)
        if result is None or result['confidence'] < self.min_confidence:
            return False
        try:
            raw = json.loads(text)
            key = raw.get('evidence_stamp')
            taken = float(raw.get('captured_at') or 0.0)
        except (ValueError, AttributeError, TypeError):
            key, taken = None, 0.0
        meta = {**result, **context, 'pose': pose, 'stop_index': stop_index, 'detector': 'robot'}
        if math.isfinite(taken) and taken > 0.0 and now - taken > LATE_REPORT_SEC:
            # a frame from the interval queue: where and when it was taken, not now
            meta.update(ts=taken, pose=poses.at(taken), stop_index=None)
        self.pending.append((time.monotonic() + EVIDENCE_WAIT_SEC, meta, key))
        return True

    def ready(self, now: float) -> tuple[list, int]:
        """Return ([(meta, jpeg)] to spool, findings dropped for want of any frame)."""
        evidence = dict(self.evidence)
        waiting, out, dropped = [], [], 0
        for deadline, meta, key in self.pending:
            jpeg = evidence.get(key) if key else None
            if jpeg is None and key and time.monotonic() < deadline:
                waiting.append((deadline, meta, key))
                continue
            if jpeg is None:
                stamp, jpeg = self.latest
                if not jpeg or now - stamp > LATEST_FRAME_MAX_AGE_SEC:
                    dropped += 1
                    continue
                meta['evidence'] = 'latest_frame'
            else:
                meta['evidence'] = 'analysed_frame'
            out.append((meta, jpeg))
        self.pending = waiting
        return out, dropped
