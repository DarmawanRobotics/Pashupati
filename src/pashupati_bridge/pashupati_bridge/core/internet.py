"""The internet check: ping results and what the handheld and the monitor show of them."""

from __future__ import annotations

from collections import deque
import json
import re

ROUND_TRIP = re.compile(r'time[=<]\s*([\d.]+)\s*ms')
WINDOW = 12  # the last pings: the share lost
SLOW_MS = 300.0  # a slower round trip warns
LOSSY_PCT = 25  # this share of the last pings lost warns


def round_trip(output: str) -> float | None:
    """Return the round trip (ms) in the output of `ping -c 1`, None when the ping got lost."""
    found = ROUND_TRIP.search(output or '')
    return float(found.group(1)) if found else None


class PingLog:
    """The last pings of one target: (when, round trip in ms or None when lost)."""

    def __init__(self, target: str, window: int = WINDOW):
        self.target = target
        self.pings: deque = deque(maxlen=window)
        self.problem = ''  # why it cannot ping at all

    def add(self, now: float, ms: float | None):
        """Record one ping."""
        self.pings.append((now, ms))

    def summary(self, now: float) -> dict | None:
        """Return {online, ms, loss, target, age} for the handheld; None before the first ping."""
        if not self.pings:
            return None
        last_at = self.pings[-1][0]
        recent = [ms for _, ms in list(self.pings)[-2:]]
        online = any(ms is not None for ms in recent)  # one lost ping is not offline yet
        answered = [ms for _, ms in self.pings if ms is not None]
        lost = len(self.pings) - len(answered)
        return {
            'online': online,
            'ms': round(answered[-1], 1) if online else None,
            'loss': round(100.0 * lost / len(self.pings)),
            'target': self.target,
            'age': round(now - last_at, 1),
        }


def monitor_status(summary: dict | None, problem: str = '') -> str:
    """Return internet/status: JSON for the Dharma console, with its level."""
    if problem:
        return json.dumps({'status': problem, 'level': 'warn'})
    if summary is None:
        return json.dumps({'status': 'checking', 'level': 'info'})
    online, ms, loss = summary['online'], summary['ms'], summary['loss']
    slow = ms is not None and ms > SLOW_MS
    level = 'error' if not online else 'warn' if loss >= LOSSY_PCT or slow else 'ok'
    return json.dumps(
        {
            'status': 'online' if online else 'offline',
            'ping': f'{ms:.0f} ms' if ms is not None else '-',
            'lost': f'{loss} % of the last pings',
            'target': summary['target'],
            'level': level,
        }
    )
