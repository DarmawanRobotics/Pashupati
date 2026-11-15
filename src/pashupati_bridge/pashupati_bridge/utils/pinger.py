"""Pinging the internet check's target from a thread, once every period (the system ping)."""

from __future__ import annotations

import math
import subprocess
import threading
import time

from pashupati_bridge.core.internet import PingLog, round_trip


def ping_once(target: str, timeout: float) -> float | None:
    """Return the round trip of one ping (ms), None when lost; OSError without a ping command."""
    wait = str(max(1, math.ceil(timeout)))
    command = ['ping', '-n', '-c', '1', '-W', wait, target]
    try:
        done = subprocess.run(
            command, capture_output=True, text=True, timeout=timeout + 2.0, check=False
        )
    except subprocess.TimeoutExpired:
        return None
    return round_trip(done.stdout) if done.returncode == 0 else None


class Pinger:
    """A daemon thread that pings the target every period into a PingLog."""

    def __init__(self, target: str, period: float, timeout: float, ping=ping_once):
        self.log = PingLog(target)
        self.lock = threading.Lock()
        self.period, self.timeout, self.ping = period, timeout, ping
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop.is_set():
            try:
                ms = self.ping(self.log.target, self.timeout)
            except OSError as error:  # no ping command: say so, try no more
                with self.lock:
                    self.log.problem = f'cannot ping: {error.strerror or error}'
                return
            with self.lock:
                self.log.add(time.monotonic(), ms)
            self._stop.wait(self.period)

    def summary(self) -> tuple[dict | None, str]:
        """Return (the handheld's summary or None, why it cannot ping)."""
        with self.lock:
            return self.log.summary(time.monotonic()), self.log.problem

    def stop(self):
        """Stop pinging."""
        self._stop.set()
