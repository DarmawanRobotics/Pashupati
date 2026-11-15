"""The internet check: ping output, the summary for the handheld and the monitor's status."""

from __future__ import annotations

import json
import time

from pashupati_bridge.core.internet import PingLog, monitor_status, round_trip
from pashupati_bridge.utils import pinger
from pashupati_bridge.utils.pinger import Pinger

LINUX = """PING 8.8.8.8 (8.8.8.8) 56(84) bytes of data.
64 bytes from 8.8.8.8: icmp_seq=1 ttl=117 time=38.4 ms

--- 8.8.8.8 ping statistics ---
1 packets transmitted, 1 received, 0% packet loss, time 0ms
rtt min/avg/max/mdev = 38.412/38.412/38.412/0.000 ms
"""


def test_round_trips_of_ping_outputs():
    assert round_trip(LINUX) == 38.4
    assert round_trip('64 bytes from 127.0.0.1: icmp_seq=1 ttl=64 time<1 ms') == 1.0
    assert round_trip('1 packets transmitted, 0 received, 100% packet loss') is None
    assert round_trip('') is None


def test_one_lost_ping_is_not_offline_two_are():
    log = PingLog('8.8.8.8')
    assert log.summary(0.0) is None
    log.add(0.0, 40.0)
    log.add(5.0, None)
    assert log.summary(6.0) == {
        'online': True,
        'ms': 40.0,
        'loss': 50,
        'target': '8.8.8.8',
        'age': 1.0,
    }
    log.add(10.0, None)
    summary = log.summary(10.5)
    assert summary['online'] is False and summary['ms'] is None and summary['loss'] == 67


def test_the_monitor_status_carries_its_level():
    log = PingLog('8.8.8.8')
    assert json.loads(monitor_status(None))['level'] == 'info'
    for _ in range(12):
        log.add(0.0, 30.0)
    ok = json.loads(monitor_status(log.summary(1.0)))
    assert ok == {
        'status': 'online',
        'ping': '30 ms',
        'lost': '0 % of the last pings',
        'target': '8.8.8.8',
        'level': 'ok',
    }
    log.add(1.0, 900.0)
    assert json.loads(monitor_status(log.summary(1.0)))['level'] == 'warn'
    log.add(2.0, None)
    log.add(3.0, None)
    assert json.loads(monitor_status(log.summary(3.0)))['level'] == 'error'
    assert json.loads(monitor_status(None, 'cannot ping: no such file'))['level'] == 'warn'


def test_the_pinger_fills_its_log_and_reports_a_missing_ping():
    answers = iter([20.0, None])
    log_pinger = Pinger('8.8.8.8', 0.05, 1.0, ping=lambda target, timeout: next(answers, 25.0))
    time.sleep(0.3)
    log_pinger.stop()
    summary, problem = log_pinger.summary()
    assert summary['online'] and problem == ''

    def missing(target, timeout):
        raise FileNotFoundError(2, 'No such file or directory')

    broken = Pinger('8.8.8.8', 0.05, 1.0, ping=missing)
    time.sleep(0.2)
    assert broken.summary() == (None, 'cannot ping: No such file or directory')


def test_ping_once_reads_the_system_ping(monkeypatch):
    class Done:
        returncode, stdout = 0, LINUX

    seen = []
    monkeypatch.setattr(pinger.subprocess, 'run', lambda cmd, **kw: seen.append(cmd) or Done())
    assert pinger.ping_once('8.8.8.8', 2.0) == 38.4
    assert seen[0] == ['ping', '-n', '-c', '1', '-W', '2', '8.8.8.8']
    Done.returncode = 1
    assert pinger.ping_once('8.8.8.8', 2.0) is None
