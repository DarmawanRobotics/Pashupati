"""Compact parts of the handheld's `state` message (protocol v1), from the stack's status."""

from __future__ import annotations

from pashupati_utils.core.numbers import number

PEOPLE_BOX_SEC = 2.0  # older person boxes are not drawn
MAX_ISSUES = 5


def text(value, limit: int) -> str:
    """Return value as text of at most limit characters."""
    return str(value if value is not None else '')[:limit]


def inspection_summary(status: dict, stale: bool, now: float) -> dict:
    """Compact anomaly_detector/status (no model list)."""
    last = status.get('last') if isinstance(status.get('last'), dict) else None
    summary = {
        'on': status.get('enabled') is True,
        'state': 'offline' if stale else text(status.get('state'), 16),
        'model': text(status.get('model'), 120),
        'interval': number(status.get('interval_sec'), 0.0),
        'error': text(status.get('error'), 160),
        'count': int(number(status.get('inspections'), 0.0)),
        'reports': int(number(status.get('reports'), 0.0)),
        'queue': int(number(status.get('queue'), 0.0)),
        'queue_max': int(number(status.get('queue_max'), 0.0)),
        'last': None,
    }
    if last is not None:
        at = number(last.get('at'))
        summary['last'] = {
            'cat': text(last.get('category'), 40),
            'reported': last.get('reported') is True,
            'ms': int(number(last.get('latency_ms'), 0.0)),
            'age': None if at is None else max(0, round(now - at)),
            'desc': text(last.get('description'), 80),
            'source': text(last.get('source'), 16),
            'waited': round(number(last.get('waited_sec'), 0.0)),
        }
    return summary


def people_boxes(status: dict) -> list:
    """Person boxes [x1, y1, x2, y2, score] as fractions of the image, at most 20."""
    boxes = []
    raw = status.get('boxes')
    for box in raw[:20] if isinstance(raw, list) else []:
        if not isinstance(box, list) or len(box) != 5:
            continue
        values = [number(v) for v in box]
        if None in values:
            continue
        boxes.append([round(min(max(v, 0.0), 1.0), 3) for v in values])
    return boxes


def people_summary(status: dict, stale: bool, now: float) -> dict:
    """Compact perception/people_status."""
    count, at = number(status.get('count')), number(status.get('at'))
    fresh = not stale and at is not None and now - at <= PEOPLE_BOX_SEC
    return {
        'on': status.get('enabled') is True,
        'state': 'offline' if stale else text(status.get('state'), 16),
        'count': None if count is None else int(count),
        'ms': int(number(status.get('latency_ms'), 0.0)),
        'age': None if at is None else max(0, round(now - at)),
        'boxes': people_boxes(status) if fresh else [],
        'backend': text(status.get('backend'), 24),
        'error': text(status.get('error'), 160),
    }


def handheld_health(health: dict) -> dict:
    """Return the handheld's health: ok, issues, the computer's load (of health_summary)."""

    def pick(key: str, digits: int = 0):
        value = number(health.get(key))
        return None if value is None else round(value, digits)

    return {
        'ok': health.get('ok') is True,
        'issues': [str(i) for i in health.get('issues', [])][:MAX_ISSUES],
        'cpu': pick('cpu_percent'),
        'gpu': pick('gpu_percent'),
        'ram': pick('memory_percent'),
        'ram_used': pick('memory_used_gb', 1),
        'ram_total': pick('memory_total_gb', 1),
        'temp': pick('cpu_temp_c'),
    }


def mission_text(state: str, pause_reasons: list[str], index: int, count: int) -> str:
    """Return a short line about the route: 'paused: operator', 'waypoint 12/240' or ''."""
    if pause_reasons:
        return 'paused: ' + ', '.join(pause_reasons)
    if count and state not in ('NAV_INACTIVE', 'GOAL_REACHED'):
        return f'waypoint {index + 1}/{count}'
    return ''
