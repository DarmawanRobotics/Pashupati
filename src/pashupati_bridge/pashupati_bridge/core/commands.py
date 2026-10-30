"""
The handheld's commands: which service each one calls, and the checks of their arguments.

Everything here is pure: the node builds the requests from what these functions return.
"""

from __future__ import annotations

import math
import re

from pashupati_utils.core.numbers import number

# command -> std_srvs/Trigger service of the stack (relative names)
TRIGGERS = {
    'estop': 'control/estop/engage',
    'estop_release': 'control/estop/release',
    'nav_start': 'navigation/start',
    'nav_stop': 'navigation/stop',
    'nav_pause': 'navigation/pause',
    'nav_resume': 'navigation/resume',
    'record_start': 'route/start',
    'record_stop': 'route/stop',
    'localize': 'localization/start',
    'record_tags': 'localization/record_tags',
    'dock': 'docking/dock',
    'undock': 'docking/undock',
}
# answered by the gateway itself, without a service
LOCAL = ('get_route', 'inspection_categories', 'inspection_models', 'list_routes')
# commands that latch the software e-stop at once, before any service answers
ESTOP_COMMANDS = ('estop', 'emergency_stop')
MODES = ('auto', 'remote')

CATEGORY_NAME = re.compile(r'^[a-z][a-z0-9_]{0,31}$')  # as the anomaly detector takes them
MAX_CATEGORIES = 12
MAX_CATEGORY_DESC = 200
MAX_DWELL_SEC = 3600.0


def mode_of(args: dict) -> str:
    """Return the requested control mode, auto or remote."""
    mode = str(args.get('mode', 'auto')).strip().lower()
    if mode not in MODES:
        raise ValueError(f'mode must be auto or remote, got {mode[:20]!r}')
    return mode


def dwell_seconds(value) -> float:
    """Validate a stop-point dwell time from the handheld (0 < dwell <= 1 h)."""
    dwell = number(value)
    if dwell is None or not 0.0 < dwell <= MAX_DWELL_SEC:
        raise ValueError(f'dwell must be between 0 and 3600 s, got {value!r}')
    return dwell


def category_parameters(categories) -> list:
    """
    Map the handheld's finding types to the detector's category parameters.

    categories: [{name, desc, report}] in the order the model gets them; an empty desc keeps
    the built-in text of a known type. Returns (name, [str]) pairs.
    """
    if not isinstance(categories, list) or not 1 <= len(categories) <= MAX_CATEGORIES:
        raise ValueError(f'categories: 1 to {MAX_CATEGORIES} types')
    names, reported, descriptions = [], [], []
    for entry in categories:
        if not isinstance(entry, dict):
            raise ValueError('categories: {name, desc, report} each')
        name = entry.get('name')
        if not isinstance(name, str) or not CATEGORY_NAME.match(name) or name == 'none':
            raise ValueError(f'type {str(name)[:40]!r}: a-z, 0-9 and _, up to 32, not "none"')
        if name in names:
            raise ValueError(f'type {name} is listed twice')
        desc = entry.get('desc') or ''
        if not isinstance(desc, str) or len(desc) > MAX_CATEGORY_DESC:
            raise ValueError(f'{name}: a description of up to {MAX_CATEGORY_DESC} characters')
        names.append(name)
        if entry.get('report') is True:
            reported.append(name)
        if ' '.join(desc.split()):
            descriptions.append(f'{name}: {" ".join(desc.split())}')
    return [
        ('categories', names),
        ('alarm_categories', reported),
        ('category_descriptions', descriptions),
    ]


def inspection_parameters(args: dict) -> list:
    """Map inspection_set args to (name, value) of the anomaly detector's live parameters."""
    params = []
    if 'enabled' in args:
        if not isinstance(args['enabled'], bool):
            raise ValueError('enabled must be true or false')
        params.append(('enabled', args['enabled']))
    if 'model' in args:
        model = args['model']
        if not isinstance(model, str) or not model.strip() or len(model) > 200:
            raise ValueError('model must be an Ollama model name')
        params.append(('model', model.strip()))
    if 'interval' in args:
        interval = number(args['interval'])
        if interval is None or interval > 3600.0:
            raise ValueError('interval: 0 continuous, 0.1 to 3600 s, < 0 at stop points only')
        if 0.0 < interval < 0.1:
            interval = 0.1  # the camera gives no more than that anyway
        params.append(('interval_sec', max(interval, -1.0)))
    if 'categories' in args:
        params += category_parameters(args['categories'])
    if not params:
        raise ValueError('nothing to change: give enabled, model, interval or categories')
    return params


def people_parameters(args: dict) -> list:
    """Map people_set args to the person detector's live parameters."""
    if not isinstance(args.get('enabled'), bool):
        raise ValueError('enabled must be true or false')
    return [('enabled', args['enabled'])]


def patrol_speed(value, low: float, high: float) -> float:
    """Validate a patrol speed from the handheld against the allowed range."""
    speed = number(value)
    if speed is None or not low <= speed <= high:
        raise ValueError(f'speed must be {low:g} to {high:g} m/s')
    return round(speed, 2)


def command_id(value):
    """Return a command id (int or str, not bool), None when it is not one."""
    if isinstance(value, bool) or not isinstance(value, (int, str)):
        return None
    if isinstance(value, int) or (isinstance(value, str) and len(value) <= 64):
        return value
    return None


def camera_period(fps, limit: float) -> float | None:
    """Return the camera frame period for a handheld's camera_fps, capped by limit (or None)."""
    value = number(fps)
    if value is None or not math.isfinite(value):
        return None  # absent, text or NaN: keep the current rate
    return 1.0 / min(max(value, 0.5), max(limit, 0.1))
