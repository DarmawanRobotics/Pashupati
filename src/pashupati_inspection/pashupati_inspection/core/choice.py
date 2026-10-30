"""
The live choices of a detector kept across restarts (state_file, JSON).

What the operator switched on the handheld or NETRA (on / off, model, pace, finding types) wins
over the parameter files at the next start; deleting the file goes back to them.
"""

from __future__ import annotations

import math

from dharma_utils.io import expand, read_json, write_json_atomic

CATEGORY_KEYS = ('categories', 'alarm_categories', 'category_descriptions')


def _is_number(value) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def restored(data, keys: tuple[str, ...]) -> dict:
    """Return the valid saved values among keys (parameter name -> value)."""
    if not isinstance(data, dict):
        return {}
    out = {}
    for key in keys:
        value = data.get(key)
        if key == 'enabled' and isinstance(value, bool):
            out[key] = value
        elif key == 'model' and isinstance(value, str) and value.strip():
            out[key] = value.strip()
        elif key == 'interval_sec' and _is_number(value):
            out[key] = float(value)
        elif (
            key in CATEGORY_KEYS
            and isinstance(value, list)
            and all(isinstance(v, str) for v in value)
        ):
            out[key] = [*value] or ['']  # never an empty list: rclpy takes it for bytes
    return out


def read_choice(path: str, keys: tuple[str, ...]) -> tuple[dict, str | None]:
    """Return (saved values, error); a missing file is no choice and no error."""
    if not path:
        return {}, None
    data, error = read_json(expand(path))
    return restored(data, keys), error


def write_choice(path: str, choice: dict) -> str | None:
    """Keep the choice; return the error, or None."""
    if not path:
        return None
    try:
        write_json_atomic(expand(path), choice, backup=False)
    except OSError as error:
        return str(error)
    return None
