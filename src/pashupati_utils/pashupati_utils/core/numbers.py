"""Numbers from the network and for JSON: finite or nothing."""

from __future__ import annotations

import math


def number(value, default: float | None = None) -> float | None:
    """Return a finite int / float as float (bools and text are not numbers), else default."""
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        return default
    return float(value)


def finite(value, digits: int = 2) -> float | None:
    """Return value rounded for JSON, or None when it is NaN, infinite or not a number."""
    checked = number(value)
    return None if checked is None else round(checked, digits)


def text_number(value: str) -> float | None:
    """Return a number written as text ('12.5', 'nan' is None), or None."""
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None
