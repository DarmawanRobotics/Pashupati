from __future__ import annotations

from pashupati_utils.core.numbers import finite, number, text_number


def test_only_finite_numbers_count():
    assert number(1) == 1.0
    assert number(2.5) == 2.5
    for bad in (True, 'x', None, float('nan'), float('inf'), [1]):
        assert number(bad) is None
        assert number(bad, 0.0) == 0.0


def test_finite_rounds_or_drops():
    assert finite(1.234, 1) == 1.2
    assert finite(float('nan')) is None
    assert finite('3') is None


def test_text_numbers():
    assert text_number('12.5') == 12.5
    assert text_number('nan') is None
    assert text_number('') is None
    assert text_number(None) is None
