from __future__ import annotations

from pashupati_inspection.core.config import AnomalyConfig, PeopleConfig
import pytest

from dharma_utils.config import ConfigError, build


def test_defaults_are_valid():
    assert build(AnomalyConfig).categories == ('trash', 'spill', 'stain')
    assert build(PeopleConfig).rate_hz == 0.0


def test_bad_finding_types_are_refused_with_the_reason():
    with pytest.raises(ConfigError, match='a-z'):
        build(AnomalyConfig, {'categories': ['Not ok']})
    with pytest.raises(ConfigError, match='twice'):
        build(AnomalyConfig, {'categories': ['trash', 'trash']})
    with pytest.raises(ConfigError, match='name: what'):
        build(AnomalyConfig, {'category_descriptions': ['no colon here']})
    with pytest.raises(ConfigError, match='model'):
        build(AnomalyConfig, {'model': ' '})


def test_people_rate_is_auto_or_at_least_a_tenth():
    with pytest.raises(ConfigError, match='rate_hz'):
        build(PeopleConfig, {'rate_hz': 0.05})
    assert build(PeopleConfig, {'rate_hz': 0.1}).rate_hz == 0.1
