from __future__ import annotations

from dataclasses import replace

from fakes import FakeOllama
from pashupati_inspection.core.config import AnomalyConfig
from pashupati_inspection.core.engine import PERIODIC, STOP_POINT, InspectionEngine

from dharma_utils.config import build


def engine(answer: str = 'trash', **values):
    fake = FakeOllama(answer)
    return InspectionEngine(build(AnomalyConfig, values), fake), fake


def test_the_chosen_model_is_loaded_and_the_previous_one_freed():
    eng, fake = engine()
    assert eng.load('qwen3-vl:2b-instruct') and eng.state == 'running'
    eng.config = replace(eng.config, model='moondream:latest')
    assert eng.load('moondream:latest')
    assert fake.unloads == ['qwen3-vl:2b-instruct']
    assert eng.unload() == 'moondream:latest' and eng.loaded is None


def test_an_unknown_model_is_an_error_with_the_fix():
    eng, _ = engine(model='nope:1b')
    assert not eng.load('nope:1b')
    assert eng.state == 'error' and 'ollama pull nope:1b' in eng.error


def test_nothing_is_inspected_before_a_model_is_loaded():
    eng, _ = engine()
    assert eng.inspect(b'jpg', STOP_POINT, 0.0, 0.1) == (
        None,
        'model qwen3-vl:2b-instruct not loaded yet',
    )


def test_alarm_types_are_reported_and_periodic_ones_once_per_cooldown():
    eng, _ = engine(cooldown_sec=60.0)
    eng.load('qwen3-vl:2b-instruct')
    verdict, message = eng.inspect(b'jpg', PERIODIC, 100.0, 0.1)
    assert verdict.report and message.startswith('trash (')
    assert verdict.result['is_anomaly'] and verdict.result['source'] == PERIODIC
    assert not eng.inspect(b'jpg', PERIODIC, 130.0, 0.1)[0].report  # within the cooldown
    assert eng.inspect(b'jpg', STOP_POINT, 131.0, 0.1)[0].report  # a stop point always
    assert eng.inspect(b'jpg', PERIODIC, 200.0, 0.1)[0].report
    assert (eng.inspections, eng.reports) == (4, 3)


def test_types_not_reported_are_shown_only():
    eng, _ = engine('stain')
    eng.load('qwen3-vl:2b-instruct')
    verdict, _ = eng.inspect(b'jpg', STOP_POINT, 0.0, 0.1)
    assert not verdict.report and not verdict.result['is_anomaly']
    assert eng.last['category'] == 'stain' and not eng.last['reported']


def test_a_failing_model_is_reported():
    eng, fake = engine()
    eng.load('qwen3-vl:2b-instruct')
    fake.fail = True
    verdict, message = eng.inspect(b'jpg', STOP_POINT, 0.0, 0.1)
    assert verdict is None and message.startswith('VLM unavailable')
    assert eng.state == 'error'


def test_status_lists_models_and_types():
    eng, _ = engine(categories=['trash', 'bag'], alarm_categories=['bag'],
                    category_descriptions=['bag: a bag left alone'])  # fmt: skip
    assert eng.refresh_models()
    status = eng.status()
    models = {m['name']: m for m in status['models']}
    assert models['qwen3-vl:2b-instruct'] == {
        'name': 'qwen3-vl:2b-instruct', 'installed': True, 'active': True,
    }  # fmt: skip
    assert 'llama3:latest' not in models  # no vision
    assert models['ahmadwaqar/smolvlm2-2.2b-instruct:latest']['installed'] is False
    shown = {c['name']: c for c in status['categories']}
    assert shown['bag'] == {'name': 'bag', 'desc': 'a bag left alone', 'report': True,
                            'builtin': False, 'own_desc': True}  # fmt: skip
    assert shown['trash']['builtin'] and not shown['trash']['report']
