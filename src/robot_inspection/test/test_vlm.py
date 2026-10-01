import json

from robot_inspection.vlm import build_prompt, build_request, parse_response

CATS = ['trash', 'spill', 'floor_damage', 'fallen_person']


def test_prompt_lists_categories_and_none():
    """The model is told every category and none."""
    prompt = build_prompt(CATS)
    assert 'floor damage' in prompt
    assert '<one of trash, spill, floor_damage, fallen_person, none>' in prompt


def test_request_is_json_constrained():
    """Requests ask Ollama for JSON with the image attached."""
    body = build_request('moondream', 'p', b'\xff\xd8', '30m')
    assert body['format'] == 'json' and body['images'] and body['stream'] is False


def test_valid_json_answer():
    """A clean JSON answer is taken as is."""
    r = parse_response(
        json.dumps({'category': 'trash', 'confidence': 0.9, 'description': 'bottle'}), CATS
    )
    assert r == {
        'is_anomaly': True,
        'category': 'trash',
        'confidence': 0.9,
        'description': 'bottle',
    }


def test_free_text_category_is_mapped_by_keywords():
    """Off-list categories fall back to keyword matching."""
    r = parse_response(
        '{"category": "liquid", "description": "a puddle near the escalator"}', CATS
    )
    assert r['category'] == 'spill' and r['confidence'] == 0.6


def test_none_and_percent_confidence():
    """'none' is not an anomaly and 0-100 confidences are scaled."""
    assert not parse_response('{"category": "none", "confidence": 80}', CATS)['is_anomaly']
    assert (
        parse_response('{"category": "fallen person", "confidence": 85}', CATS)['confidence']
        == 0.85
    )


def test_garbage_answer_is_safe():
    """Non-JSON output does not raise."""
    assert parse_response('I see a clean corridor', CATS)['category'] == 'none'
