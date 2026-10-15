import json

from robot_inspection.vlm import build_prompt, build_request, build_schema, parse_response

CATS = ['trash', 'spill', 'stain']


def test_prompt_lists_categories_and_none():
    """The model is told every category and none, with their meaning."""
    prompt = build_prompt(CATS, describe=False, use_schema=True)
    for name in [*CATS, 'none']:
        assert f'- {name}:' in prompt
    assert 'TYPE:' not in prompt


def test_free_prompt_asks_floor_then_type():
    """Without a schema the model answers FLOOR: then TYPE:."""
    prompt = build_prompt(CATS, describe=False, use_schema=False)
    assert prompt.index('FLOOR:') < prompt.index('TYPE:')


def test_schema_puts_description_first():
    """With describe the description comes before the type."""
    schema = build_schema(CATS, describe=True)
    assert list(schema['properties']) == ['description', 'type']
    assert schema['properties']['type']['enum'] == [*CATS, 'none']
    assert list(build_schema(CATS, describe=False)['properties']) == ['type']


def test_request_format_follows_use_schema():
    """`format` is sent only in schema mode; the image is attached."""
    body = build_request('m', b'\xff\xd8', CATS, False, True, -1, 2048)
    assert body['format']['type'] == 'object' and body['images'] and body['stream'] is False
    assert 'format' not in build_request('m', b'\xff\xd8', CATS, False, False, -1, 2048)


def test_schema_answer():
    """A JSON answer is read as is."""
    r = parse_response(json.dumps({'description': 'a bottle', 'type': 'trash'}), CATS, True)
    assert r == {'category': 'trash', 'description': 'a bottle', 'valid': True}


def test_free_text_answer():
    """FLOOR:/TYPE: answers are parsed."""
    r = parse_response('FLOOR: a puddle near the door.\nTYPE: spill', CATS, False)
    assert r == {'category': 'spill', 'description': 'a puddle near the door.', 'valid': True}


def test_garbage_answer_is_none_and_invalid():
    """Unknown or broken answers never raise and map to none."""
    assert parse_response('6666666666666666', CATS, False) == {
        'category': 'none',
        'description': '',
        'valid': False,
    }
    assert not parse_response('not json', CATS, True)['valid']
    assert not parse_response('{"type": "dirty"}', CATS, True)['valid']
