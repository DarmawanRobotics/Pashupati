import json

from robot_inspection.vlm import (
    build_prompt,
    build_request,
    build_schema,
    full_name,
    model_options,
    parse_response,
)

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


def test_full_name_adds_the_latest_tag():
    """Ollama lists 'moondream' as 'moondream:latest'; namespaces and tags are kept."""
    assert full_name('moondream') == 'moondream:latest'
    assert full_name(' qwen3-vl:2b-instruct ') == 'qwen3-vl:2b-instruct'
    assert full_name('ahmadwaqar/smolvlm2-2.2b-instruct') == (
        'ahmadwaqar/smolvlm2-2.2b-instruct:latest'
    )


def test_model_options_put_configured_first_and_skip_text_models():
    """Configured models first (installed or not), then other vision models Ollama has."""
    installed = {'moondream:latest': True, 'llama3:latest': False, 'llava:7b': None}
    options = model_options(['qwen3-vl:2b-instruct', 'moondream'], installed, 'moondream')
    assert options == [
        {'name': 'qwen3-vl:2b-instruct', 'installed': False, 'active': False},
        {'name': 'moondream', 'installed': True, 'active': True},
        {'name': 'llava:7b', 'installed': True, 'active': False},
    ]
    unknown = model_options(['moondream'], None, 'other:1b')
    assert unknown == [
        {'name': 'moondream', 'installed': None, 'active': False},
        {'name': 'other:1b', 'installed': None, 'active': True},
    ]
