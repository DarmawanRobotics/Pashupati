"""Prompt, request and answer parsing for the Ollama vision model (no ROS here)."""
import base64
import json
import re

# A category name: what the model answers and NETRA files a finding under.
CATEGORY_NAME = re.compile(r'^[a-z][a-z0-9_]{0,31}$')
MAX_CATEGORIES = 12  # each one lengthens the prompt the model reads for every frame
MAX_DESCRIPTION = 200  # 12 of them stay well inside num_ctx with the image

# What each category means to the model; a category not listed here is sent by its name.
CATEGORY_HINTS = {
    'trash': (
        'any loose item lying on the floor that should not be there, such as wrappers, '
        'packaging, paper, bottles, cans, food, snack bags, plastic bags or litter'
    ),
    'spill': 'liquid, a wet patch or a puddle on the floor',
    'stain': 'a dry dirty mark or smear on the floor',
    'floor_damage': 'a crack, a hole, or a broken or loose tile in the floor',
    'fallen_person': 'a person lying or collapsed on the floor',
}


def category_names(values) -> list[str]:
    """
    Return the category names of a parameter list ('' entries dropped).

    Raises ValueError for a bad, repeated or reserved name, or too many / no names.
    """
    names = [str(v).strip() for v in values if str(v).strip()]
    for name in names:
        if not CATEGORY_NAME.match(name) or name == 'none':
            raise ValueError(f'category {name!r}: a-z, 0-9 and _, up to 32, not "none"')
    if len(set(names)) != len(names):
        raise ValueError('a category is listed twice')
    if not names or len(names) > MAX_CATEGORIES:
        raise ValueError(f'1 to {MAX_CATEGORIES} categories')
    return names


def parse_descriptions(values) -> dict:
    """
    Return {category: description} from 'name: description' entries ('' entries dropped).

    Raises ValueError for an entry without a valid name or with an empty or long description.
    """
    descriptions = {}
    for value in values:
        entry = str(value).strip()
        if not entry:
            continue
        name, _, text = entry.partition(':')
        name, text = name.strip(), ' '.join(text.split())
        if not CATEGORY_NAME.match(name) or not text or len(text) > MAX_DESCRIPTION:
            raise ValueError(
                f'{entry[:40]!r}: "name: what the model should look for", '
                f'up to {MAX_DESCRIPTION} characters'
            )
        descriptions[name] = text
    return descriptions


def describe_category(category: str, hints: dict) -> str:
    """Return what a category means to the model: own description, built-in, or its name."""
    return hints.get(category) or CATEGORY_HINTS.get(category) or category.replace('_', ' ')


def build_prompt(
    categories: list[str], describe: bool, use_schema: bool, hints: dict | None = None
) -> str:
    """Return the prompt listing every category plus 'none'."""
    lines = [f'- {c}: {describe_category(c, hints or {})}' for c in categories]
    lines.append('- none: the floor is clean with nothing lying on it')
    ignore = 'furniture, shelves and walls' if 'fallen_person' in categories else (
        'furniture, shelves, walls and people'
    )
    prompt = (
        'You are inspecting the floor of a public building for cleanliness. '
        'Classify the floor as exactly one of these types:\n'
        + '\n'.join(lines)
        + f'\nIgnore {ignore}. Only the floor matters.'
    )
    if not use_schema:
        return prompt + (
            '\nAnswer in this format:\n'
            'FLOOR: <one short sentence describing what is lying on the floor>\n'
            f'TYPE: <{", ".join([*categories, "none"])}>'
        )
    if describe:
        prompt += (
            '\nFirst describe what is lying on the floor in one short sentence, '
            'then give the type.'
        )
    return prompt


def build_schema(categories: list[str], describe: bool) -> dict:
    """
    Return the JSON schema for Ollama structured output.

    `description` comes before `type` so the model looks first and decides after.
    """
    properties, required = {}, []
    if describe:
        properties['description'] = {'type': 'string'}
        required.append('description')
    properties['type'] = {'type': 'string', 'enum': [*categories, 'none']}
    required.append('type')
    return {'type': 'object', 'properties': properties, 'required': required}


def build_request(
    model: str,
    jpeg: bytes,
    categories: list[str],
    describe: bool,
    use_schema: bool,
    keep_alive,
    num_ctx: int,
    hints: dict | None = None,
) -> dict:
    """Return an Ollama /api/generate body (hints: own category descriptions)."""
    body = {
        'model': model,
        'prompt': build_prompt(categories, describe, use_schema, hints),
        'images': [base64.b64encode(jpeg).decode()],
        'stream': False,
        'keep_alive': keep_alive,
        'options': {
            'temperature': 0.0,
            'repeat_penalty': 1.1,
            'num_ctx': num_ctx,
            # {"type": "<category>"}: room for a long own category name
            'num_predict': 96 if (describe or not use_schema) else 32,
        },
    }
    if use_schema:
        body['format'] = build_schema(categories, describe)
    return body


def parse_response(text: str, categories: list[str], use_schema: bool) -> dict:
    """
    Normalise a model answer to {category, description, valid}.

    `valid` is False when no known category could be read; category is then 'none'.
    """
    category, description = '', ''
    if use_schema:
        try:
            data = json.loads(text)
            category = str(data.get('type', '')).strip().lower()
            description = str(data.get('description', '')).strip()
        except (ValueError, AttributeError):
            pass
    else:
        m_type = re.search(r'TYPE\s*:\s*\W*(\w+)', text, re.I)
        m_floor = re.search(r'FLOOR\s*:\s*(.+)', text, re.I)
        category = m_type.group(1).lower() if m_type else ''
        description = m_floor.group(1).strip() if m_floor else ''
    valid = category in categories or category == 'none'
    return {
        'category': category if valid else 'none',
        'description': description[:300],
        'valid': valid,
    }


def full_name(model: str) -> str:
    """Return a model name as Ollama lists it: 'moondream' -> 'moondream:latest'."""
    model = model.strip()
    return model if ':' in model.rsplit('/', 1)[-1] else f'{model}:latest'


def model_options(configured: list[str], installed, current: str) -> list[dict]:
    """
    Models the operator can choose from: the configured ones first, then other vision models.

    `installed` maps the models in Ollama to whether they take images (None: unknown), or is None
    when Ollama could not be asked. Each option is {name, installed (True, False or None when
    unknown), active (the model in use)}; a text-only model is never offered.
    """
    options, seen = [], set()
    wanted = full_name(current) if current else ''

    def add(name: str, available):
        key = full_name(name)
        if key in seen:
            return
        seen.add(key)
        options.append({'name': name, 'installed': available, 'active': key == wanted})

    for name in [*configured, current]:
        if name and name.strip():
            add(name.strip(), None if installed is None else full_name(name) in installed)
    for name in sorted(installed or {}):
        if installed[name] is not False:
            add(name, True)
    return options
