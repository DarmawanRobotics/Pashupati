"""Prompt, request and answer parsing for the Ollama vision model (no ROS here)."""
import base64
import json
import re

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


def build_prompt(categories: list[str], describe: bool, use_schema: bool) -> str:
    """Return the prompt listing every category plus 'none'."""
    lines = [f'- {c}: {CATEGORY_HINTS.get(c, c.replace("_", " "))}' for c in categories]
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
) -> dict:
    """Return an Ollama /api/generate body."""
    body = {
        'model': model,
        'prompt': build_prompt(categories, describe, use_schema),
        'images': [base64.b64encode(jpeg).decode()],
        'stream': False,
        'keep_alive': keep_alive,
        'options': {
            'temperature': 0.0,
            'repeat_penalty': 1.1,
            'num_ctx': num_ctx,
            'num_predict': 96 if (describe or not use_schema) else 16,
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
