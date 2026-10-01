import base64
import json
import re

DEFAULT_CATEGORIES = {
    'trash': ('trash', 'garbage', 'litter', 'rubbish', 'bottle', 'cup', 'wrapper', 'plastic bag'),
    'spill': ('spill', 'liquid', 'puddle', 'wet floor', 'water on'),
    'floor_damage': ('crack', 'broken tile', 'damaged floor', 'hole', 'loose tile'),
    'fallen_person': (
        'fallen',
        'lying on the floor',
        'person lying',
        'collapsed',
        'on the ground',
    ),
}
NONE_WORDS = ('none', 'nothing', 'clean', 'normal', 'no anomaly', 'no issue')

PROMPT = (
    'You are a mall patrol robot inspecting the floor and corridor in this photo. '
    'Report the single most important problem among: {categories}, or "none" if the area is fine. '
    'Answer only with JSON: {{"category": "<one of {options}>", "confidence": <0..1>, '
    '"description": "<short reason>"}}.'
)


def build_prompt(categories: list[str]) -> str:
    """Return the prompt listing the categories the model must choose from."""
    names = ', '.join(c.replace('_', ' ') for c in categories)
    return PROMPT.format(categories=names, options=', '.join([*categories, 'none']))


def build_request(model: str, prompt: str, jpeg: bytes, keep_alive: str) -> dict:
    """Return an Ollama /api/generate body with JSON-constrained output."""
    return {
        'model': model,
        'prompt': prompt,
        'images': [base64.b64encode(jpeg).decode()],
        'format': 'json',
        'stream': False,
        'keep_alive': keep_alive,
        'options': {'temperature': 0.0, 'num_predict': 96},
    }


def classify_text(text: str, categories: list[str]) -> str:
    """Map free text to a category by keywords; 'none' when nothing matches."""
    low = text.lower()
    for category in categories:
        words = DEFAULT_CATEGORIES.get(category, (category.replace('_', ' '),))
        if any(w in low for w in words):
            return category
    return 'none'


def parse_response(text: str, categories: list[str]) -> dict:
    """Normalise a model answer to {is_anomaly, category, confidence, description}."""
    data: dict = {}
    match = re.search(r'\{.*\}', text, re.S)
    if match:
        try:
            data = json.loads(match.group(0))
        except ValueError:
            data = {}
    raw_category = str(data.get('category', '')).strip().lower().replace(' ', '_')
    description = str(data.get('description', '') or text).strip()[:300]
    if raw_category in categories:
        category = raw_category
    elif raw_category in ('none', 'normal', '') and not description:
        category = 'none'
    elif any(w == raw_category.replace('_', ' ') for w in NONE_WORDS):
        category = 'none'
    else:
        category = classify_text(f'{raw_category} {description}', categories)
    try:
        confidence = float(data.get('confidence', 0.6))
    except (TypeError, ValueError):
        confidence = 0.6
    confidence = min(1.0, max(0.0, confidence if confidence <= 1.0 else confidence / 100.0))
    return {
        'is_anomaly': category != 'none',
        'category': category,
        'confidence': round(confidence, 3),
        'description': description,
    }
