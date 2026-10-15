import json

CATEGORY_KEYS = ('category', 'label', 'class', 'type')
FLAG_KEYS = ('is_anomaly', 'anomaly', 'detected')


def parse_result(text: str):
    """Normalise an anomaly detector JSON result; None when it reports no anomaly."""
    try:
        data = json.loads(text)
    except ValueError:
        return None
    if not isinstance(data, dict):
        return None
    flag = next((data[k] for k in FLAG_KEYS if k in data), None)
    category = next((data[k] for k in CATEGORY_KEYS if data.get(k)), None)
    if flag is False or (flag is None and not category) or category in ('none', 'normal'):
        return None
    return {
        'category': str(category or 'unknown').lower().replace(' ', '_'),
        'confidence': float(data.get('confidence', data.get('score', 1.0)) or 0.0),
        'description': str(data.get('description', data.get('reason', ''))),
    }
