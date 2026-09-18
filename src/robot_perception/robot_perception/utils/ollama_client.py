import json
import time

import ollama

ANOMALY_PROMPT = (
    "You are monitoring a mall corridor camera. Only flag an anomaly for one of these three "
    "SPECIFIC situations, and nothing else:\n"
    "- trash: loose garbage or litter scattered on the floor (not signage, posters, pillars, "
    "decorations, or normal store fixtures/text)\n"
    "- spill: a liquid puddle visibly wet on the floor\n"
    "- fallen_person: a person lying or collapsed on the ground (not standing or walking)\n"
    "If none of these three exact situations are clearly visible, you MUST answer type=none, "
    "even if the image contains signs, text, pillars, or other normal mall objects. "
    "Keep the description to a single short sentence (under 15 words), do not repeat phrases."
)

ANOMALY_SCHEMA = {
    "type": "object",
    "properties": {
        "type": {"type": "string", "enum": ["trash", "spill", "fallen_person", "none"]},
        "description": {"type": "string"},
    },
    "required": ["type", "description"],
}


def model_is_available(model_name: str) -> bool:
    """Check if the Ollama model is present locally."""
    response = ollama.list()
    models = response.models if hasattr(response, 'models') else response.get('models', [])
    for model in models:
        if isinstance(model, dict):
            name = model.get('model', model.get('name', ''))
        else:
            name = getattr(model, 'model', '')
        if model_name.lower() in name.lower():
            return True
    return False


def ensure_model(model_name: str, logger=None) -> None:
    """Verify the Ollama model is present locally, pulling it if not."""
    if model_is_available(model_name):
        return
    if logger:
        logger.info(f"Model '{model_name}' not found locally, pulling...")
    ollama.pull(model_name)


def query_frame(jpeg_bytes: bytes, model_name: str = "moondream") -> dict:
    """Run one structured anomaly query against a JPEG-encoded frame."""
    start = time.perf_counter()
    response = ollama.chat(
        model=model_name,
        messages=[{'role': 'user', 'content': ANOMALY_PROMPT, 'images': [jpeg_bytes]}],
        format=ANOMALY_SCHEMA,
        options={'temperature': 0, 'repeat_penalty': 1.1},
    )
    latency = time.perf_counter() - start
    raw_text = response.get('message', {}).get('content', '')
    parsed = parse_response(raw_text)
    parsed['latency_sec'] = latency
    return parsed


def parse_response(text: str) -> dict:
    """Parse Ollama's JSON response into a structured dict, with fallback for non-JSON text."""
    try:
        data = json.loads(text)
        anomaly_type = str(data.get("type", "none")).lower()
        return {
            "is_anomaly": anomaly_type != "none",
            "type": anomaly_type,
            "description": str(data.get("description", "")).strip(),
            "raw": text,
        }
    except (json.JSONDecodeError, AttributeError):
        return {"is_anomaly": False, "type": "none", "description": text.strip(), "raw": text}