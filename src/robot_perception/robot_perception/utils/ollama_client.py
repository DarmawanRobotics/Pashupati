"""Ollama client helpers for Moondream-based anomaly detection.

Ported from the standalone moondream_anomaly_detector.py script -- the
CLI/OpenCV/threading parts were dropped, this module keeps only the
Ollama-facing logic (model check/pull, structured chat call, response
parsing, GPU/CPU status check) so the ROS2 node can stay focused on
pub/sub plumbing.
"""

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

# `anomaly` is deliberately NOT a separate field -- deriving is_anomaly from
# `type` alone makes an anomaly=true/type="none" contradiction impossible.
ANOMALY_SCHEMA = {
    "type": "object",
    "properties": {
        "type": {"type": "string", "enum": ["trash", "spill", "fallen_person", "none"]},
        "description": {"type": "string"},
    },
    "required": ["type", "description"],
}


def model_is_available(model_name: str) -> bool:
    response = ollama.list()
    models = (
        response.models if hasattr(response, "models") else response.get("models", [])
    )
    for model in models:
        name = (
            getattr(model, "model", "")
            if not isinstance(model, dict)
            else model.get("model", model.get("name", ""))
        )
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
    """Run one structured anomaly query against a JPEG-encoded frame.

    Returns: {is_anomaly, type, description, latency_sec, raw}
    """
    t0 = time.perf_counter()
    response = ollama.chat(
        model=model_name,
        messages=[{"role": "user", "content": ANOMALY_PROMPT, "images": [jpeg_bytes]}],
        format=ANOMALY_SCHEMA,
        options={"temperature": 0, "repeat_penalty": 1.1},
    )
    latency = time.perf_counter() - t0
    raw_text = response.get("message", {}).get("content", "")
    parsed = _parse_response(raw_text)
    parsed["latency_sec"] = latency
    return parsed


def _parse_response(text: str) -> dict:
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
        return {
            "is_anomaly": False,
            "type": "none",
            "description": text.strip(),
            "raw": text,
        }


def get_processor_status(model_name: str) -> str:
    """Report whether the currently loaded model is running on GPU or CPU.
    Only meaningful after at least one query_frame() call (model must be
    loaded into memory first -- `ollama ps` only shows loaded models)."""
    try:
        response = ollama.ps()
        models = (
            response.models
            if hasattr(response, "models")
            else response.get("models", [])
        )
        for model in models:
            name = (
                getattr(model, "model", "")
                if not isinstance(model, dict)
                else model.get("model", model.get("name", ""))
            )
            if model_name.lower() in name.lower():
                size = (
                    getattr(model, "size", 0)
                    if not isinstance(model, dict)
                    else model.get("size", 0)
                )
                size_vram = (
                    getattr(model, "size_vram", 0)
                    if not isinstance(model, dict)
                    else model.get("size_vram", 0)
                )
                if size == 0:
                    return "CPU/GPU (undetermined)"
                gpu_pct = (size_vram / size) * 100
                if gpu_pct >= 99.9:
                    return f"100% GPU (VRAM: {size_vram / (1024**3):.2f} GB)"
                elif gpu_pct > 0:
                    return f"Hybrid (GPU: {gpu_pct:.1f}%, CPU: {100 - gpu_pct:.1f}%)"
                return "100% CPU"
        return "not loaded yet"
    except Exception:
        return "unknown (failed to query Ollama)"
