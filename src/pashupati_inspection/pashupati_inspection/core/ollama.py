"""The Ollama HTTP API, just what the inspection needs (urllib, no ROS)."""

from __future__ import annotations

import json
import time
import urllib.error
import urllib.request

from pashupati_inspection.core.vlm import build_request, parse_response

ERRORS = (urllib.error.URLError, TimeoutError, ValueError, OSError)
LOAD_TIMEOUT_SEC = 180.0  # a cold model on a Jetson takes a while


class OllamaError(RuntimeError):
    """Ollama could not do what was asked; the message says why and how to fix it."""


class Ollama:
    """Load, ask and free vision models on an Ollama server."""

    def __init__(self, url: str, keep_alive, num_ctx: int):
        self.url = url.rstrip('/')
        self.keep_alive = keep_alive_value(keep_alive)
        self.num_ctx = int(num_ctx)

    def post(self, body: dict, timeout: float, path: str = '/api/generate') -> dict:
        """POST a JSON body and return the JSON reply."""
        request = urllib.request.Request(
            self.url + path,
            data=json.dumps(body).encode(),
            headers={'Content-Type': 'application/json'},
        )
        with urllib.request.urlopen(request, timeout=timeout) as reply:
            return json.loads(reply.read())

    def installed(self) -> list[str] | None:
        """Return the model names Ollama has, None when it does not answer."""
        try:
            with urllib.request.urlopen(self.url + '/api/tags', timeout=3.0) as reply:
                models = json.loads(reply.read()).get('models', [])
        except (*ERRORS, AttributeError):
            return None
        return [str(m.get('name', '')) for m in models if isinstance(m, dict) and m.get('name')]

    def takes_images(self, model: str) -> bool | None:
        """Return whether a model takes images (its capabilities), None when unknown."""
        try:
            capabilities = self.post({'model': model, 'name': model}, 5.0, '/api/show').get(
                'capabilities'
            )
        except (*ERRORS, AttributeError):
            return None
        return 'vision' in capabilities if isinstance(capabilities, list) else None

    def load(self, model: str):
        """Load a model with the options of the inspections; raise OllamaError with the fix."""
        body = {
            'model': model,
            'prompt': '',
            'keep_alive': self.keep_alive,
            'options': {'num_ctx': self.num_ctx},  # Ollama reloads a model whose num_ctx changes
        }
        try:
            self.post(body, LOAD_TIMEOUT_SEC)
        except urllib.error.HTTPError as error:
            hint = f': ollama pull {model}' if error.code == 404 else ''
            raise OllamaError(
                f'Ollama could not load {model} (HTTP {error.code}){hint}'
            ) from error
        except ERRORS as error:
            raise OllamaError(f'Ollama not reachable at {self.url} ({error})') from error

    def unload(self, model: str) -> str:
        """Free a model's memory (a Jetson shares it with the GPU); '' or the error."""
        try:
            self.post({'model': model, 'keep_alive': 0}, 10.0)
        except ERRORS as error:
            return str(error)
        return ''

    def ask(self, model: str, jpeg: bytes, prompt: dict, timeout: float) -> tuple[dict, str]:
        """
        Run the model on one image; return (result, raw answer). One retry covers a restart.

        prompt: categories, describe, use_schema and hints of vlm.build_request. Raises one of
        ERRORS when Ollama fails twice.
        """
        body = build_request(
            model,
            jpeg,
            prompt['categories'],
            prompt['describe'],
            prompt['use_schema'],
            self.keep_alive,
            self.num_ctx,
            prompt['hints'],
        )
        start = time.monotonic()
        answer, result = '', {}
        for attempt in range(2):
            try:
                answer = str(self.post(body, timeout).get('response', ''))
                result = parse_response(answer, prompt['categories'], prompt['use_schema'])
                if result['valid'] or attempt == 1:
                    break
            except ERRORS:
                if attempt == 1:
                    raise
                time.sleep(1.0)
        result['model'] = model
        result['latency_ms'] = int((time.monotonic() - start) * 1000)
        return result, answer


def keep_alive_value(value):
    """'-1' -> -1 (keep forever), '300' -> 300 s, '30m' stays a duration string."""
    try:
        return int(value)
    except (TypeError, ValueError):
        return value
