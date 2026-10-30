"""An in-memory Ollama for the core tests (the same methods as core.ollama.Ollama)."""

from __future__ import annotations

from pashupati_inspection.core.ollama import OllamaError

INSTALLED = {
    'qwen3-vl:2b-instruct': True,
    'moondream:latest': True,
    'llama3:latest': False,  # cannot see images
}


class FakeOllama:
    """Answers every image with `answer`; records loads and unloads."""

    def __init__(self, answer: str = 'trash'):
        self.answer = answer
        self.fail = False
        self.loads: list[str] = []
        self.unloads: list[str] = []
        self.asked: list[dict] = []
        self.keep_alive, self.num_ctx = -1, 2048

    def installed(self):
        return list(INSTALLED)

    def takes_images(self, model):
        return INSTALLED.get(model)

    def load(self, model):
        if model not in INSTALLED:
            raise OllamaError(f'Ollama could not load {model} (HTTP 404): ollama pull {model}')
        self.loads.append(model)

    def unload(self, model):
        self.unloads.append(model)
        return ''

    def ask(self, model, jpeg, prompt, timeout):
        if self.fail:
            raise OSError('runner busy')
        self.asked.append({'model': model, 'jpeg': jpeg, **prompt})
        valid = self.answer in prompt['categories'] or self.answer == 'none'
        result = {'category': self.answer if valid else 'none', 'description': '',
                  'valid': valid, 'model': model, 'latency_ms': 5}  # fmt: skip
        return result, f'{{"type": "{self.answer}"}}'
