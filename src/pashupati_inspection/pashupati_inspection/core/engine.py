"""
The inspection itself: which model is loaded, what an answer means, what the status says.

No ROS: the node feeds it frames and publishes what it decides. Thread safe: the worker thread
and the stop-point service both inspect, one at a time.
"""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time

from pashupati_inspection.core.config import AnomalyConfig
from pashupati_inspection.core.ollama import ERRORS, Ollama, OllamaError
from pashupati_inspection.core.vlm import (
    CATEGORY_HINTS,
    category_names,
    describe_category,
    full_name,
    model_options,
    parse_descriptions,
)

PERIODIC = 'periodic'
STOP_POINT = 'stop_point'


@dataclass(frozen=True)
class Verdict:
    """One inspection: the result JSON, whether it is reported, the message for the follower."""

    result: dict
    report: bool
    message: str


def prompt_of(config: AnomalyConfig) -> dict:
    """Return the prompt settings of vlm.build_request."""
    return {
        'categories': category_names(config.categories),
        'describe': config.describe,
        'use_schema': config.use_schema,
        'hints': parse_descriptions(config.category_descriptions),
    }


def message_of(result: dict) -> str:
    """Return 'clear (850 ms)' or 'trash (850 ms) - a can on the floor'."""
    verdict = result['category'] if result['category'] != 'none' else 'clear'
    message = f'{verdict} ({result["latency_ms"]} ms)'
    return f'{message} - {result["description"]}' if result.get('description') else message


class InspectionEngine:
    """Keeps the chosen model loaded in Ollama and judges what it sees."""

    def __init__(self, config: AnomalyConfig, ollama: Ollama | None = None):
        self.config = config
        self.ollama = ollama or Ollama(config.ollama_url, config.keep_alive, config.num_ctx)
        self.busy = threading.Lock()
        self.lock = threading.Lock()
        self.loaded: str | None = None  # the model Ollama holds for us
        self.loading: str | None = None
        self.state, self.error = 'loading', ''
        self.installed: dict | None = None  # {model: takes images}; None until Ollama answered
        self.last: dict | None = None
        self.inspections = self.reports = 0
        self.last_report: dict[str, float] = {}
        self.last_answer = ('', True)  # raw model answer, whether it named a known type

    def set_state(self, state: str, error: str = '') -> bool:
        """Record the state; True when it changed."""
        with self.lock:
            changed = (state, error) != (self.state, self.error)
            self.state, self.error = state, error
        return changed

    def refresh_models(self) -> bool:
        """Ask Ollama for its models and which take images; True when it answered."""
        names = self.ollama.installed()
        if names is None:
            return False
        known = self.installed or {}
        installed = {n: known[n] if n in known else self.ollama.takes_images(n) for n in names}
        with self.lock:
            self.installed = installed
        return True

    def load(self, model: str) -> bool:
        """Load the model (unloading the one before); True when it is the chosen one now."""
        previous, self.loaded = self.loaded, None
        if previous and full_name(previous) != full_name(model):
            self.ollama.unload(previous)
        self.set_state('loading')
        self.loading = model
        try:
            self.ollama.load(model)
        except OllamaError as error:
            self.set_state('error', str(error))
            return False
        finally:
            self.loading = None
        self.loaded = model  # also when it changed meanwhile: the next load frees this one
        if model != self.config.model:
            return False
        self.set_state('running')
        return True

    def unload(self) -> str | None:
        """Free the loaded (or loading) model; return its name, or None."""
        model, self.loaded = self.loaded or self.loading, None
        if model:
            self.ollama.unload(model)
        return model

    def judge(self, result: dict, source: str, captured_at: float) -> bool:
        """Return whether a result is reported: alarm types, periodic ones once per cooldown."""
        report = result['category'] in set(self.config.alarm_categories)
        if report and source == PERIODIC:  # cooldown in capture time: queued frames are older
            last = self.last_report.get(result['category'], -1e18)
            report = captured_at - last > self.config.cooldown_sec
        if report:
            self.last_report[result['category']] = captured_at
        return report

    def inspect(self, jpeg: bytes, source: str, captured_at: float, wait: float):
        """Ask the model about one frame; return (Verdict or None, message)."""
        if not self.busy.acquire(timeout=wait):
            return None, 'inspection already running'
        try:
            model = self.loaded
            if model is None:
                return None, f'model {self.config.model} not loaded yet'
            result, answer = self.ollama.ask(
                model, jpeg, prompt_of(self.config), self.config.timeout_sec
            )
        except ERRORS as error:
            self.set_state('error', f'VLM unavailable: {error}')
            return None, f'VLM unavailable: {error}'
        finally:
            self.busy.release()
        self.last_answer = (answer, bool(result.pop('valid', True)))
        result['is_anomaly'] = result['category'] in set(self.config.alarm_categories)
        result['source'] = source
        result['captured_at'] = captured_at
        report = self.judge(result, source, captured_at)
        self.record(result, report, captured_at)
        self.set_state('running')
        return Verdict(result, report, message_of(result)), message_of(result)

    def record(self, result: dict, report: bool, captured_at: float):
        """Keep the last inspection and the counters for the status."""
        now = time.time()
        with self.lock:
            self.inspections += 1
            self.reports += 1 if report else 0
            self.last = {
                'category': result['category'],
                'reported': report,
                'latency_ms': result['latency_ms'],
                'description': result['description'],
                'source': result['source'],
                'at': now,
                'waited_sec': round(max(now - captured_at, 0.0), 1),
            }

    def status(self, queue=None) -> dict:
        """Return anomaly_detector/status (shown on the handheld, sent to NETRA)."""
        config = self.config
        hints = parse_descriptions(config.category_descriptions)
        alarm = set(config.alarm_categories)
        with self.lock:
            return {
                'enabled': config.enabled,
                'model': config.model,
                'state': self.state,
                'error': self.error,
                'interval_sec': config.interval_sec,
                'models': model_options(list(config.models), self.installed, config.model),
                'categories': [
                    {
                        'name': name,
                        'desc': describe_category(name, hints),
                        'report': name in alarm,
                        'builtin': name in CATEGORY_HINTS,
                        'own_desc': name in hints,
                    }
                    for name in category_names(config.categories)
                ],
                'last': self.last,
                'inspections': self.inspections,
                'reports': self.reports,
                'queue': len(queue) if queue is not None else 0,
                'queue_max': queue.limit if queue is not None else 0,
                'queue_dropped': queue.dropped if queue is not None else 0,
                'stamp': time.time(),
            }
