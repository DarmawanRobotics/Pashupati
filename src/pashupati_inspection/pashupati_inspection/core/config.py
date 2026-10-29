"""Configuration of anomaly_detector_node and person_detector_node."""

from __future__ import annotations

from dataclasses import dataclass
import math

from dharma_utils.config import param
from pashupati_inspection.core.vlm import category_names, parse_descriptions

MODELS = ('qwen3-vl:2b-instruct', 'moondream:latest', 'ahmadwaqar/smolvlm2-2.2b-instruct:latest')
PEOPLE_BACKENDS = ('auto', 'onnxruntime', 'opencv', 'ultralytics')


@dataclass(frozen=True)
class AnomalyConfig:
    """anomaly_detector_node; the live ones change from the handheld, NETRA or param set."""

    enabled: bool = param(True, doc='inspect (off: stop points answer "inspection off")')
    state_file: str = param(
        '~/.pashupati/inspection.json', live=False, doc='live choices kept across restarts'
    )
    image_topic: str = param(
        '/camera/camera/color/image_raw', live=False, doc='sensor_msgs/Image of the camera'
    )
    ollama_url: str = param('http://127.0.0.1:11434', live=False, doc='Ollama server')
    model: str = param('qwen3-vl:2b-instruct', doc='Ollama vision model')
    models: tuple[str, ...] = param(MODELS, live=False, doc='offered on the handheld')
    categories: tuple[str, ...] = param(
        ('trash', 'spill', 'stain'), doc='finding types the model chooses from (plus none)'
    )
    alarm_categories: tuple[str, ...] = param(
        ('trash', 'spill'), doc='only these are reported to NETRA'
    )
    category_descriptions: tuple[str, ...] = param(
        (), doc='own text per type: "name: what the model looks for"'
    )
    use_schema: bool = param(True, live=False, doc='structured answer (false: FLOOR:/TYPE:)')
    describe: bool = param(False, live=False, doc='the model also describes the floor')
    debug: bool = param(False, live=False, doc='log every inspection and the raw answer')
    keep_alive: str = param('-1', live=False, doc='Ollama keep_alive: -1 forever, or 30m')
    num_ctx: int = param(2048, min=256, max=131072, live=False, doc='model context')
    timeout_sec: float = param(
        30.0, min=1.0, max=600.0, unit='s', live=False, doc='one model answer'
    )
    max_image_age_sec: float = param(
        2.0, min=0.1, max=60.0, unit='s', live=False, doc='older camera frames are not used'
    )
    max_image_width: int = param(640, min=64, max=4096, live=False, doc='downscaled for the model')
    interval_sec: float = param(
        0.0,
        min=-1.0,
        max=3600.0,
        unit='s',
        doc='0 back to back, > 0 every N s (queued), < 0 only at stop points',
    )
    cooldown_sec: float = param(
        60.0, min=0.0, max=86400.0, unit='s', live=False, doc='a periodic type is reported once'
    )
    queue_dir: str = param(
        '~/.pashupati/inspection_queue', live=False, doc='interval queue on disk ("": memory)'
    )
    queue_max: int = param(50, min=1, max=10000, live=False, doc='queued frames at most')
    queue_min_change: float = param(
        12.0, min=0.0, max=255.0, live=False, doc='grey levels a frame must change to queue it'
    )

    def validate(self) -> list[str]:
        """Return problems of the finding types and the model name."""
        problems = []
        if not self.model.strip():
            problems.append('model is empty')
        if not math.isfinite(self.interval_sec):
            problems.append('interval_sec is not finite')
        for check in (
            lambda: category_names(self.categories),
            lambda: parse_descriptions(self.category_descriptions),
        ):
            try:
                check()
            except ValueError as error:
                problems.append(str(error))
        return problems


@dataclass(frozen=True)
class PeopleConfig:
    """person_detector_node; enabled, rate_hz and confidence change live."""

    enabled: bool = param(True, doc='detect people')
    state_file: str = param(
        '~/.pashupati/people.json', live=False, doc='live on / off kept across restarts'
    )
    image_topic: str = param(
        '/camera/camera/color/image_raw', live=False, doc='camera image (raw or compressed)'
    )
    compressed: bool = param(False, live=False, doc='image_topic is a CompressedImage')
    camera_info_topic: str = param(
        '/camera/camera/color/camera_info', live=False, doc='intrinsics for the floor position'
    )
    model_path: str = param(
        '~/dev/Pashupati/models/yolo11n-416.onnx', live=False, doc='.onnx, .pt or .engine'
    )
    backend: str = param('auto', live=False, choices=PEOPLE_BACKENDS, doc='inference runtime')
    threads: int = param(2, min=1, max=64, live=False, doc='CPU threads per inference')
    confidence: float = param(0.4, min=0.05, max=0.95, doc='detection score threshold')
    image_size: int = param(416, min=32, max=4096, live=False, doc='OpenCV / Ultralytics input')
    rate_hz: float = param(
        0.0, min=0.0, max=30.0, unit='Hz', doc='detections per second; 0: 5 on GPU, 2 on CPU'
    )
    max_range: float = param(
        12.0, min=0.5, max=100.0, unit='m', live=False, doc='farther people are not placed'
    )

    def validate(self) -> list[str]:
        """Return problems of the detection rate (0, or 0.1 to 30)."""
        if 0.0 < self.rate_hz < 0.1:
            return ['rate_hz: 0 (auto), 0.1 to 30']
        return []
