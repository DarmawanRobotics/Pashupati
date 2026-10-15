#!/usr/bin/env python3
import json
import math
import os
import threading
import time
import urllib.error
import urllib.request

import cv2
import numpy as np
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSDurabilityPolicy, QoSProfile
from robot_inspection.vlm import build_request, full_name, model_options, parse_response
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

# encoding -> (channels, cv2 conversion to BGR or None)
ENCODINGS = {
    'rgb8': (3, cv2.COLOR_RGB2BGR),
    'bgr8': (3, None),
    'rgba8': (4, cv2.COLOR_RGBA2BGR),
    'bgra8': (4, cv2.COLOR_BGRA2BGR),
    'mono8': (1, cv2.COLOR_GRAY2BGR),
}

# Changed while running (handheld, NETRA, `ros2 param set`); the rest is read at start.
LIVE_PARAMS = ('enabled', 'model', 'interval_sec')

# How often the status is republished even when nothing changed, and Ollama is asked for models.
STATUS_PERIOD_SEC = 2.0
MODELS_PERIOD_SEC = 30.0


class AnomalyDetectorNode(Node):
    """Early anomaly detection on the robot with a local VLM via Ollama."""

    def __init__(self):
        """Declare params, start the inspection loop and expose check_now."""
        super().__init__('anomaly_detector_node')
        defaults = {
            'enabled': True,
            'state_file': '~/.pashupati/inspection.json',
            'image_topic': '/camera/camera/color/image_raw',
            'ollama_url': 'http://127.0.0.1:11434',
            'model': 'qwen3-vl:2b-instruct',
            'models': [
                'qwen3-vl:2b-instruct',
                'moondream:latest',
                'ahmadwaqar/smolvlm2-2.2b-instruct:latest',
            ],
            'categories': ['trash', 'spill', 'stain'],
            'alarm_categories': ['trash', 'spill'],
            'use_schema': True,
            'describe': False,
            'debug': False,
            'keep_alive': '-1',
            'num_ctx': 2048,
            'timeout_sec': 30.0,
            'max_image_age_sec': 2.0,
            'max_image_width': 640,
            'interval_sec': 0.0,
            'cooldown_sec': 60.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        state_file = str(p('state_file')).strip()
        self._state_file = os.path.expanduser(state_file) if state_file else ''
        self.restore_choice()
        self._base_url = p('ollama_url').rstrip('/')
        self._url = self._base_url + '/api/generate'
        self._enabled = bool(p('enabled'))
        self._model = str(p('model')).strip()
        self._models = [str(m).strip() for m in p('models') if str(m).strip()]
        self._categories = [c for c in p('categories') if c]
        self._alarm = {c for c in p('alarm_categories') if c}
        self._use_schema = bool(p('use_schema'))
        self._describe = bool(p('describe'))
        self._debug = bool(p('debug'))
        self._keep_alive = keep_alive_value(p('keep_alive'))
        self._num_ctx = int(p('num_ctx'))
        self._timeout = float(p('timeout_sec'))
        self._max_age = float(p('max_image_age_sec'))
        self._max_width = int(p('max_image_width'))
        self._interval = float(p('interval_sec'))
        self._cooldown = float(p('cooldown_sec'))

        self._frame_lock = threading.Lock()
        self._frame = (0.0, None)  # (receive time, Image msg)
        self._busy = threading.Lock()
        self._stop = threading.Event()
        self._wake = threading.Event()  # a live parameter changed: re-check at once
        self._last_report: dict[str, float] = {}

        # what the handheld and NETRA show (anomaly_detector/status)
        self._status_lock = threading.Lock()
        self._loaded = None  # the model Ollama holds for us
        self._state = 'loading'
        self._error = ''
        self._installed = None  # {model: takes images (None: unknown)}; None until Ollama answered
        self._last = None  # latest inspection: category, latency, time
        self._inspections = 0
        self._reports = 0

        self._result_pub = self.create_publisher(String, 'anomaly_detector/result', 10)
        self._evidence_pub = self.create_publisher(
            CompressedImage, 'anomaly_detector/evidence', 10
        )
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._status_pub = self.create_publisher(String, 'anomaly_detector/status', latched)
        self.create_subscription(
            Image,
            p('image_topic'),
            self.on_image,
            qos_profile_sensor_data,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.create_service(
            Trigger, '~/check_now', self.check_now, callback_group=ReentrantCallbackGroup()
        )
        self.add_on_set_parameters_callback(self.on_parameters)
        self.create_timer(STATUS_PERIOD_SEC, self.publish_status)
        self._models_timer = self.create_timer(
            MODELS_PERIOD_SEC, self.refresh_models, callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.get_logger().info(
            f'model={self._model} enabled={self._enabled} categories={self._categories} '
            f'alarm={sorted(self._alarm)} schema={self._use_schema} describe={self._describe} '
            f'interval={self._interval}s'
        )
        self.publish_status()
        self._worker = threading.Thread(target=self.run, daemon=True)
        self._worker.start()
        threading.Thread(target=self.refresh_models, daemon=True).start()

    # ---------------------------------------------------------------- live parameters
    def on_parameters(self, params):
        """Accept enabled / model / interval_sec while running; the rest needs a restart."""
        updates = {}
        for param in params:
            if param.name not in LIVE_PARAMS:
                return SetParametersResult(
                    successful=False, reason=f'{param.name} is read at start, restart the node'
                )
            if param.name == 'model':
                model = str(param.value).strip()
                if not model:
                    return SetParametersResult(successful=False, reason='model is empty')
                updates['model'] = model
            elif param.name == 'interval_sec':
                interval = float(param.value)
                if not math.isfinite(interval):
                    return SetParametersResult(
                        successful=False, reason='interval_sec is not finite'
                    )
                updates['interval'] = interval
            else:
                updates['enabled'] = bool(param.value)
        if 'model' in updates:
            self._model = updates['model']
        if 'interval' in updates:
            self._interval = updates['interval']
        if 'enabled' in updates:
            self._enabled = updates['enabled']
        self.get_logger().info(
            f'inspection {"on" if self._enabled else "off"}, model {self._model}, '
            f'interval {self._interval:g} s'
        )
        self.save_choice()
        self._wake.set()
        self.publish_status()
        return SetParametersResult(successful=True)

    def restore_choice(self):
        """Apply the on / model / interval chosen live before a restart (state_file)."""
        if not self._state_file:
            return
        try:
            with open(self._state_file) as f:
                saved = json.load(f)
        except FileNotFoundError:
            return
        except (OSError, ValueError) as error:
            self.get_logger().warn(f'ignoring {self._state_file}: {error}')
            return
        if not isinstance(saved, dict):
            return
        params = []
        if isinstance(saved.get('enabled'), bool):
            params.append(Parameter('enabled', value=saved['enabled']))
        model = saved.get('model')
        if isinstance(model, str) and model.strip():
            params.append(Parameter('model', value=model.strip()))
        interval = saved.get('interval_sec')
        if (
            isinstance(interval, (int, float))
            and not isinstance(interval, bool)
            and math.isfinite(interval)
        ):
            params.append(Parameter('interval_sec', value=float(interval)))
        if params:
            self.set_parameters(params)
            restored = ', '.join(f'{param.name}={param.value}' for param in params)
            self.get_logger().info(f'live choice restored from {self._state_file}: {restored}')

    def save_choice(self):
        """Keep the live on / model / interval for the next start (state_file)."""
        if not self._state_file:
            return
        choice = {'enabled': self._enabled, 'model': self._model, 'interval_sec': self._interval}
        try:
            os.makedirs(os.path.dirname(self._state_file) or '.', exist_ok=True)
            partial = self._state_file + '.tmp'
            with open(partial, 'w') as f:
                json.dump(choice, f)
            os.replace(partial, self._state_file)
        except OSError as error:
            self.get_logger().warn(f'cannot keep the inspection choice: {error}')

    def pause(self, seconds: float):
        """Wait, but wake up at once when a live parameter changes or the node stops."""
        self._wake.wait(seconds)
        self._wake.clear()

    # ---------------------------------------------------------------- status
    def set_state(self, state: str, error: str = ''):
        """Record the inspection state; publish at once when it changed."""
        with self._status_lock:
            changed = (state, error) != (self._state, self._error)
            self._state, self._error = state, error
        if changed:
            self.publish_status()

    def status(self) -> dict:
        """Return the status JSON of anomaly_detector/status."""
        with self._status_lock:
            return {
                'enabled': self._enabled,
                'model': self._model,
                'state': self._state,
                'error': self._error,
                'interval_sec': self._interval,
                'models': model_options(self._models, self._installed, self._model),
                'last': self._last,
                'inspections': self._inspections,
                'reports': self._reports,
                'stamp': time.time(),
            }

    def publish_status(self):
        """Publish the status (latched): on every change and every STATUS_PERIOD_SEC."""
        try:
            self._status_pub.publish(String(data=json.dumps(self.status())))
        except Exception:  # noqa: B902  shutting down: the publisher is gone
            pass

    def refresh_models(self):
        """Ask Ollama which models it has and which of them take images."""
        try:
            with urllib.request.urlopen(self._base_url + '/api/tags', timeout=3.0) as reply:
                names = [m.get('name', '') for m in json.loads(reply.read()).get('models', [])]
        except (urllib.error.URLError, OSError, ValueError, AttributeError):
            return  # keep the last answer; the worker reports an unreachable Ollama
        installed = {}
        known = self._installed or {}
        for name in filter(None, names):
            installed[name] = known[name] if name in known else self.takes_images(name)
        with self._status_lock:
            self._installed = installed
        self.publish_status()

    def takes_images(self, model: str):
        """Return whether a model takes images (Ollama capabilities), None when unknown."""
        try:
            capabilities = self.post({'model': model, 'name': model}, 5.0, '/api/show').get(
                'capabilities'
            )
        except (urllib.error.URLError, OSError, ValueError, AttributeError):
            return None
        return 'vision' in capabilities if isinstance(capabilities, list) else None

    # ---------------------------------------------------------------- input
    def on_image(self, msg: Image):
        """Keep the latest camera frame; conversion happens only when inspecting."""
        with self._frame_lock:
            self._frame = (time.monotonic(), msg)

    def latest_frame(self):
        """Return (BGR image, header) of the latest fresh frame, or None."""
        with self._frame_lock:
            stamp, msg = self._frame
        if msg is None or time.monotonic() - stamp > self._max_age:
            return None
        spec = ENCODINGS.get(msg.encoding.lower())
        if spec is None:
            self.get_logger().error(
                f'unsupported encoding {msg.encoding}', throttle_duration_sec=30.0
            )
            return None
        channels, conversion = spec
        rows = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
        img = rows[:, : msg.width * channels].reshape(msg.height, msg.width, channels)
        img = img[:, :, 0] if channels == 1 else img
        img = cv2.cvtColor(img, conversion) if conversion is not None else img.copy()
        return img, msg.header

    def model_jpeg(self, img) -> bytes:
        """Downscale to max_image_width and encode as JPEG for the model."""
        if img.shape[1] > self._max_width:
            scale = self._max_width / img.shape[1]
            img = cv2.resize(
                img, (self._max_width, int(img.shape[0] * scale)), interpolation=cv2.INTER_AREA
            )
        return cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 85])[1].tobytes()

    # ---------------------------------------------------------------- model
    def post(self, body: dict, timeout: float, path: str = '/api/generate') -> dict:
        """POST a JSON body to Ollama and return the JSON reply."""
        request = urllib.request.Request(
            self._base_url + path,
            data=json.dumps(body).encode(),
            headers={'Content-Type': 'application/json'},
        )
        with urllib.request.urlopen(request, timeout=timeout) as reply:
            return json.loads(reply.read())

    def ask(self, model: str, jpeg: bytes) -> dict:
        """Run the model on one image; one retry covers an Ollama runner restart."""
        body = build_request(
            model,
            jpeg,
            self._categories,
            self._describe,
            self._use_schema,
            self._keep_alive,
            self._num_ctx,
        )
        start = time.monotonic()
        for attempt in range(2):
            try:
                answer = self.post(body, self._timeout).get('response', '')
                result = parse_response(answer, self._categories, self._use_schema)
                if result['valid'] or attempt == 1:
                    break
            except (urllib.error.URLError, TimeoutError, ValueError, OSError):
                if attempt == 1:
                    raise
                time.sleep(1.0)
        if not result['valid']:
            self.get_logger().warn(f'no valid category in answer: {answer!r}')
        if self._debug:
            self.get_logger().info(f'raw: {answer!r}')
        result.pop('valid')
        result.update({'model': model, 'latency_ms': int((time.monotonic() - start) * 1000)})
        return result

    def load_model(self, model: str) -> bool:
        """Load the chosen model into Ollama (unloading the one before it); True when loaded."""
        previous, self._loaded = self._loaded, None
        if previous and full_name(previous) != full_name(model):
            try:  # free the memory of the model before it: Jetson RAM is shared with the GPU
                self.post({'model': previous, 'keep_alive': 0}, 10.0)
            except (urllib.error.URLError, OSError, ValueError):
                pass
        self.set_state('loading')
        try:
            self.post({'model': model, 'prompt': '', 'keep_alive': self._keep_alive}, 180.0)
        except urllib.error.HTTPError as error:
            hint = f': ollama pull {model}' if error.code == 404 else ''
            self.set_state('error', f'Ollama could not load {model} (HTTP {error.code}){hint}')
            self.get_logger().error(self._error, throttle_duration_sec=60.0)
            return False
        except (urllib.error.URLError, OSError, ValueError) as error:
            self.set_state('error', f'Ollama not reachable at {self._base_url} ({error})')
            self.get_logger().warn(self._error, throttle_duration_sec=60.0)
            return False
        self._loaded = model  # also when it changed meanwhile: the next load unloads this one
        if model != self._model:
            return False
        self.get_logger().info(f'{model} loaded')
        self.set_state('running')
        return True

    # ---------------------------------------------------------------- inspection
    def inspect(self, source: str, wait: float):
        """Inspect the latest frame; returns (result or None, message)."""
        if not self._busy.acquire(timeout=wait):
            return None, 'inspection already running'
        try:
            model = self._loaded
            if model is None:
                return None, f'model {self._model} not loaded yet'
            frame = self.latest_frame()
            if frame is None:
                self.set_state('error', 'no recent camera frame')
                return None, 'no recent camera frame'
            img, header = frame
            result = self.ask(model, self.model_jpeg(img))
        except (urllib.error.URLError, TimeoutError, ValueError, OSError) as error:
            self.set_state('error', f'VLM unavailable: {error}')
            return None, f'VLM unavailable: {error}'
        finally:
            self._busy.release()

        result['is_anomaly'] = result['category'] in self._alarm
        result['source'] = source
        report = result['is_anomaly']
        if report and source == 'periodic':
            last = self._last_report.get(result['category'], -1e9)
            report = time.monotonic() - last > self._cooldown
        if report:
            self._last_report[result['category']] = time.monotonic()
            result['evidence_stamp'] = evidence_stamp(header.stamp)
            evidence = CompressedImage(header=header, format='jpeg')
            evidence.data = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 90])[1].tobytes()
            self._evidence_pub.publish(evidence)
            self._result_pub.publish(String(data=json.dumps(result)))

        with self._status_lock:
            self._inspections += 1
            self._reports += 1 if report else 0
            self._last = {
                'category': result['category'],
                'reported': report,
                'latency_ms': result['latency_ms'],
                'description': result['description'],
                'source': source,
                'at': time.time(),
            }
        self.set_state('running')
        self.publish_status()

        verdict = result['category'] if result['category'] != 'none' else 'clear'
        message = f"{verdict} ({result['latency_ms']} ms)"
        if result['description']:
            message += f" - {result['description']}"
        if report or self._debug:
            self.get_logger().info(f"{source}: {message}{'  [reported]' if report else ''}")
        return result, message

    def check_now(self, request, response):
        """Inspect at a stop point (called by path_follower_node); waits for a running check."""
        if not self._enabled:
            response.success = True
            response.message = 'inspection off'
            return response
        result, message = self.inspect('stop_point', wait=self._timeout)
        response.success = result is not None
        response.message = message
        self.get_logger().info(f'check_now: {message}')
        return response

    def run(self):
        """Keep the chosen model loaded and inspect every interval_sec (0 = back to back)."""
        while not self._stop.is_set():
            if not self._enabled:
                self.set_state('off')
                self.pause(5.0)
                continue
            if self._loaded != self._model:
                if not self.load_model(self._model) and not self._stop.is_set():
                    self.pause(10.0)  # retry, or load another model as soon as one is chosen
                continue
            if self._interval < 0.0:  # stop points only (check_now)
                self.set_state('running')
                self.pause(5.0)
                continue
            result, message = self.inspect('periodic', wait=0.5)
            if result is None and not self._stop.is_set():
                self.get_logger().warn(message, throttle_duration_sec=10.0)
                self.pause(1.0)
            # small gap so check_now can take the lock between periodic checks
            self.pause(max(self._interval, 0.05))

    def destroy_node(self):
        """Stop the inspection loop before tearing the node down."""
        self._stop.set()
        self._wake.set()
        self._worker.join(timeout=1.0)
        return super().destroy_node()


def keep_alive_value(value):
    """'-1' -> -1 (keep forever), '300' -> 300 s, '30m' stays a duration string."""
    try:
        return int(value)
    except (TypeError, ValueError):
        return value


def evidence_stamp(stamp) -> str:
    """Return a camera stamp as 'sec.nanosec', the key that links a result to its evidence."""
    return f'{stamp.sec}.{stamp.nanosec:09d}'


def main(args=None):
    """Spin the anomaly detector with a multi-threaded executor."""
    rclpy.init(args=args)
    node = AnomalyDetectorNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:  # noqa: B902
        if rclpy.ok():
            raise  # a real error while running
        # else: Ctrl-C shut the context down under spin(), a normal stop
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
