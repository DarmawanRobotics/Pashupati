#!/usr/bin/env python3
import json
import threading
import time
import urllib.error
import urllib.request

import cv2
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from robot_inspection.vlm import build_request, parse_response
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


class AnomalyDetectorNode(Node):
    """Early anomaly detection on the robot with a local VLM via Ollama."""

    def __init__(self):
        """Declare params, start the inspection loop and expose check_now."""
        super().__init__('anomaly_detector_node')
        defaults = {
            'image_topic': '/camera/camera/color/image_raw',
            'ollama_url': 'http://127.0.0.1:11434',
            'model': 'qwen3-vl:2b-instruct',
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
        self._url = p('ollama_url').rstrip('/') + '/api/generate'
        self._model = p('model')
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
        self._last_report: dict[str, float] = {}

        self._result_pub = self.create_publisher(String, 'anomaly_detector/result', 10)
        self._evidence_pub = self.create_publisher(
            CompressedImage, 'anomaly_detector/evidence', 10
        )
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

        self.get_logger().info(
            f'model={self._model} categories={self._categories} alarm={sorted(self._alarm)} '
            f'schema={self._use_schema} describe={self._describe} interval={self._interval}s'
        )
        self._worker = threading.Thread(target=self.run, daemon=True)
        self._worker.start()

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
    def post(self, body: dict, timeout: float) -> dict:
        """POST a JSON body to Ollama and return the JSON reply."""
        request = urllib.request.Request(
            self._url, data=json.dumps(body).encode(), headers={'Content-Type': 'application/json'}
        )
        with urllib.request.urlopen(request, timeout=timeout) as reply:
            return json.loads(reply.read())

    def ask(self, jpeg: bytes) -> dict:
        """Run the model on one image; one retry covers an Ollama runner restart."""
        body = build_request(
            self._model,
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
        result.update(
            {'model': self._model, 'latency_ms': int((time.monotonic() - start) * 1000)}
        )
        return result

    # ---------------------------------------------------------------- inspection
    def inspect(self, source: str, wait: float):
        """Inspect the latest frame; returns (result or None, message)."""
        if not self._busy.acquire(timeout=wait):
            return None, 'inspection already running'
        try:
            frame = self.latest_frame()
            if frame is None:
                return None, 'no recent camera frame'
            img, header = frame
            result = self.ask(self.model_jpeg(img))
        except (urllib.error.URLError, TimeoutError, ValueError, OSError) as error:
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

        verdict = result['category'] if result['category'] != 'none' else 'clear'
        message = f"{verdict} ({result['latency_ms']} ms)"
        if result['description']:
            message += f" - {result['description']}"
        if report or self._debug:
            self.get_logger().info(f"{source}: {message}{'  [reported]' if report else ''}")
        return result, message

    def check_now(self, request, response):
        """Inspect at a stop point (called by path_follower_node); waits for a running check."""
        result, message = self.inspect('stop_point', wait=self._timeout)
        response.success = result is not None
        response.message = message
        self.get_logger().info(f'check_now: {message}')
        return response

    def run(self):
        """Warm the model up, then inspect every interval_sec (0 = back to back, < 0 = off)."""
        self.warm_up()
        if self._interval < 0.0:
            return
        while not self._stop.is_set():
            result, message = self.inspect('periodic', wait=0.5)
            if result is None and not self._stop.is_set():
                self.get_logger().warn(message, throttle_duration_sec=10.0)
                self._stop.wait(1.0)
            # small gap so check_now can take the lock between periodic checks
            self._stop.wait(max(self._interval, 0.05))

    def warm_up(self):
        """Load the model into memory so the first inspection is fast."""
        body = {'model': self._model, 'prompt': '', 'keep_alive': self._keep_alive}
        while not self._stop.is_set():
            try:
                self.post(body, timeout=180.0)
                self.get_logger().info(f'{self._model} loaded')
                return
            except urllib.error.HTTPError as error:
                hint = f': ollama pull {self._model}' if error.code == 404 else ''
                self.get_logger().error(
                    f'Ollama could not load {self._model} (HTTP {error.code}){hint}',
                    throttle_duration_sec=60.0,
                )
            except (urllib.error.URLError, OSError) as error:
                self.get_logger().warn(
                    f'Ollama not reachable at {self._url} ({error}), retrying',
                    throttle_duration_sec=60.0,
                )
            self._stop.wait(10.0)

    def destroy_node(self):
        """Stop the inspection loop before tearing the node down."""
        self._stop.set()
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
