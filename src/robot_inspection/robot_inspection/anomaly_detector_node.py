#!/usr/bin/env python3
import json
import threading
import time
import urllib.error
import urllib.request

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from robot_inspection.vlm import build_prompt, build_request, parse_response
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from std_srvs.srv import Trigger

try:
    import cv2
    import numpy as np
except ImportError:  # resizing is optional
    cv2 = None


class AnomalyDetectorNode(Node):
    """Early anomaly detection on the robot with a local VLM (Moondream via Ollama)."""

    def __init__(self):
        """Declare params, warm the model up and expose check_now."""
        super().__init__('anomaly_detector_node')
        defaults = {
            'image_topic': '/camera/camera/color/image_raw/compressed',
            'ollama_url': 'http://127.0.0.1:11434',
            'model': 'moondream',
            'categories': ['trash', 'spill', 'floor_damage', 'fallen_person'],
            'keep_alive': '30m',
            'timeout_sec': 25.0,
            'max_image_age_sec': 2.0,
            'max_image_width': 640,
            'min_confidence': 0.5,
            'periodic_check_sec': 0.0,
            'cooldown_sec': 60.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self._url = p('ollama_url').rstrip('/') + '/api/generate'
        self._model = p('model')
        self._categories = [c for c in p('categories') if c]
        self._prompt = build_prompt(self._categories)
        self._keep_alive = p('keep_alive')
        self._timeout = float(p('timeout_sec'))
        self._max_age = float(p('max_image_age_sec'))
        self._max_width = int(p('max_image_width'))
        self._min_conf = float(p('min_confidence'))
        self._cooldown = float(p('cooldown_sec'))
        self._image = (0.0, b'', None)
        self._lock = threading.Lock()
        self._busy = threading.Lock()
        self._last_report: dict[str, float] = {}

        images = MutuallyExclusiveCallbackGroup()
        work = ReentrantCallbackGroup()
        self._result_pub = self.create_publisher(String, 'anomaly_detector/result', 10)
        # The exact camera frame behind each reported result, so the evidence matches the verdict
        self._evidence_pub = self.create_publisher(
            CompressedImage, 'anomaly_detector/evidence', 10
        )
        self.create_subscription(
            CompressedImage,
            p('image_topic'),
            self.on_image,
            qos_profile_sensor_data,
            callback_group=images,
        )
        self.create_service(Trigger, '~/check_now', self.check_now, callback_group=work)
        period = float(p('periodic_check_sec'))
        if period > 0.0:
            self.create_timer(period, self.periodic_check, callback_group=work)
        threading.Thread(target=self.warm_up, daemon=True).start()

    def on_image(self, msg: CompressedImage):
        """Keep the latest camera frame with its header."""
        with self._lock:
            self._image = (time.monotonic(), bytes(msg.data), msg.header)

    def latest_frame(self):
        """Return (original jpeg, jpeg for the model, header), or None when the frame is stale."""
        with self._lock:
            stamp, jpeg, header = self._image
        if not jpeg or time.monotonic() - stamp > self._max_age:
            return None
        return jpeg, self.downscale(jpeg), header

    def downscale(self, jpeg: bytes) -> bytes:
        """Shrink a JPEG to max_image_width for faster inference (unchanged when smaller)."""
        if cv2 is None:
            return jpeg
        img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        if img is None or img.shape[1] <= self._max_width:
            return jpeg
        scale = self._max_width / img.shape[1]
        img = cv2.resize(img, (self._max_width, int(img.shape[0] * scale)))
        ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 85])
        return buf.tobytes() if ok else jpeg

    def ask(self, jpeg: bytes) -> dict:
        """Run the model on one image."""
        body = json.dumps(
            build_request(self._model, self._prompt, jpeg, self._keep_alive)
        ).encode()
        request = urllib.request.Request(
            self._url, data=body, headers={'Content-Type': 'application/json'}
        )
        start = time.monotonic()
        with urllib.request.urlopen(request, timeout=self._timeout) as reply:
            answer = json.loads(reply.read()).get('response', '')
        result = parse_response(answer, self._categories)
        result.update({'model': self._model, 'latency_ms': int((time.monotonic() - start) * 1000)})
        return result

    def inspect(self, source: str):
        """Inspect the latest frame; returns (result or None, message)."""
        frame = self.latest_frame()
        if frame is None:
            return None, 'no recent camera frame'
        original, jpeg, header = frame
        if not self._busy.acquire(blocking=False):
            return None, 'inspection already running'
        try:
            result = self.ask(jpeg)
        except (urllib.error.URLError, TimeoutError, ValueError, OSError) as error:
            return None, f'VLM unavailable: {error}'
        finally:
            self._busy.release()
        result['source'] = source
        report = result['is_anomaly'] and result['confidence'] >= self._min_conf
        if report and source == 'periodic':
            last = self._last_report.get(result['category'], 0.0)
            report = time.monotonic() - last > self._cooldown
        if report:
            self._last_report[result['category']] = time.monotonic()
            result['evidence_stamp'] = evidence_stamp(header.stamp)
            evidence = CompressedImage(header=header, format='jpeg')
            evidence.data = original
            self._evidence_pub.publish(evidence)
            self._result_pub.publish(String(data=json.dumps(result)))
        verdict = result['category'] if result['is_anomaly'] else 'clear'
        return result, f"{verdict} ({result['confidence']:.2f}, {result['latency_ms']} ms)"

    def check_now(self, request, response):
        """Inspect at a stop point (called by path_follower_node)."""
        result, message = self.inspect('stop_point')
        response.success = result is not None
        response.message = message
        self.get_logger().info(f'inspection: {message}')
        return response

    def periodic_check(self):
        """Inspect while driving; only new categories are reported within cooldown_sec."""
        result, message = self.inspect('periodic')
        if result and result['is_anomaly']:
            self.get_logger().info(f'periodic inspection: {message}')

    def warm_up(self):
        """Load the model into memory so the first inspection is fast."""
        body = json.dumps({'model': self._model, 'prompt': '', 'keep_alive': self._keep_alive})
        request = urllib.request.Request(
            self._url, data=body.encode(), headers={'Content-Type': 'application/json'}
        )
        for _ in range(30):
            try:
                with urllib.request.urlopen(request, timeout=120):
                    self.get_logger().info(f'{self._model} loaded')
                    return
            except urllib.error.HTTPError as error:
                if error.code == 404:
                    self.get_logger().error(
                        f'{self._model} is not installed in Ollama: ollama pull {self._model}',
                        throttle_duration_sec=60.0,
                    )
                else:
                    self.get_logger().warn(
                        f'Ollama could not load {self._model}: HTTP {error.code}',
                        throttle_duration_sec=60.0,
                    )
            except (urllib.error.URLError, OSError) as error:
                self.get_logger().warn(
                    f'Ollama not reachable at {self._url} ({error}), retrying',
                    throttle_duration_sec=60.0,
                )
            time.sleep(10.0)
        self.get_logger().error(f'could not load {self._model} from {self._url}')


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
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: B902  already shut down by the SIGINT handler
            pass


if __name__ == '__main__':
    main()
