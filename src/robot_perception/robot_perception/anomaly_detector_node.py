#!/usr/bin/env python3
import json
import threading

import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

from robot_perception.utils import ollama_client


class AnomalyDetectorNode(Node):
    """Periodically queries a vision-language model on the latest camera frame for mall anomalies."""

    def __init__(self):
        """Declare params, verify the Ollama model, and wire the subscription/publishers/service/timer."""
        super().__init__('anomaly_detector_node')
        self.declare_parameter('ollama_model', 'moondream')
        self.declare_parameter('check_interval_sec', 2.0)
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('jpeg_quality', 90)

        self._ollama_model = self.get_parameter('ollama_model').value
        self._check_interval = float(self.get_parameter('check_interval_sec').value)
        self._publish_annotated = bool(self.get_parameter('publish_annotated').value)
        self._jpeg_quality = int(self.get_parameter('jpeg_quality').value)

        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._latest_frame = None
        self._busy = False

        self.get_logger().info(f"Verifying Ollama model '{self._ollama_model}'...")
        try:
            ollama_client.ensure_model(self._ollama_model, logger=self.get_logger())
        except Exception as error:
            self.get_logger().error(f'Could not verify/pull Ollama model: {error}')

        self.create_subscription(Image, '/front_camera/image_raw', self.image_callback, qos_profile_sensor_data)

        self._result_pub = self.create_publisher(String, 'perception/anomaly_detector/result', 10)
        self._anomaly_pub = self.create_publisher(Bool, 'perception/anomaly_detector/is_anomaly', 10)

        self._annotated_pub = None
        if self._publish_annotated:
            self._annotated_pub = self.create_publisher(Image, 'perception/anomaly_detector/annotated_image', 1)

        self.create_service(Trigger, "perception/anomaly_detector/check_now", self.check_now_callback)
        self.create_timer(self._check_interval, self.timer_callback)

        self.get_logger().info(
            f'Anomaly detector ready. interval={self._check_interval}s model={self._ollama_model}'
        )

    def image_callback(self, msg: Image):
        """Cache the latest camera frame for the next inference pass."""
        with self._lock:
            self._latest_frame = msg

    def timer_callback(self):
        """Trigger a periodic inference pass."""
        self.trigger_inference()

    def check_now_callback(self, request, response):
        """Force an immediate inference pass, bypassing the busy check."""
        triggered = self.trigger_inference(force=True)
        response.success = triggered
        response.message = 'inference triggered' if triggered else 'no frame available or already busy'
        return response

    def trigger_inference(self, force: bool = False) -> bool:
        """Start an inference pass on the latest cached frame in a background thread, if not already busy."""
        with self._lock:
            if self._busy and not force:
                return False
            if self._latest_frame is None:
                return False
            frame_msg = self._latest_frame
            self._busy = True

        threading.Thread(target=self.run_inference, args=(frame_msg,), daemon=True).start()
        return True

    def run_inference(self, frame_msg: Image):
        """Encode the frame, query Ollama, and publish the result, annotated image, and anomaly flag."""
        try:
            cv_img = self._bridge.imgmsg_to_cv2(frame_msg, desired_encoding='bgr8')
            ok, buffer = cv2.imencode('.jpg', cv_img, [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality])
            if not ok:
                self.get_logger().warn('Failed to JPEG-encode frame, skipping check.')
                return

            result = ollama_client.query_frame(buffer.tobytes(), model_name=self._ollama_model)

            self._result_pub.publish(String(data=json.dumps(result)))
            self._anomaly_pub.publish(Bool(data=result['is_anomaly']))

            log = self.get_logger().warn if result['is_anomaly'] else self.get_logger().info
            log(f"[{result['type']}] {result['description']} ({result['latency_sec']:.2f}s)")

            if self._annotated_pub is not None:
                self.publish_annotated(cv_img, frame_msg.header, result)
        except Exception as error:
            self.get_logger().error(f'Inference failed: {error}')
        finally:
            with self._lock:
                self._busy = False

    def publish_annotated(self, cv_img, header, result: dict):
        """Draw a status border and label on the frame and publish it."""
        annotated = cv_img.copy()
        h, w = annotated.shape[:2]
        color = (0, 0, 255) if result['is_anomaly'] else (0, 200, 0)
        cv2.rectangle(annotated, (0, 0), (w - 1, h - 1), color, 6)
        status = f"ANOMALY: {result['type']}" if result['is_anomaly'] else 'STATUS: normal'
        cv2.putText(annotated, status, (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2, cv2.LINE_AA)
        out_msg = self._bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        out_msg.header = header
        self._annotated_pub.publish(out_msg)


def main(args=None):
    """Spin the anomaly detector node."""
    rclpy.init(args=args)
    node = AnomalyDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()