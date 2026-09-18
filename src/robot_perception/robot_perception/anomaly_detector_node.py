"""ROS2 node: subscribes to a camera image topic, periodically runs Moondream
(via Ollama) anomaly detection (trash / spill / fallen_person) on the latest
frame, and publishes the result.

Sits downstream of an existing camera driver node (RealSense, etc.) -- it
does not open a video device itself.
"""

import json
import threading

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from robot_perception.utils.ollama_client import ollama_client


class AnomalyDetectorNode(Node):

    def __init__(self):
        super().__init__("anomaly_detector_node")

        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("result_topic", "anomaly_detector/result")
        self.declare_parameter("is_anomaly_topic", "anomaly_detector/is_anomaly")
        self.declare_parameter("annotated_topic", "anomaly_detector/annotated_image")
        self.declare_parameter("ollama_model", "moondream")
        self.declare_parameter("check_interval_sec", 2.0)
        self.declare_parameter("publish_annotated", True)
        self.declare_parameter("jpeg_quality", 90)

        self.image_topic = self.get_parameter("image_topic").value
        self.ollama_model = self.get_parameter("ollama_model").value
        self.check_interval = self.get_parameter("check_interval_sec").value
        self.publish_annotated = self.get_parameter("publish_annotated").value
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)

        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._latest_frame = None
        self._busy = False
        self._processor_logged = False

        self.get_logger().info(f"Verifying Ollama model '{self.ollama_model}'...")
        try:
            ollama_client.ensure_model(self.ollama_model, logger=self.get_logger())
        except Exception as e:
            self.get_logger().error(f"Could not verify/pull Ollama model: {e}")

        self._sub = self.create_subscription(
            Image, self.image_topic, self._image_callback, qos_profile_sensor_data
        )

        self._result_pub = self.create_publisher(
            String, self.get_parameter("result_topic").value, 10
        )
        self._anomaly_pub = self.create_publisher(
            Bool, self.get_parameter("is_anomaly_topic").value, 10
        )
        self._annotated_pub = None
        if self.publish_annotated:
            self._annotated_pub = self.create_publisher(
                Image, self.get_parameter("annotated_topic").value, 1
            )

        self._srv = self.create_service(
            Trigger, "~/check_now", self._check_now_callback
        )
        self._timer = self.create_timer(self.check_interval, self._timer_callback)

        self.get_logger().info(
            f"Anomaly detector ready. image_topic={self.image_topic} "
            f"interval={self.check_interval}s model={self.ollama_model}"
        )

    def _image_callback(self, msg: Image):
        with self._lock:
            self._latest_frame = msg

    def _timer_callback(self):
        self._trigger_inference()

    def _check_now_callback(self, request, response):
        triggered = self._trigger_inference(force=True)
        response.success = triggered
        response.message = (
            "inference triggered" if triggered else "no frame available or already busy"
        )
        return response

    def _trigger_inference(self, force: bool = False) -> bool:
        with self._lock:
            if self._busy and not force:
                return False
            if self._latest_frame is None:
                return False
            frame_msg = self._latest_frame
            self._busy = True

        threading.Thread(
            target=self._run_inference, args=(frame_msg,), daemon=True
        ).start()
        return True

    def _run_inference(self, frame_msg: Image):
        try:
            cv_img = self._bridge.imgmsg_to_cv2(frame_msg, desired_encoding="bgr8")
            ok, buffer = cv2.imencode(
                ".jpg", cv_img, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
            )
            if not ok:
                self.get_logger().warn("Failed to JPEG-encode frame, skipping check.")
                return

            result = ollama_client.query_frame(
                buffer.tobytes(), model_name=self.ollama_model
            )

            if not self._processor_logged:
                self._processor_logged = True
                self.get_logger().info(
                    f"Ollama processor status: {ollama_client.get_processor_status(self.ollama_model)}"
                )

            self._result_pub.publish(String(data=json.dumps(result)))
            self._anomaly_pub.publish(Bool(data=result["is_anomaly"]))

            log_fn = (
                self.get_logger().warn
                if result["is_anomaly"]
                else self.get_logger().info
            )
            log_fn(
                f"[{result['type']}] {result['description']} ({result['latency_sec']:.2f}s)"
            )

            if self._annotated_pub is not None:
                self._publish_annotated(cv_img, frame_msg.header, result)
        except Exception as e:
            self.get_logger().error(f"Inference failed: {e}")
        finally:
            with self._lock:
                self._busy = False

    def _publish_annotated(self, cv_img, header, result: dict):
        annotated = cv_img.copy()
        h, w = annotated.shape[:2]
        color = (0, 0, 255) if result["is_anomaly"] else (0, 200, 0)
        cv2.rectangle(annotated, (0, 0), (w - 1, h - 1), color, 6)
        status = (
            f"ANOMALY: {result['type']}" if result["is_anomaly"] else "STATUS: normal"
        )
        cv2.putText(
            annotated,
            status,
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            color,
            2,
            cv2.LINE_AA,
        )
        out_msg = self._bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        out_msg.header = header
        self._annotated_pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = AnomalyDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
