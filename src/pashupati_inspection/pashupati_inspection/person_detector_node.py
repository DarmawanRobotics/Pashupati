"""person_detector_node: people in the camera image, counted and placed on the map floor."""

from __future__ import annotations

import json
import math
import time

from geometry_msgs.msg import Pose, PoseArray
import numpy as np
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String

from dharma_utils.ros import SENSOR, DharmaNode, run
from pashupati_inspection.core.choice import read_choice, write_choice
from pashupati_inspection.core.config import PeopleConfig
from pashupati_inspection.core.images import decode_jpeg, to_bgr
from pashupati_inspection.core.people import PeopleState, detection_rate
from pashupati_inspection.core.person_projection import feet_pixel, ground_point, pixel_ray

STATUS_PERIOD_SEC = 2.0  # perception/people_status also when nothing changes
RELOAD_AFTER_SEC = 30.0  # retry a model that failed to load


class PersonDetectorNode(DharmaNode):
    """YOLO people on the camera: counted always, placed on the map while localized."""

    def __init__(self, **kwargs):
        super().__init__('person_detector_node', **kwargs)
        self.config = self.declare_config(PeopleConfig)
        self.robot = self.declare_robot()
        cfg = self.config.value
        self.people = PeopleState(cfg.enabled, cfg.model_path, cfg.image_topic)
        self.detector = None
        self.load_tried = -math.inf
        self.last = 0.0
        self.intrinsics = None
        self.people_pub = self.create_publisher(PoseArray, 'perception/people', 10)
        self.status_pub = self.latched(String, 'perception/people_status')
        self.create_subscription(CameraInfo, cfg.camera_info_topic, self.on_info, 10)
        kind = CompressedImage if cfg.compressed else Image
        self.create_subscription(kind, cfg.image_topic, self.on_image, SENSOR)
        self.config.on_change(self.on_config)
        self.restore_choice()
        self.every(STATUS_PERIOD_SEC, self.publish_status)
        self.publish_status()

    # ---------------------------------------------------------------- live choices
    def on_config(self, _old: PeopleConfig, new: PeopleConfig):
        """Apply on / off, rate and confidence; keep on / off for the next start."""
        self.people.enabled = new.enabled
        if self.detector is not None:
            self.detector.confidence = new.confidence
        self.log.info('person detection', on=new.enabled, rate_hz=new.rate_hz)
        error = write_choice(new.state_file, {'enabled': new.enabled})
        if error:
            self.log.warning('cannot keep the person detection choice', error=error)
        self.publish_status()

    def restore_choice(self):
        """Apply the on / off chosen live before a restart."""
        values, error = read_choice(self.config.state_file, ('enabled',))
        if error:
            self.log.warning('ignoring the saved person detection choice', error=error)
        if values:
            self.set_parameters([Parameter('enabled', value=values['enabled'])])

    # ---------------------------------------------------------------- status
    def status(self) -> dict:
        """Return what perception/people_status carries."""
        return self.people.status(self.config.rate_hz, self.detector is not None)

    def publish_status(self):
        """Publish the status (latched): after every detection and every STATUS_PERIOD_SEC."""
        self.status_pub.publish(String(data=json.dumps(self.status())))

    # ---------------------------------------------------------------- detection
    def ensure_detector(self) -> bool:
        """Load the model (again RELOAD_AFTER_SEC after a failure); True when ready."""
        if self.detector is not None:
            return True
        now = time.monotonic()
        if now - self.load_tried < RELOAD_AFTER_SEC:
            return False
        self.load_tried = now
        cfg = self.config.value
        try:
            from dharma_utils.io import expand
            from pashupati_inspection.core.person_detector import create_detector

            self.detector = create_detector(
                expand(cfg.model_path), cfg.confidence, cfg.image_size, cfg.threads, cfg.backend
            )
        except Exception as error:  # missing model or runtime: report it, retry later
            self.people.load_error = str(error)
            self.log.error('person detector not loaded', error=str(error))
            self.publish_status()
            return False
        self.people.load_error, self.people.backend = '', self.detector.backend
        self.log.info('person detector', model=cfg.model_path, backend=self.detector.backend)
        return True

    def on_info(self, msg: CameraInfo):
        """Keep the camera intrinsics."""
        self.intrinsics = np.array(msg.k, dtype=float).reshape(3, 3)

    def decode(self, msg) -> np.ndarray | None:
        """Return the BGR image of a compressed or raw message."""
        if isinstance(msg, CompressedImage):
            return decode_jpeg(bytes(msg.data))
        return to_bgr(msg.encoding, msg.height, msg.width, msg.step, msg.data)

    def on_image(self, msg):
        """Detect people at the detection rate; place them on the map when localized."""
        now = time.monotonic()
        self.people.frame_time = now
        rate = detection_rate(self.config.rate_hz, self.people.backend)
        if not self.config.enabled or now - self.last < 1.0 / rate:
            return
        self.last = now
        if not self.ensure_detector():
            return
        image = self.decode(msg)
        if image is None:
            return
        start = time.monotonic()
        boxes = self.detector.detect(image)
        latency = time.monotonic() - start
        placed = self.place(boxes, msg.header)
        height, width = image.shape[:2]
        self.people.detected(boxes, width, height, placed, latency)
        self.publish_status()

    def place(self, boxes: list[tuple], header) -> int:
        """Publish the people standing on the floor in the map frame; return how many."""
        frames = self.robot.frames
        t_base_cam = self.tf.static(frames.base, header.frame_id)
        found = self.tf.lookup(frames.map, frames.base)
        if t_base_cam is None or found is None or self.intrinsics is None:
            return 0  # not localized (or no camera geometry): count the people, place none
        people = PoseArray()
        people.header.stamp, people.header.frame_id = header.stamp, frames.map
        floor_z = -self.robot.base_height
        for box in boxes:
            ray = pixel_ray(*feet_pixel(box), self.intrinsics)
            point = ground_point(ray, t_base_cam, floor_z, self.config.max_range)
            if point is None:
                continue
            x, y, _ = (found.matrix @ np.append(point, 1.0))[:3]
            pose = Pose()
            pose.position.x, pose.position.y = float(x), float(y)
            pose.orientation.w = float(box[4])  # the score: poses of people carry no heading
            people.poses.append(pose)
        self.people_pub.publish(people)
        return len(people.poses)


def main(args=None):
    """Run the person detector."""
    run(PersonDetectorNode, args=args)


if __name__ == '__main__':
    main()
