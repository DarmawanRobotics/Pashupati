"""
The handheld's camera, cloud and tag streams, and the robot pose: subscribed only while used.

Deserializing a 30 Hz camera, a 10 Hz cloud and every TF message cost a third of a Jetson core
while nobody was connected, so every subscription here lives only while a handheld wants it.
"""

from __future__ import annotations

import time

import numpy as np
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2

from dharma_utils.math import yaw_of
from dharma_utils.ros import SENSOR
from dharma_utils.ros.pointcloud import CloudError, read_xyz
from dharma_utils.ros.tf import TfCache
from pashupati_bridge.core import protocol
from pashupati_bridge.core.commands import camera_period

try:
    from apriltag_msgs.msg import AprilTagDetectionArray
except ImportError:  # no AprilTag detector here: no tag overlay on the handheld video
    AprilTagDetectionArray = None


class Streams:
    """Camera JPEGs, downsampled clouds and tag corners for one handheld."""

    def __init__(self, node, config, send_stream):
        self.node = node
        self.config = config
        self.send_stream = send_stream  # (stream id, payload)
        self.wanted = {'camera': False, 'lidar': False, 'video': False}
        self.subs: dict = {}
        self.tf: TfCache | None = None
        self.camera_period = 1.0 / config.camera_fps
        self.lidar_period = 1.0 / config.lidar_hz
        self.last_camera = self.last_lidar = 0.0
        self.big_frame_warned = False
        self.tags_seen: list = []
        self.tags_time = 0.0
        self.tags_size = (0, 0)
        video = ('camera', 'video')  # the tags go over either video
        kind = CompressedImage if config.camera_compressed else Image
        sources = {
            'camera': (('camera',), kind, config.camera_topic, self.on_camera),
            'lidar': (('lidar',), PointCloud2, config.lidar_topic, self.on_lidar),
        }
        if AprilTagDetectionArray is not None:
            sources['tags'] = (video, AprilTagDetectionArray, config.tags_topic, self.on_tags)
            info = config.tags_camera_info_topic
            sources['tags_info'] = (video, CameraInfo, info, self.on_tags_info)
        self.sources = {name: s for name, s in sources.items() if s[2]}

    def update(self, values: dict):
        """Apply a handheld's streams choice (camera, lidar, video, camera_fps)."""
        self.wanted.update({k: bool(v) for k, v in values.items() if k in self.wanted})
        period = camera_period(values.get('camera_fps'), self.config.camera_fps)
        if period is not None:
            self.camera_period = period

    def sync(self, live: bool):
        """Hold the stream and TF subscriptions only while a client uses them."""
        if live and self.tf is None:
            self.tf = TfCache(self.node)
        elif not live and self.tf is not None:
            self.tf.listener.unregister()
            self.tf = None
        for name, (streams, kind, topic, callback) in self.sources.items():
            want = live and any(self.wanted[s] for s in streams)
            sub = self.subs.get(name)
            if want and sub is None:
                self.subs[name] = self.node.create_subscription(kind, topic, callback, SENSOR)
            elif not want and sub is not None:
                self.node.destroy_subscription(self.subs.pop(name))

    def pose(self, map_frame: str, base_frame: str):
        """Return the robot [x, y, yaw] in the map frame, or None."""
        found = self.tf.lookup(map_frame, base_frame) if self.tf is not None else None
        if found is None:
            return None
        m = found.matrix
        return [round(float(m[0, 3]), 3), round(float(m[1, 3]), 3), round(yaw_of(m), 3)]

    # ---------------------------------------------------------------- camera
    def on_camera(self, msg):
        """Forward a throttled JPEG frame."""
        now = time.monotonic()
        if not self.wanted['camera'] or now - self.last_camera < self.camera_period:
            return
        self.last_camera = now
        jpeg = (
            self.shrink(bytes(msg.data)) if isinstance(msg, CompressedImage) else self.encode(msg)
        )
        if jpeg:
            self.send_stream(protocol.STREAM_CAMERA, jpeg)

    def shrink(self, jpeg: bytes) -> bytes:
        """
        Re-encode a driver JPEG bigger than camera_max_bytes (drivers default to quality 95).

        A frame travels as a burst of 1400-byte datagrams and one lost datagram loses it, so a
        150 kB JPEG rarely reaches the handheld whole over Wi-Fi.
        """
        if len(jpeg) <= self.config.camera_max_bytes:
            return jpeg
        try:
            import cv2
        except ImportError:
            if not self.big_frame_warned:
                self.big_frame_warned = True
                self.node.log.warning('camera frames too big for the handheld and no OpenCV to '
                                      'shrink them', bytes=len(jpeg))  # fmt: skip
            return jpeg
        img = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        return self.compress(img) if img is not None else b''

    def encode(self, msg: Image) -> bytes:
        """JPEG-encode a raw rgb8 / bgr8 image."""
        if msg.encoding not in ('rgb8', 'bgr8'):
            return b''
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step // 3, 3)
        img = img[:, : msg.width]
        if msg.encoding == 'rgb8':
            img = np.ascontiguousarray(img[:, :, ::-1])
        return self.compress(img)

    def compress(self, img) -> bytes:
        """JPEG at camera_jpeg_quality, no wider than camera_max_width (b'' without OpenCV)."""
        try:
            import cv2
        except ImportError:
            return b''
        height, width = img.shape[:2]
        if width > self.config.camera_max_width:
            size = (
                self.config.camera_max_width,
                int(height * self.config.camera_max_width / width),
            )
            img = cv2.resize(img, size, interpolation=cv2.INTER_AREA)
        quality = [cv2.IMWRITE_JPEG_QUALITY, self.config.camera_jpeg_quality]
        ok, buf = cv2.imencode('.jpg', img, quality)
        return buf.tobytes() if ok else b''

    # ---------------------------------------------------------------- lidar
    def on_lidar(self, msg: PointCloud2):
        """Forward a throttled, voxel-downsampled cloud in the robot frame."""
        now = time.monotonic()
        if not self.wanted['lidar'] or now - self.last_lidar < self.lidar_period:
            return
        self.last_lidar = now
        base = self.node.robot.frames.base
        extrinsic = self.tf.static(base, msg.header.frame_id) if self.tf is not None else None
        if extrinsic is None:
            return
        try:
            xyz = read_xyz(msg)
        except CloudError:
            return
        xyz = (xyz @ extrinsic[:3, :3].T + extrinsic[:3, 3]).astype(np.float32)
        xyz = protocol.voxel_downsample(xyz, self.config.lidar_voxel, self.config.lidar_max_points)
        self.send_stream(protocol.STREAM_LIDAR, protocol.pack_points(xyz))

    # ---------------------------------------------------------------- tags
    def on_tags(self, msg):
        """Keep the latest AprilTag detections (pixel corners of the detector's image)."""
        self.tags_seen = [(d.id, [(c.x, c.y) for c in d.corners]) for d in msg.detections]
        self.tags_time = time.monotonic()

    def on_tags_info(self, msg: CameraInfo):
        """Size of the image the tag detector sees."""
        self.tags_size = (msg.width, msg.height)

    def tags(self) -> list:
        """Tags in the camera image now, corners as image fractions, for the overlay."""
        if not self.tags_seen or time.monotonic() - self.tags_time > self.config.tags_max_age_sec:
            return []
        return protocol.tag_quads(self.tags_seen, *self.tags_size)
