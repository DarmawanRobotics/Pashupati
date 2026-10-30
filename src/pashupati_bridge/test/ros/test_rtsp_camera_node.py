"""The camera node publishes image, JPEG and camera_info for frames it is handed."""

from __future__ import annotations

import time

import numpy as np
from pashupati_bridge.rtsp_camera_node import RtspCameraNode
import pytest
import rclpy
from rclpy.parameter import Parameter
import sensor_msgs.msg as sensor_msgs

from dharma_utils.ros import SENSOR, compat

pytest.importorskip('cv2')
NAMESPACE = f'pashupati_camera_test_{int(time.time() * 1000) % 100000}'


@pytest.fixture
def camera():
    """Return a camera node without a stream and a listener."""
    compat.init()
    node = RtspCameraNode(
        namespace=NAMESPACE,
        parameter_overrides=[
            Parameter('url', value=''),
            Parameter('max_fps', value=0.0),
            Parameter('compressed_fps', value=0.0),
            Parameter('width', value=320),
            Parameter('default_hfov_deg', value=120.0),
            Parameter('height', value=240),
        ],
    )
    listener = rclpy.create_node('camera_listener', namespace=NAMESPACE)
    got = {'raw': [], 'jpeg': [], 'info': []}
    listener.create_subscription(sensor_msgs.Image, 'image_raw', got['raw'].append, SENSOR)
    listener.create_subscription(
        sensor_msgs.CompressedImage, 'image_raw/compressed', got['jpeg'].append, SENSOR
    )
    listener.create_subscription(sensor_msgs.CameraInfo, 'camera_info', got['info'].append, SENSOR)
    yield node, listener, got
    listener.destroy_node()
    node.destroy_node()
    compat.try_shutdown()


def spin(nodes, seconds=0.5):
    """Spin the nodes for a while."""
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        for node in nodes:
            rclpy.spin_once(node, timeout_sec=0.01)


def test_frames_are_resized_and_published(camera):
    """A 640x480 frame comes out as 320x240 bgr8, JPEG and an approximate camera_info."""
    node, listener, got = camera
    time.sleep(0.3)  # discovery
    for _ in range(3):
        node.process_frame(np.full((480, 640, 3), 128, dtype=np.uint8))
        spin([listener], 0.2)
    assert got['raw'] and got['jpeg'] and got['info']
    raw = got['raw'][-1]
    assert (raw.width, raw.height, raw.encoding, raw.step) == (320, 240, 'bgr8', 960)
    assert raw.header.frame_id == 'camera_color_optical_frame'
    assert bytes(got['jpeg'][-1].data[:2]) == b'\xff\xd8'
    info = got['info'][-1]
    assert (info.width, info.height) == (320, 240)
    assert info.k[0] == pytest.approx(160 / np.tan(np.radians(60.0)))
    assert node.frames == 3
