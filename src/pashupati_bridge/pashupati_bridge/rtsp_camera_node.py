"""
rtsp_camera_node: an RTSP camera (IP camera, gimbal pod, a robot's own camera) as the camera.

The bringup starts it with camera:=rtsp and camera_url:=..., remapped onto the camera topics
every consumer reads (image_raw, image_raw/compressed, camera_info under camera_topics), so the
inspection, the people, the tag detector and the handheld do not care where the frames come from.
GStreamer decodes (the Jetson hardware decoder when present), the node reconnects on its own and
rectifies the image when a calibration file is given.
"""

from __future__ import annotations

import threading
import time

import numpy as np
from rclpy.duration import Duration
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

from dharma_utils.ros import SENSOR, DharmaNode, run
from pashupati_bridge.core.calibration import fov_intrinsics, info_values, rectifier, try_load
from pashupati_bridge.core.config import CameraConfig
from pashupati_bridge.core.rtsp import decoder_candidates, gstreamer_pipeline, opencv_has_gstreamer

RECONNECT_MAX_SEC = 30.0


class RtspCameraNode(DharmaNode):
    """One RTSP stream as a ROS camera: image_raw, image_raw/compressed, camera_info."""

    def __init__(self, **kwargs):
        super().__init__('rtsp_camera', **kwargs)
        self.config = self.declare_config(CameraConfig)
        cfg = self.config.value
        self.decoders = decoder_candidates(cfg.decoder)
        self.latency = Duration(seconds=cfg.latency_sec)
        self.calibration, why = try_load(cfg.calibration_file)
        if self.calibration is None:
            self.log.warning('no camera calibration: camera_info assumes a pinhole without '
                             'distortion, tag poses are approximate', reason=why,
                             hfov_deg=cfg.default_hfov_deg)  # fmt: skip
        self.rectify = None  # (size, new K, maps), built for the first frame size
        self.info = None
        self.last_frame = self.last_jpeg = 0.0
        self.frames = 0
        self.decoder = ''
        self.raw_pub = self.create_publisher(Image, 'image_raw', SENSOR)
        self.jpeg_pub = self.create_publisher(CompressedImage, 'image_raw/compressed', SENSOR)
        self.info_pub = self.create_publisher(CameraInfo, 'camera_info', SENSOR)
        self.running = threading.Event()
        self.thread = None
        if not cfg.url:
            self.log.error('no url: nothing to read')
            return
        import cv2

        if not opencv_has_gstreamer(cv2.getBuildInformation()):
            self.log.warning('this OpenCV has no GStreamer (a pip opencv?): FFmpeg reader only')
        self.running.set()
        self.thread = threading.Thread(target=self.capture_loop, name='rtsp', daemon=True)
        self.thread.start()

    def open_stream(self):
        """Open the stream with the first decoder that works; None when none does."""
        import cv2

        cfg = self.config.value
        for decoder in self.decoders:
            if decoder == 'ffmpeg':
                capture = cv2.VideoCapture(cfg.url, cv2.CAP_FFMPEG)
            else:
                pipeline = gstreamer_pipeline(
                    cfg.url, cfg.codec, decoder, cfg.latency_ms, cfg.protocols
                )
                capture = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            if capture.isOpened():
                if decoder != self.decoder:
                    self.log.info('camera stream open', url=cfg.url, decoder=decoder)
                self.decoder = decoder
                self.decoders.remove(decoder)
                self.decoders.insert(0, decoder)  # tried first on the next reconnect
                return capture
            capture.release()
        return None

    def capture_loop(self):
        """Read frames until stopped; reconnect when the stream drops, backing off to 30 s."""
        first = self.config.reconnect_sec
        delay = first
        while self.running.is_set():
            capture = self.open_stream()
            if capture is None:
                if delay == first:
                    self.log.warning('cannot open the camera stream, retrying',
                                     url=self.config.url, every_sec=first)  # fmt: skip
                self.pause(delay)
                delay = min(delay * 2.0, RECONNECT_MAX_SEC)
                continue
            delay = first
            while self.running.is_set():
                ok, frame = capture.read()
                if not ok or frame is None:
                    self.log.warning('camera stream lost, reconnecting')
                    break
                self.process_frame(frame)
            capture.release()
            self.pause(min(first, 1.0))

    def pause(self, seconds: float):
        """Sleep, but wake up at once when the node stops."""
        end = time.monotonic() + seconds
        while self.running.is_set() and time.monotonic() < end:
            time.sleep(0.1)

    def process_frame(self, frame: np.ndarray):
        """Resize, rectify and publish one BGR frame."""
        import cv2

        cfg = self.config.value
        now = time.monotonic()
        if cfg.max_fps > 0.0 and now - self.last_frame < 1.0 / cfg.max_fps:
            return
        self.last_frame = now
        if cfg.width > 0 and cfg.height > 0 and frame.shape[1::-1] != (cfg.width, cfg.height):
            frame = cv2.resize(frame, (cfg.width, cfg.height), interpolation=cv2.INTER_AREA)
        frame = self.rectified(frame)
        stamp = (self.get_clock().now() - self.latency).to_msg()
        self.frames += 1
        if cfg.publish_raw:
            self.raw_pub.publish(self.image_msg(frame, stamp))
        period = 1.0 / cfg.compressed_fps if cfg.compressed_fps > 0.0 else 0.0
        if cfg.publish_compressed and now - self.last_jpeg >= period:
            ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, cfg.jpeg_quality])
            if ok:
                self.last_jpeg = now
                msg = CompressedImage(format='jpeg', data=jpeg.tobytes())
                msg.header.stamp, msg.header.frame_id = stamp, cfg.frame_id
                self.jpeg_pub.publish(msg)
        info = self.camera_info(frame.shape[1], frame.shape[0])
        info.header.stamp = stamp
        self.info_pub.publish(info)

    def rectified(self, frame: np.ndarray) -> np.ndarray:
        """Undistort the frame with the calibration, scaled to the frame size."""
        if self.calibration is None:
            return frame
        import cv2

        size = (frame.shape[1], frame.shape[0])
        if self.rectify is None or self.rectify[0] != size:
            new_k, maps = rectifier(self.calibration.scaled(*size), self.config.alpha)
            self.rectify = (size, new_k, maps)
            self.info = None
        _, _, (map1, map2) = self.rectify
        return cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)

    def camera_info(self, width: int, height: int) -> CameraInfo:
        """Return camera_info of the published (rectified) image."""
        if self.info is None or (self.info.width, self.info.height) != (width, height):
            if self.rectify is not None and self.rectify[0] == (width, height):
                k = self.rectify[1]
            else:
                k = fov_intrinsics(width, height, self.config.default_hfov_deg)
            values = info_values(k, width, height)
            info = CameraInfo(width=values['width'], height=values['height'])
            info.distortion_model = values['distortion_model']
            info.d, info.k, info.r, info.p = values['d'], values['k'], values['r'], values['p']
            info.header.frame_id = self.config.frame_id
            self.info = info
        return self.info

    def image_msg(self, frame: np.ndarray, stamp) -> Image:
        """Wrap a BGR frame as sensor_msgs/Image without cv_bridge."""
        msg = Image(height=int(frame.shape[0]), width=int(frame.shape[1]), encoding='bgr8')
        msg.header.stamp, msg.header.frame_id = stamp, self.config.frame_id
        msg.step = int(frame.shape[1]) * 3
        msg.data = np.ascontiguousarray(frame).tobytes()
        return msg

    def destroy_node(self):
        """Stop the capture thread before the node goes."""
        self.running.clear()
        if self.thread is not None:
            self.thread.join(timeout=3.0)
        return super().destroy_node()


def main(args=None):
    """Run the RTSP camera."""
    run(RtspCameraNode, args=args)


if __name__ == '__main__':
    main()
