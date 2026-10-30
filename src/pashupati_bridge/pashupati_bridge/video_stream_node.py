"""
video_stream_node: H.264 video of the camera for the handheld, published to MediaMTX (WebRTC).

MediaMTX starts this node when the handheld opens the WebRTC video (runOnDemand in the
configuration the pashupati bringup writes) and stops it with SIGINT once nobody watches.
Frames go through GStreamer: the Jetson hardware encoder when it works, else x264 or OpenH264,
then over RTSP to MediaMTX on the loopback interface.
"""

from __future__ import annotations

import sys
import time

import rclpy
from sensor_msgs.msg import Image

from dharma_utils.ros import SENSOR, DharmaNode, compat
from pashupati_bridge.core import video
from pashupati_bridge.core.config import VideoConfig

try:
    import gi

    gi.require_version('Gst', '1.0')
    from gi.repository import GLib, Gst
except (ImportError, ValueError):  # python3-gi or gir1.2-gstreamer-1.0 missing
    GLib = Gst = None


def set_optional(element, encoder: str):
    """Set the OPTIONAL properties of an encoder that this GStreamer version has."""
    if element is None:
        return
    for name, value in video.OPTIONAL.get(encoder, {}).items():
        if element.find_property(name) is not None:
            element.set_property(name, value)


def probe(encoder: str, timeout_sec: float = 5.0) -> str | None:
    """Encode test frames with one encoder: None when it works, else the reason it does not."""
    try:
        pipeline = Gst.parse_launch(video.probe_description(encoder))
    except GLib.Error as error:  # the element is not installed
        return error.message
    set_optional(pipeline.get_by_name('enc'), encoder)
    try:
        pipeline.set_state(Gst.State.PLAYING)
        message = pipeline.get_bus().timed_pop_filtered(
            int(timeout_sec * Gst.SECOND), Gst.MessageType.EOS | Gst.MessageType.ERROR
        )
    finally:
        pipeline.set_state(Gst.State.NULL)
    if message is None:
        return f'no frames within {timeout_sec:g} s'
    if message.type == Gst.MessageType.ERROR:
        return message.parse_error()[0].message
    return None


class VideoStreamNode(DharmaNode):
    """Encode the camera to H.264 and publish it to MediaMTX while the handheld watches."""

    def __init__(self, **kwargs):
        super().__init__('video_stream_node', **kwargs)
        self.config = self.declare_config(VideoConfig)
        cfg = self.config.value
        self.keyframes = max(1, round(cfg.fps * cfg.keyframe_sec))
        self.encoder = self.choose_encoder(cfg.encoder)
        self.pipeline = self.src = self.shape = None
        self.last = 0.0
        self.frames = 0
        self.warned_encoding = False
        self.running = True
        self.exit_code = 0
        self.create_subscription(Image, cfg.image_topic, self.on_image, SENSOR)
        self.every(0.2, self.poll_bus)
        self.waiting = self.every(5.0, self.warn_no_frames)
        self.log.info('video', encoder=self.encoder, url=cfg.rtsp_url, camera=cfg.image_topic)

    def choose_encoder(self, wanted: str) -> str:
        """Return the first encoder that encodes test frames here."""
        candidates = video.ENCODERS if wanted == 'auto' else (wanted,)
        for encoder in candidates:
            reason = probe(encoder)
            if reason is None:
                return encoder
            self.log.info('encoder does not work here', encoder=encoder, reason=reason)
        raise RuntimeError(
            f'no working H.264 encoder ({", ".join(candidates)}): scripts/setup.sh install webrtc'
        )

    def warn_no_frames(self):
        """Say so while the camera sends nothing (the handheld waits on a black video)."""
        if self.frames:
            self.waiting.cancel()
            return
        self.log.warning(
            'no camera images yet: is the camera running?', topic=self.config.image_topic
        )

    def on_image(self, msg: Image):
        """Push a frame into the pipeline, at most fps per second."""
        now = time.monotonic()
        if now - self.last < 0.75 / self.config.fps:  # the camera's own rate passes despite jitter
            return
        if msg.encoding not in video.FORMATS:
            if not self.warned_encoding:
                self.warned_encoding = True
                self.log.error('cannot encode this image', encoding=msg.encoding,
                               use=','.join(video.FORMATS))  # fmt: skip
            return
        shape = (msg.encoding, msg.width, msg.height)
        if shape != self.shape and not self.start_pipeline(shape):
            return
        self.last = now
        self.frames += 1
        bytes_per_pixel = video.FORMATS[msg.encoding][1]
        data = video.pack_rows(
            msg.data,
            msg.height,
            msg.step,
            msg.width * bytes_per_pixel,
            video.stride(msg.width, bytes_per_pixel),
        )
        self.src.emit('push-buffer', Gst.Buffer.new_wrapped(data))

    def start_pipeline(self, shape: tuple) -> bool:
        """(Re)build the pipeline for this image encoding and size."""
        self.stop_pipeline()
        cfg = self.config.value
        encoding, width, height = shape
        out_width, out_height = video.output_size(width, height, cfg.max_width)
        description = video.pipeline_description(
            self.encoder, out_width, out_height, cfg.bitrate_kbps, self.keyframes, cfg.rtsp_url
        )
        try:
            pipeline = Gst.parse_launch(description)
        except GLib.Error as error:  # an element is not installed
            self.fail(f'video pipeline: {error.message} (scripts/setup.sh install webrtc)')
            return False
        src = pipeline.get_by_name('src')
        src.set_property('caps', Gst.Caps.from_string(video.raw_caps(encoding, width, height,
                                                                     cfg.fps)))  # fmt: skip
        src.set_property('is-live', True)
        src.set_property('do-timestamp', True)
        src.set_property('format', Gst.Format.TIME)
        set_optional(pipeline.get_by_name('enc'), self.encoder)
        if pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            pipeline.set_state(Gst.State.NULL)
            self.fail('video pipeline does not start')
            return False
        self.pipeline, self.src, self.shape = pipeline, src, shape
        self.log.info('video pipeline', size=f'{width}x{height}', encoding=encoding,
                      out=f'{out_width}x{out_height}', kbps=cfg.bitrate_kbps)  # fmt: skip
        return True

    def stop_pipeline(self):
        """Stop and drop the pipeline (closes the RTSP session)."""
        if self.pipeline is not None:
            self.pipeline.set_state(Gst.State.NULL)
        self.pipeline = self.src = self.shape = None

    def poll_bus(self):
        """Log pipeline warnings; stop the node on an error (MediaMTX starts it again)."""
        if self.pipeline is None:
            return
        bus = self.pipeline.get_bus()
        while self.running:
            message = bus.pop()  # every message: the bus would keep the others forever
            if message is None:
                return
            if message.type == Gst.MessageType.WARNING:
                self.log.warning('video pipeline', detail=message.parse_warning()[0].message)
            elif message.type == Gst.MessageType.ERROR:
                error, debug = message.parse_error()
                self.fail(f'video pipeline: {error.message} ({debug})')
            elif message.type == Gst.MessageType.EOS:
                self.fail('video pipeline ended')

    def fail(self, reason: str):
        """Log why and end the node with exit code 1."""
        self.log.error(reason)
        self.running = False
        self.exit_code = 1


def main(args=None):
    """Spin until MediaMTX stops the node (SIGINT) or the pipeline fails; exit 1 on failure."""
    compat.init(args=args)
    if Gst is None:
        sys.stderr.write('video_stream_node: no GStreamer for Python (scripts/setup.sh install '
                         'webrtc)\n')  # fmt: skip
        compat.try_shutdown()
        sys.exit(1)
    Gst.init(None)
    try:
        node = VideoStreamNode()
    except RuntimeError as error:
        sys.stderr.write(f'video_stream_node: {error}\n')
        compat.try_shutdown()
        sys.exit(1)
    try:
        while rclpy.ok() and node.running:
            rclpy.spin_once(node, timeout_sec=0.5)
    except (KeyboardInterrupt, compat.ExternalShutdownException):
        pass
    finally:
        node.stop_pipeline()
        node.destroy_node()
        compat.try_shutdown()
    sys.exit(node.exit_code)


if __name__ == '__main__':
    main()
