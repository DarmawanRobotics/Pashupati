"""Configuration of teleop_udp_node, video_stream_node and rtsp_camera_node."""

from __future__ import annotations

from dataclasses import dataclass

from dharma_utils.config import param
from pashupati_bridge.core.protocol import PORT

ROBOT_COMMANDS = ('stand_up', 'sit_down', 'emergency_stop')
ENCODERS = ('auto', 'nvv4l2h264enc', 'x264enc', 'openh264enc')
DECODERS = ('auto', 'jetson', 'software', 'ffmpeg')


@dataclass(frozen=True)
class TeleopConfig:
    """teleop_udp_node: the handheld's UDP gateway (frames and limits from robot.*)."""

    port: int = param(PORT, min=1, max=65535, live=False, doc='UDP port')
    token: str = param('', live=False, doc='shared secret, the same in the handheld app')
    robot_name: str = param('', live=False, doc='name in discovery beacons ("": hostname)')
    client_timeout_sec: float = param(
        2.0, min=0.2, max=60.0, unit='s', live=False, doc='a silent handheld is dropped'
    )
    max_vx: float = param(1.0, min=0.0, max=5.0, unit='m/s', live=False, doc='stick limit')
    max_vy: float = param(0.5, min=0.0, max=5.0, unit='m/s', live=False, doc='stick limit')
    max_wz: float = param(1.5, min=0.0, max=10.0, unit='rad/s', live=False, doc='stick limit')
    remote_source: str = param(
        'handheld', live=False, doc='control mux source: control/remote/<source>/cmd_vel'
    )
    telemetry_hz: float = param(10.0, min=0.5, max=50.0, unit='Hz', live=False, doc='state rate')
    robot_command_prefix: str = param(
        '/robot/', live=False, doc='driver Trigger services (the bringup sets /<robot_ns>/)'
    )
    robot_commands: tuple[str, ...] = param(
        ROBOT_COMMANDS, live=False, doc="the handheld's action menu, std_srvs/Trigger each"
    )
    sites_dir: str = param('~/dharma/map', live=False, doc='taught routes are listed from here')
    camera_topic: str = param(
        '/camera/camera/color/image_raw/compressed', live=False, doc='JPEG frames for the handheld'
    )
    camera_compressed: bool = param(True, live=False, doc='false: raw image, JPEG-encoded here')
    camera_fps: float = param(8.0, min=0.5, max=30.0, unit='Hz', live=False, doc='frame cap')
    camera_jpeg_quality: int = param(60, min=10, max=100, live=False, doc='re-encoded frames')
    camera_max_width: int = param(640, min=64, max=4096, live=False, doc='re-encoded frames')
    camera_max_bytes: int = param(
        60000, min=1000, max=10000000, live=False, doc='bigger driver JPEGs are re-encoded'
    )
    video_port: int = param(
        0, min=0, max=65535, live=False, doc='WebRTC (MediaMTX WHEP) port; 0: JPEG only'
    )
    video_path: str = param('camera', live=False, doc='MediaMTX path of the video')
    tags_topic: str = param(
        '/perception/apriltag/detections', live=False, doc='tags drawn on the video ("": off)'
    )
    tags_camera_info_topic: str = param(
        '/perception/apriltag/camera_info', live=False, doc='image size of the tag detector'
    )
    tags_max_age_sec: float = param(
        0.6, min=0.05, max=10.0, unit='s', live=False, doc='older tags are not drawn'
    )
    lidar_topic: str = param(
        '/cloud_registered_body', live=False, doc='cloud for the handheld ("": off)'
    )
    lidar_hz: float = param(2.0, min=0.1, max=20.0, unit='Hz', live=False, doc='cloud rate')
    lidar_voxel: float = param(0.1, min=0.01, max=2.0, unit='m', live=False, doc='downsampling')
    lidar_max_points: int = param(3000, min=10, max=100000, live=False, doc='points per cloud')
    health_diagnostics_topic: str = param(
        '/diagnostics', live=False, doc="health_monitor_node's findings (CPU, GPU, RAM)"
    )
    inspection_node: str = param(
        '/anomaly_detector_node', live=False, doc='its live parameters and status'
    )
    people_node: str = param('/person_detector_node', live=False, doc='its live on / off')
    navigation_node: str = param(
        '/path_follower_node', live=False, doc='its speed.cruise is the patrol speed'
    )
    patrol_speed_min: float = param(
        0.1, min=0.01, max=5.0, unit='m/s', live=False, doc='slowest patrol speed to set'
    )
    patrol_speed_max: float = param(
        0.6, min=0.01, max=5.0, unit='m/s', live=False, doc='fastest (and robot.limits.max_vx)'
    )

    def validate(self) -> list[str]:
        """Return cross-field problems."""
        if self.patrol_speed_min > self.patrol_speed_max:
            return ['patrol_speed_min must not exceed patrol_speed_max']
        return []


@dataclass(frozen=True)
class VideoConfig:
    """video_stream_node: H.264 of the camera for the handheld's WebRTC video."""

    image_topic: str = param(
        '/camera/camera/color/image_raw', live=False, doc='raw sensor_msgs/Image'
    )
    rtsp_url: str = param(
        'rtsp://127.0.0.1:8654/camera',
        live=False,
        doc='MediaMTX publish URL (the bringup sets it)',
    )
    fps: float = param(15.0, min=1.0, max=60.0, unit='Hz', live=False, doc='frames encoded')
    max_width: int = param(1280, min=0, max=8192, live=False, doc='wider images are scaled down')
    bitrate_kbps: int = param(1500, min=100, max=50000, live=False, doc='H.264 bitrate')
    keyframe_sec: float = param(
        1.0, min=0.1, max=10.0, unit='s', live=False, doc='a new viewer waits for a key frame'
    )
    encoder: str = param('auto', live=False, choices=ENCODERS, doc='auto: the first that works')


@dataclass(frozen=True)
class CameraConfig:
    """rtsp_camera_node: an RTSP stream as the robot's camera (camera:=rtsp)."""

    url: str = param('', live=False, doc='rtsp://host:port/path of the camera')
    codec: str = param('h264', live=False, choices=('h264', 'h265'), doc='stream codec')
    decoder: str = param(
        'auto', live=False, choices=DECODERS, doc='auto: Jetson, software, FFmpeg'
    )
    latency_ms: int = param(0, min=0, max=5000, live=False, doc='rtspsrc jitter buffer')
    protocols: str = param('tcp', live=False, choices=('tcp', 'udp'), doc='RTSP transport')
    width: int = param(0, min=0, max=8192, live=False, doc='0: the stream size')
    height: int = param(0, min=0, max=8192, live=False, doc='0: the stream size')
    max_fps: float = param(15.0, min=0.0, max=120.0, unit='Hz', live=False, doc='0: every frame')
    frame_id: str = param('camera_color_optical_frame', live=False, doc='optical frame')
    calibration_file: str = param('', live=False, doc='camera_calibration YAML: rectified')
    alpha: float = param(0.0, min=0.0, max=1.0, live=False, doc='0 crops, 1 keeps every pixel')
    default_hfov_deg: float = param(
        90.0, min=10.0, max=200.0, unit='deg', live=False, doc='camera_info without calibration'
    )
    publish_raw: bool = param(True, live=False, doc='image_raw (bgr8)')
    publish_compressed: bool = param(True, live=False, doc='image_raw/compressed (JPEG)')
    jpeg_quality: int = param(75, min=10, max=100, live=False, doc='compressed quality')
    compressed_fps: float = param(
        10.0, min=0.0, max=60.0, unit='Hz', live=False, doc='0: every frame'
    )
    latency_sec: float = param(
        0.1, min=0.0, max=5.0, unit='s', live=False, doc='stamp = receive time - this'
    )
    reconnect_sec: float = param(
        2.0, min=0.1, max=60.0, unit='s', live=False, doc='first retry, doubling to 30 s'
    )
