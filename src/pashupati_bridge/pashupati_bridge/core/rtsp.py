"""GStreamer pipelines for an RTSP camera (no ROS imports)."""

from __future__ import annotations

import math

DECODERS = ('jetson', 'software', 'ffmpeg')
DEPAY = {'h264': 'rtph264depay ! h264parse', 'h265': 'rtph265depay ! h265parse'}
SOFTWARE = {'h264': 'avdec_h264', 'h265': 'avdec_h265'}


def max_rate(max_fps: float) -> int:
    """Return videorate's max-rate (whole frames per second, at least 1) for a limit."""
    return max(1, math.ceil(max_fps))


def gstreamer_pipeline(
    url: str,
    codec: str = 'h264',
    decoder: str = 'jetson',
    latency_ms: int = 0,
    protocols: str = 'tcp',
    max_fps: float = 0.0,
    width: int = 0,
    height: int = 0,
) -> str:
    """Return the OpenCV GStreamer pipeline of an RTSP stream: BGR frames, newest only."""
    # videorate max-rate drops the frames beyond max_fps (a caps framerate does not
    # negotiate on an RTSP stream without a fixed rate, drop-only with max-rate asserts).
    # Scaling before the colour conversion (on the Jetson in hardware, nvvidconv) makes a
    # 1080p30 stream published at 5 fps cost a fraction of the CPU.
    codec = codec.lower()
    if codec not in DEPAY:
        raise ValueError(f'codec must be h264 or h265, not {codec!r}')
    size = f',width={int(width)},height={int(height)}' if width > 0 and height > 0 else ''
    rate = f'videorate max-rate={max_rate(max_fps)} ! ' if max_fps > 0.0 else ''
    if decoder == 'jetson':
        decode = f'nvv4l2decoder ! nvvidconv ! video/x-raw,format=BGRx{size} ! {rate}videoconvert'
    elif decoder == 'software':
        scale = f'videoscale ! video/x-raw{size} ! ' if size else ''
        decode = f'{SOFTWARE[codec]} ! {rate}{scale}videoconvert'
    else:
        raise ValueError(f'no GStreamer pipeline for decoder {decoder!r}')
    return (
        f'rtspsrc location={url} latency={int(latency_ms)} protocols={protocols} ! '
        f'{DEPAY[codec]} ! {decode} ! video/x-raw,format=BGR ! '
        'appsink drop=true max-buffers=1 sync=false'
    )


def decoder_candidates(decoder: str) -> list[str]:
    """Return the decoders to try in order: auto tries the Jetson, then software, then FFmpeg."""
    decoder = decoder.lower()
    if decoder == 'auto':
        return list(DECODERS)
    if decoder not in DECODERS:
        raise ValueError(f'decoder must be auto, {", ".join(DECODERS)}, not {decoder!r}')
    return [decoder]


def opencv_has_gstreamer(build_information: str) -> bool:
    """Return True when OpenCV's build information lists GStreamer support."""
    for line in build_information.splitlines():
        if line.strip().startswith('GStreamer:'):
            return 'YES' in line.upper()
    return False
