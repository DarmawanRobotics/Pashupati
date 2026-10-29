"""The handheld's WebRTC video in the launch: MediaMTX when it can run here, else JPEG only."""

from __future__ import annotations

from pashupati import bringup, video
from pashupati import launch_utils as lu
from pashupati.stack import TELEOP_NODE, Options


def unavailable(enabled: str, binary: str):
    """Return (MediaMTX binary, version, video_stream_node, '') or why there is no video."""
    if not bringup.truthy(enabled):
        return None, None, '', 'WebRTC video off (WEBRTC=false): the handheld gets JPEG frames'
    found = video.find_mediamtx(binary)
    if found is None:
        reason = 'MediaMTX not found (scripts/setup.sh install webrtc, or MEDIAMTX)'
        return None, None, '', f'WebRTC video off: {reason}; the handheld gets JPEG frames'
    version = video.mediamtx_version(found)
    if version is None or version < video.MIN_VERSION:
        reason = f'{found} is {video.version_text(version)}, it needs v1.11 or newer'
        return None, None, '', f'WebRTC video off: {reason} (scripts/setup.sh install webrtc)'
    executable = lu.video_executable()
    missing = video.gstreamer_missing() if executable else 'video_stream_node is not built'
    if missing:
        return None, None, '', f'WebRTC video off: {missing} (scripts/setup.sh install webrtc)'
    return found, version, executable, ''


def video_server(enabled: str, binary: str, options: Options, data: dict):
    """
    Return (MediaMTX action or None, log line).

    MediaMTX reads the teleop token of the merged parameters and starts video_stream_node with
    them while the handheld watches; without it the handheld gets the JPEG frames.
    """
    found, version, executable, reason = unavailable(enabled, binary)
    if reason:
        return None, reason
    token = str(bringup.node_parameter(data, TELEOP_NODE, 'token', '') or '')
    command = video.node_command(
        executable, [options.params_file], video.rtsp_url(), options.camera_image
    )
    try:
        path = video.write_config(
            video.mediamtx_config(command, token, version), lu.MEDIAMTX_CONFIG
        )
    except OSError as error:
        return None, f'WebRTC video off: cannot write {lu.MEDIAMTX_CONFIG}: {error}'
    note = (
        f'WebRTC video: MediaMTX {video.version_text(version)} ({found}), WHEP on port '
        f'{video.WEBRTC_PORT}, media on UDP {video.ICE_PORT}'
    )
    return lu.mediamtx_action(path, found), note
