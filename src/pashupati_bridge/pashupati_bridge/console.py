"""The handheld, camera and video nodes for the Dharma console (menuconfig of dharma_console)."""

from __future__ import annotations

from pashupati_bridge.core.config import CameraConfig, TeleopConfig, VideoConfig


def nodes() -> dict:
    """Return every node as launched with the configuration it declares."""
    configs = {
        'teleop_udp_node': [(TeleopConfig, '')],
        'video_stream_node': [(VideoConfig, '')],
        'rtsp_camera': [(CameraConfig, '')],
    }
    section = 'Handheld and camera'
    return {
        'package': 'pashupati_bridge',
        'defaults': ['config/bridge.yaml'],
        'nodes': [{'node': n, 'section': section, 'configs': c} for n, c in configs.items()],
    }
