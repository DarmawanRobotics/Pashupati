"""Configuration of fleet_uplink_node (frames from robot.*)."""

from __future__ import annotations

from dataclasses import dataclass

from dharma_utils.config import param


@dataclass(frozen=True)
class FleetConfig:
    """fleet_uplink_node: the robot side of the NETRA command center."""

    server_url: str = param(
        '', live=False, doc='NETRA, e.g. https://netra.example.com ("": spool)'
    )
    token: str = param('', live=False, doc='robot token issued by NETRA')
    robot_id: str = param('robot-01', live=False, doc='fleet id of this robot')
    verify_tls: bool = param(True, live=False, doc='check the server certificate')
    telemetry_hz: float = param(1.0, min=0.1, max=10.0, unit='Hz', live=False, doc='snapshots')
    upload_period_sec: float = param(
        5.0, min=0.5, max=600.0, unit='s', live=False, doc='spooled uploads are tried this often'
    )
    spool_dir: str = param(
        '~/.pashupati/spool', live=False, doc='anomalies and routes wait here until accepted'
    )
    sites_dir: str = param(
        '~/dharma/map', live=False, doc='patrol routes from NETRA go into the active site here'
    )
    crowd_cell_size: float = param(1.0, min=0.1, max=20.0, unit='m', live=False, doc='heatmap')
    crowd_window_sec: float = param(
        60.0, min=5.0, max=3600.0, unit='s', live=False, doc='one crowd sample per window'
    )
    anomaly_topic: str = param(
        '/anomaly_detector/result', live=False, doc='detector results (JSON)'
    )
    anomaly_image_topic: str = param(
        '/camera/camera/color/image_raw/compressed', live=False, doc='fallback evidence frame'
    )
    anomaly_evidence_topic: str = param(
        '/anomaly_detector/evidence', live=False, doc='the frame each result was found in'
    )
    anomaly_min_confidence: float = param(0.5, min=0.0, max=1.0, live=False, doc='lower: ignored')
    health_diagnostics_topic: str = param(
        '/diagnostics', live=False, doc="health_monitor_node's findings (CPU, GPU, RAM, rates)"
    )
