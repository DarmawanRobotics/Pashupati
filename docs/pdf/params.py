#!/usr/bin/env python3
"""
The parameter reference of the guides, as HTML tables.

Every parameter in the packages' config files, read with their comments. Used by build.py (the
PDFs) and by the documentation site (docs/site), which runs it as a script:

    python3 docs/pdf/params.py   # the tables on stdout

Needs only PyYAML.
"""

from __future__ import annotations

import html
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[2]

# Config files of the parameter reference, in reading order: (package, file, what it covers)
PARAMETER_FILES = [
    ('pashupati', 'robot.yaml', 'robot profile: footprint, limits, base height'),
    ('pashupati', 'stack.yaml', 'the Dharma stack as tuned for Pashupati'),
    ('pashupati_bridge', 'bridge.yaml', 'handheld link, video, RTSP camera'),
    ('pashupati_inspection', 'inspection.yaml', 'VLM inspection, people'),
    ('pashupati_fleet', 'fleet.yaml', 'NETRA uplink'),
]

KEY_LINE = re.compile(r'^(?P<indent> *)(?P<key>[A-Za-z0-9_./*]+):\s*(?P<rest>.*)$')

# Meanings of avoidance parameters without a comment, by the last part of their name
AVOIDANCE = {
    'steering_gain': 'obstacle-to-steering strength',
    'velocity_gain': 'obstacle-to-slow-down strength',
    'repulsion_gain': 'how hard obstacles push the robot away',
    'smoothing': '0-1 output filter (higher = smoother/slower)',
    'steering_zone_center_deg': 'deg, centre of the left and right zones that steer',
    'steering_zone_width_deg': 'deg, width of those steering zones',
    'velocity_zone_center_deg': 'deg, centre of the left and right zones that slow down',
    'velocity_zone_width_deg': 'deg, width of those slow-down zones',
    'following_zone_width_deg': 'deg, frontal zone where obstacles slow the robot',
}
# Meanings of other parameters whose config line has no comment, by node and name
MEANINGS = {
    ('path_follower_node', 'auto_mode_off_service'): (
        'std_srvs/Trigger service called when navigation stops ("" = skip)'
    ),
    ('path_follower_node', 'footprint.rear'): 'm, base_link to the rear edge',
    ('path_follower_node', 'footprint.half_width'): 'm, base_link to the side edges',
    ('path_follower_node', 'pure_pursuit.lookahead_distance'): 'm, look-ahead point on the route',
    ('path_follower_node', 'pid.lookahead_distance'): 'm, look-ahead point on the route',
    ('path_follower_node', 'pid.kp'): 'proportional gain on the heading error',
    ('path_follower_node', 'pid.ki'): 'integral gain on the heading error',
    ('path_follower_node', 'pid.kd'): 'derivative gain on the heading error',
    ('path_follower_node', 'speed_regulator.kp'): 'proportional gain of the turn slow-down',
    ('path_follower_node', 'speed_regulator.ki'): 'integral gain of the turn slow-down',
    ('path_follower_node', 'speed_regulator.kd'): 'derivative gain of the turn slow-down',
    ('path_follower_node', 'speed_regulator.min_scale'): 'lowest speed factor in a sharp turn',
    ('teleop_udp_node', 'port'): 'UDP port of the handheld link and of robot discovery',
    ('teleop_udp_node', 'client_timeout_sec'): (
        's without messages before the handheld counts as gone (its commands stop)'
    ),
    ('teleop_udp_node', 'max_vy'): 'm/s, clamp on sideways teleop commands',
    ('teleop_udp_node', 'max_wz'): 'rad/s, clamp on teleop turning',
    ('teleop_udp_node', 'telemetry_hz'): 'state messages per second to the handheld',
    ('teleop_udp_node', 'routes_dir'): 'where the handheld lists and loads routes from',
    ('teleop_udp_node', 'camera_topic'): 'camera stream for the JPEG video',
    ('teleop_udp_node', 'camera_fps'): 'JPEG frames per second (when there is no WebRTC)',
    ('teleop_udp_node', 'camera_jpeg_quality'): 'JPEG quality of re-encoded frames (0-100)',
    ('teleop_udp_node', 'camera_max_width'): 'px, wider frames are scaled down',
    ('teleop_udp_node', 'video_path'): 'MediaMTX path of the WebRTC video',
    ('teleop_udp_node', 'tags_camera_info_topic'): 'camera model for the AprilTag outlines',
    ('teleop_udp_node', 'lidar_hz'): 'LiDAR frames per second for the handheld map',
    ('teleop_udp_node', 'lidar_voxel'): 'm, LiDAR points are thinned to one per cube this size',
    ('teleop_udp_node', 'lidar_max_points'): 'most LiDAR points per frame to the handheld',
    ('video_stream_node', 'fps'): 'video frames per second',
    ('video_stream_node', 'bitrate_kbps'): 'H.264 bitrate; lower it on weak Wi-Fi',
    ('lidar_sector_node', 'pointcloud_topic'): 'cloud used with input_type pointcloud2',
    ('lidar_sector_node', 'obstacle_z_max'): 'm above base_link; higher points are ignored',
    ('lidar_sector_node', 'self_filter_x_max'): 'm, front of the robot body box',
    ('lidar_sector_node', 'self_filter_y_min'): 'm, right side of the robot body box',
    ('lidar_sector_node', 'self_filter_y_max'): 'm, left side of the robot body box',
    ('person_detector_node', 'compressed'): 'true: image_topic carries JPEG (CompressedImage)',
    ('person_detector_node', 'camera_info_topic'): 'camera model, to place people on the floor',
    ('localization_node', 'camera_frame'): 'TF frame of the camera the tags are seen in',
    ('localization_node', 'tag_record_duration_sec'): 's of averaging per tag recording',
    ('localization_node', 'max_correction_deg'): 'deg, reject auto corrections that turn more',
    ('anomaly_detector_node', 'ollama_url'): 'address of the Ollama server',
    ('anomaly_detector_node', 'num_ctx'): (
        'tokens of context the model is loaded with: the image and the type list must fit'
    ),
    ('anomaly_detector_node', 'timeout_sec'): 's, longest wait for one answer of the model',
    ('anomaly_detector_node', 'max_image_age_sec'): 's, older camera frames are not inspected',
    ('odometry_gate', 'imu_topic'): 'IMU of the LiDAR, watched for stillness',
    ('odometry_gate', 'max_gyro_rad_s'): 'rad/s, more rotation counts as moving',
    ('odometry_monitor_node', 'odom_topic'): 'LiDAR odometry to watch',
    ('odometry_monitor_node', 'max_speed_diff_mps'): (
        'm/s, LiDAR and leg odometry speeds further apart are a mismatch'
    ),
    ('odometry_monitor_node', 'min_check_speed_mps'): 'm/s, slower: speeds are not compared',
    ('odometry_monitor_node', 'restart_origin_radius_m'): (
        'm, odometry jumping back this close to its origin counts as a restart'
    ),
    ('odometry_monitor_node', 'restart_min_offset_m'): (
        'm, ... when the robot was at least this far from the origin before'
    ),
    ('route_recorder_node', 'min_waypoint_spacing_m'): (
        'm, a new route point only after moving this far'
    ),
    ('route_recorder_node', 'record_rate_hz'): 'pose samples per second while recording',
    ('health_monitor_node', 'max_cpu_temp_c'): 'deg C, hotter is an issue',
    ('health_monitor_node', 'max_memory_percent'): '%, more memory in use is an issue',
    ('health_monitor_node', 'min_disk_free_gb'): 'GB, less free disk is an issue',
    ('health_monitor_node', 'disk_path'): 'the disk that is checked',
    ('health_monitor_node', 'publish_hz'): 'health/status messages per second',
    ('fleet_uplink_node', 'robot_id'): 'robot name in NETRA (ROBOT_ID of the env file wins)',
    ('fleet_uplink_node', 'verify_tls'): 'check the server certificate',
    ('fleet_uplink_node', 'telemetry_hz'): 'telemetry messages per second to NETRA',
    ('fleet_uplink_node', 'upload_period_sec'): 's between upload attempts of waiting files',
    ('fleet_uplink_node', 'routes_dir'): 'recorded routes, uploaded to NETRA',
    ('fleet_uplink_node', 'anomaly_topic'): 'inspection results to report',
    ('fleet_uplink_node', 'anomaly_min_confidence'): '0-1, less confident results are dropped',
}


def split_comment(text: str) -> tuple:
    """Return (value, comment) of the text after 'key:', minding # inside quotes."""
    quote = None
    for i, char in enumerate(text):
        if char in '"\'' and quote in (None, char):
            quote = None if quote else char
        elif char == '#' and quote is None and (i == 0 or text[i - 1] in ' \t'):
            return text[:i].strip(), text[i + 1 :].strip()
    return text.strip(), ''


def take_comment(stripped: str, indent: int, last, node, pending: str) -> str:
    """Add a comment line to the parameter above (a run-on) or to the pending description."""
    text = ' '.join(stripped.lstrip('#').split())
    if last is not None and indent > last['indent'] + 4 and last['description']:
        last['description'] += ' ' + text  # a comment running on from the line above
        return pending
    if node is None:
        return pending
    if pending and re.match(r'[a-z_]+:\s', text) and pending[-1] not in ':;,.':
        pending += ';'  # the next item of a list like "path_shift: …  reactive: …"
    return f'{pending} {text}'.strip()


def read_parameters(path: Path) -> dict:
    """Return {node: [(name, default, description)]} of a ROS 2 parameter file, comments kept."""
    nodes: dict = {}
    node, stack, pending, last = None, [], '', None
    lines = path.read_text().splitlines()
    for number, line in enumerate(lines):
        stripped = line.strip()
        if not stripped:
            pending = ''
            continue
        indent = len(line) - len(line.lstrip(' '))
        if stripped.startswith('#'):
            pending = take_comment(stripped, indent, last, node, pending)
            continue
        if stripped.startswith('- '):
            if last is not None and last['list']:
                last['items'].append(split_comment(stripped[2:])[0])
            continue
        match = KEY_LINE.match(line)
        if not match:
            continue
        key, (value, comment) = match['key'], split_comment(match['rest'])
        if indent == 0:
            node, stack, pending, last = key, [], '', None
            nodes.setdefault(node, [])
            continue
        if key == 'ros__parameters':
            continue
        while stack and stack[-1][0] >= indent:
            stack.pop()
        following = lines[number + 1].strip() if number + 1 < len(lines) else ''
        if not value and not following.startswith('- '):
            stack.append((indent, key))  # a nested mapping (e.g. detector:)
            continue
        name = '.'.join([k for _, k in stack] + [key])
        last = {
            'indent': indent,
            'description': comment or pending,
            'list': not value,
            'items': [],
            'name': name,
            'value': value,
        }
        nodes[node].append(last)
        pending = ''
    return {
        node: [
            (
                p['name'],
                p['value'] if not p['list'] else '[' + ', '.join(p['items']) + ']',
                p['description'],
            )
            for p in params
        ]
        for node, params in nodes.items()
    }


def meaning(node: str, param: str, comment: str) -> str:
    """Return what a parameter does: its config comment, or a known meaning."""
    if comment:
        return comment
    if (node, param) in MEANINGS:
        return MEANINGS[node, param]
    leaf = param.rsplit('.', 1)[-1]
    if node == 'obstacle_avoidance_node' and leaf in AVOIDANCE:
        return AVOIDANCE[leaf]
    if leaf == 'goal_tolerance':
        return 'm, the end of an open route counts as reached within this distance'
    if leaf.endswith('_frame'):
        return 'TF frame name'
    return ''


def breakable(text: str) -> str:
    """Escape text for HTML, with line break chances after . _ / and , (no characters added)."""
    return re.sub(r'([._/,])', r'\1<wbr>', html.escape(text))


def parameter_reference() -> str:
    """Return HTML tables of every parameter of the packages, grouped by package and node."""
    parts = []
    for package, name, what in PARAMETER_FILES:
        path = ROOT / 'src' / package / 'config' / name
        parts.append(
            f'<h2 id="ref-{package.replace("_", "-")}">{package}</h2>\n'
            f'<p class="source">{html.escape(what)} · <code>src/{package}/config/{name}</code></p>'
        )
        for node, params in read_parameters(path).items():
            rows = '\n'.join(
                f'<tr><td><code>{breakable(param)}</code></td>'
                f'<td><code>{breakable(default)}</code></td>'
                f'<td>{html.escape(meaning(node, param, description))}</td></tr>'
                for param, default, description in params
            )
            parts.append(
                f'<h3 class="node">{html.escape(node)}</h3>\n<table class="params">'
                '<thead><tr><th>Parameter</th><th>Default</th><th>Meaning</th></tr></thead>'
                f'<tbody>{rows}</tbody></table>'
            )
    return '\n\n'.join(parts)  # one raw HTML block each for Markdown


if __name__ == '__main__':
    print(parameter_reference())
