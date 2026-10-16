#!/usr/bin/env python3
"""
Build the PDF guides from their Markdown sources.

    pip install weasyprint markdown pyyaml       # macOS: brew install pango first
    python3 docs/pdf/build.py                    # every guide
    python3 docs/pdf/build.py manual             # one of: manual, tuning

Pages are A5, which reads well on a phone. Text is Barlow Semi Condensed (docs/pdf/fonts, SIL
Open Font License). A source starts with a YAML front matter (title, subtitle, version, date,
revisions) and may contain `{{ parameter_reference }}`, replaced by tables of every parameter in
the packages' config files, read with their comments.
"""

import html
from pathlib import Path
import re
import sys

import markdown
from weasyprint import CSS, HTML
from weasyprint.text.fonts import FontConfiguration
import yaml

DOCS = Path(__file__).resolve().parents[1]
ROOT = DOCS.parent
GUIDES = {
    'manual': (DOCS / 'manual' / 'user-manual.md', DOCS / 'Pashupati-User-Manual.pdf'),
    'tuning': (DOCS / 'tuning' / 'tuning-guide.md', DOCS / 'Pashupati-Tuning-Guide.pdf'),
}

# Config files of the parameter reference, in reading order: (package, file, what it covers)
PARAMETER_FILES = [
    ('robot_navigation', 'navigation_params.yaml', 'route following, avoidance, stops'),
    ('robot_bridge', 'bridge_params.yaml', 'control mux, protective field, handheld, video'),
    ('robot_perception', 'perception_params.yaml', 'LiDAR sectors, people, AprilTags'),
    ('robot_localization', 'localization_params.yaml', 'map frame from AprilTags'),
    ('robot_inspection', 'inspection_params.yaml', 'AI inspection'),
    ('robot_odometry', 'odometry_params.yaml', 'odometry start, monitor, route recording'),
    ('robot_health', 'health_params.yaml', 'health monitor'),
    ('robot_fleet', 'fleet_params.yaml', 'NETRA uplink'),
]

KEY_LINE = re.compile(r'^(?P<indent> *)(?P<key>[A-Za-z0-9_./]+):\s*(?P<rest>.*)$')

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
            return text[:i].strip(), text[i + 1:].strip()
    return text.strip(), ''


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
            text = ' '.join(stripped.lstrip('#').split())
            if last is not None and indent > last['indent'] + 4 and last['description']:
                last['description'] += ' ' + text  # a comment running on from the line above
            elif node is not None:
                if pending and re.match(r'[a-z_]+:\s', text) and pending[-1] not in ':;,.':
                    pending += ';'  # the next item of a list like "path_shift: …  reactive: …"
                pending = f'{pending} {text}'.strip()
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


def front_matter(text: str) -> tuple:
    """Return (metadata, body) of a source with a --- YAML --- header."""
    if not text.startswith('---\n'):
        return {}, text
    end = text.index('\n---\n', 4)
    return yaml.safe_load(text[4:end]) or {}, text[end + 5:]


def number_headings(body: str) -> tuple:
    """Give h1 (1) and h2 (1.1) headings their numbers; return the HTML and the contents."""
    chapter, section, entries = 0, 0, []

    def number(match):
        nonlocal chapter, section
        level, attrs, text = int(match[1]), match[2], match[3]
        if 'unnumbered' in attrs:
            entries.append((level, '', re.search(r'id="([^"]+)"', attrs)[1], text))
            return match[0]
        if level == 1:
            chapter, section = chapter + 1, 0
            label = f'{chapter}'
        else:
            section += 1
            label = f'{chapter}.{section}'
        anchor = re.search(r'id="([^"]+)"', attrs)[1]
        entries.append((level, label, anchor, text))
        return f'<h{level}{attrs}><span class="num">{label}</span> {text}</h{level}>'

    body = re.sub(r'<h([12])([^>]*)>(.*?)</h\1>', number, body)
    return body, entries


def figures(body: str) -> str:
    """Turn a paragraph holding only an image into a figure with its alt text as caption."""

    def figure(match):
        tag = match[1]
        alt = re.search(r'alt="([^"]*)"', tag)
        css = re.search(r'class="([^"]*)"', tag)
        caption = f'<figcaption>{alt[1]}</figcaption>' if alt and alt[1] else ''
        return f'<figure class="{css[1] if css else ""}">{tag}{caption}</figure>'

    return re.sub(r'<p>\s*(<img [^>]+>)\s*</p>', figure, body)


def build(name: str):
    """Render one guide to its PDF."""
    source, target = GUIDES[name]
    meta, text = front_matter(source.read_text())
    text = text.replace('{{ parameter_reference }}', parameter_reference())
    body = markdown.markdown(
        text,
        extensions=['tables', 'attr_list', 'admonition', 'toc', 'fenced_code', 'md_in_html',
                    'sane_lists'],
        extension_configs={'toc': {'permalink': False}},
    )
    body, entries = number_headings(figures(body))
    contents = '\n'.join(
        f'<li class="toc{level}"><a href="#{anchor}"><span class="num">{label}</span> '
        f'{re.sub("<[^>]+>", "", text)}</a></li>'
        for level, label, anchor, text in entries
    )
    revisions = '\n'.join(
        f'<tr><td>{html.escape(str(v))}</td><td>{html.escape(str(d))}</td>'
        f'<td>{html.escape(str(n))}</td></tr>'
        for v, d, n in meta.get('revisions', [])
    )
    title = html.escape(meta.get('title', name))
    page = f"""<!doctype html><html lang="en"><head><meta charset="utf-8"><title>{title}</title>
</head><body>
<section class="cover">
  <img class="logo" src="{(DOCS / 'logo.png').as_uri()}" alt="">
  <div class="brand">PASHUPATI</div>
  <h1 class="cover-title unnumbered">{title}</h1>
  <p class="cover-subtitle">{html.escape(meta.get('subtitle', ''))}</p>
  <p class="cover-version">Version {html.escape(str(meta.get('version', '')))} ·
     {html.escape(str(meta.get('date', '')))}</p>
  <p class="cover-for">{html.escape(meta.get('audience', ''))}</p>
</section>
<section class="front">
  <h2 class="front-title">Revision history</h2>
  <table class="revisions"><thead><tr><th>Version</th><th>Date</th><th>Changes</th></tr></thead>
  <tbody>{revisions}</tbody></table>
  <p class="front-note">{html.escape(meta.get('note', ''))}</p>
</section>
<section class="front contents">
  <h2 class="front-title">Contents</h2>
  <ul class="toc">{contents}</ul>
</section>
<main>{body}</main>
</body></html>"""
    css = (Path(__file__).parent / 'style.css').read_text().replace(
        '%HEADER%', meta.get('header', meta.get('title', ''))
    )
    fonts = FontConfiguration()  # loads the @font-face fonts of the stylesheet
    HTML(string=page, base_url=str(source.parent)).write_pdf(
        target,
        stylesheets=[CSS(string=css, base_url=str(Path(__file__).parent), font_config=fonts)],
        font_config=fonts,
    )
    print(f'{target.relative_to(ROOT)}: {len(entries)} sections')


if __name__ == '__main__':
    for guide in sys.argv[1:] or GUIDES:
        build(guide)
