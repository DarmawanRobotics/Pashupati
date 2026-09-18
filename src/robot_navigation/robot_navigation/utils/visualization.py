import math

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker


def lookahead_marker(frame_id: str, stamp, x: float, y: float) -> Marker:
    """Build a sphere marker at the controller's lookahead point."""
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = 'pure_pursuit'
    marker.id = 0
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.orientation.w = 1.0
    marker.scale.x = marker.scale.y = marker.scale.z = 0.2
    marker.color = ColorRGBA(r=0.1, g=1.0, b=0.1, a=0.9)
    return marker


def nearest_point_marker(frame_id: str, stamp, x: float, y: float) -> Marker:
    """Build a small sphere marker at the nearest path point to the robot."""
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = 'pure_pursuit'
    marker.id = 1
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.orientation.w = 1.0
    marker.scale.x = marker.scale.y = marker.scale.z = 0.12
    marker.color = ColorRGBA(r=1.0, g=0.85, b=0.1, a=0.9)
    return marker


def curvature_arc_marker(frame_id: str, stamp, curvature: float, arc_length: float, num_points: int = 20) -> Marker:
    """Build a line strip tracing the arc the robot will follow at the current curvature."""
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = 'pure_pursuit'
    marker.id = 2
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.03
    marker.color = ColorRGBA(r=0.2, g=0.6, b=1.0, a=0.9)

    for i in range(num_points + 1):
        s = arc_length * i / num_points
        if abs(curvature) < 1e-4:
            x, y = s, 0.0
        else:
            x = math.sin(curvature * s) / curvature
            y = (1.0 - math.cos(curvature * s)) / curvature
        marker.points.append(Point(x=x, y=y, z=0.0))
    return marker


def rollout_marker(frame_id: str, stamp, points_xy) -> Marker:
    """Build a line strip showing a predicted trajectory rollout (e.g. MPPI's best sample)."""
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = 'pure_pursuit'
    marker.id = 4
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.03
    marker.color = ColorRGBA(r=1.0, g=0.4, b=0.9, a=0.8)
    for x, y in points_xy:
        marker.points.append(Point(x=float(x), y=float(y), z=0.0))
    return marker


def status_text_marker(frame_id: str, stamp, x: float, y: float, text: str, color: ColorRGBA) -> Marker:
    """Build a text marker showing follower status/debug info above the given position."""
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = 'pure_pursuit'
    marker.id = 3
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0.6
    marker.pose.orientation.w = 1.0
    marker.scale.z = 0.25
    marker.color = color
    marker.text = text
    return marker


def status_color(status: str) -> ColorRGBA:
    """Map a follower status keyword to a marker color for quick visual triage."""
    colors = {
        'FOLLOWING': ColorRGBA(r=0.2, g=1.0, b=0.2, a=1.0),
        'GOAL_REACHED': ColorRGBA(r=0.3, g=0.6, b=1.0, a=1.0),
        'EMERGENCY_STOP': ColorRGBA(r=1.0, g=0.2, b=0.2, a=1.0),
        'NO_PATH': ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0),
        'TF_UNAVAILABLE': ColorRGBA(r=1.0, g=0.2, b=0.2, a=1.0),
        'NAV_INACTIVE': ColorRGBA(r=0.6, g=0.6, b=0.6, a=1.0),
    }
    return colors.get(status, ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0))
