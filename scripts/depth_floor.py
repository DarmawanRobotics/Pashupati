#!/usr/bin/env python3
"""
Where the depth camera sees the floor: its mount's offset from its TF, measured.

Robot standing, about 3 m of free floor ahead, the stack running:

    source scripts/env.sh && python3 scripts/depth_floor.py [base_height] [topic]

Prints the heights the cloud shows above the nominal floor (robot.base_height below base_link,
0.35 by default) by distance, then the floor plane found by RANSAC among the planes within 35
deg of level (no window around the nominal floor, so it also finds a floor the perception fit
gives up on), and what it means: the floor's depth under base_link and the pitch and roll to
add to the camera's rpy in the URDF (Tuning Guide 5.3).
"""

from __future__ import annotations

import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener

from dharma_utils.ros.pointcloud import read_xyz

BASE_HEIGHT = float(sys.argv[1]) if len(sys.argv) > 1 else 0.35
TOPIC = sys.argv[2] if len(sys.argv) > 2 else '/camera/camera/depth/color/points'
BANDS = ((0.4, 0.6), (0.6, 0.8), (0.8, 1.2), (1.2, 1.6), (1.6, 2.0), (2.0, 2.5), (2.5, 3.0))


def rotation(q) -> np.ndarray:
    """Return the rotation matrix of a quaternion message."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def cloud_in_base(node: Node) -> tuple[np.ndarray, np.ndarray]:
    """Return one cloud in base_link and the camera origin there."""
    tf = Buffer()
    TransformListener(tf, node)
    clouds = []
    node.create_subscription(PointCloud2, TOPIC, clouds.append, qos_profile_sensor_data)
    end = time.time() + 8.0
    while time.time() < end and len(clouds) < 5:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not clouds:
        sys.exit(f'no cloud on {TOPIC} in 8 s')
    msg = clouds[-1]
    for _ in range(60):
        try:
            t = tf.lookup_transform('base_link', msg.header.frame_id, Time())
            break
        except Exception:  # the TF buffer fills during the first spins
            rclpy.spin_once(node, timeout_sec=0.1)
    else:
        sys.exit(f'no TF base_link <- {msg.header.frame_id}')
    o = t.transform.translation
    origin = np.array([o.x, o.y, o.z])
    xyz = read_xyz(msg).astype(float)
    return xyz @ rotation(t.transform.rotation).T + origin, origin


def floor_plane(points: np.ndarray) -> tuple[float, float, float, int]:
    """Return z = a x + b y + c of the floor (RANSAC, then least squares) and its inliers."""
    rng = np.random.default_rng(0)
    best, plane = 0, None
    for _ in range(500):
        s = points[rng.choice(len(points), 3, replace=False)]
        n = np.cross(s[1] - s[0], s[2] - s[0])
        if np.linalg.norm(n) < 1e-9:
            continue
        n = n / np.linalg.norm(n)
        n = n if n[2] > 0 else -n
        if n[2] < math.cos(math.radians(35.0)):
            continue
        inliers = int((np.abs((points - s[0]) @ n) < 0.02).sum())
        if inliers > best:
            best, plane = inliers, (n, s[0])
    if plane is None:
        sys.exit('no floor plane within 35 deg of level')
    n, p0 = plane
    keep = np.abs((points - p0) @ n) < 0.02
    design = np.column_stack((points[keep, 0], points[keep, 1], np.ones(int(keep.sum()))))
    (a, b, c), *_ = np.linalg.lstsq(design, points[keep, 2], rcond=None)
    return float(a), float(b), float(c), int(keep.sum())


def main():
    """Measure and print."""
    rclpy.init()
    node = Node('depth_floor')
    base, origin = cloud_in_base(node)
    height = base[:, 2] + BASE_HEIGHT
    distance = np.hypot(base[:, 0], base[:, 1])
    print(f'cloud: {len(base)} points; camera at {np.round(origin, 3)} in base_link')
    print('heights above the nominal floor (m), by distance from base_link:')
    print('  distance   points   lowest 5 %   median')
    for low, high in BANDS:
        m = (distance >= low) & (distance < high)
        if m.sum() < 20:
            print(f'  {low:.1f}-{high:.1f} m  {int(m.sum()):7d}')
            continue
        lowest, median = np.percentile(height[m], 5), np.median(height[m])
        print(f'  {low:.1f}-{high:.1f} m  {int(m.sum()):7d}   {lowest:+.3f}      {median:+.3f}')
    near = base[distance < 3.0]
    a, b, c, inliers = floor_plane(near)
    print(f'floor plane: {inliers} of {len(near)} points within 3 m')
    print(f'  z = {a:+.4f} x {b:+.4f} y {c:+.4f} in base_link')
    print(f'  floor {-c:.3f} m below base_link (robot.base_height {BASE_HEIGHT:.3f})')
    pitch, roll = math.degrees(math.atan(a)), -math.degrees(math.atan(b))
    print(f'  the mount looks off its TF by pitch {pitch:+.1f} deg, roll {roll:+.1f} deg')
    off = np.abs(near[:, 2] - (a * near[:, 0] + b * near[:, 1] + c)) > 0.10
    print(f'points more than 10 cm off that floor within 3 m: {int(off.sum())}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
