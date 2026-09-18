import math

import numpy as np
import tf_transformations as tft
from geometry_msgs.msg import TransformStamped


def transform_to_matrix(t: TransformStamped) -> np.ndarray:
    """Convert a TransformStamped into a 4x4 homogeneous transformation matrix."""
    trans = t.transform.translation
    rot = t.transform.rotation
    m = tft.quaternion_matrix([rot.x, rot.y, rot.z, rot.w])
    m[0, 3] = trans.x
    m[1, 3] = trans.y
    m[2, 3] = trans.z
    return m


def matrix_to_transform(m: np.ndarray, parent_frame: str, child_frame: str, stamp) -> TransformStamped:
    """Convert a 4x4 homogeneous transformation matrix into a TransformStamped."""
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = parent_frame
    t.child_frame_id = child_frame
    t.transform.translation.x = float(m[0, 3])
    t.transform.translation.y = float(m[1, 3])
    t.transform.translation.z = float(m[2, 3])
    q = tft.quaternion_from_matrix(m)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    return t


def pose_values_to_matrix(values) -> np.ndarray:
    """Convert a [x, y, z, qx, qy, qz, qw] list into a 4x4 homogeneous matrix."""
    x, y, z, qx, qy, qz, qw = values
    m = tft.quaternion_matrix([qx, qy, qz, qw])
    m[0, 3] = x
    m[1, 3] = y
    m[2, 3] = z
    return m


def flatten_to_yaw(m: np.ndarray) -> np.ndarray:
    """Project a pose onto the ground plane: keep x, y, and yaw, discard roll/pitch/z.
    Shared by both localization sources -- neither AprilTag's decoded rotation nor an
    RViz click's drag-to-set-heading should ever override FAST-LIO's own gravity-
    referenced roll/pitch, only x/y/yaw are meant to be corrected."""
    yaw = math.atan2(m[1, 0], m[0, 0])
    flattened = tft.euler_matrix(0.0, 0.0, yaw)
    flattened[0, 3] = m[0, 3]
    flattened[1, 3] = m[1, 3]
    flattened[2, 3] = 0.0
    return flattened


def map_odom_from_tag(
    m_map_tag: np.ndarray, m_base_cam: np.ndarray, m_cam_tag: np.ndarray, m_caminit_body: np.ndarray
) -> np.ndarray:
    """Compute map->odom so base_link lands on the tag's known map pose, flattened to x/y/yaw."""
    m_base_tag = m_base_cam @ m_cam_tag
    m_map_base = flatten_to_yaw(m_map_tag @ tft.inverse_matrix(m_base_tag))
    return m_map_base @ tft.inverse_matrix(m_caminit_body)


def map_odom_from_pose_estimate(pose_msg, m_caminit_body: np.ndarray) -> np.ndarray:
    """Compute map->odom so base_link lands on an RViz '2D Pose Estimate' click, flattened to x/y/yaw."""
    p = pose_msg.pose.pose.position
    q = pose_msg.pose.pose.orientation
    m_map_base = pose_values_to_matrix([p.x, p.y, p.z, q.x, q.y, q.z, q.w])
    return flatten_to_yaw(m_map_base) @ tft.inverse_matrix(m_caminit_body)