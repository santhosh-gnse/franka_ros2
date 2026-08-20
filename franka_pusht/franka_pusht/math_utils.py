"""Small quaternion/rigid-transform helpers using scalar-first quaternions."""

import math
import numpy as np


def normalize_quaternion(q):
    q = np.asarray(q, dtype=np.float64)
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1e-8:
        raise ValueError("invalid quaternion")
    q = q / norm
    return -q if q[0] < 0.0 else q


def quaternion_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ], dtype=np.float64)


def quaternion_inverse(q):
    q = normalize_quaternion(q)
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float64)


def quaternion_to_matrix(q):
    w, x, y, z = normalize_quaternion(q)
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ], dtype=np.float64)


def compose_pose(p_parent_child, q_parent_child, p_child_object, q_child_object):
    q_parent_child = normalize_quaternion(q_parent_child)
    p = np.asarray(p_parent_child) + quaternion_to_matrix(q_parent_child) @ np.asarray(p_child_object)
    q = normalize_quaternion(quaternion_multiply(q_parent_child, normalize_quaternion(q_child_object)))
    return p, q


def relative_pose(p_world_object, q_world_object, p_world_reference, q_world_reference):
    q_world_reference = normalize_quaternion(q_world_reference)
    rotation_reference_world = quaternion_to_matrix(q_world_reference).T
    p = rotation_reference_world @ (np.asarray(p_world_object) - np.asarray(p_world_reference))
    q = normalize_quaternion(quaternion_multiply(quaternion_inverse(q_world_reference), q_world_object))
    return p, q


def quaternion_to_rotation_vector(q, eps=1e-8):
    """Axis * angle (radians) for a scalar-first quaternion."""
    w, x, y, z = normalize_quaternion(q)
    w = min(1.0, max(-1.0, w))
    sin_half = math.sqrt(max(0.0, 1.0 - w * w))
    if sin_half < eps:
        return np.zeros(3, dtype=np.float64)
    return (2.0 * math.acos(w)) * np.array([x, y, z], dtype=np.float64) / sin_half


def orientation_error_vector(q_desired, q_current):
    """Rotation vector taking q_current to q_desired, in the parent frame.

    Right-invariant (spatial) error q_e = q_desired * q_current^-1, so the
    result is an angular velocity directly commandable in the parent frame.
    Port of pusht_mjx environment.py's _quat_error_body, which the sim's
    differential IK uses to hold the pusher's orientation fixed.
    """
    error = quaternion_multiply(normalize_quaternion(q_desired),
                               quaternion_inverse(q_current))
    if error[0] < 0.0:  # shortest arc
        error = -error
    return quaternion_to_rotation_vector(error)


def yaw_quaternion(yaw):
    return np.array([math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)])

