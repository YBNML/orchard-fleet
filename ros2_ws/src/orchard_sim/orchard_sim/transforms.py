"""동차변환 유틸 — 외부 의존 없이 SDF 포즈와 TF 를 다룬다."""
from __future__ import annotations

import math

import numpy as np


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def matrix_to_quat(R):
    """회전행렬 → (x, y, z, w). trace 부호에 따라 안정적인 분기를 쓴다."""
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def quat_to_matrix(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def make_tf(translation, R):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = translation
    return T


def tf_from_pose_str(s):
    """SDF `<pose>x y z roll pitch yaw</pose>` → 4×4."""
    v = [float(t) for t in s.split()]
    if len(v) == 3:
        v = v + [0.0, 0.0, 0.0]
    if len(v) != 6:
        raise ValueError(f"pose 는 3개 또는 6개 값이어야 합니다: {s!r}")
    return make_tf(np.array(v[:3]), rpy_to_matrix(*v[3:]))


def tf_from_pos_quat(p, q):
    """(x,y,z), (x,y,z,w) → 4×4."""
    return make_tf(np.asarray(p, float), quat_to_matrix(*q))


def invert(T):
    R = T[:3, :3]
    t = T[:3, 3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ t
    return out


def decompose(T):
    """4×4 → ((x,y,z), (qx,qy,qz,qw))."""
    return T[:3, 3].copy(), matrix_to_quat(T[:3, :3])


def yaw_of(T):
    return math.atan2(T[1, 0], T[0, 0])
