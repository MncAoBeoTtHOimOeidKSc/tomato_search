"""Geometry for the search prototype (metres; RPY degrees only for arm transforms)."""
import math
import numpy as np


def rigid(xyz, rpy_deg):
    x, y, z = np.deg2rad(rpy_deg)
    a, b, c = np.cos([x, y, z])
    d, e, f = np.sin([x, y, z])
    t = np.eye(4)
    t[:3, :3] = [[c*b, c*e*d-f*a, c*e*a+f*d],
                 [f*b, f*e*d+c*a, f*e*a-c*d], [-e, b*d, b*a]]
    t[:3, 3] = xyz
    return t


def checked_matrix(value):
    t = np.asarray(value, dtype=float)
    if (t.shape != (4, 4) or not np.isfinite(t).all() or
            not np.allclose(t[3], [0, 0, 0, 1]) or
            not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=.001) or
            not np.isclose(np.linalg.det(t[:3, :3]), 1., atol=.001)):
        raise ValueError('INVALID_RIGID_TRANSFORM')
    return t


def mean_region_point(depth_m, mask, intrinsics, minimum, maximum):
    """Average the 3D points of valid aligned depth pixels in one plant region."""
    valid = mask & np.isfinite(depth_m) & (depth_m >= minimum) & (depth_m <= maximum)
    ys, xs = np.nonzero(valid)
    if len(xs) < 30:
        return None
    z = depth_m[ys, xs].astype(float)
    return np.array([np.mean((xs - intrinsics['cx']) * z) / intrinsics['fx'],
                     np.mean((ys - intrinsics['cy']) * z) / intrinsics['fy'], np.mean(z)])


def plant_map(camera_point, arm_pose, handeye, mount, base_pose):
    x, y, yaw = base_pose
    p = rigid([x, y, 0.], [0., 0., math.degrees(yaw)]) @ mount @ rigid(
        arm_pose[:3], arm_pose[3:]) @ handeye @ np.r_[camera_point, 1.]
    if not np.isfinite(p).all():
        raise ValueError('INVALID_PLANT_POINT')
    return p[:3]


def approach(base_pose, plant, standoff):
    base = np.asarray(base_pose[:2], dtype=float)
    delta = np.asarray(plant[:2], dtype=float) - base
    distance = float(np.linalg.norm(delta))
    if not math.isfinite(distance) or not math.isfinite(standoff) or standoff <= 0:
        raise ValueError('INVALID_APPROACH_GEOMETRY')
    if distance <= standoff + .10:
        return None
    xy = base + delta * (1. - standoff / distance)
    return [float(xy[0]), float(xy[1]), math.atan2(delta[1], delta[0])]
