"""Small, ROS-independent helpers for map-local vehicle poses."""

from __future__ import annotations

import math
from typing import Iterable

import numpy as np


def valid_wgs84_position(latitude: float, longitude: float) -> bool:
    """Reject non-finite/out-of-range coordinates and MORAI's (0, 0) outage sentinel."""

    latitude = float(latitude)
    longitude = float(longitude)
    return (
        math.isfinite(latitude)
        and math.isfinite(longitude)
        and -90.0 <= latitude <= 90.0
        and -180.0 <= longitude <= 180.0
        and not (abs(latitude) <= 1e-9 and abs(longitude) <= 1e-9)
    )


def quaternion_yaw_rad(x: float, y: float, z: float, w: float) -> float:
    values = np.asarray([x, y, z, w], dtype=np.float64)
    norm = float(np.linalg.norm(values))
    if not np.isfinite(values).all() or norm <= 1e-12:
        raise ValueError("orientation quaternion must be finite and non-zero")
    x, y, z, w = (values / norm).tolist()
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def projected_gps_to_base_link(
    projected_xy: Iterable[float],
    map_origin_xy: Iterable[float],
    yaw_rad: float,
    lever_arm_xy: Iterable[float] = (0.35, 0.0),
) -> np.ndarray:
    """Convert projected GPS antenna position to rear-axle map coordinates."""

    projected = np.asarray(tuple(projected_xy), dtype=np.float64)
    origin = np.asarray(tuple(map_origin_xy), dtype=np.float64)
    lever = np.asarray(tuple(lever_arm_xy), dtype=np.float64)
    values = np.concatenate((projected, origin, lever, [float(yaw_rad)]))
    if projected.shape != (2,) or origin.shape != (2,) or lever.shape != (2,):
        raise ValueError("projected, origin, and lever-arm values must be XY pairs")
    if not np.isfinite(values).all():
        raise ValueError("pose conversion values must be finite")
    cosine, sine = math.cos(yaw_rad), math.sin(yaw_rad)
    rotated_lever = np.asarray([
        cosine * lever[0] - sine * lever[1],
        sine * lever[0] + cosine * lever[1],
    ])
    return projected - origin - rotated_lever
