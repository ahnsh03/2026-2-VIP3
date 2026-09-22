"""Small NumPy-only geometry helpers used by the offline audit."""

from __future__ import annotations

import math
from typing import Iterable

import numpy as np


def points_xyz(raw: Iterable[Iterable[float]]) -> np.ndarray:
    arr = np.asarray(raw, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[1] < 2:
        return np.zeros((0, 3), dtype=np.float64)
    if arr.shape[1] == 2:
        arr = np.column_stack((arr, np.zeros(len(arr))))
    return arr[:, :3]


def resample_polyline(raw: Iterable[Iterable[float]], step_m: float) -> tuple[np.ndarray, np.ndarray]:
    """Sample a polyline at no more than ``step_m`` and return XY tangents."""

    if step_m <= 0:
        raise ValueError("step_m must be positive")
    points = points_xyz(raw)
    if len(points) < 2:
        return points, np.zeros((len(points), 2), dtype=np.float64)

    sampled: list[np.ndarray] = []
    tangents: list[np.ndarray] = []
    for start, end in zip(points[:-1], points[1:]):
        direction = end[:2] - start[:2]
        length = float(np.linalg.norm(direction))
        if length < 1e-9:
            continue
        tangent = direction / length
        count = max(1, int(math.ceil(length / step_m)))
        for index in range(count):
            sampled.append(start + (end - start) * (index / count))
            tangents.append(tangent)

    if not sampled:
        return points[:1], np.zeros((1, 2), dtype=np.float64)
    sampled.append(points[-1])
    tangents.append(tangents[-1])
    return np.asarray(sampled), np.asarray(tangents)


def cumulative_distance(points: np.ndarray) -> np.ndarray:
    if len(points) == 0:
        return np.zeros(0, dtype=np.float64)
    if len(points) == 1:
        return np.zeros(1, dtype=np.float64)
    segment = np.linalg.norm(np.diff(points[:, :2], axis=0), axis=1)
    return np.concatenate(([0.0], np.cumsum(segment)))


def offset_polyline(points: np.ndarray, tangents: np.ndarray, offset_m: float) -> np.ndarray:
    if len(points) != len(tangents):
        raise ValueError("points and tangents must have equal length")
    normal = np.column_stack((-tangents[:, 1], tangents[:, 0]))
    result = points.copy()
    result[:, :2] += float(offset_m) * normal
    return result


def quantile_or_none(values: np.ndarray, q: float) -> float | None:
    finite = np.asarray(values, dtype=np.float64)
    finite = finite[np.isfinite(finite)]
    return None if len(finite) == 0 else float(np.quantile(finite, q))
