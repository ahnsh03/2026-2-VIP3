"""Deterministic map-frame crops for planner-facing HD-map topics.

The helpers in this module do not import ROS.  Local products remain in the
global ``map`` frame: only their spatial extent changes.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Optional, Sequence, Tuple

import numpy as np


Bounds = Tuple[float, float, float, float]


@dataclass(frozen=True)
class OccupancyCrop:
    """A fixed-size, grid-aligned occupancy crop."""

    data: np.ndarray
    origin_x: float
    origin_y: float
    resolution: float

    @property
    def width(self) -> int:
        return int(self.data.shape[1])

    @property
    def height(self) -> int:
        return int(self.data.shape[0])


def geometry_bounds(point_sets: Iterable[Iterable[Iterable[float]]]) -> Optional[Bounds]:
    """Return the finite XY bounds of one or more point sequences."""

    finite_values = []
    for raw in point_sets:
        points = np.asarray(list(raw), dtype=np.float64)
        if points.size == 0:
            continue
        if points.ndim != 2 or points.shape[1] < 2:
            raise ValueError("geometry points must have shape N x >=2")
        xy = points[:, :2]
        finite_values.append(xy[np.isfinite(xy).all(axis=1)])
    finite_values = [value for value in finite_values if len(value)]
    if not finite_values:
        return None
    points = np.concatenate(finite_values, axis=0)
    minimum = np.min(points, axis=0)
    maximum = np.max(points, axis=0)
    return float(minimum[0]), float(minimum[1]), float(maximum[0]), float(maximum[1])


def crop_bounds(center_xy: Sequence[float], extent_m: float) -> Bounds:
    """Return an axis-aligned square centered on ``center_xy``."""

    center = np.asarray(tuple(center_xy), dtype=np.float64)
    extent = float(extent_m)
    if center.shape != (2,) or not np.isfinite(center).all():
        raise ValueError("crop center must contain two finite values")
    if not math.isfinite(extent) or extent <= 0.0:
        raise ValueError("crop extent must be positive and finite")
    half = 0.5 * extent
    return center[0] - half, center[1] - half, center[0] + half, center[1] + half


def bounds_intersect(first: Bounds, second: Bounds) -> bool:
    """Return true when two closed XY bounding boxes intersect."""

    return not (
        first[2] < second[0]
        or first[0] > second[2]
        or first[3] < second[1]
        or first[1] > second[3]
    )


def select_intersecting_bounds(
    item_bounds: Sequence[Optional[Bounds]], center_xy: Sequence[float], extent_m: float
) -> list[int]:
    """Select geometry indices whose bounds touch the requested local square."""

    window = crop_bounds(center_xy, extent_m)
    return [
        index
        for index, bounds in enumerate(item_bounds)
        if bounds is not None and bounds_intersect(bounds, window)
    ]


def crop_occupancy_grid(
    occupancy: np.ndarray,
    resolution: float,
    origin_xy: Sequence[float],
    center_xy: Sequence[float],
    extent_m: float,
    unknown_value: int = -1,
) -> OccupancyCrop:
    """Create a fixed-size, map-aligned crop and pad outside cells as unknown."""

    source = np.asarray(occupancy)
    if source.ndim != 2:
        raise ValueError("occupancy grid must be a two-dimensional array")
    resolution = float(resolution)
    if not math.isfinite(resolution) or resolution <= 0.0:
        raise ValueError("grid resolution must be positive and finite")
    origin = np.asarray(tuple(origin_xy), dtype=np.float64)
    center = np.asarray(tuple(center_xy), dtype=np.float64)
    if origin.shape != (2,) or center.shape != (2,):
        raise ValueError("grid origin and crop center must be XY pairs")
    if not np.isfinite(np.concatenate((origin, center))).all():
        raise ValueError("grid origin and crop center must be finite")
    extent = float(extent_m)
    if not math.isfinite(extent) or extent <= 0.0:
        raise ValueError("crop extent must be positive and finite")

    cells = max(1, int(math.ceil(extent / resolution - 1e-12)))
    lower_world = center - 0.5 * cells * resolution
    lower_cell = np.floor((lower_world - origin) / resolution + 1e-12).astype(np.int64)
    output_origin = origin + lower_cell.astype(np.float64) * resolution
    output = np.full((cells, cells), int(unknown_value), dtype=source.dtype)

    source_x0 = max(0, int(lower_cell[0]))
    source_y0 = max(0, int(lower_cell[1]))
    source_x1 = min(source.shape[1], int(lower_cell[0]) + cells)
    source_y1 = min(source.shape[0], int(lower_cell[1]) + cells)
    if source_x1 > source_x0 and source_y1 > source_y0:
        destination_x0 = source_x0 - int(lower_cell[0])
        destination_y0 = source_y0 - int(lower_cell[1])
        output[
            destination_y0:destination_y0 + (source_y1 - source_y0),
            destination_x0:destination_x0 + (source_x1 - source_x0),
        ] = source[source_y0:source_y1, source_x0:source_x1]

    return OccupancyCrop(
        data=output,
        origin_x=float(output_origin[0]),
        origin_y=float(output_origin[1]),
        resolution=resolution,
    )
