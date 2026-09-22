"""Deterministic grid index for dependency-light map audits."""

from __future__ import annotations

from collections import defaultdict
import math
from typing import Iterable, Sequence

import numpy as np


class GridIndex:
    def __init__(
        self,
        points: np.ndarray,
        tangents: np.ndarray | None = None,
        owners: Sequence[str] | None = None,
        cell_size_m: float = 4.0,
    ) -> None:
        self.points = np.asarray(points, dtype=np.float64)
        self.tangents = None if tangents is None else np.asarray(tangents, dtype=np.float64)
        self.owners = None if owners is None else np.asarray(owners, dtype=object)
        self.cell_size_m = float(cell_size_m)
        if self.points.ndim != 2 or self.points.shape[1] < 2:
            raise ValueError("points must be Nx2+")
        if self.tangents is not None and len(self.tangents) != len(self.points):
            raise ValueError("tangents length mismatch")
        if self.owners is not None and len(self.owners) != len(self.points):
            raise ValueError("owners length mismatch")
        self._cells: dict[tuple[int, int], list[int]] = defaultdict(list)
        for index, point in enumerate(self.points):
            self._cells[self._cell(point)].append(index)

    def _cell(self, point: Iterable[float]) -> tuple[int, int]:
        point = tuple(point)
        return (
            math.floor(point[0] / self.cell_size_m),
            math.floor(point[1] / self.cell_size_m),
        )

    def query(
        self,
        point: np.ndarray,
        tangent: np.ndarray | None = None,
        max_distance_m: float = 8.0,
        max_heading_deg: float = 30.0,
        undirected_heading: bool = True,
    ) -> tuple[float, float, int | None]:
        base_x, base_y = self._cell(point)
        radius = int(math.ceil(max_distance_m / self.cell_size_m))
        candidates: list[int] = []
        for x in range(base_x - radius, base_x + radius + 1):
            for y in range(base_y - radius, base_y + radius + 1):
                candidates.extend(self._cells.get((x, y), ()))
        if not candidates:
            return math.inf, math.inf, None

        indices = np.asarray(candidates, dtype=np.int64)
        distance = np.linalg.norm(self.points[indices, :2] - point[:2], axis=1)
        valid = distance <= max_distance_m
        heading = np.zeros(len(indices), dtype=np.float64)
        if tangent is not None and self.tangents is not None:
            dot = np.clip(self.tangents[indices] @ tangent, -1.0, 1.0)
            if undirected_heading:
                dot = np.abs(dot)
            heading = np.degrees(np.arccos(dot))
            valid &= heading <= max_heading_deg
        if not np.any(valid):
            return math.inf, math.inf, None
        local = np.where(valid)[0]
        best = int(local[np.argmin(distance[local])])
        return float(distance[best]), float(heading[best]), int(indices[best])
