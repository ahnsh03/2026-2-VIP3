"""Coordinate-frame contracts for map-to-map transfer."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class LocalMapFrame:
    """A local ENU frame whose origin is expressed in a global metric CRS."""

    crs: str
    origin_xyz: tuple[float, float, float]

    @classmethod
    def from_global_info(cls, info: dict) -> "LocalMapFrame":
        origin = info.get("local_origin_in_global")
        if not isinstance(origin, list) or len(origin) < 2:
            raise ValueError("global_info.local_origin_in_global must have at least x/y")
        xyz = tuple(float(v) for v in (origin + [0.0, 0.0, 0.0])[:3])
        crs = str(info.get("global_coordinate_system") or "").strip()
        if not crs:
            raise ValueError("global_info.global_coordinate_system is required")
        return cls(crs=crs, origin_xyz=xyz)

    def local_to_global(self, points: Iterable[Iterable[float]]) -> np.ndarray:
        arr = _xyz_array(points)
        return arr + np.asarray(self.origin_xyz, dtype=np.float64)

    def global_to_local(self, points: Iterable[Iterable[float]]) -> np.ndarray:
        arr = _xyz_array(points)
        return arr - np.asarray(self.origin_xyz, dtype=np.float64)


def _xyz_array(points: Iterable[Iterable[float]]) -> np.ndarray:
    arr = np.asarray(points, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[1] < 2:
        raise ValueError("points must be an Nx2 or Nx3 array")
    if arr.shape[1] == 2:
        arr = np.column_stack((arr, np.zeros(len(arr), dtype=np.float64)))
    return arr[:, :3]


def _normalized_crs(crs: str) -> set[str]:
    """Compare semantically relevant UTM tokens, not PROJ spelling order."""

    tokens = {token.strip() for token in crs.split() if token.strip()}
    # WGS84 ``+ellps=WGS84`` and ``+datum=WGS84`` are equivalent for this
    # local translation audit.  Preserve all other tokens.
    if "+ellps=WGS84" in tokens or "+datum=WGS84" in tokens:
        tokens.discard("+ellps=WGS84")
        tokens.discard("+datum=WGS84")
        tokens.add("+wgs84")
    return tokens


def origin_translation(source: LocalMapFrame, target: LocalMapFrame) -> np.ndarray:
    """Return the translation that maps source-local XYZ to target-local XYZ.

    No ICP rotation or scale is fitted.  A transfer is allowed only when both
    maps declare the same global metric CRS.
    """

    if _normalized_crs(source.crs) != _normalized_crs(target.crs):
        raise ValueError(f"incompatible map CRS: {source.crs!r} vs {target.crs!r}")
    return np.asarray(source.origin_xyz) - np.asarray(target.origin_xyz)


def transform_points(
    points: Iterable[Iterable[float]],
    source: LocalMapFrame,
    target: LocalMapFrame,
) -> np.ndarray:
    return _xyz_array(points) + origin_translation(source, target)
