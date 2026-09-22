"""RViz 오버레이용 순수 기하 (ROS 비의존).

``SegmentLayer`` 가 시각화의 전체 기하 엔진이다. MGeo 폴리라인을 독립 선분
배열로 펼치고, ego 주변으로 크롭하면서 ``base_link`` 로 회전까지 해 준다.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable

import numpy as np


def lane_shape_group(feature: dict) -> str:
    """MGeo lane_shape 를 solid/broken 두 갈래로 정규화한다.

    주의: KATRI 에는 복합값 'Solid Solid'x14, 'Broken Solid'x2, 'Solid Broken'x2 가
    실제로 있다. 여기서는 'broken' 토큰이 하나라도 있으면 broken 으로 본다 —
    즉 이중선 18개는 한 겹으로만 그려진다. 이중선을 두 줄로 그리려면
    ``double_line_interval`` (KATRI 전부 0.1 m) 로 오프셋을 줘야 한다.
    """

    attributes = feature.get("attributes") or {}
    raw = attributes.get("lane_shape")
    if raw is None:
        raw = attributes.get("pattern")
    if raw is None:
        raw = (attributes.get("mgeo") or {}).get("lane_shape")
    values = raw if isinstance(raw, (list, tuple, set)) else [raw]
    lowered = {str(value).strip().lower() for value in values if value is not None}
    return "broken" if lowered.intersection({"broken", "dashed", "dash"}) else "solid"


def dashed_polyline_segments(
    polylines: Iterable[Iterable[Iterable[float]]],
    dash_length_m: float,
    gap_length_m: float,
    phase_m: float = 0.0,
) -> np.ndarray:
    """Convert continuous metric polylines to exact dashed line segments."""

    dash = float(dash_length_m)
    gap = float(gap_length_m)
    phase = float(phase_m)
    if not np.isfinite([dash, gap, phase]).all() or dash <= 0.0 or gap < 0.0:
        raise ValueError("dash length must be positive and gap length non-negative")
    cycle = dash + gap
    phase %= cycle
    output: list[np.ndarray] = []
    for raw in polylines:
        points = np.asarray(raw, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] < 2 or len(points) < 2:
            continue
        xy = points[:, :2]
        lengths = np.linalg.norm(np.diff(xy, axis=0), axis=1)
        cumulative = np.concatenate(([0.0], np.cumsum(lengths)))
        total = float(cumulative[-1])
        if total <= np.finfo(np.float64).eps:
            continue

        def interpolate(station: float) -> np.ndarray:
            if station >= total:
                return xy[-1].copy()
            index = min(int(np.searchsorted(cumulative, station, side="right") - 1), len(lengths) - 1)
            if lengths[index] <= np.finfo(np.float64).eps:
                return xy[index].copy()
            ratio = (station - cumulative[index]) / lengths[index]
            return xy[index] + ratio * (xy[index + 1] - xy[index])

        start = -phase
        while start + dash <= 0.0:
            start += cycle
        while start < total - np.finfo(np.float64).eps:
            visible_start = max(0.0, start)
            end = min(start + dash, total)
            if end <= visible_start + np.finfo(np.float64).eps:
                start += cycle
                continue
            cuts = [visible_start]
            cuts.extend(
                float(value) for value in cumulative[1:-1]
                if visible_start < value < end
            )
            cuts.append(end)
            vertices = [interpolate(value) for value in cuts]
            output.extend(
                np.stack((first, second))
                for first, second in zip(vertices[:-1], vertices[1:])
                if np.linalg.norm(second - first) > np.finfo(np.float64).eps
            )
            start += cycle
    if not output:
        return np.zeros((0, 2, 2), dtype=np.float64)
    return np.asarray(output, dtype=np.float64)


@dataclass(frozen=True)
class SegmentLayer:
    """A collection of independent XY line segments in the current map frame."""

    segments_xy: np.ndarray
    _midpoints_xy: np.ndarray = field(init=False, repr=False)
    _half_lengths_m: np.ndarray = field(init=False, repr=False)

    def __post_init__(self) -> None:
        value = np.asarray(self.segments_xy, dtype=np.float64)
        if value.ndim != 3 or value.shape[1:] != (2, 2):
            raise ValueError("segments_xy must have shape N x 2 x 2")
        if not np.isfinite(value).all():
            raise ValueError("segments_xy contains non-finite coordinates")
        object.__setattr__(self, "segments_xy", np.ascontiguousarray(value))
        if len(value) == 0:
            object.__setattr__(self, "_midpoints_xy", np.zeros((0, 2)))
            object.__setattr__(self, "_half_lengths_m", np.zeros(0))
        else:
            object.__setattr__(self, "_midpoints_xy", np.mean(value, axis=1))
            object.__setattr__(
                self,
                "_half_lengths_m",
                0.5 * np.linalg.norm(value[:, 1] - value[:, 0], axis=1),
            )

    def __len__(self) -> int:
        """Return the number of independent line segments in this layer."""

        return len(self.segments_xy)

    @classmethod
    def from_features(
        cls,
        features: Iterable[dict],
        translation_xy: Iterable[float] = (0.0, 0.0),
    ) -> "SegmentLayer":
        translation = np.asarray(tuple(translation_xy), dtype=np.float64)
        if translation.shape != (2,) or not np.isfinite(translation).all():
            raise ValueError("translation_xy must contain two finite values")
        segments = []
        for feature in features:
            points = np.asarray(feature.get("points", ()), dtype=np.float64)
            if points.ndim != 2 or points.shape[1] < 2 or len(points) < 2:
                continue
            xy = points[:, :2] + translation
            finite = np.isfinite(xy).all(axis=1)
            valid = finite[:-1] & finite[1:]
            if np.any(valid):
                segments.append(np.stack((xy[:-1][valid], xy[1:][valid]), axis=1))
        if not segments:
            return cls(np.zeros((0, 2, 2), dtype=np.float64))
        return cls(np.concatenate(segments, axis=0))

    def around_ego(
        self,
        ego_xy: Iterable[float],
        heading_deg: float,
        radius_m: float,
    ) -> np.ndarray:
        """Return nearby segments expressed in rear-axle ``base_link`` XY."""

        ego = np.asarray(tuple(ego_xy), dtype=np.float64)
        values = np.asarray([*ego, float(heading_deg), float(radius_m)])
        if ego.shape != (2,) or not np.isfinite(values).all() or radius_m <= 0.0:
            raise ValueError("ego pose and positive radius must be finite")
        if len(self.segments_xy) == 0:
            return self.segments_xy.copy()

        keep = (
            np.linalg.norm(self._midpoints_xy - ego, axis=1)
            <= radius_m + self._half_lengths_m
        )
        selected = self.segments_xy[keep]
        if not len(selected):
            return np.zeros((0, 2, 2), dtype=np.float64)

        heading = np.deg2rad(float(heading_deg))
        forward = np.asarray([np.cos(heading), np.sin(heading)])
        left = np.asarray([-np.sin(heading), np.cos(heading)])
        delta = selected - ego.reshape(1, 1, 2)
        return np.stack((delta @ forward, delta @ left), axis=2)

    def nearest_distance_m(self, point_xy: Iterable[float]) -> float:
        """Return the exact 2-D distance from a point to the closest segment."""

        point = np.asarray(tuple(point_xy), dtype=np.float64)
        if point.shape != (2,) or not np.isfinite(point).all():
            raise ValueError("point_xy must contain two finite values")
        if len(self.segments_xy) == 0:
            return float("inf")

        starts = self.segments_xy[:, 0]
        vectors = self.segments_xy[:, 1] - starts
        squared_lengths = np.einsum("ij,ij->i", vectors, vectors)
        factors = np.zeros(len(vectors), dtype=np.float64)
        nonzero = squared_lengths > np.finfo(np.float64).eps
        factors[nonzero] = (
            np.einsum("ij,ij->i", point - starts[nonzero], vectors[nonzero])
            / squared_lengths[nonzero]
        )
        factors = np.clip(factors, 0.0, 1.0)
        closest = starts + factors[:, None] * vectors
        return float(np.min(np.linalg.norm(closest - point, axis=1)))


# KATRI 실측 lane_type 별 dash 규격 (dash_interval_L1 / L2, 단위 m).
# ASMC 가 쓰던 하드코딩 3.0/3.0 을 그대로 쓰면 503 차선의 점선 간격이 틀린다.
LANE_TYPE_DASH_M = {
    503: (3.0, 5.0),    # 차선
    525: (0.75, 0.75),  # 교차로 유도선
}
# 정지선. 폭 0.6 m 로 유일하게 굵고, 별도 레이어/토픽으로 분리한다.
STOP_LINE_LANE_TYPE = 530
# lane_type 을 읽을 수 없을 때 쓰는 그룹 키.
UNKNOWN_LANE_TYPE = -1


def lane_type_group(feature: dict) -> int:
    """MGeo lane_boundary 의 lane_type 을 정수 하나로 정규화한다.

    lane_type 은 리스트다 (``[505]``). 한 폴리라인이 구간별로 다른 타입을 가질 수
    있고 (``lane_type_offset`` 과 병렬), KATRI 에는 그런 다중 구간이 1,579개 중
    18개뿐이라 **첫 값만** 쓴다. 나중에 '왜 저 선만 색이 이상하지'로 헤매지 않도록
    이 사실을 여기 적어 둔다.
    """

    raw = feature.get("lane_type")
    if raw is None:
        raw = (feature.get("attributes") or {}).get("lane_type")
    values = raw if isinstance(raw, (list, tuple)) else [raw]
    for value in values:
        try:
            return int(value)
        except (TypeError, ValueError):
            continue
    return UNKNOWN_LANE_TYPE


def grouped_lane_layers_by_type(
    features: Iterable[dict], translation_xy: Iterable[float] = (0.0, 0.0)
) -> dict[int, SegmentLayer]:
    """lane_type 정수별 SegmentLayer. 530(정지선)도 자기 키로 따로 나온다."""

    grouped: dict[int, list] = {}
    for feature in features:
        grouped.setdefault(lane_type_group(feature), []).append(feature)
    return {
        lane_type: SegmentLayer.from_features(values, translation_xy)
        for lane_type, values in sorted(grouped.items())
    }


def lane_color_group(feature: dict) -> str:
    """Return a stable visualization group for an MGeo lane boundary."""

    raw = feature.get("lane_color") or []
    values = {str(value).strip().lower() for value in raw}
    for known in ("yellow", "white", "blue"):
        if known in values:
            return known
    return "unknown"


def grouped_lane_layers(
    features: Iterable[dict], translation_xy: Iterable[float]
) -> dict[str, SegmentLayer]:
    grouped = {name: [] for name in ("white", "yellow", "blue", "unknown")}
    for feature in features:
        grouped[lane_color_group(feature)].append(feature)
    return {
        name: SegmentLayer.from_features(values, translation_xy)
        for name, values in grouped.items()
    }
