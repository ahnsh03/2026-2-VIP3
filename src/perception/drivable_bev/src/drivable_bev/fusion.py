"""Deterministic quality-weighted fusion for projected camera semantics."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Optional

import numpy as np

from .grid import BevGridSpec
from .projector import CameraBevProjector, ProjectedSemantic


@dataclass(frozen=True)
class FusedSemantic:
    drivable_probability: np.ndarray
    road_marking_class_id: np.ndarray
    road_marking_confidence: np.ndarray
    coverage: np.ndarray
    source_count: np.ndarray
    source_view: np.ndarray
    # secondary_head=lane 일 때만 채워진다 (가중 평균). road_marking 모드에서는 None.
    lane_probability: Optional[np.ndarray] = None


def timestamp_span_ns(stamps_ns) -> int:
    values = [int(value) for value in stamps_ns]
    if not values or any(value <= 0 for value in values):
        raise ValueError("source timestamps must be positive")
    return max(values) - min(values)


def timestamps_within_slop(stamps_ns, max_skew_ns: int) -> bool:
    if int(max_skew_ns) < 0:
        raise ValueError("max timestamp skew cannot be negative")
    return timestamp_span_ns(stamps_ns) <= int(max_skew_ns)


def build_quality_maps(
    projectors: Mapping[str, CameraBevProjector],
    grid: BevGridSpec,
    edge_taper_fraction: float = 0.12,
    near_full_quality_radius_m: float = 6.0,
    max_quality_radius_m: float = 12.0,
    boundary_weight: float = 0.20,
    per_view_density_normalisation: bool = True,
) -> Mapping[str, np.ndarray]:
    """Build static overlap weights from image support and ground resolution.

    ASMC 대비 두 가지가 바뀌었다. 둘 다 후방 카메라와 측면 카메라를 살리기 위한
    변경이고, 주차에서는 이 둘이 없으면 융합 결과가 사실상 전방 카메라만 남는다.

    1) range prior 를 **등방(isotropic)** 으로 바꿨다.
       ASMC 는 forward_fade_start_m=15 / lateral_fade_start_m=6 /
       rear_fade_start_m=-4 로 "앞으로 달린다"는 전제를 그대로 가중치에 박아뒀다.
       특히 rear_fade 는 x < -4 m, 즉 **후방 카메라가 담당하는 영역 자체**를
       boundary_weight(0.20) 까지 깎는다. 측정값: 후방 뷰 평균 quality 가
       0.3854 -> 0.2583 (x0.670) 으로 떨어졌다. 전방은 x0.979.
       지금은 base_link 로부터의 거리 radius = hypot(x, y) 하나만 본다.
       near_full_quality_radius_m 안쪽은 방향과 무관하게 1.0,
       max_quality_radius_m 에서 boundary_weight 까지 선형으로 내려간다.
       -> 같은 반경이면 앞/뒤/옆이 동일한 가중치를 받는다.

    2) ground-sampling density 정규화를 **뷰별**로 한다.
       VIP3 front/rear 는 1280x720 FOV90 (focal 640 px), left/right 는
       640x480 FOV130 (focal 149 px) 이라 지면 샘플링 밀도가 ~18 배 차이난다.
       전체 뷰를 하나의 95 퍼센타일로 정규화하면 측면 카메라의 density_weight 가
       0.13 근처에 고정되어, 전방과 겹치는 셀의 95.8 % 를 전방이 가져간다.
       주차선을 실제로 보는 것은 측면 카메라이므로 이건 치명적이다.
       per_view_density_normalisation=True 면 각 뷰의 유효 밀도만으로 스케일을
       잡아, "그 카메라 기준 잘 보이는 영역"을 뜻하게 만든다.
       (전역 정규화로 되돌리려면 False. 뷰 간 절대 해상도 비교가 필요할 때만.)
    """
    if not projectors:
        raise ValueError("at least one projector is required")
    if not 0.0 < edge_taper_fraction < 0.5:
        raise ValueError("edge_taper_fraction must lie in (0,0.5)")
    if not 0.0 <= boundary_weight <= 1.0:
        raise ValueError("boundary_weight must lie in [0,1]")
    if near_full_quality_radius_m < 0.0 or max_quality_radius_m <= 0.0:
        raise ValueError("quality radii must be positive")

    epsilon = float(np.finfo(np.float32).eps)

    def density_scale_of(values: np.ndarray) -> float:
        positive = values[values > 0.0]
        if not positive.size:
            return epsilon
        return max(float(np.percentile(np.sqrt(positive), 95.0)), epsilon)

    if per_view_density_normalisation:
        density_scales = {
            str(view): density_scale_of(projector.homography.ground_sampling_density)
            for view, projector in projectors.items()
        }
    else:
        shared = density_scale_of(
            np.concatenate(
                [
                    projector.homography.ground_sampling_density.reshape(-1)
                    for projector in projectors.values()
                ]
            )
        )
        density_scales = {str(view): shared for view in projectors}

    rows, columns = np.indices(grid.shape, dtype=np.float64)
    ground = grid.pixel_to_metric(
        np.column_stack([columns.reshape(-1), rows.reshape(-1)])
    ).reshape((*grid.shape, 2))

    def fade(value, start, end):
        if end <= start:
            return np.ones_like(value, dtype=np.float64)
        unit = np.clip((end - value) / (end - start), 0.0, 1.0)
        return boundary_weight + (1.0 - boundary_weight) * unit

    radius = np.hypot(ground[:, :, 0], ground[:, :, 1])
    range_weight = fade(radius, near_full_quality_radius_m, max_quality_radius_m)

    result = {}
    for view, projector in projectors.items():
        model = projector.homography
        camera = projector.camera
        pixels = model.source_pixels
        u = pixels[:, :, 0]
        v = pixels[:, :, 1]
        edge_distance = np.minimum.reduce(
            [u, camera.width - 1.0 - u, v, camera.height - 1.0 - v]
        )
        edge_scale = edge_taper_fraction * min(camera.width, camera.height)
        edge_weight = np.clip(edge_distance / max(edge_scale, 1e-6), 0.0, 1.0)
        density_weight = np.clip(
            np.sqrt(model.ground_sampling_density) / density_scales[str(view)],
            0.05,
            1.0,
        )
        quality = edge_weight * density_weight * range_weight
        quality[model.coverage == 0] = 0.0
        result[str(view)] = np.ascontiguousarray(quality, dtype=np.float32)
    return result


class CameraBevFusion:
    """Fuse aligned per-view rasters without ever averaging categorical IDs."""

    def __init__(
        self,
        quality_by_view: Mapping[str, np.ndarray],
        view_ids: Optional[Mapping[str, int]] = None,
        max_class_id: int = 3,
    ) -> None:
        self.max_class_id = int(max_class_id)
        if not 0 < self.max_class_id <= 255:
            raise ValueError("max_class_id must lie in [1,255]")
        if not quality_by_view:
            raise ValueError("at least one quality map is required")
        self.views = tuple(str(view) for view in quality_by_view)
        self.quality_by_view = {
            str(view): self._quality(value, str(view))
            for view, value in quality_by_view.items()
        }
        shapes = {value.shape for value in self.quality_by_view.values()}
        if len(shapes) != 1:
            raise ValueError("all quality maps must have the same shape")
        self.shape = next(iter(shapes))
        configured_ids = view_ids or {view: index + 1 for index, view in enumerate(self.views)}
        if set(configured_ids) != set(self.views):
            raise ValueError("view_ids must cover every fusion view")
        ids = [int(configured_ids[view]) for view in self.views]
        if any(value <= 0 or value > 255 for value in ids) or len(set(ids)) != len(ids):
            raise ValueError("view IDs must be unique uint8 values greater than zero")
        self.view_ids = np.asarray(ids, dtype=np.uint8)

    def fuse(
        self,
        projected_by_view: Mapping[str, ProjectedSemantic],
        valid_by_view: Optional[Mapping[str, np.ndarray]] = None,
    ) -> FusedSemantic:
        if set(projected_by_view) != set(self.views):
            raise ValueError("projected views differ from configured fusion views")
        if valid_by_view is not None and set(valid_by_view) != set(self.views):
            raise ValueError("valid masks differ from configured fusion views")

        drivable = []
        marking = []
        confidence = []
        weights = []
        lane = []
        for view in self.views:
            value = projected_by_view[view]
            drivable.append(self._mono8(value.drivable_probability, "drivable"))
            if value.lane_probability is not None:
                lane.append(self._mono8(value.lane_probability, "lane probability"))
            classes = self._mono8(value.road_marking_class_id, "road marking")
            if np.any(classes > self.max_class_id):
                raise ValueError(
                    "road-marking class IDs must lie in [0,{}]".format(
                        self.max_class_id
                    )
                )
            marking.append(classes)
            confidence.append(
                self._mono8(value.road_marking_confidence, "road marking confidence")
            )
            weight = self.quality_by_view[view].copy()
            if valid_by_view is not None:
                valid = np.asarray(valid_by_view[view])
                if valid.shape != self.shape:
                    raise ValueError("valid mask shape differs from fusion grid")
                weight[valid <= 0] = 0.0
            weights.append(weight)

        drivable_stack = np.stack(drivable).astype(np.float32)
        marking_stack = np.stack(marking)
        confidence_stack = np.stack(confidence).astype(np.float32)
        weight_stack = np.stack(weights)
        weight_sum = weight_stack.sum(axis=0)
        valid_union = weight_sum > 0.0

        fused_drivable = np.zeros(self.shape, dtype=np.uint8)
        weighted_drivable = (drivable_stack * weight_stack).sum(axis=0)
        fused_drivable[valid_union] = np.rint(
            weighted_drivable[valid_union] / weight_sum[valid_union]
        ).astype(np.uint8)

        # lane 은 확률이므로 drivable 과 똑같이 가중 평균한다 (argmax 아님).
        # 모든 뷰가 lane 레이어를 줄 때만 융합한다 — 섞이면 조용히 틀린다.
        fused_lane = None
        if lane:
            if len(lane) != len(self.views):
                raise ValueError(
                    "lane probability must be present for every view or for none"
                )
            lane_stack = np.stack(lane).astype(np.float32)
            fused_lane = np.zeros(self.shape, dtype=np.uint8)
            weighted_lane = (lane_stack * weight_stack).sum(axis=0)
            fused_lane[valid_union] = np.rint(
                weighted_lane[valid_union] / weight_sum[valid_union]
            ).astype(np.uint8)
            fused_lane = np.ascontiguousarray(fused_lane)

        # Geometry owns the seam; confidence refines ties without allowing an
        # overconfident, poorly resolved edge pixel to dominate a central view.
        owner_score = weight_stack * (0.5 + 0.5 * confidence_stack / 255.0)
        owner = np.argmax(owner_score, axis=0)
        gather = owner[None, :, :]
        fused_marking = np.take_along_axis(marking_stack, gather, axis=0)[0]
        fused_confidence = np.take_along_axis(
            confidence_stack.astype(np.uint8), gather, axis=0
        )[0]
        source_view = self.view_ids[owner]
        fused_marking[~valid_union] = 0
        fused_confidence[~valid_union] = 0
        source_view[~valid_union] = 0

        source_count = np.count_nonzero(weight_stack > 0.0, axis=0).astype(np.uint8)
        coverage = (valid_union.astype(np.uint8) * 255)
        return FusedSemantic(
            drivable_probability=np.ascontiguousarray(fused_drivable),
            road_marking_class_id=np.ascontiguousarray(fused_marking),
            road_marking_confidence=np.ascontiguousarray(fused_confidence),
            coverage=np.ascontiguousarray(coverage),
            source_count=np.ascontiguousarray(source_count),
            source_view=np.ascontiguousarray(source_view),
            lane_probability=fused_lane,
        )

    def _mono8(self, value, name):
        array = np.asarray(value)
        if array.shape != self.shape or array.dtype != np.uint8:
            raise ValueError("{} must be uint8 {}".format(name, self.shape))
        return array

    @staticmethod
    def _quality(value, view):
        array = np.asarray(value, dtype=np.float32)
        if array.ndim != 2 or not np.isfinite(array).all() or np.any(array < 0.0):
            raise ValueError("{} quality map must be a finite non-negative raster".format(view))
        return np.ascontiguousarray(array)
