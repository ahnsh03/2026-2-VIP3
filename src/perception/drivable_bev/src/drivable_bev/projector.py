"""Cached camera semantic projection onto an ego-centric grid."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np

from .calibration import CameraCalibration
from .grid import BevGridSpec
from .homography import HomographyModel, build_homography


@dataclass(frozen=True)
class ProjectedSemantic:
    drivable_probability: np.ndarray
    road_marking_class_id: np.ndarray
    road_marking_confidence: np.ndarray
    coverage: np.ndarray
    # binary(drivable + lane) 체크포인트에서만 채워진다. road_marking 모드에서는
    # None 이고, 그때는 class_id/confidence 쪽이 실제 내용이다.
    lane_probability: Optional[np.ndarray] = None


class CameraBevProjector:
    def __init__(
        self,
        camera: CameraCalibration,
        grid: BevGridSpec,
        # 주차 격자 기본값. config/bev_grid.yaml 의 max_ground_range_m 와 같은 값을
        # 둬서 런치가 파라미터를 빠뜨려도 커버리지가 조용히 넓어지지 않게 한다.
        max_ground_range_m: float = 25.0,
        min_camera_depth_m: float = 0.1,
        max_class_id: int = 3,
    ) -> None:
        self.camera = camera
        self.grid = grid
        # TwinLite v6 는 0=bg / 1=white / 2=yellow / 3=stopline 이다. 주차선/슬롯
        # 코너 같은 클래스가 추가되면 런치에서 이 값을 올려야 한다.
        self.max_class_id = int(max_class_id)
        if not 0 < self.max_class_id <= 255:
            raise ValueError("max_class_id must lie in [1,255]")
        self.homography: HomographyModel = build_homography(
            camera, grid, max_ground_range_m, min_camera_depth_m
        )

    @property
    def _size(self):
        return (self.grid.width_px, self.grid.height_px)

    def _warp(self, source: np.ndarray, interpolation: int) -> np.ndarray:
        return cv2.warpPerspective(
            source,
            self.homography.bev_from_image,
            self._size,
            flags=interpolation,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=0,
        )

    def project_lane(
        self,
        drivable_probability: np.ndarray,
        lane_probability: np.ndarray,
    ) -> ProjectedSemantic:
        """Project the binary head: two continuous mono8 probability layers.

        drivable / lane 둘 다 확률이므로 INTER_LINEAR 로 warp 한다. 범주형 id 가
        없으므로 class_id / confidence 는 0 으로 채워 fusion 의 argmax 를 무해하게
        통과시킨다.
        """
        drivable = self._validate_source(drivable_probability, "drivable")
        lane = self._validate_source(lane_probability, "lane probability")
        projected_drivable = self._warp(drivable, cv2.INTER_LINEAR)
        projected_lane = self._warp(lane, cv2.INTER_LINEAR)
        valid = self.homography.coverage > 0
        projected_drivable[~valid] = 0
        projected_lane[~valid] = 0
        zeros = np.zeros(self.grid.shape, dtype=np.uint8)
        return ProjectedSemantic(
            drivable_probability=np.ascontiguousarray(projected_drivable),
            road_marking_class_id=zeros,
            road_marking_confidence=zeros.copy(),
            coverage=self.homography.coverage.copy(),
            lane_probability=np.ascontiguousarray(projected_lane),
        )

    def project(
        self,
        drivable_probability: np.ndarray,
        road_marking_class_id: np.ndarray,
        road_marking_confidence: np.ndarray = None,
    ) -> ProjectedSemantic:
        drivable = self._validate_source(drivable_probability, "drivable")
        marking = self._validate_source(road_marking_class_id, "road marking")
        confidence = (
            np.full(marking.shape, 255, dtype=np.uint8)
            if road_marking_confidence is None
            else self._validate_source(road_marking_confidence, "road marking confidence")
        )
        if np.any(marking > self.max_class_id):
            raise ValueError(
                "road-marking class IDs must be in [0,{}]".format(self.max_class_id)
            )

        projected_drivable = self._warp(drivable, cv2.INTER_LINEAR)
        # class id 는 절대 보간하지 않는다.
        projected_marking = self._warp(marking, cv2.INTER_NEAREST)
        projected_confidence = self._warp(confidence, cv2.INTER_LINEAR)
        valid = self.homography.coverage > 0
        projected_drivable[~valid] = 0
        projected_marking[~valid] = 0
        projected_confidence[~valid] = 0
        return ProjectedSemantic(
            drivable_probability=np.ascontiguousarray(projected_drivable),
            road_marking_class_id=np.ascontiguousarray(projected_marking),
            road_marking_confidence=np.ascontiguousarray(projected_confidence),
            coverage=self.homography.coverage.copy(),
        )

    def _validate_source(self, value: np.ndarray, name: str) -> np.ndarray:
        array = np.asarray(value)
        expected = (self.camera.height, self.camera.width)
        if array.shape != expected:
            raise ValueError(
                "{} image must have shape {}, got {}".format(
                    name, expected, array.shape
                )
            )
        if array.dtype != np.uint8:
            raise ValueError("{} image must use uint8/mono8".format(name))
        return np.ascontiguousarray(array)
