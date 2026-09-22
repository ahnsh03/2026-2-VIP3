#!/usr/bin/env python3
"""후방 카메라(yaw 180)가 BEV 기하에서 실제로 동작하는지 지키는 회귀 테스트.

ASMC drivable_bev 는 전방 3대로만 돌았다. VIP3 는 후방 카메라를 추가했고, 후진 주차의
핵심 뷰다. 이 파일은 "후방이 조용히 빈 BEV 를 내보내는" 회귀를 잡는다.

여기서 깨지면 의심할 곳은 세 군데다.
  1. cameras_vip3_v1.yaml 의 rear rotation_deg yaw 값 (180 이어야 한다)
  2. ground_plane.z_at_origin_m 부호
  3. MORAI pitch 부호 규약 (양수 = 렌즈 아래)
"""

from __future__ import annotations

import math
import unittest
from pathlib import Path

import numpy as np

from drivable_bev.calibration import load_calibration_snapshot
from drivable_bev.fusion import build_quality_maps
from drivable_bev.grid import BevGridSpec
from drivable_bev.homography import build_homography
from drivable_bev.projector import CameraBevProjector

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SNAPSHOT = PACKAGE_ROOT / "config" / "cameras_vip3_v1.yaml"
GRID = BevGridSpec(-10.0, 10.0, -8.0, 8.0, 0.05)


class RearCameraGeometryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        _, cls.cameras = load_calibration_snapshot(SNAPSHOT)

    def test_all_four_views_are_configured(self):
        self.assertEqual(
            {"front", "left", "right", "rear"}, set(self.cameras)
        )

    def test_rear_camera_faces_backwards(self):
        rear = self.cameras["rear"]
        self.assertAlmostEqual(180.0, abs(rear.rotation_deg[2]), places=6)
        # 광축(+z_optical)이 body -x 를 향해야 한다.
        forward_in_body = rear.optical_from_body_rotation.T @ np.asarray(
            [0.0, 0.0, 1.0]
        )
        self.assertLess(forward_in_body[0], -0.9)

    def test_rear_homography_is_not_singular(self):
        model = build_homography(self.cameras["rear"], GRID, 25.0, 0.10)
        determinant = float(np.linalg.det(model.image_from_bev))
        self.assertGreater(abs(determinant), 1e-6)
        self.assertTrue(np.isfinite(model.bev_from_image).all())

    def test_rear_camera_covers_the_area_behind_the_vehicle(self):
        model = build_homography(self.cameras["rear"], GRID, 25.0, 0.10)
        coverage = model.coverage > 0
        self.assertGreater(coverage.mean(), 0.15, "rear BEV coverage collapsed")

        # 커버된 셀은 전부 차량 뒤쪽(x < 0)이어야 한다.
        rows, columns = np.indices(GRID.shape, dtype=float)
        x = GRID.x_max_m - rows * GRID.resolution_m
        self.assertLess(float(x[coverage].max()), 0.0)

    def test_front_and_rear_do_not_overlap(self):
        front = build_homography(self.cameras["front"], GRID, 25.0, 0.10)
        rear = build_homography(self.cameras["rear"], GRID, 25.0, 0.10)
        both = (front.coverage > 0) & (rear.coverage > 0)
        self.assertEqual(0, int(both.sum()))

    def test_four_view_union_covers_most_of_the_parking_grid(self):
        stack = [
            build_homography(self.cameras[view], GRID, 25.0, 0.10).coverage > 0
            for view in ("front", "left", "right", "rear")
        ]
        union = np.logical_or.reduce(stack)
        self.assertGreater(union.mean(), 0.80)

    def test_near_field_blind_disc_is_documented(self):
        """차 바로 주변이 사각인 것은 버그가 아니라 임시 센서셋의 성질이다.

        docs/sensors.md §4 가 이 수치의 근거다. 센서셋을 개선하면 이 테스트의
        상한을 올리고 문서도 같이 고친다.
        """
        stack = [
            build_homography(self.cameras[view], GRID, 25.0, 0.10).coverage > 0
            for view in ("front", "left", "right", "rear")
        ]
        union = np.logical_or.reduce(stack)
        rows, columns = np.indices(GRID.shape, dtype=float)
        x = GRID.x_max_m - rows * GRID.resolution_m
        y = GRID.y_max_m - columns * GRID.resolution_m
        near = np.hypot(x, y) < 2.0
        self.assertLess(
            union[near].mean(),
            0.30,
            "근거리 가시율이 올라갔다면 센서셋이 바뀐 것이다. docs/sensors.md 를 갱신하라",
        )

    def test_rear_view_is_not_penalised_by_the_fusion_prior(self):
        """ASMC 의 rear_fade 는 후방 카메라 영역 자체를 깎았다. 등방 prior 로 바꿨다."""
        projectors = {
            view: CameraBevProjector(self.cameras[view], GRID, 25.0, 0.10)
            for view in ("front", "left", "right", "rear")
        }
        quality = build_quality_maps(projectors, GRID)
        means = {}
        for view, value in quality.items():
            covered = projectors[view].homography.coverage > 0
            means[view] = float(value[covered].mean())
        # 후방이 전방 대비 크게 불리하면 안 된다.
        self.assertGreater(means["rear"], 0.5 * means["front"])

    def test_rear_projection_round_trips_a_synthetic_mask(self):
        camera = self.cameras["rear"]
        projector = CameraBevProjector(camera, GRID, 25.0, 0.10)
        drivable = np.full((camera.height, camera.width), 200, dtype=np.uint8)
        lane = np.zeros((camera.height, camera.width), dtype=np.uint8)
        lane[camera.height // 2 :, :] = 255
        projected = projector.project_lane(drivable, lane)
        self.assertEqual(GRID.shape, projected.drivable_probability.shape)
        self.assertIsNotNone(projected.lane_probability)
        self.assertGreater(int(np.count_nonzero(projected.drivable_probability)), 0)
        self.assertGreater(int(np.count_nonzero(projected.lane_probability)), 0)

    def test_intrinsics_ignore_the_wrong_sensor_set_focal_length(self):
        """센서셋 JSON 의 focalLengthpixel=320 은 네 카메라 모두 틀렸다."""
        front = self.cameras["front"].intrinsic
        left = self.cameras["left"].intrinsic
        self.assertAlmostEqual(640.0, float(front[0, 0]), places=3)
        self.assertAlmostEqual(
            (0.5 * 640) / math.tan(math.radians(65.0)), float(left[0, 0]), places=3
        )


if __name__ == "__main__":
    unittest.main()
