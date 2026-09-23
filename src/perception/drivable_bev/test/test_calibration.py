#!/usr/bin/env python3

import json
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

import numpy as np

from drivable_bev.calibration import (
    GroundPlane,
    calibrations_from_dict,
    load_calibration_snapshot,
    validate_sensor_set_snapshot,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PACKAGE_ROOT.parents[2]
SNAPSHOT = PACKAGE_ROOT / "config" / "cameras_vip3_v1.yaml"
SENSOR_SET = REPO_ROOT / "config" / "VIP3_sensor_set_v1_ros.json"


class CameraCalibrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.snapshot, cls.cameras = load_calibration_snapshot(SNAPSHOT)

    def test_snapshot_matches_checked_in_sensor_set(self):
        validate_sensor_set_snapshot(self.snapshot, self.cameras, SENSOR_SET)

    def test_ground_plane_is_explicit_and_shared(self):
        planes = {camera.ground_plane for camera in self.cameras.values()}
        self.assertEqual(1, len(planes))
        plane = planes.pop()
        self.assertEqual("base_link", plane.frame_id)
        self.assertEqual("plane", plane.model)
        self.assertEqual("provisional_asmc_carryover_20260923", plane.source)
        np.testing.assert_allclose(plane.coefficients, [0.0, 0.0, -0.35])

    def test_missing_or_invalid_ground_plane_is_rejected(self):
        missing = dict(self.snapshot)
        missing.pop("ground_plane")
        with self.assertRaisesRegex(ValueError, "ground_plane"):
            calibrations_from_dict(missing)
        invalid = dict(self.snapshot)
        invalid["ground_plane"] = dict(self.snapshot["ground_plane"])
        invalid["ground_plane"]["z_at_origin_m"] = float("nan")
        with self.assertRaisesRegex(ValueError, "finite"):
            calibrations_from_dict(invalid)

    def test_intrinsics_are_derived_from_fov(self):
        front = self.cameras["front"]
        left = self.cameras["left"]
        self.assertAlmostEqual(640.0, front.intrinsic[0, 0], places=6)
        self.assertAlmostEqual(640.0, front.intrinsic[1, 1], places=6)
        self.assertAlmostEqual(640.0, front.intrinsic[0, 2], places=6)
        self.assertAlmostEqual(360.0, front.intrinsic[1, 2], places=6)
        self.assertAlmostEqual(149.2184506, left.intrinsic[0, 0], places=5)

    def test_front_ground_center_projects_below_horizon(self):
        front = self.cameras["front"]
        pixels, depth = front.project_ground([[10.0, 0.0], [20.0, 0.0]])
        self.assertTrue(np.all(depth > 0.0))
        np.testing.assert_allclose(pixels[:, 0], [640.0, 640.0], atol=1e-6)
        self.assertTrue(np.all(pixels[:, 1] > 360.0))
        self.assertGreater(pixels[0, 1], pixels[1, 1])

    def test_side_camera_directions_are_not_mirrored(self):
        left_pixels, left_depth = self.cameras["left"].project_ground([[0.0, 5.0]])
        right_pixels, right_depth = self.cameras["right"].project_ground([[0.0, -5.0]])
        self.assertGreater(left_depth[0], 0.0)
        self.assertGreater(right_depth[0], 0.0)
        self.assertLess(left_pixels[0, 0], 0.5 * self.cameras["left"].width)
        self.assertGreater(right_pixels[0, 0], 0.5 * self.cameras["right"].width)
        self.assertAlmostEqual(
            self.cameras["left"].width,
            left_pixels[0, 0] + right_pixels[0, 0],
            places=6,
        )
        self.assertAlmostEqual(left_pixels[0, 1], right_pixels[0, 1], places=6)

    def test_continuous_ground_roundtrip_for_variable_asymmetric_lanes(self):
        widths = (2.5, 3.0, 3.5, 4.0)
        distances = (5.0, 10.0, 15.0, 20.0)
        for camera in self.cameras.values():
            inverse = np.linalg.inv(camera.ground_to_image_homography)
            for width in widths:
                # Deliberately asymmetric about y=0: no ego-centred lane premise.
                left_y = 0.37 + 0.61 * width
                right_y = left_y - width
                points = np.asarray(
                    [[distance, y] for distance in distances for y in (left_y, right_y)],
                    dtype=np.float64,
                )
                pixels, depth = camera.project_ground(points)
                visible = (
                    (depth > 0.0)
                    & (pixels[:, 0] >= 0.0)
                    & (pixels[:, 0] <= camera.width - 1.0)
                    & (pixels[:, 1] >= 0.0)
                    & (pixels[:, 1] <= camera.height - 1.0)
                )
                if not np.any(visible):
                    continue
                pixels_h = np.column_stack([pixels[visible], np.ones(np.sum(visible))])
                recovered_h = (inverse @ pixels_h.T).T
                recovered = recovered_h[:, :2] / recovered_h[:, 2:3]
                np.testing.assert_allclose(recovered, points[visible], atol=1e-6)

    def test_sloped_ground_plane_roundtrip(self):
        sloped = GroundPlane(
            frame_id="base_link",
            model="plane",
            z_at_origin_m=-0.35,
            dz_dx=0.012,
            dz_dy=-0.007,
            source="synthetic_regression",
        )
        for camera in self.cameras.values():
            calibrated = replace(camera, ground_plane=sloped)
            points = np.asarray([[5.0, -1.1], [9.0, 0.3], [15.0, 2.2]])
            pixels, depth = calibrated.project_ground(points)
            visible = depth > 0.0
            inverse = np.linalg.inv(calibrated.ground_to_image_homography)
            pixels_h = np.column_stack([pixels[visible], np.ones(np.sum(visible))])
            recovered_h = (inverse @ pixels_h.T).T
            recovered = recovered_h[:, :2] / recovered_h[:, 2:3]
            np.testing.assert_allclose(recovered, points[visible], atol=1e-6)

    def test_changed_sensor_set_is_rejected(self):
        """센서셋 내용이 바뀌면 기동을 막는다 (파일 이름은 같게 둬서 해시 경로를 탄다)."""
        document = json.loads(SENSOR_SET.read_text(encoding="utf-8"))
        document["cameraList"][0]["rot"]["pitch"] = "3.000"
        with tempfile.TemporaryDirectory() as temporary:
            changed = Path(temporary) / SENSOR_SET.name
            changed.write_text(json.dumps(document), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "SHA-256 mismatch"):
                validate_sensor_set_snapshot(self.snapshot, self.cameras, changed)

    def test_wrong_sensor_set_file_is_rejected_by_name(self):
        """런치가 다른 센서셋(예: SVM v2)을 가리키면 해시만으로는 원인을 알기 어렵다.

        센서셋을 바꾸면서 cameras_*.yaml 을 안 고치는 것이 가장 흔한 실수라
        파일 이름을 먼저 본다.
        """
        with tempfile.TemporaryDirectory() as temporary:
            other = Path(temporary) / "VIP3_sensor_set_v2_svm.json"
            other.write_text(SENSOR_SET.read_text(encoding="utf-8"), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "sensor-set file mismatch"):
                validate_sensor_set_snapshot(self.snapshot, self.cameras, other)

    def test_snapshot_without_a_name_skips_the_name_check(self):
        """source_sensor_set 이 없는 옛 스냅샷은 해시 검사만 한다 (하위 호환)."""
        snapshot = dict(self.snapshot)
        snapshot.pop("source_sensor_set", None)
        with tempfile.TemporaryDirectory() as temporary:
            other = Path(temporary) / "anything.json"
            other.write_text(SENSOR_SET.read_text(encoding="utf-8"), encoding="utf-8")
            validate_sensor_set_snapshot(snapshot, self.cameras, other)


if __name__ == "__main__":
    unittest.main()
