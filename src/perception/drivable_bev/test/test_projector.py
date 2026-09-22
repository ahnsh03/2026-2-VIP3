#!/usr/bin/env python3

import unittest
from pathlib import Path

import cv2
import numpy as np

from drivable_bev.calibration import load_calibration_snapshot
from drivable_bev.grid import BevGridSpec
from drivable_bev.projector import CameraBevProjector


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


class CameraBevProjectorTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        _, cameras = load_calibration_snapshot(
            PACKAGE_ROOT / "config" / "cameras_vip3_v1.yaml"
        )
        cls.camera = cameras["front"]
        cls.grid = BevGridSpec(-5.0, 20.0, -8.0, 8.0, 0.1)
        cls.projector = CameraBevProjector(cls.camera, cls.grid)

    def test_ground_image_bev_homographies_round_trip(self):
        points_ground = np.asarray([[5.0, 0.0], [10.0, 2.0], [19.0, -3.0]])
        points_h = np.column_stack([points_ground, np.ones(len(points_ground))])
        image_h = (self.projector.homography.image_from_ground @ points_h.T).T
        image = image_h[:, :2] / image_h[:, 2:3]
        image_cv = image.astype(np.float64).reshape(-1, 1, 2)
        bev = cv2.perspectiveTransform(
            image_cv, self.projector.homography.bev_from_image
        ).reshape(-1, 2)
        expected = self.grid.metric_to_pixel(points_ground)
        np.testing.assert_allclose(bev, expected, atol=1e-6)

    def test_variable_lane_widths_survive_image_to_bev_geometry(self):
        for width in (2.5, 3.0, 3.5, 4.0):
            for distance in (5.0, 10.0, 15.0, 20.0):
                left_y = 0.45 + 0.58 * width
                right_y = left_y - width
                ground = np.asarray([[distance, left_y], [distance, right_y]])
                pixels, depth = self.camera.project_ground(ground)
                self.assertTrue(np.all(depth > 0.0))
                bev = cv2.perspectiveTransform(
                    pixels.astype(np.float64).reshape(-1, 1, 2),
                    self.projector.homography.bev_from_image,
                ).reshape(-1, 2)
                recovered = self.grid.pixel_to_metric(bev)
                np.testing.assert_allclose(recovered, ground, atol=1e-6)
                self.assertAlmostEqual(width, recovered[0, 1] - recovered[1, 1], places=6)

    def test_projection_shape_types_classes_and_coverage(self):
        height, width = self.camera.height, self.camera.width
        drivable = np.tile(
            np.linspace(0, 255, width, dtype=np.uint8), (height, 1)
        )
        marking = np.zeros((height, width), dtype=np.uint8)
        marking[:, : width // 3] = 1
        marking[:, width // 3 : 2 * width // 3] = 2
        marking[:, 2 * width // 3 :] = 3
        confidence = np.full((height, width), 217, dtype=np.uint8)
        output = self.projector.project(drivable, marking, confidence)
        self.assertEqual(self.grid.shape, output.drivable_probability.shape)
        self.assertEqual(np.uint8, output.drivable_probability.dtype)
        self.assertEqual(self.grid.shape, output.road_marking_class_id.shape)
        self.assertEqual(self.grid.shape, output.road_marking_confidence.shape)
        self.assertTrue(set(np.unique(output.road_marking_class_id)).issubset({0, 1, 2, 3}))
        self.assertTrue(set(np.unique(output.coverage)).issubset({0, 255}))
        invalid = output.coverage == 0
        self.assertTrue(np.all(output.drivable_probability[invalid] == 0))
        self.assertTrue(np.all(output.road_marking_class_id[invalid] == 0))
        self.assertTrue(np.all(output.road_marking_confidence[invalid] == 0))

    def test_projection_is_deterministic(self):
        rng = np.random.default_rng(7)
        drivable = rng.integers(
            0, 256, (self.camera.height, self.camera.width), dtype=np.uint8
        )
        marking = rng.integers(
            0, 4, (self.camera.height, self.camera.width), dtype=np.uint8
        )
        confidence = rng.integers(
            0, 256, (self.camera.height, self.camera.width), dtype=np.uint8
        )
        first = self.projector.project(drivable, marking, confidence)
        second = self.projector.project(drivable, marking, confidence)
        np.testing.assert_array_equal(first.drivable_probability, second.drivable_probability)
        np.testing.assert_array_equal(first.road_marking_class_id, second.road_marking_class_id)
        np.testing.assert_array_equal(
            first.road_marking_confidence, second.road_marking_confidence
        )

    def test_invalid_input_is_rejected(self):
        valid = np.zeros((self.camera.height, self.camera.width), dtype=np.uint8)
        with self.assertRaises(ValueError):
            self.projector.project(valid[:, :-1], valid[:, :-1])
        invalid_classes = valid.copy()
        invalid_classes[0, 0] = 4
        with self.assertRaises(ValueError):
            self.projector.project(valid, invalid_classes)


if __name__ == "__main__":
    unittest.main()
