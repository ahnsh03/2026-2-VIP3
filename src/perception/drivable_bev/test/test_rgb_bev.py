#!/usr/bin/env python3

import unittest

import numpy as np

from drivable_bev.grid import BevGridSpec
from drivable_bev.rgb_bev import (
    StampedEgoPose,
    draw_metric_polyline,
    fuse_rgb_views,
    nearest_ego_pose,
    warp_rgb_to_bev,
)


class RgbBevTest(unittest.TestCase):
    def test_nearest_pose_respects_age_gate(self):
        poses = [
            StampedEgoPose(100, 1.0, 2.0, 3.0),
            StampedEgoPose(130, 4.0, 5.0, 6.0),
        ]
        pose, delta = nearest_ego_pose(poses, 122, 10)
        self.assertEqual(130, pose.stamp_ns)
        self.assertEqual(8, delta)
        pose, delta = nearest_ego_pose(poses, 200, 10)
        self.assertIsNone(pose)
        self.assertEqual(-70, delta)

    def test_weighted_rgb_fusion_and_uncovered_pixels(self):
        images = {
            "front": np.full((2, 2, 3), 20, dtype=np.uint8),
            "left": np.full((2, 2, 3), 100, dtype=np.uint8),
        }
        quality = {
            "front": np.asarray([[1.0, 0.0], [1.0, 0.0]], dtype=np.float32),
            "left": np.asarray([[1.0, 0.0], [0.0, 0.0]], dtype=np.float32),
        }
        fused, count = fuse_rgb_views(images, quality)
        self.assertEqual([60, 60, 60], fused[0, 0].tolist())
        self.assertEqual([0, 0, 0], fused[0, 1].tolist())
        self.assertEqual([20, 20, 20], fused[1, 0].tolist())
        np.testing.assert_array_equal(count, [[2, 0], [1, 0]])

    def test_identity_warp_is_deterministic(self):
        image = np.arange(48, dtype=np.uint8).reshape(4, 4, 3)
        first = warp_rgb_to_bev(image, np.eye(3), (4, 4))
        second = warp_rgb_to_bev(image, np.eye(3), (4, 4))
        np.testing.assert_array_equal(first, image)
        np.testing.assert_array_equal(first, second)

    def test_draw_metric_polyline_marks_only_the_line(self):
        grid = BevGridSpec(0.0, 10.0, -5.0, 5.0, 0.1)
        image = np.zeros((*grid.shape, 3), dtype=np.uint8)
        x = np.linspace(1.0, 9.0, 17)
        points = np.column_stack((x, np.zeros_like(x)))
        draw_metric_polyline(image, points, grid, (0, 255, 255), 2)
        self.assertGreater(int(np.count_nonzero(image)), 100)
        self.assertTrue(np.all(image[0, 0] == 0))


if __name__ == "__main__":
    unittest.main()
