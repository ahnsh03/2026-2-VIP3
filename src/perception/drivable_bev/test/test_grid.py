#!/usr/bin/env python3

import unittest
from pathlib import Path

import numpy as np
import yaml

from drivable_bev.grid import BevGridSpec


class BevGridSpecTest(unittest.TestCase):
    def setUp(self):
        self.grid = BevGridSpec(-5.0, 20.0, -8.0, 8.0, 0.1)

    def test_default_shape_and_ego_anchor(self):
        self.assertEqual((250, 160), self.grid.shape)
        self.assertEqual((80.0, 200.0), self.grid.ego_pixel)

    def test_metric_pixel_round_trip(self):
        points = np.asarray(
            [[0.0, 0.0], [20.0, 8.0], [-5.0, -8.0], [17.25, -3.75]]
        )
        restored = self.grid.pixel_to_metric(self.grid.metric_to_pixel(points))
        np.testing.assert_allclose(restored, points, atol=1e-12)

    def test_forward_is_up_and_left_is_image_left(self):
        origin, forward, left = self.grid.metric_to_pixel(
            [[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]]
        )
        self.assertLess(forward[1], origin[1])
        self.assertLess(left[0], origin[0])

    def test_pixel_ground_homography_matches_conversion(self):
        pixels = np.asarray([[80.0, 200.0, 1.0], [50.0, 100.0, 1.0]])
        homogeneous = (self.grid.bev_pixel_to_ground_homography() @ pixels.T).T
        metric = homogeneous[:, :2] / homogeneous[:, 2:3]
        np.testing.assert_allclose(metric, self.grid.pixel_to_metric(pixels[:, :2]))

    def test_invalid_grid_is_rejected(self):
        with self.assertRaises(ValueError):
            BevGridSpec(0.0, 1.0, 0.0, 1.0, 0.3)
        with self.assertRaises(ValueError):
            BevGridSpec(1.0, 0.0, 0.0, 1.0, 0.1)

if __name__ == "__main__":
    unittest.main()
