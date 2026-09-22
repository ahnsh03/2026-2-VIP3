import unittest
from pathlib import Path

import yaml

import numpy as np

from vip3_hd_map.local_crop import (
    bounds_intersect,
    crop_bounds,
    crop_occupancy_grid,
    geometry_bounds,
    select_intersecting_bounds,
)


class LocalCropTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.repo_root = Path(__file__).resolve().parents[3]

    def test_geometry_bounds_ignores_non_finite_points(self):
        bounds = geometry_bounds((
            [[1.0, 2.0], [3.0, 4.0]],
            [[float("nan"), 9.0], [-2.0, 5.0]],
        ))
        self.assertEqual(bounds, (-2.0, 2.0, 3.0, 5.0))
        self.assertIsNone(geometry_bounds(([],)))

    def test_lane_selection_uses_closed_map_frame_crop(self):
        window = crop_bounds((0.0, 0.0), 20.0)
        self.assertEqual(window, (-10.0, -10.0, 10.0, 10.0))
        self.assertTrue(bounds_intersect((-12.0, -1.0, -10.0, 1.0), window))
        self.assertFalse(bounds_intersect((10.1, -1.0, 11.0, 1.0), window))
        selected = select_intersecting_bounds(
            [(-12.0, -1.0, -10.0, 1.0), None, (10.1, -1.0, 11.0, 1.0)],
            (0.0, 0.0),
            20.0,
        )
        self.assertEqual(selected, [0])

    def test_occupancy_crop_preserves_source_cells_and_alignment(self):
        source = np.arange(20, dtype=np.int8).reshape(4, 5)
        result = crop_occupancy_grid(
            source, resolution=1.0, origin_xy=(10.0, 20.0),
            center_xy=(12.0, 22.0), extent_m=4.0,
        )
        self.assertEqual((result.height, result.width), (4, 4))
        self.assertEqual((result.origin_x, result.origin_y), (10.0, 20.0))
        self.assertTrue(np.array_equal(result.data, source[:, :4]))

    def test_outside_grid_is_unknown_with_fixed_dimensions(self):
        source = np.arange(20, dtype=np.int8).reshape(4, 5)
        result = crop_occupancy_grid(
            source, resolution=1.0, origin_xy=(10.0, 20.0),
            center_xy=(9.0, 19.0), extent_m=4.0,
        )
        self.assertEqual((result.height, result.width), (4, 4))
        self.assertEqual((result.origin_x, result.origin_y), (7.0, 17.0))
        self.assertEqual(int(result.data[3, 3]), int(source[0, 0]))
        self.assertEqual(int(np.count_nonzero(result.data == -1)), 15)

    def test_competition_default_is_600_cells_at_point_two_meters(self):
        source = np.zeros((7232, 3085), dtype=np.int8)
        result = crop_occupancy_grid(
            source, resolution=0.2, origin_xy=(-200.0, -600.0),
            center_xy=(-131.6898, -428.3310), extent_m=120.0,
        )
        self.assertEqual((result.height, result.width), (600, 600))
        self.assertTrue(np.all(result.data == 0))

    def test_invalid_inputs_are_rejected(self):
        with self.assertRaises(ValueError):
            crop_bounds((0.0, 0.0), 0.0)
        with self.assertRaises(ValueError):
            crop_occupancy_grid(
                np.zeros((2, 2), dtype=np.int8), 0.0, (0.0, 0.0), (0.0, 0.0), 2.0
            )


if __name__ == "__main__":
    unittest.main()
