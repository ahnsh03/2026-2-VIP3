#!/usr/bin/env python3

import unittest

import numpy as np

from drivable_bev.lane_evidence import LaneNode
from drivable_bev.lane_tracking import LaneTrack
from drivable_bev.spline_fitting import (
    bspline_basis,
    fit_lane_track,
    fit_lane_tracks_with_audit,
)


class SplineFittingTest(unittest.TestCase):
    def test_basis_is_partition_of_unity(self):
        from drivable_bev.spline_fitting import _open_uniform_knots

        knots = _open_uniform_knots(6, 3)
        basis = bspline_basis(np.linspace(0.0, 1.0, 101), knots, 3, 6)
        np.testing.assert_allclose(basis.sum(axis=1), 1.0, atol=1e-9)

    def test_cubic_fit_rejects_one_outlier_and_is_deterministic(self):
        x = np.linspace(0.0, 12.0, 25)
        y = 1.5 + 0.02 * x * x
        y[12] += 1.0
        nodes = tuple(
            LaneNode(
                np.asarray([px, py]),
                1,
                0.9,
                1.0,
                1,
                10,
                index,
            )
            for index, (px, py) in enumerate(zip(x, y))
        )
        first = fit_lane_track(LaneTrack(nodes), lane_id=1)
        second = fit_lane_track(LaneTrack(nodes), lane_id=1)
        np.testing.assert_allclose(first.sample_points, second.sample_points)
        self.assertLess(first.residual_p50_m, 0.10)
        self.assertGreater(len(first.sample_points), 20)
        self.assertEqual(first.sample_points.shape[:1], first.sample_observed.shape)
        self.assertEqual(1, first.class_id)
        self.assertGreaterEqual(
            first.valid_x_min_m,
            float(np.min(first.sample_points[:, 0])) - 1e-9,
        )
        self.assertLessEqual(
            first.valid_x_max_m,
            float(np.max(first.sample_points[:, 0])) + 1e-9,
        )

    def test_long_gap_is_tagged_as_bridged_not_observed(self):
        x = np.asarray([0.0, 0.25, 0.5, 2.0, 2.25, 2.5])
        nodes = tuple(
            LaneNode(np.asarray([value, 1.0]), 1, 0.9, 1.0, 1, 8, index)
            for index, value in enumerate(x)
        )
        lane = fit_lane_track(LaneTrack(nodes), lane_id=1)
        middle = (lane.sample_points[:, 0] > 0.75) & (lane.sample_points[:, 0] < 1.75)
        self.assertTrue(np.any(middle))
        self.assertFalse(np.any(lane.sample_observed[middle]))
        self.assertGreater(lane.bridged_length_ratio, 0.0)

    def test_batch_audit_counts_fit_failures(self):
        too_short = LaneTrack(
            tuple(
                LaneNode(np.asarray([float(index), 1.0]), 1, 0.9, 1.0, 1, 8, index)
                for index in range(3)
            )
        )
        lanes, audit = fit_lane_tracks_with_audit([too_short])
        self.assertEqual([], lanes)
        self.assertEqual(1, audit["rejected"]["value_error"])



if __name__ == "__main__":
    unittest.main()
