#!/usr/bin/env python3

import unittest
from pathlib import Path

import numpy as np

from drivable_bev.calibration import load_calibration_snapshot
from drivable_bev.fusion import (
    CameraBevFusion,
    build_quality_maps,
    timestamp_span_ns,
    timestamps_within_slop,
)
from drivable_bev.grid import BevGridSpec
from drivable_bev.projector import CameraBevProjector, ProjectedSemantic


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def semantic(drivable, marking, confidence):
    value = np.asarray(drivable, dtype=np.uint8)
    return ProjectedSemantic(
        drivable_probability=value,
        road_marking_class_id=np.asarray(marking, dtype=np.uint8),
        road_marking_confidence=np.asarray(confidence, dtype=np.uint8),
        coverage=np.full(value.shape, 255, dtype=np.uint8),
    )


class FusionTest(unittest.TestCase):
    def test_timestamp_slop_is_inclusive(self):
        self.assertEqual(35_000_000, timestamp_span_ns([1, 35_000_001]))
        self.assertTrue(timestamps_within_slop([1, 35_000_001], 35_000_000))
        self.assertFalse(timestamps_within_slop([1, 36_000_001], 35_000_000))

    def test_weighted_drivable_and_confident_class_owner(self):
        quality = {
            "front": np.ones((2, 2), dtype=np.float32),
            "left": np.ones((2, 2), dtype=np.float32),
        }
        fusion = CameraBevFusion(quality, {"front": 1, "left": 2})
        result = fusion.fuse(
            {
                "front": semantic([[0, 10], [20, 30]], [[1, 1], [1, 1]], [[100] * 2] * 2),
                "left": semantic([[200, 210], [220, 230]], [[2, 2], [2, 2]], [[240] * 2] * 2),
            }
        )
        np.testing.assert_array_equal(
            result.drivable_probability, [[100, 110], [120, 130]]
        )
        np.testing.assert_array_equal(result.road_marking_class_id, 2)
        np.testing.assert_array_equal(result.road_marking_confidence, 240)
        np.testing.assert_array_equal(result.source_view, 2)
        np.testing.assert_array_equal(result.source_count, 2)

    def test_geometry_remains_primary_over_low_quality_confidence(self):
        quality = {
            "front": np.ones((1, 1), dtype=np.float32),
            "left": np.full((1, 1), 0.4, dtype=np.float32),
        }
        result = CameraBevFusion(quality).fuse(
            {
                "front": semantic([[100]], [[1]], [[0]]),
                "left": semantic([[200]], [[2]], [[255]]),
            }
        )
        self.assertEqual(1, int(result.road_marking_class_id[0, 0]))

    def test_uncovered_and_dynamic_invalid_cells_are_zero(self):
        quality = {"front": np.asarray([[1.0, 0.0]], dtype=np.float32)}
        result = CameraBevFusion(quality).fuse(
            {"front": semantic([[255, 255]], [[3, 3]], [[255, 255]])},
            valid_by_view={"front": np.asarray([[0, 255]], dtype=np.uint8)},
        )
        self.assertEqual([0, 0], result.coverage.tolist()[0])
        self.assertEqual([0, 0], result.road_marking_class_id.tolist()[0])

    def test_quality_map_matches_cropped_geometry(self):
        _, cameras = load_calibration_snapshot(
            PACKAGE_ROOT / "config" / "cameras_vip3_v1.yaml"
        )
        grid = BevGridSpec(-5.0, 20.0, -8.0, 8.0, 0.1)
        projectors = {
            name: CameraBevProjector(camera, grid, 22.0, 0.1)
            for name, camera in cameras.items()
        }
        quality = build_quality_maps(projectors, grid)
        stack = np.stack([quality[name] > 0.0 for name in cameras])
        self.assertGreater(float(np.mean(np.any(stack, axis=0))), 0.87)
        self.assertGreater(float(np.mean(np.count_nonzero(stack, axis=0) >= 2)), 0.43)
        for value in quality.values():
            self.assertEqual(grid.shape, value.shape)
            self.assertEqual(np.float32, value.dtype)
            self.assertTrue(np.isfinite(value).all())


if __name__ == "__main__":
    unittest.main()
