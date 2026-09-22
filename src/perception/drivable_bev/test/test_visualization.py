#!/usr/bin/env python3

import unittest
from pathlib import Path

import numpy as np

from drivable_bev.calibration import load_calibration_snapshot
from drivable_bev.grid import BevGridSpec
from drivable_bev.visualization import (
    bev_crop_slices,
    build_bev_mosaic,
    colorize_road_marking,
    crop_bev_layers,
    draw_metric_lanes,
    draw_stateful_lane_family,
    draw_ground_grid_overlay,
    render_fused_bev,
    render_fused_quality,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


class VisualizationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        _, cls.cameras = load_calibration_snapshot(
            PACKAGE_ROOT / "config" / "cameras_vip3_v1.yaml"
        )
        cls.grid = BevGridSpec(-5.0, 20.0, -8.0, 8.0, 0.1)

    def test_road_marking_colors(self):
        classes = np.asarray([[0, 1, 2, 3]], dtype=np.uint8)
        colored = colorize_road_marking(classes)
        np.testing.assert_array_equal(colored[0, 0], [0, 0, 0])
        np.testing.assert_array_equal(colored[0, 1], [255, 255, 255])
        np.testing.assert_array_equal(colored[0, 2], [0, 255, 255])
        np.testing.assert_array_equal(colored[0, 3], [0, 0, 255])

    def test_bev_mosaic_has_one_column_per_view(self):
        base = np.zeros(self.grid.shape, dtype=np.uint8)
        layers = {
            view: {
                "drivable": base,
                "road_marking": base,
                "coverage": np.full(self.grid.shape, 255, dtype=np.uint8),
            }
            for view in ("left", "front", "right", "rear")
        }
        mosaic = build_bev_mosaic(layers, self.grid, tile_size=(300, 600))
        # 3 rows x 4 views. VIP3 는 후방 카메라를 포함해 4열이다.
        self.assertEqual((1800, 1200, 3), mosaic.shape)

        three = {view: layers[view] for view in ("left", "front", "right")}
        self.assertEqual(
            (1800, 900, 3),
            build_bev_mosaic(
                three,
                self.grid,
                view_order=("left", "front", "right"),
                tile_size=(300, 600),
            ).shape,
        )

    def test_camera_grid_overlay_preserves_image_shape(self):
        camera = self.cameras["front"]
        image = np.zeros((camera.height, camera.width, 3), dtype=np.uint8)
        overlay = draw_ground_grid_overlay(image, camera)
        self.assertEqual(image.shape, overlay.shape)
        self.assertGreater(np.count_nonzero(overlay), 0)
        self.assertEqual(0, np.count_nonzero(image))

    def test_fused_bev_combines_layers_without_vehicle_marker(self):
        drivable = np.full(self.grid.shape, 180, dtype=np.uint8)
        marking = np.zeros(self.grid.shape, dtype=np.uint8)
        confidence = np.full(self.grid.shape, 255, dtype=np.uint8)
        coverage = np.full(self.grid.shape, 255, dtype=np.uint8)
        source_count = np.full(self.grid.shape, 2, dtype=np.uint8)
        marking[30:40, 100:103] = 1
        marking[30:40, 130:133] = 2
        marking[80:83, 105:135] = 3
        rendered = render_fused_bev(
            drivable,
            marking,
            confidence,
            coverage,
            source_count,
            self.grid,
        )
        self.assertEqual((*self.grid.shape, 3), rendered.shape)
        self.assertGreater(np.count_nonzero(rendered), 0)
        self.assertGreater(np.count_nonzero(rendered[:, :, 1]), 0)

    def test_invalid_road_class_is_rejected(self):
        with self.assertRaises(ValueError):
            colorize_road_marking(np.asarray([[4]], dtype=np.uint8))

    def test_fused_quality_shows_overlap_and_confidence(self):
        marking = np.zeros(self.grid.shape, dtype=np.uint8)
        confidence = np.zeros(self.grid.shape, dtype=np.uint8)
        coverage = np.full(self.grid.shape, 255, dtype=np.uint8)
        source_count = np.ones(self.grid.shape, dtype=np.uint8)
        source_count[:, self.grid.width_px // 2 :] = 2
        marking[20:30, 30:33] = 2
        confidence[20:25, 30:33] = 64
        confidence[25:30, 30:33] = 255
        rendered = render_fused_quality(
            marking,
            confidence,
            coverage,
            source_count,
            self.grid,
            label_prefix="QUALITY TEST",
        )
        self.assertEqual((*self.grid.shape, 3), rendered.shape)
        self.assertFalse(
            np.array_equal(rendered[101, 21], rendered[101, 121])
        )
        self.assertGreater(
            int(rendered[27, 31, 1]), int(rendered[22, 31, 1])
        )

    def test_aligned_display_crop_has_requested_metric_shape(self):
        display = BevGridSpec(0.0, 15.0, -6.0, 6.0, 0.1)
        rows, columns = bev_crop_slices(self.grid, display)
        source = np.arange(np.prod(self.grid.shape), dtype=np.int64).reshape(
            self.grid.shape
        )
        result = crop_bev_layers({"value": source}, self.grid, display)["value"]
        self.assertEqual(display.shape, result.shape)
        np.testing.assert_array_equal(result, source[rows, columns])

    def test_metric_lane_draw_uses_base_link_grid(self):
        canvas = np.zeros((*self.grid.shape, 3), dtype=np.uint8)
        points = np.column_stack(
            [np.linspace(0.0, 10.0, 30), np.full(30, 1.5)]
        )
        draw_metric_lanes(canvas, [(1, points)], self.grid)
        self.assertGreater(np.count_nonzero(canvas), 0)

    def test_stateful_shared_family_has_distinct_state_colors(self):
        canvas = np.zeros((*self.grid.shape, 3), dtype=np.uint8)
        segments = [
            (
                "left_outer_boundary",
                "observed",
                1,
                np.asarray([[2.0, 2.0], [8.0, 2.0]]),
            ),
            (
                "yellow_centre_boundary",
                "structure_inferred",
                2,
                np.asarray([[2.0, 0.0], [8.0, 0.0]]),
            ),
            (
                "right_outer_boundary",
                "invalid",
                1,
                np.asarray([[2.0, -2.0], [8.0, -2.0]]),
            ),
        ]
        rendered = draw_stateful_lane_family(
            canvas, segments, self.grid, status_text="shared=accepted"
        )
        self.assertIs(rendered, canvas)
        observed_pixel = np.rint(
            self.grid.metric_to_pixel([[5.0, 2.0]])[0]
        ).astype(int)
        inferred_pixel = np.rint(
            self.grid.metric_to_pixel([[2.0, 0.0]])[0]
        ).astype(int)
        invalid_pixel = np.rint(
            self.grid.metric_to_pixel([[2.0, -2.0]])[0]
        ).astype(int)
        self.assertGreater(
            int(canvas[observed_pixel[1], observed_pixel[0], 0]), 200
        )
        self.assertGreater(
            int(canvas[inferred_pixel[1], inferred_pixel[0], 1]), 100
        )
        self.assertGreater(
            int(canvas[invalid_pixel[1], invalid_pixel[0], 2]), 100
        )


if __name__ == "__main__":
    unittest.main()
