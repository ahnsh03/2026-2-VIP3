#!/usr/bin/env python3

import unittest

import numpy as np

from camera_semantic_perception.visualization import (
    DRIVABLE_COLOR_BGR,
    LANE_COLOR_BGR,
    STOPLINE_COLOR_BGR,
    WHITE_LANE_COLOR_BGR,
    YELLOW_LANE_COLOR_BGR,
    hard_mask,
    render_mosaic,
    render_overlay,
    render_panel,
)


class VisualizationTest(unittest.TestCase):
    def test_hard_mask_accepts_uint8_probability(self):
        probability = np.array([[0, 127, 128, 255]], dtype=np.uint8)
        np.testing.assert_array_equal(
            hard_mask(probability, 0.5), [[False, False, True, True]]
        )

    def test_overlay_gives_lane_priority(self):
        image = np.zeros((4, 6, 3), dtype=np.uint8)
        drivable = np.ones((4, 6), dtype=np.float32)
        lane = np.zeros((4, 6), dtype=np.float32)
        lane[1, 2] = 1.0
        overlay = render_overlay(image, drivable, lane, alpha=1.0)
        self.assertEqual(tuple(overlay[0, 0]), DRIVABLE_COLOR_BGR)
        self.assertEqual(tuple(overlay[1, 2]), LANE_COLOR_BGR)

    def test_panel_places_original_drivable_lane_in_three_rows(self):
        image = np.full((720, 1280, 3), 7, dtype=np.uint8)
        drivable = np.ones((720, 1280), dtype=np.float32)
        lane = np.ones((720, 1280), dtype=np.float32)
        panel = render_panel(image, drivable, lane, "front", panel_width=640)
        self.assertEqual(panel.shape, (1080, 640, 3))
        self.assertEqual(tuple(panel[100, 100]), (7, 7, 7))
        self.assertEqual(tuple(panel[460, 100]), DRIVABLE_COLOR_BGR)
        self.assertEqual(tuple(panel[820, 100]), LANE_COLOR_BGR)

    def test_panel_colors_four_class_road_marking(self):
        image = np.full((720, 1280, 3), 7, dtype=np.uint8)
        drivable = np.ones((720, 1280), dtype=np.float32)
        marking = np.zeros((720, 1280), dtype=np.uint8)
        marking[:, :400] = 1
        marking[:, 400:800] = 2
        marking[:, 800:] = 3
        panel = render_panel(
            image,
            drivable,
            marking,
            "front",
            secondary_mode="road_marking",
            panel_width=640,
        )
        self.assertEqual(panel.shape, (1080, 640, 3))
        self.assertEqual(tuple(panel[820, 100]), WHITE_LANE_COLOR_BGR)
        self.assertEqual(tuple(panel[820, 300]), YELLOW_LANE_COLOR_BGR)
        self.assertEqual(tuple(panel[820, 500]), STOPLINE_COLOR_BGR)

    def test_four_view_mosaic_order_and_shape(self):
        view_values = {"left": 10, "front": 20, "right": 30, "rear": 40}
        panels = {}
        for view, base_value in view_values.items():
            rows = [
                np.full((100, 200, 3), base_value + row, dtype=np.uint8)
                for row in (1, 2, 3)
            ]
            panels[view] = np.concatenate(rows, axis=0)
        mosaic = render_mosaic(
            panels, ("left", "front", "right", "rear"), per_view_width=200
        )
        self.assertEqual(mosaic.shape, (300, 800, 3))
        for row_index, y in enumerate((50, 150, 250), start=1):
            for column_index, x in enumerate((100, 300, 500, 700), start=1):
                self.assertEqual(
                    int(mosaic[y, x, 0]), column_index * 10 + row_index
                )

    def test_vip3_mosaic_fits_1920x1080(self):
        # front/rear panel 640x1080(3x360), left/right panel 640x1440(3x480).
        # mosaic_view_width=480 이면 정확히 1920x1080 이 나와야 한다.
        panels = {
            "front": np.zeros((1080, 640, 3), dtype=np.uint8),
            "rear": np.zeros((1080, 640, 3), dtype=np.uint8),
            "left": np.zeros((1440, 640, 3), dtype=np.uint8),
            "right": np.zeros((1440, 640, 3), dtype=np.uint8),
        }
        mosaic = render_mosaic(
            panels, ("left", "front", "right", "rear"), per_view_width=480
        )
        self.assertEqual(mosaic.shape, (1080, 1920, 3))

    def test_mosaic_letterboxes_views_with_different_aspect_ratios(self):
        front = np.full((300, 200, 3), 10, dtype=np.uint8)
        side = np.full((450, 200, 3), 20, dtype=np.uint8)
        mosaic = render_mosaic(
            {"front": front, "left": side},
            ("front", "left"),
            per_view_width=200,
        )
        self.assertEqual(mosaic.shape, (450, 400, 3))
        self.assertEqual(tuple(mosaic[10, 100]), (0, 0, 0))
        self.assertEqual(tuple(mosaic[30, 100]), (10, 10, 10))
        self.assertEqual(tuple(mosaic[10, 300]), (20, 20, 20))

    def test_shape_mismatch_is_rejected(self):
        image = np.zeros((10, 20, 3), dtype=np.uint8)
        probability = np.zeros((9, 20), dtype=np.float32)
        with self.assertRaisesRegex(ValueError, "does not match"):
            render_overlay(image, probability, probability)


if __name__ == "__main__":
    unittest.main()
