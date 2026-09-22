import unittest

import numpy as np

from vip3_hd_map.map_overlay import (
    SegmentLayer,
    dashed_polyline_segments,
    grouped_lane_layers,
    lane_color_group,
    lane_shape_group,
)


class MapOverlayTest(unittest.TestCase):
    def test_origin_translation_and_base_link_transform(self):
        layer = SegmentLayer.from_features(
            [{"points": [[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]]}],
            translation_xy=(10.0, 20.0),
        )
        local = layer.around_ego((10.0, 20.0), heading_deg=90.0, radius_m=5.0)
        self.assertEqual(len(layer), 1)
        self.assertTrue(
            np.allclose(local, [[[0.0, 0.0], [0.0, -2.0]]], atol=1e-9)
        )

    def test_local_radius_keeps_crossing_segment(self):
        layer = SegmentLayer(
            np.asarray([[[-10.0, 0.0], [10.0, 0.0]]], dtype=np.float64)
        )
        local = layer.around_ego((0.0, 0.0), heading_deg=0.0, radius_m=1.0)
        self.assertEqual(local.shape, (1, 2, 2))

    def test_nearest_distance_uses_segment_projection(self):
        layer = SegmentLayer(
            np.asarray([[[0.0, 0.0], [10.0, 0.0]], [[20.0, 5.0], [20.0, 5.0]]])
        )
        self.assertAlmostEqual(layer.nearest_distance_m((4.0, 3.0)), 3.0)
        self.assertAlmostEqual(layer.nearest_distance_m((-2.0, 0.0)), 2.0)

    def test_nearest_distance_rejects_invalid_point(self):
        layer = SegmentLayer(np.asarray([[[0.0, 0.0], [1.0, 0.0]]]))
        with self.assertRaises(ValueError):
            layer.nearest_distance_m((float("nan"), 0.0))

    def test_lane_colors_are_grouped_without_promoting_unknown(self):
        features = [
            {"lane_color": ["White"], "points": [[0, 0], [1, 0]]},
            {"lane_color": ["Yellow"], "points": [[0, 1], [1, 1]]},
            {"lane_color": ["Purple"], "points": [[0, 2], [1, 2]]},
        ]
        grouped = grouped_lane_layers(features, (0.0, 0.0))
        self.assertEqual(len(grouped["white"].segments_xy), 1)
        self.assertEqual(len(grouped["yellow"].segments_xy), 1)
        self.assertEqual(len(grouped["unknown"].segments_xy), 1)
        self.assertEqual(lane_color_group(features[2]), "unknown")

    def test_lane_shape_reads_canonical_and_preserved_mgeo_values(self):
        self.assertEqual(lane_shape_group({"attributes": {"lane_shape": "broken"}}), "broken")
        self.assertEqual(
            lane_shape_group({"attributes": {"mgeo": {"lane_shape": ["Broken"]}}}),
            "broken",
        )
        self.assertEqual(lane_shape_group({"attributes": {"lane_shape": "solid"}}), "solid")
        self.assertEqual(lane_shape_group({"attributes": {"pattern": "broken"}}), "broken")
        self.assertEqual(lane_shape_group({"attributes": {}}), "solid")

    def test_metric_dash_segments_preserve_dash_and_gap_lengths(self):
        segments = dashed_polyline_segments(
            [[[0.0, 0.0], [10.0, 0.0]]], dash_length_m=2.0, gap_length_m=1.0,
        )
        self.assertEqual(segments.shape, (4, 2, 2))
        self.assertTrue(np.allclose(segments[:, 0, 0], [0.0, 3.0, 6.0, 9.0]))
        self.assertTrue(np.allclose(segments[:, 1, 0], [2.0, 5.0, 8.0, 10.0]))
        phased = dashed_polyline_segments(
            [[[0.0, 0.0], [10.0, 0.0]]],
            dash_length_m=2.0, gap_length_m=1.0, phase_m=2.5,
        )
        self.assertTrue(np.allclose(phased[:, 0, 0], [0.5, 3.5, 6.5, 9.5]))
        self.assertTrue(np.allclose(phased[:, 1, 0], [2.5, 5.5, 8.5, 10.0]))
        with self.assertRaises(ValueError):
            dashed_polyline_segments([[[0.0, 0.0], [1.0, 0.0]]], 0.0, 1.0)


if __name__ == "__main__":
    unittest.main()
