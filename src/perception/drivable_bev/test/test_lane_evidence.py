#!/usr/bin/env python3

import unittest

import numpy as np

from drivable_bev.lane_evidence import reduce_evidence_to_nodes
from drivable_bev.model_grid_projection import MetricEvidence


class LaneEvidenceTest(unittest.TestCase):
    def test_line_thickness_and_views_collapse_to_one_node(self):
        evidence = MetricEvidence(
            xy=np.asarray([[1.02, 1.0], [1.08, 1.05], [1.10, 1.08]]),
            class_id=np.asarray([1, 1, 1], dtype=np.uint8),
            confidence=np.asarray([0.8, 0.9, 0.7], dtype=np.float32),
            geometric_quality=np.ones(3, dtype=np.float32),
            source_mask=np.asarray([1, 2, 4], dtype=np.uint8),
        )
        nodes = reduce_evidence_to_nodes(evidence, x_origin_m=0.0)
        self.assertEqual(1, len(nodes))
        self.assertEqual(7, nodes[0].source_mask)
        self.assertEqual(3, nodes[0].support)

    def test_different_classes_and_lateral_lines_stay_separate(self):
        evidence = MetricEvidence(
            xy=np.asarray([[1.0, -2.0], [1.0, 2.0], [1.0, 2.05]]),
            class_id=np.asarray([1, 1, 2], dtype=np.uint8),
            confidence=np.ones(3, dtype=np.float32),
            geometric_quality=np.ones(3, dtype=np.float32),
            source_mask=np.ones(3, dtype=np.uint8),
        )
        nodes = reduce_evidence_to_nodes(evidence, x_origin_m=0.0)
        self.assertEqual(3, len(nodes))
        self.assertEqual([1, 1, 2], [node.class_id for node in nodes])


if __name__ == "__main__":
    unittest.main()
