#!/usr/bin/env python3
"""수신 시각 기준 신선도 판정. rosbridge 가 조용히 멈추는 상황을 잡기 위한 것이다."""

from __future__ import annotations

import unittest

from vip3_vehicle_state.freshness import NEVER_RECEIVED, age_seconds, is_steady_fresh


class FreshnessTest(unittest.TestCase):
    def test_never_received_is_not_fresh(self):
        self.assertFalse(is_steady_fresh(NEVER_RECEIVED, 100.0, 0.5))
        self.assertEqual(float("inf"), age_seconds(NEVER_RECEIVED, 100.0))

    def test_within_timeout_is_fresh(self):
        self.assertTrue(is_steady_fresh(100.0, 100.4, 0.5))
        self.assertTrue(is_steady_fresh(100.0, 100.5, 0.5))

    def test_beyond_timeout_is_stale(self):
        self.assertFalse(is_steady_fresh(100.0, 100.6, 0.5))

    def test_negative_age_is_rejected(self):
        self.assertFalse(is_steady_fresh(100.0, 99.0, 0.5))

    def test_non_positive_timeout_is_rejected(self):
        self.assertFalse(is_steady_fresh(100.0, 100.1, 0.0))
        self.assertFalse(is_steady_fresh(100.0, 100.1, -1.0))

    def test_age_seconds_is_plain_difference(self):
        self.assertAlmostEqual(0.25, age_seconds(100.0, 100.25))


if __name__ == "__main__":
    unittest.main()
