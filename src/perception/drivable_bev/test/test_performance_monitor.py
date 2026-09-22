#!/usr/bin/env python3

import unittest
from unittest.mock import patch

from drivable_bev.performance_monitor import (
    PerformanceMonitor,
    source_time_gate,
    source_time_gate_with_reset,
)


class PerformanceMonitorTest(unittest.TestCase):
    def test_source_time_gate_keeps_average_rate_without_drift(self):
        period_ns = 100_000_000
        input_period_ns = 66_666_667
        deadline_ns = 0
        published = []
        for index in range(31):
            stamp_ns = 1_000_000_000 + index * input_period_ns
            publish, deadline_ns = source_time_gate(
                stamp_ns, deadline_ns, period_ns
            )
            if publish:
                published.append(stamp_ns)

        self.assertEqual(len(published), 21)
        self.assertGreaterEqual(deadline_ns, published[-1])
        self.assertLess(deadline_ns - published[-1], period_ns)

    def test_source_time_gate_rejects_early_frame(self):
        publish, deadline_ns = source_time_gate(1_000, 0, 100)
        self.assertTrue(publish)
        self.assertEqual(deadline_ns, 1_100)
        publish, unchanged_deadline = source_time_gate(1_050, deadline_ns, 100)
        self.assertFalse(publish)
        self.assertEqual(unchanged_deadline, deadline_ns)

    def test_source_time_gate_resets_for_new_replay_epoch(self):
        publish, deadline, reset = source_time_gate_with_reset(
            1_000, 5_000, 5_100, 100
        )
        self.assertTrue(publish)
        self.assertTrue(reset)
        self.assertEqual(1_100, deadline)

    def test_source_time_gate_does_not_reset_for_small_out_of_order_sample(self):
        publish, deadline, reset = source_time_gate_with_reset(
            4_950, 5_000, 5_100, 100
        )
        self.assertFalse(publish)
        self.assertFalse(reset)
        self.assertEqual(5_100, deadline)

    def test_counts_and_latency_statistics_are_separate(self):
        monitor = PerformanceMonitor(["front"])
        monitor.count_input("front", 123456789)
        monitor.count_throttled("front")
        monitor.count_error("front")

        with patch(
            "drivable_bev.performance_monitor.time.monotonic",
            side_effect=[10.0, 10.1],
        ):
            monitor.count_output(
                "front", 2.0, 0.5, source_stamp_ns=100, result_stamp_ns=110
            )
            monitor.count_output(
                "front", 4.0, 1.5, source_stamp_ns=200, result_stamp_ns=220
            )

        stats = monitor.snapshot()["front"]
        self.assertEqual(stats["input_pairs"], 1)
        self.assertEqual(stats["outputs"], 2)
        self.assertEqual(stats["throttled"], 1)
        self.assertEqual(stats["errors"], 1)
        self.assertEqual(stats["last_input_stamp_ns"], 123456789)
        self.assertEqual(stats["last_output_source_stamp_ns"], 200)
        self.assertEqual(stats["last_result_stamp_ns"], 220)
        self.assertAlmostEqual(stats["processing_fps"], 10.0)
        self.assertAlmostEqual(stats["projection_ms_mean"], 3.0)
        self.assertAlmostEqual(stats["publish_ms_mean"], 1.0)
        self.assertAlmostEqual(stats["total_ms_mean"], 4.0)
        self.assertGreaterEqual(stats["total_ms_p95"], 5.0)

    def test_empty_snapshot_is_zero(self):
        stats = PerformanceMonitor(["left"]).snapshot()["left"]
        self.assertEqual(stats["outputs"], 0)
        self.assertEqual(stats["processing_fps"], 0.0)
        self.assertEqual(stats["projection_ms_p95"], 0.0)
        self.assertEqual(stats["total_ms_p95"], 0.0)


if __name__ == "__main__":
    unittest.main()
