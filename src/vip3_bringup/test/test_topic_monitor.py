import argparse
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from morai_topic_monitor import TopicWindow, format_report, positive_float


class TopicMonitorTest(unittest.TestCase):
    def report(self, window, now, nominal=10):
        return window.snapshot(now, nominal, stale_after=2, slow_ratio=0.6)

    def test_normal_pause_and_resume(self):
        window = TopicWindow(0)
        for index in range(50):
            window.receive(index / 10)
        first = self.report(window, 5)
        self.assertEqual((first["hz"], first["status"]), (10, "OK"))
        paused = self.report(window, 10)
        self.assertEqual((paused["hz"], paused["status"]), (0, "STALE"))
        for index in range(50):
            window.receive(10 + index / 10)
        resumed = self.report(window, 15)
        self.assertEqual((resumed["hz"], resumed["status"]), (10, "OK"))

    def test_startup_without_messages(self):
        row = self.report(TopicWindow(0), 5)
        self.assertEqual(row["status"], "MISSING")
        self.assertIsNone(row["age"])

    def test_burst_then_stall_is_not_healthy(self):
        window = TopicWindow(0)
        for index in range(50):
            window.receive(index / 100)
        row = self.report(window, 5)
        self.assertEqual(row["hz"], 10)
        self.assertEqual(row["status"], "STALE")

    def test_slow_stream_uses_elapsed_window(self):
        window = TopicWindow(0)
        for index in range(20):
            window.receive(index / 4)
        row = self.report(window, 5)
        self.assertEqual((row["hz"], row["status"]), (4, "SLOW"))

    def test_low_nominal_rate_allows_normal_gaps(self):
        window = TopicWindow(0)
        window.receive(0)
        row = self.report(window, 5, nominal=0.1)
        self.assertEqual(row["status"], "OK")
        self.assertEqual(row["age"], 5)

    def test_connection_and_missing_diagnostics(self):
        row = self.report(TopicWindow(0), 5)
        entries = [dict(name="sample", nominal_hz=10)]
        self.assertIn("clients=unknown", format_report(entries, [row], None, 5))
        message = format_report(entries, [row], 0, 5)
        self.assertIn("MISSING", message)
        self.assertIn("last=never", message)
        self.assertIn("ROS Connect", message)

    def test_reject_invalid_timers(self):
        for value in ("0", "-1", "nan", "inf", "-inf"):
            with self.subTest(value=value), self.assertRaises(argparse.ArgumentTypeError):
                positive_float(value)


if __name__ == "__main__":
    unittest.main()
