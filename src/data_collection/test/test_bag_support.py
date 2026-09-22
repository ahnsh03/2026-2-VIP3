#!/usr/bin/env python3

import os
import tempfile
import unittest
from datetime import datetime, timezone

import yaml

from data_collection.bag_profile import BagProfileError, load_bag_profile
from data_collection.bag_run import build_rosbag_command, make_bag_run_id


class BagProfileTest(unittest.TestCase):
    def write(self, root, name, value):
        with open(os.path.join(root, name + ".yaml"), "w") as stream:
            yaml.safe_dump(value, stream)

    def test_non_gt_profile_rejects_privileged_topics(self):
        with tempfile.TemporaryDirectory() as root:
            self.write(
                root,
                "bad",
                {
                    "privileged_gt": False,
                    "required_topics": ["/velodyne_points_instance"],
                },
            )
            with self.assertRaisesRegex(BagProfileError, "privileged"):
                load_bag_profile(root, "bad")

    def test_alias_keeps_child_name(self):
        with tempfile.TemporaryDirectory() as root:
            self.write(
                root,
                "parent",
                {"name": "parent", "required_topics": ["/camera"]},
            )
            self.write(root, "alias", {"extends": "parent"})
            profile = load_bag_profile(root, "alias")
            self.assertEqual(profile["name"], "alias")
            self.assertEqual(profile["required_topics"], ["/camera"])

    def test_repository_profiles_parse_and_keep_runtime_gt_separate(self):
        root = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "..", "config", "bag_profiles")
        )
        names = [
            os.path.splitext(name)[0]
            for name in os.listdir(root)
            if name.endswith(".yaml")
        ]
        self.assertGreaterEqual(len(names), 7)
        for name in names:
            profile = load_bag_profile(root, name)
            listed = set(profile["required_topics"] + profile["optional_topics"])
            if not profile["privileged_gt"]:
                self.assertNotIn("/velodyne_points_instance", listed)
                self.assertNotIn("/Object_topic", listed)
                self.assertFalse(any(topic.startswith("/sem_") for topic in listed))


class BagRunTest(unittest.TestCase):
    def test_run_id_is_kst_and_slugged(self):
        now = datetime(2026, 9, 9, 5, 30, 12, tzinfo=timezone.utc)
        value = make_bag_run_id(
            "KCity-2025", "Comp Sample", "Foggy", 13, "Game Wheel", now=now
        )
        self.assertEqual(
            value,
            "20260909_143012_kcity_2025_comp_sample_foggy_1pm_game_wheel",
        )

    def test_rosbag_command_applies_split_and_lz4(self):
        command = build_rosbag_command(
            {"compression": "lz4"}, "/tmp/run/bags/model", ["/result"],
            split_gb=4, buffer_mb=1024
        )
        self.assertIn("--size=4096", command)
        self.assertIn("--buffsize=1024", command)
        self.assertIn("--lz4", command)
        self.assertEqual(command[-1], "/result")


if __name__ == "__main__":
    unittest.main()
