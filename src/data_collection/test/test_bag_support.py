#!/usr/bin/env python3

import os
import tempfile
import unittest
from pathlib import Path
from datetime import datetime, timezone

import yaml

from data_collection.bag_profile import (
    BagProfileError,
    load_bag_profile,
    simulator_gt_topics,
)
from data_collection.bag_run import build_rosbag_command, make_bag_run_id


class BagProfileTest(unittest.TestCase):
    def write(self, root, name, value):
        with open(os.path.join(root, name + ".yaml"), "w") as stream:
            yaml.safe_dump(value, stream)

    def test_non_gt_profile_rejects_simulator_ground_truth(self):
        """시뮬만 아는 정보와 실차가 볼 수 있는 정보를 섞으면 평가가 무의미해진다."""
        for topic in ("/Object_topic", "/CollisionData", "/sem_front/image"):
            with tempfile.TemporaryDirectory() as root:
                self.write(
                    root,
                    "bad",
                    {"simulator_gt": False, "required_topics": [topic]},
                )
                with self.assertRaisesRegex(BagProfileError, "simulator_gt"):
                    load_bag_profile(root, "bad")

    def test_gt_profile_may_record_ground_truth(self):
        with tempfile.TemporaryDirectory() as root:
            self.write(
                root,
                "ok",
                {"simulator_gt": True, "required_topics": ["/Object_topic"]},
            )
            profile = load_bag_profile(root, "ok")
            self.assertTrue(profile["simulator_gt"])

    def test_udp_transport_is_rejected(self):
        """VIP3 전송은 rosbridge 하나뿐이다. ASMC 프로파일을 복붙하면 여기서 터진다."""
        with tempfile.TemporaryDirectory() as root:
            self.write(
                root,
                "legacy",
                {"transport": "udp", "required_topics": ["/Ego_topic"]},
            )
            with self.assertRaisesRegex(BagProfileError, "transport"):
                load_bag_profile(root, "legacy")

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
            # rosbridge 외 전송은 존재하지 않는다.
            self.assertIn(profile["transport"], ("rosbridge", "replay"))
            if not profile["simulator_gt"]:
                self.assertNotIn("/Object_topic", listed)
                self.assertNotIn("/CollisionData", listed)
                self.assertFalse(any(topic.startswith("/sem_") for topic in listed))
                self.assertFalse(any(topic.startswith("/inst_") for topic in listed))
        # 실제로 GT 프로파일이 존재해야 위 분기가 의미가 있다.
        gt_names = [
            name
            for name in names
            if load_bag_profile(root, name)["simulator_gt"]
        ]
        self.assertTrue(gt_names, "simulator_gt 프로파일이 하나도 없다")


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


class SimulatorGtGateTest(unittest.TestCase):
    """`bag_replay_node` 가 이 함수 이름에 의존한다.

    ASMC 의 대회 게이트(`privileged_gt`)를 VIP3 게이트(`simulator_gt`)로 바꿀 때
    라이브러리만 고치고 재생 노드의 import 를 안 고쳐서, **bag 재생 노드가 기동 즉시
    ImportError 로 죽는** 상태가 한동안 있었다. 노드는 rospy 없이 import 할 수 없어
    테스트 190개가 전부 통과하는 동안에도 아무도 몰랐다. 이름을 여기서 고정한다.
    (`tools/check_node_imports.py` 가 같은 사고를 정적으로도 잡는다.)
    """

    def test_detects_simulator_only_topics(self):
        found = simulator_gt_topics(
            ["/Ego_topic", "/Object_topic", "/sem_rear/image", "/velodyne_points"]
        )
        self.assertEqual(["/Object_topic", "/sem_rear/image"], found)

    def test_plain_sensor_topics_are_not_ground_truth(self):
        self.assertEqual(
            [], simulator_gt_topics(["/cam_front/image_jpeg/compressed", "/gps", "/imu"])
        )

    def test_replay_node_gates_on_this_name(self):
        node = (
            Path(__file__).resolve().parents[1] / "scripts" / "bag_replay_node.py"
        ).read_text(encoding="utf-8")
        self.assertIn("simulator_gt_topics", node)
        self.assertIn("allow_simulator_gt", node)
        self.assertNotIn("privileged", node, "ASMC 대회 게이트 이름이 남아 있다")


if __name__ == "__main__":
    unittest.main()
