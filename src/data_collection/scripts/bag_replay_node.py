#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Replay immutable raw bag chunks with a guarded ROS clock lifecycle."""

from __future__ import print_function

import glob
import os
import signal
import subprocess
import time

import rosbag
import rosgraph
import rospy
import rospkg
import yaml
from std_srvs.srv import Trigger, TriggerResponse

from data_collection.bag_profile import (
    BagProfileError,
    load_bag_profile,
    privileged_topics_present,
)


class BagReplayNode(object):
    def __init__(self):
        value = os.path.abspath(os.path.expanduser(str(rospy.get_param("~bag_path"))))
        self.bag_files, self.run_root = self._resolve_bags(value)
        self.allow_privileged_gt = bool(
            rospy.get_param("~allow_privileged_gt", False)
        )
        self.rate = float(rospy.get_param("~rate", 1.0))
        if self.rate <= 0:
            raise ValueError("rate must be positive")
        self.wait_for_start = bool(rospy.get_param("~wait_for_start", True))
        self.post_roll_sec = float(rospy.get_param("~post_roll_sec", 2.0))
        if self.post_roll_sec < 0:
            raise ValueError("post_roll_sec must be non-negative")
        package = rospkg.RosPack().get_path("data_collection")
        profile_dir = os.path.join(package, "config", "bag_profiles")
        self.input_profile = load_bag_profile(
            profile_dir, str(rospy.get_param("~required_profile", "camera_raw"))
        )
        self.process = None
        self.stop_requested = False
        self.finalized = False
        self.start_requested = not self.wait_for_start
        self.start_service = None
        self.previous_exists = rospy.has_param("/use_sim_time")
        self.previous_use_sim_time = rospy.get_param("/use_sim_time", False)
        self._validate_contents()
        self._reject_live_publishers()
        rospy.set_param("/use_sim_time", True)
        if self.wait_for_start:
            self.start_service = rospy.Service("~start", Trigger, self._start)
        rospy.on_shutdown(self.request_stop)

    def _start(self, _request):
        if self.start_requested:
            return TriggerResponse(success=False, message="replay already started")
        self.start_requested = True
        return TriggerResponse(success=True, message="replay start accepted")

    @staticmethod
    def _resolve_bags(path):
        if os.path.isfile(path):
            return [path], os.path.dirname(os.path.dirname(path))
        if not os.path.isdir(path):
            raise OSError("bag path does not exist: {}".format(path))
        candidates = sorted(glob.glob(os.path.join(path, "bags", "*.bag")))
        if not candidates:
            candidates = sorted(glob.glob(os.path.join(path, "*.bag")))
        if not candidates:
            raise OSError("no closed .bag files found under {}".format(path))
        return candidates, path

    def _validate_contents(self):
        topics = set()
        for path in self.bag_files:
            with rosbag.Bag(path, "r") as bag:
                topics.update(bag.get_type_and_topic_info().topics)
        missing = set(self.input_profile["required_topics"]).difference(topics)
        if missing:
            raise BagProfileError(
                "bag is missing required {} topics: {}".format(
                    self.input_profile["name"], sorted(missing)
                )
            )
        present_gt = privileged_topics_present(topics)
        metadata_path = os.path.join(self.run_root, "metadata.yaml")
        privileged_metadata = False
        if os.path.isfile(metadata_path):
            with open(metadata_path, "r") as stream:
                privileged_metadata = bool(
                    (yaml.safe_load(stream) or {}).get("privileged_gt", False)
                )
        if (present_gt or privileged_metadata) and not self.allow_privileged_gt:
            raise BagProfileError(
                "privileged GT bag replay is blocked; set allow_privileged_gt:=true "
                "only for offline oracle analysis"
            )
        self.bag_topics = topics

    def _reject_live_publishers(self):
        published = dict(rosgraph.Master(rospy.get_name()).getSystemState()[0])
        conflicts = {
            topic: nodes for topic, nodes in published.items()
            if topic in self.bag_topics and nodes
        }
        if conflicts:
            raise BagProfileError(
                "live publishers conflict with replay topics; stop the UDP/ROS bridge "
                "before replay: {}".format(conflicts)
            )

    def run(self):
        command = ["rosbag", "play", "--quiet", "--clock", "--rate", str(self.rate)]
        command.extend(self.bag_files)
        if self.wait_for_start:
            rospy.loginfo(
                "replay clock prepared; start downstream nodes, then call %s/start",
                rospy.get_name(),
            )
            while not rospy.is_shutdown() and not self.start_requested:
                time.sleep(0.1)
        if rospy.is_shutdown():
            self.finalize()
            return
        rospy.loginfo("replaying %d bag chunk(s) at %.2fx", len(self.bag_files), self.rate)
        try:
            self.process = subprocess.Popen(command, start_new_session=True)
            while not rospy.is_shutdown() and self.process.poll() is None:
                time.sleep(0.2)
            return_code = self.process.poll()
            if return_code == 0 and self.post_roll_sec:
                rospy.loginfo(
                    "replay complete; keeping simulated clock enabled for %.1fs post-roll",
                    self.post_roll_sec,
                )
                deadline = time.monotonic() + self.post_roll_sec
                while not rospy.is_shutdown() and time.monotonic() < deadline:
                    time.sleep(0.05)
        finally:
            self.finalize()
        if return_code not in (0, None):
            raise RuntimeError("rosbag play exited with rc={}".format(return_code))

    def request_stop(self):
        if self.stop_requested:
            return
        self.stop_requested = True
        if self.process is not None and self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)

    def finalize(self):
        if self.finalized:
            return
        self.finalized = True
        if self.process is not None and self.process.poll() is None:
            if not self.stop_requested:
                self.request_stop()
            try:
                self.process.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGTERM)
        if self.previous_exists:
            rospy.set_param("/use_sim_time", self.previous_use_sim_time)
        elif rospy.has_param("/use_sim_time"):
            rospy.delete_param("/use_sim_time")


def main():
    rospy.init_node("asmc_bag_replay")
    try:
        BagReplayNode().run()
    except (BagProfileError, OSError, RuntimeError, ValueError) as exc:
        rospy.logfatal("bag replay could not start: %s", exc)
        raise SystemExit(2)


if __name__ == "__main__":
    main()
