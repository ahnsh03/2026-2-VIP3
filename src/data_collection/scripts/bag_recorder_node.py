#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""프로파일 기반 rosbag 레코더.

allowlist 로 `rosbag record` 서브프로세스를 띄우고, run_id 자동 생성·디스크 가드·
토픽별 수신율 통계·bag sha256·git 상태를 metadata.yaml 에 남긴다.

VIP3 변경점 (ASMC 대비):
  - UDP 브리지 노드 lookup / 브리지 파라미터 스냅샷 삭제. 전송은 rosbridge 하나뿐이고
    모든 메시지가 websocket 수신 시각으로 찍히므로 timestamp_mode 는 항상 `receive` 다.
  - gRPC(:7789) weather/sim_hour 자동 조회와 environment_timeline.jsonl 삭제.
    대신 `weather:=sunny sim_hour:=11` 런치 인자로 명시한다.
  - result/parent lineage(모델 결과 bag) 삭제. VIP3 프로파일은 전부 kind: raw 다.
  - privileged_gt / deployment_allowed 게이트 삭제.

브리지 헬스 신호는 여기 TopicCounter.summary() 의 average_hz / start_window_hz /
end_window_hz 와 scripts/audit_rosbag_run.py 의 header-lag 통계로 본다.
"""

from __future__ import print_function

import glob
import json
import os
import shutil
import signal
import subprocess
import threading
import time
from collections import deque
from datetime import datetime, timezone

import rosgraph
import rospy
import rospkg
import yaml
from diagnostic_msgs.msg import DiagnosticArray

from data_collection.bag_profile import BagProfileError, load_bag_profile
from data_collection.bag_run import (
    build_rosbag_command,
    default_rosbag_root,
    make_bag_run_id,
    sha256_file,
    slug,
)

def utc_now_iso():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def write_yaml_atomic(path, value):
    temporary = path + ".tmp"
    with open(temporary, "w") as stream:
        yaml.safe_dump(value, stream, allow_unicode=True, sort_keys=True)
    os.replace(temporary, path)


def git_metadata(repo_root):
    def run(*args):
        git_dir = os.path.join(repo_root, ".git")
        return subprocess.check_output(
            ["git", "--git-dir={}".format(git_dir),
             "--work-tree={}".format(repo_root)] + list(args),
            stderr=subprocess.DEVNULL,
            universal_newlines=True,
        ).strip()

    result = {"root": repo_root, "commit": "unknown", "branch": "unknown"}
    try:
        result["commit"] = run("rev-parse", "HEAD")
        result["branch"] = run("rev-parse", "--abbrev-ref", "HEAD")
        result["dirty"] = bool(run("status", "--porcelain"))
    except (OSError, subprocess.CalledProcessError) as exc:
        result["error"] = "{}: {}".format(type(exc).__name__, exc)
        result["dirty"] = None
    return result


class TopicCounter(object):
    def __init__(self, topics):
        self._lock = threading.Lock()
        self._values = {
            topic: {
                "messages": 0,
                "first_wall": None,
                "last_wall": None,
                "start_window": [],
                "end_window": deque(maxlen=512),
            }
            for topic in topics
        }
        self._latest_diagnostics = {}
        self._subscribers = [
            rospy.Subscriber(
                topic,
                DiagnosticArray if topic.endswith("/diagnostics") else rospy.AnyMsg,
                self._callback,
                callback_args=topic, queue_size=1, buff_size=1 << 24
            )
            for topic in topics
        ]

    def _callback(self, _message, topic):
        now = time.monotonic()
        with self._lock:
            value = self._values[topic]
            value["messages"] += 1
            if value["first_wall"] is None:
                value["first_wall"] = now
            value["last_wall"] = now
            if now - value["first_wall"] <= 5.0:
                value["start_window"].append(now)
            value["end_window"].append(now)
            while value["end_window"] and now - value["end_window"][0] > 5.0:
                value["end_window"].popleft()
            if topic.endswith("/diagnostics") and hasattr(_message, "status"):
                self._latest_diagnostics[topic] = {
                    status.name: {
                        "level": int(status.level),
                        "message": status.message,
                        "values": dict((item.key, item.value) for item in status.values),
                    }
                    for status in _message.status
                }

    def summary(self):
        def window_rate(values):
            if len(values) < 2 or values[-1] <= values[0]:
                return 0.0
            return (len(values) - 1) / (values[-1] - values[0])

        result = {}
        with self._lock:
            values = {}
            for key, value in self._values.items():
                copied = dict(value)
                copied["start_window"] = list(value["start_window"])
                copied["end_window"] = list(value["end_window"])
                values[key] = copied
        for topic, value in values.items():
            duration = None
            overall_rate = 0.0
            if value["first_wall"] is not None and value["last_wall"] is not None:
                duration = max(0.0, value["last_wall"] - value["first_wall"])
                if value["messages"] > 1 and duration > 0:
                    overall_rate = (value["messages"] - 1) / duration
            result[topic] = {
                "messages": value["messages"],
                "observed_seconds": None if duration is None else round(duration, 3),
                "average_hz": round(overall_rate, 3),
                "start_window_hz": round(
                    window_rate(value["start_window"]), 3
                ),
                "end_window_hz": round(window_rate(value["end_window"]), 3),
            }
        return result

    def diagnostics(self):
        with self._lock:
            return dict(self._latest_diagnostics)

    def close(self):
        for subscriber in self._subscribers:
            subscriber.unregister()


class BagRecorderNode(object):
    def __init__(self):
        self.package_path = rospkg.RosPack().get_path("data_collection")
        self.repo_root = os.path.abspath(os.path.join(self.package_path, "..", ".."))
        profile_dir = os.path.join(self.package_path, "config", "bag_profiles")
        self.profile = load_bag_profile(
            profile_dir, str(rospy.get_param("~profile", "parking_raw"))
        )
        self.profile_name = self.profile["name"]
        self.process = None
        self.counter = None
        self._stop_lock = threading.Lock()
        self._finalized = False
        self._requested_stop = False

        scenario_value = str(rospy.get_param("~scenario", "")).strip()
        controller_value = str(rospy.get_param("~controller", "")).strip()
        self.scenario = slug(scenario_value, fallback="scenario")
        self.controller = slug(controller_value, fallback="controller")
        self.map_slug = slug(
            rospy.get_param("~map_slug", self.profile.get("map", "katri")),
            fallback="katri",
        )
        self.root = os.path.abspath(
            os.path.expanduser(rospy.get_param("~rosbag_root", "") or default_rosbag_root())
        )
        self.topic_timeout = float(rospy.get_param("~topic_timeout_sec", 10.0))
        self.min_free_gb = float(rospy.get_param("~min_free_gb", 20.0))
        self.split_gb = float(rospy.get_param("~split_gb", 4.0))
        self.buffer_mb = int(rospy.get_param("~buffer_mb", 1024))

        if not scenario_value or not controller_value:
            raise BagProfileError(
                "녹화하려면 scenario 와 controller 를 명시해야 한다 "
                "(예: scenario:=slot_a controller:=wheel)"
            )

        # 날씨/시뮬 시각은 MORAI 에 ROS 로 물어볼 방법이 없다(26.R1 에 해당 서비스 없음).
        # gRPC 조회를 삭제했으므로 런치 인자로 운전자가 직접 적는다. 안 적으면 unknown 으로
        # run_id 에 남고, 나중에 어떤 조건에서 찍은 데이터인지 알 수 없게 된다.
        weather_override = str(rospy.get_param("~weather", "")).strip()
        sim_hour_override = str(rospy.get_param("~sim_hour", "")).strip()
        weather = weather_override or "unknown"
        sim_hour = int(sim_hour_override) if sim_hour_override else None
        if not weather_override or not sim_hour_override:
            rospy.logwarn(
                "weather/sim_hour 가 비어 있다. `weather:=sunny sim_hour:=11` 처럼 "
                "런치 인자로 넘기면 run_id 와 metadata 에 기록된다"
            )
        self.naming_environment = {
            "weather": weather,
            "weather_source": "launch_override" if weather_override else "unset",
            "sim_hour": sim_hour,
            "sim_hour_source": "launch_override" if sim_hour_override else "unset",
        }
        initial_environment = {
            "weather": weather,
            "sim_hour": sim_hour,
            "source": "launch_override",
        }

        requested_run_id = str(rospy.get_param("~run_id", "")).strip()
        auto_run_id = make_bag_run_id(
            self.map_slug, self.scenario, weather, sim_hour, self.controller
        )
        self.run_id = slug(requested_run_id, fallback=auto_run_id, max_length=120) \
            if requested_run_id else auto_run_id

        self.run_root = self._resolve_run_root()
        self.bags_dir = os.path.join(self.run_root, "bags")
        self.metadata_path = os.path.join(self.run_root, "metadata.yaml")

        published = self._wait_for_topics()
        topics = list(self.profile["required_topics"])
        topics.extend(
            topic for topic in self.profile["optional_topics"] if topic in published
        )
        self._reject_duplicate_publishers(topics)
        self._validate_timestamp_contract()
        self._prepare_storage()
        self.recorded_topics = topics
        self.topic_types = dict((topic, published[topic]) for topic in topics)
        self.counter = TopicCounter(topics)

        output_prefix = os.path.join(self.bags_dir, self.run_id)
        self.command = build_rosbag_command(
            self.profile, output_prefix, topics,
            split_gb=self.split_gb, buffer_mb=self.buffer_mb
        )
        self.metadata = self._initial_metadata(initial_environment)
        write_yaml_atomic(self.metadata_path, self.metadata)
        self.initial_environment = initial_environment
        rospy.on_shutdown(self.request_stop)

    def _resolve_run_root(self):
        return os.path.join(self.root, "raw", self.run_id)

    def _prepare_storage(self):
        os.makedirs(self.root, exist_ok=True)
        free_gb = shutil.disk_usage(self.root).free / float(1024 ** 3)
        if free_gb < self.min_free_gb:
            raise OSError(
                "insufficient free space: {:.2f} GiB < {:.2f} GiB".format(
                    free_gb, self.min_free_gb
                )
            )
        if os.path.exists(self.run_root):
            raise OSError("run already exists; refusing overwrite: {}".format(self.run_root))
        os.makedirs(self.bags_dir, exist_ok=False)

    def _wait_for_topics(self):
        deadline = time.monotonic() + self.topic_timeout
        published = {}
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            published = dict(rospy.get_published_topics())
            missing = set(self.profile["required_topics"]).difference(published)
            if not missing:
                break
            time.sleep(0.2)
        missing = set(self.profile["required_topics"]).difference(published)
        if missing:
            raise BagProfileError(
                "required topics are not published: {}".format(sorted(missing))
            )
        for topic, expected in (self.profile.get("expected_types") or {}).items():
            if topic in published and published[topic] != expected:
                raise BagProfileError(
                    "{} type mismatch: expected {}, got {}".format(
                        topic, expected, published[topic]
                    )
                )
        return published

    @staticmethod
    def _publishers_by_topic():
        state = rosgraph.Master(rospy.get_name()).getSystemState()
        return dict(state[0])

    def _reject_duplicate_publishers(self, topics):
        publishers = self._publishers_by_topic()
        duplicates = dict(
            (topic, publishers[topic])
            for topic in topics
            if len(publishers.get(topic, [])) > 1
        )
        if duplicates:
            raise BagProfileError(
                "multiple publishers on canonical topics: {}".format(duplicates)
            )

    def _validate_timestamp_contract(self):
        # rosbridge 로 받는 실시간 녹화는 언제나 수신 시각(receive) 이다. 확인할
        # 브리지 노드도 timestamp_mode 파라미터도 없다. 검증할 게 있는 경우는
        # bag 재생 위에서 다시 녹화할 때뿐이다.
        if self.profile["timestamp_mode"] == "replay":
            if not bool(rospy.get_param("/use_sim_time", False)):
                raise BagProfileError(
                    "timestamp_mode=replay 프로파일은 /use_sim_time=true 가 필요하다. "
                    "bag_replay 를 먼저 띄울 것"
                )

    def _initial_metadata(self, environment):
        # 브리지 설정은 정적이다. 조회할 브리지 노드가 없다(rosbridge 는 파라미터를
        # 내걸지 않는다). rosbridge_websocket 파라미터가 보이면 참고로 같이 적는다.
        bridge_configuration = {
            "transport": "rosbridge",
            "url": str(rospy.get_param("~bridge_url", "ws://127.0.0.1:9090")),
            "network_set": "VIP3_network_v1",
            "sensor_set": "VIP3_sensor_set_v1",
            "morai_msgs_branch": "26.R1 @ 4c9be6f",
            "rosbridge_websocket": rospy.get_param("/rosbridge_websocket", {}),
        }
        bridge_configuration_source = "static_vip3_config"
        return {
            "schema_version": "vip3-rosbag-1.0.0",
            "status": "recording",
            "run_id": self.run_id,
            "profile": self.profile_name,
            "profile_source": self.profile.get("resolved_from"),
            "kind": self.profile["kind"],
            "transport": self.profile["transport"],
            "timestamp_mode": self.profile["timestamp_mode"],
            "timestamp_provenance": self.profile.get("timestamp_provenance", {}),
            "sensor_set": self.profile.get("sensor_set", "unknown"),
            "network_set": self.profile.get("network_set", "unknown"),
            "morai_msgs_branch": self.profile.get(
                "morai_msgs_branch", "26.R1 @ 4c9be6f"
            ),
            "map": self.map_slug,
            "scenario": self.scenario,
            "controller": self.controller,
            "started_at_utc": utc_now_iso(),
            "environment_initial": environment,
            "naming_environment": self.naming_environment,
            "git": git_metadata(self.repo_root),
            "ros": {
                "use_sim_time": bool(rospy.get_param("/use_sim_time", False)),
                "topics": self.topic_types,
            },
            "bridge_configuration": bridge_configuration,
            "bridge_configuration_source": bridge_configuration_source,
            "replay_configuration": (
                rospy.get_param("/vip3_bag_replay", {})
                if self.profile["timestamp_mode"] == "replay" else None
            ),
            "recording": {
                "compression": self.profile["compression"],
                "split_gb": self.split_gb,
                "buffer_mb": self.buffer_mb,
                "command": self.command,
            },
        }

    def run(self):
        rospy.loginfo(
            "rosbag recorder ready: profile=%s run=%s transport=%s root=%s",
            self.profile_name, self.run_id, self.profile["transport"], self.run_root
        )
        try:
            self.process = subprocess.Popen(self.command, start_new_session=True)
            while not rospy.is_shutdown():
                return_code = self.process.poll()
                if return_code is not None:
                    if not self._requested_stop:
                        rospy.logerr(
                            "rosbag record exited unexpectedly: rc=%d", return_code
                        )
                    break
                time.sleep(0.2)
        finally:
            self._finalize()

    def request_stop(self):
        with self._stop_lock:
            if self._requested_stop:
                return
            self._requested_stop = True
        if self.process is not None and self.process.poll() is None:
            try:
                os.killpg(self.process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass

    def _finalize(self):
        with self._stop_lock:
            if self._finalized:
                return
            self._finalized = True
        return_code = None
        if self.process is not None:
            if self.process.poll() is None:
                try:
                    if not self._requested_stop:
                        self._requested_stop = True
                        os.killpg(self.process.pid, signal.SIGINT)
                    return_code = self.process.wait(timeout=30.0)
                except subprocess.TimeoutExpired:
                    os.killpg(self.process.pid, signal.SIGTERM)
                    try:
                        return_code = self.process.wait(timeout=5.0)
                    except subprocess.TimeoutExpired:
                        os.killpg(self.process.pid, signal.SIGKILL)
                        return_code = self.process.wait()
            else:
                return_code = self.process.returncode
        topic_stats = self.counter.summary() if self.counter is not None else {}
        bridge_diagnostics = self.counter.diagnostics() if self.counter is not None else {}
        if self.counter is not None:
            self.counter.close()

        bag_files = sorted(glob.glob(os.path.join(self.bags_dir, "*.bag")))
        active_files = sorted(glob.glob(os.path.join(self.bags_dir, "*.active")))
        bag_info = [
            {
                "file": os.path.basename(path),
                "bytes": os.path.getsize(path),
                "sha256": sha256_file(path),
            }
            for path in bag_files
        ]
        status = (
            "complete"
            if self._requested_stop
            and return_code in (0, None)
            and bag_files
            and not active_files
            else "interrupted"
        )
        self.metadata.update(
            {
                "status": status,
                "closed_at_utc": utc_now_iso(),
                "rosbag_return_code": return_code,
                "topic_statistics": topic_stats,
                "diagnostics_final": bridge_diagnostics,
                "bags": bag_info,
                "active_files": [os.path.basename(path) for path in active_files],
            }
        )
        write_yaml_atomic(self.metadata_path, self.metadata)
        rospy.loginfo("rosbag run closed: status=%s root=%s", status, self.run_root)


def main():
    rospy.init_node("vip3_bag_recorder")
    try:
        BagRecorderNode().run()
    except (BagProfileError, OSError, ValueError) as exc:
        rospy.logfatal("bag recorder could not start: %s", exc)
        raise SystemExit(2)


if __name__ == "__main__":
    main()
