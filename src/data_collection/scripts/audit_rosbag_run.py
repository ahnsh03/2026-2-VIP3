#!/usr/bin/env python3
"""기록한 bag 의 토픽 rate, header stamp 단조성, RGB-Semantic 시각 정합을 감사한다.

ASMC 원본은 Instance LiDAR(`/velodyne_points_instance`) 스키마 검사가 큰 비중이었다.
VIP3 센서셋에는 Instance LiDAR 가 없으므로 그 부분은 전부 걷어냈다.

카메라 목표 rate 는 `--camera-min-hz` 로 준다. 기본 8 Hz 인 이유는 rosbridge 가
20 Hz x 4대를 못 버틸 가능성이 높아서다 (docs/simulator.md §5). 실측한 뒤 올린다.
"""

from __future__ import print_function

import argparse
import bisect
import glob
import json
import math
import os
import sys

import rosbag


# RGB <-> Semantic 시각 정합 검사. semantic 카메라는 강도균 파트가 붙인 뒤에만 존재하므로
# 한쪽이 없으면 그 쌍은 조용히 건너뛴다 (semantic_gt bag 에서만 검사된다).
# 토픽 이름은 config/vip3_topics.yaml 과 bag_profiles/semantic_gt.yaml 을 따른다.
VIEWS = ("front", "left", "right", "rear")
PAIR_LIMITS_MS = {
    "rgb_sem_{}".format(view): (
        "/cam_{}/image_jpeg/compressed".format(view),
        "/sem_{}/image".format(view),
        30.0,
    )
    for view in VIEWS
}
CAMERA_TOPICS = tuple(
    "/cam_{}/image_jpeg/compressed".format(view) for view in VIEWS
) + tuple("/sem_{}/image".format(view) for view in VIEWS)


def percentile(values, percent):
    if not values:
        return None
    ordered = sorted(values)
    index = (len(ordered) - 1) * float(percent) / 100.0
    lower = int(math.floor(index))
    upper = int(math.ceil(index))
    if lower == upper:
        return ordered[lower]
    fraction = index - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def resolve_bags(path):
    path = os.path.abspath(os.path.expanduser(path))
    if os.path.isfile(path):
        return [path]
    candidates = sorted(glob.glob(os.path.join(path, "bags", "*.bag")))
    if not candidates:
        candidates = sorted(glob.glob(os.path.join(path, "*.bag")))
    if not candidates:
        raise OSError("no closed .bag files found under {}".format(path))
    return candidates


def nearest_gaps_ms(left, right):
    if not left or not right:
        return []
    right = sorted(right)
    values = []
    for stamp in sorted(left):
        index = bisect.bisect_left(right, stamp)
        options = []
        if index < len(right):
            options.append(abs(right[index] - stamp))
        if index:
            options.append(abs(right[index - 1] - stamp))
        values.append(min(options) * 1000.0)
    return values


def audit(paths, camera_min_hz=8.0):
    bag_stamps = {}
    header_stamps = {}
    header_regressions = {}
    message_counts = {}
    first_bag_time = None
    last_bag_time = None
    object_ids = {"npc": set(), "pedestrian": set(), "obstacle": set()}

    for path in paths:
        with rosbag.Bag(path, "r") as bag:
            for topic, message, bag_time in bag.read_messages():
                current_bag = bag_time.to_sec()
                first_bag_time = current_bag if first_bag_time is None else min(first_bag_time, current_bag)
                last_bag_time = current_bag if last_bag_time is None else max(last_bag_time, current_bag)
                message_counts[topic] = message_counts.get(topic, 0) + 1
                bag_stamps.setdefault(topic, []).append(current_bag)
                if hasattr(message, "header") and message.header.stamp.to_nsec() > 0:
                    source = message.header.stamp.to_sec()
                    values = header_stamps.setdefault(topic, [])
                    if values and source < values[-1]:
                        header_regressions[topic] = header_regressions.get(topic, 0) + 1
                    values.append(source)
                if topic == "/Object_topic":
                    for key, attribute in (
                        ("npc", "npc_list"),
                        ("pedestrian", "pedestrian_list"),
                        ("obstacle", "obstacle_list"),
                    ):
                        for item in getattr(message, attribute, []):
                            value = int(getattr(item, "unique_id", 0))
                            if value:
                                object_ids[key].add(value)

    duration = max(0.0, (last_bag_time or 0.0) - (first_bag_time or 0.0))
    topics = {}
    for topic, count in sorted(message_counts.items()):
        times = bag_stamps[topic]
        observed = times[-1] - times[0] if len(times) > 1 else 0.0
        rate = (len(times) - 1) / observed if observed > 0 else 0.0
        source = header_stamps.get(topic, [])
        ages = []
        if len(source) == len(times):
            ages = [(bag_value - source_value) * 1000.0
                    for bag_value, source_value in zip(times, source)]
        topics[topic] = {
            "messages": count,
            "average_hz": round(rate, 3),
            "header_regressions": header_regressions.get(topic, 0),
            "bag_minus_header_ms_p50": None if not ages else round(percentile(ages, 50), 3),
            "bag_minus_header_ms_p95": None if not ages else round(percentile(ages, 95), 3),
        }

    pairs = {}
    for name, (left, right, limit) in PAIR_LIMITS_MS.items():
        if left not in header_stamps or right not in header_stamps:
            continue
        gaps = nearest_gaps_ms(header_stamps.get(left, []), header_stamps.get(right, []))
        p95 = percentile(gaps, 95)
        pairs[name] = {
            "left": left,
            "right": right,
            "samples": len(gaps),
            "gap_ms_p50": None if not gaps else round(percentile(gaps, 50), 3),
            "gap_ms_p95": None if p95 is None else round(p95, 3),
            "gap_ms_max": None if not gaps else round(max(gaps), 3),
            "limit_ms_p95": limit,
            "ok": p95 is not None and p95 <= limit,
        }

    checks = {
        "has_messages": bool(topics),
        "header_stamps_monotonic": all(
            value == 0 for value in header_regressions.values()
        ),
    }
    present_cameras = [name for name in CAMERA_TOPICS if name in topics]
    if present_cameras:
        checks["camera_rates_at_least_{:g}hz".format(camera_min_hz)] = all(
            topics[name]["average_hz"] >= camera_min_hz for name in present_cameras
        )
        # 뷰 하나만 크게 느리면 그 카메라만 문제라는 뜻이다. 평균에 묻히지 않게 따로 본다.
        rgb_present = [n for n in present_cameras if n.startswith("/cam_")]
        if len(rgb_present) > 1:
            rates = [topics[n]["average_hz"] for n in rgb_present]
            checks["camera_rate_spread_within_25pct"] = (
                max(rates) > 0 and (max(rates) - min(rates)) / max(rates) <= 0.25
            )
    if "/velodyne_points" in topics:
        checks["lidar_rate_at_least_7hz"] = (
            topics["/velodyne_points"]["average_hz"] >= 7.0
        )
    for name, value in pairs.items():
        if value["samples"]:
            checks[name + "_p95"] = value["ok"]

    return {
        "schema_version": "vip3-rosbag-audit-1.0.0",
        "bag_files": [os.path.basename(path) for path in paths],
        "duration_seconds": round(duration, 3),
        "topics": topics,
        "pairing": pairs,
        # /Object_topic 의 id 목록. 주차칸 점유 라벨링에서 어떤 NPC 가 등장했는지
        # 되짚을 때 쓴다 (simulator_gt 프로파일에서만 채워진다).
        "object_topic_ids": (
            dict((key, sorted(values)) for key, values in object_ids.items())
            if any(object_ids.values()) else None
        ),
        "checks": checks,
        "ok": bool(checks) and all(checks.values()),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_path", help="raw run directory, bags directory, or .bag file")
    parser.add_argument("--json-out", help="write the report outside the immutable raw run")
    parser.add_argument("--strict", action="store_true", help="exit 2 when any applicable gate fails")
    parser.add_argument(
        "--camera-min-hz", type=float, default=8.0,
        help="카메라 rate 게이트 (기본 8). rosbridge 실측 뒤 올린다 — docs/simulator.md 5절",
    )
    args = parser.parse_args()
    try:
        report = audit(resolve_bags(args.bag_path), camera_min_hz=args.camera_min_hz)
    except (OSError, rosbag.bag.ROSBagException) as exc:
        raise SystemExit(str(exc))
    output = json.dumps(report, indent=2, ensure_ascii=False, sort_keys=True) + "\n"
    if args.json_out:
        path = os.path.abspath(os.path.expanduser(args.json_out))
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as stream:
            stream.write(output)
    sys.stdout.write(output)
    return 0 if report["ok"] or not args.strict else 2


if __name__ == "__main__":
    sys.exit(main())
