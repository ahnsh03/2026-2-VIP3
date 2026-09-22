#!/usr/bin/env python3
"""Audit rates, source stamps, GT pairing, and Instance LiDAR schema in bags."""

from __future__ import print_function

import argparse
import bisect
import glob
import json
import math
import os
import sys

import rosbag
from sensor_msgs.msg import PointField


PAIR_LIMITS_MS = {
    "rgb_sem_front": ("/image_jpeg/compressed", "/sem_front/image_jpeg/compressed", 30.0),
    "rgb_sem_left": ("/cam_left/image_jpeg/compressed", "/sem_left/image_jpeg/compressed", 30.0),
    "rgb_sem_right": ("/cam_right/image_jpeg/compressed", "/sem_right/image_jpeg/compressed", 30.0),
    "intensity_instance_lidar": ("/velodyne_points", "/velodyne_points_instance", 50.0),
}


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


def pointcloud_schema(message):
    return dict((field.name, int(field.datatype)) for field in message.fields)


def instance_nonzero_sample(message, max_points=200000):
    field = next((item for item in message.fields if item.name == "instance_id"), None)
    if field is None or field.datatype != PointField.UINT32 or message.point_step <= 0:
        return None
    endian = "big" if message.is_bigendian else "little"
    count = min(int(message.width * message.height), int(max_points))
    data = memoryview(message.data)
    nonzero = 0
    maximum = 0
    unique_nonzero_ids = set()
    for index in range(count):
        start = index * message.point_step + field.offset
        value = int.from_bytes(data[start:start + 4], byteorder=endian, signed=False)
        maximum = max(maximum, value)
        nonzero += int(value != 0)
        if value:
            unique_nonzero_ids.add(value)
    return {
        "sampled_points": count,
        "nonzero_points": nonzero,
        "maximum_id": maximum,
        "unique_nonzero_ids": sorted(unique_nonzero_ids),
    }


def audit(paths):
    bag_stamps = {}
    header_stamps = {}
    header_regressions = {}
    message_counts = {}
    first_bag_time = None
    last_bag_time = None
    instance_schema = None
    instance_samples = {"sampled_points": 0, "nonzero_points": 0, "maximum_id": 0}
    instance_ids = set()
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
                if topic == "/velodyne_points_instance" and instance_schema is None:
                    instance_schema = pointcloud_schema(message)
                if topic == "/velodyne_points_instance":
                    sample = instance_nonzero_sample(message, max_points=20000)
                    if sample:
                        for key in ("sampled_points", "nonzero_points"):
                            instance_samples[key] += sample[key]
                        instance_samples["maximum_id"] = max(
                            instance_samples["maximum_id"], sample["maximum_id"]
                        )
                        instance_ids.update(sample["unique_nonzero_ids"])
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

    schema_ok = (
        instance_schema is not None
        and instance_schema.get("instance_id") == PointField.UINT32
        and "intensity" not in instance_schema
    )
    checks = {
        "has_messages": bool(topics),
        "header_stamps_monotonic": all(
            value == 0 for value in header_regressions.values()
        ),
    }
    camera_topics = (
        "/image_jpeg/compressed", "/cam_left/image_jpeg/compressed",
        "/cam_right/image_jpeg/compressed", "/sem_front/image_jpeg/compressed",
        "/sem_left/image_jpeg/compressed", "/sem_right/image_jpeg/compressed",
    )
    present_cameras = [name for name in camera_topics if name in topics]
    if present_cameras:
        checks["camera_rates_at_least_18hz"] = all(
            topics[name]["average_hz"] >= 18.0 for name in present_cameras
        )
    lidar_topics = ("/velodyne_points", "/velodyne_points_instance")
    if all(name in topics for name in lidar_topics):
        rates = [topics[name]["average_hz"] for name in lidar_topics]
        checks["lidar_rates_at_least_7hz"] = min(rates) >= 7.0
        checks["lidar_rate_difference_within_15pct"] = (
            max(rates) > 0 and (max(rates) - min(rates)) / max(rates) <= 0.15
        )
        checks["instance_lidar_schema"] = schema_ok
        all_object_ids = set().union(*object_ids.values())
        if instance_ids and "/Object_topic" in topics:
            checks["instance_ids_resolved_by_object_topic"] = instance_ids.issubset(
                all_object_ids
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
        "instance_lidar": (
            {
                "fields": instance_schema,
                "schema_ok": schema_ok,
                "sample": dict(
                    instance_samples,
                    unique_nonzero_ids=sorted(instance_ids),
                ),
                "object_topic_ids": dict(
                    (key, sorted(values)) for key, values in object_ids.items()
                ),
                "unresolved_instance_ids": sorted(
                    instance_ids.difference(set().union(*object_ids.values()))
                ),
            }
            if instance_schema is not None else None
        ),
        "checks": checks,
        "ok": bool(checks) and all(checks.values()),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_path", help="raw run directory, bags directory, or .bag file")
    parser.add_argument("--json-out", help="write the report outside the immutable raw run")
    parser.add_argument("--strict", action="store_true", help="exit 2 when any applicable gate fails")
    args = parser.parse_args()
    try:
        report = audit(resolve_bags(args.bag_path))
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
