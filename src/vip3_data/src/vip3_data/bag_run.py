# -*- coding: utf-8 -*-
"""run 이름·메타데이터·rosbag 커맨드 생성용 순수 헬퍼 (ROS 의존 없음)."""

from __future__ import print_function

import hashlib
import os
import re
from datetime import datetime, timedelta, timezone


SAFE_SLUG = re.compile(r"[^a-z0-9_]+")
KST = timezone(timedelta(hours=9))


def slug(value, fallback="unknown", max_length=40):
    text = str(value or "").strip().lower().replace("-", "_").replace(" ", "_")
    text = SAFE_SLUG.sub("_", text).strip("_")
    return (text or fallback)[:max_length]


def sim_time_slug(hour):
    try:
        hour = int(hour)
    except (TypeError, ValueError):
        return "unknown"
    if not 0 <= hour <= 23:
        return "unknown"
    suffix = "am" if hour < 12 else "pm"
    display = hour % 12 or 12
    return "{}{}".format(display, suffix)


def make_bag_run_id(map_name, scenario, weather, sim_hour, controller, now=None):
    now = now or datetime.now(KST)
    if now.tzinfo is None:
        now = now.replace(tzinfo=KST)
    return "{}_{}_{}_{}_{}_{}".format(
        now.astimezone(KST).strftime("%Y%m%d_%H%M%S"),
        slug(map_name, fallback="map"),
        slug(scenario, fallback="scenario"),
        slug(weather),
        sim_time_slug(sim_hour),
        slug(controller, fallback="controller"),
    )


def build_rosbag_command(profile, output_prefix, topics, split_gb=4, buffer_mb=1024):
    if not topics:
        raise ValueError("at least one topic is required")
    split_mb = int(float(split_gb) * 1024)
    if split_mb <= 0 or int(buffer_mb) <= 0:
        raise ValueError("split_gb and buffer_mb must be positive")
    command = [
        "rosbag", "record",
        "--buffsize={}".format(int(buffer_mb)),
        "--split", "--size={}".format(split_mb),
        "-O", output_prefix,
    ]
    if profile.get("compression") == "lz4":
        command.append("--lz4")
    command.extend(topics)
    return command


def sha256_file(path, chunk_size=4 << 20):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        while True:
            block = stream.read(chunk_size)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def default_rosbag_root():
    data_root = os.environ.get("VIP3_DATA", "/data")
    return os.path.join(os.path.abspath(os.path.expanduser(data_root)), "rosbags")
