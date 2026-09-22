# -*- coding: utf-8 -*-
"""수집 프로파일 YAML 로더 (+ 환경변수 오버라이드).

토픽 정본은 config/vip3_topics.yaml 이다. 여기 DEFAULT_TOPICS 는 그 사본이며,
프로파일 YAML 의 topics: 블록이 항상 우선한다.
"""

from __future__ import print_function

import copy
import os

import yaml


DEFAULT_TOPICS = {
    "ego": "/Ego_topic",
    "objects": "/Object_topic",
    "collision": "/CollisionData",
    "lidar": "/velodyne_points",
    "gps": "/gps",
    "imu": "/imu",
    "cam_front": "/cam_front/image_jpeg/compressed",
    "cam_left": "/cam_left/image_jpeg/compressed",
    "cam_right": "/cam_right/image_jpeg/compressed",
    "cam_rear": "/cam_rear/image_jpeg/compressed",
}

# 카메라 채널 이름 -> 뷰 이름. 노드가 순회 순서로 쓴다.
CAMERA_CHANNELS = ("cam_front", "cam_left", "cam_right", "cam_rear")


def _deep_merge(base, overlay):
    out = copy.deepcopy(base)
    for k, v in (overlay or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _deep_merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def load_profile(path):
    with open(path, "r") as f:
        raw = yaml.safe_load(f) or {}

    channels = raw.get("channels") or {}
    topics = _deep_merge(DEFAULT_TOPICS, raw.get("topics") or {})

    profile = {
        "name": raw.get("name") or os.path.splitext(os.path.basename(path))[0],
        "description": raw.get("description") or "",
        "collect_hz": float(raw.get("collect_hz", 10.0)),
        "mode": raw.get("mode") or "latest_timer",
        "max_frames": int(raw.get("max_frames") or 0),
        "require_ego": bool(raw.get("require_ego", True)),
        "jpeg_quality_note": raw.get("jpeg_quality_note") or "store CompressedImage bytes as-is",
        "channels": {
            "ego": bool(channels.get("ego", True)),
            "objects": bool(channels.get("objects", False)),
            "collision": bool(channels.get("collision", False)),
            "lidar": bool(channels.get("lidar", False)),
            "gps": bool(channels.get("gps", False)),
            "imu": bool(channels.get("imu", False)),
            "cam_front": bool(channels.get("cam_front", False)),
            "cam_left": bool(channels.get("cam_left", False)),
            "cam_right": bool(channels.get("cam_right", False)),
            "cam_rear": bool(channels.get("cam_rear", False)),
        },
        "topics": topics,
        "lidar_format": raw.get("lidar_format") or "bin_xyzi",
        "map": raw.get("map") or "R_KR_PG_KATRI",
        "vehicle": raw.get("vehicle") or "2023_Hyundai_ioniq5",
        "bridge_type": raw.get("bridge_type") or "rosbridge",
        "notes": raw.get("notes") or "",
    }

    env_profile = os.environ.get("VIP3_COLLECT_PROFILE")
    if env_profile and env_profile != profile["name"]:
        # informational only; launch/arg chooses file
        profile["env_hint"] = env_profile

    enable = os.environ.get("VIP3_COLLECT_ENABLE", "").strip()
    if enable:
        for name in enable.split(","):
            name = name.strip()
            if name in profile["channels"]:
                profile["channels"][name] = True

    disable = os.environ.get("VIP3_COLLECT_DISABLE", "").strip()
    if disable:
        for name in disable.split(","):
            name = name.strip()
            if name in profile["channels"]:
                profile["channels"][name] = False

    return profile
