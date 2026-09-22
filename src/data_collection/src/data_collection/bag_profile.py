# -*- coding: utf-8 -*-
"""rosbag 프로파일 로더 (엄격 검증).

ASMC 의 privileged_gt / deployment_allowed 게이트는 대회 규정 장치라 VIP3 에서는
삭제했다. VIP3 는 /Object_topic 같은 시뮬 GT 를 주차칸 점유 라벨링에 쓸 계획이므로
"기록 금지" 목록 자체가 없다. (최종 시연에서 GT 사용이 허용되는지는 팀이 확인할 것.)
"""

from __future__ import print_function

import copy
import os

import yaml


class BagProfileError(ValueError):
    pass


def _merge(base, overlay):
    result = copy.deepcopy(base)
    for key, value in (overlay or {}).items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def _profile_path(profile_dir, name):
    safe = str(name or "").strip()
    if not safe or os.path.basename(safe) != safe:
        raise BagProfileError("profile must be a simple file stem")
    if safe.endswith(".yaml"):
        safe = safe[:-5]
    return os.path.join(profile_dir, safe + ".yaml")


def load_bag_profile(profile_dir, name, _seen=None):
    """Load one profile, resolving a local ``extends`` chain."""
    profile_dir = os.path.abspath(profile_dir)
    path = _profile_path(profile_dir, name)
    if not os.path.isfile(path):
        raise BagProfileError("bag profile not found: {}".format(path))
    seen = set(_seen or ())
    stem = os.path.splitext(os.path.basename(path))[0]
    if stem in seen:
        raise BagProfileError("cyclic bag profile extends: {}".format(stem))
    seen.add(stem)
    with open(path, "r") as stream:
        raw = yaml.safe_load(stream) or {}
    child_has_name = "name" in raw
    parent = raw.pop("extends", None)
    if parent:
        raw = _merge(load_bag_profile(profile_dir, parent, seen), raw)
    if not child_has_name:
        raw["name"] = stem
    raw["resolved_from"] = path
    validate_bag_profile(raw)
    return raw


def _topic_list(profile, key):
    value = profile.get(key) or []
    if not isinstance(value, list):
        raise BagProfileError("{} must be a list".format(key))
    topics = [str(item).strip() for item in value]
    if any(not item.startswith("/") for item in topics):
        raise BagProfileError("all {} entries must be absolute ROS topics".format(key))
    if len(topics) != len(set(topics)):
        raise BagProfileError("{} contains duplicate topics".format(key))
    return topics


def validate_bag_profile(profile):
    required = _topic_list(profile, "required_topics")
    optional = _topic_list(profile, "optional_topics")
    overlap = set(required).intersection(optional)
    if overlap:
        raise BagProfileError(
            "topics cannot be both required and optional: {}".format(sorted(overlap))
        )
    kind = str(profile.get("kind", "raw"))
    if kind not in ("raw", "result"):
        raise BagProfileError("kind must be raw or result")
    # 'udp' 는 일부러 뺐다. VIP3 전송은 rosbridge 하나뿐이고, ASMC 에서 복붙한
    # 프로파일이 남아 있으면 조용히 도는 대신 여기서 터져야 한다.
    transport = str(profile.get("transport", "rosbridge"))
    if transport not in ("rosbridge", "replay"):
        raise BagProfileError("transport must be rosbridge or replay")
    # rosbridge 에서는 모든 메시지가 websocket 수신 시각으로 찍힌다. 'source'(UDP
    # 패킷 소스타임) 경로는 존재하지 않는다.
    timestamp_mode = str(profile.get("timestamp_mode", "receive"))
    if timestamp_mode not in ("receive", "replay"):
        raise BagProfileError("timestamp_mode must be receive or replay")

    expected = profile.get("expected_types") or {}
    unknown_expected = set(expected).difference(required + optional)
    if unknown_expected:
        raise BagProfileError(
            "expected_types contains unlisted topics: {}".format(
                sorted(unknown_expected)
            )
        )

    profile["required_topics"] = required
    profile["optional_topics"] = optional
    profile["kind"] = kind
    profile["transport"] = transport
    profile["timestamp_mode"] = timestamp_mode
    profile["compression"] = str(profile.get("compression", "none"))
    if profile["compression"] not in ("none", "lz4"):
        raise BagProfileError("compression must be none or lz4")
    return profile
