#!/usr/bin/env python3
"""저장소의 ROS 토픽 문자열이 `config/vip3_topics.yaml` 계약과 맞는지 검사한다.

ASMC 에서 이식할 때 가장 조용히 틀리는 것이 토픽 이름이다. 노드는 멀쩡히 뜨고
구독만 안 돼서 시뮬레이터를 켜기 전에는 알 수가 없다.

    python3 tools/check_topic_contract.py          # 문제 있으면 exit 1
    python3 tools/check_topic_contract.py --list   # 발견한 MORAI 토픽 전부 나열

검사 대상
  1. **금지 토픽** — ASMC 잔재 (`/image_jpeg/compressed`, `/cam_extra`, `/lidar3D`, …)
  2. 계약에 없는 MORAI 토픽 — 오타이거나 배선이 계약을 앞질렀다는 뜻
  3. 계약에 있는데 코드에서 안 쓰는 토픽 (경고만)

설계 메모
  - MORAI 센서셋/네트워크 JSON 은 정규식이 아니라 **구조로 파싱**한다. 정규식으로 긁으면
    `ros2NativeConfig` 같은 안 쓰는 블록까지 걸려 오탐이 난다.
  - 센서셋의 카메라 `Topic` 은 **`/compressed` 가 빠진 base 이름**이다. MORAI 가 붙인다.
    그래서 두 형태를 모두 허용한다.
  - `/perception/...` 같은 팀 내부 토픽은 MORAI 계약 밖이라 검사하지 않는다.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
CONTRACT = REPO_ROOT / "config" / "vip3_topics.yaml"
SELF = Path(__file__).resolve()

SEARCH_DIRS = ("src", "scripts", "tools")
SEARCH_SUFFIXES = (".py", ".yaml", ".yml", ".launch", ".sh", ".cpp", ".h")
SKIP_PARTS = ("morai_msgs", "__pycache__", ".git", "weights")

TOPIC_PATTERN = re.compile(r"[\"'](/[A-Za-z][A-Za-z0-9_/]*)[\"']")

MORAI_PREFIXES = ("/cam_", "/sem_", "/velodyne", "/gps", "/imu", "/lidar")
MORAI_EXACT = {
    "/Ego_topic", "/Object_topic", "/CollisionData", "/tf", "/tf_static",
    "/ctrl_cmd", "/SaveSensorData", "/ScenarioLoad", "/lamps",
    "/GetTrafficLightStatus", "/IntscnTL_topic", "/InsnStatus", "/SyncModeInfo",
    "/ReplayInfo_topic", "/SetTrafficLight", "/SensorPosControl", "/InsnControl",
    "/Service_MoraiEventCmd", "/Service_MoraiMapSpec",
}
# 토픽이 아니라 rosparam / 노드 이름. 검사 대상이 아니다.
NOT_TOPICS = {"/use_sim_time", "/rosbridge_websocket", "/clock", "/rosout"}

FORBIDDEN = {
    "/image_jpeg/compressed": "ASMC 전방 카메라. VIP3 는 /cam_front/image_jpeg/compressed",
    "/cam_extra/image_jpeg/compressed": "ASMC 하향 카메라. VIP3 4번은 후방이다",
    "/velodyne_points_instance": "Instance LiDAR GT. VIP3 센서셋에 없다",
    "/Competition_topic": "대회 전용",
    "/lidar3D": "MORAI 기본 이름. VIP3 는 /velodyne_points 로 통일",
}


def contract_topics() -> set:
    document = yaml.safe_load(CONTRACT.read_text(encoding="utf-8"))
    topics = set()
    for camera in (document.get("cameras") or {}).values():
        topic = camera["topic"]
        topics.add(topic)
        # 센서셋 JSON 에는 MORAI 가 /compressed 를 붙이기 전의 base 이름이 들어간다.
        if topic.endswith("/compressed"):
            topics.add(topic[: -len("/compressed")])
    for sensor in (document.get("sensors") or {}).values():
        topics.add(sensor["topic"])
    network = document.get("network") or {}
    for group in ("subscribe", "publish", "missing"):
        for entry in (network.get(group) or {}).values():
            topics.add(entry["topic"])
    for entry in (network.get("services") or {}).values():
        topics.add(entry["name"])
    # semantic 카메라는 센서셋에 아직 없지만 bag 프로파일이 이름을 예약해 뒀다.
    for view in (document.get("cameras") or {}):
        topics.add("/sem_{}/image".format(view))
    topics.update({"/tf", "/tf_static"})
    return topics


def network_preset_topics() -> set:
    """VIP3_network_v1.json 에 실제로 설정된 토픽. MORAI 가 아는 이름이므로 허용한다."""
    path = REPO_ROOT / "config" / "VIP3_network_v1.json"
    if not path.is_file():
        return set()
    document = json.loads(path.read_text(encoding="utf-8"))
    topics = set()
    for ego in document.get("egoNetworkData", []):
        for info in ego.get("listEgoNetworkInfo", []):
            topic = ((info.get("config") or {}).get("rosConfig") or {}).get("Topic", "")
            if topic.startswith("/") and topic != "/dafault_topic":
                topics.add(topic)
    return topics


def is_morai_topic(topic: str) -> bool:
    if topic in NOT_TOPICS:
        return False
    if topic in MORAI_EXACT:
        return True
    for prefix in MORAI_PREFIXES:
        # 접두어 그 자체(코드의 f-string 조각)는 토픽이 아니다.
        if topic.startswith(prefix) and len(topic) > len(prefix) + 1:
            return True
    return False


def scan_code() -> dict:
    found = {}
    for directory in SEARCH_DIRS:
        base = REPO_ROOT / directory
        if not base.is_dir():
            continue
        for path in base.rglob("*"):
            if path.suffix not in SEARCH_SUFFIXES or not path.is_file():
                continue
            if any(part in SKIP_PARTS for part in path.parts):
                continue
            if path.resolve() == SELF:        # 자기 자신은 건너뛴다
                continue
            try:
                text = path.read_text(encoding="utf-8")
            except (UnicodeDecodeError, OSError):
                continue
            for match in TOPIC_PATTERN.finditer(text):
                topic = match.group(1)
                if "." in topic.rsplit("/", 1)[-1]:
                    continue
                found.setdefault(topic, set()).add(str(path.relative_to(REPO_ROOT)))
    return found


def scan_sensor_sets() -> dict:
    """센서셋 JSON 의 rosConfig.Topic 만 구조로 읽는다 (ros2NativeConfig 는 무시)."""
    found = {}
    for path in sorted((REPO_ROOT / "config").glob("VIP3_sensor_set_*.json")):
        document = json.loads(path.read_text(encoding="utf-8"))
        for key in ("cameraList", "Lidar3DList", "GPSList", "IMUList"):
            for sensor in document.get(key, []):
                for block in sensor.values():
                    if not isinstance(block, dict):
                        continue
                    topic = (block.get("rosConfig") or {}).get("Topic", "")
                    if topic.startswith("/") and topic != "/dafault_topic":
                        found.setdefault(topic, set()).add(
                            str(path.relative_to(REPO_ROOT))
                        )
    return found


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--list", action="store_true")
    arguments = parser.parse_args()

    allowed = contract_topics() | network_preset_topics()
    found = scan_code()
    for topic, places in scan_sensor_sets().items():
        found.setdefault(topic, set()).update(places)

    morai_found = {t: p for t, p in found.items() if is_morai_topic(t)}

    if arguments.list:
        print("발견한 MORAI 토픽:")
        for topic in sorted(morai_found):
            print("  {} {:<44} {}개 파일".format(
                "OK" if topic in allowed else "??", topic, len(morai_found[topic])
            ))
        print()

    problems = []
    for topic, reason in FORBIDDEN.items():
        if topic in found:
            for where in sorted(found[topic]):
                problems.append("금지 토픽 {}  ({})  -> {}".format(topic, reason, where))
    for topic, places in sorted(morai_found.items()):
        if topic in allowed or topic in FORBIDDEN:
            continue
        problems.append(
            "계약에 없는 MORAI 토픽 {}  -> {}".format(topic, ", ".join(sorted(places)))
        )

    unused = sorted(
        topic for topic in contract_topics()
        if topic not in found
        and not topic.startswith("/sem_")
        and topic not in {"/tf_static"}
        and not (topic + "/compressed") in found
    )

    print("계약 토픽 {}개 · 네트워크 프리셋 {}개 · 저장소에서 발견한 MORAI 토픽 {}개".format(
        len(contract_topics()), len(network_preset_topics()), len(morai_found)
    ))
    if unused:
        print("\n[참고] 계약에 있는데 코드·설정에서 안 쓰는 토픽 — 배선이 빠졌을 수 있다:")
        for topic in unused:
            print("  - {}".format(topic))

    if problems:
        print("\n문제 {}건:".format(len(problems)))
        for problem in problems:
            print("  ! {}".format(problem))
        return 1
    print("\n토픽 계약 위반 없음")
    return 0


if __name__ == "__main__":
    sys.exit(main())
