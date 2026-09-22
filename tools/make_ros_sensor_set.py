#!/usr/bin/env python3
"""MORAI 센서셋 JSON을 UDP 구성에서 ROS(rosbridge) 구성으로 변환한다.

VIP3는 UDP를 쓰지 않는다. MORAI Sensor 설정을 전부 ROS로 바꾸고 토픽·frame_id를
팀 규약으로 고정하기 위해 원본 센서셋을 입력으로 받아 ROS 판을 생성한다.

사용법:
    python3 tools/make_ros_sensor_set.py \
        --input  "../data/VIP3_sensor_set_v1.json" \
        --output "config/VIP3_sensor_set_v1_ros.json"

주의: `--comm-type`(ROS를 가리키는 MORAI enum 값)은 시뮬레이터 UI에서 한 번
확인한다. 네트워크 설정(VIP3_network_v1.json)의 ROS 항목이 netType/commType=3을
쓰므로 기본값도 3이다. UI에서 ROS로 저장한 센서셋과 값이 다르면 그 값을 넘긴다.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


# m_SensorUniqueID -> (역할, MORAI Topic 필드, frame_id)
# 카메라 Topic 은 MORAI 가 `/compressed` 를 붙여 CompressedImage 로 발행한다.
CAMERA_PLAN = {
    1: ("front", "/cam_front/image_jpeg", "cam_front"),
    2: ("left", "/cam_left/image_jpeg", "cam_left"),
    3: ("right", "/cam_right/image_jpeg", "cam_right"),
    4: ("rear", "/cam_rear/image_jpeg", "cam_rear"),
}

# 리스트 이름 -> (설정 키, 기본 Topic, 기본 frame_id)
OTHER_PLAN = {
    "Lidar3DList": ("lc", "/velodyne_points", "velodyne"),
    "GPSList": ("gc", "/gps", "gps"),
    "IMUList": ("ic", "/imu", "imu"),
}

CONFIG_KEYS = ("cc", "lc", "gc", "ic", "rc", "uc", "gtc", "dc")


def _config_block(sensor: dict) -> tuple[str, dict]:
    for key in CONFIG_KEYS:
        value = sensor.get(key)
        if isinstance(value, dict):
            return key, value
    raise KeyError(f"센서 설정 블록을 찾지 못했다: {sorted(sensor)}")


def convert(document: dict, comm_type: int, bridge_ip: str, bridge_port: int) -> dict:
    url = f"ws://{bridge_ip}:{bridge_port}"
    changed = []

    for sensor in document.get("cameraList", []):
        sensor_id = int(sensor["m_SensorUniqueID"])
        if sensor_id not in CAMERA_PLAN:
            raise KeyError(f"카메라 {sensor_id} 의 토픽 계획이 없다")
        role, topic, frame_id = CAMERA_PLAN[sensor_id]
        _, block = _config_block(sensor)
        block["commType"] = comm_type
        ros = block["rosConfig"]
        ros["Topic"] = topic
        ros["frameID"] = frame_id
        ros["RosMessageOnOff"] = True
        ros["RosBridgeServerUrl"] = url
        ros["RosBridgeIP"] = bridge_ip
        ros["RosBridgePort"] = bridge_port
        block["sensorName"] = f"cam_{role}"
        changed.append(f"camera {sensor_id} {role} -> {topic} ({frame_id})")

    for list_name, (_, topic, frame_id) in OTHER_PLAN.items():
        for sensor in document.get(list_name, []):
            sensor_id = int(sensor["m_SensorUniqueID"])
            _, block = _config_block(sensor)
            block["commType"] = comm_type
            ros = block["rosConfig"]
            ros["Topic"] = topic
            ros["frameID"] = frame_id
            ros["RosMessageOnOff"] = True
            ros["RosBridgeServerUrl"] = url
            ros["RosBridgeIP"] = bridge_ip
            ros["RosBridgePort"] = bridge_port
            changed.append(f"{list_name} {sensor_id} -> {topic} ({frame_id})")

    for line in changed:
        print("  " + line)
    return document


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--comm-type", type=int, default=3)
    parser.add_argument("--bridge-ip", default="127.0.0.1")
    parser.add_argument("--bridge-port", type=int, default=9090)
    arguments = parser.parse_args()

    document = json.loads(arguments.input.read_text(encoding="utf-8"))
    print(f"입력: {arguments.input}")
    converted = convert(
        document, arguments.comm_type, arguments.bridge_ip, arguments.bridge_port
    )
    payload = json.dumps(converted, ensure_ascii=False, indent=2) + "\n"
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text(payload, encoding="utf-8")
    digest = hashlib.sha256(payload.encode("utf-8")).hexdigest()
    print(f"출력: {arguments.output}")
    print(f"sha256: {digest}")


if __name__ == "__main__":
    main()
