#!/usr/bin/env python3
"""센서셋 JSON 에서 static_transform_publisher 인자를 계산해 출력한다.

센서 위치를 바꾸면 `src/vip3_bringup/launch/static_tf.launch` 의 숫자도 같이 바꿔야 한다.
손으로 고치면 반드시 틀리므로 이 스크립트 출력을 붙여넣는다.

    python3 tools/dump_static_tf.py --sensor-set config/VIP3_sensor_set_v1_ros.json

주의: static_transform_publisher 의 args 는 `x y z yaw pitch roll` [rad] 순서다.
센서셋의 rot 은 `roll pitch yaw` [deg] 라 순서가 반대다. 여기서 자주 틀린다.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

CAMERA_FRAMES = {1: "cam_front", 2: "cam_left", 3: "cam_right", 4: "cam_rear"}
OTHER_FRAMES = {"Lidar3DList": "velodyne", "GPSList": "gps", "IMUList": "imu"}


def emit(frame, position, rotation_deg, note):
    x, y, z = position
    roll, pitch, yaw = rotation_deg
    yaw = ((yaw + 180.0) % 360.0) - 180.0
    print('  <node pkg="tf2_ros" type="static_transform_publisher" '
          'name="tf_{}"'.format(frame))
    print('        args="{:.3f} {:.3f} {:.3f} {:.6f} {:.6f} {:.6f} '
          'base_link {}"/>'.format(
              x, y, z,
              math.radians(yaw), math.radians(pitch), math.radians(roll), frame))
    if note:
        print("  <!-- {} -->".format(note))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--sensor-set", type=Path, default=Path("config/VIP3_sensor_set_v1_ros.json")
    )
    arguments = parser.parse_args()
    document = json.loads(arguments.sensor_set.read_text(encoding="utf-8"))

    for sensor in document.get("cameraList", []):
        sensor_id = int(sensor["m_SensorUniqueID"])
        frame = CAMERA_FRAMES.get(sensor_id, "cam_{}".format(sensor_id))
        position = tuple(float(sensor["pos"][k]) for k in ("x", "y", "z"))
        rotation = tuple(
            float(sensor["rot"][k]) for k in ("roll", "pitch", "yaw")
        )
        emit(frame, position, rotation,
             "sensor {} rot(roll {} pitch {} yaw {}) deg".format(sensor_id, *rotation))

    for list_name, frame in OTHER_FRAMES.items():
        for sensor in document.get(list_name, []):
            position = tuple(float(sensor["pos"][k]) for k in ("x", "y", "z"))
            rotation = tuple(
                float(sensor["rot"][k]) for k in ("roll", "pitch", "yaw")
            )
            emit(frame, position, rotation, "sensor {}".format(
                sensor["m_SensorUniqueID"]))


if __name__ == "__main__":
    main()
