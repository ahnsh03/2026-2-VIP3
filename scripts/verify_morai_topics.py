#!/usr/bin/env python3
"""MORAI rosbridge 토픽 수신을 세고, 누락·저속 토픽이 있으면 실패로 끝낸다.

기대 토픽/주기 표는 이 파일에 적지 않는다. 정본은 config/vip3_topics.yaml 이고
여기서는 그것을 읽기만 한다 (센서셋이 바뀌면 YAML 한 곳만 고치면 된다).

컨테이너 안에서:
    ./scripts/verify_morai_topics.py --duration 8
ROS 없이 표만 확인할 때:
    ./scripts/verify_morai_topics.py --list
"""

import argparse
import bisect
import json
import math
import os
import sys
import threading
import time
from pathlib import Path

# rate < SLOW_RATIO * nominal 이면 SLOW. rosbridge 포화의 1차 신호다.
SLOW_RATIO = 0.6

# 처리량이 모자랄 때 시도할 순서. 화면과 JSON 리포트에 그대로 찍는다.
ROSBRIDGE_MITIGATION = (
    "1) 컨테이너에 ujson 설치 확인: python3 -c 'import ujson'"
    " (rosbridge_library 가 자동으로 집어 쓴다)",
    "2) 카메라 4대 sensorPeriod 0.05 -> 0.1 (20 Hz -> 10 Hz)."
    " 주차는 저속이라 10 Hz 로 충분하다",
    "3) 전방·후방 해상도 1280x720 -> 640x480 으로 통일"
    " (근거리 BEV 는 해상도보다 시야각이 중요하다)",
    "4) 그래도 모자라면 그 작업에 필요한 카메라만 켠다",
)

ROSBRIDGE_BANDWIDTH_NOTE = (
    "rosbridge 는 CompressedImage 를 base64 JSON 으로 감싸 단일 WebSocket 으로"
    " 나른다. rate 미달은 MORAI 송신 cadence 가 아니라 rosbridge 처리량 포화일"
    " 가능성이 크다. 위 mitigation 을 순서대로 시도한다."
)

# 26.R1 EgoVehicleStatus 스키마 게이트 (beta_drive 로 잘못 빌드한 것을 잡는다)
EGO_REQUIRED_FIELDS = ("angular_velocity", "front_steer_angle", "rear_steer_angle")
EGO_FORBIDDEN_FIELDS = ("wheel_angle",)


def default_topics_config():
    """<repo>/config/vip3_topics.yaml. 컨테이너(/root/ws)와 호스트 clone 모두에서 동작."""
    return Path(__file__).resolve().parent.parent / "config" / "vip3_topics.yaml"


def _hz_from_period(period):
    try:
        period = float(period)
    except (TypeError, ValueError):
        return None
    if period <= 0.0:
        return None
    return round(1.0 / period, 4)


def load_topic_table(config_path):
    """vip3_topics.yaml -> 검증용 토픽 표.

    rospy 없이 동작한다(호스트에서 --list 로 확인 가능).
    반환: [{name, topic, msg_type, nominal_hz, groups}] 순서 고정.
    """
    import yaml  # PyYAML 은 ROS Noetic 의 필수 의존성이라 항상 있다

    with open(str(config_path), "r", encoding="utf-8") as handle:
        doc = yaml.safe_load(handle) or {}

    entries = []

    # 1) Ego 상태 — 없으면 아무것도 검증할 수 없다
    network = doc.get("network") or {}
    subscribe = network.get("subscribe") or {}
    ego = subscribe.get("ego") or {}
    if ego.get("topic"):
        entries.append({
            "name": "ego",
            "topic": ego["topic"],
            "msg_type": ego.get("type", "morai_msgs/EgoVehicleStatus"),
            "nominal_hz": float(ego["hz"]) if ego.get("hz") else None,
            "groups": ("runtime", "full"),
        })

    # 2) 카메라 4대 — 순서를 고정해 출력이 매번 같게 한다
    cameras = doc.get("cameras") or {}
    for view in ("front", "left", "right", "rear"):
        camera = cameras.get(view)
        if not camera or not camera.get("topic"):
            continue
        entries.append({
            "name": "cam_" + view,
            "topic": camera["topic"],
            "msg_type": camera.get("type", "sensor_msgs/CompressedImage"),
            "nominal_hz": _hz_from_period(camera.get("period_s")),
            "groups": ("runtime", "full", "cameras"),
        })

    # 3) LiDAR / GPS / IMU
    sensors = doc.get("sensors") or {}
    for key in ("lidar", "gps", "imu"):
        sensor = sensors.get(key)
        if not sensor or not sensor.get("topic"):
            continue
        entries.append({
            "name": key,
            "topic": sensor["topic"],
            "msg_type": sensor.get("type", ""),
            "nominal_hz": _hz_from_period(sensor.get("period_s")),
            "groups": ("runtime", "full"),
        })

    # 4) 선택 토픽 — 주차에 쓰지만 없어도 파이프라인은 돈다
    for key in ("objects", "collision"):
        item = subscribe.get(key)
        if not item or not item.get("topic"):
            continue
        entries.append({
            "name": key,
            "topic": item["topic"],
            "msg_type": item.get("type", ""),
            "nominal_hz": float(item["hz"]) if item.get("hz") else None,
            "groups": ("full",),
        })

    return entries


def select_topics(table, profile):
    """프로필에 해당하는 표 항목만 순서대로 고른다."""
    return [entry for entry in table if profile in entry["groups"]]


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--min-messages", type=int, default=2)
    parser.add_argument(
        "--profile", choices=("runtime", "full", "cameras"), default="runtime",
        help="runtime=필수 8개 / full=+선택 토픽 / cameras=카메라 4대만",
    )
    parser.add_argument(
        "--topics-config", default=str(default_topics_config()),
        help="토픽·주기 정본 YAML (기본: config/vip3_topics.yaml)",
    )
    parser.add_argument(
        "--required-topic", action="append", dest="required_topics",
        help="표 대신 검증할 토픽을 직접 지정 (반복 가능)",
    )
    parser.add_argument(
        "--list", action="store_true",
        help="ROS 없이 기대 토픽·주기 표만 출력하고 끝낸다",
    )
    parser.add_argument("--json-out")
    return parser.parse_args(argv)


def percentile(values, percent):
    if not values:
        return None
    ordered = sorted(values)
    index = (len(ordered) - 1) * float(percent) / 100.0
    low, high = int(math.floor(index)), int(math.ceil(index))
    if low == high:
        return ordered[low]
    return ordered[low] * (high - index) + ordered[high] * (index - low)


def nearest_p95_ms(left, right):
    if not left or not right:
        return None
    right = sorted(right)
    gaps = []
    for stamp in left:
        index = bisect.bisect_left(right, stamp)
        candidates = []
        if index < len(right):
            candidates.append(abs(stamp - right[index]))
        if index:
            candidates.append(abs(stamp - right[index - 1]))
        gaps.append(min(candidates) * 1000.0)
    return percentile(gaps, 95)


def interval_summary(arrivals):
    """구독자가 본 도착 간격. 네트워크 패킷 손실을 주장하지 않는다."""
    if len(arrivals) < 2:
        return {
            "samples": 0, "p50_ms": None, "p95_ms": None, "max_ms": None,
            "jitter_p95_ms": None, "gap_events": 0,
            "long_gap_equivalent_periods": 0,
        }
    intervals = [
        (right - left) * 1000.0
        for left, right in zip(arrivals[:-1], arrivals[1:])
        if right >= left
    ]
    median = percentile(intervals, 50)
    jitter = [abs(value - median) for value in intervals]
    threshold = median * 1.5 if median and median > 0 else float("inf")
    gap_values = [value for value in intervals if value > threshold]
    long_gap_equivalent_periods = sum(
        max(0, int(round(value / median)) - 1) for value in gap_values
    ) if median and median > 0 else 0
    return {
        "samples": len(intervals),
        "p50_ms": round(median, 3),
        "p95_ms": round(percentile(intervals, 95), 3),
        "max_ms": round(max(intervals), 3),
        "jitter_p95_ms": round(percentile(jitter, 95), 3),
        "gap_events": len(gap_values),
        "long_gap_equivalent_periods": long_gap_equivalent_periods,
    }


def check_ego_schema(message):
    """첫 /Ego_topic 메시지로 morai_msgs 가 26.R1 인지 확인한다."""
    missing = [name for name in EGO_REQUIRED_FIELDS if not hasattr(message, name)]
    legacy = [name for name in EGO_FORBIDDEN_FIELDS if hasattr(message, name)]
    return {
        "expected": "morai_msgs 26.R1 (4c9be6f)",
        "missing_fields": missing,
        "beta_drive_fields_present": legacy,
        "ok": (not missing) and (not legacy),
    }


def print_table(entries):
    print("TOPIC                                             TYPE                              NOMINAL(Hz)")
    for entry in entries:
        print("{:<49} {:<33} {:>10}".format(
            entry["topic"], entry["msg_type"] or "-",
            "-" if entry["nominal_hz"] is None else "{:.2f}".format(entry["nominal_hz"]),
        ))


def main(argv=None):
    args = parse_args(argv)
    if args.duration <= 0:
        raise SystemExit("--duration 은 양수여야 한다")
    if args.min_messages < 1:
        raise SystemExit("--min-messages 는 1 이상이어야 한다")

    config_path = Path(args.topics_config)
    if not config_path.is_file():
        raise SystemExit("토픽 정본을 찾을 수 없다: {}".format(config_path))
    table = load_topic_table(config_path)

    if args.required_topics:
        known = {entry["topic"]: entry for entry in table}
        entries = [
            known.get(topic, {
                "name": topic, "topic": topic, "msg_type": "",
                "nominal_hz": None, "groups": (),
            })
            for topic in args.required_topics
        ]
    else:
        entries = select_topics(table, args.profile)
    if not entries:
        raise SystemExit("검증할 토픽이 없다 — {} 를 확인하라".format(config_path))

    if args.list:
        print_table(entries)
        return 0

    try:
        import rospy
        from morai_msgs.msg import CollisionData, EgoVehicleStatus, GPSMessage, ObjectStatusList
        from sensor_msgs.msg import CompressedImage, Imu, PointCloud2
    except ImportError as exc:
        raise SystemExit(
            "ROS 메시지 패키지가 필요하다 — vip3-ros-noetic 컨테이너 안에서 실행하라 "
            "(source /opt/ros/noetic/setup.bash && source devel/setup.bash)"
        ) from exc

    type_map = {
        "morai_msgs/EgoVehicleStatus": EgoVehicleStatus,
        "morai_msgs/ObjectStatusList": ObjectStatusList,
        "morai_msgs/CollisionData": CollisionData,
        "morai_msgs/GPSMessage": GPSMessage,
        "sensor_msgs/CompressedImage": CompressedImage,
        "sensor_msgs/Imu": Imu,
        "sensor_msgs/PointCloud2": PointCloud2,
    }

    topics = [entry["topic"] for entry in entries]
    nominal = {entry["topic"]: entry["nominal_hz"] for entry in entries}
    lock = threading.Lock()
    samples = {topic: [] for topic in topics}
    source_stamps = {topic: [] for topic in topics}
    header_ages_ms = {topic: [] for topic in topics}
    source_regressions = {topic: 0 for topic in topics}
    ego_schema = {"box": None}
    client_count = {"value": None}
    started = time.monotonic()

    def callback(message, topic_name):
        now = time.monotonic()
        wall_now = time.time()
        with lock:
            samples[topic_name].append(now)
            if hasattr(message, "header") and message.header.stamp.to_nsec() > 0:
                stamp = message.header.stamp.to_sec()
                if source_stamps[topic_name] and stamp < source_stamps[topic_name][-1]:
                    source_regressions[topic_name] += 1
                source_stamps[topic_name].append(stamp)
                age_ms = (wall_now - stamp) * 1000.0
                if 0.0 <= age_ms <= 60000.0:
                    header_ages_ms[topic_name].append(age_ms)
            if ego_schema["box"] is None and isinstance(message, EgoVehicleStatus):
                ego_schema["box"] = check_ego_schema(message)

    def clients_callback(message):
        with lock:
            client_count["value"] = len(getattr(message, "clients", []) or [])

    rospy.init_node("vip3_morai_topic_verifier", anonymous=True, disable_signals=True)
    subscribers = []
    for entry in entries:
        message_type = type_map.get(entry["msg_type"])
        if message_type is None:
            message_type = (
                CompressedImage if entry["topic"].endswith("/compressed")
                else rospy.AnyMsg
            )
        subscribers.append(rospy.Subscriber(
            entry["topic"], message_type, callback,
            callback_args=entry["topic"], queue_size=1,
        ))

    # MORAI 가 실제로 붙었는지 보는 유일한 ROS 신호. UDP 브리지 diagnostics 의 대체물.
    clients_available = True
    try:
        from rosbridge_msgs.msg import ConnectedClients
        subscribers.append(rospy.Subscriber(
            "/connected_clients", ConnectedClients, clients_callback, queue_size=1,
        ))
    except ImportError:
        clients_available = False

    deadline = started + args.duration
    while time.monotonic() < deadline and not rospy.is_shutdown():
        time.sleep(0.05)

    finished = time.monotonic()
    report = {
        "profile": args.profile,
        "topics_config": str(config_path),
        "duration_seconds": round(finished - started, 6),
        "min_messages": args.min_messages,
        "slow_ratio": SLOW_RATIO,
        "topics": {},
    }
    failures = []
    slow_topics = []
    with lock:
        snapshot = {topic: list(values) for topic, values in samples.items()}

    print("TOPIC                                             COUNT   RATE(Hz)  NOM(Hz)   STATUS")
    for topic in topics:
        values = snapshot[topic]
        count = len(values)
        rate = 0.0
        if count >= 2 and values[-1] > values[0]:
            rate = (count - 1) / (values[-1] - values[0])
        present = count >= args.min_messages
        nominal_hz = nominal.get(topic)
        slow = bool(present and nominal_hz and rate < SLOW_RATIO * nominal_hz)
        if not present:
            failures.append(topic)
            status = "MISSING"
        elif slow:
            slow_topics.append(topic)
            status = "SLOW"
        else:
            status = "OK"
        report["topics"][topic] = {
            "count": count,
            "rate_hz": round(rate, 3),
            "nominal_rate_hz": nominal_hz,
            "nominal_rate_ratio": (
                None if not nominal_hz else round(rate / nominal_hz, 4)
            ),
            "arrival_intervals": interval_summary(values),
            "header_age_ms": {
                "samples": len(header_ages_ms[topic]),
                "p50": (
                    None if not header_ages_ms[topic]
                    else round(percentile(header_ages_ms[topic], 50), 3)
                ),
                "p95": (
                    None if not header_ages_ms[topic]
                    else round(percentile(header_ages_ms[topic], 95), 3)
                ),
                "max": (
                    None if not header_ages_ms[topic]
                    else round(max(header_ages_ms[topic]), 3)
                ),
            },
            "source_stamp_regressions": source_regressions[topic],
            "slow": slow,
            "ok": present,
        }
        print("{:<49} {:>5} {:>10.3f} {:>8}   {}".format(
            topic, count, rate,
            "-" if nominal_hz is None else "{:.1f}".format(nominal_hz),
            status,
        ))

    for subscriber in subscribers:
        subscriber.unregister()

    # rosbridge 클라이언트(= MORAI) 연결 여부
    with lock:
        clients = client_count["value"]
    report["rosbridge_clients"] = {
        "available": clients_available,
        "count": clients,
        "ok": (clients is None) if not clients_available else bool(clients and clients >= 1),
    }
    if clients_available and not report["rosbridge_clients"]["ok"]:
        failures.append("/connected_clients:no_morai_client")

    # morai_msgs 26.R1 스키마 게이트
    schema = ego_schema["box"]
    if schema is not None:
        report["morai_msgs_schema"] = schema
        if not schema["ok"]:
            failures.append("morai_msgs:not_26r1")
            print("[FAIL] morai_msgs 가 26.R1 이 아니다 — "
                  "git -C src/morai_msgs checkout 4c9be6f && ./scripts/build_ws.sh")
    else:
        report["morai_msgs_schema"] = {
            "expected": "morai_msgs 26.R1 (4c9be6f)",
            "ok": None,
            "reason": "EgoVehicleStatus 메시지를 한 건도 받지 못해 확인 불가",
        }

    report["slow_topics"] = slow_topics
    report["missing_topics"] = [name for name in failures if name.startswith("/")]
    report["ok"] = not failures
    report["interpretation"] = {
        "rosbridge_bandwidth": ROSBRIDGE_BANDWIDTH_NOTE,
        "mitigation_order": list(ROSBRIDGE_MITIGATION),
        "nominal_rate_shortfall": (
            "설정 주기 미달 자체가 센서 고장의 증거는 아니다. rosbridge 직렬화"
            " 포화와 WebSocket 지연을 먼저 의심한다"
        ),
        "long_gap_equivalent_periods": (
            "구독자가 본 긴 공백을 중앙 주기 단위로 환산한 값. jitter/burst 의"
            " 증거이지 드랍 프레임 수가 아니다"
        ),
        "header_age_ms": "ROS publish -> subscriber 도착 지연(헤더 stamp 기준)",
    }

    if slow_topics:
        print("")
        print("[SLOW] rate < {:.0%} x nominal: {}".format(SLOW_RATIO, ", ".join(slow_topics)))
        print("  " + ROSBRIDGE_BANDWIDTH_NOTE)
        for line in ROSBRIDGE_MITIGATION:
            print("  " + line)

    if args.json_out:
        output = Path(args.json_out)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
        )
    print(json.dumps(report, ensure_ascii=False))
    return 0 if not failures else 2


if __name__ == "__main__":
    sys.exit(main())
