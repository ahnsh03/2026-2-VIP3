#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MORAI Capture Mode 를 일정 주기로 자동 트리거하는 노드.

제어 명령을 **발행하지 않는다.** 그래서 휠/키보드 수동주행 중에도, 나중에
제어 노드가 /ctrl_cmd 를 쏘는 중에도 그대로 쓸 수 있다.

전송은 rosbridge 하나뿐이다. Capture 트리거 = morai_msgs/SaveSensorData 를
/SaveSensorData 에 publish. (ASMC 의 gRPC :7789 백엔드는 VIP3 에서 삭제했다.)

!! 반드시 알아야 할 한계 !!
ROS publish 에는 애플리케이션 레벨 저장 완료 응답이 없다. 여기서 기록하는
`success: true` 는 **"publish 가 리턴했다"** 는 뜻일 뿐, MORAI 가 PNG 를 실제로
썼다는 보장이 전혀 아니다. 디스크 가득참·권한 오류로 저장이 실패해도 매니페스트는
성공으로 남는다. 그래서 주행이 끝나면 **반드시**

    python3 scripts/sync_capture_data.py --run-id <run_id> --dry-run

으로 프레임당 18파일 preflight 를 돌려야 한다. 이게 유일한 검증 게이트다.
"""

from __future__ import print_function

import copy
import threading
import time

import rospy
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus, SaveSensorData

from data_collection.capture_mode import (
    CaptureRunWriter,
    CaptureSchedule,
    ctrl_message_to_dict,
    default_capture_data_root,
    ego_message_to_dict,
    make_run_id,
    sanitize_name,
    speed_mps_from_ego,
    utc_now_iso,
)


class CaptureBackendError(RuntimeError):
    """Capture 트리거 경로를 준비하지 못했다."""


class RosTopicCaptureClient(object):
    """MORAI 기본 /SaveSensorData 커맨드를 발행한다. 제어에는 손대지 않는다."""

    def __init__(self, topic):
        self.topic = topic
        self._publisher = rospy.Publisher(topic, SaveSensorData, queue_size=1)

    def wait_ready(self, timeout_sec):
        deadline = time.monotonic() + float(timeout_sec)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self._publisher.get_num_connections() > 0:
                return
            time.sleep(0.1)
        raise CaptureBackendError(
            "{} 를 구독하는 MORAI(rosbridge) 가 {:.1f}s 안에 붙지 않았다".format(
                self.topic, float(timeout_sec)
            )
        )

    def capture(self, custom_name, file_dir=""):
        msg = SaveSensorData()
        msg.is_custom_file_name = True
        msg.custom_file_name = custom_name
        msg.file_dir = file_dir or ""
        started = time.monotonic()
        self._publisher.publish(msg)
        # success=True 는 publish 가 리턴했다는 뜻뿐이다. 파일 존재 검증이 아니다.
        # rpc_latency_ms 도 로컬 publish 비용(≈0)일 뿐, 시뮬의 실제 저장 시간이 아니다.
        # morai_sim_time_* 은 gRPC 전용이었으므로 항상 None (스키마 안정성용으로 유지).
        return {
            "success": True,
            "capture_backend": "ros_topic",
            "grpc_status": None,
            "rpc_latency_ms": round((time.monotonic() - started) * 1000.0, 3),
            "morai_sim_time_raw": None,
            "morai_sim_time_unit": None,
            "morai_sim_time_ns": None,
            "transport_note": (
                "rosbridge publish 에는 저장 완료 응답이 없다. "
                "success=true == publish returned. "
                "sync_capture_data.py --dry-run 으로 반드시 검증할 것"
            ),
        }

    def close(self):
        self._publisher.unregister()


class CaptureModeCollector(object):
    def __init__(self):
        self.hz = float(rospy.get_param("~capture_hz", 1.0))
        self.max_captures = int(rospy.get_param("~max_captures", 0))
        self.start_delay_sec = float(rospy.get_param("~start_delay_sec", 2.0))
        self.only_when_moving = bool(rospy.get_param("~only_when_moving", False))
        self.min_speed_mps = float(rospy.get_param("~min_speed_mps", 0.2))
        self.require_ego = bool(rospy.get_param("~require_ego", False))
        self.dry_run = bool(rospy.get_param("~dry_run", False))
        # ROS 백엔드에서는 capture 가 예외를 던지는 일이 사실상 없으므로 이 자동 중단은
        # 거의 발동하지 않는다. 남겨두는 이유는 publish 자체가 죽는 경우를 잡기 위해서다.
        self.max_consecutive_failures = int(
            rospy.get_param("~max_consecutive_failures", 3)
        )
        self.control_source_timeout = float(
            rospy.get_param("~control_source_timeout_sec", 1.0)
        )
        self.file_dir = str(rospy.get_param("~morai_file_dir", ""))
        self._lock = threading.Lock()
        self._latest_ego = None
        self._latest_ego_wall = None
        self._latest_ctrl = None
        self._latest_ctrl_wall = None
        self._closed = False

        run_id = rospy.get_param("~run_id", "") or make_run_id("katri_capture")
        self.run_id = sanitize_name(run_id, fallback="katri_capture")
        data_root = rospy.get_param("~data_root", "") or default_capture_data_root()
        save_topic = str(rospy.get_param("~save_topic", "/SaveSensorData"))
        connect_timeout = float(rospy.get_param("~connect_timeout_sec", 5.0))

        metadata = {
            "capture_hz": self.hz,
            "max_captures": self.max_captures,
            "only_when_moving": self.only_when_moving,
            "min_speed_mps": self.min_speed_mps,
            "require_ego": self.require_ego,
            "capture_backend_requested": "ros_topic",
            "transport": "rosbridge",
            "bridge_url": str(
                rospy.get_param("~bridge_url", "ws://127.0.0.1:9090")
            ),
            "morai_msgs_branch": "26.R1 @ 4c9be6f",
            "save_topic": save_topic,
            "morai_file_dir": self.file_dir,
            "dry_run": self.dry_run,
            "ego_topic": str(rospy.get_param("~ego_topic", "/Ego_topic")),
            "ctrl_topic": str(rospy.get_param("~ctrl_topic", "/ctrl_cmd")),
            "map": str(rospy.get_param("~map", "R_KR_PG_KATRI")),
            "vehicle": str(rospy.get_param("~vehicle", "2023_Hyundai_ioniq5")),
            "capture_policy": "wall_clock_fixed_rate_no_catchup",
            "control_independent": True,
            "notes": (
                "Capture Mode GT 원본은 MORAI SensorData(Windows)에 있다. "
                "여기에는 트리거/상태 lineage 만 남는다. "
                "success=true 는 publish 성공일 뿐이므로 "
                "sync_capture_data.py --dry-run 이 필수 게이트다."
            ),
        }
        self.writer = CaptureRunWriter(data_root, self.run_id, metadata)
        self.schedule = CaptureSchedule(self.hz, self.start_delay_sec)
        self.client = None

        ego_topic = metadata["ego_topic"]
        ctrl_topic = metadata["ctrl_topic"]
        self._ego_sub = rospy.Subscriber(
            ego_topic, EgoVehicleStatus, self._on_ego, queue_size=1
        )
        # VIP3_network_v1.json 에는 아직 Ego Ctrl Cmd 채널이 없다. 그래서 당분간
        # control_source 는 항상 "manual_or_unknown" 이 정상이다. 버그가 아니다.
        self._ctrl_sub = rospy.Subscriber(ctrl_topic, CtrlCmd, self._on_ctrl, queue_size=1)

        if self.hz > 2.0:
            rospy.logwarn(
                "capture_hz=%.2f 는 아직 검증 안 된 구간이다. "
                "이 데이터를 쓰기 전에 100프레임 파일 무결성 QA 를 돌릴 것",
                self.hz,
            )

        self.backend_active = "dry_run"
        if not self.dry_run:
            try:
                self.client = RosTopicCaptureClient(save_topic)
                self.client.wait_ready(connect_timeout)
                self.backend_active = "ros_topic"
            except Exception:
                if self.client is not None:
                    self.client.close()
                self.writer.close()
                raise

        rospy.on_shutdown(self.close)
        rospy.loginfo(
            "Capture Mode collector ready: %.2f Hz, run=%s, backend=%s, manifest=%s, dry_run=%s",
            self.hz,
            self.run_id,
            self.backend_active,
            self.writer.manifest_path,
            self.dry_run,
        )
        rospy.logwarn(
            "이 노드는 제어를 발행하지 않는다. 그리고 rosbridge 백엔드에는 저장 완료 "
            "응답이 없으므로 manifest 의 success=true 는 'publish 가 리턴했다'는 "
            "뜻뿐이다. 주행 종료 후 반드시 "
            "`python3 scripts/sync_capture_data.py --run-id %s --dry-run` 을 돌릴 것",
            self.run_id,
        )

    def _on_ego(self, msg):
        now = time.time()
        value = ego_message_to_dict(msg)
        with self._lock:
            self._latest_ego = value
            self._latest_ego_wall = now

    def _on_ctrl(self, msg):
        now = time.time()
        value = ctrl_message_to_dict(msg)
        with self._lock:
            self._latest_ctrl = value
            self._latest_ctrl_wall = now

    def _snapshot(self):
        now = time.time()
        with self._lock:
            ego = copy.deepcopy(self._latest_ego)
            ctrl = copy.deepcopy(self._latest_ctrl)
            ego_wall = self._latest_ego_wall
            ctrl_wall = self._latest_ctrl_wall
        ego_age = None if ego_wall is None else max(0.0, now - ego_wall)
        ctrl_age = None if ctrl_wall is None else max(0.0, now - ctrl_wall)
        ros_ctrl_recent = ctrl_age is not None and ctrl_age <= self.control_source_timeout
        return {
            "ego": ego,
            "ego_age_sec": None if ego_age is None else round(ego_age, 4),
            "ctrl_cmd": ctrl if ros_ctrl_recent else None,
            "ctrl_cmd_age_sec": None if ctrl_age is None else round(ctrl_age, 4),
            "control_source": "ros_ctrl" if ros_ctrl_recent else "manual_or_unknown",
        }

    def _motion_allows_capture(self, snapshot):
        ego = snapshot.get("ego")
        if ego is None:
            return not self.require_ego and not self.only_when_moving
        if self.only_when_moving and speed_mps_from_ego(ego) < self.min_speed_mps:
            return False
        return True

    def run(self):
        seq = 0
        consecutive_failures = 0
        while not rospy.is_shutdown():
            if self.max_captures > 0 and self.writer.succeeded >= self.max_captures:
                rospy.loginfo("max_captures=%d reached", self.max_captures)
                break
            remaining = self.schedule.remaining()
            if remaining > 0.0:
                time.sleep(min(remaining, 0.1))
                continue

            snapshot = self._snapshot()
            if not self._motion_allows_capture(snapshot):
                self.writer.mark_motion_skip()
                self.schedule.advance()
                rospy.loginfo_throttle(
                    5.0,
                    "capture waiting for ego motion (require_ego=%s min_speed=%.2f m/s)",
                    self.require_ego,
                    self.min_speed_mps,
                )
                continue

            custom_name = "{}_{:06d}".format(self.run_id, seq)
            seq += 1
            record = {
                "sequence": seq - 1,
                "custom_name": custom_name,
                "requested_at_utc": utc_now_iso(),
                "request_wall_time_ns": time.time_ns(),
                "ros_time_ns": int(rospy.Time.now().to_nsec()),
                "state": snapshot,
                "success": False,
            }
            try:
                if self.dry_run:
                    result = {
                        "success": True,
                        "capture_backend": "dry_run",
                        "grpc_status": None,
                        "rpc_latency_ms": 0.0,
                        "morai_sim_time_raw": None,
                        "morai_sim_time_unit": None,
                        "morai_sim_time_ns": None,
                        "dry_run": True,
                    }
                else:
                    result = self.client.capture(custom_name, self.file_dir)
                    result.setdefault("capture_backend", self.backend_active)
                record.update(result)
            except Exception as exc:
                record["error"] = "{}: {}".format(type(exc).__name__, exc)
                rospy.logerr("capture %s failed: %s", custom_name, record["error"])

            record["completed_at_utc"] = utc_now_iso()
            record["completed_wall_time_ns"] = time.time_ns()
            self.writer.write_capture(record)
            if record.get("success"):
                consecutive_failures = 0
                rospy.loginfo(
                    "capture published seq=%06d (저장 확인 아님) publish=%.1fms source=%s",
                    seq - 1,
                    float(record.get("rpc_latency_ms") or 0.0),
                    snapshot["control_source"],
                )
            else:
                consecutive_failures += 1
                rospy.logerr(
                    "capture publish 실패 seq=%06d error=%s consecutive_failures=%d",
                    seq - 1,
                    record.get("error"),
                    consecutive_failures,
                )
                if consecutive_failures >= self.max_consecutive_failures:
                    rospy.signal_shutdown("capture publish failed repeatedly")
                    break
            self.schedule.advance()

        self.close()

    def close(self):
        if self._closed:
            return
        self._closed = True
        if self.client is not None:
            self.client.close()
        self.writer.close()
        rospy.loginfo(
            "capture run closed: root=%s attempted=%d published=%d failed=%d motion_skips=%d",
            self.writer.root,
            self.writer.attempted,
            self.writer.succeeded,
            self.writer.failed,
            self.writer.skipped_motion,
        )
        rospy.logwarn(
            "다음 단계(필수): python3 scripts/sync_capture_data.py --run-id %s --dry-run",
            self.run_id,
        )


def main():
    rospy.init_node("capture_collector")
    try:
        collector = CaptureModeCollector()
        collector.run()
    except (CaptureBackendError, ValueError, OSError) as exc:
        rospy.logfatal("Capture Mode collector 시작 실패: %s", exc)
        raise SystemExit(2)


if __name__ == "__main__":
    main()
