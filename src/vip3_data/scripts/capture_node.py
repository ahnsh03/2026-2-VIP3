#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MORAI Capture Mode 를 capture_hz 주기로 트리거하는 노드.

/SaveSensorData (morai_msgs/SaveSensorData) 를 발행할 뿐 제어 명령은 내지 않는다.

!! success=true 는 "publish 가 리턴했다"는 뜻일 뿐 PNG 저장 확인이 아니다 !!
MORAI 는 저장 완료 응답을 주지 않는다. 주행이 끝나면 반드시

    rosrun vip3_data check_capture.py --run-id <run_id>

으로 프레임당 12파일(4 카메라 x Intensity/Semantic/Instance)을 검사한다.
카메라마다 Active List 의 Sensor Capture Mode 버튼이 켜져 있어야 3종이 나온다.
"""

from __future__ import print_function

import time

import rospy
from morai_msgs.msg import SaveSensorData

from vip3_data.capture import (
    CaptureRunWriter,
    CaptureSchedule,
    FILES_PER_FRAME,
    capture_name,
    default_data_root,
    make_run_id,
    sanitize_name,
    utc_now_iso,
)


class CaptureNode(object):
    def __init__(self):
        self.hz = float(rospy.get_param("~capture_hz", 1.0))
        self.max_captures = int(rospy.get_param("~max_captures", 0))
        self.start_delay_sec = float(rospy.get_param("~start_delay_sec", 2.0))
        self.dry_run = bool(rospy.get_param("~dry_run", False))
        self.max_consecutive_failures = int(rospy.get_param("~max_consecutive_failures", 3))
        self.file_dir = str(rospy.get_param("~morai_file_dir", ""))
        self.save_topic = str(rospy.get_param("~save_topic", "/SaveSensorData"))
        self.width = int(rospy.get_param("~image_width", 1280))
        self.height = int(rospy.get_param("~image_height", 960))
        connect_timeout = float(rospy.get_param("~connect_timeout_sec", 5.0))

        run_id = rospy.get_param("~run_id", "") or make_run_id("vip3_capture")
        self.run_id = sanitize_name(run_id, fallback="vip3_capture")
        data_root = rospy.get_param("~data_root", "") or default_data_root()

        self.schedule = CaptureSchedule(self.hz, self.start_delay_sec)
        self.writer = CaptureRunWriter(data_root, self.run_id, {
            "capture_hz": self.hz,
            "max_captures": self.max_captures,
            "save_topic": self.save_topic,
            "morai_file_dir": self.file_dir,
            "dry_run": self.dry_run,
            "image_width": self.width,
            "image_height": self.height,
            "sensor_set": "261110_VIP3_Sensor_26.R1.H3.json",
        })
        self._publisher = None
        self._closed = False
        try:
            if not self.dry_run:
                self._publisher = rospy.Publisher(
                    self.save_topic, SaveSensorData, queue_size=1)
                self._wait_for_subscriber(connect_timeout)
        except Exception:
            self.close()
            raise
        if self.hz > 0.5:
            rospy.logwarn("capture_hz=%.2f > 0.5: 실측에서 1.0Hz 는 프레임이 통째로 누락됐다 "
                          "(MORAI 가 저장 중 요청을 버림). check_capture.py 로 반드시 확인", self.hz)
        rospy.on_shutdown(self.close)
        rospy.loginfo("capture ready: %.2f Hz run=%s dry_run=%s manifest=%s",
                      self.hz, self.run_id, self.dry_run, self.writer.manifest_path)
        rospy.logwarn("success=true 는 publish 성공일 뿐 저장 확인이 아니다. 종료 후 "
                      "check_capture.py --run-id %s 으로 %d파일/프레임을 검사할 것",
                      self.run_id, FILES_PER_FRAME)

    def _wait_for_subscriber(self, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self._publisher.get_num_connections() > 0:
                return
            time.sleep(0.1)
        raise RuntimeError("{} 를 구독하는 MORAI(rosbridge) 가 {:.1f}s 안에 붙지 않았다"
                           .format(self.save_topic, timeout_sec))

    def _publish(self, name):
        if self.dry_run:
            return
        msg = SaveSensorData()
        msg.is_custom_file_name = True
        msg.custom_file_name = name
        msg.file_dir = self.file_dir
        self._publisher.publish(msg)

    def run(self):
        seq = 0
        failures = 0
        while not rospy.is_shutdown():
            if self.max_captures > 0 and self.writer.published >= self.max_captures:
                rospy.loginfo("max_captures=%d reached", self.max_captures)
                break
            remaining = self.schedule.remaining()
            if remaining > 0.0:
                time.sleep(min(remaining, 0.1))
                continue

            name = capture_name(self.run_id, seq)
            record = {"sequence": seq, "custom_name": name,
                      "requested_at_utc": utc_now_iso(), "success": False,
                      "dry_run": self.dry_run}
            seq += 1
            try:
                self._publish(name)
                record["success"] = True
            except Exception as exc:  # publish 자체가 죽은 경우만 잡힌다
                record["error"] = "{}: {}".format(type(exc).__name__, exc)
                rospy.logerr("capture %s publish failed: %s", name, record["error"])
            self.writer.write_capture(record)
            self.schedule.advance()

            failures = 0 if record["success"] else failures + 1
            if failures >= self.max_consecutive_failures:
                rospy.signal_shutdown("capture publish failed repeatedly")
                break
        self.close()

    def close(self):
        if self._closed:
            return
        self._closed = True
        if self._publisher is not None:
            self._publisher.unregister()
        self.writer.close()
        rospy.loginfo("capture closed: attempted=%d published=%d failed=%d root=%s",
                      self.writer.attempted, self.writer.published,
                      self.writer.failed, self.writer.root)
        rospy.logwarn("다음 단계(필수): check_capture.py --run-id %s", self.run_id)


def main():
    rospy.init_node("vip3_capture")
    try:
        CaptureNode().run()
    except (RuntimeError, ValueError, OSError) as exc:
        rospy.logfatal("capture 시작 실패: %s", exc)
        raise SystemExit(2)


if __name__ == "__main__":
    main()
