#!/usr/bin/env python3
"""카메라 semantic 출력을 뷰별 base_link BEV 레이어로 투영한다.

VIP3 는 두 가지 secondary head 를 지원한다.

  secondary_head=lane          (기본, VIP3 v5 binary 체크포인트)
      입력 2장: drivable/probability + lane/probability  (둘 다 연속 확률 mono8)
      출력   : drivable_probability, lane_probability, coverage
  secondary_head=road_marking  (v6 4-class 체크포인트)
      입력 3장: drivable/probability + road_marking/class_id + road_marking/confidence
      출력   : drivable_probability, road_marking/class_id, road_marking/confidence, coverage

ASMC 원본은 road_marking 3장 경로만 있었다. VIP3 기본 체크포인트가 binary 라
그 경로만으로는 이 노드가 영원히 아무것도 발행하지 않는다.
"""

from __future__ import annotations

import time
from functools import partial
from pathlib import Path

import message_filters
import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import Image

from drivable_bev.calibration import (
    calibrations_from_dict,
    validate_sensor_set_snapshot,
)
from drivable_bev.grid import BevGridSpec
from drivable_bev.performance_monitor import (
    PerformanceMonitor,
    source_time_gate_with_reset,
)
from drivable_bev.projector import CameraBevProjector

SECONDARY_HEADS = ("lane", "road_marking")


def _enabled_views(value):
    if isinstance(value, str):
        values = [item.strip() for item in value.split(",")]
    else:
        values = [str(item).strip() for item in value]
    values = [item for item in values if item]
    if not values or len(values) != len(set(values)):
        raise ValueError("enabled_views must contain unique camera names")
    return values


class CameraBevNode:
    def __init__(self) -> None:
        grid_values = {
            name: rospy.get_param("~" + name)
            for name in (
                "frame_id",
                "x_min_m",
                "x_max_m",
                "y_min_m",
                "y_max_m",
                "resolution_m",
            )
        }
        self.grid = BevGridSpec.from_mapping(grid_values)
        self.secondary_head = str(rospy.get_param("~secondary_head", "lane"))
        if self.secondary_head not in SECONDARY_HEADS:
            raise ValueError(
                "secondary_head must be one of {}, got {!r}".format(
                    SECONDARY_HEADS, self.secondary_head
                )
            )
        self.max_publish_hz = float(rospy.get_param("~max_publish_hz", 20.0))
        if self.max_publish_hz <= 0.0:
            raise ValueError("max_publish_hz must be positive")
        self.minimum_period_ns = int(round(1e9 / self.max_publish_hz))
        max_ground_range = float(rospy.get_param("~max_ground_range_m", 25.0))
        min_camera_depth = float(rospy.get_param("~min_camera_depth_m", 0.1))
        max_class_id = int(rospy.get_param("~max_class_id", 3))

        camera_values = rospy.get_param("~cameras")
        if not isinstance(camera_values, dict):
            raise ValueError("cameras parameter must be a mapping")
        ground_plane_values = rospy.get_param("~ground_plane")
        calibration_document = {
            "ground_plane": ground_plane_values,
            "cameras": camera_values,
        }
        all_cameras = calibrations_from_dict(calibration_document)
        self.views = _enabled_views(
            rospy.get_param("~enabled_views", ["front", "left", "right", "rear"])
        )
        unknown = sorted(set(self.views) - set(all_cameras))
        if unknown:
            raise ValueError("unknown camera views: {}".format(unknown))
        self.cameras = {view: all_cameras[view] for view in self.views}

        self.source_sha256 = str(rospy.get_param("~source_sha256", ""))
        if bool(rospy.get_param("~verify_sensor_set", True)):
            sensor_set_path = Path(str(rospy.get_param("~sensor_set_path")))
            snapshot = {
                "source_sha256": self.source_sha256,
                "ground_plane": ground_plane_values,
                "cameras": camera_values,
            }
            validate_sensor_set_snapshot(snapshot, all_cameras, sensor_set_path)
            rospy.loginfo("validated camera calibration against %s", sensor_set_path)

        self.projectors = {
            view: CameraBevProjector(
                self.cameras[view],
                self.grid,
                max_ground_range_m=max_ground_range,
                min_camera_depth_m=min_camera_depth,
                max_class_id=max_class_id,
            )
            for view in self.views
        }
        self.coverage_ratio = {
            view: float(np.mean(self.projectors[view].homography.coverage > 0))
            for view in self.views
        }
        for view in self.views:
            if self.coverage_ratio[view] <= 0.0:
                # 후방 카메라 yaw 180 이 여기서 0 이 나오면 pitch 부호나 지면 z 가
                # 틀렸다는 뜻이다. 조용히 빈 BEV 를 내보내지 않고 크게 경고한다.
                rospy.logwarn(
                    "%s BEV coverage is ZERO — check camera pose sign convention "
                    "and ground_plane.z_at_origin_m",
                    view,
                )
        self.monitor = PerformanceMonitor(self.views)
        self.last_input_stamp_ns = {view: 0 for view in self.views}
        self.next_output_stamp_ns = {view: 0 for view in self.views}
        self.timestamp_resets = {view: 0 for view in self.views}
        self.publishers = {}
        self.subscribers = []
        self.synchronizers = []

        for view in self.views:
            namespace = "/perception/bev/debug/{}".format(view)
            publishers = {
                "drivable": rospy.Publisher(
                    namespace + "/drivable_probability", Image, queue_size=1
                ),
                "coverage": rospy.Publisher(
                    namespace + "/coverage", Image, queue_size=1, latch=True
                ),
            }
            if self.secondary_head == "lane":
                publishers["lane"] = rospy.Publisher(
                    namespace + "/lane_probability", Image, queue_size=1
                )
            else:
                publishers["road_marking"] = rospy.Publisher(
                    namespace + "/road_marking/class_id", Image, queue_size=1
                )
                publishers["road_marking_confidence"] = rospy.Publisher(
                    namespace + "/road_marking/confidence", Image, queue_size=1
                )
            self.publishers[view] = publishers

            camera = self.cameras[view]
            topics = (camera.drivable_topic,) + camera.secondary_topics(
                self.secondary_head
            )
            subscribers = [
                message_filters.Subscriber(topic, Image, queue_size=2)
                for topic in topics
            ]
            synchronizer = message_filters.TimeSynchronizer(
                subscribers, queue_size=4
            )
            synchronizer.registerCallback(partial(self._callback, view))
            self.subscribers.extend(subscribers)
            self.synchronizers.append(synchronizer)

        self.diagnostics_publisher = rospy.Publisher(
            "/perception/bev/debug/diagnostics", DiagnosticArray, queue_size=1
        )
        interval = float(rospy.get_param("~diagnostics_interval_sec", 2.0))
        self.diagnostics_timer = rospy.Timer(
            rospy.Duration(max(0.2, interval)), self._publish_diagnostics
        )
        rospy.loginfo(
            "camera BEV ready views=%s head=%s grid=%dx%d x=[%.1f,%.1f] "
            "y=[%.1f,%.1f] resolution=%.3f max_hz=%.1f coverage=%s",
            self.views,
            self.secondary_head,
            self.grid.width_px,
            self.grid.height_px,
            self.grid.x_min_m,
            self.grid.x_max_m,
            self.grid.y_min_m,
            self.grid.y_max_m,
            self.grid.resolution_m,
            self.max_publish_hz,
            {view: round(value, 4) for view, value in self.coverage_ratio.items()},
        )

    def _callback(self, view: str, *messages) -> None:
        drivable_message = messages[0]
        stamp_ns = int(drivable_message.header.stamp.to_nsec())
        self.monitor.count_input(view, stamp_ns)
        paired_stamps = {int(item.header.stamp.to_nsec()) for item in messages}
        if stamp_ns <= 0 or len(paired_stamps) != 1:
            self.monitor.count_error(view)
            rospy.logwarn_throttle(
                2.0, "%s BEV received invalid/mismatched stamps", view
            )
            return
        previous_input_ns = self.last_input_stamp_ns[view]
        (
            should_publish,
            following_output_ns,
            timestamp_reset,
        ) = source_time_gate_with_reset(
            stamp_ns,
            previous_input_ns,
            self.next_output_stamp_ns[view],
            self.minimum_period_ns,
        )
        if previous_input_ns and stamp_ns <= previous_input_ns and not timestamp_reset:
            self.monitor.count_error(view)
            rospy.logwarn_throttle(
                2.0, "%s BEV source timestamp did not increase", view
            )
            return
        if timestamp_reset:
            self.timestamp_resets[view] += 1
            rospy.loginfo("%s BEV source-time epoch reset", view)
        self.last_input_stamp_ns[view] = stamp_ns
        if not should_publish:
            self.monitor.count_throttled(view)
            return

        try:
            arrays = [self._mono8_array(item) for item in messages]
            started = time.perf_counter()
            if self.secondary_head == "lane":
                projected = self.projectors[view].project_lane(arrays[0], arrays[1])
            else:
                projected = self.projectors[view].project(
                    arrays[0], arrays[1], arrays[2]
                )
            projection_ms = (time.perf_counter() - started) * 1000.0

            started = time.perf_counter()
            header = drivable_message.header
            publishers = self.publishers[view]
            publishers["drivable"].publish(
                self._image_message(projected.drivable_probability, header)
            )
            if self.secondary_head == "lane":
                publishers["lane"].publish(
                    self._image_message(projected.lane_probability, header)
                )
            else:
                publishers["road_marking"].publish(
                    self._image_message(projected.road_marking_class_id, header)
                )
                publishers["road_marking_confidence"].publish(
                    self._image_message(projected.road_marking_confidence, header)
                )
            publishers["coverage"].publish(
                self._image_message(projected.coverage, header)
            )
            publish_ms = (time.perf_counter() - started) * 1000.0
        except Exception as exc:
            self.monitor.count_error(view)
            rospy.logerr_throttle(2.0, "%s BEV projection failed: %s", view, exc)
            return

        self.next_output_stamp_ns[view] = following_output_ns
        result_stamp_ns = int(rospy.Time.now().to_nsec())
        self.monitor.count_output(
            view,
            projection_ms,
            publish_ms,
            source_stamp_ns=stamp_ns,
            result_stamp_ns=result_stamp_ns,
        )

    @staticmethod
    def _mono8_array(message: Image) -> np.ndarray:
        if message.encoding not in ("mono8", "8UC1"):
            raise ValueError("expected mono8 image, got {}".format(message.encoding))
        if message.step < message.width:
            raise ValueError("image step is smaller than width")
        data = np.frombuffer(bytes(message.data), dtype=np.uint8)
        required = int(message.height) * int(message.step)
        if data.size < required:
            raise ValueError("image data is shorter than height*step")
        return np.ascontiguousarray(
            data[:required].reshape(int(message.height), int(message.step))[
                :, : int(message.width)
            ]
        )

    def _image_message(self, value: np.ndarray, source_header) -> Image:
        array = np.ascontiguousarray(value, dtype=np.uint8)
        message = Image()
        message.header.stamp = source_header.stamp
        message.header.frame_id = self.grid.frame_id
        message.height, message.width = array.shape
        message.encoding = "mono8"
        message.is_bigendian = 0
        message.step = message.width
        message.data = array.tobytes()
        return message

    def _publish_diagnostics(self, _event) -> None:
        message = DiagnosticArray()
        message.header.stamp = rospy.Time.now()
        snapshot = self.monitor.snapshot()
        for view in self.views:
            values = snapshot[view]
            outputs = int(values["outputs"])
            errors = int(values["errors"])
            level = DiagnosticStatus.OK
            text = "projection active"
            if outputs == 0:
                level = DiagnosticStatus.WARN
                text = "waiting for exact-stamp semantic pair"
            elif errors:
                level = DiagnosticStatus.WARN
                text = "projection active with errors"
            status = DiagnosticStatus(
                level=level,
                name="drivable_bev/{}".format(view),
                message=text,
                hardware_id="camera-{}".format(self.cameras[view].sensor_id),
            )
            source_stamp_ns = int(values["last_output_source_stamp_ns"])
            result_stamp_ns = int(values["last_result_stamp_ns"])
            source_age_ms = 0.0
            result_age_ms = 0.0
            now_ns = int(message.header.stamp.to_nsec())
            if source_stamp_ns and now_ns >= source_stamp_ns:
                source_age_ms = (now_ns - source_stamp_ns) / 1e6
            if result_stamp_ns and now_ns >= result_stamp_ns:
                result_age_ms = (now_ns - result_stamp_ns) / 1e6
            items = {
                **values,
                "secondary_head": self.secondary_head,
                "source_age_ms": source_age_ms,
                "result_age_ms": result_age_ms,
                "coverage_ratio": self.coverage_ratio[view],
                "timestamp_resets": self.timestamp_resets[view],
                "calibration_sha256": self.source_sha256,
                "ground_plane_frame": self.cameras[view].ground_plane.frame_id,
                "ground_plane_coefficients": self.cameras[view].ground_plane.coefficients,
                "ground_plane_source": self.cameras[view].ground_plane.source,
                "frame_id": self.grid.frame_id,
                "grid_width_px": self.grid.width_px,
                "grid_height_px": self.grid.height_px,
                "resolution_m": self.grid.resolution_m,
                "x_range_m": "{:.3f},{:.3f}".format(
                    self.grid.x_min_m, self.grid.x_max_m
                ),
                "y_range_m": "{:.3f},{:.3f}".format(
                    self.grid.y_min_m, self.grid.y_max_m
                ),
            }
            status.values = [
                KeyValue(key=str(key), value=str(value))
                for key, value in items.items()
            ]
            message.status.append(status)
        self.diagnostics_publisher.publish(message)


def main() -> None:
    rospy.init_node("camera_bev")
    CameraBevNode()
    rospy.spin()


if __name__ == "__main__":
    main()
