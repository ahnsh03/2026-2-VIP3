#!/usr/bin/env python3
"""Synchronize and fuse three projected camera semantic BEV streams."""

from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass
from functools import partial

import message_filters
import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import Image

from drivable_bev.calibration import calibrations_from_dict
from drivable_bev.fusion import (
    CameraBevFusion,
    build_quality_maps,
    timestamp_span_ns,
)
from drivable_bev.grid import BevGridSpec
from drivable_bev.performance_monitor import source_time_gate_with_reset
from drivable_bev.projector import CameraBevProjector, ProjectedSemantic


@dataclass(frozen=True)
class _ViewFrame:
    view: str
    stamp_ns: int
    header: object
    semantic: ProjectedSemantic


def _names(value):
    values = (
        [item.strip() for item in value.split(",")]
        if isinstance(value, str)
        else [str(item).strip() for item in value]
    )
    values = [item for item in values if item]
    if not values or len(values) != len(set(values)):
        raise ValueError("fusion views must be non-empty and unique")
    return values


class CameraBevFusionNode:
    def __init__(self) -> None:
        self.grid = BevGridSpec.from_mapping(
            {
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
        )
        self.views = _names(
            rospy.get_param("~views", ["front", "left", "right", "rear"])
        )
        self.secondary_head = str(rospy.get_param("~secondary_head", "lane"))
        if self.secondary_head not in ("lane", "road_marking"):
            raise ValueError(
                "secondary_head must be lane|road_marking, got {!r}".format(
                    self.secondary_head
                )
            )
        camera_values = rospy.get_param("~cameras")
        cameras = calibrations_from_dict(
            {
                "ground_plane": rospy.get_param("~ground_plane"),
                "cameras": camera_values,
            }
        )
        self.ground_plane = next(iter(cameras.values())).ground_plane
        self.calibration_sha256 = str(rospy.get_param("~source_sha256", ""))
        missing = sorted(set(self.views) - set(cameras))
        if missing:
            raise ValueError("camera calibration is missing views: {}".format(missing))

        max_ground_range = float(rospy.get_param("~max_ground_range_m", 25.0))
        min_depth = float(rospy.get_param("~min_camera_depth_m", 0.1))
        projectors = {
            view: CameraBevProjector(
                cameras[view], self.grid, max_ground_range, min_depth
            )
            for view in self.views
        }
        quality = build_quality_maps(
            projectors,
            self.grid,
            edge_taper_fraction=float(rospy.get_param("~edge_taper_fraction", 0.12)),
            near_full_quality_radius_m=float(
                rospy.get_param("~near_full_quality_radius_m", 6.0)
            ),
            max_quality_radius_m=float(
                rospy.get_param("~max_quality_radius_m", 12.0)
            ),
            boundary_weight=float(rospy.get_param("~boundary_weight", 0.2)),
            per_view_density_normalisation=bool(
                rospy.get_param("~per_view_density_normalisation", True)
            ),
        )
        view_ids = {
            str(name): int(value)
            for name, value in dict(rospy.get_param("~view_ids")).items()
            if str(name) in self.views
        }
        self.fusion = CameraBevFusion(quality, view_ids=view_ids)
        self.max_skew_ns = int(
            round(float(rospy.get_param("~max_timestamp_skew_ms", 35.0)) * 1e6)
        )
        if self.max_skew_ns < 0:
            raise ValueError("max_timestamp_skew_ms cannot be negative")
        self.max_fused_hz = float(rospy.get_param("~max_fused_hz", 10.0))
        if self.max_fused_hz <= 0.0:
            raise ValueError("max_fused_hz must be positive")
        self.minimum_output_period_ns = int(round(1e9 / self.max_fused_hz))
        self.next_output_stamp_ns = 0
        self.queue_size = int(rospy.get_param("~sync_queue_size", 12))
        if self.queue_size <= 0:
            raise ValueError("sync_queue_size must be positive")

        self.buffers = {view: deque() for view in self.views}
        self.buffer_lock = threading.Lock()
        self.processing_lock = threading.Lock()
        self.inputs = {view: 0 for view in self.views}
        self.queue_drops = {view: 0 for view in self.views}
        self.pair_errors = {view: 0 for view in self.views}
        self.sync_rejected = 0
        self.output_throttled = 0
        self.outputs = 0
        self.fusion_errors = 0
        self.last_skew_ms = 0.0
        self.last_source_stamp_ns = 0
        self.last_input_source_stamp_ns = 0
        self.last_result_stamp_ns = 0
        self.timestamp_resets = 0
        self.fusion_ms = deque(maxlen=512)
        self.end_to_end_ms = deque(maxlen=512)
        self.output_times = deque(maxlen=256)

        base = "/perception/bev/camera"
        self.publishers = {
            "drivable": rospy.Publisher(
                base + "/drivable_probability", Image, queue_size=1
            ),
            "coverage": rospy.Publisher(base + "/coverage", Image, queue_size=1),
            "source_count": rospy.Publisher(
                base + "/source_count", Image, queue_size=1
            ),
            "source_view": rospy.Publisher(
                "/perception/bev/debug/source_view", Image, queue_size=1
            ),
        }
        if self.secondary_head == "lane":
            self.publishers["lane"] = rospy.Publisher(
                base + "/lane_probability", Image, queue_size=1
            )
        else:
            self.publishers["road_marking"] = rospy.Publisher(
                base + "/road_marking/class_id", Image, queue_size=1
            )
            self.publishers["road_marking_confidence"] = rospy.Publisher(
                base + "/road_marking/confidence", Image, queue_size=1
            )

        self.subscribers = []
        self.synchronizers = []
        for view in self.views:
            namespace = "/perception/bev/debug/{}".format(view)
            drivable = message_filters.Subscriber(
                namespace + "/drivable_probability", Image, queue_size=2
            )
            if self.secondary_head == "lane":
                inputs = [
                    drivable,
                    message_filters.Subscriber(
                        namespace + "/lane_probability", Image, queue_size=2
                    ),
                ]
            else:
                inputs = [
                    drivable,
                    message_filters.Subscriber(
                        namespace + "/road_marking/class_id", Image, queue_size=2
                    ),
                    message_filters.Subscriber(
                        namespace + "/road_marking/confidence", Image, queue_size=2
                    ),
                ]
            synchronizer = message_filters.TimeSynchronizer(inputs, queue_size=4)
            synchronizer.registerCallback(partial(self._view_callback, view))
            self.subscribers.extend(inputs)
            self.synchronizers.append(synchronizer)

        self.diagnostics_publisher = rospy.Publisher(
            "/perception/bev/diagnostics", DiagnosticArray, queue_size=1
        )
        interval = max(
            0.2, float(rospy.get_param("~diagnostics_interval_sec", 2.0))
        )
        self.diagnostics_timer = rospy.Timer(
            rospy.Duration(interval), self._publish_diagnostics
        )
        coverage_count = np.count_nonzero(np.stack(list(quality.values())) > 0.0, axis=0)
        rospy.loginfo(
            "camera BEV fusion ready views=%s grid=%dx%d skew<=%.1fms "
            "union=%.4f overlap2=%.4f",
            self.views,
            self.grid.width_px,
            self.grid.height_px,
            self.max_skew_ns / 1e6,
            float(np.mean(coverage_count > 0)),
            float(np.mean(coverage_count >= 2)),
        )

    def _view_callback(self, view: str, drivable: Image, *secondary) -> None:
        stamps = [
            int(message.header.stamp.to_nsec())
            for message in (drivable,) + tuple(secondary)
        ]
        self.inputs[view] += 1
        if any(value <= 0 for value in stamps) or len(set(stamps)) != 1:
            self.pair_errors[view] += 1
            rospy.logwarn_throttle(2.0, "%s fused BEV input stamps differ", view)
            return
        try:
            zeros = np.zeros(self.grid.shape, dtype=np.uint8)
            if self.secondary_head == "lane":
                semantic = ProjectedSemantic(
                    drivable_probability=self._mono8(drivable),
                    road_marking_class_id=zeros,
                    road_marking_confidence=zeros.copy(),
                    coverage=zeros.copy(),
                    lane_probability=self._mono8(secondary[0]),
                )
            else:
                semantic = ProjectedSemantic(
                    drivable_probability=self._mono8(drivable),
                    road_marking_class_id=self._mono8(secondary[0]),
                    road_marking_confidence=self._mono8(secondary[1]),
                    coverage=zeros,
                )
        except Exception as exc:
            self.pair_errors[view] += 1
            rospy.logwarn_throttle(2.0, "%s fused BEV input invalid: %s", view, exc)
            return
        frame = _ViewFrame(view, stamps[0], drivable.header, semantic)
        ready = None
        with self.buffer_lock:
            queue = self.buffers[view]
            if len(queue) >= self.queue_size:
                queue.popleft()
                self.queue_drops[view] += 1
            queue.append(frame)
            ready = self._take_synchronized_locked()
        if ready is not None:
            self._fuse_and_publish(ready)

    def _take_synchronized_locked(self):
        while all(self.buffers[view] for view in self.views):
            frames = [self.buffers[view][0] for view in self.views]
            stamps = [frame.stamp_ns for frame in frames]
            if timestamp_span_ns(stamps) <= self.max_skew_ns:
                return {
                    view: self.buffers[view].popleft() for view in self.views
                }
            oldest_view = self.views[int(np.argmin(stamps))]
            self.buffers[oldest_view].popleft()
            self.sync_rejected += 1
        return None

    def _fuse_and_publish(self, frames) -> None:
        with self.processing_lock:
            stamp_ns = max(frame.stamp_ns for frame in frames.values())
            should_publish, following_output_ns, timestamp_reset = (
                source_time_gate_with_reset(
                    stamp_ns,
                    self.last_input_source_stamp_ns,
                    self.next_output_stamp_ns,
                    self.minimum_output_period_ns,
                )
            )
            if timestamp_reset:
                self.timestamp_resets += 1
                rospy.loginfo("fused BEV source-time epoch reset")
            self.last_input_source_stamp_ns = max(
                stamp_ns,
                0 if timestamp_reset else self.last_input_source_stamp_ns,
            )
            if not should_publish:
                self.output_throttled += 1
                return
            started = time.perf_counter()
            try:
                fused = self.fusion.fuse(
                    {view: frames[view].semantic for view in self.views}
                )
                header = max(frames.values(), key=lambda frame: frame.stamp_ns).header
                messages = {
                    "drivable": fused.drivable_probability,
                    "coverage": fused.coverage,
                    "source_count": fused.source_count,
                    "source_view": fused.source_view,
                }
                if self.secondary_head == "lane":
                    messages["lane"] = fused.lane_probability
                else:
                    messages["road_marking"] = fused.road_marking_class_id
                    messages["road_marking_confidence"] = (
                        fused.road_marking_confidence
                    )
                for name, value in messages.items():
                    self.publishers[name].publish(self._message(value, header))
            except Exception as exc:
                self.fusion_errors += 1
                rospy.logerr_throttle(2.0, "camera BEV fusion failed: %s", exc)
                return
            elapsed_ms = (time.perf_counter() - started) * 1000.0
            self.next_output_stamp_ns = following_output_ns
            self.outputs += 1
            self.fusion_ms.append(elapsed_ms)
            self.output_times.append(time.monotonic())
            self.last_source_stamp_ns = stamp_ns
            self.last_result_stamp_ns = int(rospy.Time.now().to_nsec())
            end_to_end_ms = (self.last_result_stamp_ns - stamp_ns) / 1e6
            if 0.0 <= end_to_end_ms <= 60000.0:
                self.end_to_end_ms.append(end_to_end_ms)
            self.last_skew_ms = timestamp_span_ns(
                [frame.stamp_ns for frame in frames.values()]
            ) / 1e6

    def _mono8(self, message: Image):
        if message.encoding not in ("mono8", "8UC1"):
            raise ValueError("expected mono8, got {}".format(message.encoding))
        if (int(message.height), int(message.width)) != self.grid.shape:
            raise ValueError("BEV image shape differs from configured grid")
        if int(message.step) < int(message.width):
            raise ValueError("BEV image step is smaller than width")
        data = np.frombuffer(bytes(message.data), dtype=np.uint8)
        required = int(message.height) * int(message.step)
        if data.size < required:
            raise ValueError("BEV image data is incomplete")
        return np.ascontiguousarray(
            data[:required].reshape(int(message.height), int(message.step))[
                :, : int(message.width)
            ]
        )

    def _message(self, value, source_header):
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
        values = np.asarray(self.fusion_ms, dtype=np.float64)
        end_to_end = np.asarray(self.end_to_end_ms, dtype=np.float64)
        fps = 0.0
        if len(self.output_times) >= 2:
            elapsed = self.output_times[-1] - self.output_times[0]
            fps = (len(self.output_times) - 1) / max(elapsed, 1e-9)
        total_inputs = sum(self.inputs.values())
        rejection_rate = self.sync_rejected / max(1, total_inputs)
        status = DiagnosticStatus()
        status.name = "drivable_bev/fusion"
        status.hardware_id = "camera-bev"
        if self.fusion_errors:
            status.level = DiagnosticStatus.ERROR
            status.message = "fusion errors"
        elif self.outputs == 0 or rejection_rate >= 0.05:
            status.level = DiagnosticStatus.WARN
            status.message = "waiting for synchronized views"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "fusion active"
        items = {
            "inputs_by_view": self.inputs,
            "outputs": self.outputs,
            "queue_drops_by_view": self.queue_drops,
            "pair_errors_by_view": self.pair_errors,
            "sync_rejected": self.sync_rejected,
            "sync_rejection_rate": rejection_rate,
            "output_throttled": self.output_throttled,
            "fusion_errors": self.fusion_errors,
            "processing_fps": fps,
            "fusion_ms_mean": float(values.mean()) if values.size else 0.0,
            "fusion_ms_p95": float(np.percentile(values, 95.0)) if values.size else 0.0,
            "camera_to_fused_ms_p50": (
                float(np.percentile(end_to_end, 50.0))
                if end_to_end.size
                else "unavailable"
            ),
            "camera_to_fused_ms_p95": (
                float(np.percentile(end_to_end, 95.0))
                if end_to_end.size
                else "unavailable"
            ),
            "last_timestamp_skew_ms": self.last_skew_ms,
            "max_timestamp_skew_ms": self.max_skew_ns / 1e6,
            "max_fused_hz": self.max_fused_hz,
            "last_source_stamp_ns": self.last_source_stamp_ns,
            "timestamp_resets": self.timestamp_resets,
            "last_result_stamp_ns": self.last_result_stamp_ns,
            "ground_plane_frame": self.ground_plane.frame_id,
            "ground_plane_coefficients": self.ground_plane.coefficients,
            "ground_plane_source": self.ground_plane.source,
            "calibration_sha256": self.calibration_sha256,
            "grid_shape": "{},{}".format(self.grid.height_px, self.grid.width_px),
            "x_range_m": "{:.1f},{:.1f}".format(self.grid.x_min_m, self.grid.x_max_m),
            "y_range_m": "{:.1f},{:.1f}".format(self.grid.y_min_m, self.grid.y_max_m),
        }
        status.values = [KeyValue(key=str(k), value=str(v)) for k, v in items.items()]
        message = DiagnosticArray()
        message.header.stamp = rospy.Time.now()
        message.status = [status]
        self.diagnostics_publisher.publish(message)


def main() -> None:
    rospy.init_node("camera_bev_fusion")
    CameraBevFusionNode()
    rospy.spin()


if __name__ == "__main__":
    main()
