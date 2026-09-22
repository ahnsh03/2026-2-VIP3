#!/usr/bin/env python3
"""4대 카메라 원본 RGB 를 지면 평면으로 투영해 한 장의 서라운드 BEV 로 합친다.

모델도 학습도 필요 없다. 카메라와 rosbridge 만 살아 있으면 돈다. 그래서 두 가지에 쓴다.

  1. **캘리브레이션 게이트.** 투영된 BEV 에서 차선·주차선이 직선으로 이어지지 않으면
     pitch 부호나 지면 z 가 틀린 것이다. 이 화면이 맞기 전에는 semantic BEV 를 믿지 않는다.
  2. 발표·점검용 그림. "4개 카메라가 붙었고 기하가 맞는다"를 한 장으로 보여준다.

ASMC 원본(`shadow_map_rgb_bev_node.py`)은 K-City 음영구간 pseudo-GT 를 겹쳐 그리는
노드였다. 그 부분은 가져오지 않았다.
"""

from __future__ import annotations

import threading
import time
from collections import deque
from functools import partial

import cv2
import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import CompressedImage

from drivable_bev.calibration import calibrations_from_dict
from drivable_bev.fusion import build_quality_maps
from drivable_bev.grid import BevGridSpec
from drivable_bev.projector import CameraBevProjector
from drivable_bev.rgb_bev import fuse_rgb_views, warp_rgb_to_bev


def _names(value):
    values = (
        [item.strip() for item in value.split(",")]
        if isinstance(value, str)
        else [str(item).strip() for item in value]
    )
    values = [item for item in values if item]
    if not values or len(values) != len(set(values)):
        raise ValueError("views must be non-empty and unique")
    return values


class RgbBevNode:
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
        cameras = calibrations_from_dict(
            {
                "ground_plane": rospy.get_param("~ground_plane"),
                "cameras": rospy.get_param("~cameras"),
            }
        )
        missing = sorted(set(self.views) - set(cameras))
        if missing:
            raise ValueError("camera calibration is missing views: {}".format(missing))
        self.cameras = {view: cameras[view] for view in self.views}

        max_range = float(rospy.get_param("~max_ground_range_m", 25.0))
        min_depth = float(rospy.get_param("~min_camera_depth_m", 0.10))
        self.projectors = {
            view: CameraBevProjector(
                self.cameras[view], self.grid, max_range, min_depth
            )
            for view in self.views
        }
        self.quality = build_quality_maps(
            self.projectors,
            self.grid,
            edge_taper_fraction=float(rospy.get_param("~edge_taper_fraction", 0.12)),
            near_full_quality_radius_m=float(
                rospy.get_param("~near_full_quality_radius_m", 6.0)
            ),
            max_quality_radius_m=float(
                rospy.get_param("~max_quality_radius_m", 12.0)
            ),
            boundary_weight=float(rospy.get_param("~boundary_weight", 0.2)),
        )

        self.max_publish_hz = float(rospy.get_param("~max_publish_hz", 5.0))
        if self.max_publish_hz <= 0.0:
            raise ValueError("max_publish_hz must be positive")
        self.publish_period = 1.0 / self.max_publish_hz
        self.jpeg_quality = int(rospy.get_param("~jpeg_quality", 90))
        self.draw_grid = bool(rospy.get_param("~draw_metric_grid", True))
        self.grid_step_m = float(rospy.get_param("~metric_grid_step_m", 1.0))
        self.sync_slop_ns = int(
            round(float(rospy.get_param("~camera_sync_slop_ms", 60.0)) * 1e6)
        )

        self.latest = {view: None for view in self.views}
        self.lock = threading.Lock()
        self.last_publish = 0.0
        self.decode_errors = 0
        self.outputs = 0
        self.render_ms = deque(maxlen=64)

        self.raw_publisher = rospy.Publisher(
            "/perception/bev/debug/rgb/compressed", CompressedImage, queue_size=1
        )
        self.diagnostics_publisher = rospy.Publisher(
            "/perception/bev/debug/rgb/diagnostics", DiagnosticArray, queue_size=1
        )
        self.subscribers = [
            rospy.Subscriber(
                self.cameras[view].image_topic,
                CompressedImage,
                partial(self._callback, view),
                queue_size=1,
                buff_size=16 * 1024 * 1024,
            )
            for view in self.views
        ]
        interval = max(0.5, float(rospy.get_param("~diagnostics_interval_sec", 2.0)))
        self.diagnostics_timer = rospy.Timer(
            rospy.Duration(interval), self._publish_diagnostics
        )
        coverage = np.count_nonzero(
            np.stack([value > 0.0 for value in self.quality.values()]), axis=0
        )
        rospy.loginfo(
            "RGB BEV ready views=%s grid=%dx%d union=%.4f overlap2=%.4f",
            self.views,
            self.grid.width_px,
            self.grid.height_px,
            float(np.mean(coverage > 0)),
            float(np.mean(coverage >= 2)),
        )

    def _callback(self, view: str, message: CompressedImage) -> None:
        if self.raw_publisher.get_num_connections() == 0:
            return
        stamp_ns = int(message.header.stamp.to_nsec())
        image = cv2.imdecode(
            np.frombuffer(bytes(message.data), dtype=np.uint8), cv2.IMREAD_COLOR
        )
        if image is None:
            self.decode_errors += 1
            rospy.logwarn_throttle(2.0, "%s RGB BEV JPEG decode failed", view)
            return
        camera = self.cameras[view]
        if image.shape[:2] != (camera.height, camera.width):
            rospy.logwarn_throttle(
                2.0,
                "%s image is %dx%d but calibration says %dx%d — sensor set drift",
                view,
                image.shape[1],
                image.shape[0],
                camera.width,
                camera.height,
            )
            return
        with self.lock:
            self.latest[view] = (stamp_ns, image)
        self._maybe_publish()

    def _maybe_publish(self) -> None:
        now = time.monotonic()
        if now - self.last_publish < self.publish_period:
            return
        with self.lock:
            if any(self.latest[view] is None for view in self.views):
                return
            stamps = [self.latest[view][0] for view in self.views]
            frames = {view: self.latest[view][1] for view in self.views}
        if self.sync_slop_ns > 0 and max(stamps) - min(stamps) > self.sync_slop_ns:
            rospy.logwarn_throttle(
                5.0,
                "RGB BEV views span %.1f ms > slop; showing anyway",
                (max(stamps) - min(stamps)) / 1e6,
            )
        self.last_publish = now

        started = time.perf_counter()
        try:
            warped = {
                view: warp_rgb_to_bev(
                    frames[view],
                    self.projectors[view].homography.bev_from_image,
                    self.grid.shape,
                )
                for view in self.views
            }
            fused, source_count = fuse_rgb_views(warped, self.quality)
            if self.draw_grid:
                fused = self._draw_metric_grid(fused)
        except Exception as exc:
            rospy.logerr_throttle(2.0, "RGB BEV fusion failed: %s", exc)
            return
        self.render_ms.append((time.perf_counter() - started) * 1000.0)
        self.outputs += 1

        message = CompressedImage()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.grid.frame_id
        message.format = "jpeg"
        message.data = np.array(
            cv2.imencode(
                ".jpg", fused, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
            )[1]
        ).tobytes()
        self.raw_publisher.publish(message)

    def _draw_metric_grid(self, image: np.ndarray) -> np.ndarray:
        canvas = image.copy()
        step = max(0.25, self.grid_step_m)
        colour = (70, 70, 70)
        x = math_ceil_to(self.grid.x_min_m, step)
        while x <= self.grid.x_max_m:
            row = int(round((self.grid.x_max_m - x) / self.grid.resolution_m))
            if 0 <= row < canvas.shape[0]:
                cv2.line(canvas, (0, row), (canvas.shape[1] - 1, row), colour, 1)
            x += step
        y = math_ceil_to(self.grid.y_min_m, step)
        while y <= self.grid.y_max_m:
            column = int(round((self.grid.y_max_m - y) / self.grid.resolution_m))
            if 0 <= column < canvas.shape[1]:
                cv2.line(canvas, (column, 0), (column, canvas.shape[0] - 1), colour, 1)
            y += step
        ego_column, ego_row = self.grid.ego_pixel
        cv2.circle(canvas, (int(ego_column), int(ego_row)), 4, (0, 0, 255), -1)
        cv2.putText(
            canvas,
            "base_link  grid {:.1f} m".format(step),
            (8, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        return canvas

    def _publish_diagnostics(self, _event) -> None:
        status = DiagnosticStatus(
            level=DiagnosticStatus.OK if self.outputs else DiagnosticStatus.WARN,
            name="drivable_bev/rgb",
            message="rendering" if self.outputs else "waiting for all views",
            hardware_id="cameras",
        )
        values = {
            "views": ",".join(self.views),
            "outputs": self.outputs,
            "decode_errors": self.decode_errors,
            "render_ms_mean": round(float(np.mean(self.render_ms)), 2)
            if self.render_ms
            else 0.0,
            "grid": "{}x{}".format(self.grid.width_px, self.grid.height_px),
            "resolution_m": self.grid.resolution_m,
        }
        status.values = [
            KeyValue(key=str(key), value=str(value)) for key, value in values.items()
        ]
        message = DiagnosticArray()
        message.header.stamp = rospy.Time.now()
        message.status = [status]
        self.diagnostics_publisher.publish(message)


def math_ceil_to(value: float, step: float) -> float:
    return step * np.ceil(value / step)


def main() -> None:
    rospy.init_node("rgb_bev")
    RgbBevNode()
    rospy.spin()


if __name__ == "__main__":
    main()
