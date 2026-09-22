#!/usr/bin/env python3
"""Visualize per-view camera BEV layers and camera-ground calibration grids."""

from __future__ import annotations

import threading
import time
from functools import partial

import cv2
import message_filters
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage, Image

from drivable_bev.calibration import calibrations_from_dict
from drivable_bev.grid import BevGridSpec
from drivable_bev.visualization import (
    build_bev_mosaic,
    crop_bev_layers,
    draw_ground_grid_overlay,
    render_fused_bev,
    render_fused_quality,
)


def _enabled_views(value):
    if isinstance(value, str):
        values = [item.strip() for item in value.split(",")]
    else:
        values = [str(item).strip() for item in value]
    return [item for item in values if item]


class CameraBevVisualizerNode:
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
        self.display_grid = BevGridSpec(
            x_min_m=float(rospy.get_param("~display_x_min_m", self.grid.x_min_m)),
            x_max_m=float(rospy.get_param("~display_x_max_m", self.grid.x_max_m)),
            y_min_m=float(rospy.get_param("~display_y_min_m", self.grid.y_min_m)),
            y_max_m=float(rospy.get_param("~display_y_max_m", self.grid.y_max_m)),
            resolution_m=self.grid.resolution_m,
            frame_id=self.grid.frame_id,
        )
        camera_values = rospy.get_param("~cameras")
        all_cameras = calibrations_from_dict(
            {
                "ground_plane": rospy.get_param("~ground_plane"),
                "cameras": camera_values,
            }
        )
        self.views = _enabled_views(
            rospy.get_param("~enabled_views", ["front", "left", "right", "rear"])
        )
        self.cameras = {view: all_cameras[view] for view in self.views}
        # 모자이크 열 순서. 설정된 뷰만 남기고 나머지는 선언 순서를 따른다.
        preferred = ("left", "front", "right", "rear")
        self.view_order = [view for view in preferred if view in self.views]
        self.view_order += [v for v in self.views if v not in self.view_order]

        self.jpeg_quality = int(rospy.get_param("~jpeg_quality", 85))
        self.publish_per_view_mosaic = bool(
            rospy.get_param("~publish_per_view_mosaic", False)
        )
        self.fused_size = (
            int(rospy.get_param("~fused_width", 672)),
            int(rospy.get_param("~fused_height", 1050)),
        )
        if min(self.fused_size) <= 0:
            raise ValueError("fused visualization dimensions must be positive")
        self.display_label = str(rospy.get_param("~display_label", "FUSED"))
        self.tile_size = (
            int(rospy.get_param("~tile_width", 300)),
            int(rospy.get_param("~tile_height", 600)),
        )
        self.longitudinal_step_m = float(
            rospy.get_param("~camera_grid_longitudinal_step_m", 5.0)
        )
        self.lateral_step_m = float(
            rospy.get_param("~camera_grid_lateral_step_m", 2.0)
        )
        camera_grid_hz = float(rospy.get_param("~camera_grid_max_hz", 5.0))
        self.camera_grid_period = 1.0 / max(0.1, camera_grid_hz)
        self.last_grid_output = {view: 0.0 for view in self.views}

        self.secondary_head = str(rospy.get_param("~secondary_head", "lane"))
        if self.secondary_head not in ("lane", "road_marking"):
            raise ValueError(
                "secondary_head must be lane|road_marking, got {!r}".format(
                    self.secondary_head
                )
            )
        # lane 모드에서는 뷰별 토픽이 .../lane_probability 하나이고,
        # road_marking 모드에서는 class_id + confidence 두 장이다.
        self.secondary_layer = self.secondary_head
        self.mosaic_layers = ("drivable", self.secondary_layer, "coverage")

        self.layers = {}
        self.latest_stamp = rospy.Time()
        self.dirty = False
        self.fused_layers = None
        self.fused_stamp = rospy.Time()
        self.fused_dirty = False
        self.lock = threading.Lock()
        self.subscribers = []
        self.synchronizers = []
        self.grid_publishers = {}
        for view in self.views:
            namespace = "/perception/bev/debug/{}".format(view)
            if self.publish_per_view_mosaic:
                drivable = message_filters.Subscriber(
                    namespace + "/drivable_probability", Image, queue_size=2
                )
                secondary_topic = (
                    namespace + "/lane_probability"
                    if self.secondary_head == "lane"
                    else namespace + "/road_marking/class_id"
                )
                marking = message_filters.Subscriber(
                    secondary_topic, Image, queue_size=2
                )
                coverage = message_filters.Subscriber(
                    namespace + "/coverage", Image, queue_size=2
                )
                sync = message_filters.TimeSynchronizer(
                    [drivable, marking, coverage], queue_size=4
                )
                sync.registerCallback(partial(self._bev_callback, view))
                self.subscribers.extend([drivable, marking, coverage])
                self.synchronizers.append(sync)

            self.grid_publishers[view] = rospy.Publisher(
                namespace + "/camera_grid_overlay/compressed",
                CompressedImage,
                queue_size=1,
            )
            self.subscribers.append(
                rospy.Subscriber(
                    self.cameras[view].image_topic,
                    CompressedImage,
                    partial(self._camera_callback, view),
                    queue_size=1,
                    buff_size=16 * 1024 * 1024,
                )
            )

        fused_namespace = "/perception/bev/camera"
        if self.secondary_head == "lane":
            fused_secondary = [fused_namespace + "/lane_probability"]
        else:
            fused_secondary = [
                fused_namespace + "/road_marking/class_id",
                fused_namespace + "/road_marking/confidence",
            ]
        fused_topics = (
            [fused_namespace + "/drivable_probability"]
            + fused_secondary
            + [
                fused_namespace + "/coverage",
                fused_namespace + "/source_count",
            ]
        )
        fused_subscribers = [
            message_filters.Subscriber(topic, Image, queue_size=2)
            for topic in fused_topics
        ]
        fused_sync = message_filters.TimeSynchronizer(
            fused_subscribers, queue_size=4
        )
        fused_sync.registerCallback(self._fused_callback)
        self.subscribers.extend(fused_subscribers)
        self.synchronizers.append(fused_sync)
        self.fused_publisher = rospy.Publisher(
            "/perception/bev/debug/fused/compressed", CompressedImage, queue_size=1
        )
        self.quality_publisher = rospy.Publisher(
            "/perception/bev/debug/fused_quality/compressed",
            CompressedImage,
            queue_size=1,
        )
        fused_hz = float(rospy.get_param("~fused_max_hz", 5.0))
        self.fused_timer = rospy.Timer(
            rospy.Duration(1.0 / max(0.1, fused_hz)), self._publish_fused
        )

        self.mosaic_publisher = None
        self.mosaic_timer = None
        if self.publish_per_view_mosaic:
            self.mosaic_publisher = rospy.Publisher(
                "/perception/bev/debug/mosaic/compressed",
                CompressedImage,
                queue_size=1,
            )
            mosaic_hz = float(rospy.get_param("~mosaic_max_hz", 5.0))
            self.mosaic_timer = rospy.Timer(
                rospy.Duration(1.0 / max(0.1, mosaic_hz)), self._publish_mosaic
            )
        rospy.loginfo(
            "camera BEV visualizer ready views=%s fused=%sx%s per_view_mosaic=%s",
            self.views,
            self.fused_size[0],
            self.fused_size[1],
            self.publish_per_view_mosaic,
        )

    def _fused_callback(self, drivable: Image, *rest) -> None:
        # lane 모드: (secondary, coverage, source_count)
        # road_marking 모드: (class_id, confidence, coverage, source_count)
        try:
            layers = {
                "drivable": self._mono8_array(drivable, self.grid.shape),
                self.secondary_layer: self._mono8_array(rest[0], self.grid.shape),
                "coverage": self._mono8_array(rest[-2], self.grid.shape),
                "source_count": self._mono8_array(rest[-1], self.grid.shape),
            }
            if self.secondary_head == "road_marking":
                layers["confidence"] = self._mono8_array(rest[1], self.grid.shape)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "fused BEV visualization input invalid: %s", exc)
            return
        with self.lock:
            self.fused_layers = layers
            self.fused_stamp = drivable.header.stamp
            self.fused_dirty = True

    def _bev_callback(
        self, view: str, drivable: Image, marking: Image, coverage: Image
    ) -> None:
        try:
            layers = {
                "drivable": self._mono8_array(drivable, self.grid.shape),
                self.secondary_layer: self._mono8_array(marking, self.grid.shape),
                "coverage": self._mono8_array(coverage, self.grid.shape),
            }
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "%s BEV visualization input invalid: %s", view, exc)
            return
        with self.lock:
            self.layers[view] = layers
            self.latest_stamp = drivable.header.stamp
            self.dirty = True

    def _camera_callback(self, view: str, message: CompressedImage) -> None:
        if self.grid_publishers[view].get_num_connections() == 0:
            return
        now = time.monotonic()
        if now - self.last_grid_output[view] < self.camera_grid_period:
            return
        image = cv2.imdecode(
            np.frombuffer(bytes(message.data), dtype=np.uint8), cv2.IMREAD_COLOR
        )
        if image is None:
            rospy.logwarn_throttle(2.0, "%s calibration image decode failed", view)
            return
        try:
            overlay = draw_ground_grid_overlay(
                image,
                self.cameras[view],
                x_min_m=self.display_grid.x_min_m,
                x_max_m=self.display_grid.x_max_m,
                y_min_m=self.display_grid.y_min_m,
                y_max_m=self.display_grid.y_max_m,
                longitudinal_step_m=self.longitudinal_step_m,
                lateral_step_m=self.lateral_step_m,
            )
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "%s calibration overlay failed: %s", view, exc)
            return
        self.grid_publishers[view].publish(
            self._compressed(overlay, message.header.stamp, message.header.frame_id)
        )
        self.last_grid_output[view] = now

    def _publish_mosaic(self, _event) -> None:
        if (
            self.mosaic_publisher is None
            or self.mosaic_publisher.get_num_connections() == 0
        ):
            return
        with self.lock:
            if not self.dirty:
                return
            layers = dict(self.layers)
            stamp = self.latest_stamp
            self.dirty = False
        try:
            mosaic = build_bev_mosaic(
                layers,
                self.grid,
                self.view_order,
                self.tile_size,
                layer_names=self.mosaic_layers,
            )
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "BEV mosaic failed: %s", exc)
            return
        self.mosaic_publisher.publish(
            self._compressed(mosaic, stamp, self.grid.frame_id)
        )

    def _publish_fused(self, _event) -> None:
        publish_fused = self.fused_publisher.get_num_connections() > 0
        publish_quality = self.quality_publisher.get_num_connections() > 0
        if not publish_fused and not publish_quality:
            return
        with self.lock:
            if not self.fused_dirty or self.fused_layers is None:
                return
            layers = dict(self.fused_layers)
            stamp = self.fused_stamp
            self.fused_dirty = False
        try:
            layers = crop_bev_layers(layers, self.grid, self.display_grid)
            if publish_fused:
                image = render_fused_bev(
                    layers["drivable"],
                    layers["road_marking"],
                    layers["confidence"],
                    layers["coverage"],
                    layers["source_count"],
                    self.display_grid,
                    label_prefix=self.display_label,
                )
                image = cv2.resize(
                    image,
                    self.fused_size,
                    interpolation=cv2.INTER_NEAREST,
                )
                self.fused_publisher.publish(
                    self._compressed(image, stamp, self.grid.frame_id)
                )
            if publish_quality:
                quality = render_fused_quality(
                    layers["road_marking"],
                    layers["confidence"],
                    layers["coverage"],
                    layers["source_count"],
                    self.display_grid,
                    label_prefix=self.display_label.replace("FUSED", "QUALITY", 1),
                )
                quality = cv2.resize(
                    quality,
                    self.fused_size,
                    interpolation=cv2.INTER_NEAREST,
                )
                self.quality_publisher.publish(
                    self._compressed(quality, stamp, self.grid.frame_id)
                )
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "fused BEV render failed: %s", exc)
            return

    @staticmethod
    def _mono8_array(message: Image, expected_shape) -> np.ndarray:
        if message.encoding not in ("mono8", "8UC1"):
            raise ValueError("expected mono8, got {}".format(message.encoding))
        if (int(message.height), int(message.width)) != tuple(expected_shape):
            raise ValueError(
                "expected shape {}, got {}".format(
                    tuple(expected_shape), (int(message.height), int(message.width))
                )
            )
        data = np.frombuffer(bytes(message.data), dtype=np.uint8)
        required = int(message.height) * int(message.step)
        if message.step < message.width or data.size < required:
            raise ValueError("invalid image step/data length")
        return np.ascontiguousarray(
            data[:required].reshape(int(message.height), int(message.step))[
                :, : int(message.width)
            ]
        )

    def _compressed(
        self, image: np.ndarray, stamp, frame_id: str
    ) -> CompressedImage:
        ok, payload = cv2.imencode(
            ".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        )
        if not ok:
            raise RuntimeError("JPEG encoding failed")
        message = CompressedImage()
        message.header.stamp = stamp
        message.header.frame_id = frame_id
        message.format = "jpeg"
        message.data = payload.tobytes()
        return message


def main() -> None:
    rospy.init_node("camera_bev_visualizer")
    CameraBevVisualizerNode()
    rospy.spin()


if __name__ == "__main__":
    main()
