#!/usr/bin/env python3
"""Publish origin-aligned legacy MGeo around Ego for visual review only."""

from __future__ import annotations

import math
from pathlib import Path
import time

from geometry_msgs.msg import Point
from morai_msgs.msg import EgoVehicleStatus
import rospy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
import yaml

from vip3_hd_map.frames import LocalMapFrame, origin_translation
from vip3_hd_map.io_mgeo import load_json, resolve_source, sha256_file
from vip3_hd_map.map_overlay import SegmentLayer, grouped_lane_layers


COLORS = {
    "white": (0.95, 0.95, 0.95, 0.95),
    "yellow": (1.00, 0.82, 0.05, 0.98),
    "blue": (0.10, 0.45, 1.00, 0.90),
    "unknown": (0.65, 0.65, 0.65, 0.65),
}


def _load_route(path: Path) -> list[dict]:
    points = []
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            values = line.strip().replace(",", " ").split()
            if len(values) >= 2:
                points.append([float(values[0]), float(values[1]), 0.0])
    if len(points) < 2:
        raise ValueError("competition route must contain at least two points")
    return [{"idx": "competition_route", "points": points}]


def _point(x: float, y: float, z: float) -> Point:
    result = Point()
    result.x, result.y, result.z = float(x), float(y), float(z)
    return result


def _delete_all(stamp) -> Marker:
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.header.stamp = stamp
    marker.action = Marker.DELETEALL
    return marker


def _line_marker(
    *, namespace: str, marker_id: int, segments, stamp, width_m: float, color
) -> Marker:
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.header.stamp = stamp
    marker.ns = namespace
    marker.id = marker_id
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = float(width_m)
    marker.color = ColorRGBA(*[float(value) for value in color])
    for segment in segments:
        marker.points.append(_point(segment[0, 0], segment[0, 1], 0.04))
        marker.points.append(_point(segment[1, 0], segment[1, 1], 0.04))
    return marker


class LegacyOriginOverlayNode:
    def __init__(self) -> None:
        config_path = Path(rospy.get_param("~config_path")).resolve()
        with config_path.open("r", encoding="utf-8") as stream:
            config = yaml.safe_load(stream)
        configured_root = str(rospy.get_param("~repo_root", "")).strip()
        if configured_root:
            self.repo_root = Path(configured_root).resolve()
        else:
            self.repo_root = next(
                (
                    parent
                    for parent in config_path.parents
                    if (parent / "R_KR_PG_KATRI").is_dir()
                    and (parent / "src").is_dir()
                ),
                None,
            )
            if self.repo_root is None:
                raise ValueError(
                    "repository root was not found from config_path; set ~repo_root"
                )
        sources = config["sources"]
        current_global_path = resolve_source(
            self.repo_root, sources["current"]["global_info"]
        )
        legacy_global_path = resolve_source(
            self.repo_root, sources["legacy"]["global_info"]
        )
        current_frame = LocalMapFrame.from_global_info(load_json(current_global_path))
        legacy_frame = LocalMapFrame.from_global_info(load_json(legacy_global_path))
        translation = origin_translation(legacy_frame, current_frame)

        legacy_lane_path = resolve_source(
            self.repo_root, sources["legacy"]["lane_boundary_set"]
        )
        legacy_link_path = resolve_source(
            self.repo_root, sources["legacy"]["link_set"]
        )
        current_link_path = resolve_source(
            self.repo_root, sources["current"]["link_set"]
        )
        route_path = resolve_source(self.repo_root, sources["current"]["route"])

        self.legacy_lanes = grouped_lane_layers(
            load_json(legacy_lane_path), translation[:2]
        )
        self.legacy_links = SegmentLayer.from_features(
            load_json(legacy_link_path), translation[:2]
        )
        self.current_links = SegmentLayer.from_features(load_json(current_link_path))
        self.route = SegmentLayer.from_features(_load_route(route_path))
        self.source_hashes = {
            "legacy_lane": sha256_file(legacy_lane_path),
            "legacy_link": sha256_file(legacy_link_path),
            "current_link": sha256_file(current_link_path),
            "route": sha256_file(route_path),
        }

        self.radius_m = float(rospy.get_param("~local_radius_m", 60.0))
        self.max_hz = float(rospy.get_param("~max_publish_hz", 5.0))
        if self.radius_m <= 0.0 or self.max_hz <= 0.0:
            raise ValueError("local_radius_m and max_publish_hz must be positive")
        self.minimum_period_s = 1.0 / self.max_hz
        self.last_publish_monotonic = 0.0
        self.publish_legacy_links = bool(
            rospy.get_param("~publish_legacy_links", False)
        )

        self.publishers = {
            "legacy_lanes": rospy.Publisher(
                "/vip3_hd_map/debug/legacy_lanes", MarkerArray, queue_size=1
            ),
            "legacy_links": rospy.Publisher(
                "/vip3_hd_map/debug/legacy_link_centers", MarkerArray, queue_size=1
            ),
            "current_links": rospy.Publisher(
                "/vip3_hd_map/debug/current_link_centers", MarkerArray, queue_size=1
            ),
            "route": rospy.Publisher(
                "/vip3_hd_map/debug/competition_route", MarkerArray, queue_size=1
            ),
            "context": rospy.Publisher(
                "/vip3_hd_map/debug/ego_context", MarkerArray, queue_size=1
            ),
        }
        self.subscriber = rospy.Subscriber(
            str(rospy.get_param("~ego_topic", "/Ego_topic")),
            EgoVehicleStatus,
            self._on_ego,
            queue_size=1,
        )
        rospy.logwarn(
            "legacy MGeo overlay is review-only; origin translation=%s, "
            "fitted yaw=0, fitted scale=1, operational map unchanged",
            [round(float(value), 6) for value in translation],
        )
        rospy.loginfo(
            "legacy overlay ready radius=%.1fm rate<=%.1fHz route_sha256=%s",
            self.radius_m,
            self.max_hz,
            self.source_hashes["route"],
        )

    @staticmethod
    def _stamp(message):
        return message.header.stamp if message.header.stamp.to_nsec() > 0 else rospy.Time.now()

    @staticmethod
    def _has_subscriber(publisher) -> bool:
        return publisher.get_num_connections() > 0

    def _publish_layer(
        self, key: str, layer: SegmentLayer, pose, stamp, namespace, width, color
    ) -> None:
        publisher = self.publishers[key]
        if not self._has_subscriber(publisher):
            return
        local = layer.around_ego(pose[:2], pose[2], self.radius_m)
        publisher.publish(
            MarkerArray(
                markers=[
                    _delete_all(stamp),
                    _line_marker(
                        namespace=namespace,
                        marker_id=0,
                        segments=local,
                        stamp=stamp,
                        width_m=width,
                        color=color,
                    ),
                ]
            )
        )

    def _publish_lanes(self, pose, stamp) -> None:
        publisher = self.publishers["legacy_lanes"]
        if not self._has_subscriber(publisher):
            return
        markers = [_delete_all(stamp)]
        for marker_id, name in enumerate(("white", "yellow", "blue", "unknown")):
            local = self.legacy_lanes[name].around_ego(
                pose[:2], pose[2], self.radius_m
            )
            markers.append(
                _line_marker(
                    namespace="legacy_lane_" + name,
                    marker_id=marker_id,
                    segments=local,
                    stamp=stamp,
                    width_m=0.13,
                    color=COLORS[name],
                )
            )
        publisher.publish(MarkerArray(markers=markers))

    def _publish_context(self, stamp) -> None:
        publisher = self.publishers["context"]
        if not self._has_subscriber(publisher):
            return
        vehicle = Marker()
        vehicle.header.frame_id = "base_link"
        vehicle.header.stamp = stamp
        vehicle.ns = "ego_vehicle"
        vehicle.id = 0
        vehicle.type = Marker.CUBE
        vehicle.action = Marker.ADD
        vehicle.pose.position.x = 1.5275
        vehicle.pose.position.z = 0.025
        vehicle.pose.orientation.w = 1.0
        vehicle.scale.x = 4.635
        vehicle.scale.y = 1.892
        vehicle.scale.z = 0.05
        vehicle.color = ColorRGBA(0.15, 0.65, 1.0, 0.55)

        arrow = Marker()
        arrow.header = vehicle.header
        arrow.ns = "ego_direction"
        arrow.id = 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose.orientation.w = 1.0
        arrow.points = [_point(0.0, 0.0, 0.10), _point(5.0, 0.0, 0.10)]
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.10, 0.30, 0.30
        arrow.color = ColorRGBA(0.15, 0.65, 1.0, 0.95)
        publisher.publish(MarkerArray(markers=[_delete_all(stamp), vehicle, arrow]))

    def _on_ego(self, message: EgoVehicleStatus) -> None:
        now = time.monotonic()
        if now - self.last_publish_monotonic < self.minimum_period_s:
            return
        pose = (
            float(message.position.x),
            float(message.position.y),
            float(message.heading),
        )
        if not all(math.isfinite(value) for value in pose):
            rospy.logwarn_throttle(2.0, "legacy overlay ignored non-finite Ego pose")
            return
        stamp = self._stamp(message)
        self._publish_lanes(pose, stamp)
        if self.publish_legacy_links:
            self._publish_layer(
                "legacy_links", self.legacy_links, pose, stamp,
                "legacy_link_center", 0.05, (1.0, 0.45, 0.05, 0.55),
            )
        self._publish_layer(
            "current_links", self.current_links, pose, stamp,
            "current_link_center", 0.06, (0.05, 0.90, 0.90, 0.65),
        )
        self._publish_layer(
            "route", self.route, pose, stamp,
            "competition_route", 0.12, (1.0, 0.10, 0.85, 0.90),
        )
        self._publish_context(stamp)
        self.last_publish_monotonic = now


def main() -> None:
    rospy.init_node("legacy_origin_overlay")
    LegacyOriginOverlayNode()
    rospy.spin()


if __name__ == "__main__":
    main()
