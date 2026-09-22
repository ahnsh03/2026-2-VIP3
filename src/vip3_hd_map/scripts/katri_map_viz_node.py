#!/usr/bin/env python3
"""KATRI MGeo 를 RViz MarkerArray 로 띄운다.

커스텀 메시지를 쓰지 않는다 — 전부 stock `visualization_msgs/MarkerArray` 라 RViz 만
있으면 누구나 본다.

두 종류를 낸다.
  global : `map` frame, latched, 기동 시 한 번. 맵 전체 조망.
  local  : `base_link` frame, ego 를 따라가며 갱신. 주차 근접 확인용.

맵 디렉터리 이름이 한글+공백(`KATRI 맵 데이터 자료`)이라 런치 파일에 리터럴로 박지 않고
`~map_dir` 파라미터로 넘긴다.
"""

from __future__ import annotations

import os
import time
from pathlib import Path

import numpy as np
import rospy
from geometry_msgs.msg import Point, PoseStamped
from morai_msgs.msg import EgoVehicleStatus, GPSMessage
from sensor_msgs.msg import Imu
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from vip3_hd_map.map_overlay import SegmentLayer, grouped_lane_layers_by_type
from vip3_hd_map.mgeo_layers import (
    KatriMGeo,
    closed_polygon,
    lane_type_label,
    parking_space_polygon,
)

# lane_type -> (r, g, b, a, 선 굵기 m)
LANE_STYLE = {
    505: (0.95, 0.95, 0.95, 0.95, 0.15),
    501: (1.00, 0.82, 0.05, 0.98, 0.15),
    503: (0.95, 0.95, 0.95, 0.90, 0.15),
    530: (1.00, 0.10, 0.10, 0.95, 0.60),
    525: (0.60, 0.85, 1.00, 0.85, 0.15),
    506: (0.75, 0.55, 1.00, 0.80, 0.15),
    504: (0.10, 0.45, 1.00, 0.90, 0.15),
}
LANE_STYLE_DEFAULT = (0.65, 0.65, 0.65, 0.65, 0.15)

LINK_STYLE = (0.35, 0.35, 0.35, 0.60, 0.05)
SURFACE_STYLE = (1.00, 0.55, 0.15, 0.85, 0.10)
CROSSWALK_STYLE = (0.45, 1.00, 1.00, 0.85, 0.12)
PARKING_STYLE = (0.20, 1.00, 0.35, 0.95, 0.12)
EGO_STYLE = (0.10, 0.80, 1.00, 0.80, 0.08)


def _colour(style):
    return ColorRGBA(r=style[0], g=style[1], b=style[2], a=style[3])


class KatriMapVizNode:
    def __init__(self) -> None:
        config_path = Path(
            str(
                rospy.get_param(
                    "~config_path",
                    os.path.join(
                        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        "config",
                        "katri_map_sources.yaml",
                    ),
                )
            )
        )
        map_dir_override = str(rospy.get_param("~map_dir", "")).strip()
        started = time.monotonic()
        self.mgeo = KatriMGeo.from_config(config_path, map_dir_override)
        rospy.loginfo(
            "loaded KATRI MGeo from %s in %.1fs", self.mgeo.map_dir, time.monotonic() - started
        )
        if self.mgeo.missing_layers:
            rospy.logwarn("missing layers: %s", self.mgeo.missing_layers)

        missing = self.mgeo.missing_nonempty_files()
        if missing:
            rospy.logwarn(
                "원본 export 에 있었으나 전달되지 않은 파일 %d 개: %s — "
                "MORAI 에 요청할 것 (docs/katri-map.md §1)",
                len(missing),
                [row["file"] for row in missing],
            )

        self.map_frame = str(rospy.get_param("~map_frame", "map"))
        self.base_frame = str(rospy.get_param("~base_frame", "base_link"))
        self.radius_m = float(rospy.get_param("~local_radius_m", 30.0))
        self.max_hz = float(rospy.get_param("~max_publish_hz", 10.0))
        if self.max_hz <= 0.0:
            raise ValueError("max_publish_hz must be positive")
        self.min_period = 1.0 / self.max_hz
        self.publish_global = bool(rospy.get_param("~publish_global", True))
        self.pose_source = str(rospy.get_param("~pose_source", "ego_status"))
        if self.pose_source not in ("ego_status", "gps_imu"):
            raise ValueError("pose_source must be ego_status or gps_imu")
        self.pose_warn_m = float(rospy.get_param("~pose_distance_warning_m", 20.0))
        self.ego_length = float(rospy.get_param("~ego_length_m", 4.635))
        self.ego_width = float(rospy.get_param("~ego_width_m", 1.892))
        self.ego_rear_overhang = float(rospy.get_param("~ego_rear_overhang_m", 0.790))

        self.parking_spaces = self.mgeo.parking_spaces()
        rospy.loginfo(
            "parking spaces: %d (source=%s)",
            len(self.parking_spaces),
            self.mgeo.parking_space_source,
        )
        if not self.parking_spaces:
            rospy.logwarn(
                "주차면 기하가 없다. parking_space_set.json 을 받거나 "
                "config/vip3_parking_spaces.json 에 손으로 적어야 한다."
            )

        # 로컬 크롭용 세그먼트 레이어 (한 번만 만든다).
        self.lane_layers = grouped_lane_layers_by_type(self.mgeo.lane_boundaries())
        self.link_layer = SegmentLayer.from_features(self.mgeo.links())

        self.publishers = {}
        for scope in ("global", "local"):
            latch = scope == "global"
            for name in (
                "links",
                "lane_boundaries",
                "stop_lines",
                "surface_markings",
                "crosswalks",
                "parking_spaces",
            ):
                self.publishers[(scope, name)] = rospy.Publisher(
                    "/vip3_hd_map/{}/{}".format(scope, name),
                    MarkerArray,
                    queue_size=1,
                    latch=latch,
                )
        self.publishers[("local", "ego_context")] = rospy.Publisher(
            "/vip3_hd_map/local/ego_context", MarkerArray, queue_size=1
        )
        self.pose_publisher = rospy.Publisher(
            "/vip3_hd_map/debug/pose", PoseStamped, queue_size=1
        )

        self.last_publish = 0.0
        self.pose_checked = False
        self.gps_latest = None
        self.imu_latest = None

        if self.publish_global:
            self._publish_global()

        if self.pose_source == "ego_status":
            self.subscriber = rospy.Subscriber(
                str(rospy.get_param("~ego_topic", "/Ego_topic")),
                EgoVehicleStatus,
                self._ego_callback,
                queue_size=1,
            )
        else:
            self.subscriber = rospy.Subscriber(
                str(rospy.get_param("~gps_topic", "/gps")),
                GPSMessage,
                self._gps_callback,
                queue_size=1,
            )
            self.imu_subscriber = rospy.Subscriber(
                str(rospy.get_param("~imu_topic", "/imu")),
                Imu,
                self._imu_callback,
                queue_size=1,
            )
        rospy.loginfo(
            "katri map viz ready pose_source=%s radius=%.1fm max_hz=%.1f global=%s",
            self.pose_source,
            self.radius_m,
            self.max_hz,
            self.publish_global,
        )

    # -- marker helpers ----------------------------------------------------

    def _line_marker(self, namespace, index, frame, style, points, stamp, marker_type=None):
        marker = Marker()
        marker.header.frame_id = frame
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = int(index)
        marker.type = marker_type if marker_type is not None else Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(style[4])
        marker.color = _colour(style)
        marker.points = points
        return marker

    @staticmethod
    def _delete_all(frame, stamp):
        marker = Marker()
        marker.header.frame_id = frame
        marker.header.stamp = stamp
        marker.action = Marker.DELETEALL
        return marker

    @staticmethod
    def _segment_points(segments, z=0.05):
        points = []
        for segment in segments:
            points.append(Point(x=float(segment[0][0]), y=float(segment[0][1]), z=z))
            points.append(Point(x=float(segment[1][0]), y=float(segment[1][1]), z=z))
        return points

    @staticmethod
    def _polyline_points(xy, z=0.05, close=False):
        points = []
        count = len(xy)
        if count < 2:
            return points
        last = count if close else count - 1
        for index in range(last):
            a = xy[index]
            b = xy[(index + 1) % count]
            points.append(Point(x=float(a[0]), y=float(a[1]), z=z))
            points.append(Point(x=float(b[0]), y=float(b[1]), z=z))
        return points

    # -- global ------------------------------------------------------------

    def _publish_global(self) -> None:
        stamp = rospy.Time.now()

        links = MarkerArray()
        links.markers.append(self._delete_all(self.map_frame, stamp))
        points = self._segment_points(self.link_layer.segments_xy)
        if points:
            links.markers.append(
                self._line_marker("links", 0, self.map_frame, LINK_STYLE, points, stamp)
            )
        self.publishers[("global", "links")].publish(links)

        lanes = MarkerArray()
        stops = MarkerArray()
        lanes.markers.append(self._delete_all(self.map_frame, stamp))
        stops.markers.append(self._delete_all(self.map_frame, stamp))
        for index, (lane_type, layer) in enumerate(sorted(self.lane_layers.items())):
            if len(layer) == 0:
                continue
            slug, _ = lane_type_label(lane_type)
            style = LANE_STYLE.get(int(lane_type), LANE_STYLE_DEFAULT)
            marker = self._line_marker(
                "lb_{}_{}".format(lane_type, slug),
                index,
                self.map_frame,
                style,
                self._segment_points(layer.segments_xy),
                stamp,
            )
            (stops if int(lane_type) == 530 else lanes).markers.append(marker)
        self.publishers[("global", "lane_boundaries")].publish(lanes)
        self.publishers[("global", "stop_lines")].publish(stops)

        markings = MarkerArray()
        markings.markers.append(self._delete_all(self.map_frame, stamp))
        points = []
        for feature in self.mgeo.surface_markings():
            points.extend(self._polyline_points(closed_polygon(feature), close=True))
        if points:
            markings.markers.append(
                self._line_marker(
                    "surface_markings", 0, self.map_frame, SURFACE_STYLE, points, stamp
                )
            )
        self.publishers[("global", "surface_markings")].publish(markings)

        crosswalks = MarkerArray()
        crosswalks.markers.append(self._delete_all(self.map_frame, stamp))
        points = []
        for feature in self.mgeo.crosswalks():
            points.extend(self._polyline_points(closed_polygon(feature), close=True))
        if points:
            crosswalks.markers.append(
                self._line_marker(
                    "crosswalks", 0, self.map_frame, CROSSWALK_STYLE, points, stamp
                )
            )
        self.publishers[("global", "crosswalks")].publish(crosswalks)

        self.publishers[("global", "parking_spaces")].publish(
            self._parking_markers(self.map_frame, stamp, self.parking_spaces)
        )
        rospy.loginfo("published global map layers on /vip3_hd_map/global/*")

    def _parking_markers(self, frame, stamp, spaces, translation=None):
        array = MarkerArray()
        array.markers.append(self._delete_all(frame, stamp))
        points = []
        for space in spaces:
            # parking_space_polygon 이 이미 닫힌 폴리곤(N+1)을 주므로 close=False 다.
            # close=True 를 주면 마지막 변을 두 번 그린다.
            polygon = parking_space_polygon(space)
            if translation is not None:
                polygon = polygon + np.asarray(translation, dtype=np.float64)
            points.extend(self._polyline_points(polygon, z=0.06, close=False))
        if points:
            array.markers.append(
                self._line_marker(
                    "parking_spaces", 0, frame, PARKING_STYLE, points, stamp
                )
            )
        return array

    # -- pose --------------------------------------------------------------

    def _ego_callback(self, message: EgoVehicleStatus) -> None:
        position = message.position
        if position.x == 0.0 and position.y == 0.0 and position.z == 0.0:
            rospy.logwarn_throttle(5.0, "/Ego_topic position is (0,0,0) — ego 미스폰")
            return
        self._update((position.x, position.y), float(message.heading), message.header.stamp)

    def _gps_callback(self, message: GPSMessage) -> None:
        self.gps_latest = message

    def _imu_callback(self, message: Imu) -> None:
        self.imu_latest = message

    def _update(self, ego_xy, heading_deg, stamp) -> None:
        if not self.pose_checked:
            self.pose_checked = True
            distance = self.link_layer.nearest_distance_m(ego_xy)
            if distance > self.pose_warn_m:
                rospy.logwarn(
                    "첫 ego pose (%.1f, %.1f) 가 가장 가까운 링크에서 %.1f m 떨어져 있다. "
                    "/Ego_topic 의 원점이 MGeo local frame 과 다를 수 있다 "
                    "(docs/katri-map.md §3).",
                    ego_xy[0],
                    ego_xy[1],
                    distance,
                )
            else:
                rospy.loginfo(
                    "ego pose 가 링크에서 %.2f m — MGeo 원점과 일치한다", distance
                )

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = float(ego_xy[0])
        pose.pose.position.y = float(ego_xy[1])
        yaw = np.radians(heading_deg)
        pose.pose.orientation.z = float(np.sin(0.5 * yaw))
        pose.pose.orientation.w = float(np.cos(0.5 * yaw))
        self.pose_publisher.publish(pose)

        now = time.monotonic()
        if now - self.last_publish < self.min_period:
            return
        self.last_publish = now
        self._publish_local(ego_xy, heading_deg, stamp)

    # -- local -------------------------------------------------------------

    def _publish_local(self, ego_xy, heading_deg, stamp) -> None:
        lanes = MarkerArray()
        stops = MarkerArray()
        lanes.markers.append(self._delete_all(self.base_frame, stamp))
        stops.markers.append(self._delete_all(self.base_frame, stamp))
        for index, (lane_type, layer) in enumerate(sorted(self.lane_layers.items())):
            segments = layer.around_ego(ego_xy, heading_deg, self.radius_m)
            if len(segments) == 0:
                continue
            slug, _ = lane_type_label(lane_type)
            style = LANE_STYLE.get(int(lane_type), LANE_STYLE_DEFAULT)
            marker = self._line_marker(
                "lb_{}_{}".format(lane_type, slug),
                index,
                self.base_frame,
                style,
                self._segment_points(segments),
                stamp,
            )
            (stops if int(lane_type) == 530 else lanes).markers.append(marker)
        self.publishers[("local", "lane_boundaries")].publish(lanes)
        self.publishers[("local", "stop_lines")].publish(stops)

        links = MarkerArray()
        links.markers.append(self._delete_all(self.base_frame, stamp))
        segments = self.link_layer.around_ego(ego_xy, heading_deg, self.radius_m)
        if len(segments):
            links.markers.append(
                self._line_marker(
                    "links", 0, self.base_frame, LINK_STYLE,
                    self._segment_points(segments), stamp,
                )
            )
        self.publishers[("local", "links")].publish(links)

        self.publishers[("local", "ego_context")].publish(
            self._ego_context(stamp)
        )

    def _ego_context(self, stamp):
        array = MarkerArray()
        array.markers.append(self._delete_all(self.base_frame, stamp))
        # base_link = 뒷바퀴축 중심. 후진 주차에서 실제로 부딪히는 곳은 후범퍼다.
        rear = -self.ego_rear_overhang
        front = self.ego_length - self.ego_rear_overhang
        half = 0.5 * self.ego_width
        footprint = np.asarray(
            [[front, half], [front, -half], [rear, -half], [rear, half]],
            dtype=np.float64,
        )
        array.markers.append(
            self._line_marker(
                "ego_footprint", 0, self.base_frame, EGO_STYLE,
                self._polyline_points(footprint, z=0.02, close=True), stamp,
            )
        )
        return array


def main() -> None:
    rospy.init_node("katri_map_viz")
    KatriMapVizNode()
    rospy.spin()


if __name__ == "__main__":
    main()
