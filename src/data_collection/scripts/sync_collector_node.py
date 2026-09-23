#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
프로파일 기반 스냅샷 수집기 (sync collector).

기본 모드 latest_timer: 토픽별 최신 메시지를 래치해 collect_hz 타이머에서 한 번에 찍는다.
카메라는 CompressedImage 바이트를 그대로 .jpg 로 저장한다 — **JPEG 손실이므로
Semantic 라벨 원본으로는 쓸 수 없다.** 학습 GT 는 capture_collector_node.py 쪽이다.

전송은 rosbridge ws://127.0.0.1:9090 하나뿐이다.

사용법:
  export VIP3_DATA=...
  roslaunch data_collection sync_collector.launch profile:=dry_run      # 첫 연결 확인
  roslaunch data_collection sync_collector.launch profile:=parking_core
"""

from __future__ import print_function

import os
import struct
import sys

import rospy
import rospkg

try:
    import numpy as np
except ImportError:
    np = None

from sensor_msgs.msg import CompressedImage, Imu, PointCloud2
from sensor_msgs import point_cloud2 as pc2
from morai_msgs.msg import (
    CollisionData,
    EgoVehicleStatus,
    GPSMessage,
    ObjectStatusList,
)

from data_collection.profile_loader import CAMERA_CHANNELS, load_profile
from data_collection.writer import DatasetWriter, default_vip3_data_root


def _stamp_ns(header):
    if header is None:
        return 0
    return int(header.stamp.secs) * 10**9 + int(header.stamp.nsecs)


def _vec3(v):
    return {"x": float(v.x), "y": float(v.y), "z": float(v.z)}


def ego_to_dict(msg):
    """morai_msgs/EgoVehicleStatus (26.R1) -> dict.

    단위: heading/front_steer_angle/rear_steer_angle 는 deg, angular_velocity 는 deg/s,
    velocity 는 m/s. CtrlCmd 의 front_steer/rear_steer(rad)와 섞지 말 것.
    26.R1 에는 wheel_angle 이 없다.
    """
    d = {
        "unique_id": int(msg.unique_id),
        "acceleration": _vec3(msg.acceleration),
        "position": _vec3(msg.position),
        "velocity": _vec3(msg.velocity),
        "heading_deg": float(msg.heading),
        "accel": float(msg.accel),
        "brake": float(msg.brake),
    }
    if hasattr(msg, "angular_velocity"):
        d["angular_velocity_dps"] = _vec3(msg.angular_velocity)
    if hasattr(msg, "front_steer_angle"):
        d["front_steer_angle_deg"] = float(msg.front_steer_angle)
    if hasattr(msg, "rear_steer_angle"):
        d["rear_steer_angle_deg"] = float(msg.rear_steer_angle)
    if hasattr(msg, "lateral_offset"):
        d["lateral_offset"] = float(msg.lateral_offset)
    return d


def objects_to_dict(msg):
    def one(o):
        return {
            "unique_id": int(o.unique_id),
            "type": int(getattr(o, "type", 0)),
            "name": getattr(o, "name", ""),
            "position": _vec3(o.position),
            "velocity": _vec3(o.velocity),
            "acceleration": _vec3(o.acceleration) if hasattr(o, "acceleration") else None,
            "heading": float(getattr(o, "heading", 0.0)),
            "size": _vec3(o.size) if hasattr(o, "size") else None,
        }

    return {
        "num_of_npcs": int(getattr(msg, "num_of_npcs", len(msg.npc_list))),
        "num_of_pedestrian": int(getattr(msg, "num_of_pedestrian", len(msg.pedestrian_list))),
        "num_of_obstacle": int(getattr(msg, "num_of_obstacle", len(msg.obstacle_list))),
        "npc_list": [one(o) for o in msg.npc_list],
        "pedestrian_list": [one(o) for o in msg.pedestrian_list],
        "obstacle_list": [one(o) for o in msg.obstacle_list],
    }


def collision_to_dict(msg):
    # CollisionData fields vary; keep a shallow dump of known attrs
    out = {}
    for name in ("global_offset_x", "global_offset_y", "global_offset_z"):
        if hasattr(msg, name):
            out[name] = float(getattr(msg, name))
    if hasattr(msg, "collision_object") and msg.collision_object:
        out["objects"] = []
        for o in msg.collision_object:
            item = {"unique_id": int(getattr(o, "unique_id", -1))}
            if hasattr(o, "position"):
                item["position"] = _vec3(o.position)
            out["objects"].append(item)
    return out


def gps_to_dict(msg):
    return {
        "latitude": float(msg.latitude),
        "longitude": float(msg.longitude),
        "altitude": float(getattr(msg, "altitude", 0.0)),
        "eastOffset": float(getattr(msg, "eastOffset", 0.0)),
        "northOffset": float(getattr(msg, "northOffset", 0.0)),
    }


def imu_to_dict(msg):
    return {
        "orientation": {
            "x": float(msg.orientation.x),
            "y": float(msg.orientation.y),
            "z": float(msg.orientation.z),
            "w": float(msg.orientation.w),
        },
        "angular_velocity": _vec3(msg.angular_velocity),
        "linear_acceleration": _vec3(msg.linear_acceleration),
    }


def pointcloud_to_xyzi_bytes(msg):
    """x,y,z,intensity 를 float32 little-endian 으로 pack. (bytes, n) 반환.

    주의: MORAI Lidar3D 가 intensity 필드를 안 실어주면 read_points 가 0점을 돌려준다.
    첫 연동 때 `rostopic echo -n1 /velodyne_points | head -40` 으로 필드를 확인할 것.
    """
    if np is None:
        pts = list(pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True))
        buf = bytearray()
        for p in pts:
            x, y, z = p[0], p[1], p[2]
            inten = p[3] if len(p) > 3 else 0.0
            buf.extend(struct.pack("<ffff", float(x), float(y), float(z), float(inten)))
        return bytes(buf), len(pts)
    arr = list(pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True))
    if not arr:
        return b"", 0
    a = np.asarray(arr, dtype=np.float32)
    if a.ndim == 1:
        a = a.reshape(1, -1)
    if a.shape[1] < 4:
        pad = np.zeros((a.shape[0], 4 - a.shape[1]), dtype=np.float32)
        a = np.concatenate([a, pad], axis=1)
    a = a[:, :4].astype(np.float32, copy=False)
    return a.tobytes(order="C"), int(a.shape[0])


class SyncCollector(object):
    def __init__(self):
        rp = rospkg.RosPack()
        pkg = rp.get_path("data_collection")
        profile_name = rospy.get_param("~profile", "dry_run")
        profile_path = rospy.get_param(
            "~profile_file",
            os.path.join(pkg, "config", "profiles", "{}.yaml".format(profile_name)),
        )
        if not os.path.isfile(profile_path):
            rospy.logfatal("profile not found: %s", profile_path)
            sys.exit(1)

        self.profile = load_profile(profile_path)
        max_frames = int(rospy.get_param("~max_frames", 0))
        if max_frames > 0:
            self.profile["max_frames"] = max_frames
        collect_hz = float(rospy.get_param("~collect_hz", 0))
        if collect_hz > 0:
            self.profile["collect_hz"] = collect_hz
        # 프로파일 YAML 이 map/vehicle 의 기본값을 갖지만, 다른 맵·차량으로 한 번
        # 돌릴 때 YAML 을 고치지 않고 런치에서 덮어쓸 수 있어야 한다. 이 값은 run
        # 메타데이터에 그대로 기록되므로 틀리면 나중에 run 출처를 못 믿는다.
        overrides = {
            "map": str(rospy.get_param("~map", "")).strip(),
            "vehicle": str(rospy.get_param("~vehicle", "")).strip(),
        }
        for key, override in overrides.items():
            if override and override != self.profile.get(key):
                rospy.loginfo(
                    "profile %s: %s -> %s (launch override)",
                    key, self.profile.get(key), override,
                )
                self.profile[key] = override

        run_id = rospy.get_param("~run_id", "") or None
        data_root = rospy.get_param("~data_root", "") or default_vip3_data_root()
        repo_root = os.environ.get("VIP3") or os.path.dirname(os.path.dirname(pkg))

        self.writer = DatasetWriter(
            self.profile, run_id=run_id, data_root=data_root, repo_cwd=repo_root
        )
        rospy.loginfo(
            "collecting → %s (profile=%s hz=%.1f)",
            self.writer.root,
            self.profile["name"],
            self.profile["collect_hz"],
        )

        self._latest = {}
        self._subs = []
        ch = self.profile["channels"]
        topics = self.profile["topics"]

        def sub(key, msg_type, store_key=None):
            if not ch.get(key):
                return
            sk = store_key or key
            topic = topics[key]
            self._subs.append(
                rospy.Subscriber(
                    topic,
                    msg_type,
                    lambda m, k=sk: self._cb(k, m),
                    queue_size=1,
                )
            )
            rospy.loginfo("subscribe %s → %s", topic, sk)

        sub("ego", EgoVehicleStatus)
        sub("objects", ObjectStatusList)
        sub("collision", CollisionData)
        sub("lidar", PointCloud2)
        sub("gps", GPSMessage)
        sub("imu", Imu)
        for cam in CAMERA_CHANNELS:
            sub(cam, CompressedImage)

        self._stopping = False
        hz = max(0.1, float(self.profile["collect_hz"]))
        self._timer = rospy.Timer(rospy.Duration(1.0 / hz), self._on_tick)
        rospy.on_shutdown(self._shutdown)

    def _cb(self, key, msg):
        self._latest[key] = msg

    def _on_tick(self, _evt):
        if self._stopping:
            return
        max_frames = int(self.profile.get("max_frames") or 0)
        if max_frames > 0 and self.writer.frame_idx >= max_frames:
            rospy.loginfo("max_frames=%d reached — shutting down", max_frames)
            rospy.signal_shutdown("max_frames")
            return

        ch = self.profile["channels"]
        missing = []
        payload = {"missing": missing, "cams": {}}

        ego = self._latest.get("ego")
        if ch.get("ego"):
            if ego is None:
                if self.profile.get("require_ego", True):
                    self.writer._skipped["no_ego"] += 1
                    return
                missing.append("ego")
            else:
                payload["ego"] = ego_to_dict(ego)
                payload["sim_time"] = float(ego.header.stamp.to_sec()) if ego.header.stamp else rospy.get_time()
                payload["stamp_ns"] = _stamp_ns(ego.header)

        if payload.get("sim_time") is None:
            payload["sim_time"] = rospy.get_time()
            payload["stamp_ns"] = int(payload["sim_time"] * 1e9)

        for cam in CAMERA_CHANNELS:
            if not ch.get(cam):
                continue
            msg = self._latest.get(cam)
            if msg is None:
                missing.append(cam)
            else:
                payload["cams"][cam] = bytes(msg.data)

        if ch.get("lidar"):
            msg = self._latest.get("lidar")
            if msg is None:
                missing.append("lidar")
            else:
                blob, n = pointcloud_to_xyzi_bytes(msg)
                payload["lidar_xyzi"] = blob
                payload["lidar_n"] = n

        if ch.get("objects"):
            msg = self._latest.get("objects")
            if msg is None:
                missing.append("objects")
            else:
                payload["objects"] = objects_to_dict(msg)

        if ch.get("collision"):
            msg = self._latest.get("collision")
            if msg is None:
                missing.append("collision")
            else:
                payload["collision"] = collision_to_dict(msg)

        if ch.get("gps"):
            msg = self._latest.get("gps")
            if msg is None:
                missing.append("gps")
            else:
                payload["gps"] = gps_to_dict(msg)

        if ch.get("imu"):
            msg = self._latest.get("imu")
            if msg is None:
                missing.append("imu")
            else:
                payload["imu"] = imu_to_dict(msg)

        idx = self.writer.write_frame(payload)
        if idx % 20 == 0:
            rospy.loginfo(
                "frame %d missing=%s root=%s",
                idx,
                missing or "-",
                self.writer.root,
            )

    def _shutdown(self):
        self._stopping = True
        self.writer.close()
        rospy.loginfo(
            "closed run %s frames=%d skipped=%s",
            self.writer.run_id,
            self.writer.frame_idx,
            self.writer._skipped,
        )


def main():
    rospy.init_node("sync_collector")
    SyncCollector()
    rospy.spin()


if __name__ == "__main__":
    main()
