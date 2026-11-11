#!/usr/bin/env python3
"""Four-camera SVM preview with source images, freshness checks and capture."""
import json
from collections import deque
from functools import partial
from pathlib import Path
import threading
import time

import cv2
import numpy as np
import rospy
import yaml
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from drivable_bev.svm_pose import PoseSample, projection_pose, source_quaternion

from drivable_bev.fisheye_svm import CameraFrame, FisheyeSvm, VIEWS, select_frames
from drivable_bev.vehicle_overlay import VehicleOverlay


class SvmNode:
    def __init__(self):
        self.config = rospy.get_param("~svm")
        self.svm = FisheyeSvm(self.config)
        self.config = self.svm.config
        self.show_vehicle_mask = rospy.get_param("~vehicle_mask", True)
        self.vehicle_overlay = None
        if rospy.get_param("~vehicle_overlay", True):
            try:
                self.vehicle_overlay = VehicleOverlay(rospy.get_param("~vehicle_image", ""), self.config)
                rospy.loginfo("SVM display vehicle: %s", self.vehicle_overlay.metadata)
            except (ValueError, KeyError, cv2.error) as exc:
                rospy.logwarn("SVM vehicle overlay unavailable; using body rectangle: %s", exc)
        self.buffers = {v: deque(maxlen=10) for v in VIEWS}
        self.lock = threading.Lock()
        self.poses = deque(maxlen=200)
        self.pose_config = self.config.get('pose_compensation', {})
        self.pose_status = 'disabled'
        self.last_key = None
        self.saved = False
        self.save_dir = rospy.get_param("~save_dir", "")
        self.lens_label = {
            'ground_anchor_validated': 'LENS FIT',
            'simulator_edge_validated_preview': 'EDGE FIT | PREVIEW',
        }.get(self.config.get('calibration_status'), 'LENS UNVERIFIED')
        self.max_age = float(self.config["max_age_s"])
        self.slop = float(self.config["sync_slop_s"])
        self.preview = rospy.Publisher("/perception/svm/preview/compressed", CompressedImage, queue_size=1)
        self.panel = rospy.Publisher("/perception/svm/panel/compressed", CompressedImage, queue_size=1)
        self.panel_raw = rospy.Publisher("/perception/svm/panel", Image, queue_size=1)
        self.mask = rospy.Publisher("/perception/svm/valid_mask", Image, queue_size=1)
        self.subscribers = [rospy.Subscriber(self.config["cameras"][v]["topic"], CompressedImage,
                            partial(self.receive, v), queue_size=1, buff_size=4 * 1024 * 1024) for v in VIEWS]
        self.status = rospy.Publisher('/perception/svm/status', String, queue_size=1, latch=True)
        if self.pose_config.get('enabled', False):
            self.subscribers.append(rospy.Subscriber(self.pose_config.get('topic', '/tf'),
                                                    TFMessage, self.receive_pose, queue_size=20))
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.config["publish_hz"]), self.tick)
        rospy.logwarn("SVM PREVIEW: profile=%s; verify all overlaps, slopes and moving poses before parking use",
                      self.config.get('calibration_profile', 'unspecified'))
        rospy.loginfo("SVM geometry: ground z=%.4fm, center x=%.4fm, body mask=%s; source=%s",
                      self.config['ground_z_m'], self.config['center_x_m'], self.config['vehicle_mask'],
                      self.config.get('ground_plane', {}).get('source', 'historical_config'))

    def receive(self, view, msg):
        arrived = time.monotonic()
        image = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
        cam = self.config["cameras"][view]
        if image is None or image.shape != (cam["height"], cam["width"], 3):
            rospy.logwarn_throttle(3, "%s invalid JPEG or dimensions", view)
            return
        if msg.header.frame_id != cam["frame_id"]:
            rospy.logwarn_throttle(3, "%s frame mismatch: %s", view, msg.header.frame_id)
            return
        frame = CameraFrame(image, msg.header.stamp.to_sec(), arrived, msg.header.frame_id)
        with self.lock:
            if self.buffers[view] and frame.stamp < self.buffers[view][-1].stamp:
                for buf in self.buffers.values():
                    buf.clear()
                self.poses.clear()
            self.buffers[view].append(frame)

    def receive_pose(self, msg):
        parent = self.pose_config.get('parent_frame', 'map').lstrip('/')
        child = self.pose_config.get('child_frame', 'base_link').lstrip('/')
        for transform in msg.transforms:
            if (transform.header.frame_id.lstrip('/') != parent
                    or transform.child_frame_id.lstrip('/') != child):
                continue
            t, q = transform.transform.translation, transform.transform.rotation
            try:
                pose = PoseSample(transform.header.stamp.to_sec(), time.monotonic(),
                                  [t.x, t.y, t.z], source_quaternion([q.x, q.y, q.z, q.w],
                                      self.pose_config.get('quaternion_convention', 'ros_xyzw')))
            except ValueError as exc:
                rospy.logwarn_throttle(5, 'SVM invalid TF: %s', exc)
                continue
            with self.lock:
                if self.poses and pose.stamp < self.poses[-1].stamp:
                    self.poses.clear()
                    for buffer in self.buffers.values():
                        buffer.clear()
                self.poses.append(pose)

    def publish_status(self, valid, reason, stamp=None):
        self.status.publish(String(data=json.dumps(dict(
            input_valid=valid, pose_status=reason, pose_compensated=(valid and reason == "pose compensated"),
            calibration_status=self.config.get('calibration_status', 'unverified'),
            calibration_profile=self.config.get('calibration_profile', 'unspecified'),
            projection_surface_offset_m=self.config.get('projection_surface_offset_m', 0.),
            ground_assumption=self.pose_config.get('ground_assumption', 'static configuration'),
            stamp=stamp, ground_plane=self.svm.active_plane))))

    def send_mask(self, stamp, valid=True):
        msg = Image()
        msg.header.stamp, msg.header.frame_id = stamp, 'base_link'
        msg.height = msg.width = self.svm.size
        msg.encoding, msg.step = 'mono8', self.svm.size
        msg.data = (self.svm.valid_mask if valid else np.zeros_like(self.svm.valid_mask)).tobytes()
        self.mask.publish(msg)

    @staticmethod
    def send(pub, image, stamp):
        ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 90])
        if not ok:
            raise RuntimeError("JPEG encoding failed")
        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.header.frame_id = "base_link"
        msg.format = "jpeg"
        msg.data = encoded.tobytes()
        pub.publish(msg)

    def tick(self, _event):
        with self.lock:
            buffers = {v: list(self.buffers[v]) for v in VIEWS}
            poses = list(self.poses)
        frames, clock, span = select_frames(buffers, time.monotonic(), self.max_age, self.slop)
        if frames is None:
            rospy.logwarn_throttle(5, "SVM %s; received buffers %s", clock,
                                   {v: len(b) for v, b in buffers.items()})
            # Explicitly invalidate the visible panel when the simulator stops.
            panel = np.zeros((580, 1180, 3), np.uint8)
            cv2.putText(panel, "SVM: " + clock + " - waiting for four live cameras", (20, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 150, 255), 2)
            self.send(self.panel, panel, rospy.Time.now())
            self.send_raw_panel(panel, rospy.Time.now())
            self.send(self.preview, cv2.resize(panel, (self.svm.size, self.svm.size)), rospy.Time.now())
            self.send_mask(rospy.Time.now(), valid=False)
            self.publish_status(False, clock)
            self.last_key = None
            return
        key = tuple(frames[v].arrival for v in VIEWS)
        if key == self.last_key:
            return
        self.last_key = key
        started = time.perf_counter()
        images = {v: frames[v].image for v in VIEWS}
        compensation = None
        self.pose_status = 'disabled'
        if self.pose_config.get('enabled', False):
            compensation, self.pose_status = projection_pose(
                poses, frames, time.monotonic(), self.pose_config, -self.config['ground_z_m'],
                float(self.config.get('projection_surface_offset_m', 0.)))
        if compensation is None:
            self.svm.set_projection()
        else:
            self.svm.set_projection(compensation['plane'], compensation['motion'])
        projection = self.svm.render(images)
        display_projection = projection if self.show_vehicle_mask else self.svm.render(images, apply_vehicle_mask=False)
        image = self.display_image(display_projection)
        grid_image = self.display_image(self.svm.debug_image(display_projection))
        # Preview topic also carries a warning so it cannot silently become training input.
        for display in (image, grid_image):
            cv2.putText(display, "SVM PREVIEW | " + ("POSE OK" if compensation else "STATIC"), (8, self.svm.size - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 180, 255), 1, cv2.LINE_AA)
            cv2.putText(display, self.lens_label, (8, self.svm.size - 52),
                        cv2.FONT_HERSHEY_SIMPLEX, .48, (0, 180, 255), 1, cv2.LINE_AA)
            if not self.show_vehicle_mask:
                cv2.putText(display, "BODY MASK OFF | DEBUG", (8, self.svm.size - 34),
                            cv2.FONT_HERSHEY_SIMPLEX, .48, (0, 180, 255), 1, cv2.LINE_AA)
        stamp = rospy.Time.from_sec(max(f.stamp for f in frames.values())) if clock == "header" else rospy.Time.now()
        self.send(self.preview, image, stamp)
        panel = np.zeros((580, 1180, 3), np.uint8)
        for i, view in enumerate(VIEWS):
            x, y = (i % 2) * 320, 50 + (i // 2) * 250
            panel[y:y + 240, x:x + 320] = cv2.resize(images[view], (320, 240))
            cv2.putText(panel, view.upper(), (x + 8, y + 22), cv2.FONT_HERSHEY_SIMPLEX,
                        .6, (0, 220, 255), 2, cv2.LINE_AA)
        panel[50:562, 660:1172] = cv2.resize(grid_image, (512, 512))
        elapsed = (time.perf_counter() - started) * 1000
        label = "SVM | {} | sync {:.1f} ms | render {:.1f} ms".format(self.pose_status, span * 1000, elapsed)
        self.publish_status(True, self.pose_status, stamp.to_sec())
        rospy.loginfo_throttle(5, 'SVM %s | sync %.1fms | render %.1fms | lens=%s',
                              self.pose_status, span*1000, elapsed, self.config['calibration_status'])
        cv2.putText(panel, label, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 200, 255), 1, cv2.LINE_AA)
        self.send(self.panel, panel, stamp)
        self.send_raw_panel(panel, stamp)
        self.send_mask(stamp)
        if self.save_dir and not self.saved:
            path = Path(self.save_dir)
            path.mkdir(parents=True, exist_ok=True)
            for view in VIEWS:
                cv2.imwrite(str(path / (view + ".png")), images[view])
            cv2.imwrite(str(path / "svm_preview.png"), image)
            cv2.imwrite(str(path / "svm_projection.png"), projection)
            if not self.show_vehicle_mask:
                cv2.imwrite(str(path / "svm_unmasked.png"), display_projection)
            cv2.imwrite(str(path / "svm_panel.png"), panel)
            cv2.imwrite(str(path / "valid_mask.png"), self.svm.valid_mask)
            (path / "config.yaml").write_text(yaml.safe_dump(self.config, sort_keys=False))
            motion = compensation['motion'] if compensation else {v: (np.eye(3), np.zeros(3)) for v in VIEWS}
            report = {"calibrated": False,
                      "calibration_status": self.config['calibration_status'],
                      "pose_quaternion_convention": "ros_xyzw",
                      "projection_state": {"plane": self.svm.active_plane,
                          "motion": {v: {"rotation": r.tolist(), "translation": t.tolist()}
                                     for v, (r, t) in motion.items()}}, "clock": clock, "span_ms": span * 1000,
                      "display_vehicle_mask": self.show_vehicle_mask,
                      "vehicle_overlay": self.vehicle_overlay.metadata if self.vehicle_overlay else None,
                      "pose_status": self.pose_status, "ground_plane": self.svm.active_plane,
                      "poses": [dict(stamp=p.stamp, position=p.position.tolist(),
                                     quaternion_xyzw=p.quaternion.tolist()) for p in poses],
                      "render_ms": elapsed, "frames": {v: {"stamp": f.stamp, "frame_id": f.frame_id}
                                                         for v, f in frames.items()}}
            (path / "capture.json").write_text(json.dumps(report, indent=2) + "\n")
            self.saved = True
            rospy.loginfo("SVM actual input and preview saved: %s", path)

    def display_image(self, projection):
        return self.vehicle_overlay.apply(projection) if self.vehicle_overlay else projection.copy()

    def send_raw_panel(self, panel, stamp):
        if not self.panel_raw.get_num_connections():
            return
        msg = Image()
        msg.header.stamp, msg.header.frame_id = stamp, "base_link"
        msg.height, msg.width = panel.shape[:2]
        msg.encoding, msg.step = "bgr8", panel.shape[1] * 3
        msg.data = panel.tobytes()
        self.panel_raw.publish(msg)


if __name__ == "__main__":
    rospy.init_node("fisheye_svm_preview")
    node = SvmNode()
    rospy.spin()
