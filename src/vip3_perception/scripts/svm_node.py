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

from vip3_perception.svm.pose import PoseSample, projection_pose, source_quaternion
from vip3_perception.svm.ground import GroundRefiner

from vip3_perception.svm.projection import FisheyeSvm
from vip3_perception.svm.frames import VIEWS, select_frames
from vip3_perception.svm.frames import EncodedCameraFrame, SelectedFrameDecoder
from vip3_perception.svm.visualization import VehicleOverlay, decorate


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
        self.decoder = SelectedFrameDecoder(self.config['cameras'])
        self.lock = threading.Lock()
        self.wake = threading.Event()
        self.stopping = threading.Event()
        self.poses = deque(maxlen=200)
        self.pose_config = self.config.get('pose_compensation', {})
        self.ground = GroundRefiner(self.config.get('ground_refinement', {}))
        self.pose_status = 'disabled'
        self.last_key = None
        self.saved = False
        self.save_dir = rospy.get_param("~save_dir", "")
        self.lens_label = {
            'ground_anchor_validated': 'LENS FIT',
            'simulator_edge_validated_preview': 'EDGE FIT | PREVIEW',
            'simulator_multiview_preview': 'EGO CAL | PREVIEW',
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
        rate = float(self.config['publish_hz'])
        if not np.isfinite(rate) or rate <= 0:
            raise ValueError('publish_hz must be finite and positive')
        self.period = 1./rate
        rospy.logwarn("SVM PREVIEW: profile=%s; verify all overlaps, slopes and moving poses before parking use",
                      self.config.get('calibration_profile', 'unspecified'))
        rospy.loginfo("SVM geometry: ground z=%.4fm, center x=%.4fm, body mask=%s; source=%s",
                      self.config['ground_z_m'], self.config['center_x_m'], self.config['vehicle_mask'],
                      self.config.get('ground_plane', {}).get('source', 'historical_config'))
        self.worker = threading.Thread(target=self.run, name='svm_processor', daemon=True)
        rospy.on_shutdown(self.stop)
        self.worker.start()

    def run(self):
        """Wake on input; cap output rate without waiting for a free-running timer.

        One worker owns decode/maps/display. Incoming frames remain compressed
        in bounded buffers while processing; there is no queue of render jobs.
        The timeout also invalidates the display when all cameras stop.
        """
        next_allowed = 0.
        while not self.stopping.is_set():
            if self.stopping.wait(max(0., next_allowed-time.monotonic())):
                break
            self.wake.wait(timeout=min(self.period, .1))
            self.wake.clear()
            if self.stopping.is_set():
                break
            started = time.monotonic()
            try:
                published = self.tick()
            except Exception as exc:
                if not rospy.is_shutdown():
                    rospy.logerr('SVM processor stopped: %s', exc)
                    rospy.signal_shutdown('SVM processing failed')
                break
            if published:
                # Keep the rate deadline anchored: adding a period to the
                # actual wake time would accumulate scheduler jitter and turn
                # a 10Hz camera into an ever-older 9.9Hz stream.
                if next_allowed and started < next_allowed+self.period:
                    next_allowed += self.period
                else:
                    next_allowed = started+self.period

    def stop(self):
        self.stopping.set()
        self.wake.set()
        if threading.current_thread() is not self.worker:
            self.worker.join(timeout=2.)

    def receive(self, view, msg):
        arrived = time.monotonic()
        cam = self.config["cameras"][view]
        if msg.header.frame_id != cam["frame_id"]:
            rospy.logwarn_throttle(3, "%s frame mismatch: %s", view, msg.header.frame_id)
            return
        # Callback stays cheap. Decode only the four selected images in tick(),
        # so skipped frames cannot queue expensive JPEG work behind newer input.
        frame = EncodedCameraFrame(bytes(msg.data), msg.header.stamp.to_sec(), arrived, msg.header.frame_id)
        with self.lock:
            if self.buffers[view] and frame.stamp < self.buffers[view][-1].stamp:
                for buf in self.buffers.values():
                    buf.clear()
                self.poses.clear()
            self.buffers[view].append(frame)
        self.wake.set()

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

    def publish_status(self, valid, reason, stamp=None, timing=None):
        self.status.publish(String(data=json.dumps(dict(
            input_valid=valid, pose_status=reason, pose_compensated=(valid and reason == "pose compensated"),
            calibration_status=self.config.get('calibration_status', 'unverified'),
            calibration_profile=self.config.get('calibration_profile', 'unspecified'),
            projection_surface_offset_m=self.config.get('projection_surface_offset_m', 0.),
            ground_refinement=self.ground.status,
            ground_assumption=self.pose_config.get('ground_assumption', 'static configuration'),
            stamp=stamp, ground_plane=self.svm.active_plane, timing=timing))))

    def send_mask(self, stamp, valid=True):
        msg = Image()
        msg.header.stamp, msg.header.frame_id = stamp, 'base_link'
        msg.height = msg.width = self.svm.size
        msg.encoding, msg.step = 'mono8', self.svm.size
        msg.data = (self.svm.valid_mask if valid else np.zeros_like(self.svm.valid_mask)).tobytes()
        self.mask.publish(msg)

    @staticmethod
    def send(pub, image, stamp):
        if not pub.get_num_connections():
            return
        ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 90])
        if not ok:
            raise RuntimeError("JPEG encoding failed")
        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.header.frame_id = "base_link"
        msg.format = "jpeg"
        msg.data = encoded.tobytes()
        pub.publish(msg)

    def invalidate(self, reason):
        self.ground.reset()
        stamp = rospy.Time.now()
        if any(p.get_num_connections() for p in (self.panel, self.panel_raw, self.preview)):
            panel = np.zeros((580, 1180, 3), np.uint8)
            cv2.putText(panel, 'SVM: ' + reason, (20, 80), cv2.FONT_HERSHEY_SIMPLEX,
                        .7, (0, 150, 255), 2)
            self.send(self.panel, panel, stamp)
            self.send_raw_panel(panel, stamp)
            if self.preview.get_num_connections():
                self.send(self.preview, cv2.resize(panel, (self.svm.size, self.svm.size)), stamp)
        self.send_mask(stamp, valid=False)
        self.publish_status(False, reason)
        self.last_key = None

    def decorate(self, projection, compensated, grid=False):
        image = decorate(projection, self.svm, self.vehicle_overlay, compensated,
                         self.lens_label, self.show_vehicle_mask, grid)
        if self.ground.enabled:
            label = 'GROUND ' + ('VISION' if self.ground.status['source'].startswith('vision') else 'ASSUMED')
            cv2.putText(image, label, (10, self.svm.size-65), cv2.FONT_HERSHEY_SIMPLEX,
                        .45, (0, 200, 255), 1, cv2.LINE_AA)
        return image

    def tick(self):
        started = time.perf_counter()
        with self.lock:
            buffers = {v: list(self.buffers[v]) for v in VIEWS}
        selected_at = time.monotonic()
        frames, clock, span = select_frames(buffers, selected_at, self.max_age, self.slop)
        if frames is None:
            rospy.logwarn_throttle(5, 'SVM %s; received buffers %s', clock,
                                   {v: len(b) for v, b in buffers.items()})
            self.invalidate(clock)
            return False
        key = tuple(frames[v].arrival for v in VIEWS)
        if key == self.last_key:
            return False
        oldest_arrival = min(f.arrival for f in frames.values())
        timing = dict(clock='local_monotonic_receive', input_age_ms=(selected_at-oldest_arrival)*1000)
        decode_started = time.perf_counter()
        try:
            images = self.decoder.decode(frames)
        except ValueError as exc:
            rospy.logwarn_throttle(3, 'SVM %s', exc)
            self.invalidate(str(exc))
            return False
        timing['decode_ms'] = (time.perf_counter()-decode_started)*1000
        # A clock reset during decode must not publish a pre-reset bundle.
        with self.lock:
            replaced = any(not any(f is frames[v] for f in self.buffers[v]) for v in VIEWS)
            poses = list(self.poses)
        if replaced:
            self.invalidate('input replaced/reset during decode')
            return False
        if time.monotonic()-oldest_arrival > self.max_age:
            self.invalidate('input expired during decode')
            return False
        self.last_key = key
        pose_started = time.perf_counter()
        compensation = None
        self.pose_status = 'disabled'
        if self.pose_config.get('enabled', False):
            compensation, self.pose_status = projection_pose(
                poses, frames, time.monotonic(), self.pose_config, -self.config['ground_z_m'],
                float(self.config.get('projection_surface_offset_m', 0.)))
        timing['pose_ms'] = (time.perf_counter()-pose_started)*1000
        maps_started = time.perf_counter()
        if compensation is None:
            self.ground.plane_for(None, time.monotonic())
            rebuilt = self.svm.set_projection()
        else:
            plane = self.ground.plane_for(compensation, time.monotonic())
            rebuilt = self.svm.set_projection(plane, compensation['motion'])
        timing['maps_ms'] = (time.perf_counter()-maps_started)*1000
        timing['maps_rebuilt'] = rebuilt
        ground_started = time.perf_counter()
        refined = self.ground.refine(self.svm, images, compensation, time.monotonic())
        if refined is not None:
            timing['maps_rebuilt'] |= self.svm.set_projection(refined, compensation['motion'])
        timing['ground_refinement_ms'] = (time.perf_counter()-ground_started)*1000
        saving = bool(self.save_dir and not self.saved)
        want_preview = saving or bool(self.preview.get_num_connections())
        want_panel = saving or bool(self.panel.get_num_connections() or self.panel_raw.get_num_connections())
        warp_started = time.perf_counter()
        projection = self.svm.render(images, apply_vehicle_mask=(self.show_vehicle_mask or saving))
        display_projection = (self.svm.render(images, apply_vehicle_mask=False)
                              if saving and not self.show_vehicle_mask else projection)
        timing['warp_ms'] = (time.perf_counter()-warp_started)*1000
        display_started = time.perf_counter()
        image = self.decorate(display_projection, compensation is not None) if want_preview else None
        panel = None
        if want_panel:
            panel = np.zeros((580, 1180, 3), np.uint8)
            for i, view in enumerate(VIEWS):
                x, y = (i % 2) * 320, 50 + (i // 2) * 250
                panel[y:y + 240, x:x + 320] = cv2.resize(images[view], (320, 240))
                cv2.putText(panel, view.upper(), (x + 8, y + 22), cv2.FONT_HERSHEY_SIMPLEX,
                            .6, (0, 220, 255), 2, cv2.LINE_AA)
            panel[50:562, 660:1172] = self.decorate(display_projection, compensation is not None, grid=True)
            label = 'SVM | {} | sync {:.1f} ms | compute {:.1f} ms'.format(
                self.pose_status, span*1000, (time.perf_counter()-started)*1000)
            cv2.putText(panel, label, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 200, 255), 1, cv2.LINE_AA)
        timing['display_ms'] = (time.perf_counter()-display_started)*1000
        stamp = rospy.Time.from_sec(max(f.stamp for f in frames.values())) if clock == 'header' else rospy.Time.now()
        # No stale or pre-reset result should become valid merely because its
        # inputs were fresh when the expensive processing began.
        with self.lock:
            replaced = any(not any(f is frames[v] for f in self.buffers[v]) for v in VIEWS)
        if replaced or time.monotonic()-oldest_arrival > self.max_age:
            self.invalidate('input replaced/expired during processing')
            return False
        publish_started = time.perf_counter()
        if want_preview:
            self.send(self.preview, image, stamp)
        if want_panel:
            self.send(self.panel, panel, stamp)
            self.send_raw_panel(panel, stamp)
        self.send_mask(stamp)
        timing['publish_ms'] = (time.perf_counter()-publish_started)*1000
        timing['processing_ms'] = (time.perf_counter()-started)*1000
        timing['received_to_publish_ms'] = (time.monotonic()-oldest_arrival)*1000
        self.publish_status(True, self.pose_status, stamp.to_sec(), timing)
        rospy.loginfo_throttle(5, 'SVM %s | sync %.1fms | decode %.1fms | maps %.1fms | warp %.1fms | '
                              'total %.1fms | receive-to-publish %.1fms', self.pose_status, span*1000,
                              timing['decode_ms'], timing['maps_ms'], timing['warp_ms'],
                              timing['processing_ms'], timing['received_to_publish_ms'])
        if saving:
            self.save_capture(images, frames, poses, projection, display_projection,
                              image, panel, compensation, clock, span, timing)
        return True

    def save_capture(self, images, frames, poses, projection, display_projection,
                     image, panel, compensation, clock, span, timing):
        path = Path(self.save_dir)
        path.mkdir(parents=True, exist_ok=True)
        for view in VIEWS:
            cv2.imwrite(str(path / (view + '.png')), images[view])
        cv2.imwrite(str(path / 'svm_preview.png'), image)
        cv2.imwrite(str(path / 'svm_projection.png'), projection)
        if not self.show_vehicle_mask:
            cv2.imwrite(str(path / 'svm_unmasked.png'), display_projection)
        cv2.imwrite(str(path / 'svm_panel.png'), panel)
        cv2.imwrite(str(path / 'valid_mask.png'), self.svm.valid_mask)
        (path / 'config.yaml').write_text(yaml.safe_dump(self.config, sort_keys=False))
        motion = compensation['motion'] if compensation else {v: (np.eye(3), np.zeros(3)) for v in VIEWS}
        report = {'calibrated': False, 'calibration_status': self.config['calibration_status'],
                  'pose_quaternion_convention': 'ros_xyzw',
                  'projection_state': {'plane': self.svm.active_plane,
                      'motion': {v: {'rotation': r.tolist(), 'translation': t.tolist()}
                                 for v, (r, t) in motion.items()}}, 'clock': clock, 'span_ms': span*1000,
                  'display_vehicle_mask': self.show_vehicle_mask,
                  'vehicle_overlay': self.vehicle_overlay.metadata if self.vehicle_overlay else None,
                  'pose_status': self.pose_status, 'ground_plane': self.svm.active_plane,
                  'ground_refinement': self.ground.status,
                  'poses': [dict(stamp=p.stamp, position=p.position.tolist(),
                                 quaternion_xyzw=p.quaternion.tolist()) for p in poses],
                  'render_ms': timing['processing_ms'], 'timing': timing,
                  'frames': {v: {'stamp': f.stamp, 'frame_id': f.frame_id} for v, f in frames.items()}}
        (path / 'capture.json').write_text(json.dumps(report, indent=2)+'\n')
        self.saved = True
        rospy.loginfo('SVM actual input and preview saved: %s', path)

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
