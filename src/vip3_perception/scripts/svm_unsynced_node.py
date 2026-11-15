#!/usr/bin/env python3
"""Four-camera SVM preview that accepts camera frames with arbitrary skew."""
import importlib.util
from pathlib import Path

import cv2
import numpy as np
import rospy
import rospkg
import time

from vip3_perception.svm.frames import EncodedCameraFrame, VIEWS

# catkin creates executable relay wrappers in devel/lib; importing `svm_node`
# would load that wrapper rather than the source module containing SvmNode.
svm_source = Path(rospkg.RosPack().get_path("vip3_perception")) / "scripts" / "svm_node.py"
spec = importlib.util.spec_from_file_location("_vip3_svm_node_impl", str(svm_source))
svm_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(svm_module)
SvmNode = svm_module.SvmNode


class UnsyncedSvmNode(SvmNode):
    def __init__(self):
        self.missing_frames = {}
        super().__init__()

    def fallback_frame(self, view, now):
        if view not in self.missing_frames:
            camera = self.config['cameras'][view]
            blank = np.zeros((camera['height'], camera['width'], 3), dtype=np.uint8)
            cv2.putText(blank, 'NO FRAME', (40, 80), cv2.FONT_HERSHEY_SIMPLEX,
                        2.0, (0, 200, 255), 4, cv2.LINE_AA)
            ok, encoded = cv2.imencode('.jpg', blank)
            if not ok:
                raise ValueError('could not create blank fallback for {}'.format(view))
            self.missing_frames[view] = EncodedCameraFrame(
                encoded.tobytes(), 0., now, camera['frame_id'])
        return self.missing_frames[view]

    def receive(self, view, msg):
        # Accept out-of-order stamps and normalize the frame ID for decoding.
        frame = EncodedCameraFrame(bytes(msg.data), msg.header.stamp.to_sec(),
                                   time.monotonic(), self.config['cameras'][view]['frame_id'])
        with self.lock:
            self.buffers[view].append(frame)
        self.wake.set()

    def decode_frames(self, frames):
        # Substitute a labeled blank for malformed JPEGs instead of invalidating the view.
        for _ in VIEWS:
            try:
                return self.decoder.decode(frames)
            except ValueError as exc:
                view = str(exc).split(maxsplit=1)[0]
                if view not in VIEWS:
                    raise
                frames[view] = self.fallback_frame(view, time.monotonic())
        return self.decoder.decode(frames)

    def select_frames(self, buffers, now):
        """Use each camera's newest received frame without age/sync limits."""
        selected = {}
        for view in VIEWS:
            if not buffers[view]:
                selected[view] = self.fallback_frame(view, now)
            else:
                selected[view] = max(buffers[view], key=lambda frame: frame.arrival)
        stamps = [frame.stamp for frame in selected.values()]
        if all(stamp > 0 for stamp in stamps):
            span = max(stamps) - min(stamps)
        else:
            arrivals = [frame.arrival for frame in selected.values()]
            span = max(arrivals) - min(arrivals)
        return selected, "arrival (age/sync ignored)", span

    def tick(self):
        """Render the latest available set without stale/sync invalidation gates."""
        started = time.perf_counter()
        with self.lock:
            buffers = {view: list(self.buffers[view]) for view in VIEWS}
            poses = list(self.poses)
        selected_at = time.monotonic()
        frames, clock, span = self.select_frames(buffers, selected_at)
        key = tuple(frames[view].arrival for view in VIEWS)
        if key == self.last_key:
            return False
        oldest_arrival = min(frame.arrival for frame in frames.values())
        timing = dict(clock='local_monotonic_receive',
                      input_age_ms=(selected_at-oldest_arrival)*1000)

        decode_started = time.perf_counter()
        images = self.decode_frames(frames)
        timing['decode_ms'] = (time.perf_counter()-decode_started)*1000
        self.last_key = key

        pose_started = time.perf_counter()
        compensation = None
        self.pose_status = 'disabled'
        if self.pose_config.get('enabled', False):
            compensation, self.pose_status = svm_module.projection_pose(
                poses, frames, time.monotonic(), self.pose_config,
                -self.config['ground_z_m'],
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
            for index, view in enumerate(VIEWS):
                x, y = (index % 2)*320, 50+(index // 2)*250
                panel[y:y+240, x:x+320] = cv2.resize(images[view], (320, 240))
                cv2.putText(panel, view.upper(), (x+8, y+22), cv2.FONT_HERSHEY_SIMPLEX,
                            .6, (0, 220, 255), 2, cv2.LINE_AA)
            panel[50:562, 660:1172] = self.decorate(
                display_projection, compensation is not None, grid=True)
            label = 'SVM UNSYNCED | {} | skew ignored | compute {:.1f} ms'.format(
                self.pose_status, (time.perf_counter()-started)*1000)
            cv2.putText(panel, label, (10, 28), cv2.FONT_HERSHEY_SIMPLEX,
                        .55, (0, 200, 255), 1, cv2.LINE_AA)
        timing['display_ms'] = (time.perf_counter()-display_started)*1000

        stamp = rospy.Time.now()
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
        rospy.loginfo_throttle(5, 'SVM unsynced | decode %.1fms | maps %.1fms | warp %.1fms | '
                              'total %.1fms | receive-to-publish %.1fms',
                              timing['decode_ms'], timing['maps_ms'], timing['warp_ms'],
                              timing['processing_ms'], timing['received_to_publish_ms'])
        if saving:
            self.save_capture(images, frames, poses, projection, display_projection,
                              image, panel, compensation, clock, span, timing)
        return True


if __name__ == "__main__":
    rospy.init_node("fisheye_svm_unsynced_preview")
    node = UnsyncedSvmNode()
    rospy.logwarn("SVM UNSYNCED PREVIEW: camera skew and input age checks are disabled")
    rospy.spin()
