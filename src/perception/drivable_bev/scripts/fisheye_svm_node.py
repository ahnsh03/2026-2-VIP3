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

from drivable_bev.fisheye_svm import CameraFrame, FisheyeSvm, VIEWS, select_frames


class SvmNode:
    def __init__(self):
        self.config = rospy.get_param("~svm")
        self.svm = FisheyeSvm(self.config)
        self.buffers = {v: deque(maxlen=10) for v in VIEWS}
        self.lock = threading.Lock()
        self.last_key = None
        self.saved = False
        self.save_dir = rospy.get_param("~save_dir", "")
        self.max_age = float(self.config["max_age_s"])
        self.slop = float(self.config["sync_slop_s"])
        self.preview = rospy.Publisher("/perception/svm/preview/compressed", CompressedImage, queue_size=1)
        self.panel = rospy.Publisher("/perception/svm/panel/compressed", CompressedImage, queue_size=1)
        self.panel_raw = rospy.Publisher("/perception/svm/panel", Image, queue_size=1)
        self.mask = rospy.Publisher("/perception/svm/valid_mask", Image, queue_size=1)
        self.subscribers = [rospy.Subscriber(self.config["cameras"][v]["topic"], CompressedImage,
                            partial(self.receive, v), queue_size=1, buff_size=4 * 1024 * 1024) for v in VIEWS]
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.config["publish_hz"]), self.tick)
        rospy.logwarn("SVM PREVIEW: lens K/D, ground height and occlusion require calibration; not a parking input")

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
            self.buffers[view].append(frame)

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
            return
        key = tuple(frames[v].arrival for v in VIEWS)
        if key == self.last_key:
            return
        self.last_key = key
        started = time.perf_counter()
        images = {v: frames[v].image for v in VIEWS}
        image = self.svm.render(images)
        # Preview topic also carries a warning so it cannot silently become training input.
        cv2.putText(image, "UNCALIBRATED PREVIEW", (8, self.svm.size - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 180, 255), 1, cv2.LINE_AA)
        stamp = rospy.Time.from_sec(max(f.stamp for f in frames.values())) if clock == "header" else rospy.Time.now()
        self.send(self.preview, image, stamp)
        panel = np.zeros((580, 1180, 3), np.uint8)
        for i, view in enumerate(VIEWS):
            x, y = (i % 2) * 320, 50 + (i // 2) * 250
            panel[y:y + 240, x:x + 320] = cv2.resize(images[view], (320, 240))
            cv2.putText(panel, view.upper(), (x + 8, y + 22), cv2.FONT_HERSHEY_SIMPLEX,
                        .6, (0, 220, 255), 2, cv2.LINE_AA)
        panel[50:562, 660:1172] = cv2.resize(self.svm.debug_image(image), (512, 512))
        elapsed = (time.perf_counter() - started) * 1000
        label = "SVM PREVIEW | UNCALIBRATED | sync {} {:.1f} ms | render {:.1f} ms".format(clock, span * 1000, elapsed)
        cv2.putText(panel, label, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 200, 255), 1, cv2.LINE_AA)
        self.send(self.panel, panel, stamp)
        self.send_raw_panel(panel, stamp)
        mask = Image()
        mask.header.stamp, mask.header.frame_id = stamp, "base_link"
        mask.height = mask.width = self.svm.size
        mask.encoding, mask.step = "mono8", self.svm.size
        mask.data = self.svm.valid_mask.tobytes()
        self.mask.publish(mask)
        if self.save_dir and not self.saved:
            path = Path(self.save_dir)
            path.mkdir(parents=True, exist_ok=True)
            for view in VIEWS:
                cv2.imwrite(str(path / (view + ".png")), images[view])
            cv2.imwrite(str(path / "svm_preview.png"), image)
            cv2.imwrite(str(path / "svm_panel.png"), panel)
            cv2.imwrite(str(path / "valid_mask.png"), self.svm.valid_mask)
            (path / "config.yaml").write_text(yaml.safe_dump(self.config, sort_keys=False))
            report = {"calibrated": False, "clock": clock, "span_ms": span * 1000,
                      "render_ms": elapsed, "frames": {v: {"stamp": f.stamp, "frame_id": f.frame_id}
                                                         for v, f in frames.items()}}
            (path / "capture.json").write_text(json.dumps(report, indent=2) + "\n")
            self.saved = True
            rospy.loginfo("SVM actual input and preview saved: %s", path)

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
