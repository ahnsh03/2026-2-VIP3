#!/usr/bin/env python3
"""Run the external GCN parking-slot model on the clean SVM BEV topic."""
import json
from pathlib import Path
import time

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from vip3_perception.coordinates import pixel_to_vehicle
from vip3_perception.detection.gcn_adapter import (
    draw_slots, infer, load_model, slot_endpoints,
)


class GcnSlotDetector:
    def __init__(self):
        cfg = rospy.get_param("~gcn")
        # Source tree layout is stable and keeps the upstream clone outside catkin src.
        project_root = Path(__file__).resolve().parents[4]
        repository = Path(rospy.get_param(
            "~repository", str(project_root / "external" / "parking" / "gcn-parking-slot")))
        checkpoint = rospy.get_param("~checkpoint", "")
        self.input_size = int(cfg.get("input_size", 512))
        # Read the active SVM geometry, which derives center_x_m from vehicle dimensions.
        self.extent_m = float(rospy.get_param("/perception/svm/extent_m", cfg["extent_m"]))
        self.max_age = float(cfg.get("max_input_age_s", 0.5))
        if self.input_size < 1 or self.extent_m <= 0 or self.max_age <= 0:
            raise ValueError("input_size, extent_m and max_input_age_s must be positive")

        self.model, self.torch, self.device = load_model(
            repository, rospy.get_param("~config_file", str(repository / "config" / cfg["config_file"])),
            checkpoint, rospy.get_param("~device", cfg.get("device", "auto")))
        self.slots_pub = rospy.Publisher(cfg["output_topic"], String, queue_size=1)
        self.overlay_pub = rospy.Publisher(cfg["overlay_topic"], CompressedImage, queue_size=1)
        self.busy = False
        self.sub = rospy.Subscriber(cfg["input_topic"], CompressedImage,
                                    self.receive, queue_size=1, buff_size=8 * 1024 * 1024)
        rospy.loginfo("GCN slot detector ready: device=%s, input=%s, checkpoint=%s",
                      self.device, cfg["input_topic"], checkpoint)

    def receive(self, msg):
        if self.busy:
            return
        self.busy = True
        started = time.monotonic()
        try:
            if msg.header.stamp and (rospy.Time.now() - msg.header.stamp).to_sec() > self.max_age:
                rospy.logwarn_throttle(5, "Dropping stale SVM BEV frame")
                return
            image = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
            resized, slots = infer(self.model, self.torch, self.device, image, self.input_size)
            endpoints = slot_endpoints(slots, self.input_size)
            if not rospy.has_param("/perception/svm/center_x_m"):
                rospy.logwarn_throttle(5, "Waiting for SVM BEV geometry parameters")
                return
            bev_size = int(rospy.get_param("/perception/svm/bev_size", self.input_size))
            if bev_size != self.input_size:
                raise ValueError("GCN input_size must match SVM BEV size ({} != {})".format(
                    self.input_size, bev_size))
            self.extent_m = float(rospy.get_param("/perception/svm/extent_m", self.extent_m))
            center_x = float(rospy.get_param("/perception/svm/center_x_m"))
            width = resized.shape[1]
            serialized = []
            for slot, (p0, p1) in zip(slots, endpoints):
                (x0, y0), (x1, y1) = p0, p1
                x0m, y0m = pixel_to_vehicle(y0, x0, self.input_size, self.extent_m, center_x)
                x1m, y1m = pixel_to_vehicle(y1, x1, self.input_size, self.extent_m, center_x)
                score = slot[0]
                score = float(score.detach().cpu().item() if hasattr(score, "detach") else score)
                serialized.append({"confidence": score, "endpoints_px": [p0, p1],
                                   "endpoints_base_link_m": [[float(x0m), float(y0m)],
                                                              [float(x1m), float(y1m)]]})
            output = String()
            output.data = json.dumps({
                "header": {"stamp": msg.header.stamp.to_sec(),
                           "frame_id": "base_link", "source": "gcn-parking-slot"},
                "image": {"width": width, "height": resized.shape[0],
                          "extent_m": self.extent_m, "center_x_m": center_x,
                          "calibration_profile": rospy.get_param(
                              "/perception/svm/calibration_profile", "unspecified"),
                          "calibration_status": rospy.get_param(
                              "/perception/svm/calibration_status", "unverified"),
                          "coordinate_convention": "x forward, y left; pixel centers; front up"},
                "slots": serialized,
                "inference_ms": (time.monotonic() - started) * 1000.0,
            }, separators=(",", ":"))
            self.slots_pub.publish(output)
            overlay = draw_slots(resized, endpoints)
            ok, encoded = cv2.imencode(".jpg", overlay, [cv2.IMWRITE_JPEG_QUALITY, 90])
            if ok:
                result = CompressedImage()
                result.header = msg.header
                result.format = "jpeg"
                result.data = encoded.tobytes()
                self.overlay_pub.publish(result)
        except Exception as exc:
            rospy.logerr_throttle(2, "GCN inference failed: %s", exc)
        finally:
            self.busy = False


if __name__ == "__main__":
    rospy.init_node("gcn_slot_detector")
    GcnSlotDetector()
    rospy.spin()
