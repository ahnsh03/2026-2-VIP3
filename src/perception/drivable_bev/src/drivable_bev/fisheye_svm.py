"""Ground-plane fisheye preview. FOV alone is NOT a camera calibration.

Uses the OpenCV fisheye theta polynomial. The shipped K/D are an explicit
equidistant initialization, pending MORAI correspondence-based calibration.
"""
from dataclasses import dataclass
import math

import cv2
import numpy as np


VIEWS = ("front", "rear", "left", "right")


def optical_rotation(rotation_deg):
    roll, pitch, yaw = np.radians(rotation_deg)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]]) @ (rz @ ry @ rx).T


def project_fisheye(points_optical, intrinsic, distortion):
    """Project forward rays using OpenCV's fisheye model, returning (uv, theta)."""
    points = np.asarray(points_optical, dtype=np.float64)
    radius = np.linalg.norm(points[..., :2], axis=-1)
    theta = np.arctan2(radius, points[..., 2])
    t2 = theta * theta
    k1, k2, k3, k4 = distortion
    distorted = theta * (1 + t2 * (k1 + t2 * (k2 + t2 * (k3 + t2 * k4))))
    scale = np.divide(distorted, radius, out=np.zeros_like(radius), where=radius > 1e-12)
    uv = points[..., :2] * scale[..., None]
    uv[..., 0] = uv[..., 0] * intrinsic[0][0] + intrinsic[0][2]
    uv[..., 1] = uv[..., 1] * intrinsic[1][1] + intrinsic[1][2]
    return uv, theta


@dataclass
class CameraFrame:
    image: np.ndarray
    stamp: float
    arrival: float
    frame_id: str


def select_frames(buffers, now, max_age, slop):
    """Select a fresh synchronized set; never keep displaying stale cameras.

    Header timestamps are compared only to each other (simulation clocks may
    differ from wall time). Zero timestamps use monotonic arrival times and
    are reported explicitly. A negative/expired arrival is never accepted.
    """
    fresh = {view: [f for f in buffers[view] if 0 <= now - f.arrival <= max_age]
             for view in VIEWS}
    if any(not frames for frames in fresh.values()):
        return None, "waiting/stale", None
    use_stamp = all(frames[-1].stamp > 0 for frames in fresh.values())
    clock = (lambda f: f.stamp) if use_stamp else (lambda f: f.arrival)
    anchor_view = min(VIEWS, key=lambda v: clock(fresh[v][-1]))
    for anchor in reversed(fresh[anchor_view]):
        selected = {v: min(fresh[v], key=lambda f: abs(clock(f) - clock(anchor))) for v in VIEWS}
        values = [clock(f) for f in selected.values()]
        span = max(values) - min(values)
        if span <= slop:
            return selected, "header" if use_stamp else "arrival (zero header)", span
    return None, "unsynchronized", None


class FisheyeSvm:
    def __init__(self, config):
        self.config = config
        self.size = int(config["output_size"])
        self.extent = float(config["extent_m"])
        self.center_x = float(config["center_x_m"])
        if self.size <= 0 or self.extent <= 0:
            raise ValueError("positive output size and extent required")
        # Pixel centers: forward is up, left is left. Center is vehicle body center.
        row, col = np.mgrid[:self.size, :self.size]
        self.x = self.center_x + self.extent / 2 - (row + .5) * self.extent / self.size
        self.y = self.extent / 2 - (col + .5) * self.extent / self.size
        ground_z = float(config["ground_z_m"])
        xyz = np.stack([self.x, self.y, np.full_like(self.x, ground_z)], axis=-1)
        body = config["vehicle_mask"]
        self.body_mask = ((self.x >= body["x_min_m"]) & (self.x <= body["x_max_m"])
                          & (self.y >= body["y_min_m"]) & (self.y <= body["y_max_m"]))
        self.maps = {}
        for view in VIEWS:
            camera = config["cameras"][view]
            width, height = int(camera["width"]), int(camera["height"])
            optical = (xyz - camera["translation_m"]) @ optical_rotation(camera["rotation_deg"]).T
            uv, theta = project_fisheye(optical, camera["K"], camera["D"])
            limit = math.radians(float(camera["max_ray_angle_deg"]))
            valid = ((optical[..., 2] > 0) & (theta < limit) & (uv[..., 0] >= 1)
                     & (uv[..., 0] < width - 2) & (uv[..., 1] >= 1)
                     & (uv[..., 1] < height - 2) & ~self.body_mask)
            # Approximate body-occlusion test sampled along the ground ray.
            # It cannot replace visibility validation against the actual vehicle mesh.
            position = np.asarray(camera["translation_m"])
            for fraction in (.05, .15, .3, .5, .75):
                qx = position[0] + fraction * (self.x - position[0])
                qy = position[1] + fraction * (self.y - position[1])
                valid &= ~((qx > body["x_min_m"]) & (qx < body["x_max_m"])
                           & (qy > body["y_min_m"]) & (qy < body["y_max_m"]))
            edge = np.minimum.reduce([uv[..., 0], width - 1 - uv[..., 0],
                                      uv[..., 1], height - 1 - uv[..., 1]])
            weight = (np.clip(edge / 25, 0, 1) * np.clip(1 - theta / limit, 0, 1) ** 2
                      * valid).astype(np.float32)
            self.maps[view] = (uv[..., 0].astype(np.float32), uv[..., 1].astype(np.float32), weight)
        self.weight_sum = sum(item[2] for item in self.maps.values())
        self.valid_mask = ((self.weight_sum > 1e-6) & ~self.body_mask).astype(np.uint8) * 255

    def render(self, images):
        fused = np.zeros((self.size, self.size, 3), dtype=np.float32)
        for view in VIEWS:
            camera = self.config["cameras"][view]
            image = images[view]
            if image.shape != (camera["height"], camera["width"], 3):
                raise ValueError("{} image dimensions differ from configuration".format(view))
            mx, my, weight = self.maps[view]
            warped = cv2.remap(image, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
            fused += warped.astype(np.float32) * weight[..., None]
        fused /= np.maximum(self.weight_sum[..., None], 1e-6)
        result = np.clip(fused, 0, 255).astype(np.uint8)
        result[self.body_mask] = (40, 40, 40)
        return result

    def debug_image(self, image):
        result = image.copy()
        for distance in range(-5, 6):
            row = int((self.center_x + self.extent / 2 - distance) * self.size / self.extent)
            col = int((self.extent / 2 - distance) * self.size / self.extent)
            if 0 <= row < self.size:
                cv2.line(result, (0, row), (self.size - 1, row), (75, 75, 75), 1)
            if 0 <= col < self.size:
                cv2.line(result, (col, 0), (col, self.size - 1), (75, 75, 75), 1)
        cv2.putText(result, "FRONT", (self.size // 2 - 28, 24), cv2.FONT_HERSHEY_SIMPLEX,
                    .5, (0, 220, 255), 1, cv2.LINE_AA)
        return result
