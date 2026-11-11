"""Ground-plane fisheye preview. FOV alone is NOT a camera calibration.

Uses the OpenCV fisheye theta polynomial. The shipped K/D are an explicit
equidistant initialization, pending MORAI correspondence-based calibration.
"""
from dataclasses import dataclass
import math

import cv2
import numpy as np

from .vehicle_geometry import resolve_svm_geometry

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


def camera_transform(camera):
    """Nominal MORAI mount plus a separately versioned calibration correction."""
    correction = camera.get('correction', {})
    position = np.asarray(camera['translation_m'], dtype=float) + np.asarray(
        correction.get('translation_m', [0, 0, 0]), dtype=float)
    angles = np.asarray(camera['rotation_deg'], dtype=float) + np.asarray(
        correction.get('rotation_deg', [0, 0, 0]), dtype=float)
    if position.shape != (3,) or angles.shape != (3,) or not np.isfinite(np.r_[position, angles]).all():
        raise ValueError('camera mount and correction must be finite xyz/rpy')
    return position, optical_rotation(angles)


def validate_lens(camera):
    k, d = np.asarray(camera['K'], float), np.asarray(camera['D'], float)
    if (k.shape != (3, 3) or d.shape != (4,) or not np.isfinite(k).all()
            or not np.isfinite(d).all() or k[0, 0] <= 0 or k[1, 1] <= 0
            or not np.allclose(k[2], [0, 0, 1]) or k[0, 1] != 0 or k[1, 0] != 0):
        raise ValueError('finite zero-skew fisheye K and four distortion coefficients required')
    limit = float(camera['max_ray_angle_deg'])
    if not 0 < limit < 90:
        raise ValueError('fisheye projection requires max ray angle in (0, 90)')
    t2 = np.linspace(0, math.radians(limit), 256) ** 2
    derivative = 1 + t2*(3*d[0] + t2*(5*d[1] + t2*(7*d[2] + t2*9*d[3])))
    if np.min(derivative) <= .05:
        raise ValueError('fisheye distortion must be monotonic over the usable lens')


def pixels_to_ground(pixels, camera, plane):
    """Inverse lens and rigid transform; return XY and valid ray/plane intersections."""
    validate_lens(camera)
    pixels = np.asarray(pixels, dtype=float).reshape(-1, 2)
    k, d = np.asarray(camera['K'], float), np.asarray(camera['D'], float)
    xy = (pixels - [k[0, 2], k[1, 2]])/[k[0, 0], k[1, 1]]
    radius = np.linalg.norm(xy, axis=1)
    limit = math.radians(camera['max_ray_angle_deg'])
    theta = np.minimum(radius, limit)
    # OpenCV 4.2 undistortPoints clamps distorted radius to pi/2. Invert the
    # monotonic polynomial directly so valid positive-D edge rays still round-trip.
    for _ in range(12):
        t2 = theta*theta
        value = theta*(1+t2*(d[0]+t2*(d[1]+t2*(d[2]+t2*d[3]))))
        derivative = 1+t2*(3*d[0]+t2*(5*d[1]+t2*(7*d[2]+t2*9*d[3])))
        theta = np.clip(theta-(value-radius)/derivative, 0, limit)
    t2 = theta*theta
    converged = np.abs(theta*(1+t2*(d[0]+t2*(d[1]+t2*(d[2]+t2*d[3]))))-radius) < 1e-8
    scale = np.divide(np.sin(theta), radius, out=np.ones_like(theta), where=radius > 1e-12)
    position, rotation = camera_transform(camera)
    rays = np.c_[xy*scale[:, None], np.cos(theta)] @ rotation
    normal = np.array([-plane.get('dz_dx', 0.), -plane.get('dz_dy', 0.), 1.])
    denominator = rays @ normal
    distance = np.divide(plane['z_at_origin_m'] - position @ normal, denominator,
                         out=np.full(len(pixels), np.nan), where=np.abs(denominator) > 1e-9)
    xyz = position + distance[:, None]*rays
    valid = (converged & (distance > 0) & np.isfinite(xyz).all(axis=1)
             & (theta < math.radians(camera['max_ray_angle_deg']))
             & (pixels[:, 0] >= 0) & (pixels[:, 0] < camera['width'])
             & (pixels[:, 1] >= 0) & (pixels[:, 1] < camera['height']))
    return xyz[:, :2], valid


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
        config = resolve_svm_geometry(config)
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
        body = config["vehicle_mask"]
        self.body_mask = ((self.x >= body["x_min_m"]) & (self.x <= body["x_max_m"])
                          & (self.y >= body["y_min_m"]) & (self.y <= body["y_max_m"]))
        for camera in config['cameras'].values():
            validate_lens(camera)
        self.static_plane = dict(z_at_origin_m=float(config['ground_z_m']),
                                 dz_dx=float(config.get('ground_plane', {}).get('dz_dx', 0)),
                                 dz_dy=float(config.get('ground_plane', {}).get('dz_dy', 0)))
        self._map_key = None
        self.set_projection()

    def set_projection(self, plane=None, motion=None):
        """Rebuild maps for an explicit surface and per-camera timestamp transforms.

        Motion maps each camera's body frame into the output reference body.
        Exact repeated transforms reuse maps; no stale attitude quantization.
        """
        plane = dict(self.static_plane if plane is None else plane)
        parameters = np.array([plane['z_at_origin_m'], plane.get('dz_dx', 0), plane.get('dz_dy', 0)])
        if not np.isfinite(parameters).all() or parameters[0] >= 0:
            raise ValueError('ground plane must be finite and below base_link')
        motion = motion or {v: (np.eye(3), np.zeros(3)) for v in VIEWS}
        key = tuple(parameters) + tuple(float(n) for v in VIEWS for a in motion[v]
                                        for n in np.asarray(a).ravel())
        if key == self._map_key:
            return False
        self.active_plane = plane
        config = self.config
        body = config['vehicle_mask']
        ground = parameters[0] + parameters[1]*self.x + parameters[2]*self.y
        xyz = np.stack([self.x, self.y, ground], axis=-1)
        self.maps = {}
        self.unmasked_weights = {}
        for view in VIEWS:
            camera = config["cameras"][view]
            width, height = int(camera["width"]), int(camera["height"])
            position, rotation = camera_transform(camera)
            delta_r, delta_t = (np.asarray(a, dtype=float) for a in motion[view])
            if (delta_r.shape != (3, 3) or delta_t.shape != (3,)
                    or not np.isfinite(delta_r).all() or not np.isfinite(delta_t).all()
                    or not np.allclose(delta_r.T @ delta_r, np.eye(3), atol=1e-5)
                    or np.linalg.det(delta_r) < .99):
                raise ValueError('camera motion must be a finite rigid transform')
            mount_position = position.copy()
            source_xyz = (xyz - delta_t) @ delta_r
            position = delta_r @ position + delta_t
            rotation = rotation @ delta_r.T
            optical = (xyz - position) @ rotation.T
            uv, theta = project_fisheye(optical, camera["K"], camera["D"])
            limit = math.radians(float(camera["max_ray_angle_deg"]))
            valid = ((optical[..., 2] > 0) & (theta < limit) & (uv[..., 0] >= 1)
                     & (uv[..., 0] < width - 2) & (uv[..., 1] >= 1)
                     & (uv[..., 1] < height - 2))
            edge = np.minimum.reduce([uv[..., 0], width - 1 - uv[..., 0],
                                      uv[..., 1], height - 1 - uv[..., 1]])
            lens_weight = np.clip(edge / 25, 0, 1) * np.clip(1 - theta / limit, 0, 1) ** 2
            # Diagnostic only: show the sampled image even where the body hides the ground.
            self.unmasked_weights[view] = (lens_weight * valid).astype(np.float32)
            valid &= ~self.body_mask
            # Approximate body-occlusion test sampled along the ground ray.
            # It cannot replace visibility validation against the actual vehicle mesh.
            for fraction in (.05, .15, .3, .5, .75):
                # Occlusion belongs to the body at image capture time, not the
                # reference body (which may already have moved past this camera).
                qx = mount_position[0] + fraction * (source_xyz[..., 0] - mount_position[0])
                qy = mount_position[1] + fraction * (source_xyz[..., 1] - mount_position[1])
                valid &= ~((qx > body["x_min_m"]) & (qx < body["x_max_m"])
                           & (qy > body["y_min_m"]) & (qy < body["y_max_m"]))
            weight = (lens_weight * valid).astype(np.float32)
            self.maps[view] = (uv[..., 0].astype(np.float32), uv[..., 1].astype(np.float32), weight)
        self.weight_sum = sum(item[2] for item in self.maps.values())
        self.unmasked_weight_sum = sum(self.unmasked_weights.values())
        self.valid_mask = ((self.weight_sum > 1e-6) & ~self.body_mask).astype(np.uint8) * 255
        self._map_key = key
        return True

    def render(self, images, apply_vehicle_mask=True):
        """Disabling the mask reveals body-distorted samples for display inspection only."""
        fused = np.zeros((self.size, self.size, 3), dtype=np.float32)
        for view in VIEWS:
            camera = self.config["cameras"][view]
            image = images[view]
            if image.shape != (camera["height"], camera["width"], 3):
                raise ValueError("{} image dimensions differ from configuration".format(view))
            mx, my, weight = self.maps[view]
            if not apply_vehicle_mask:
                weight = self.unmasked_weights[view]
            warped = cv2.remap(image, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
            fused += warped.astype(np.float32) * weight[..., None]
        weight_sum = self.weight_sum if apply_vehicle_mask else self.unmasked_weight_sum
        fused /= np.maximum(weight_sum[..., None], 1e-6)
        result = np.clip(fused, 0, 255).astype(np.uint8)
        fill = self.config.get('vehicle_geometry', {}).get('mask', {}).get('fill_bgr', (40, 40, 40))
        if apply_vehicle_mask:
            result[self.body_mask] = fill
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
