"""Metric ground projection and four-camera fusion."""
import math
import cv2
import numpy as np
from .frames import VIEWS
from .lens import validate_lens, camera_transform, _project_components
from .vehicle_geometry import resolve_svm_geometry
from .pose import offset_ground_plane
from ..coordinates import pixel_to_vehicle

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
        self.x, self.y = pixel_to_vehicle(row, col, self.size, self.extent, self.center_x)
        self._x_column, self._y_row = self.x[:, :1], self.y[:1, :]
        body = config["vehicle_mask"]
        self.body_mask = ((self.x >= body["x_min_m"]) & (self.x <= body["x_max_m"])
                          & (self.y >= body["y_min_m"]) & (self.y <= body["y_max_m"]))
        for camera in config['cameras'].values():
            validate_lens(camera)
        self.static_plane = dict(z_at_origin_m=float(config['ground_z_m']),
                                 dz_dx=float(config.get('ground_plane', {}).get('dz_dx', 0)),
                                 dz_dy=float(config.get('ground_plane', {}).get('dz_dy', 0)))
        self.static_plane = offset_ground_plane(
            self.static_plane, float(config.get('projection_surface_offset_m', 0.)))
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
        # A ground plane and a rigid transform compose into three affine
        # functions of X/Y. Evaluate those directly on separable row/column
        # grids: exact at every output pixel, with no coarse-grid interpolation.
        z0, a, b = parameters
        basis = np.array([[1., 0.], [0., 1.], [a, b]])
        origin = np.array([0., 0., z0])
        def affine(coefficients, constant):
            return coefficients[0]*self._x_column + coefficients[1]*self._y_row + constant
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
            source_basis = delta_r.T @ basis
            source_origin = (origin - delta_t) @ delta_r
            position = delta_r @ position + delta_t
            rotation = rotation @ delta_r.T
            optical_basis = rotation @ basis
            optical_origin = rotation @ (origin - position)
            ox, oy, oz = [affine(optical_basis[i], optical_origin[i]) for i in range(3)]
            u, v, theta = _project_components(ox, oy, oz, camera["K"], camera["D"])
            limit = math.radians(float(camera["max_ray_angle_deg"]))
            valid = ((oz > 0) & (theta < limit) & (u >= 1)
                     & (u < width - 2) & (v >= 1) & (v < height - 2))
            edge = np.minimum(np.minimum(u, width - 1 - u), np.minimum(v, height - 1 - v))
            lens_weight = np.clip(edge / 25, 0, 1) * np.clip(1 - theta / limit, 0, 1) ** 2
            # Diagnostic only: show the sampled image even where the body hides the ground.
            self.unmasked_weights[view] = (lens_weight * valid).astype(np.float32)
            valid &= ~self.body_mask
            source_x = affine(source_basis[0], source_origin[0]) - mount_position[0]
            source_y = affine(source_basis[1], source_origin[1]) - mount_position[1]
            # Approximate body-occlusion test sampled along the ground ray.
            # It cannot replace visibility validation against the actual vehicle mesh.
            for fraction in (.05, .15, .3, .5, .75):
                # Occlusion belongs to the body at image capture time, not the
                # reference body (which may already have moved past this camera).
                qx = mount_position[0] + fraction * source_x
                qy = mount_position[1] + fraction * source_y
                valid &= ~((qx > body["x_min_m"]) & (qx < body["x_max_m"])
                           & (qy > body["y_min_m"]) & (qy < body["y_max_m"]))
            weight = (lens_weight * valid).astype(np.float32)
            self.maps[view] = (u.astype(np.float32), v.astype(np.float32), weight)
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
