"""Fisheye lens model and calibrated camera transforms."""
import math
import numpy as np

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
    u, v, theta = _project_components(points[..., 0], points[..., 1], points[..., 2],
                                       intrinsic, distortion)
    return np.stack((u, v), axis=-1), theta


def _project_components(x, y, z, intrinsic, distortion):
    """Same double-precision lens formula without an H x W x 3 temporary."""
    radius = np.sqrt(x*x + y*y)
    theta = np.arctan2(radius, z)
    t2 = theta * theta
    k1, k2, k3, k4 = distortion
    distorted = theta * (1 + t2 * (k1 + t2 * (k2 + t2 * (k3 + t2 * k4))))
    scale = np.divide(distorted, radius, out=np.zeros_like(radius), where=radius > 1e-12)
    return (x*scale*intrinsic[0][0] + intrinsic[0][2],
            y*scale*intrinsic[1][1] + intrinsic[1][2], theta)
