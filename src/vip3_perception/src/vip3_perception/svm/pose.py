"""Timestamped rigid poses and ground-plane compensation (no ROS dependency).

The surface normal is an explicit map-frame assumption, not an estimate from
vehicle attitude: a car can tilt on a level road or follow a sloping road.
"""
from dataclasses import dataclass
import math

import numpy as np


def source_quaternion(quaternion_xyzw, convention='ros_xyzw'):
    """MORAI 26.R1.H3 adapter, independently checked against four nominal mounts.

    This is NOT a generic ROS/Unity conversion. Keep standard ROS untouched.
    Evidence and the source TF samples are recorded in the calibration report.
    """
    q = np.asarray(quaternion_xyzw, dtype=float).copy()
    if q.shape != (4,):
        raise ValueError('source quaternion must contain xyzw')
    if convention == 'morai_26r1_x_flipped':
        q[0] *= -1
    elif convention != 'ros_xyzw':
        raise ValueError('unknown quaternion convention')
    quaternion_matrix(q)
    return q


def quaternion_matrix(quaternion_xyzw):
    q = np.asarray(quaternion_xyzw, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-9:
        raise ValueError('pose needs a finite nonzero xyzw quaternion')
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


@dataclass
class PoseSample:
    stamp: float
    arrival: float
    position: np.ndarray
    quaternion: np.ndarray

    def __post_init__(self):
        self.position = np.asarray(self.position, dtype=float)
        self.quaternion = np.asarray(self.quaternion, dtype=float)
        quaternion_matrix(self.quaternion)
        self.quaternion = self.quaternion / np.linalg.norm(self.quaternion)
        if self.position.shape != (3,) or not np.isfinite(self.position).all():
            raise ValueError('pose position must be finite xyz')
        if not math.isfinite(self.stamp) or self.stamp <= 0 or not math.isfinite(self.arrival):
            raise ValueError('pose needs a positive source stamp and finite arrival')

    @property
    def rotation(self):
        return quaternion_matrix(self.quaternion)


def interpolate_pose(samples, stamp, now, max_age=.5, max_offset=.10):
    """Interpolate nearby samples, or use a bounded nearest sample; never extrapolate."""
    fresh = sorted((p for p in samples if 0 <= now-p.arrival <= max_age), key=lambda p: p.stamp)
    if not fresh or not math.isfinite(stamp) or stamp <= 0:
        return None
    nearest = min(fresh, key=lambda p: abs(p.stamp-stamp))
    if abs(nearest.stamp-stamp) > max_offset:
        return None
    before = [p for p in fresh if p.stamp <= stamp]
    after = [p for p in fresh if p.stamp >= stamp]
    if not before or not after:
        return nearest
    a, b = before[-1], after[0]
    if stamp-a.stamp > max_offset or b.stamp-stamp > max_offset:
        return None
    if b.stamp-a.stamp < 1e-9:
        return a
    t = (stamp-a.stamp)/(b.stamp-a.stamp)
    qa, qb = a.quaternion, b.quaternion.copy()
    dot = float(np.dot(qa, qb))
    if dot < 0:
        qb, dot = -qb, -dot
    dot = min(1., dot)
    if dot > .9995:
        q = (1-t)*qa + t*qb
    else:
        angle = math.acos(dot)
        q = (math.sin((1-t)*angle)*qa + math.sin(t*angle)*qb)/math.sin(angle)
    return PoseSample(stamp, min(a.arrival, b.arrival), (1-t)*a.position+t*b.position, q)


def ground_plane_from_pose(pose, height_m, normal_map=(0., 0., 1.), max_tilt_deg=15.):
    """Return z=z0+a*x+b*y for a plane at normal distance height below the axle."""
    normal = np.asarray(normal_map, dtype=float)
    if normal.shape != (3,) or not np.isfinite(normal).all() or np.linalg.norm(normal) < 1e-9:
        raise ValueError('ground normal must be a finite nonzero xyz vector')
    if not math.isfinite(height_m) or height_m <= 0 or not 0 < max_tilt_deg < 80:
        raise ValueError('invalid ground height or maximum tilt')
    normal /= np.linalg.norm(normal)
    if normal[2] <= 0:
        raise ValueError('ground normal must point upward in map')
    body = pose.rotation.T @ normal
    if body[2] < math.cos(math.radians(max_tilt_deg)):
        raise ValueError('vehicle/ground tilt exceeds configured projection range')
    return dict(z_at_origin_m=float(-height_m/body[2]),
                dz_dx=float(-body[0]/body[2]), dz_dy=float(-body[1]/body[2]))


def offset_ground_plane(plane, offset_m=0.):
    """Move the projection surface along its upward normal, leaving the axle fixed.

    K-City parking paint is a mesh above the collision road. Its measured height
    belongs to the projection surface, not to the vehicle's physical dimensions.
    """
    result = dict(plane)
    z, a, b = (float(plane.get(key, 0.)) for key in
               ('z_at_origin_m', 'dz_dx', 'dz_dy'))
    if not np.isfinite([z, a, b, offset_m]).all() or z >= 0:
        raise ValueError('projection surface must be finite and below base_link')
    result['z_at_origin_m'] = z + float(offset_m)*math.sqrt(1+a*a+b*b)
    if result['z_at_origin_m'] >= 0:
        raise ValueError('offset projection surface must remain below base_link')
    return result


def relative_pose(reference, sample):
    """Transform body at sample time into body at the output reference time."""
    return (reference.rotation.T @ sample.rotation,
            reference.rotation.T @ (sample.position-reference.position))


def projection_pose(samples, frames, now, config, height_m, surface_offset_m=0.):
    """Find poses at ALL four camera stamps; partial compensation is not allowed."""
    if any(f.stamp <= 0 for f in frames.values()):
        return None, 'no image source timestamps'
    poses = {view: interpolate_pose(samples, f.stamp, now,
                                    float(config.get('max_age_s', .5)),
                                    float(config.get('max_offset_s', .10)))
             for view, f in frames.items()}
    if any(p is None for p in poses.values()):
        return None, 'pose missing/stale/outside image time'
    view = max(frames, key=lambda v: frames[v].stamp)
    reference = poses[view]
    try:
        plane = ground_plane_from_pose(reference, height_m,
                                       config.get('normal_map', (0., 0., 1.)),
                                       float(config.get('max_tilt_deg', 15.)))
        plane = offset_ground_plane(plane, surface_offset_m)
    except ValueError as exc:
        return None, str(exc)
    motion = {v: relative_pose(reference, poses[v]) for v in frames}
    return dict(plane=plane, motion=motion, stamp=frames[view].stamp,
                reference=reference), 'pose compensated'
