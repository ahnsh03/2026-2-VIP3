"""Local ground plane from overlapping camera images, with bounded pose-aware reuse.

No map/slot GT is used. Sparse bidirectional optical flow on approximate BEV
warps supplies stereo rays; robust triangulation rejects non-ground matches.
The resulting plane is a projection surface, not an obstacle/free-space map.
"""
import math

import cv2
import numpy as np

from .frames import VIEWS
from .lens import camera_transform
from ..coordinates import pixel_to_vehicle

PAIRS = (('front', 'left'), ('front', 'right'), ('rear', 'left'), ('rear', 'right'))


def fit_plane(points, pairs):
    """Reject poorly supported/degenerate planes instead of fitting one seam."""
    points, pairs = np.asarray(points, float), np.asarray(pairs)
    if len(points) < 20:
        return None, dict(reason='too few stereo points', matches=len(points))
    design = np.c_[np.ones(len(points)), points[:, :2]]
    rng = np.random.RandomState(42)
    best = None
    for _ in range(250):
        ids = rng.choice(len(points), 3, replace=False)
        if np.linalg.cond(design[ids]) > 100:
            continue
        beta = np.linalg.solve(design[ids], points[ids, 2])
        if not (-.44 < beta[0] < -.24 and np.linalg.norm(beta[1:]) < .08):
            continue
        inside = np.abs(design @ beta-points[:, 2]) < .02
        if best is None or inside.sum() > best.sum():
            best = inside
    if best is None:
        return None, dict(reason='no plausible ground plane', matches=len(points))
    support = {str(p): int(np.sum(best & (pairs == p))) for p in np.unique(pairs)}
    report = dict(matches=len(points), inliers=int(best.sum()), pairs=support)
    if best.sum() < 20 or best.mean() < .55 or sum(n >= 2 for n in support.values()) < 3:
        return None, dict(report, reason='insufficient overlap support')
    xy = points[best, :2]
    if np.min(np.linalg.eigvalsh(np.cov(xy.T))) < .3:
        return None, dict(report, reason='ground points are nearly collinear')
    a, z = design[best], points[best, 2]
    beta = np.linalg.lstsq(a, z, rcond=None)[0]
    for _ in range(5):
        weights = 1/np.sqrt(1+((a@beta-z)/.005)**2)
        beta = np.linalg.lstsq(a*weights[:, None]**.5, z*weights**.5, rcond=None)[0]
    rmse = float(np.sqrt(np.mean((a@beta-z)**2)))
    report['rmse_m'] = rmse
    if rmse > .015 or not (-.44 < beta[0] < -.24 and np.linalg.norm(beta[1:]) < .08):
        return None, dict(report, reason='ground fit outside bounds')
    return dict(z_at_origin_m=float(beta[0]), dz_dx=float(beta[1]), dz_dy=float(beta[2])), dict(report, reason='accepted')


def estimate_ground(svm, images, motion):
    """Estimate a plane in the current output body frame from image content only."""
    gray = {v: cv2.cvtColor(cv2.remap(images[v], *svm.maps[v][:2], cv2.INTER_LINEAR),
                            cv2.COLOR_BGR2GRAY) for v in VIEWS}
    plane = svm.active_plane

    def rays(view, pixels):
        x, y = pixel_to_vehicle(pixels[:, 1], pixels[:, 0], svm.size, svm.extent, svm.center_x)
        points = np.c_[x, y, plane['z_at_origin_m']+plane['dz_dx']*x+plane['dz_dy']*y]
        mount, _ = camera_transform(svm.config['cameras'][view])
        dr, dt = motion[view]
        origin = dr@mount+dt
        direction = points-origin
        direction /= np.linalg.norm(direction, axis=1)[:, None]
        return origin, direction

    points, tags, ray_pairs = [], [], []
    for va, vb in PAIRS:
        mask = ((svm.maps[va][2] > .002) & (svm.maps[vb][2] > .002)).astype(np.uint8)*255
        mask = cv2.erode(mask, np.ones((23, 23), np.uint8))
        pa = cv2.goodFeaturesToTrack(gray[va], 500, .008, 6, mask=mask, blockSize=5)
        if pa is None:
            continue
        options = dict(winSize=(15, 15), maxLevel=1, criteria=(3, 30, .01))
        pb, ok, error = cv2.calcOpticalFlowPyrLK(gray[va], gray[vb], pa, None, **options)
        back, ok_back, _ = cv2.calcOpticalFlowPyrLK(gray[vb], gray[va], pb, None, **options)
        pa, pb, back = pa[:, 0], pb[:, 0], back[:, 0]
        good = ((ok[:, 0] > 0) & (ok_back[:, 0] > 0) & (error[:, 0] < 30)
                & (np.linalg.norm(back-pa, axis=1) < .6)
                & (np.linalg.norm(pb-pa, axis=1) < .5*svm.size/svm.extent)
                & (pb[:, 0] > 6) & (pb[:, 0] < svm.size-7)
                & (pb[:, 1] > 6) & (pb[:, 1] < svm.size-7))
        pa, pb = pa[good], pb[good]
        if not len(pa):
            continue
        oa, da = rays(va, pa)
        ob, db = rays(vb, pb)
        for a, b in zip(da, db):
            distance = np.linalg.lstsq(np.c_[a, -b], ob-oa, rcond=None)[0]
            aa, bb = oa+distance[0]*a, ob+distance[1]*b
            point = (aa+bb)/2
            if ((distance > 0).all() and np.linalg.norm(aa-bb) < .035
                    and a@b < math.cos(math.radians(8)) and -.55 < point[2] < -.15):
                points.append(point)
                tags.append(va+'-'+vb)
                ray_pairs.append((oa, a, ob, b))
    candidate, report = fit_plane(points, tags)
    if candidate is None:
        return None, report

    def differences(surface):
        normal = np.array([-surface['dz_dx'], -surface['dz_dy'], 1.])
        errors = []
        for oa, a, ob, b in ray_pairs:
            distances = [(surface['z_at_origin_m']-o@normal)/(d@normal) for o, d in ((oa, a), (ob, b))]
            errors.append(np.linalg.norm((oa+distances[0]*a)-(ob+distances[1]*b)))
        return np.array(errors)

    before, after = differences(plane), differences(candidate)
    points = np.asarray(points)
    on_plane = np.abs(points[:, 2] - candidate['z_at_origin_m']
                      - candidate['dz_dx']*points[:, 0] - candidate['dz_dy']*points[:, 1]) < .02
    report.update(all_median_before_m=float(np.median(before)), all_median_after_m=float(np.median(after)),
                  median_before_m=float(np.median(before[on_plane])), median_after_m=float(np.median(after[on_plane])))
    # Once aligned, keep the supported plane instead of chasing small noisy
    # updates. The tracker refreshes its age only when new matches support it.
    if np.median(after[on_plane]) > .85*np.median(before[on_plane]) and np.median(after[on_plane]) > .02:
        return None, dict(report, reason='no useful seam improvement')
    tags = np.asarray(tags)
    for tag in np.unique(tags):
        use = (tags == tag) & on_plane
        if not use.any():
            continue
        report.setdefault('seam_checks', {})[str(tag)] = dict(n=int(use.sum()), before_m=float(np.median(before[use])), after_m=float(np.median(after[use])))
        if use.sum() >= 4 and np.median(after[use]) > np.median(before[use])+.015:
            return None, dict(report, reason='an overlap worsened')
    return candidate, report


class GroundRefiner:
    """Cache the observed world plane briefly; never reuse across a pose reset."""
    def __init__(self, config):
        self.enabled = bool(config.get('enabled', False))
        self.interval = float(config.get('interval_s', 1.))
        self.max_age = float(config.get('max_age_s', 2.))
        self.max_distance = float(config.get('max_distance_m', 1.5))
        if not all(np.isfinite(v) and v > 0 for v in (self.interval, self.max_age, self.max_distance)):
            raise ValueError('positive finite ground refinement limits required')
        self.reset()

    def reset(self):
        self.cache = None
        self.last_attempt = -math.inf
        self.status = dict(source='assumed', reason='no visual ground estimate')

    def plane_for(self, compensation, now):
        if not self.enabled or compensation is None:
            self.reset()
            return None if compensation is None else compensation['plane']
        ref = compensation['reference']
        if self.cache is not None:
            saved, normal, distance, created = self.cache
            age = now-created
            if (0 <= age <= self.max_age and ref.stamp >= saved.stamp
                    and np.linalg.norm(ref.position-saved.position) <= self.max_distance):
                body = ref.rotation.T@normal
                height = (distance-normal@ref.position)/body[2] if body[2] > .98 else 0.
                if body[2] > .98 and -.6 < height < -.15:
                    self.status.update(source='vision held', age_s=age)
                    return dict(z_at_origin_m=float(height),
                                dz_dx=float(-body[0]/body[2]), dz_dy=float(-body[1]/body[2]))
            self.reset()
        self.status['source'] = 'assumed'
        return compensation['plane']

    def refine(self, svm, images, compensation, now):
        if not self.enabled or compensation is None or now-self.last_attempt < self.interval:
            return None
        self.last_attempt = now
        candidate, report = estimate_ground(svm, images, compensation['motion'])
        self.status = dict(source=self.status['source'], **report)
        if candidate is not None:
            ref = compensation['reference']
            normal = ref.rotation@np.array([-candidate['dz_dx'], -candidate['dz_dy'], 1.])
            point = ref.position+ref.rotation@np.array([0., 0., candidate['z_at_origin_m']])
            self.cache = (ref, normal, float(normal@point), now)
            self.status.update(source='vision', age_s=0.)
        elif (self.cache is not None and report['reason'] == 'no useful seam improvement'
              and report.get('median_before_m', math.inf) < .10):
            # Fresh, spatially distributed ground matches still support this
            # plane. Keep its geometry and spatial anchor; refresh time only.
            saved, normal, distance, _ = self.cache
            self.cache = (saved, normal, distance, now)
            self.status.update(source='vision held', age_s=0.)
        return candidate
