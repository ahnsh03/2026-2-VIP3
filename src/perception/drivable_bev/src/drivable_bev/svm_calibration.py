"""Metric ground correspondences for fisheye calibration; no image appearance fitting.

Click-Calib motivates shared ground points. Here measured ground anchors fix the
metric frame and camera translations stay fixed, preventing scale/height drift.
"""
from copy import deepcopy

import numpy as np
from scipy.optimize import least_squares

from .fisheye_svm import VIEWS, pixels_to_ground, validate_lens


def _candidate(config, parameters):
    candidate = deepcopy(config)
    for index, view in enumerate(VIEWS):
        camera = candidate['cameras'][view]
        camera['K'][0][0] *= 1 + float(parameters[0])
        camera['K'][1][1] *= 1 + float(parameters[0])
        camera['D'][0] += float(parameters[1])
        camera['D'][1] += float(parameters[2])
        correction = camera.setdefault('correction', {})
        correction['rotation_deg'] = (np.asarray(correction.get('rotation_deg', [0, 0, 0]))
                                      + parameters[3+3*index:6+3*index]).tolist()
    return candidate


def correspondence_residuals(config, observations):
    """Signed XY errors in metres, batched per camera; invalid rays get a penalty."""
    result = np.empty((len(observations), 2), dtype=float)
    for view in VIEWS:
        indices = [i for i, o in enumerate(observations) if o['view'] == view]
        if not indices:
            continue
        for i in indices:
            o = observations[i]
            xy, valid = pixels_to_ground([o['pixel']], config['cameras'][view], o['plane'])
            result[i] = xy[0] - o['ground_xy'] if valid[0] else [100., 100.]
    return result


def error_metrics(residuals):
    distances = np.linalg.norm(residuals, axis=1)
    return dict(count=len(distances), rmse_m=float(np.sqrt(np.mean(distances**2))),
                median_m=float(np.median(distances)), p95_m=float(np.percentile(distances, 95)),
                max_m=float(np.max(distances)))


def fit_ground_calibration(config, observations, max_validation_rmse_m=.15):
    """Fit shared focal scale/k1/k2 and each camera's small Euler mount correction.

    Required: >=8 train and >=4 held-out metric anchors per camera, identified by
    physical point_id. Splits are supplied, never chosen by the optimizer. This
    is a bounded local refinement, not calibration from an arbitrary initial rig.
    """
    ids = {}
    for o in observations:
        if o['view'] not in VIEWS or o['split'] not in ('train', 'validation'):
            raise ValueError('unknown camera or split')
        if not np.isfinite(np.r_[o['pixel'], o['ground_xy']]).all():
            raise ValueError('correspondence must be finite')
        if len(o['pixel']) != 2 or len(o['ground_xy']) != 2:
            raise ValueError('pixel and ground_xy must each contain two values')
        key = str(o['point_id'])
        if key in ids and ids[key] != o['split']:
            raise ValueError('physical point reused across train/validation')
        ids[key] = o['split']
    train = [o for o in observations if o['split'] == 'train']
    validation = [o for o in observations if o['split'] == 'validation']
    for view in VIEWS:
        for group, minimum in ((train, 8), (validation, 4)):
            selected = [o for o in group if o['view'] == view]
            points = np.array([o['ground_xy'] for o in selected])
            if len({o['point_id'] for o in selected}) < minimum:
                raise ValueError('each camera needs >=8 train and >=4 validation physical points')
            if np.linalg.matrix_rank(points - points.mean(axis=0), tol=.1) < 2:
                raise ValueError('ground anchors must span two dimensions')
    bound = np.array([.08, .06, .015] + [3.] * 12)
    def objective(p):
        try:
            residual = correspondence_residuals(_candidate(config, p), train).ravel()
        except ValueError:
            residual = np.full(len(train)*2, 100.)
        # Weak regularization discourages large corrections without hiding data residuals.
        return np.r_[residual, .005*p/bound]
    result = least_squares(objective, np.zeros(15), bounds=(-bound, bound),
                           loss='soft_l1', f_scale=.03, x_scale=bound, max_nfev=250)
    candidate = _candidate(config, result.x)
    for camera in candidate['cameras'].values():
        validate_lens(camera)
    report = dict(success=bool(result.success), parameter_delta=result.x.tolist(),
                  near_bound=bool(np.any(np.abs(result.x/bound) > .98)))
    for label, group in (('train', train), ('validation', validation)):
        report[label] = dict(before=error_metrics(correspondence_residuals(config, group)),
                             after=error_metrics(correspondence_residuals(candidate, group)))
    before = report['validation']['before']['rmse_m']
    after = report['validation']['after']['rmse_m']
    report['accepted'] = bool(result.success and not report['near_bound']
                              and after <= max_validation_rmse_m and after < .8*before)
    # Every camera must improve independently; one good camera cannot hide a bad one.
    report['cameras'] = {}
    for view in VIEWS:
        group = [o for o in validation if o['view'] == view]
        b = error_metrics(correspondence_residuals(config, group))
        a = error_metrics(correspondence_residuals(candidate, group))
        report['cameras'][view] = dict(before=b, after=a)
        report['accepted'] &= a['rmse_m'] <= max_validation_rmse_m and a['rmse_m'] <= b['rmse_m']
    candidate['calibration_status'] = 'ground_anchor_validated' if report['accepted'] else 'rejected_candidate'
    return candidate, report
