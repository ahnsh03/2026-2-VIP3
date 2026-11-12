import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

from vip3_perception.svm.ground import fit_plane, estimate_ground, GroundRefiner
from vip3_perception.svm.pose import PoseSample, quaternion_matrix
from vip3_perception.svm.frames import VIEWS
from vip3_perception.svm.projection import FisheyeSvm
from vip3_perception.svm.vehicle_geometry import load_svm_config


class TestVisualGround(unittest.TestCase):
    def cloud(self):
        rng = np.random.RandomState(3)
        xy = rng.uniform([-3, -4], [6, 4], (100, 2))
        z = -.34+.012*xy[:, 0]-.008*xy[:, 1]+rng.normal(0, .003, len(xy))
        tags = np.array(['front-left', 'front-right', 'rear-left', 'rear-right']*25)
        return np.c_[xy, z], tags

    def test_recovers_sloped_plane_with_outliers(self):
        points, tags = self.cloud()
        points[::5, 2] += .15
        plane, report = fit_plane(points, tags)
        self.assertEqual(report['reason'], 'accepted')
        np.testing.assert_allclose([plane['z_at_origin_m'], plane['dz_dx'], plane['dz_dy']],
                                   [-.34, .012, -.008], atol=.002)

    def test_one_overlap_or_collinear_points_cannot_define_full_ground(self):
        points, tags = self.cloud()
        self.assertIsNone(fit_plane(points, ['front-left']*len(points))[0])
        points[:, 1] = 2*points[:, 0]
        self.assertIsNone(fit_plane(points, tags)[0])
        self.assertIsNone(fit_plane([], [])[0])

    def test_obstacle_height_is_not_accepted_as_ground(self):
        points, tags = self.cloud()
        points[:, 2] += 1.
        self.assertIsNone(fit_plane(points, tags)[0])

    def test_blank_images_produce_no_estimate(self):
        config = load_svm_config(Path(__file__).resolve().parents[1]/'config/svm.yaml')
        svm = FisheyeSvm(config)
        images = {v: np.zeros((cam['height'], cam['width'], 3), np.uint8)
                  for v, cam in config['cameras'].items()}
        motion = {v: (np.eye(3), np.zeros(3)) for v in VIEWS}
        plane, report = estimate_ground(svm, images, motion)
        self.assertIsNone(plane)
        self.assertEqual(report['matches'], 0)

    def test_cached_world_plane_follows_pose_and_expires(self):
        refiner = GroundRefiner(dict(enabled=True, interval_s=1, max_age_s=2, max_distance_m=1.5))
        reference = PoseSample(10., 0., [10., 20., 3.], [0., 0., 0., 1.])
        surface = dict(z_at_origin_m=-.34, dz_dx=.01, dz_dy=-.02)
        fallback = dict(z_at_origin_m=-.34, dz_dx=0., dz_dy=0.)
        compensation = dict(reference=reference, plane=fallback, motion={})
        with patch('vip3_perception.svm.ground.estimate_ground', return_value=(surface, dict(reason='accepted'))):
            self.assertEqual(refiner.refine(None, {}, compensation, 1.), surface)
        current = PoseSample(10.5, 0., [10.2, 19.9, 3.01], [.01, -.02, .03, 1.])
        compensation['reference'] = current
        result = refiner.plane_for(compensation, 1.5)
        for x, y in ((4., 2.), (-2., -3.)):
            z = result['z_at_origin_m']+result['dz_dx']*x+result['dz_dy']*y
            world = current.rotation@np.array([x, y, z])+current.position
            delta = world-reference.position
            self.assertAlmostEqual(delta[2], -.34+.01*delta[0]-.02*delta[1], places=10)
        self.assertEqual(refiner.plane_for(compensation, 3.1), fallback)
        self.assertEqual(refiner.status['source'], 'assumed')

    def test_pose_reset_drops_cached_ground(self):
        refiner = GroundRefiner(dict(enabled=True))
        p = PoseSample(10., 0., [0., 0., 0.], [0., 0., 0., 1.])
        refiner.cache = (p, np.array([0., 0., 1.]), -.34, 1.)
        fallback = dict(z_at_origin_m=-.34, dz_dx=0., dz_dy=0.)
        reference = PoseSample(1., 0., [0., 0., 0.], [0., 0., 0., 1.])
        self.assertEqual(refiner.plane_for(dict(reference=reference, plane=fallback), 1.2), fallback)
        self.assertIsNone(refiner.cache)

    def test_fresh_ground_support_refreshes_age_without_moving_plane(self):
        refiner = GroundRefiner(dict(enabled=True))
        p = PoseSample(10., 0., [0., 0., 0.], [0., 0., 0., 1.])
        normal = np.array([.01, -.02, 1.])
        refiner.cache = (p, normal, -.34, 1.)
        report = dict(reason='no useful seam improvement', median_before_m=.03)
        with patch('vip3_perception.svm.ground.estimate_ground', return_value=(None, report)):
            refiner.refine(None, {}, dict(reference=p, motion={}), 2.)
        self.assertEqual(refiner.cache[3], 2.)
        np.testing.assert_array_equal(refiner.cache[1], normal)
        self.assertEqual(refiner.cache[2], -.34)
        self.assertEqual(refiner.status['source'], 'vision held')


if __name__ == '__main__':
    unittest.main()
