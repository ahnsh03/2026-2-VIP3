from copy import deepcopy
from pathlib import Path
import unittest

import numpy as np

from vip3_perception.svm.frames import VIEWS
from vip3_perception.svm.lens import camera_transform, project_fisheye
from vip3_perception.svm.calibration import fit_ground_calibration
from vip3_perception.svm.vehicle_geometry import load_svm_config


class TestGroundCalibration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.initial = load_svm_config(Path(__file__).resolve().parents[1] / 'config/svm.yaml')
        truth = deepcopy(cls.initial)
        rng = np.random.RandomState(42)
        cls.observations = []
        for index, view in enumerate(VIEWS):
            cam = truth['cameras'][view]
            cam['K'][0][0] *= 1.015
            cam['K'][1][1] *= 1.015
            cam['D'] = [.008, -.001, 0, 0]
            cam['correction'] = dict(rotation_deg=[.2, -.5, .3])
            position, rotation = camera_transform(cam)
            points = rng.uniform([-3, -4, -.35], [7, 4, -.35], (500, 3))
            uv, theta = project_fisheye((points-position) @ rotation.T, cam['K'], cam['D'])
            visible = ((theta < np.deg2rad(75)) & (uv[:, 0] > 30) & (uv[:, 0] < 1250)
                       & (uv[:, 1] > 30) & (uv[:, 1] < 930))
            for i, (pixel, point) in enumerate(zip(uv[visible][:32], points[visible][:32])):
                cls.observations.append(dict(view=view, pixel=pixel.tolist(), ground_xy=point[:2].tolist(),
                    plane=dict(z_at_origin_m=-.35, dz_dx=0, dz_dy=0), point_id=view+str(i),
                    split='validation' if i % 4 == 0 else 'train'))

    def test_joint_lens_and_extrinsic_fit_improves_unseen_metric_points(self):
        result, report = fit_ground_calibration(self.initial, self.observations)
        self.assertTrue(report['accepted'], report)
        self.assertLess(report['validation']['after']['rmse_m'], .003)
        self.assertLess(report['validation']['after']['rmse_m'], report['validation']['before']['rmse_m']*.1)
        self.assertNotEqual(result['cameras']['front']['D'], self.initial['cameras']['front']['D'])
        for view in VIEWS:
            self.assertEqual(result['cameras'][view]['translation_m'], self.initial['cameras'][view]['translation_m'])
            self.assertEqual(result['cameras'][view]['rotation_deg'], self.initial['cameras'][view]['rotation_deg'])

    def test_data_leakage_and_insufficient_anchors_are_rejected(self):
        with self.assertRaises(ValueError):
            fit_ground_calibration(self.initial, self.observations[:20])
        contaminated = deepcopy(self.observations)
        contaminated[1]['point_id'] = contaminated[0]['point_id']
        with self.assertRaises(ValueError):
            fit_ground_calibration(self.initial, contaminated)

    def test_already_correct_rig_is_not_changed_by_needless_fit(self):
        observations = deepcopy(self.observations)
        for o in observations:
            cam = self.initial['cameras'][o['view']]
            t, r = camera_transform(cam)
            xyz = np.r_[o['ground_xy'], -.35]
            o['pixel'] = project_fisheye((xyz-t) @ r.T, cam['K'], cam['D'])[0].tolist()
        _, report = fit_ground_calibration(self.initial, observations)
        self.assertFalse(report['accepted'])


if __name__ == '__main__':
    unittest.main()
