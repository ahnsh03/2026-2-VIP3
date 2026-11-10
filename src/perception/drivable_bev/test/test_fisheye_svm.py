import unittest
import tempfile
from pathlib import Path

import cv2
import numpy as np
import yaml

from drivable_bev.fisheye_svm import CameraFrame, FisheyeSvm, VIEWS, project_fisheye, select_frames
from drivable_bev.vehicle_geometry import load_svm_config, vehicle_geometry


class TestFisheyeSvm(unittest.TestCase):
    def test_projection_matches_opencv_with_distortion(self):
        k = np.array([[205., 0, 320], [0, 207., 240], [0, 0, 1.]])
        d = np.array([.01, -.002, .0003, -.00004])
        points = np.array([[0, 0, 1], [1, 0, .02], [-.2, .5, 1], [.3, -.6, .5]])
        expected, _ = cv2.fisheye.projectPoints(points.reshape(1, -1, 3), np.zeros(3), np.zeros(3), k, d)
        actual, _ = project_fisheye(points, k, d)
        np.testing.assert_allclose(actual, expected.reshape(-1, 2), atol=1e-9)

    def test_real_preset_has_coverage_and_body_is_masked(self):
        config = load_svm_config(Path(__file__).resolve().parents[1] / 'config/fisheye_svm_261110.yaml')
        svm = FisheyeSvm(config)
        images = {v: np.full((config['cameras'][v]['height'], config['cameras'][v]['width'], 3),
                             180, np.uint8) for v in VIEWS}
        image = svm.render(images)
        self.assertEqual(image.shape, (512, 512, 3))
        self.assertGreater(np.mean(svm.valid_mask > 0), .60)
        self.assertTrue(np.all(svm.valid_mask[svm.body_mask] == 0))
        # Constant radiance must remain constant through overlaps and boundaries.
        self.assertLessEqual(np.max(np.abs(image[svm.valid_mask > 0].astype(int) - 180)), 1)
        for view in VIEWS:
            self.assertGreater(np.count_nonzero(svm.maps[view][2]), 1000, view)
        with self.assertRaises(ValueError):
            svm.render({v: np.zeros((240, 320, 3), np.uint8) for v in VIEWS})

    def test_geometry_uses_rear_axle_not_body_center_origin(self):
        config = load_svm_config(Path(__file__).resolve().parents[1] / 'config/fisheye_svm_261110.yaml')
        self.assertAlmostEqual(config['center_x_m'], 1.5275)
        self.assertAlmostEqual(config['vehicle_mask']['x_min_m'], -0.79)
        self.assertAlmostEqual(config['vehicle_mask']['x_max_m'], 3.845)
        self.assertAlmostEqual(config['vehicle_mask']['y_max_m'], 0.946)
        self.assertAlmostEqual(config['ground_z_m'], -config['vehicle_geometry']['ground_reference']['base_link_height_m'])
        # Camera z is relative to the axle frame, never the documented body height.
        self.assertAlmostEqual(config['cameras']['front']['translation_m'][2] - config['ground_z_m'],
                               0.3 + config['vehicle_geometry']['ground_reference']['base_link_height_m'])
        config['vehicle_geometry']['documented_spec']['length_m'] += 1
        with self.assertRaises(ValueError):
            vehicle_geometry(config['vehicle_geometry'])

    def test_ground_slope_changes_projection_without_changing_body_mask(self):
        config = load_svm_config(Path(__file__).resolve().parents[1] / 'config/fisheye_svm_261110.yaml')
        flat = FisheyeSvm(config)
        config['vehicle_geometry']['ground_reference']['dz_dx'] = .02
        sloped = FisheyeSvm(config)
        np.testing.assert_array_equal(flat.body_mask, sloped.body_mask)
        self.assertGreater(np.max(np.abs(flat.maps['front'][1] - sloped.maps['front'][1])), 1)

    def test_saved_capture_replays_without_external_vehicle_config(self):
        config = load_svm_config(Path(__file__).resolve().parents[1] / 'config/fisheye_svm_261110.yaml')
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'config.yaml'
            path.write_text(yaml.safe_dump(config))
            restored = load_svm_config(path)
        self.assertEqual(restored, config)

    @staticmethod
    def buffers(stamps=(100, 100.01, 100.02, 100.03), arrivals=(10, 10, 10, 10)):
        return {v: [CameraFrame(None, stamp, arrival, v)]
                for v, stamp, arrival in zip(VIEWS, stamps, arrivals)}

    def test_simulation_clock_is_not_compared_to_wall_clock(self):
        frames, mode, span = select_frames(self.buffers(), 10.1, .5, .06)
        self.assertIsNotNone(frames)
        self.assertEqual(mode, 'header')
        self.assertAlmostEqual(span, .03)

    def test_stale_camera_invalidates_whole_set(self):
        buffers = self.buffers(arrivals=(10, 10, 10, 9))
        self.assertIsNone(select_frames(buffers, 10.1, .5, .06)[0])

    def test_unsynchronized_set_is_rejected(self):
        self.assertIsNone(select_frames(self.buffers(stamps=(100, 100, 100, 101)), 10.1, .5, .06)[0])

    def test_buffer_keeps_matching_previous_frame(self):
        buffers = self.buffers()
        buffers['front'].append(CameraFrame(None, 100.15, 10.12, 'front'))
        frames, _, _ = select_frames(buffers, 10.2, .5, .06)
        self.assertEqual(frames['front'].stamp, 100)

    def test_zero_stamps_use_arrivals_explicitly(self):
        frames, mode, _ = select_frames(self.buffers(stamps=(0, 0, 0, 0)), 10.1, .5, .06)
        self.assertIsNotNone(frames)
        self.assertEqual(mode, 'arrival (zero header)')


if __name__ == '__main__':
    unittest.main()
