import unittest
from types import SimpleNamespace

import numpy as np

from drivable_bev.svm_pose import (PoseSample, quaternion_matrix, interpolate_pose,
                                  ground_plane_from_pose, relative_pose, projection_pose, source_quaternion,
                                  offset_ground_plane)
from drivable_bev.fisheye_svm import (FisheyeSvm, pixels_to_ground, project_fisheye,
                                     camera_transform, validate_lens, VIEWS)
from drivable_bev.vehicle_geometry import load_svm_config
from pathlib import Path


def pose(stamp=100., arrival=10., position=(0, 0, 0), q=(0, 0, 0, 1)):
    return PoseSample(stamp, arrival, position, q)


class TestSvmPose(unittest.TestCase):
    def test_measured_morai_quaternions_recover_mounts_without_changing_ros(self):
        from drivable_bev.fisheye_svm import optical_rotation
        measured = [([0, .3007057607, 0, .9537169337], [0, 35, 0]),
                    ([-.3007057607, 0, -.9537169337, 0], [0, 35, 180]),
                    ([-.2705980837, -.2705980837, -.6532814503, -.6532814503], [0, 45, 90]),
                    ([-.2705980837, .2705980837, -.6532814503, .6532814503], [0, 45, 270])]
        optical = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]])
        for raw, angles in measured:
            corrected = source_quaternion(raw, 'morai_26r1_x_flipped')
            expected = optical_rotation(angles).T @ optical
            np.testing.assert_allclose(quaternion_matrix(corrected), expected, atol=2e-7)
            np.testing.assert_array_equal(source_quaternion(raw), raw)
            np.testing.assert_allclose(quaternion_matrix(source_quaternion(-np.array(raw),
                                       'morai_26r1_x_flipped')), expected, atol=2e-7)

    def test_quaternion_validation_and_sign(self):
        for bad in ([0, 0, 0, 0], [0, 0, float('nan'), 1], [0, 0, 1]):
            with self.assertRaises(ValueError):
                quaternion_matrix(bad)
        np.testing.assert_allclose(quaternion_matrix([0, 0, 0, -3]), np.eye(3))

    def test_pose_interpolates_translation_and_shortest_quaternion_arc(self):
        p = interpolate_pose([pose(), pose(100.1, position=(1, 0, 0), q=(0, 0, -1, -1))],
                             100.05, 10.1)
        np.testing.assert_allclose(p.position, [.5, 0, 0], atol=1e-10)
        np.testing.assert_allclose(p.rotation @ [1, 0, 0], [2**-.5, 2**-.5, 0], atol=1e-10)

    def test_missing_stale_and_wrong_clock_are_rejected(self):
        for stamp, now in ((100, 11), (100, 9), (101, 10), (0, 10)):
            self.assertIsNone(interpolate_pose([pose()], stamp, now))
        self.assertIsNone(interpolate_pose([pose(99.8), pose(100.01)], 100, 10))

    def test_roll_plane_has_correct_sign_and_height(self):
        angle = np.deg2rad(8)
        p = pose(q=(np.sin(angle/2), 0, 0, np.cos(angle/2)))
        plane = ground_plane_from_pose(p, .35)
        self.assertAlmostEqual(plane['dz_dy'], -np.tan(angle))
        self.assertAlmostEqual(plane['z_at_origin_m'], -.35/np.cos(angle))
        for x, y in ((1, 2), (-2, -1)):
            xyz = np.array([x, y, plane['z_at_origin_m'] + plane['dz_dx']*x + plane['dz_dy']*y])
            self.assertAlmostEqual((p.rotation @ xyz)[2], -.35)
        with self.assertRaises(ValueError):
            ground_plane_from_pose(p, .35, max_tilt_deg=5)

    def test_surface_normal_is_distinct_from_body_orientation(self):
        p = pose(q=(.03, .02, .1, 1))
        # Vehicle aligned with a graded road must still see level ground in its own frame.
        plane = ground_plane_from_pose(p, .35, p.rotation @ [0, 0, 1])
        np.testing.assert_allclose([plane['dz_dx'], plane['dz_dy']], [0, 0], atol=1e-10)

    def test_marking_height_offsets_surface_not_vehicle_and_uses_normal_distance(self):
        p = pose(q=(.04, -.02, .1, 1))
        road = ground_plane_from_pose(p, .35)
        marking = offset_ground_plane(road, .01)
        self.assertEqual(road['dz_dx'], marking['dz_dx'])
        for x, y in ((4., 2.), (-3., -1.)):
            z = marking['z_at_origin_m'] + marking['dz_dx']*x + marking['dz_dy']*y
            self.assertAlmostEqual((p.rotation @ [x, y, z])[2], -.34)
        for bad in (float('nan'), float('inf'), .36):
            with self.assertRaises(ValueError):
                offset_ground_plane(road, bad)
        c = load_svm_config(Path(__file__).resolve().parents[1] / 'config/fisheye_svm.yaml')
        c['projection_surface_offset_m'] = 0.
        road_svm = FisheyeSvm(c)
        c['projection_surface_offset_m'] = .01
        paint_svm = FisheyeSvm(c)
        np.testing.assert_array_equal(road_svm.body_mask, paint_svm.body_mask)
        self.assertEqual(road_svm.config['ground_z_m'], paint_svm.config['ground_z_m'])
        frames = {v: SimpleNamespace(stamp=100.) for v in VIEWS}
        result, _ = projection_pose([pose()], frames, 10.1, {}, .35, .01)
        self.assertAlmostEqual(result['plane']['z_at_origin_m'],
                               paint_svm.static_plane['z_at_origin_m'])

    def test_relative_pose_maps_same_world_point(self):
        a = pose(position=(12, 20, 5), q=(.01, .02, .1, 1))
        b = pose(position=(12.02, 20.03, 5), q=(.02, .01, .11, 1))
        r, t = relative_pose(a, b)
        xyz = np.array([3, 2, -.35])
        np.testing.assert_allclose(a.rotation @ (r @ xyz+t)+a.position,
                                   b.rotation @ xyz+b.position, atol=1e-10)

    def test_all_cameras_need_timestamped_pose(self):
        frames = {v: SimpleNamespace(stamp=100.) for v in VIEWS}
        result, _ = projection_pose([pose()], frames, 10.1, {}, .35)
        self.assertIsNotNone(result)
        frames['front'].stamp = 0
        self.assertIsNone(projection_pose([pose()], frames, 10.1, {}, .35)[0])
        frames['front'].stamp = 101
        self.assertIsNone(projection_pose([pose()], frames, 10.1, {}, .35)[0])

    def test_distorted_pixel_ground_roundtrip_and_no_horizon_intersection(self):
        c = load_svm_config(Path(__file__).resolve().parents[1] / 'config/fisheye_svm.yaml')
        camera = c['cameras']['front']
        camera['D'] = [.01, -.002, .0003, -.00004]
        camera['correction'] = dict(rotation_deg=[.5, -.3, .2])
        plane = dict(z_at_origin_m=-.35, dz_dx=.01, dz_dy=-.02)
        xy = np.array([[4.2, .4], [5, 1.1], [7, -1.5]])
        xyz = np.c_[xy, -.35+.01*xy[:, 0]-.02*xy[:, 1]]
        t, r = camera_transform(camera)
        uv, _ = project_fisheye((xyz-t) @ r.T, camera['K'], camera['D'])
        recovered, valid = pixels_to_ground(uv, camera, plane)
        self.assertTrue(valid.all())
        np.testing.assert_allclose(xy, recovered, atol=1e-8)
        self.assertFalse(pixels_to_ground([[640, 0]], camera, plane)[1][0])
        camera['D'] = [-1., 0, 0, 0]
        with self.assertRaises(ValueError):
            validate_lens(camera)

    def test_wide_positive_distortion_roundtrips_without_pi_over_two_clamp(self):
        c = load_svm_config(Path(__file__).resolve().parents[1] / 'config/fisheye_svm.yaml')
        camera = c['cameras']['front']
        camera.update(width=2048, height=2048, K=[[400., 0, 1024.], [0, 400., 1024.], [0, 0, 1]],
                      D=[.04, .001, 0, 0])
        angle = np.deg2rad(88)
        t, r = camera_transform(camera)
        ray = r.T @ [np.sin(angle), 0, np.cos(angle)]
        point = t + ray*(-.35-t[2])/ray[2]
        uv, _ = project_fisheye((point-t) @ r.T, camera['K'], camera['D'])
        xy, valid = pixels_to_ground([uv], camera, dict(z_at_origin_m=-.35))
        self.assertTrue(valid[0])
        np.testing.assert_allclose(xy[0], point[:2], atol=1e-7)

    def test_motion_projection_matches_independent_world_coordinates(self):
        c = load_svm_config(Path(__file__).resolve().parents[1] / 'config/fisheye_svm.yaml')
        c['output_size'] = 32
        c['projection_surface_offset_m'] = 0.
        svm = FisheyeSvm(c)
        reference = pose(position=(10, 30, 2), q=(.01, .02, .3, 1))
        earlier = pose(position=(9.96, 30.01, 2.002), q=(.012, .019, .29, 1))
        motion = {v: relative_pose(reference, earlier) for v in VIEWS}
        svm.set_projection(motion=motion)
        points = np.stack([svm.x, svm.y, np.full_like(svm.x, -.35)], axis=-1)
        world_points = points @ reference.rotation.T + reference.position
        for view in VIEWS:
            camera = c['cameras'][view]
            t, r = camera_transform(camera)
            camera_world = earlier.rotation @ t + earlier.position
            optical = (world_points-camera_world) @ earlier.rotation @ r.T
            uv, _ = project_fisheye(optical, camera['K'], camera['D'])
            np.testing.assert_allclose(svm.maps[view][0], uv[:, :, 0], atol=1e-4)
            np.testing.assert_allclose(svm.maps[view][1], uv[:, :, 1], atol=1e-4)

    def test_motion_does_not_occlude_front_camera_with_future_vehicle_body(self):
        c = load_svm_config(Path(__file__).resolve().parents[1] / 'config/fisheye_svm.yaml')
        svm = FisheyeSvm(c)
        motion = {v: (np.eye(3), np.array([-.2, 0., 0.])) for v in VIEWS}
        svm.set_projection(motion=motion)
        row, col = np.unravel_index(np.argmin((svm.x-4.5)**2+svm.y**2), svm.x.shape)
        self.assertGreater(svm.maps['front'][2][row, col], 0.)
        self.assertFalse(svm.body_mask[row, col])

    def test_dynamic_projection_resets_and_preserves_body(self):
        c = load_svm_config(Path(__file__).resolve().parents[1] / 'config/fisheye_svm.yaml')
        svm = FisheyeSvm(c)
        original = svm.maps['front'][0].copy()
        body = svm.body_mask.copy()
        self.assertFalse(svm.set_projection())
        self.assertTrue(svm.set_projection(dict(z_at_origin_m=-.35, dz_dx=.02, dz_dy=-.01)))
        self.assertGreater(np.max(np.abs(svm.maps['front'][0]-original)), 1)
        np.testing.assert_array_equal(svm.body_mask, body)
        svm.set_projection()
        np.testing.assert_array_equal(svm.maps['front'][0], original)


if __name__ == '__main__':
    unittest.main()
