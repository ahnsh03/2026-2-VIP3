import subprocess
import sys
import unittest
import numpy as np
from vip3_perception.coordinates import pixel_to_vehicle, vehicle_to_pixel


class TestCoordinates(unittest.TestCase):
    def test_corners_body_centre_and_subpixel_roundtrip(self):
        rows = np.array([0., 511., 255.5, 123.456])
        cols = np.array([0., 511., 255.5, 321.123])
        x, y = pixel_to_vehicle(rows, cols, 512, 10., 1.5275)
        np.testing.assert_allclose([x[0], y[0]], [1.5275+5.-5./512, 5.-5./512])
        np.testing.assert_allclose([x[2], y[2]], [1.5275, 0.])
        r, c = vehicle_to_pixel(x, y, 512, 10., 1.5275)
        np.testing.assert_allclose(r, rows, atol=1e-12)
        np.testing.assert_allclose(c, cols, atol=1e-12)

    def test_svm_import_does_not_load_ros_ml_or_old_packages(self):
        subprocess.run([sys.executable, '-c',
            'import sys; from vip3_perception.svm.projection import FisheyeSvm; '
            'assert not any(n.split(".")[0] in {"torch", "rospy", "drivable_bev", '
            '"camera_semantic_perception", "twinlite_morai"} for n in sys.modules)'], check=True)
