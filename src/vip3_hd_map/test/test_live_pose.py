import math
import unittest

import numpy as np

from vip3_hd_map.live_pose import (
    projected_gps_to_base_link,
    quaternion_yaw_rad,
    valid_wgs84_position,
)


class LivePoseTest(unittest.TestCase):
    def test_quaternion_yaw_matches_morai_heading(self):
        yaw = math.radians(60.0)
        self.assertAlmostEqual(
            quaternion_yaw_rad(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)),
            yaw,
            places=12,
        )
        # q and -q encode the same rotation.
        self.assertAlmostEqual(
            quaternion_yaw_rad(0.0, 0.0, -math.sin(yaw / 2.0), -math.cos(yaw / 2.0)),
            yaw,
            places=12,
        )

    def test_gps_lever_arm_is_removed_in_vehicle_heading(self):
        origin = (302595.0, 4124145.0)
        base = np.asarray((10.0, -4.0))
        for degrees in (0.0, 60.0, 90.0, -135.0):
            yaw = math.radians(degrees)
            lever = 0.35 * np.asarray((math.cos(yaw), math.sin(yaw)))
            gps = np.asarray(origin) + base + lever
            recovered = projected_gps_to_base_link(gps, origin, yaw)
            self.assertTrue(np.allclose(recovered, base, atol=1e-9))

    def test_invalid_quaternion_is_rejected(self):
        with self.assertRaises(ValueError):
            quaternion_yaw_rad(0.0, 0.0, 0.0, 0.0)

    def test_morai_zero_coordinate_outage_is_rejected(self):
        self.assertFalse(valid_wgs84_position(0.0, 0.0))
        self.assertFalse(valid_wgs84_position(float("nan"), 127.0))
        self.assertFalse(valid_wgs84_position(37.0, 181.0))
        self.assertTrue(valid_wgs84_position(37.123, 127.456))


if __name__ == "__main__":
    unittest.main()
