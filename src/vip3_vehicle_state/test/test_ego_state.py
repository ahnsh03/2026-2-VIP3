#!/usr/bin/env python3
"""EgoVehicleStatus 변환 단위 테스트. ROS 없이 돈다."""

from __future__ import annotations

import math
import unittest

from vip3_vehicle_state.ego_state import convert_ego, yaw_to_quaternion


def make(**overrides):
    values = dict(
        stamp_s=1.0,
        position=(10.0, -4.0, 0.2),
        velocity=(2.0, 0.0),
        acceleration_x=0.5,
        angular_velocity_z_dps=18.0,
        heading_deg=90.0,
    )
    values.update(overrides)
    return convert_ego(**values)


class EgoStateTest(unittest.TestCase):
    def test_heading_degrees_become_radians(self):
        state = make(heading_deg=90.0)
        self.assertAlmostEqual(math.pi / 2.0, state.yaw, places=9)

    def test_angular_velocity_degrees_become_radians(self):
        # 26.R1 은 angular_velocity 를 deg/s 로 준다. beta_drive 에는 없던 필드다.
        state = make(angular_velocity_z_dps=180.0)
        self.assertAlmostEqual(math.pi, state.yaw_rate, places=9)

    def test_signed_speed_keeps_reverse_negative(self):
        """후진 주차에는 부호가 필요하다. ASMC 는 hypot 이라 항상 >= 0 이었다."""
        state = convert_ego(
            stamp_s=1.0,
            position=(1.0, 1.0, 0.0),
            velocity=(-1.5, 0.0),
            acceleration_x=0.0,
            angular_velocity_z_dps=0.0,
            heading_deg=0.0,
            signed_speed=True,
        )
        self.assertLess(state.speed, 0.0)

        unsigned = convert_ego(
            stamp_s=1.0,
            position=(1.0, 1.0, 0.0),
            velocity=(-1.5, 0.0),
            acceleration_x=0.0,
            angular_velocity_z_dps=0.0,
            heading_deg=0.0,
            signed_speed=False,
        )
        self.assertAlmostEqual(1.5, unsigned.speed, places=9)

    def test_map_frame_velocity_is_rotated_into_body(self):
        # heading 90 도에서 map +y 로 2 m/s 면 차량 기준으로는 전방 2 m/s 다.
        state = convert_ego(
            stamp_s=1.0,
            position=(0.0, 0.0, 0.0),
            velocity=(0.0, 2.0),
            acceleration_x=0.0,
            angular_velocity_z_dps=0.0,
            heading_deg=90.0,
            velocity_is_body_frame=False,
        )
        self.assertAlmostEqual(2.0, state.vx_body, places=6)
        self.assertAlmostEqual(0.0, state.vy_body, places=6)

    def test_zero_position_is_rejected(self):
        """MORAI 가 ego 스폰 전에 (0,0,0) 패킷을 보낸다. TF 로 쏘면 순간이동한다."""
        state = make(position=(0.0, 0.0, 0.0))
        self.assertFalse(state.valid)
        self.assertFalse(state.valid_ignoring_stamp())
        self.assertIn("position", state.reason())

    def test_zero_stamp_is_rejected_only_in_strict_mode(self):
        state = make(stamp_s=0.0)
        self.assertFalse(state.valid)
        self.assertTrue(state.valid_ignoring_stamp())
        self.assertIn("stamp", state.reason())

    def test_non_finite_is_rejected(self):
        state = make(position=(float("nan"), 0.0, 0.0))
        self.assertFalse(state.valid)
        self.assertFalse(state.valid_ignoring_stamp())

    def test_valid_state_passes(self):
        state = make()
        self.assertTrue(state.valid)
        self.assertEqual("", state.reason())

    def test_yaw_to_quaternion_round_trips(self):
        for yaw in (0.0, 0.5, -1.2, math.pi / 2.0):
            x, y, z, w = yaw_to_quaternion(yaw)
            self.assertAlmostEqual(0.0, x)
            self.assertAlmostEqual(0.0, y)
            self.assertAlmostEqual(yaw, 2.0 * math.atan2(z, w), places=9)


if __name__ == "__main__":
    unittest.main()
