import math
import unittest

import numpy as np

from vip3_hd_map.live_pose import (
    projected_gps_to_base_link,
    quaternion_yaw_rad,
    valid_wgs84_position,
)
from vip3_vehicle_state.gps_transform import gps_to_utm52n

# KATRI MGeo global_info.local_origin_in_global (EPSG:32652).
KATRI_ORIGIN_XY = (302459.942, 4122635.537)
# 위 UTM 좌표에 해당하는 WGS84 위경도. gps_to_utm52n 을 역으로 수치해서 구했다.
# 소수 7자리는 위경도 1e-7도 ~ 1 cm 라서 UTM 왕복에 mm 급 잔차가 남는다.
# 그래서 위치 단언은 cm 허용오차를 쓰고, 순수 평행이동 단언은 이 lat/lon 의
# UTM 결과를 원점으로 삼아 잔차를 소거한다.
KATRI_ORIGIN_LATLON = (37.2293241, 126.7732979)
ORIGIN_ROUNDING_M = 0.01
# VIP3_sensor_set_v1_ros.json 의 GPS pos.x
GPS_LEVER_ARM_M = 0.350


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
        origin = KATRI_ORIGIN_XY
        base = np.asarray((10.0, -4.0))
        for degrees in (0.0, 60.0, 90.0, -135.0):
            yaw = math.radians(degrees)
            lever = GPS_LEVER_ARM_M * np.asarray((math.cos(yaw), math.sin(yaw)))
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


class GpsToMapChainTest(unittest.TestCase):
    """`katri_map_viz_node` 의 `pose_source:=gps_imu` 경로 전체를 ROS 없이 돈다.

    이 경로는 한동안 죽어 있었다 — 노드가 `/gps`·`/imu` 를 구독해서 최신 메시지를
    저장만 하고 `_update()` 를 부르지 않아, `gps_imu` 를 고르면 마커가 **하나도**
    안 뜨는데 로그는 멀쩡했다. 노드는 rospy 없이 못 돌리므로 수학 사슬을 여기서 지킨다.
    """

    def test_katri_origin_round_trips(self):
        easting, northing = gps_to_utm52n(*KATRI_ORIGIN_LATLON)
        self.assertAlmostEqual(easting, KATRI_ORIGIN_XY[0], delta=ORIGIN_ROUNDING_M)
        self.assertAlmostEqual(northing, KATRI_ORIGIN_XY[1], delta=ORIGIN_ROUNDING_M)

    def test_vehicle_at_map_origin_maps_to_zero(self):
        """원점에 선 차량(yaw=0)의 GPS 안테나는 (0.35, 0) 에 있고 base_link 는 (0,0)."""

        latitude, longitude = KATRI_ORIGIN_LATLON
        # 안테나는 차량보다 0.35 m 앞(동쪽). 위도 1도 ~ 111 km 로 근사하지 않고
        # UTM 평면에서 직접 더한다.
        easting, northing = gps_to_utm52n(latitude, longitude)
        antenna = (easting + GPS_LEVER_ARM_M, northing)
        xy = projected_gps_to_base_link(
            antenna, KATRI_ORIGIN_XY, 0.0, (GPS_LEVER_ARM_M, 0.0)
        )
        self.assertTrue(np.allclose(xy, (0.0, 0.0), atol=ORIGIN_ROUNDING_M))

    def test_imu_quaternion_drives_the_lever_arm_direction(self):
        """후진 주차에서 yaw 가 180도면 안테나는 차량 뒤(서쪽)에 찍혀야 한다."""

        easting, northing = gps_to_utm52n(*KATRI_ORIGIN_LATLON)
        yaw = quaternion_yaw_rad(0.0, 0.0, math.sin(math.pi / 2.0), math.cos(math.pi / 2.0))
        self.assertAlmostEqual(abs(yaw), math.pi, places=9)
        antenna = (easting - GPS_LEVER_ARM_M, northing)
        xy = projected_gps_to_base_link(
            antenna, KATRI_ORIGIN_XY, yaw, (GPS_LEVER_ARM_M, 0.0)
        )
        self.assertTrue(np.allclose(xy, (0.0, 0.0), atol=ORIGIN_ROUNDING_M))

    def test_a_metre_east_is_a_metre_in_map_x(self):
        """맵 프레임은 UTM 을 평행이동만 한 것이다 — 회전·스케일이 끼면 안 된다."""

        easting, northing = gps_to_utm52n(*KATRI_ORIGIN_LATLON)
        xy = projected_gps_to_base_link(
            (easting + 1.0, northing + 2.0), (easting, northing), 0.0, (0.0, 0.0)
        )
        self.assertTrue(np.allclose(xy, (1.0, 2.0), atol=1e-9))


if __name__ == "__main__":
    unittest.main()
