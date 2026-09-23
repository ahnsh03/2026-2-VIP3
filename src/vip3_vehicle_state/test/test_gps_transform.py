#!/usr/bin/env python3
"""GPS/IMU 좌표 변환 단위 테스트.

이 모듈은 **평소에는 안 쓰는 예비 경로**다. `/Ego_topic` 이 GT pose 를 주므로
정상 상황에서는 필요 없다. 다만 `/Ego_topic.position` 의 원점이 MGeo local frame 과
다르다는 것이 확인되면(docs/katri-map.md §3 의 미확인 항목) 이쪽으로 갈아타야 하므로,
그때 처음 검증하는 일이 없도록 지금 테스트를 걸어 둔다.

KATRI 맵 원점(UTM52N 302459.942, 4122635.537)을 기준으로 검산한다.
"""

from __future__ import annotations

import math
import unittest

from vip3_vehicle_state.gps_transform import (
    gps_to_enu,
    gps_to_utm52n,
    quaternion_to_yaw,
)

# KATRI global_info.local_origin_in_global
KATRI_ORIGIN_E = 302459.942
KATRI_ORIGIN_N = 4122635.537


class UtmTest(unittest.TestCase):
    def test_central_meridian_gives_false_easting(self):
        """UTM52N 중앙자오선(129E)에서 easting 은 정확히 500000 이어야 한다."""
        easting, _ = gps_to_utm52n(37.0, 129.0)
        self.assertAlmostEqual(500000.0, easting, delta=0.01)

    def test_equator_gives_zero_northing(self):
        _, northing = gps_to_utm52n(0.0, 129.0)
        self.assertAlmostEqual(0.0, northing, delta=0.05)

    def test_katri_origin_is_reachable_from_a_korean_lat_lon(self):
        """KATRI 원점 UTM 좌표에 대응하는 위경도가 실제로 존재하고 1 m 안에서 재현된다.

        역변환 구현이 없으므로 위도·경도를 번갈아 이분 탐색한다. easting 이 위도에도
        약하게 의존하므로 한 번씩만 하면 수렴하지 않는다 (몇 m 남는다).
        """
        latitude, longitude = 37.2, 126.77
        for _ in range(6):
            low, high = latitude - 0.2, latitude + 0.2
            for _ in range(50):
                middle = 0.5 * (low + high)
                if gps_to_utm52n(middle, longitude)[1] < KATRI_ORIGIN_N:
                    low = middle
                else:
                    high = middle
            latitude = low
            low, high = longitude - 0.5, longitude + 0.5
            for _ in range(50):
                middle = 0.5 * (low + high)
                if gps_to_utm52n(latitude, middle)[0] < KATRI_ORIGIN_E:
                    low = middle
                else:
                    high = middle
            longitude = low
        easting, northing = gps_to_utm52n(latitude, longitude)
        self.assertAlmostEqual(KATRI_ORIGIN_E, easting, delta=1.0)
        self.assertAlmostEqual(KATRI_ORIGIN_N, northing, delta=1.0)
        # 경기 남부(화성 KATRI) 범위 안이어야 한다. 완전히 틀린 투영이면 여기서 걸린다.
        self.assertTrue(36.5 < latitude < 37.8, latitude)
        self.assertTrue(126.0 < longitude < 127.5, longitude)

    def test_east_step_shows_the_expected_grid_convergence(self):
        """정동쪽으로 움직여도 northing 이 조금 변한다 — UTM 자오선 수렴각 때문이다.

        처음에 '변하지 않아야 한다'고 썼다가 20.85 m 차이로 실패했다. 버그가 아니라
        투영의 성질이다. 수렴각 gamma ~= dlon * sin(lat) 로 예측한 값과 맞는지 본다.
        """
        latitude = 37.2
        e0, n0 = gps_to_utm52n(latitude, 126.77)
        e1, n1 = gps_to_utm52n(latitude, 126.78)
        east_step = e1 - e0
        self.assertGreater(east_step, 800.0)

        convergence = math.radians(126.77 + 0.005 - 129.0) * math.sin(
            math.radians(latitude)
        )
        predicted = east_step * math.tan(convergence)
        self.assertAlmostEqual(predicted, n1 - n0, delta=1.0)
        self.assertLess(n1 - n0, 0.0, "중앙자오선 서쪽이라 northing 이 줄어야 한다")


class EnuTest(unittest.TestCase):
    def test_reference_point_maps_to_origin(self):
        east, north, up = gps_to_enu(37.2, 126.77, 30.0, 37.2, 126.77, 30.0)
        for value in (east, north, up):
            self.assertAlmostEqual(0.0, value, delta=1e-6)

    def test_north_step_is_positive_north(self):
        east, north, _ = gps_to_enu(37.201, 126.77, 30.0, 37.2, 126.77, 30.0)
        self.assertGreater(north, 100.0)
        self.assertLess(abs(east), 1.0)

    def test_east_step_is_positive_east(self):
        east, north, _ = gps_to_enu(37.2, 126.771, 30.0, 37.2, 126.77, 30.0)
        self.assertGreater(east, 80.0)
        self.assertLess(abs(north), 1.0)

    def test_altitude_difference_becomes_up(self):
        _, _, up = gps_to_enu(37.2, 126.77, 35.0, 37.2, 126.77, 30.0)
        self.assertAlmostEqual(5.0, up, delta=0.01)

    def test_enu_and_utm_agree_in_length_but_differ_by_convergence(self):
        """ENU 와 UTM 은 **길이**가 거의 같고 **방향**만 수렴각만큼 돌아가 있다.

        둘을 성분별로 같다고 보면 안 된다 (처음에 그렇게 썼다가 틀렸다).
        UTM 은 격자 북쪽, ENU 는 진북 기준이다.
        """
        east, north, _ = gps_to_enu(37.201, 126.771, 30.0, 37.2, 126.77, 30.0)
        e0, n0 = gps_to_utm52n(37.2, 126.77)
        e1, n1 = gps_to_utm52n(37.201, 126.771)
        grid_east, grid_north = e1 - e0, n1 - n0

        enu_length = math.hypot(east, north)
        utm_length = math.hypot(grid_east, grid_north)
        # UTM 축척(중앙자오선 0.9996, 여기서는 약간 확대)까지 포함해 0.2% 이내.
        self.assertAlmostEqual(1.0, utm_length / enu_length, delta=0.002)

        rotation = math.atan2(grid_north, grid_east) - math.atan2(north, east)
        expected = math.radians(126.77 - 129.0) * math.sin(math.radians(37.2))
        self.assertAlmostEqual(expected, rotation, delta=math.radians(0.05))


class YawTest(unittest.TestCase):
    def test_identity_quaternion_is_zero_yaw(self):
        self.assertAlmostEqual(0.0, quaternion_to_yaw(1.0, 0.0, 0.0, 0.0), places=9)

    def test_known_yaw_round_trips(self):
        for yaw in (0.0, 0.5, -1.2, math.pi / 2.0, -math.pi / 2.0):
            half = 0.5 * yaw
            value = quaternion_to_yaw(math.cos(half), 0.0, 0.0, math.sin(half))
            self.assertAlmostEqual(yaw, value, places=9)


if __name__ == "__main__":
    unittest.main()
