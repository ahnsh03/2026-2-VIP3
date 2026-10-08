"""parking_eval 단위 테스트.

실제 주차칸 데이터(parking_space_set.json)에서 테스트에 쓰는 칸:
  - A5256W000008 : 정상 동작 확인용. 4점, 모서리 직각, 약 4.9 x 2.5m.
  - A5256W000005 : 큰 구역 경고 확인용. 약 29 x 2.3m (한 칸이 아닐 수 있음).
  - A5256W000032 : 사선(평행사변형) 경고 확인용. 모서리 약 41도/139도.

주의:
  - 이 파일의 width/length/angle(2.5/5/90)은 31칸 모두 같은 기본값이라 쓰지 않고,
    points 좌표로 중심, 크기, 방향을 계산한다.
  - 위 칸이 실제 주차칸인지는 파일에 종류 정보가 없어 확인하지 못했다.
    크기와 모양으로만 고른 것이며, 시뮬레이터에서 위치를 확인해야 한다.
  - 테스트가 파일 위치에 의존하지 않도록 좌표를 이 파일에 복사해 두었다.

python3 -m unittest discover -s src/vip3_eval/test -p 'test_*.py'
로 실행한다.
"""

import math
import unittest

from parking_eval import (
    REAR_AXLE_TO_CENTER_M,
    WARN_NOT_RECTANGULAR,
    WARN_TOO_LARGE,
    evaluate_parking,
    result_to_csv_row,
    slot_from_points,
)

# 실제 칸 좌표 (parking_space_set.json 에서 복사, mm 단위로 반올림)
W000008 = [
    [-132.023, -453.632, 28.411],
    [-133.237, -455.764, 28.411],
    [-137.495, -453.339, 28.411],
    [-136.257, -451.166, 28.411],
]
W000005 = [
    [-120.537, -443.426, 28.467],
    [-119.435, -441.426, 28.467],
    [-93.962, -455.301, 28.467],
    [-95.074, -457.249, 28.467],
]
W000032 = [
    [-129.454, -429.56, 28.5],
    [-123.923, -419.54, 28.5],
    [-116.591, -416.851, 28.5],
    [-122.185, -426.913, 28.5],
]
# 일부러 틀린 기본값(2.5/5/90)을 붙여, 계산 시 이 값이 무시되는지 확인한다.
STALE = {"width": 2.5, "length": 5, "angle": 90}

# 테스트용 목표 칸 (임의 값): 폭 2.5 m x 길이 5.0 m, +y 방향 직각주차 칸.
# 아이오닉 5(4.635 x 1.892 m) 기준 정렬 시 좌우 여유 0.304 m, 앞뒤 여유 0.1825 m.
TARGET = {
    "idx": "P0001",
    "center_point": [0.0, 0.0, 0.0],
    "width": 2.5,
    "length": 5.0,
    "angle": 90.0,
}


def run(pose, collisions=0, **kwargs):
    return evaluate_parking("test", pose, TARGET, 10.0, collisions, **kwargs)


class TestParkingEval(unittest.TestCase):
    def test_centered_and_aligned(self):
        """칸 정중앙, 정렬됨 -> 모든 오차 0, 두 기준 모두 성공."""
        r = run((0.0, 0.0, 90.0))
        self.assertAlmostEqual(r["corner_max_error_m"], 0.0, places=3)
        self.assertAlmostEqual(r["lateral_error_m"], 0.0, places=3)
        self.assertAlmostEqual(r["longitudinal_error_m"], 0.0, places=3)
        self.assertAlmostEqual(r["heading_error_deg"], 0.0, places=3)
        self.assertAlmostEqual(r["slot_margin_m"], 0.1825, places=2)
        self.assertTrue(r["success_tsr"])
        self.assertTrue(r["success_in_slot"])

    def test_lateral_shift_30cm(self):
        """30 cm 밀림 -> 횡오차·코너오차 0.30, 여유가 0.304 이므로 칸 안."""
        r = run((0.30, 0.0, 90.0))
        self.assertAlmostEqual(r["lateral_error_m"], 0.30, places=3)
        self.assertAlmostEqual(r["corner_max_error_m"], 0.30, places=3)
        self.assertTrue(r["success_in_slot"])

    def test_lateral_shift_31cm_leaves_slot(self):
        """31 cm 밀림 -> 칸 밖으로 나가지만 TSR 기준으로는 성공."""
        r = run((0.31, 0.0, 90.0))
        self.assertFalse(r["success_in_slot"])
        self.assertTrue(r["success_tsr"])

    def test_longitudinal_shift(self):
        """종방향 1.0 m 밀림 -> TSR 경계값, 칸 안 판정은 실패."""
        r = run((0.0, 1.0, 90.0))
        self.assertAlmostEqual(r["longitudinal_error_m"], 1.0, places=3)
        self.assertTrue(r["success_tsr"])
        self.assertFalse(r["success_in_slot"])

    def test_lateral_shift_exceeds_tsr(self):
        """횡 0.7 m -> TSR 실패."""
        self.assertFalse(run((0.7, 0.0, 90.0))["success_tsr"])

    def test_rotated_10deg(self):
        """10도 틀어짐 -> heading오차 10, TSR 경계값이나 칸 밖(약 -8.4 cm)."""
        r = run((0.0, 0.0, 100.0))
        self.assertAlmostEqual(r["heading_error_deg"], 10.0, places=3)
        self.assertAlmostEqual(r["slot_margin_m"], -0.084, places=2)
        self.assertTrue(r["success_tsr"])
        self.assertFalse(r["success_in_slot"])

    def test_rotated_5deg_stays_in_slot(self):
        """5도 틀어짐 -> 중앙 정렬이면 칸 안 (여유 약 10 cm)."""
        r = run((0.0, 0.0, 95.0))
        self.assertAlmostEqual(r["slot_margin_m"], 0.106, places=2)
        self.assertTrue(r["success_in_slot"])

    def test_corner_error_from_rotation(self):
        """중심 고정 10도 회전 -> 코너오차는 외곽 반대각선 반지름 기준 약 0.436 m."""
        r = run((0.0, 0.0, 100.0))
        self.assertAlmostEqual(r["corner_max_error_m"], 0.436, places=2)

    def test_collision_forces_failure(self):
        """오차가 작아도 충돌이 있으면 두 기준 모두 실패."""
        r = run((0.0, 0.0, 90.0), collisions=1)
        self.assertFalse(r["success_tsr"])
        self.assertFalse(r["success_in_slot"])

    def test_heading_wraparound(self):
        """-170 vs 170 처럼 경계를 넘어도 오차가 20으로 계산된다."""
        target = dict(TARGET, angle=170.0)
        r = evaluate_parking("test", target_space=target, car_pose=(0.0, 0.0, -170.0),
                             elapsed_time_s=1.0, collision_count=0)
        self.assertAlmostEqual(r["heading_error_deg"], 20.0, places=3)

    def test_rear_axle_reference(self):
        """기준점이 후륜축일 때 오프셋을 주면 중심 기준으로 정렬로 판정된다."""
        pose = (0.0, -REAR_AXLE_TO_CENTER_M, 90.0)
        r = run(pose, ref_to_center_m=REAR_AXLE_TO_CENTER_M)
        self.assertAlmostEqual(r["longitudinal_error_m"], 0.0, places=3)
        self.assertTrue(r["success_in_slot"])

    def test_rear_axle_reference_ignored_gives_error(self):
        """오프셋을 안 주면 같은 자세가 종방향 약 1.53 m 오차로 나온다."""
        r = run((0.0, -REAR_AXLE_TO_CENTER_M, 90.0))
        self.assertAlmostEqual(r["longitudinal_error_m"], REAR_AXLE_TO_CENTER_M, places=3)


class TestRealSlot(unittest.TestCase):
    """실제 지도 칸(points) 기준 테스트."""

    def setUp(self):
        self.slot = slot_from_points(W000008)
        self.target = dict(STALE, idx="A5256W000008", points=W000008)
        a = math.radians(self.slot["angle"])
        self.cx, self.cy = self.slot["center_point"][:2]
        self.heading = self.slot["angle"]
        self.u = (math.cos(a), math.sin(a))      # 칸 길이 방향
        self.v = (-math.sin(a), math.cos(a))     # 칸 폭 방향

    def pose(self, du=0.0, dv=0.0, heading=None):
        h = self.heading if heading is None else heading
        return (
            self.cx + du * self.u[0] + dv * self.v[0],
            self.cy + du * self.u[1] + dv * self.v[1],
            h,
        )

    def run_real(self, pose, collisions=0):
        return evaluate_parking("real", pose, self.target, 10.0, collisions)

    def test_size_from_points_not_defaults(self):
        """파일의 2.5/5/90 이 아니라 좌표에서 계산한 크기와 방향이 나온다."""
        self.assertAlmostEqual(self.slot["length"], 4.90, delta=0.05)
        self.assertAlmostEqual(self.slot["width"], 2.50, delta=0.05)
        self.assertAlmostEqual(self.slot["angle"], 150.3, delta=0.5)
        self.assertEqual(self.slot["warnings"], [])

    def test_centered_succeeds(self):
        """칸 정중앙, 정렬 -> 둘 다 성공, 경고 없음."""
        r = self.run_real(self.pose())
        self.assertAlmostEqual(r["lateral_error_m"], 0.0, places=2)
        self.assertAlmostEqual(r["heading_error_deg"], 0.0, places=2)
        self.assertGreater(r["slot_margin_m"], 0.0)
        self.assertTrue(r["success_tsr"])
        self.assertTrue(r["success_in_slot"])
        self.assertEqual(r["warnings"], [])

    def test_tail_in_also_succeeds(self):
        """앞뒤가 반대(후진 주차)여도 같은 칸이면 성공."""
        r = self.run_real(self.pose(heading=self.heading + 180.0))
        self.assertAlmostEqual(r["heading_error_deg"], 0.0, places=2)
        self.assertTrue(r["success_in_slot"])

    def test_shift_along_slot_width(self):
        """칸 폭 방향으로 0.3m -> TSR 성공, 0.7m -> TSR 실패. (칸이 비스듬해서 칸 기준으로 민다)"""
        self.assertTrue(self.run_real(self.pose(dv=0.3))["success_tsr"])
        r = self.run_real(self.pose(dv=0.7))
        self.assertAlmostEqual(r["lateral_error_m"], 0.7, places=2)
        self.assertFalse(r["success_tsr"])
        self.assertFalse(r["success_in_slot"])

    def test_collision_forces_failure(self):
        r = self.run_real(self.pose(), collisions=1)
        self.assertFalse(r["success_tsr"])
        self.assertFalse(r["success_in_slot"])

    def test_closing_point_is_optional(self):
        """마지막에 첫 점을 한 번 더 붙인 파일 형식도 같은 결과."""
        closed = slot_from_points(W000008 + [W000008[0]])
        self.assertAlmostEqual(closed["length"], self.slot["length"], places=6)
        self.assertAlmostEqual(closed["angle"], self.slot["angle"], places=6)

    def test_without_points_uses_old_fields(self):
        """points 가 없으면 기존 center_point/angle/length/width 방식."""
        target = {"center_point": [0.0, 0.0, 0.0], "width": 2.5, "length": 5.0, "angle": 90.0}
        r = evaluate_parking("old", (0.0, 0.0, 90.0), target, 1.0, 0)
        self.assertTrue(r["success_in_slot"])
        self.assertEqual(r["warnings"], [])


class TestSlotWarnings(unittest.TestCase):
    def test_large_zone_warns(self):
        """약 29 x 2.3m 구역 -> 한 칸이 아닐 수 있다는 경고."""
        s = slot_from_points(W000005)
        self.assertGreater(s["length"], 25.0)
        self.assertIn(WARN_TOO_LARGE, s["warnings"])

    def test_parallelogram_warns(self):
        """모서리 41/139도 사선 칸 -> 직사각형이 아니라는 경고."""
        s = slot_from_points(W000032)
        self.assertIn(WARN_NOT_RECTANGULAR, s["warnings"])

    def test_warnings_reach_result_and_csv(self):
        """경고는 평가 결과와 CSV 에도 나온다. 평가 자체는 막지 않는다."""
        target = {"points": W000005}
        c = slot_from_points(W000005)["center_point"]
        r = evaluate_parking("big", (c[0], c[1], 0.0), target, 1.0, 0)
        self.assertIn(WARN_TOO_LARGE, r["warnings"])
        self.assertIn(WARN_TOO_LARGE, result_to_csv_row(r))

    def test_too_few_points(self):
        with self.assertRaises(ValueError):
            slot_from_points([[0, 0, 0], [1, 0, 0]])


if __name__ == "__main__":
    unittest.main()