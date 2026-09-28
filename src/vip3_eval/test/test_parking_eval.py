"""
python3 -m unittest discover -s src/vip3_eval/test -p 'test_*.py'
로 실행한다.
"""

import unittest

from parking_eval import REAR_AXLE_TO_CENTER_M, evaluate_parking

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


if __name__ == "__main__":
    unittest.main()