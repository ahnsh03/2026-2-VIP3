"""
python3 -m unittest discover -s src/vip3_eval/test -p 'test_*.py'
로 실행한다.
"""

import unittest

from parking_eval import evaluate_parking, corner_max_error_m, heading_error_deg


# 참고: 주차칸 폭 2.5 m, 차폭 1.892 m -> 좌우 여유 30 cm 씩.
# 아래 두 값은 잠정 기준선이며, 실제 사용 전 재검토가 필요하다.
CORNER_THRESHOLD_M = 0.30
HEADING_THRESHOLD_DEG = 10.0

# 테스트용 목표 칸 (임의 값).
# 폭 2.5 m x 길이 5.0 m, 원점에 정렬된 직각주차 칸.
FAKE_TARGET_SPACE = {
    "idx": "P0001",
    "center_point": [0.0, 0.0, 0.0],
    "width": 2.5,
    "length": 5.0,
    "angle": 90.0,
}


class TestParkingEval(unittest.TestCase):
    def test_perfectly_centered_and_aligned(self):
        """칸 정중앙, 정렬됨 -> 코너오차 ~ 0, heading오차 ~ 0, 성공."""
        car_pose = (0.0, 0.0, 90.0)  # 목표와 완전히 같은 자세
        result = evaluate_parking(
            run_id="test_centered",
            car_pose=car_pose,
            target_space=FAKE_TARGET_SPACE,
            elapsed_time_s=12.0,
            collision_count=0,
            corner_threshold_m=CORNER_THRESHOLD_M,
            heading_threshold_deg=HEADING_THRESHOLD_DEG,
        )
        self.assertAlmostEqual(result["corner_max_error_m"], 0.0, places=3)
        self.assertAlmostEqual(result["heading_error_deg"], 0.0, places=3)
        self.assertTrue(result["success"])

    def test_shifted_30cm(self):
        """30 cm 밀림 -> 코너오차 ~ 0.30."""
        car_pose = (0.30, 0.0, 90.0)  # x 방향으로 30cm 이동
        corner_err = corner_max_error_m(car_pose, FAKE_TARGET_SPACE)
        self.assertAlmostEqual(corner_err, 0.30, places=3)

    def test_rotated_10deg(self):
        """10도 틀어짐 -> heading오차 ~ 10."""
        car_pose = (0.0, 0.0, 100.0)  # 90도 -> 100도, 10도 틀어짐
        heading_err = heading_error_deg(car_pose, FAKE_TARGET_SPACE)
        self.assertAlmostEqual(heading_err, 10.0, places=3)

    def test_collision_forces_failure(self):
        """오차가 다 기준 이내여도 충돌이 있으면 실패로 판정한다."""
        car_pose = (0.0, 0.0, 90.0)
        result = evaluate_parking(
            run_id="test_collision",
            car_pose=car_pose,
            target_space=FAKE_TARGET_SPACE,
            elapsed_time_s=8.0,
            collision_count=1,
            corner_threshold_m=CORNER_THRESHOLD_M,
            heading_threshold_deg=HEADING_THRESHOLD_DEG,
        )
        self.assertFalse(result["success"])

    def test_heading_wraparound(self):
        """헤딩이 -170 vs 170 처럼 경계를 넘어도 오차가 20으로 정상 계산된다."""
        target = dict(FAKE_TARGET_SPACE, angle=170.0)
        car_pose = (0.0, 0.0, -170.0)
        heading_err = heading_error_deg(car_pose, target)
        self.assertAlmostEqual(heading_err, 20.0, places=3)


if __name__ == "__main__":
    unittest.main()
