"""
vip3_eval.parking_eval

주차 성공을 숫자로 판정하는 순수 함수 모음.
ROS·시뮬레이터 없이 좌표만 넣으면 동작한다.

나중에 ROS 노드를 씌울 때는:
    /Ego_topic 에서 (x, y, heading) 을 뽑아 evaluate_parking() 에 넣고,
    vip3_parking_spaces.json 의 항목 하나를 target_space 로 그대로 넘기면 된다.
"""

import math


def _wrap_deg(angle_deg):
    """각도를 [-180, 180) 범위로 정규화한다."""
    return (angle_deg + 180.0) % 360.0 - 180.0


def _rectangle_corners(center_x, center_y, heading_deg, width, length):
    """
    중심점 + 방향 + 폭 + 길이로 직사각형 네 모서리를 계산한다.
    순서: 앞왼쪽, 앞오른쪽, 뒤오른쪽, 뒤왼쪽 (heading 방향이 '앞'이다).

    width  : 차폭 방향 치수 (m)
    length : 진행 방향 치수 (m)
    """
    theta = math.radians(heading_deg)
    cos_t, sin_t = math.cos(theta), math.sin(theta)

    half_w = width / 2.0
    half_l = length / 2.0

    # 차체 좌표계에서의 네 모서리 (앞왼쪽부터 시계방향)
    local_corners = [
        (half_l, half_w),
        (half_l, -half_w),
        (-half_l, -half_w),
        (-half_l, half_w),
    ]

    world_corners = []
    for lx, ly in local_corners:
        wx = center_x + lx * cos_t - ly * sin_t
        wy = center_y + lx * sin_t + ly * cos_t
        world_corners.append((wx, wy))
    return world_corners


def _target_corners(target_space):
    """
    target_space(dict, vip3_parking_spaces.SCHEMA.md 형식)에서
    목표 칸의 네 모서리를 뽑는다.

    'points' 가 있으면 그대로 쓰고(z 좌표는 버림),
    없으면 center_point + angle + width + length 로 계산한다.
    """
    points = target_space.get("points")
    if points:
        return [(p[0], p[1]) for p in points]

    cx, cy = target_space["center_point"][:2]
    angle = target_space.get("angle", 90.0)
    width = target_space.get("width", 2.5)
    length = target_space.get("length", 5.0)
    return _rectangle_corners(cx, cy, angle, width, length)


def corner_max_error_m(car_pose, target_space):
    """
    차량이 목표 칸과 같은 크기의 사각형을 이뤘다고 가정했을 때,
    그 사각형 네 모서리와 목표 칸 네 모서리 사이 최대 거리(m).

    car_pose: (x, y, heading_deg)
    target_space: vip3_parking_spaces.SCHEMA.md 형식의 dict
    """
    car_x, car_y, car_heading = car_pose
    width = target_space.get("width", 2.5)
    length = target_space.get("length", 5.0)

    achieved_corners = _rectangle_corners(car_x, car_y, car_heading, width, length)
    target_corners = _target_corners(target_space)

    errors = [
        math.hypot(ax - tx, ay - ty)
        for (ax, ay), (tx, ty) in zip(achieved_corners, target_corners)
    ]
    return max(errors)


def heading_error_deg(car_pose, target_space):
    """차량 방향과 목표 칸 방향의 절대 각도 차이(deg), 0~180 범위."""
    _, _, car_heading = car_pose
    target_heading = target_space.get("angle", 90.0)
    return abs(_wrap_deg(car_heading - target_heading))


def evaluate_parking(
    run_id,
    car_pose,
    target_space,
    elapsed_time_s,
    collision_count,
    corner_threshold_m,
    heading_threshold_deg,
):
    """
    주차 한 번의 결과를 판정한다. 시뮬레이터·ROS 불필요한 순수 함수.

    car_pose: (x, y, heading_deg) - 차량 최종 위치 (예: /Ego_topic 에서 뽑은 값)
    target_space: vip3_parking_spaces.json 항목 하나 (dict)
    elapsed_time_s: 주차에 걸린 시간(초) - 밖에서 측정해서 넘긴다
    collision_count: 충돌 횟수 - 밖에서 측정해서 넘긴다
    corner_threshold_m: 코너오차가 이 값 이내면 성공 조건 중 하나를 만족
    heading_threshold_deg: heading오차가 이 값 이내면 성공 조건 중 하나를 만족

    반환: dict (CSV 한 줄에 대응)
    """
    # 부동소수점 오차로 경계값 비교가 잘못되지 않도록 mm 단위로 반올림 후 비교
    corner_err = round(corner_max_error_m(car_pose, target_space), 3)
    heading_err = round(heading_error_deg(car_pose, target_space), 3)

    success = (
        corner_err <= corner_threshold_m
        and heading_err <= heading_threshold_deg
        and collision_count == 0
    )

    return {
        "run_id": run_id,
        "corner_max_error_m": corner_err,
        "heading_error_deg": heading_err,
        "elapsed_time_s": round(elapsed_time_s, 3),
        "collision_count": collision_count,
        "success": success,
    }


CSV_FIELDS = [
    "run_id",
    "corner_max_error_m",
    "heading_error_deg",
    "elapsed_time_s",
    "collision_count",
    "success",
]


def result_to_csv_row(result):
    """evaluate_parking() 반환값을 CSV 한 줄(문자열)로 바꾼다."""
    return ",".join(str(result[field]) for field in CSV_FIELDS)
