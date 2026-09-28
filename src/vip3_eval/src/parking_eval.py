"""
vip3_eval.parking_eval

주차 결과를 차량 외곽(footprint) 기준으로 평가하는 순수 함수 모음.
ROS·시뮬레이터 없이 좌표만 넣으면 동작한다.

좌표계 가정: 평면 ENU [m], heading 은 +x 축에서 반시계 방향 [deg].
"""

import math

# 차량 제원: Hyundai IONIQ 5 (m). 차폭은 제원표 1.890, 여기서는 1.892 사용.
VEHICLE_LENGTH_M = 4.635
VEHICLE_WIDTH_M = 1.892
# 후륜축 ~ 차량 기하 중심 거리 = 전장/2 - 후방 오버행(0.790)
REAR_AXLE_TO_CENTER_M = 1.5275

# 논문 비교용 성공 기준: E2E Parking 벤치마크의 TSR 정의
# (차량 중심이 칸 중심에서 횡 0.6 m, 종 1.0 m 이내, 방향 오차 10° 이하)
TSR_LATERAL_M = 0.6
TSR_LONGITUDINAL_M = 1.0
TSR_HEADING_DEG = 10.0


def _wrap_deg(angle_deg):
    """각도를 [-180, 180) 범위로 정규화한다."""
    return (angle_deg + 180.0) % 360.0 - 180.0


def _rect_corners(cx, cy, heading_deg, length, width):
    """
    중심·방향·길이·폭으로 직사각형 네 모서리를 계산한다.
    순서: 앞왼, 앞오른, 뒤오른, 뒤왼 (heading 방향이 앞).
    """
    theta = math.radians(heading_deg)
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    half_l, half_w = length / 2.0, width / 2.0

    local = [(half_l, half_w), (half_l, -half_w), (-half_l, -half_w), (-half_l, half_w)]
    return [
        (cx + lx * cos_t - ly * sin_t, cy + lx * sin_t + ly * cos_t)
        for lx, ly in local
    ]


def evaluate_parking(
    run_id,
    car_pose,
    target_space,
    elapsed_time_s,
    collision_count,
    ref_to_center_m=0.0,
    vehicle_length_m=VEHICLE_LENGTH_M,
    vehicle_width_m=VEHICLE_WIDTH_M,
):
    """
    주차 한 번의 결과를 평가한다.

    car_pose: (x, y, heading_deg) - Ego 기준점의 최종 자세
    target_space: dict - center_point, angle, length, width 필요 (칸 길이 방향이 angle)
    elapsed_time_s, collision_count: 밖에서 측정해서 넘긴다
    ref_to_center_m: Ego 기준점에서 차량 기하 중심까지의 전방 거리.
        기준점이 차량 중심이면 0, 후륜축이면 REAR_AXLE_TO_CENTER_M.

    반환 지표:
        corner_max_error_m   실제 차량 외곽과 이상 자세 외곽의 대응 모서리 간 최대 거리
        lateral_error_m      차량 중심의 칸 폭 방향 오차
        longitudinal_error_m 차량 중심의 칸 길이 방향 오차
        heading_error_deg    방향 오차
        slot_margin_m        차량 외곽에서 칸 경계까지 최소 거리 (음수면 칸 밖)
        success_tsr          논문 비교용 성공 (TSR 정의 + 무충돌)
        success_in_slot      차량 외곽이 칸 안에 완전히 들어감 + 무충돌
    """
    x, y, car_heading = car_pose
    theta = math.radians(car_heading)
    car_cx = x + ref_to_center_m * math.cos(theta)
    car_cy = y + ref_to_center_m * math.sin(theta)

    slot_cx, slot_cy = target_space["center_point"][:2]
    slot_angle = target_space["angle"]
    slot_len = target_space["length"]
    slot_wid = target_space["width"]

    # 칸 좌표계: u = 길이 방향, v = 폭 방향
    a = math.radians(slot_angle)
    ux, uy = math.cos(a), math.sin(a)
    vx, vy = -math.sin(a), math.cos(a)

    dx, dy = car_cx - slot_cx, car_cy - slot_cy
    longitudinal = abs(dx * ux + dy * uy)
    lateral = abs(dx * vx + dy * vy)
    heading_err = abs(_wrap_deg(car_heading - slot_angle))

    achieved = _rect_corners(car_cx, car_cy, car_heading, vehicle_length_m, vehicle_width_m)
    ideal = _rect_corners(slot_cx, slot_cy, slot_angle, vehicle_length_m, vehicle_width_m)
    corner_err = max(math.hypot(px - qx, py - qy) for (px, py), (qx, qy) in zip(achieved, ideal))

    margin = min(
        min(
            slot_len / 2.0 - abs((px - slot_cx) * ux + (py - slot_cy) * uy),
            slot_wid / 2.0 - abs((px - slot_cx) * vx + (py - slot_cy) * vy),
        )
        for px, py in achieved
    )

    # 부동소수점 오차로 경계값 비교가 잘못되지 않도록 mm 단위로 반올림 후 비교
    corner_err = round(corner_err, 3)
    lateral = round(lateral, 3)
    longitudinal = round(longitudinal, 3)
    heading_err = round(heading_err, 3)
    margin = round(margin, 3)

    no_collision = collision_count == 0
    success_tsr = (
        lateral <= TSR_LATERAL_M
        and longitudinal <= TSR_LONGITUDINAL_M
        and heading_err <= TSR_HEADING_DEG
        and no_collision
    )
    success_in_slot = margin >= 0 and no_collision

    return {
        "run_id": run_id,
        "corner_max_error_m": corner_err,
        "lateral_error_m": lateral,
        "longitudinal_error_m": longitudinal,
        "heading_error_deg": heading_err,
        "slot_margin_m": margin,
        "elapsed_time_s": round(elapsed_time_s, 3),
        "collision_count": collision_count,
        "success_tsr": success_tsr,
        "success_in_slot": success_in_slot,
    }


CSV_FIELDS = [
    "run_id",
    "corner_max_error_m",
    "lateral_error_m",
    "longitudinal_error_m",
    "heading_error_deg",
    "slot_margin_m",
    "elapsed_time_s",
    "collision_count",
    "success_tsr",
    "success_in_slot",
]


def result_to_csv_row(result):
    """evaluate_parking() 반환값을 CSV 한 줄(문자열)로 바꾼다."""
    return ",".join(str(result[field]) for field in CSV_FIELDS)