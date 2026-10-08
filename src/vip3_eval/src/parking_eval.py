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


# 목표 칸 경고 기준 (평가는 막지 않고 warnings 로만 알린다)
MAX_SLOT_LENGTH_M = 7.0   # 이보다 길면 한 칸이 아닐 수 있음
MAX_SLOT_WIDTH_M = 4.0    # 이보다 넓으면 한 칸이 아닐 수 있음
MIN_RECT_FILL = 0.95      # 칸 면적 / 외접 사각형 면적. 이보다 작으면 직사각형이 아님

WARN_TOO_LARGE = "slot_too_large"
WARN_NOT_RECTANGULAR = "slot_not_rectangular"


def _wrap_deg(angle_deg):
    """각도를 [-180, 180) 범위로 정규화한다."""
    return (angle_deg + 180.0) % 360.0 - 180.0


def _convex_hull(pts):
    """Andrew monotone chain. 반시계 방향 볼록 껍질을 반환한다."""
    pts = sorted(set(pts))
    if len(pts) < 3:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def _polygon_area(pts):
    """신발끈 공식으로 다각형 면적을 구한다."""
    s = 0.0
    for i in range(len(pts)):
        x0, y0 = pts[i]
        x1, y1 = pts[(i + 1) % len(pts)]
        s += x0 * y1 - x1 * y0
    return abs(s) / 2.0


def slot_from_points(points):
    """
    칸 꼭짓점(points)에서 중심·방향·길이·폭을 계산한다.

    꼭짓점 전체를 감싸는 가장 작은 직사각형(최소 외접 사각형)을 쓴다.
    파일의 width/length/angle 은 31칸 모두 2.5/5/90 기본값이라 쓰지 않는다.

    points: [[x, y, z], ...] 마지막 점이 첫 점과 같아도 된다.
    반환: dict(center_point, angle, length, width, warnings)
        angle 은 긴 변의 방향 [0, 180). 앞/뒤 방향은 정해지지 않는다.
    """
    pts = [(float(p[0]), float(p[1])) for p in points]
    if len(pts) > 1 and pts[0] == pts[-1]:
        pts = pts[:-1]
    if len(pts) < 3:
        raise ValueError("points 는 서로 다른 점이 3개 이상이어야 한다")
    z = sum(float(p[2]) for p in points if len(p) > 2) / max(
        1, sum(1 for p in points if len(p) > 2)
    )

    hull = _convex_hull(pts)
    best = None
    for i in range(len(hull)):
        x0, y0 = hull[i]
        x1, y1 = hull[(i + 1) % len(hull)]
        a = math.atan2(y1 - y0, x1 - x0)
        c, s = math.cos(a), math.sin(a)
        us = [x * c + y * s for x, y in hull]
        vs = [-x * s + y * c for x, y in hull]
        du, dv = max(us) - min(us), max(vs) - min(vs)
        if best is None or du * dv < best[0]:
            cu, cv = (max(us) + min(us)) / 2.0, (max(vs) + min(vs)) / 2.0
            best = (du * dv, a, du, dv, cu * c - cv * s, cu * s + cv * c)

    area, a, du, dv, cx, cy = best
    if du >= dv:
        length, width, angle = du, dv, math.degrees(a)
    else:
        length, width, angle = dv, du, math.degrees(a) + 90.0

    warnings = []
    if length > MAX_SLOT_LENGTH_M or width > MAX_SLOT_WIDTH_M:
        warnings.append(WARN_TOO_LARGE)
    if area > 0 and _polygon_area(pts) / area < MIN_RECT_FILL:
        warnings.append(WARN_NOT_RECTANGULAR)

    return {
        "center_point": [cx, cy, z],
        "angle": angle % 180.0,
        "length": length,
        "width": width,
        "warnings": warnings,
    }


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
    target_space: dict
        - points 가 있으면 거기서 중심·방향·크기를 계산한다 (slot_from_points).
        - 없으면 center_point, angle, length, width 를 쓴다 (칸 길이 방향이 angle).
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

    warnings = []
    if target_space.get("points"):
        # points 가 있으면 width/length/angle 은 무시하고 좌표에서 계산한다.
        slot = slot_from_points(target_space["points"])
        warnings = slot["warnings"]
        slot_angle = slot["angle"]
        # 지도 데이터에는 앞/뒤 방향이 없으므로 전진·후진 주차를 모두 허용한다.
        # 차 방향과 가까운 쪽(angle 또는 angle+180)을 기준으로 삼는다.
        if abs(_wrap_deg(car_heading - slot_angle)) > 90.0:
            slot_angle += 180.0
    else:
        slot = target_space
        slot_angle = slot["angle"]

    slot_cx, slot_cy = slot["center_point"][:2]
    slot_len = slot["length"]
    slot_wid = slot["width"]

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
        "warnings": warnings,
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
    "warnings",
]


def result_to_csv_row(result):
    """evaluate_parking() 반환값을 CSV 한 줄(문자열)로 바꾼다. 경고는 | 로 이어 붙인다."""
    return ",".join(
        "|".join(result[field]) if field == "warnings" else str(result[field])
        for field in CSV_FIELDS
    )