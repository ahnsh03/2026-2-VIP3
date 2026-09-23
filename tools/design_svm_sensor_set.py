#!/usr/bin/env python3
"""SVM(어라운드뷰) 카메라 배치를 평가·탐색한다.

주차 슬롯 검출 논문 대부분이 **AVM/SVM 합성 영상**을 입력으로 가정한다. 그러려면 카메라
4대가 차 주변 360도 근거리 지면을 빈틈없이 덮어야 하는데, 지금 임시 센서셋은 그 반대다
(반경 1.65 m 안쪽 전부 사각, 후좌·후우 대각에 12 m대 쐐기).

이 도구는 `drivable_bev` 런타임과 **같은 캘리브레이션 코드**로 후보 배치를 평가한다.
따라서 여기서 좋게 나온 값은 BEV 에서도 같게 나온다.

    # 현재 센서셋 평가
    python3 tools/design_svm_sensor_set.py --sensor-set config/VIP3_sensor_set_v1_ros.json

    # 제안 SVM 배치 평가 + 비교 + 그림
    python3 tools/design_svm_sensor_set.py --preset svm_v2 \
        --compare config/VIP3_sensor_set_v1_ros.json --image /tmp/svm.png

    # FOV·pitch 격자 탐색
    python3 tools/design_svm_sensor_set.py --sweep

    # MORAI 센서셋 JSON 으로 내보내기 (기존 파일을 템플릿으로 삼는다)
    python3 tools/design_svm_sensor_set.py --preset svm_v2 \
        --export-sensor-set config/VIP3_sensor_set_v2_svm.json \
        --template config/VIP3_sensor_set_v1_ros.json

MORAI 카메라는 **핀홀**이다 (`f = w / (2*tan(fov/2))`). 진짜 어안(>=180 deg)이 아니라
초광각 핀홀이므로 `drivable_bev` 의 호모그래피가 그대로 동작한다. 대신 화면 가장자리의
지면 해상도가 급격히 떨어지는 것은 감수해야 한다 — 이 도구가 그걸 cm/px 로 보여준다.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
# drivable_bev 패키지의 __init__ 이 model_grid_projection 을 통해
# camera_semantic_perception 을 import 한다. 두 경로를 다 넣는다.
sys.path.insert(0, str(REPO_ROOT / "src" / "perception" / "drivable_bev" / "src"))
sys.path.insert(
    0, str(REPO_ROOT / "src" / "perception" / "camera_semantic_perception" / "src")
)

from drivable_bev.calibration import CameraCalibration, GroundPlane  # noqa: E402
from drivable_bev.grid import BevGridSpec  # noqa: E402
from drivable_bev.homography import build_homography  # noqa: E402

# 차량 제원 (2023 Hyundai Ioniq5 기준, base_link = 뒷바퀴축 중심).
# ⚠ VIP3 가 실제로 스폰하는 차량으로 확인 전까지 잠정값이다.
VEHICLE = {
    "length_m": 4.635,
    "width_m": 1.890,
    "wheelbase_m": 3.000,
    "rear_overhang_m": 0.790,
}
FRONT_BUMPER_X = VEHICLE["length_m"] - VEHICLE["rear_overhang_m"]   # +3.845
REAR_BUMPER_X = -VEHICLE["rear_overhang_m"]                          # -0.790
HALF_WIDTH = 0.5 * VEHICLE["width_m"]                                # 0.945

GROUND = GroundPlane(
    frame_id="base_link",
    model="plane",
    z_at_origin_m=-0.35,
    dz_dx=0.0,
    dz_dy=0.0,
    source="provisional_asmc_carryover_20260923",
)

# SVM 평가 격자. 주차면 하나가 2.5 x 5.0 m 이므로 좌우 한 칸 + 앞뒤 한 칸이 들어간다.
SVM_GRID = BevGridSpec(-6.0, 8.0, -5.0, 5.0, 0.05)

PRESETS = {
    # 지금 쓰는 임시 센서셋 (비교 기준)
    "current": {
        "front": dict(width=1280, height=720, fov=90.0,
                      t=(1.900, 0.000, 1.200), rot=(0.0, 2.0, 0.0)),
        "left": dict(width=640, height=480, fov=130.0,
                     t=(1.150, 0.650, 1.200), rot=(0.0, 10.0, 70.0)),
        "right": dict(width=640, height=480, fov=130.0,
                      t=(1.150, -0.650, 1.200), rot=(0.0, 10.0, -70.0)),
        "rear": dict(width=1280, height=720, fov=90.0,
                     t=(-0.100, 0.000, 1.200), rot=(0.0, 2.0, 180.0)),
    },
    # 권장 SVM 배치. 근거는 README / docs/sensors.md §5.
    # 4:3 은 같은 수평 FOV 에서 수직 화각을 16:9 보다 6~8 deg 더 준다. 지면을
    # 얼마나 아래까지 보는지가 SVM 의 전부라 종횡비가 FOV 만큼 중요하다.
    "svm_v2": {
        "front": dict(width=1280, height=960, fov=150.0,
                      t=(3.700, 0.000, 0.650), rot=(0.0, 38.0, 0.0)),
        "left": dict(width=1280, height=960, fov=150.0,
                     t=(2.400, 1.000, 1.000), rot=(0.0, 55.0, 90.0)),
        "right": dict(width=1280, height=960, fov=150.0,
                      t=(2.400, -1.000, 1.000), rot=(0.0, 55.0, -90.0)),
        "rear": dict(width=1280, height=960, fov=150.0,
                     t=(-0.750, 0.000, 0.950), rot=(0.0, 42.0, 180.0)),
    },
    # rosbridge 대역폭이 모자랄 때. 화각·자세는 같고 해상도만 낮춘다.
    "svm_v2_light": {
        "front": dict(width=640, height=480, fov=150.0,
                      t=(3.700, 0.000, 0.650), rot=(0.0, 38.0, 0.0)),
        "left": dict(width=640, height=480, fov=150.0,
                     t=(2.400, 1.000, 1.000), rot=(0.0, 55.0, 90.0)),
        "right": dict(width=640, height=480, fov=150.0,
                      t=(2.400, -1.000, 1.000), rot=(0.0, 55.0, -90.0)),
        "rear": dict(width=640, height=480, fov=150.0,
                     t=(-0.750, 0.000, 0.950), rot=(0.0, 42.0, 180.0)),
    },
    # [사용자 제안] 후방을 양쪽 상단 코너 2대 + 번호판 fisheye 로 나눈 6대 구성.
    # 후방 코너: 테일게이트 상단 모서리 (루프라인 근처). 뒤-바깥 대각을 본다.
    # 번호판: 실제 AVM 후방 카메라 위치. 낮고 가파르게 숙여 근접 지면을 본다.
    "svm_v3_rear6": {
        "front": dict(width=1280, height=960, fov=150.0,
                      t=(3.700, 0.000, 0.650), rot=(0.0, 38.0, 0.0)),
        "left": dict(width=1280, height=960, fov=150.0,
                     t=(2.400, 1.000, 1.000), rot=(0.0, 55.0, 90.0)),
        "right": dict(width=1280, height=960, fov=150.0,
                      t=(2.400, -1.000, 1.000), rot=(0.0, 55.0, -90.0)),
        "rear_left": dict(width=1280, height=960, fov=150.0,
                          t=(-0.700, 0.900, 1.450), rot=(0.0, 40.0, 135.0)),
        "rear_right": dict(width=1280, height=960, fov=150.0,
                           t=(-0.700, -0.900, 1.450), rot=(0.0, 40.0, -135.0)),
        "rear_plate": dict(width=1280, height=960, fov=150.0,
                           t=(-0.790, 0.000, 0.550), rot=(0.0, 50.0, 180.0)),
    },
    # 위에서 번호판 fisheye 만 뺀 5대. 코너 2대가 정후방을 대신할 수 있는지 본다.
    "svm_v3_rear5": {
        "front": dict(width=1280, height=960, fov=150.0,
                      t=(3.700, 0.000, 0.650), rot=(0.0, 38.0, 0.0)),
        "left": dict(width=1280, height=960, fov=150.0,
                     t=(2.400, 1.000, 1.000), rot=(0.0, 55.0, 90.0)),
        "right": dict(width=1280, height=960, fov=150.0,
                      t=(2.400, -1.000, 1.000), rot=(0.0, 55.0, -90.0)),
        "rear_left": dict(width=1280, height=960, fov=150.0,
                          t=(-0.700, 0.900, 1.450), rot=(0.0, 40.0, 135.0)),
        "rear_right": dict(width=1280, height=960, fov=150.0,
                           t=(-0.700, -0.900, 1.450), rot=(0.0, 40.0, -135.0)),
    },
    # 기존 v1 을 유지한 채 후방만 보강하는 최소 변경안 (전/좌/우는 손대지 않는다).
    "v1_plus_rear3": {
        "front": dict(width=1280, height=720, fov=90.0,
                      t=(1.900, 0.000, 1.200), rot=(0.0, 2.0, 0.0)),
        "left": dict(width=640, height=480, fov=130.0,
                     t=(1.150, 0.650, 1.200), rot=(0.0, 10.0, 70.0)),
        "right": dict(width=640, height=480, fov=130.0,
                      t=(1.150, -0.650, 1.200), rot=(0.0, 10.0, -70.0)),
        "rear_left": dict(width=1280, height=960, fov=150.0,
                          t=(-0.700, 0.900, 1.450), rot=(0.0, 40.0, 135.0)),
        "rear_right": dict(width=1280, height=960, fov=150.0,
                           t=(-0.700, -0.900, 1.450), rot=(0.0, 40.0, -135.0)),
        "rear_plate": dict(width=1280, height=960, fov=150.0,
                           t=(-0.790, 0.000, 0.550), rot=(0.0, 50.0, 180.0)),
    },
    # MORAI UI 가 FOV 를 130 deg 로 제한하는 경우의 대안.
    # 화각이 좁아진 만큼 pitch 를 더 눕히고 카메라를 낮춘다.
    "svm_v2_fov130": {
        "front": dict(width=1280, height=960, fov=130.0,
                      t=(3.700, 0.000, 0.600), rot=(0.0, 48.0, 0.0)),
        # 탐색 결과 FOV 130 은 pitch 65 한 점에서만 근거리 100% 다 (--sweep).
        "left": dict(width=1280, height=960, fov=130.0,
                     t=(2.400, 1.000, 0.950), rot=(0.0, 65.0, 90.0)),
        "right": dict(width=1280, height=960, fov=130.0,
                      t=(2.400, -1.000, 0.950), rot=(0.0, 65.0, -90.0)),
        "rear": dict(width=1280, height=960, fov=130.0,
                     t=(-0.750, 0.000, 0.900), rot=(0.0, 52.0, 180.0)),
    },
}

# 표시 순서. 프리셋에 없는 이름은 건너뛰고, 여기 없는 이름은 뒤에 붙는다.
VIEW_ORDER = (
    "front", "left", "right", "rear",
    "rear_left", "rear_right", "rear_plate",
)


def ordered_views(preset_or_cameras) -> list:
    known = [n for n in VIEW_ORDER if n in preset_or_cameras]
    return known + [n for n in preset_or_cameras if n not in VIEW_ORDER]


def make_camera(name: str, spec: dict, sensor_id: int) -> CameraCalibration:
    yaw = ((float(spec["rot"][2]) + 180.0) % 360.0) - 180.0
    return CameraCalibration(
        name=name,
        sensor_id=sensor_id,
        width=int(spec["width"]),
        height=int(spec["height"]),
        horizontal_fov_deg=float(spec["fov"]),
        translation_m=tuple(float(v) for v in spec["t"]),
        rotation_deg=(float(spec["rot"][0]), float(spec["rot"][1]), yaw),
        image_topic="",
        drivable_topic="",
        road_marking_topic="",
        road_marking_confidence_topic="",
        lane_topic="",
        ground_plane=GROUND,
    )


def cameras_from_preset(preset: dict) -> dict:
    return {
        name: make_camera(name, preset[name], index + 1)
        for index, name in enumerate(ordered_views(preset))
    }


def cameras_from_sensor_set(path: Path) -> dict:
    document = json.loads(Path(path).read_text(encoding="utf-8"))
    names = {1: "front", 2: "left", 3: "right", 4: "rear"}
    cameras = {}
    for sensor in document.get("cameraList", []):
        sensor_id = int(sensor["m_SensorUniqueID"])
        cc = sensor["cc"]
        spec = dict(
            width=cc["cameraResWidth"],
            height=cc["cameraResHeight"],
            fov=cc["cameraFOV"],
            t=tuple(float(sensor["pos"][k]) for k in ("x", "y", "z")),
            rot=tuple(float(sensor["rot"][k]) for k in ("roll", "pitch", "yaw")),
        )
        cameras[names.get(sensor_id, f"cam{sensor_id}")] = make_camera(
            names.get(sensor_id, f"cam{sensor_id}"), spec, sensor_id
        )
    return cameras


def vertical_half_fov_deg(camera: CameraCalibration) -> float:
    half = math.radians(0.5 * camera.horizontal_fov_deg)
    return math.degrees(math.atan(math.tan(half) * camera.height / camera.width))


def evaluate(cameras: dict, grid: BevGridSpec = SVM_GRID, max_range_m: float = 15.0):
    """뷰별 coverage 와 지면 샘플링 밀도를 계산한다."""
    models = {
        name: build_homography(camera, grid, max_range_m, 0.10)
        for name, camera in cameras.items()
    }
    rows, columns = np.indices(grid.shape, dtype=float)
    x = grid.x_max_m - rows * grid.resolution_m
    y = grid.y_max_m - columns * grid.resolution_m
    radius = np.hypot(x, y)

    # 차체가 깔고 앉은 셀은 어차피 볼 수 없다. 평가에서 뺀다.
    body = (x >= REAR_BUMPER_X) & (x <= FRONT_BUMPER_X) & (np.abs(y) <= HALF_WIDTH)

    coverage = {name: model.coverage > 0 for name, model in models.items()}
    union = np.logical_or.reduce(list(coverage.values()))
    count = np.sum([c.astype(int) for c in coverage.values()], axis=0)

    # 지면 샘플링 밀도 -> cm/px (선 하나가 몇 픽셀에 걸리는지의 척도)
    density = np.zeros(grid.shape)
    for model in models.values():
        density = np.maximum(density, model.ground_sampling_density)
    with np.errstate(divide="ignore", invalid="ignore"):
        cm_per_px = np.where(density > 0, 100.0 / np.sqrt(density), np.inf)

    return dict(
        grid=grid, models=models, coverage=coverage, union=union, count=count,
        x=x, y=y, radius=radius, body=body, cm_per_px=cm_per_px,
    )


def body_boundary_distance(azimuth_deg: float) -> float:
    """base_link 에서 그 방위로 나갔을 때 차체 밖으로 나가는 거리.

    차체가 깔고 앉은 지면은 어떤 카메라로도 볼 수 없다. 따라서 "사각"은
    최근접 가시거리 그 자체가 아니라 **범퍼 밖으로 얼마나 더 나가야 보이는가**로
    재야 한다. 이걸 안 빼면 정면 방위가 항상 사각으로 잡힌다.
    """
    dx = math.cos(math.radians(azimuth_deg))
    dy = math.sin(math.radians(azimuth_deg))
    candidates = []
    if abs(dx) > 1e-9:
        bound = FRONT_BUMPER_X if dx > 0 else REAR_BUMPER_X
        distance = bound / dx
        if distance > 0 and abs(dy * distance) <= HALF_WIDTH + 1e-9:
            candidates.append(distance)
    if abs(dy) > 1e-9:
        distance = (HALF_WIDTH if dy > 0 else -HALF_WIDTH) / dy
        if distance > 0 and REAR_BUMPER_X - 1e-9 <= dx * distance <= FRONT_BUMPER_X + 1e-9:
            candidates.append(distance)
    return max(candidates) if candidates else 0.0


def nearest_visible(cameras: dict, max_range_m: float = 20.0, step: float = 0.02):
    """방위별 최근접 가시 지면 거리."""
    probe = np.arange(0.1, max_range_m, step)
    out = {}
    for azimuth in range(0, 360, 10):
        direction = np.asarray(
            [math.cos(math.radians(azimuth)), math.sin(math.radians(azimuth))]
        )
        samples = probe.reshape(-1, 1) * direction.reshape(1, 2)
        xyz = np.column_stack([samples, np.full(len(samples), GROUND.z_at_origin_m)])
        best, owner = None, "—"
        for name, camera in cameras.items():
            optical = camera.ground_to_optical(xyz)
            ahead = optical[:, 2] > 0.10
            pixels = np.full((len(samples), 2), np.nan)
            homogeneous = (camera.intrinsic @ optical.T).T
            pixels[ahead] = homogeneous[ahead, :2] / homogeneous[ahead, 2:3]
            visible = (
                ahead
                & (pixels[:, 0] >= 0) & (pixels[:, 0] <= camera.width - 1)
                & (pixels[:, 1] >= 0) & (pixels[:, 1] <= camera.height - 1)
            )
            if visible.any():
                candidate = float(probe[np.argmax(visible)])
                if best is None or candidate < best:
                    best, owner = candidate, name
        out[azimuth] = (best, owner)
    return out


def report(title: str, cameras: dict, verbose: bool = True) -> dict:
    result = evaluate(cameras)
    grid, union, count = result["grid"], result["union"], result["count"]
    radius, body, cm_per_px = result["radius"], result["body"], result["cm_per_px"]
    outside = ~body

    print("=" * 78)
    print(title)
    print("=" * 78)
    print(f"{'view':<11}{'FOV':>6}{'vFOV/2':>8}{'해상도':>12}{'pos(x,y,z)':>24}"
          f"{'pitch':>7}{'yaw':>7}{'cover':>8}")
    print("-" * 78)
    for name in ordered_views(cameras):
        camera = cameras[name]
        share = result["coverage"][name][outside].mean() * 100.0
        pose = "({:.2f},{:.2f},{:.2f})".format(*camera.translation_m)
        print(f"{name:<11}{camera.horizontal_fov_deg:>6.0f}"
              f"{vertical_half_fov_deg(camera):>8.1f}"
              f"{f'{camera.width}x{camera.height}':>12}{pose:>24}"
              f"{camera.rotation_deg[1]:>7.1f}{camera.rotation_deg[2]:>7.1f}"
              f"{share:>7.1f}%")
    print("-" * 78)

    rings = []
    for low, high in ((0, 1), (1, 2), (2, 3), (3, 5), (5, 8)):
        mask = (radius >= low) & (radius < high) & outside
        if mask.any():
            rings.append((low, high, union[mask].mean() * 100.0))
    print("반경 링별 가시율 (차체 footprint 제외):")
    for low, high, value in rings:
        bar = "#" * int(round(value / 4))
        print(f"  r=[{low},{high}) m   {value:5.1f}%  {bar}")

    near = (radius < 3.0) & outside
    print(f"\n  근거리(r<3 m) 가시율        {union[near].mean()*100:5.1f}%")
    print(f"  전체(격자, 차체 제외)        {union[outside].mean()*100:5.1f}%")
    print(f"  2대 이상 중복(스티칭 여유)   {(count>=2)[outside].mean()*100:5.1f}%")

    visible_near = near & union
    if visible_near.any():
        values = cm_per_px[visible_near]
        values = values[np.isfinite(values)]
        if values.size:
            print(f"  근거리 지면 해상도 중앙값    {np.median(values):5.1f} cm/px"
                  f"   (p90 {np.percentile(values, 90):.1f})")
            print(f"    -> 15 cm 주차선이 중앙값 기준 약 "
                  f"{15.0/np.median(values):.1f} px 폭")

    nearest = nearest_visible(cameras)
    gaps = {}
    for azimuth, (distance, owner) in nearest.items():
        edge = body_boundary_distance(azimuth)
        gaps[azimuth] = (
            None if distance is None else max(0.0, distance - edge), owner, edge
        )
    blind = [a for a, (g, _, _) in gaps.items() if g is None or g > 0.5]
    worst = max((g for g, _, _ in gaps.values() if g is not None), default=None)
    print(f"\n  범퍼 밖 사각 폭 최악          "
          f"{f'{worst:.2f} m' if worst is not None else 'none'}")
    print(f"  범퍼에서 0.5 m 넘게 안 보이는 방위: {blind if blind else '없음'}")

    if verbose:
        print("\n  방위(deg) -> 범퍼 밖 사각 폭(m) / 담당 카메라  [0.00 = 범퍼 바로 옆부터 보임]")
        line = "   "
        for azimuth in range(0, 360, 30):
            gap, owner, _ = gaps[azimuth]
            text = f"{gap:.2f}" if gap is not None else "—"
            line += f"{azimuth:>4}:{text:>6}({owner[:1]})"
            if azimuth % 90 == 60:
                print(line)
                line = "   "
        if line.strip():
            print(line)

        print("\n  대표 지점 지면 해상도 (cm/px, 작을수록 선이 또렷하다)")
        probes = [("좌 1 m", 0.0, 2.0), ("좌 3 m", 0.0, 4.0),
                  ("후 1 m", REAR_BUMPER_X - 1.0, 0.0),
                  ("후 3 m", REAR_BUMPER_X - 3.0, 0.0),
                  ("전 1 m", FRONT_BUMPER_X + 1.0, 0.0),
                  ("후좌 대각", REAR_BUMPER_X - 1.5, 2.0)]
        for label, px, py in probes:
            row = grid.metric_to_pixel(np.asarray([[px, py]]))[0]
            column, line_index = int(round(row[0])), int(round(row[1]))
            if not (0 <= line_index < grid.height_px and 0 <= column < grid.width_px):
                continue
            value = cm_per_px[line_index, column]
            seen = union[line_index, column]
            text = f"{value:5.2f} cm/px  (주차선 {15.0/value:4.1f} px)" if (
                seen and np.isfinite(value)) else "안 보임"
            print(f"    {label:<10} ({px:5.2f},{py:5.2f})  {text}")
    print()
    return result


def render(result: dict, cameras: dict, path: Path) -> None:
    import cv2

    grid = result["grid"]
    canvas = np.zeros((grid.height_px, grid.width_px, 3), np.uint8)
    colours = {
        "front": (60, 220, 60), "left": (220, 160, 40),
        "right": (60, 160, 240), "rear": (200, 80, 220),
        "rear_left": (90, 220, 220), "rear_right": (150, 120, 255),
        "rear_plate": (255, 210, 120),
    }
    for name, mask in result["coverage"].items():
        colour = np.asarray(colours.get(name, (180, 180, 180)), np.uint8)
        canvas[mask] = np.maximum(canvas[mask], colour)
    canvas[result["count"] >= 2] = (255, 255, 255)

    def to_px(x, y):
        return (int(round((grid.y_max_m - y) / grid.resolution_m)),
                int(round((grid.x_max_m - x) / grid.resolution_m)))

    # 차체
    corners = [(FRONT_BUMPER_X, HALF_WIDTH), (FRONT_BUMPER_X, -HALF_WIDTH),
               (REAR_BUMPER_X, -HALF_WIDTH), (REAR_BUMPER_X, HALF_WIDTH)]
    cv2.polylines(canvas, [np.asarray([to_px(*c) for c in corners], np.int32)],
                  True, (0, 0, 255), 2)
    # 주차면 한 칸 (2.5 x 5.0 m) 참고선 — 차 바로 오른쪽
    slot = [(-0.5, -1.2), (4.5, -1.2), (4.5, -3.7), (-0.5, -3.7)]
    cv2.polylines(canvas, [np.asarray([to_px(*c) for c in slot], np.int32)],
                  True, (0, 200, 255), 1)
    for name, camera in cameras.items():
        cv2.circle(canvas, to_px(camera.translation_m[0], camera.translation_m[1]),
                   4, (255, 255, 255), -1)
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), canvas)
    print(f"가시영역 이미지: {path}")
    print("  초록 front · 주황 left · 파랑 right · 자홍 rear · 흰색 2대 이상")
    print("  빨간 사각 = 차체, 노란 사각 = 주차면 한 칸(2.5 x 5.0 m) 참고선\n")


def max_ground_range(camera: CameraCalibration) -> float:
    """광축 방위에서 지면을 볼 수 있는 최대 거리.

    위쪽 광선이 지평선 위를 향하면 무한대다. pitch 를 눕힐수록 근거리는 좋아지지만
    이 값이 줄어든다 — SVM 설계의 실제 trade-off 는 여기에 있다.
    """
    height = camera.translation_m[2] - GROUND.z_at_origin_m
    top = camera.rotation_deg[1] - vertical_half_fov_deg(camera)
    if top <= 0.5:
        return float("inf")
    return height / math.tan(math.radians(top))


def sweep() -> None:
    """측면 카메라 FOV x pitch. 근거리 가시율과 최대 지면거리를 같이 본다."""
    print("=" * 78)
    print("탐색: 측면 카메라 FOV x pitch (front/rear 는 svm_v2 고정)")
    print("측면이 SVM 의 병목이다 — 주차선을 실제로 보는 것이 측면 카메라다.")
    print("=" * 78)
    base = PRESETS["svm_v2"]

    print("\n[1] 근거리(r<3 m, 차체 제외) 가시율 — 높을수록 좋다")
    fovs = (120, 130, 140, 150, 160)
    header = f"{'pitch\\FOV':>10}" + "".join(f"{f:>9}" for f in fovs)
    print(header)
    for pitch in (40, 45, 50, 55, 60, 65, 70):
        row = f"{pitch:>10}"
        for fov in fovs:
            preset = copy.deepcopy(base)
            for side in ("left", "right"):
                preset[side]["fov"] = float(fov)
                preset[side]["rot"] = (0.0, float(pitch), preset[side]["rot"][2])
            result = evaluate(cameras_from_preset(preset))
            near = (result["radius"] < 3.0) & (~result["body"])
            row += f"{result['union'][near].mean()*100:>8.1f}%"
        print(row)

    print("\n[2] 측면 카메라의 최대 지면 거리 (m) — 낮으면 접근 구간이 안 보인다")
    print(header)
    for pitch in (40, 45, 50, 55, 60, 65, 70):
        row = f"{pitch:>10}"
        for fov in fovs:
            spec = dict(base["left"])
            spec["fov"] = float(fov)
            spec["rot"] = (0.0, float(pitch), 90.0)
            reach = max_ground_range(make_camera("left", spec, 2))
            row += ("     inf" if reach == float("inf") else f"{reach:>8.1f}") + " "
        print(row)

    print("""
  읽는 법 — [1] 100% 이고 [2] 10 m 이상인 칸만 SVM 에 쓸 수 있다.
    FOV 150~160  pitch 40~70 전 구간에서 둘 다 만족. 여유가 크다 -> 권장
    FOV 140      pitch 45 부터 [1] 100%, [2] 는 70 에서도 13.1 m. 쓸 수 있다
    FOV 130      pitch 65 **한 점에서만** 성립 ([1] 100%, [2] 11.2 m).
                 pitch 60 이면 [1] 이 99.8%, pitch 70 이면 [2] 가 6.4 m 로 무너진다.
                 여유가 없으므로 차량 제원이나 지면 높이가 조금만 틀려도 깨진다
    FOV 120      어떤 pitch 로도 [1] 이 100% 가 안 된다 — SVM 으로 못 쓴다
  결론: **화각이 pitch 보다 훨씬 중요하다.** MORAI UI 가 허용하는 최대 화각을 먼저
  확인하고, 그 값이 140 미만이면 SVM 을 포기하고 근거리 보조 카메라를 추가하는 쪽이 낫다.
""")


def export_sensor_set(preset: dict, template: Path, output: Path) -> None:
    document = json.loads(Path(template).read_text(encoding="utf-8"))
    names = {1: "front", 2: "left", 3: "right", 4: "rear"}
    for sensor in document.get("cameraList", []):
        sensor_id = int(sensor["m_SensorUniqueID"])
        name = names.get(sensor_id)
        if name not in preset:
            continue
        spec = preset[name]
        sensor["pos"] = {k: "{:.3f}".format(v)
                         for k, v in zip(("x", "y", "z"), spec["t"])}
        sensor["rot"] = {k: "{:.3f}".format(v)
                         for k, v in zip(("roll", "pitch", "yaw"), spec["rot"])}
        cc = sensor["cc"]
        cc["cameraResWidth"] = int(spec["width"])
        cc["cameraResHeight"] = int(spec["height"])
        cc["cameraFOV"] = float(spec["fov"])
        cc["principalPoint"] = {
            "x": spec["width"] / 2.0, "y": spec["height"] / 2.0,
            "_x": "{:.3f}".format(spec["width"] / 2.0),
            "_y": "{:.3f}".format(spec["height"] / 2.0),
        }
        # paramType 0 = FOV 기준. focalLengthpixel 은 MORAI 가 다시 계산하므로
        # 참고값으로만 채운다 (BEV 코드는 이 값을 쓰지 않는다).
        cc["focalLengthpixel"] = round(
            (spec["width"] / 2.0) / math.tan(math.radians(spec["fov"] / 2.0)), 3
        )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(document, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    import hashlib
    digest = hashlib.sha256(output.read_bytes()).hexdigest()
    print(f"센서셋 내보냄: {output}")
    print(f"  sha256 {digest}")
    print("  drivable_bev/config/cameras_*.yaml 의 source_sha256 도 같이 갱신할 것.\n")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--preset", choices=sorted(PRESETS))
    parser.add_argument("--sensor-set", type=Path)
    parser.add_argument("--compare", type=Path)
    parser.add_argument("--sweep", action="store_true")
    parser.add_argument("--image", type=Path)
    parser.add_argument("--export-sensor-set", type=Path)
    parser.add_argument("--template", type=Path,
                        default=REPO_ROOT / "config" / "VIP3_sensor_set_v1_ros.json")
    arguments = parser.parse_args()

    if arguments.sweep:
        sweep()
        if not (arguments.preset or arguments.sensor_set):
            return

    target = None
    if arguments.sensor_set:
        target = (str(arguments.sensor_set), cameras_from_sensor_set(arguments.sensor_set))
    elif arguments.preset:
        target = (f"preset: {arguments.preset}",
                  cameras_from_preset(PRESETS[arguments.preset]))
    else:
        target = ("preset: current (임시 센서셋)", cameras_from_preset(PRESETS["current"]))

    if arguments.compare:
        report(f"[비교 기준] {arguments.compare}",
               cameras_from_sensor_set(arguments.compare), verbose=False)

    result = report(f"[대상] {target[0]}", target[1])

    if arguments.image:
        render(result, target[1], arguments.image)
    if arguments.export_sensor_set:
        if not arguments.preset:
            parser.error("--export-sensor-set 은 --preset 과 함께 쓴다")
        export_sensor_set(PRESETS[arguments.preset], arguments.template,
                          arguments.export_sensor_set)


if __name__ == "__main__":
    main()
