#!/usr/bin/env python3
"""MORAI 센서셋의 지면 BEV 가시영역을 계산한다.

주차는 차 주변 근거리가 전부다. 이 도구는 센서셋 JSON만 있으면 시뮬레이터를
켜지 않고도 "차 주변 어디가 안 보이는지"를 숫자와 그림으로 답한다. 카메라 후보를
바꿔가며 돌려서 사각을 줄이는 것이 목적이다.

계산 모델은 `drivable_bev` 런타임과 같다.
  - 지면은 base_link 기준 z = z0 평면 (기본 -0.35 m, 뒷바퀴축 중심이 원점)
  - 내부 파라미터는 해상도와 수평 FOV에서 유도 (센서셋의 focalLengthpixel 은 쓰지 않는다)
  - 외부 파라미터는 R = Rz(yaw) @ Ry(pitch) @ Rx(roll), mount->optical 은 REP-103 기준

사용법:
    python3 tools/analyze_sensor_set_coverage.py \
        --sensor-set "../data/VIP3_sensor_set_v1.json" \
        --image /tmp/coverage.png

렌즈 왜곡(fisheye)은 모델링하지 않는다. 핀홀 FOV 기준이므로 fisheye 후보를 볼 때는
실제 화각보다 보수적인 값이 나온다는 점을 감안한다.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

MOUNT_TO_OPTICAL = np.asarray([[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]])

# 색은 시각화 전용. 5번째 카메라부터는 자동 생성한다.
DEFAULT_COLORS = [
    (60, 220, 60),
    (220, 160, 40),
    (60, 160, 240),
    (200, 80, 220),
    (80, 220, 220),
    (150, 150, 255),
]


def _rotation(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    roll, pitch, yaw = np.radians([roll_deg, pitch_deg, yaw_deg])
    rx = np.asarray([[1, 0, 0], [0, math.cos(roll), -math.sin(roll)], [0, math.sin(roll), math.cos(roll)]])
    ry = np.asarray([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0], [-math.sin(pitch), 0, math.cos(pitch)]])
    rz = np.asarray([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
    return rz @ ry @ rx


class Camera:
    def __init__(self, name, width, height, fov_deg, translation, rotation_deg):
        self.name = name
        self.width = int(width)
        self.height = int(height)
        self.fov_deg = float(fov_deg)
        self.translation = np.asarray(translation, dtype=float)
        self.rotation_deg = np.asarray(rotation_deg, dtype=float)

    @property
    def focal_px(self) -> float:
        return (0.5 * self.width) / math.tan(math.radians(0.5 * self.fov_deg))

    @property
    def intrinsic(self) -> np.ndarray:
        f = self.focal_px
        return np.asarray([[f, 0, 0.5 * self.width], [0, f, 0.5 * self.height], [0, 0, 1]])

    @property
    def optical_from_body(self):
        rotation = MOUNT_TO_OPTICAL @ _rotation(*self.rotation_deg).T
        return rotation, -rotation @ self.translation

    def sees(self, points_xy: np.ndarray, z0: float, min_depth: float) -> np.ndarray:
        points = np.asarray(points_xy, dtype=float).reshape(-1, 2)
        xyz = np.column_stack([points, np.full(len(points), z0)])
        rotation, translation = self.optical_from_body
        optical = (rotation @ xyz.T + translation.reshape(3, 1)).T
        ahead = optical[:, 2] > min_depth
        pixels = np.full((len(points), 2), np.nan)
        homogeneous = (self.intrinsic @ optical.T).T
        pixels[ahead] = homogeneous[ahead, :2] / homogeneous[ahead, 2:3]
        return (
            ahead
            & (pixels[:, 0] >= 0.0)
            & (pixels[:, 0] <= self.width - 1.0)
            & (pixels[:, 1] >= 0.0)
            & (pixels[:, 1] <= self.height - 1.0)
        )


def load_cameras(path: Path, names: dict[int, str]) -> list[Camera]:
    document = json.loads(Path(path).read_text(encoding="utf-8"))
    cameras = []
    for sensor in document.get("cameraList", []):
        sensor_id = int(sensor["m_SensorUniqueID"])
        cc = sensor["cc"]
        yaw = ((float(sensor["rot"]["yaw"]) + 180.0) % 360.0) - 180.0
        cameras.append(
            Camera(
                names.get(sensor_id, f"cam{sensor_id}"),
                cc["cameraResWidth"],
                cc["cameraResHeight"],
                cc["cameraFOV"],
                [float(sensor["pos"][k]) for k in ("x", "y", "z")],
                [float(sensor["rot"]["roll"]), float(sensor["rot"]["pitch"]), yaw],
            )
        )
    if not cameras:
        raise ValueError(f"{path} 에 카메라가 없다")
    return cameras


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sensor-set", required=True, type=Path)
    parser.add_argument("--x-min", type=float, default=-10.0)
    parser.add_argument("--x-max", type=float, default=10.0)
    parser.add_argument("--y-min", type=float, default=-8.0)
    parser.add_argument("--y-max", type=float, default=8.0)
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--ground-z", type=float, default=-0.35)
    parser.add_argument("--min-depth", type=float, default=0.10)
    parser.add_argument(
        "--pitch-sign",
        type=float,
        default=1.0,
        choices=[1.0, -1.0],
        help="MORAI pitch 부호 규약. 1.0 = 양수 pitch 가 렌즈 아래 (ASMC 3-view 로 검증된 해석), "
        "-1.0 = 반대. 첫 연동에서 지면 격자 오버레이로 확정한 뒤 고정한다.",
    )
    parser.add_argument("--max-range", type=float, default=25.0)
    parser.add_argument("--image", type=Path, default=None, help="가시영역 PNG 경로")
    parser.add_argument(
        "--names",
        default="1=front,2=left,3=right,4=rear",
        help="센서 ID=이름 목록",
    )
    arguments = parser.parse_args()

    names = {}
    for item in filter(None, (part.strip() for part in arguments.names.split(","))):
        key, _, value = item.partition("=")
        names[int(key)] = value
    cameras = load_cameras(arguments.sensor_set, names)
    if arguments.pitch_sign < 0:
        for camera in cameras:
            camera.rotation_deg = camera.rotation_deg * np.asarray([1.0, -1.0, 1.0])

    height = int(round((arguments.x_max - arguments.x_min) / arguments.resolution))
    width = int(round((arguments.y_max - arguments.y_min) / arguments.resolution))
    rows, columns = np.indices((height, width), dtype=float)
    x = arguments.x_max - rows * arguments.resolution
    y = arguments.y_max - columns * arguments.resolution
    points = np.column_stack([x.ravel(), y.ravel()])
    radius = np.hypot(points[:, 0], points[:, 1])
    in_range = radius <= arguments.max_range

    print(f"센서셋: {arguments.sensor_set}")
    print(f"pitch 부호 규약: {'양수 = 아래' if arguments.pitch_sign > 0 else '양수 = 위'}")
    print(
        f"격자: x[{arguments.x_min},{arguments.x_max}] y[{arguments.y_min},{arguments.y_max}] "
        f"res={arguments.resolution} -> {height}x{width} px, 지면 z={arguments.ground_z}"
    )
    print()
    per_view = {}
    union = np.zeros(len(points), dtype=bool)
    print(f"{'view':<10}{'FOV':>7}{'해상도':>12}{'fx(px)':>9}{'yaw':>8}{'pitch':>7}{'커버리지':>10}")
    print("-" * 65)
    for camera in cameras:
        visible = camera.sees(points, arguments.ground_z, arguments.min_depth) & in_range
        per_view[camera.name] = visible
        union |= visible
        print(
            f"{camera.name:<10}{camera.fov_deg:>7.1f}"
            f"{f'{camera.width}x{camera.height}':>12}{camera.focal_px:>9.1f}"
            f"{camera.rotation_deg[2]:>8.1f}{camera.rotation_deg[1]:>7.1f}"
            f"{visible.mean() * 100:>9.2f}%"
        )
    print("-" * 65)
    print(f"{'UNION':<10}{'':>7}{'':>12}{'':>9}{'':>8}{'':>7}{union.mean() * 100:>9.2f}%")

    print()
    print("반경 링별 가시율:")
    for low, high in [(0, 2), (2, 4), (4, 6), (6, 8), (8, 10), (10, 15)]:
        ring = (radius >= low) & (radius < high)
        if not ring.any():
            continue
        detail = "  ".join(f"{n}={per_view[n][ring].mean() * 100:5.1f}%" for n in per_view)
        print(f"  r=[{low:2d},{high:2d})m  union={union[ring].mean() * 100:5.1f}%   {detail}")

    print()
    print("방위별 최근접 가시 지면 거리 (m). '—' 는 max-range 안에서 안 보인다는 뜻이다:")
    probe = np.arange(0.2, arguments.max_range, 0.05)
    blind = []
    for azimuth in range(0, 360, 10):
        direction = np.asarray([math.cos(math.radians(azimuth)), math.sin(math.radians(azimuth))])
        samples = probe.reshape(-1, 1) * direction.reshape(1, 2)
        nearest, owner = None, "—"
        for camera in cameras:
            visible = camera.sees(samples, arguments.ground_z, arguments.min_depth)
            if visible.any():
                candidate = float(probe[np.argmax(visible)])
                if nearest is None or candidate < nearest:
                    nearest, owner = candidate, camera.name
        if nearest is None or nearest > 6.0:
            blind.append(azimuth)
        rendered = f"{nearest:.2f}" if nearest is not None else "—"
        marker = "   <<< 사각" if (nearest is None or nearest > 6.0) else ""
        print(f"  {azimuth:>3}°  {rendered:>6}  ({owner}){marker}")
    print()
    print(f"6 m 안에서 지면을 전혀 못 보는 방위: {blind if blind else '없음'}")

    if arguments.image is not None:
        import cv2

        canvas = np.zeros((height, width, 3), np.uint8)
        counts = np.zeros(len(points), dtype=int)
        for index, camera in enumerate(cameras):
            color = DEFAULT_COLORS[index % len(DEFAULT_COLORS)]
            mask = per_view[camera.name].reshape(height, width)
            canvas[mask] = np.maximum(canvas[mask], np.asarray(color, np.uint8))
            counts += per_view[camera.name].astype(int)
        canvas[(counts >= 2).reshape(height, width)] = (255, 255, 255)
        ego = (int(arguments.y_max / arguments.resolution), int(arguments.x_max / arguments.resolution))
        cv2.circle(canvas, ego, 4, (0, 0, 255), -1)
        arguments.image.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(arguments.image), canvas)
        print()
        print(f"가시영역 이미지: {arguments.image}  (흰색=2대 이상 중복, 빨강=base_link 원점)")


if __name__ == "__main__":
    main()
