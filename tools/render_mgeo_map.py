#!/usr/bin/env python3
"""MGeo(KATRI) 맵 세트를 위에서 내려다본 PNG 로 렌더링한다.

RViz 를 띄우지 않고 맵 안에 무엇이 들어 있는지 눈으로 확인하기 위한 오프라인 도구다.
`vip3_hd_map` 런타임 노드와 같은 JSON 을 읽지만 ROS 의존성이 없다.

사용법:
    python3 tools/render_mgeo_map.py --mgeo "../data/KATRI 맵 데이터 자료" \
        --out /tmp/katri.png --scale 4
    # 관심 영역만 (중심 x,y 와 반경, 단위 m)
    python3 tools/render_mgeo_map.py --mgeo ... --center -60 -190 --radius 80 --scale 20
"""

from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path

import cv2
import numpy as np

# NGII ngii_model2 노면선표시 코드. MGeo lane_boundary/lane_marking 의 lane_type.
LANE_TYPE_KO = {
    501: "중앙선",
    502: "유턴구역선",
    503: "차선",
    504: "버스전용차선",
    505: "길가장자리구역선",
    506: "진로변경제한선",
    515: "좌회전유도차선",
    525: "유도선",
    530: "정지선",
    531: "안전지대",
    535: "미확정(535)",
    599: "기타",
}

# NGII 노면기타표시 sub_type.
SURFACE_SUB_TYPE_KO = {
    "5371": "직진",
    "5372": "좌회전",
    "5373": "우회전",
    "5374": "직진+좌회전",
    "5375": "직진+우회전",
    "5376": "좌우회전",
    "5377": "직진+좌우회전",
    "5379": "유턴",
    "5381": "횡단보도예고",
    "5382": "자전거횡단도예고",
    "5431": "오르막경사면",
    "5432": "도로中",
}

LANE_TYPE_BGR = {
    501: (60, 230, 255),
    503: (255, 255, 255),
    505: (200, 200, 200),
    506: (180, 140, 255),
    515: (0, 200, 255),
    525: (120, 200, 120),
    530: (80, 80, 255),
    531: (255, 200, 120),
    535: (255, 120, 255),
    599: (150, 150, 150),
    504: (255, 180, 80),
    502: (200, 255, 200),
}


def _load(directory: Path, name: str):
    path = directory / name
    if not path.is_file():
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def _points(record):
    value = record.get("points")
    if not value:
        return None
    array = np.asarray(value, dtype=float)
    if array.ndim != 2 or array.shape[1] < 2:
        return None
    return array[:, :2]


def _lane_type(record) -> int:
    value = record.get("lane_type")
    if isinstance(value, list):
        value = value[0] if value else None
    try:
        return int(value)
    except (TypeError, ValueError):
        return -1


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mgeo", required=True, type=Path)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--scale", type=float, default=4.0, help="픽셀/미터")
    parser.add_argument("--center", nargs=2, type=float, default=None, metavar=("X", "Y"))
    parser.add_argument("--radius", type=float, default=None)
    parser.add_argument("--margin", type=float, default=20.0)
    parser.add_argument("--no-link", action="store_true")
    arguments = parser.parse_args()

    directory = arguments.mgeo
    links = _load(directory, "link_set.json") or []
    boundaries = _load(directory, "lane_boundary_set.json") or []
    markings = _load(directory, "lane_marking_set.json") or []
    surfaces = _load(directory, "surface_marking_set.json") or []
    crosswalks = _load(directory, "crosswalk_set.json") or []

    lines = markings if markings else boundaries
    source = "lane_marking_set.json" if markings else "lane_boundary_set.json"

    everything = []
    for group in (links, lines, surfaces, crosswalks):
        for record in group:
            points = _points(record)
            if points is not None:
                everything.append(points)
    if not everything:
        raise SystemExit(f"{directory} 에서 기하 데이터를 찾지 못했다")
    stacked = np.vstack(everything)

    if arguments.center is not None and arguments.radius is not None:
        cx, cy = arguments.center
        x_min, x_max = cx - arguments.radius, cx + arguments.radius
        y_min, y_max = cy - arguments.radius, cy + arguments.radius
    else:
        x_min, y_min = stacked.min(axis=0) - arguments.margin
        x_max, y_max = stacked.max(axis=0) + arguments.margin

    width = int(round((x_max - x_min) * arguments.scale))
    height = int(round((y_max - y_min) * arguments.scale))
    if width <= 0 or height <= 0 or width * height > 200_000_000:
        raise SystemExit(f"렌더 크기가 비정상이다: {width}x{height}. --scale 을 낮춰라")
    canvas = np.zeros((height, width, 3), np.uint8)
    canvas[:] = (24, 24, 24)

    def to_px(points):
        columns = (points[:, 0] - x_min) * arguments.scale
        rows = (y_max - points[:, 1]) * arguments.scale
        return np.stack([columns, rows], axis=1).round().astype(np.int32)

    if not arguments.no_link:
        for record in links:
            points = _points(record)
            if points is None:
                continue
            cv2.polylines(canvas, [to_px(points)], False, (70, 70, 70), 1, cv2.LINE_AA)

    counts = Counter()
    for record in lines:
        points = _points(record)
        if points is None:
            continue
        lane_type = _lane_type(record)
        counts[lane_type] += 1
        color = LANE_TYPE_BGR.get(lane_type, (0, 255, 0))
        cv2.polylines(canvas, [to_px(points)], False, color, 1, cv2.LINE_AA)

    for record in crosswalks:
        points = _points(record)
        if points is None:
            continue
        cv2.polylines(canvas, [to_px(points)], True, (120, 255, 255), 1, cv2.LINE_AA)

    surface_counts = Counter()
    for record in surfaces:
        points = _points(record)
        if points is None:
            continue
        surface_counts[str(record.get("sub_type"))] += 1
        cv2.polylines(canvas, [to_px(points)], True, (100, 160, 255), 1, cv2.LINE_AA)

    arguments.out.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(arguments.out), canvas)

    print(f"맵: {directory}")
    print(f"선 소스: {source} ({len(lines)} 개)")
    print(f"범위: x[{x_min:.1f},{x_max:.1f}] y[{y_min:.1f},{y_max:.1f}]  -> {width}x{height} px")
    print(f"출력: {arguments.out}")
    print()
    print("lane_type 분포:")
    for lane_type, count in sorted(counts.items(), key=lambda kv: -kv[1]):
        print(f"  {lane_type:>4}  {LANE_TYPE_KO.get(lane_type, '미상'):<14} {count}")
    print()
    print("surface_marking sub_type 분포:")
    for sub_type, count in sorted(surface_counts.items(), key=lambda kv: -kv[1]):
        print(f"  {sub_type:>6}  {SURFACE_SUB_TYPE_KO.get(sub_type, '미상'):<14} {count}")


if __name__ == "__main__":
    main()
