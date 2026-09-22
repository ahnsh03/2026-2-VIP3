#!/usr/bin/env python3
"""KATRI MGeo 오프라인 점검 — ROS 없이 돈다.

무엇에 쓰나
  1. 맵을 처음 받은 사람이 "안에 뭐가 들었나"를 한 번에 본다.
  2. **MORAI 가 빠뜨린 parking_space_set.json 이 들어왔는지 1초에 확인한다.**
     ``global_info.mgeo_file_hash`` 매니페스트와 실제 파일을 대조해서,
     '빠졌는데 원본에는 내용이 있었다(missing_nonempty)'를 따로 찍어 준다.
  3. 좌표계/원점/범위가 우리가 문서에 적어 둔 값과 같은지 재현한다.

사용법
    python3 scripts/inspect_katri_mgeo.py --map-dir "$VIP3_DATA/KATRI 맵 데이터 자료"
    python3 scripts/inspect_katri_mgeo.py --check      # 주차 레이어 있으면 exit 0
    python3 scripts/inspect_katri_mgeo.py --ascii      # 전체 맵 ASCII 조감도

의존: python3 표준 라이브러리 + numpy. matplotlib/shapely/ROS 금지.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from vip3_hd_map.mgeo_layers import (  # noqa: E402
    EXPECTED_MISSING_SHA256,
    KatriMGeo,
    lane_type_label,
    points_xyz,
    surface_marking_label,
)


DEFAULT_MAP_DIRNAME = "KATRI 맵 데이터 자료"


def default_map_dir() -> str:
    data_root = os.environ.get("VIP3_DATA", "").strip()
    if data_root:
        return str(Path(data_root) / DEFAULT_MAP_DIRNAME)
    return DEFAULT_MAP_DIRNAME


def rule(title: str) -> None:
    print("\n" + "=" * 78)
    print(title)
    print("=" * 78)


def format_bounds(bounds) -> str:
    if bounds is None:
        return "(비어 있음)"
    return "x[%8.1f, %8.1f] y[%8.1f, %8.1f] z[%6.1f, %6.1f]" % (
        bounds[0], bounds[3], bounds[1], bounds[4], bounds[2], bounds[5],
    )


def print_frame(mgeo: KatriMGeo) -> None:
    rule("1. 좌표계 / 원점")
    info = mgeo.global_info
    frame = mgeo.frame()
    print("  global_coordinate_system : %s" % frame.crs)
    print("  local_origin_in_global   : %s   <- map_xy = utm52n_xy - 이 값" % (
        list(frame.origin_xyz),))
    print("  workspace_origin         : %s   <- 에디터 뷰포트 값. 쓰지 말 것" % (
        info.get("workspace_origin"),))
    print("  maj/min ver              : %s / %s" % (info.get("maj_ver"), info.get("min_ver")))
    print("  traffic_dir / road_type  : %s / %s" % (info.get("traffic_dir"), info.get("road_type")))
    print("  saved_utc_time           : %s" % info.get("saved_utc_time"))
    print("  전체 범위                : %s" % format_bounds(mgeo.bounds()))
    print("  points 는 이미 로컬 미터 좌표다. RViz map frame 에 그대로 찍으면 된다.")


def print_layers(mgeo: KatriMGeo) -> None:
    rule("2. 레이어별 개수와 좌표 범위")
    for name, stat in mgeo.layer_stats().items():
        print("  %-24s %6d  %s" % (name, stat["count"], format_bounds(stat["bounds"])))
    if mgeo.missing_layers:
        print("  [없음] %s" % ", ".join(mgeo.missing_layers))
    if mgeo.missing_optional_layers:
        print("  [선택 레이어 없음] %s" % ", ".join(mgeo.missing_optional_layers))


def print_lane_distribution(mgeo: KatriMGeo) -> None:
    rule("3. lane_boundary 분포")
    lanes = mgeo.lane_boundaries()
    print("  총 %d개" % len(lanes))
    print("  %-8s %6s  %-18s %-22s %s" % ("type", "n", "slug", "한국어", "길이 min/avg/max (m)"))
    counts = mgeo.lane_type_counts()
    for lane_type, count in counts.items():
        lengths = []
        for feature in mgeo.lane_boundaries_of_type(lane_type):
            points = points_xyz(feature)[:, :2]
            if len(points) >= 2:
                lengths.append(float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum()))
        slug, korean = lane_type_label(lane_type)
        span = "-" if not lengths else "%.2f / %.1f / %.1f" % (
            min(lengths), sum(lengths) / len(lengths), max(lengths))
        print("  %-8s %6d  %-18s %-22s %s" % (lane_type, count, slug, korean, span))
    for field in ("lane_shape", "lane_color", "lane_width", "lane_type_def", "lane_sub_type"):
        print("  %-14s %s" % (field, mgeo.value_counts("lane_boundary_set", field)))


def print_surface_markings(mgeo: KatriMGeo) -> None:
    rule("4. surface_marking 분포와 쿼드 치수")
    print("  총 %d개  (type 분포 %s)" % (
        len(mgeo.surface_markings()), mgeo.value_counts("surface_marking_set", "type")))
    print("  %-8s %5s  %-22s %-14s %s" % ("sub_type", "n", "slug", "단변(m) 중앙", "장변(m) 중앙"))
    grouped: dict = {}
    for feature in mgeo.surface_markings():
        from vip3_hd_map.mgeo_layers import as_int
        grouped.setdefault(as_int(feature.get("sub_type"), -1), []).append(feature)
    for sub_type, count in mgeo.surface_marking_sub_type_counts().items():
        short, long = [], []
        for feature in grouped.get(sub_type, ()):
            points = points_xyz(feature)[:, :2]
            if len(points) < 3:
                continue
            edges = np.linalg.norm(np.diff(np.vstack((points, points[:1])), axis=0), axis=1)
            edges = np.sort(edges)
            short.append(float(edges[0]))
            long.append(float(edges[-1]))
        slug, _ = surface_marking_label(sub_type)
        print("  %-8s %5d  %-22s %-14s %s" % (
            sub_type, count, slug,
            "-" if not short else "%.2f" % float(np.median(short)),
            "-" if not long else "%.2f" % float(np.median(long)),
        ))
    print("  전부 0.5~2.3 m x ~5 m. 주차면 규격(2.3~2.5 m x 5.0 m 평행 반복)이 아니라")
    print("  한국 노면 화살표/문자 표시 규격이고, 어느 sub_type 도 한 곳에 뭉쳐 있지 않다.")


def print_manifest(mgeo: KatriMGeo, verify: bool) -> list:
    rule("5. global_info.mgeo_file_hash 매니페스트 대조  ← 이 표가 핵심")
    report = mgeo.manifest_report(verify=verify)
    if not report:
        print("  매니페스트를 읽지 못했다 (global_info.json 확인).")
        return report
    print("  status 뜻:")
    print("    ok               파일 있음 + sha256 일치")
    print("    hash_mismatch    파일은 있는데 내용이 다르다 (누가 편집했다)")
    print("    missing_empty    빠졌지만 원본도 빈 리스트였다 — 받을 게 없다")
    print("    missing_nonempty 빠졌는데 원본에는 내용이 있었다 — MORAI 에 요청해야 한다")
    print()
    print("  %-16s %-36s %s" % ("status", "file", "expected sha256"))
    for row in report:
        print("  %-16s %-36s %s" % (row["status"], row["file"], row["expected_sha256"]))
        if row["status"] == "hash_mismatch":
            print("  %-16s %-36s actual %s" % ("", "", row["actual_sha256"]))
    missing = [row for row in report if row["status"] == "missing_nonempty"]
    if missing:
        print()
        print("  >>> 원본 export 에 실재했으나 전달되지 않은 파일 %d개:" % len(missing))
        for row in missing:
            print("      %s  sha256 %s" % (row["file"], row["expected_sha256"]))
        print("  >>> MORAI 협력 교원 / 원종훈 교수님께 R_KR_PG_KATRI 원본 MGeo export 를 요청한다.")
        print("      검증 기준을 같이 보내면 왕복이 줄어든다 (위 sha256 이 그대로 나와야 한다).")
    else:
        print()
        print("  >>> 매니페스트의 모든 파일이 있다. 주차 레이어를 바로 쓸 수 있다.")
    return report


def print_parking(mgeo: KatriMGeo) -> None:
    rule("6. 주차 레이어")
    spaces = mgeo.parking_spaces()
    print("  parking_space_set.json   : %s" % (
        "있음 (%d개)" % len(mgeo.layer("parking_space_set"))
        if "parking_space_set" not in mgeo.missing_optional_layers else "없음"))
    manual = mgeo.manual_parking_path
    print("  손으로 적은 fallback     : %s" % (
        "%s (%s)" % (manual, "있음" if manual and manual.is_file() else "없음")
        if manual else "지정 안 됨"))
    print("  실제로 쓰이는 출처       : %s" % mgeo.parking_space_source)
    print("  주차면 개수              : %d" % len(spaces))
    for space in spaces[:10]:
        center = space.get("center_point")
        print("    %-14s center=%s %sx%s angle=%s type=%s" % (
            space["idx"],
            "-" if center is None else "(%.2f, %.2f)" % (center[0], center[1]),
            space["width"], space["length"], space["angle"], space["parking_type"]))
    if not spaces:
        print("  배포된 KATRI MGeo 에는 주차면 기하가 없다. 그때까지 주차 GT 는")
        print("  config/vip3_parking_spaces.json 에 손으로 적는다 (같은 스키마, 같은 코드 경로).")


def print_parking_cluster_heuristic(mgeo: KatriMGeo, cell_m: float = 25.0) -> None:
    rule("7. 주차장 휴리스틱 — 짧은(3~8 m) 세그먼트 밀집 셀 상위 10")
    cells: dict = {}
    for feature in mgeo.lane_boundaries():
        points = points_xyz(feature)[:, :2]
        if len(points) < 2:
            continue
        length = float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())
        if not 3.0 <= length <= 8.0:
            continue
        center = points.mean(axis=0)
        key = (int(np.floor(center[0] / cell_m)), int(np.floor(center[1] / cell_m)))
        from vip3_hd_map.mgeo_layers import as_int
        entry = cells.setdefault(key, {"n": 0, "types": {}})
        entry["n"] += 1
        lane_type = as_int(feature.get("lane_type"), -1)
        entry["types"][lane_type] = entry["types"].get(lane_type, 0) + 1
    ranked = sorted(cells.items(), key=lambda item: -item[1]["n"])[:10]
    print("  %-22s %4s  %s" % ("셀 중심 (m)", "n", "lane_type 구성"))
    for (cell_x, cell_y), entry in ranked:
        print("  %-22s %4d  %s" % (
            "(%.0f, %.0f)" % ((cell_x + 0.5) * cell_m, (cell_y + 0.5) * cell_m),
            entry["n"],
            dict(sorted(entry["types"].items(), key=lambda item: -item[1])),
        ))
    print("  주차장 한 구획(10칸)이면 한 셀에 짧은 평행선 20~40개가 나와야 한다.")
    print("  최대 밀도가 그보다 훨씬 낮고 구성이 530(정지선) 위주면 주차장이 아니다.")


def print_ascii_map(mgeo: KatriMGeo, width: int = 76, height: int = 40) -> None:
    rule("8. ASCII 조감도 (lane_boundary, 세로가 +y)")
    bounds = mgeo.bounds()
    if bounds is None:
        print("  그릴 기하가 없다.")
        return
    min_x, min_y, _, max_x, max_y, _ = bounds
    span_x = max(max_x - min_x, 1e-6)
    span_y = max(max_y - min_y, 1e-6)
    grid = [[" "] * width for _ in range(height)]
    for feature in mgeo.lane_boundaries():
        points = points_xyz(feature)[:, :2]
        for point in points:
            column = int((point[0] - min_x) / span_x * (width - 1))
            row = int((1.0 - (point[1] - min_y) / span_y) * (height - 1))
            grid[row][column] = "."
    for feature in mgeo.links():
        points = points_xyz(feature)[:, :2]
        for point in points:
            column = int((point[0] - min_x) / span_x * (width - 1))
            row = int((1.0 - (point[1] - min_y) / span_y) * (height - 1))
            grid[row][column] = "#"
    print("  범위 x[%.1f, %.1f] y[%.1f, %.1f]   '#'=link '.'=lane_boundary" % (
        min_x, max_x, min_y, max_y))
    for row in grid:
        print("  |" + "".join(row) + "|")


def run_check(mgeo: KatriMGeo) -> int:
    spaces = mgeo.parking_spaces()
    if spaces:
        print("OK  주차면 %d개 (출처: %s)" % (len(spaces), mgeo.parking_space_source))
        return 0
    print("NG  주차면 기하가 없다.")
    print()
    print("MORAI 에 요청할 파일 (원본 export 에는 있었다는 증거가 global_info 안에 있다):")
    for name, digest in sorted(EXPECTED_MISSING_SHA256.items()):
        print("  %-26s sha256 %s" % (name, digest))
    print()
    print("받기 전까지는 config/vip3_parking_spaces.json 에 손으로 적는다.")
    print("스키마는 config/vip3_parking_spaces.SCHEMA.md 를 볼 것 (MORAI ParkingSpace 와 동일).")
    return 1


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="KATRI MGeo 오프라인 점검")
    parser.add_argument("--map-dir", default=default_map_dir(),
                        help="MGeo 디렉터리 (기본 $VIP3_DATA/%s)" % DEFAULT_MAP_DIRNAME)
    parser.add_argument("--manual-parking", default=str(
        Path(__file__).resolve().parents[1] / "config" / "vip3_parking_spaces.json"),
        help="손으로 적은 주차면 fallback JSON")
    parser.add_argument("--check", action="store_true",
                        help="주차 레이어가 있으면 exit 0, 없으면 exit 1")
    parser.add_argument("--no-verify", action="store_true",
                        help="sha256 재계산을 건너뛴다 (빠름)")
    parser.add_argument("--ascii", action="store_true", help="ASCII 조감도까지 출력")
    args = parser.parse_args(argv)

    map_dir = Path(os.path.expandvars(os.path.expanduser(args.map_dir)))
    if not map_dir.is_dir():
        print("맵 디렉터리가 없다: %s" % map_dir, file=sys.stderr)
        print("VIP3_DATA 를 설정했는지, 디렉터리 이름을 따옴표로 감쌌는지 확인할 것.",
              file=sys.stderr)
        return 2

    manual = Path(args.manual_parking) if args.manual_parking else None
    mgeo = KatriMGeo.from_dir(map_dir, manual_parking_path=manual)

    if args.check:
        return run_check(mgeo)

    print("KATRI MGeo 점검: %s" % map_dir)
    print_frame(mgeo)
    print_layers(mgeo)
    print_lane_distribution(mgeo)
    print_surface_markings(mgeo)
    print_manifest(mgeo, verify=not args.no_verify)
    print_parking(mgeo)
    print_parking_cluster_heuristic(mgeo)
    if args.ascii:
        print_ascii_map(mgeo)
    print()
    print("주차 레이어 유무만 확인하려면: %s --check" % Path(__file__).name)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
