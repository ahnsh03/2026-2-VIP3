#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""저장된 Capture Mode 파일 검사 (ROS 불필요).

프레임당 12파일(CAMERA_1~4 x Intensity/Semantic/Instance)이 존재하고, 비어 있지 않고,
1280x960 PNG 인지 본다. manifest 의 success=true 는 publish 성공일 뿐이므로 이 검사가
유일한 저장 확인이다. 검사만 하고 아무것도 복사/삭제/수정하지 않는다
(--dry-run 은 받기만 하는 호환용 플래그이며 있든 없든 동작은 같다).

  python3 check_capture.py --run-id <run_id>
  python3 check_capture.py --names a_000000 a_000001     # manifest 없이

종료 코드: 0 전부 정상, 1 누락/불량 있음, 2 입력 오류.
"""

from __future__ import print_function

import argparse
import json
import os
import sys

# catkin 설치 없이 소스 트리에서 바로 실행할 수 있게 한다.
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from vip3_data.capture import (
    FILES_PER_FRAME,
    IMAGE_HW,
    MANIFEST_NAME,
    check_run,
    default_data_root,
    default_sensor_root,
    load_published_names,
)


def build_parser():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--run-id", help="capture_node 가 만든 run (manifest 로 기대 이름을 얻는다)")
    p.add_argument("--names", nargs="+", help="manifest 없이 직접 지정하는 custom_file_name 들")
    p.add_argument("--data-root", default=None, help="기본 $VIP3_DATA")
    p.add_argument("--sensor-root", default=None,
                   help="MORAI SensorData (기본 $VIP3_MORAI_SENSOR_ROOT 또는 /mnt/c/MoraiLauncher_Win/...)")
    p.add_argument("--width", type=int, help="기대 가로(기본: run 의 meta.json, 없으면 1280)")
    p.add_argument("--height", type=int, help="기대 세로(기본: run 의 meta.json, 없으면 960)")
    p.add_argument("--skip-last", type=int, default=0,
                   help="수집 중 검사할 때 아직 쓰는 중일 마지막 N프레임 제외")
    p.add_argument("--dry-run", action="store_true", help="검사 전용(기본 동작과 동일)")
    p.add_argument("--json", action="store_true", help="보고서를 JSON 으로 출력")
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    if bool(args.run_id) == bool(args.names):
        print("--run-id 와 --names 중 정확히 하나를 지정하라", file=sys.stderr)
        return 2
    sensor_root = os.path.expanduser(args.sensor_root) if args.sensor_root else default_sensor_root()
    if not os.path.isdir(sensor_root):
        print("sensor root 가 없다: {}".format(sensor_root), file=sys.stderr)
        return 2

    if args.run_id:
        data_root = args.data_root or default_data_root()
        manifest = os.path.join(os.path.expanduser(data_root), "capture_runs",
                                args.run_id, MANIFEST_NAME)
        if not os.path.isfile(manifest):
            print("manifest 가 없다: {}".format(manifest), file=sys.stderr)
            return 2
        try:
            with open(os.path.join(os.path.dirname(manifest), "meta.json"), encoding="utf-8") as stream:
                meta = json.load(stream)
            meta_hw = (int(meta["image_height"]), int(meta["image_width"]))
        except (OSError, ValueError, KeyError):
            meta_hw = None
        try:
            names = [name for _, name in load_published_names(manifest)]
        except ValueError as exc:
            print(str(exc), file=sys.stderr)
            return 2
    else:
        names, meta_hw = args.names, None

    base = meta_hw or IMAGE_HW
    hw = (args.height or base[0], args.width or base[1])
    report = check_run(sensor_root, names, skip_last=args.skip_last, expected_hw=hw)
    if args.json:
        print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))
    else:
        print("sensor_root : {}".format(report["sensor_root"]))
        print("frames      : checked={} ok={} bad={} skipped={}  ({} files/frame)".format(
            report["frames_checked"], report["frames_ok"], report["frames_bad"],
            report["frames_skipped"], FILES_PER_FRAME))
        for name, problems in sorted(report["bad"].items())[:20]:
            print("  BAD {}: {} problem(s)".format(name, len(problems)))
            for item in problems[:FILES_PER_FRAME]:
                print("      {}/{}: {}".format(item["view"], item["modality"], item["problem"]))
        if report["frames_bad"] > 20:
            print("  ... {} more bad frames".format(report["frames_bad"] - 20))
        if not report["frames_checked"]:
            print("검사할 프레임이 없다")
        elif report["ok"]:
            print("OK: 전 프레임 {}파일 확인".format(report["files_expected"]))
        else:
            whole = sum(1 for p in report["bad"].values() if len(p) == FILES_PER_FRAME)
            if whole:
                print("FAIL: {}프레임은 12파일이 전부 없다 -> MORAI 가 요청을 버렸을 가능성. "
                      "capture_hz 를 낮춰라".format(whole))
            if report["frames_bad"] > whole:
                print("FAIL: 일부 파일만 없다 -> 카메라별 Sensor Capture Mode 버튼/크기를 확인하라")
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
