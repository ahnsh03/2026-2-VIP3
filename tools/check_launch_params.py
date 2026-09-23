#!/usr/bin/env python3
"""런치 파일이 노드의 `~param` 을 설정할 수 있는지 본다.

    python3 tools/check_launch_params.py          # rosparam 파일이 깨졌으면 exit 1
    python3 tools/check_launch_params.py --all    # 런치에서 못 바꾸는 파라미터도 나열

두 가지를 본다.

1. **`<rosparam file=...>` 가 실제 파일을 가리키는가** — 아니면 `roslaunch` 가 기동
   즉시 죽는다. 이건 오류로 취급한다 (`$(arg ...)`·`$(find ...)`·`$(optenv ...)` 를 편다).
2. 노드가 읽는데 런치가 안 주는 `~param` — **보고만 한다.** 노드 기본값에 맡기는 건
   정상적인 ROS 관행이라 그 자체로는 오류가 아니다. 다만 "런치 인자로 문서화된 기능이
   실제로는 못 켜지는" 경우를 눈으로 잡을 수 있다. 실제로 그렇게 죽은 기능이 있었다
   (`katri_map_viz` 의 `pose_source:=gps_imu`).
"""

from __future__ import annotations

import argparse
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC = REPO_ROOT / "src"
SKIP = ("morai_msgs", "__pycache__")
PACKAGES = {
    path.parent.name: path.parent
    for path in SRC.rglob("package.xml")
    if not any(part in SKIP for part in path.parts)
}


def substitute(raw: str, args: dict, depth: int = 0) -> str:
    if depth > 8:
        return raw
    out = re.sub(r"\$\(arg ([A-Za-z0-9_]+)\)",
                 lambda m: args.get(m.group(1), m.group(0)), raw)
    out = re.sub(r"\$\(optenv [A-Za-z0-9_]+ ([^)]*)\)", lambda m: m.group(1), out)
    return substitute(out, args, depth + 1) if out != raw else out


def resolve(raw: str, args: dict):
    match = re.match(r"\$\(find ([A-Za-z0-9_]+)\)(.*)", substitute(raw or "", args))
    if match and match.group(1) in PACKAGES:
        return PACKAGES[match.group(1)] / match.group(2).lstrip("/")
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--all", action="store_true",
                        help="런치에서 못 바꾸는 ~param 도 나열한다")
    arguments = parser.parse_args()

    errors, notes, launches = [], [], 0
    for launch in sorted(SRC.rglob("*.launch")):
        if any(part in SKIP for part in launch.parts):
            continue
        launches += 1
        tree = ET.parse(launch).getroot()
        args = {a.get("name"): (a.get("default") or a.get("value") or "")
                for a in tree.iter("arg")}
        for node in tree.iter("node"):
            script = node.get("type", "")
            if not script.endswith(".py"):
                continue
            candidates = [p for p in SRC.rglob(script)
                          if not any(part in SKIP for part in p.parts)]
            if len(candidates) != 1:
                continue
            read = set(re.findall(
                r'get_param\(\s*"~([A-Za-z0-9_]+)"',
                candidates[0].read_text(encoding="utf-8"),
            ))
            given = {p.get("name") for p in node.findall("param")}
            for rosparam in node.findall("rosparam"):
                if rosparam.get("param"):
                    given.add(rosparam.get("param"))
                    continue
                path = resolve(rosparam.get("file", ""), args)
                if path and path.is_file():
                    document = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
                    namespace = (rosparam.get("ns") or "").strip("/")
                    for key in document:
                        given.add("{}/{}".format(namespace, key) if namespace else str(key))
                elif rosparam.get("file"):
                    errors.append("{}: rosparam file 을 찾을 수 없다 -> {}".format(
                        launch.relative_to(REPO_ROOT), rosparam.get("file")))
            missing = sorted(read - given)
            if missing:
                notes.append("{}  node={}\n      {}".format(
                    launch.relative_to(REPO_ROOT), node.get("name"),
                    ", ".join("~" + name for name in missing)))

    print("런치 {}개 검사".format(launches))
    if arguments.all and notes:
        print("\n[참고] 런치에서 못 바꾸는 ~param (노드 기본값을 그대로 쓴다):")
        for note in notes:
            print("  " + note)
    if errors:
        print("\n문제 {}건:".format(len(errors)))
        for error in errors:
            print("  ! {}".format(error))
        return 1
    print("\nrosparam 파일 경로 문제 없음")
    return 0


if __name__ == "__main__":
    sys.exit(main())
