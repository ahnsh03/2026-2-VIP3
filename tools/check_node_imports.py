#!/usr/bin/env python3
"""ROS 노드 스크립트가 **존재하지 않는 이름을 import 하는지** 정적으로 검사한다.

    python3 tools/check_node_imports.py       # 문제 있으면 exit 1

왜 필요한가. 노드 스크립트는 `rospy` 가 있어야 import 되므로 호스트 단위 테스트가
건드리지 못한다. 그래서 라이브러리 쪽 이름을 바꾸면 노드는 조용히 깨진 채로 남고,
`roslaunch` 를 돌려야 비로소 `ImportError` 가 난다.

실제로 그랬다 — `bag_profile` 의 대회 게이트(`privileged_gt`)를 VIP3 게이트
(`simulator_gt`)로 바꾸면서 `bag_replay_node.py` 의 import 를 안 고쳐서, bag 재생
노드가 기동 즉시 죽는 상태였다. 테스트 190개가 전부 통과하는 동안에도.

ast 만 쓴다. 모듈을 실제로 import 하지 않으므로 rospy·torch 없이 돈다.
"""

from __future__ import annotations

import ast
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SKIP_PARTS = ("morai_msgs", "__pycache__", ".git")


def package_roots() -> dict:
    """import 가능한 파이썬 패키지 이름 -> 디렉터리."""

    roots = {}
    for init in REPO_ROOT.glob("src/**/src/*/__init__.py"):
        if any(part in SKIP_PARTS for part in init.parts):
            continue
        roots[init.parent.name] = init.parent
    return roots


def module_names(path: Path) -> set:
    """모듈이 **내보내는** 최상위 이름 (`__all__` 이 있으면 그것)."""

    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    names, explicit = set(), None
    for node in tree.body:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            names.add(node.name)
        elif isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name):
                    names.add(target.id)
                    if target.id == "__all__" and isinstance(node.value, (ast.List, ast.Tuple)):
                        explicit = {
                            element.value for element in node.value.elts
                            if isinstance(element, ast.Constant)
                        }
        elif isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
            names.add(node.target.id)
        elif isinstance(node, (ast.Import, ast.ImportFrom)):
            for alias in node.names:
                if alias.name == "*":
                    # 재수출을 정적으로 따라가지 않는다. 이 모듈은 검사 대상에서 뺀다.
                    return set()
                names.add(alias.asname or alias.name.split(".")[0])
    return explicit if explicit is not None else names


def main() -> int:
    packages = package_roots()
    problems = []
    scanned = 0

    for path in sorted(REPO_ROOT.glob("src/**/*.py")) + sorted(REPO_ROOT.glob("scripts/*.py")):
        if any(part in SKIP_PARTS for part in path.parts):
            continue
        try:
            tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        except SyntaxError as error:
            problems.append("{}: 구문 오류 {}".format(path.relative_to(REPO_ROOT), error))
            continue
        scanned += 1
        for node in ast.walk(tree):
            if not isinstance(node, ast.ImportFrom) or node.level or not node.module:
                continue
            head, _, rest = node.module.partition(".")
            if head not in packages or not rest:
                continue
            target = packages[head] / (rest.replace(".", "/") + ".py")
            if not target.is_file():
                target = packages[head] / rest.replace(".", "/") / "__init__.py"
            if not target.is_file():
                problems.append("{}:{}  모듈 없음: {}".format(
                    path.relative_to(REPO_ROOT), node.lineno, node.module))
                continue
            exported = module_names(target)
            if not exported:
                continue
            for alias in node.names:
                if alias.name not in exported:
                    problems.append("{}:{}  {} 에 {} 가 없다".format(
                        path.relative_to(REPO_ROOT), node.lineno,
                        node.module, alias.name))

    print("파이썬 파일 {}개 · 패키지 {}개 검사".format(scanned, len(packages)))
    if problems:
        print("\n문제 {}건:".format(len(problems)))
        for problem in problems:
            print("  ! {}".format(problem))
        return 1
    print("\n패키지 간 import 이름 불일치 없음")
    return 0


if __name__ == "__main__":
    sys.exit(main())
