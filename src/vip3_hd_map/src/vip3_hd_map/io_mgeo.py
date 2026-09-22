"""Read-only adapters for the KATRI MGeo JSON files.

오프라인 audit 전용이던 ``feature_cloud`` / ``source_record`` 는 VIP3로 가져오지
않았다. 여기 남은 넷은 경로 해석과 sha256 검증에만 쓴다.
"""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
from typing import Iterable


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_json(path: Path):
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def resolve_source(repo_root: Path, raw: str) -> Path:
    expanded = Path(os.path.expandvars(os.path.expanduser(raw)))
    return expanded if expanded.is_absolute() else (repo_root / expanded).resolve()


def first_existing_source(repo_root: Path, candidates: Iterable[str]) -> tuple[Path | None, int | None]:
    for index, raw in enumerate(candidates):
        path = resolve_source(repo_root, raw)
        if path.is_file():
            return path, index
    return None, None
