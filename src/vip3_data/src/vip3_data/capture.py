# -*- coding: utf-8 -*-
"""MORAI Capture Mode 수집용 순수 파이썬 헬퍼 (ROS import 없음).

트리거는 morai_msgs/SaveSensorData 를 /SaveSensorData 에 발행하는 것 하나뿐이다.
MORAI 는 저장 완료 응답을 주지 않는다. manifest 의 ``success`` 는 "publish 가
리턴했다"는 뜻일 뿐이므로, 실제 저장은 ``check_run`` 으로 파일을 직접 검사해야 한다.

센서셋: config/morai/261110_VIP3_Sensor_26.R1.H3.json
어안 카메라 4대(CAMERA_1 front, 2 rear, 3 left, 4 right), 1280x960.
카메라마다 Active List 의 Sensor Capture Mode 버튼이 켜져 있어야 3종이 나온다.
저장 위치: <sensor_root>/CAMERA_<ID>/<이름>_{Intensity,Semantic,Instance}.png (Depth 없음).
"""

from __future__ import print_function

import json
import math
import os
import re
import struct
import time
from datetime import datetime, timezone

# (MORAI 카메라 ID, 뷰 이름)
CAMERAS = ((1, "front"), (2, "rear"), (3, "left"), (4, "right"))
MODALITIES = ("Intensity", "Semantic", "Instance")
IMAGE_HW = (960, 1280)  # (height, width) 기본값. 261115 센서셋은 (480, 640)
FILES_PER_FRAME = len(CAMERAS) * len(MODALITIES)

DEFAULT_SENSOR_ROOT = "/mnt/c/MoraiLauncher_Win/MoraiLauncher_Win/SensorData"
MANIFEST_NAME = "capture_manifest.jsonl"
SCHEMA_VERSION = "vip3-capture-1.0.0"

_SAFE_NAME = re.compile(r"[^A-Za-z0-9_-]+")
_PNG_MAGIC = b"\x89PNG\r\n\x1a\n"


def sanitize_name(value, fallback="capture", max_length=80):
    """MORAI 에 보낼 이름을 파일명 안전 문자로 정규화한다."""
    value = _SAFE_NAME.sub("_", str(value or "").strip()).strip("_-")
    return (value or fallback)[:max_length]


def utc_now_iso():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def make_run_id(prefix="capture"):
    now = datetime.now(timezone.utc)
    return "{}_{}_{:03d}".format(
        sanitize_name(prefix, max_length=24),
        now.strftime("%Y%m%dT%H%M%S"),
        now.microsecond // 1000,
    )


def default_sensor_root():
    return os.path.expanduser(
        os.environ.get("VIP3_MORAI_SENSOR_ROOT", DEFAULT_SENSOR_ROOT)
    )


def default_data_root():
    if os.environ.get("VIP3_DATA"):
        return os.path.expanduser(os.environ["VIP3_DATA"])
    ws = os.environ.get("VIP3_WS_ROOT")
    if ws:
        return os.path.join(os.path.expanduser(ws), "data")
    return os.path.expanduser("~/projects/2026-2-Vertically Integrated Project 3/data")


def capture_name(run_id, sequence):
    return "{}_{:06d}".format(run_id, int(sequence))


class CaptureSchedule(object):
    """고정 주기 스케줄. 놓친 슬롯은 몰아서 쏘지 않고 건너뛴다."""

    def __init__(self, hz, start_delay_sec=1.0, monotonic_fn=None):
        hz = float(hz)
        if not math.isfinite(hz) or hz <= 0.0 or hz > 10.0:
            raise ValueError("capture_hz must be in (0, 10]")
        self.hz = hz
        self.period_sec = 1.0 / hz
        self._monotonic = monotonic_fn or time.monotonic
        self.next_deadline = self._monotonic() + max(0.0, float(start_delay_sec))

    def remaining(self):
        return max(0.0, self.next_deadline - self._monotonic())

    def advance(self):
        now = self._monotonic()
        self.next_deadline += self.period_sec
        if self.next_deadline <= now:
            self.next_deadline = now + self.period_sec
        return self.next_deadline


def _write_json(path, value):
    with open(path, "w", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")


class CaptureRunWriter(object):
    """한 run 의 트리거 기록(meta.json, capture_manifest.jsonl, summary.json)."""

    def __init__(self, data_root, run_id, metadata=None):
        self.run_id = sanitize_name(run_id, fallback="capture_run")
        self.root = os.path.join(
            os.path.abspath(os.path.expanduser(data_root)), "capture_runs", self.run_id
        )
        self.manifest_path = os.path.join(self.root, MANIFEST_NAME)
        self.attempted = 0
        self.published = 0
        self.failed = 0
        os.makedirs(self.root, exist_ok=False)  # 기존 run 덮어쓰기 거부
        meta = dict(metadata or {})
        meta.update(
            {
                "schema_version": SCHEMA_VERSION,
                "run_id": self.run_id,
                "created_at_utc": utc_now_iso(),
                "expected_files_per_frame": FILES_PER_FRAME,
            }
        )
        _write_json(os.path.join(self.root, "meta.json"), meta)
        self._manifest = open(self.manifest_path, "a", buffering=1, encoding="utf-8")

    def write_capture(self, record):
        rec = dict(record)
        rec.setdefault("recorded_at_utc", utc_now_iso())
        self._manifest.write(json.dumps(rec, ensure_ascii=False, sort_keys=True) + "\n")
        self.attempted += 1
        if rec.get("success"):
            self.published += 1
        else:
            self.failed += 1

    def close(self):
        if self._manifest is None:
            return
        self._manifest.close()
        self._manifest = None
        _write_json(
            os.path.join(self.root, "summary.json"),
            {
                "run_id": self.run_id,
                "closed_at_utc": utc_now_iso(),
                "attempted": self.attempted,
                "published": self.published,
                "failed": self.failed,
                "note": "published 는 저장 확인이 아니다. check_capture.py 로 파일을 검사할 것",
            },
        )


# --- 저장 파일 검사 -------------------------------------------------------


def frame_files(sensor_root, custom_name):
    """프레임 1개가 가져야 하는 12개 파일: [(view, modality, path)]."""
    out = []
    for cam_id, view in CAMERAS:
        for modality in MODALITIES:
            path = os.path.join(
                sensor_root,
                "CAMERA_{}".format(cam_id),
                "{}_{}.png".format(custom_name, modality),
            )
            out.append((view, modality, path))
    return out


def png_hw(path):
    """PNG (height, width). 이미지 라이브러리 없이 IHDR 만 읽는다."""
    with open(path, "rb") as stream:
        header = stream.read(24)
    if len(header) != 24 or header[:8] != _PNG_MAGIC or header[12:16] != b"IHDR":
        raise ValueError("invalid PNG header: {}".format(path))
    width, height = struct.unpack(">II", header[16:24])
    return int(height), int(width)


def check_frame(sensor_root, custom_name, expected_hw=IMAGE_HW):
    """한 프레임의 문제 목록. 비어 있으면 12파일 모두 정상."""
    problems = []
    for view, modality, path in frame_files(sensor_root, custom_name):
        label = "{}/{}".format(view, modality)
        try:
            size = os.stat(path).st_size
        except OSError:
            problems.append({"file": path, "view": view, "modality": modality,
                             "problem": "missing"})
            continue
        if size <= 0:
            problems.append({"file": path, "view": view, "modality": modality,
                             "problem": "empty"})
            continue
        try:
            hw = png_hw(path)
        except (OSError, ValueError) as exc:
            problems.append({"file": path, "view": view, "modality": modality,
                             "problem": "bad_png: {}".format(exc)})
            continue
        if hw != tuple(expected_hw):
            problems.append({"file": path, "view": view, "modality": modality,
                             "problem": "shape {} != {}".format(hw, tuple(expected_hw))})
    return problems


def load_published_names(manifest_path):
    """manifest 에서 publish 성공한 (sequence, custom_name) 목록. 중복은 오류."""
    rows, seen_seq, seen_name = [], set(), set()
    with open(manifest_path, "r", encoding="utf-8") as stream:
        for number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                rec = json.loads(line)
            except ValueError as exc:
                raise ValueError("invalid JSON at {}:{}: {}".format(manifest_path, number, exc))
            if not rec.get("success"):
                continue
            seq, name = int(rec["sequence"]), str(rec["custom_name"])
            if seq in seen_seq or name in seen_name:
                raise ValueError("duplicate sequence/name at line {}".format(number))
            seen_seq.add(seq)
            seen_name.add(name)
            rows.append((seq, name))
    rows.sort()
    return rows


def check_run(sensor_root, names, skip_last=0, expected_hw=IMAGE_HW):
    """names 전체를 검사해 보고서 dict 를 돌려준다.

    skip_last: 뒤쪽 N개 프레임은 아직 쓰는 중일 수 있어 검사에서 제외한다.
    """
    names = list(names)
    skipped = names[len(names) - skip_last:] if skip_last > 0 else []
    targets = names[: len(names) - skip_last] if skip_last > 0 else names
    bad = {}
    for name in targets:
        problems = check_frame(sensor_root, name, expected_hw)
        if problems:
            bad[name] = problems
    return {
        "sensor_root": sensor_root,
        "expected_hw": list(expected_hw),
        "frames_checked": len(targets),
        "frames_ok": len(targets) - len(bad),
        "frames_bad": len(bad),
        "frames_skipped": len(skipped),
        "files_expected": len(targets) * FILES_PER_FRAME,
        "bad": bad,
        "ok": bool(targets) and not bad,
    }
