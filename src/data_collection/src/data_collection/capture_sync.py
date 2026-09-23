# -*- coding: utf-8 -*-
"""MORAI Capture Mode 산출물 동기화 헬퍼 (오프라인).

시뮬레이터는 Windows 쪽에 capture 파일을 쓴다. 이 모듈은 run 하나를
capture_manifest 와 대조해 검증한 뒤 $VIP3_DATA 로 복사한다.
ROS 의존이 없다 — WSL 호스트 python3 로 바로 돌아간다.

**중요**: ROS `/SaveSensorData` 백엔드는 저장 완료 응답을 주지 않는다.
capture_manifest 의 `success` 는 "publish 가 리턴했다"는 뜻일 뿐이다.
실제 파일이 있는지 확인하는 유일한 게이트가 여기의 preflight 다.
그래서 `scripts/sync_capture_data.py --dry-run` 이 선택이 아니라 필수다.
"""

from __future__ import print_function

import json
import os
import shutil
import struct
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
from pathlib import Path


SYNC_SCHEMA_VERSION = "vip3-capture-sync-1.0.0"

# (MORAI SensorData 하위 폴더, 우리 뷰 이름, 원본 (H, W))
# config/vip3_topics.yaml 의 cameras 블록이 정본이다. 값이 어긋나면 preflight 가
# 프레임마다 "shape mismatch" 로 터진다 — 이름만 바꾸지 말 것.
# CAMERA_4 는 ASMC 의 하향 traffic_light 카메라가 아니라 **후방** 카메라다.
# 광학이 front 와 동일(1280x720, FOV 90)하고, 학습에 쓰는 뷰다.
CAMERAS = (
    ("CAMERA_1", "front", (720, 1280)),
    ("CAMERA_2", "left", (480, 640)),
    ("CAMERA_3", "right", (480, 640)),
    ("CAMERA_4", "rear", (720, 1280)),
)
# **뷰 이름의 정본.** curation / mask_baker / perception_dataset 이 전부 여기를 import 한다.
# 예전에는 네 곳이 각자 ("front","left","right") 를 들고 있었고, 수집은 4뷰인데
# 이후 단계가 3뷰라 후방 데이터가 조용히 버려졌다. 같은 실수를 막으려고 하나로 묶었다.
# 카메라를 추가하면 위 CAMERAS 만 고치면 전 단계가 따라온다.
VIEWS = tuple(view for _, view, _ in CAMERAS)

MODALITIES = (
    ("Intensity", "intensity"),
    ("Semantic", "semantic"),
    ("Instance", "instance"),
    ("Depth", "depth"),
)
STATE_SENSORS = (("GPS_6", "gps"), ("IMU_7", "imu"))


class CaptureSyncError(RuntimeError):
    pass


def _utc_now_iso():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def load_successful_records(manifest_path):
    """Load successful captures and reject duplicate sequence/name keys."""
    records = []
    sequences = set()
    names = set()
    with Path(manifest_path).open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                record = json.loads(line)
            except ValueError as exc:
                raise CaptureSyncError(
                    "invalid JSON at {}:{}: {}".format(
                        manifest_path, line_number, exc
                    )
                )
            if not record.get("success"):
                continue
            sequence = int(record["sequence"])
            custom_name = str(record["custom_name"])
            if sequence in sequences:
                raise CaptureSyncError("duplicate sequence: {}".format(sequence))
            if custom_name in names:
                raise CaptureSyncError("duplicate custom_name: {}".format(custom_name))
            sequences.add(sequence)
            names.add(custom_name)
            records.append(record)
    records.sort(key=lambda item: int(item["sequence"]))
    if not records:
        raise CaptureSyncError("manifest has no successful captures")
    return records


def select_records(records, excluded_sequences=None):
    """Return kept/excluded records after validating explicit exclusions."""
    requested = {int(value) for value in (excluded_sequences or ())}
    available = {int(record["sequence"]) for record in records}
    unknown = sorted(requested - available)
    if unknown:
        raise CaptureSyncError(
            "excluded sequence(s) are not successful manifest records: {}".format(
                unknown
            )
        )
    kept = [
        record for record in records if int(record["sequence"]) not in requested
    ]
    excluded = [
        record for record in records if int(record["sequence"]) in requested
    ]
    if not kept:
        raise CaptureSyncError("explicit exclusions removed every successful capture")
    return kept, excluded


def png_hw(path):
    """Return PNG (height, width) without an image-library dependency."""
    with Path(path).open("rb") as stream:
        header = stream.read(24)
    if len(header) != 24 or header[:8] != b"\x89PNG\r\n\x1a\n":
        raise CaptureSyncError("invalid PNG header: {}".format(path))
    width, height = struct.unpack(">II", header[16:24])
    return int(height), int(width)


def files_per_frame():
    """frame 1개가 가져야 하는 원본 파일 수 (카메라 x 모달리티 + 상태 센서)."""
    return len(CAMERAS) * len(MODALITIES) + len(STATE_SENSORS)


def frame_sources(sensor_root, record):
    """capture record 1개가 기대하는 원본 파일 전부 (현재 18개)."""
    sensor_root = Path(sensor_root)
    custom_name = str(record["custom_name"])
    sources = []
    for sensor_dir, view, expected_hw in CAMERAS:
        for source_suffix, modality in MODALITIES:
            sources.append(
                {
                    "kind": "image",
                    "view": view,
                    "modality": modality,
                    "expected_hw": expected_hw,
                    "path": sensor_root
                    / sensor_dir
                    / "{}_{}.png".format(custom_name, source_suffix),
                }
            )
    for sensor_dir, name in STATE_SENSORS:
        sources.append(
            {
                "kind": "state",
                "name": name,
                "path": sensor_root / sensor_dir / "{}.txt".format(custom_name),
            }
        )
    return sources


def preflight_run(sensor_root, records, validate_png=True):
    """Validate completeness, file sizes, and native camera dimensions."""
    frames = []
    total_bytes = 0
    errors = []
    for record in records:
        entries = frame_sources(sensor_root, record)
        for entry in entries:
            path = entry["path"]
            try:
                size = path.stat().st_size
            except FileNotFoundError:
                errors.append("missing: {}".format(path))
                continue
            if size <= 0:
                errors.append("empty: {}".format(path))
                continue
            entry["size"] = int(size)
            total_bytes += size
            if validate_png and entry["kind"] == "image":
                try:
                    actual_hw = png_hw(path)
                except (OSError, CaptureSyncError) as exc:
                    errors.append(str(exc))
                    continue
                if actual_hw != entry["expected_hw"]:
                    errors.append(
                        "shape mismatch: {} expected={} actual={}".format(
                            path, entry["expected_hw"], actual_hw
                        )
                    )
        frames.append((record, entries))
    if errors:
        preview = "\n".join(errors[:20])
        suffix = "\n... {} more".format(len(errors) - 20) if len(errors) > 20 else ""
        raise CaptureSyncError(
            "preflight failed with {} error(s):\n{}{}".format(
                len(errors), preview, suffix
            )
        )
    return frames, total_bytes


def _destination_path(run_root, sequence, entry):
    stem = "{:06d}".format(int(sequence))
    if entry["kind"] == "image":
        return run_root / "frames" / entry["modality"] / entry["view"] / (stem + ".png")
    return run_root / "state" / entry["name"] / (stem + ".txt")


def _copy_atomic(source, destination):
    if destination.exists() and destination.stat().st_size == source.stat().st_size:
        return "skipped", source.stat().st_size
    temporary = destination.with_name(destination.name + ".part")
    if temporary.exists():
        temporary.unlink()
    shutil.copy2(str(source), str(temporary))
    os.replace(str(temporary), str(destination))
    return "copied", source.stat().st_size


def _write_json_atomic(path, value):
    temporary = path.with_name(path.name + ".part")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")
    os.replace(str(temporary), str(path))


def _write_jsonl_atomic(path, rows):
    temporary = path.with_name(path.name + ".part")
    with temporary.open("w", encoding="utf-8") as stream:
        for row in rows:
            stream.write(json.dumps(row, ensure_ascii=False, sort_keys=True) + "\n")
    os.replace(str(temporary), str(path))


def sync_run(
    sensor_root,
    capture_root,
    dataset_root,
    run_id,
    jobs=4,
    validate_png=True,
    excluded_sequences=None,
):
    """Preflight and resumably copy one run. Source files are never deleted."""
    capture_dir = Path(capture_root) / run_id
    source_manifest = capture_dir / "capture_manifest.jsonl"
    source_summary = capture_dir / "summary.json"
    if not source_manifest.is_file():
        raise CaptureSyncError("capture manifest not found: {}".format(source_manifest))
    if not source_summary.is_file():
        raise CaptureSyncError("capture summary not found: {}".format(source_summary))

    source_records = load_successful_records(source_manifest)
    with source_summary.open("r", encoding="utf-8") as stream:
        summary = json.load(stream)
    expected_successes = int(summary.get("succeeded", -1))
    if expected_successes != len(source_records):
        raise CaptureSyncError(
            "summary/manifest mismatch: succeeded={} records={}".format(
                expected_successes, len(source_records)
            )
        )

    records, excluded_records = select_records(
        source_records, excluded_sequences=excluded_sequences
    )
    frames, total_bytes = preflight_run(sensor_root, records, validate_png=validate_png)
    run_root = Path(dataset_root) / run_id
    run_root.mkdir(parents=True, exist_ok=True)

    tasks = []
    manifest_rows = []
    directories = set()
    for record, entries in frames:
        sequence = int(record["sequence"])
        paths = {modality: {} for _, modality in MODALITIES}
        paths["state"] = {}
        source_files = []
        for entry in entries:
            destination = _destination_path(run_root, sequence, entry)
            directories.add(destination.parent)
            tasks.append((entry["path"], destination))
            relative = destination.relative_to(run_root).as_posix()
            if entry["kind"] == "image":
                paths[entry["modality"]][entry["view"]] = relative
            else:
                paths["state"][entry["name"]] = relative
            source_files.append(str(entry["path"]))
        manifest_rows.append(
            {
                "schema_version": SYNC_SCHEMA_VERSION,
                "run_id": run_id,
                "frame_id": sequence,
                "custom_name": record["custom_name"],
                "morai_sim_time_raw": record.get(
                    "morai_sim_time_raw", record.get("morai_sim_time_ms")
                ),
                "morai_sim_time_unit": record.get(
                    "morai_sim_time_unit",
                    "us_legacy_mislabeled"
                    if record.get("morai_sim_time_ms") is not None else None,
                ),
                "morai_sim_time_ns": record.get(
                    "morai_sim_time_ns",
                    int(record["morai_sim_time_ms"]) * 1000
                    if record.get("morai_sim_time_ms") is not None else None,
                ),
                "ros_time_ns": record.get("ros_time_ns"),
                "requested_at_utc": record.get("requested_at_utc"),
                "state_snapshot": record.get("state"),
                "paths": paths,
                "source_files": source_files,
                "valid": True,
            }
        )

    for directory in directories:
        directory.mkdir(parents=True, exist_ok=True)

    missing_bytes = sum(
        source.stat().st_size
        for source, destination in tasks
        if not destination.exists() or destination.stat().st_size != source.stat().st_size
    )
    free_bytes = shutil.disk_usage(str(run_root)).free
    if free_bytes < int(missing_bytes * 1.05):
        raise CaptureSyncError(
            "not enough free space: need={} available={}".format(
                missing_bytes, free_bytes
            )
        )

    copied_files = 0
    skipped_files = 0
    copied_bytes = 0
    with ThreadPoolExecutor(max_workers=max(1, int(jobs))) as executor:
        futures = [executor.submit(_copy_atomic, source, destination) for source, destination in tasks]
        for index, future in enumerate(as_completed(futures), 1):
            status, size = future.result()
            if status == "copied":
                copied_files += 1
                copied_bytes += size
            else:
                skipped_files += 1
            if index % 1000 == 0 or index == len(futures):
                print("[sync] {}/{} files".format(index, len(futures)), flush=True)

    lineage_dir = run_root / "lineage"
    lineage_dir.mkdir(parents=True, exist_ok=True)
    for name in ("meta.json", "summary.json", "capture_manifest.jsonl"):
        source = capture_dir / name
        if source.is_file():
            _copy_atomic(source, lineage_dir / name)

    _write_jsonl_atomic(run_root / "manifest.jsonl", manifest_rows)
    dataset_meta = {
        "schema_version": SYNC_SCHEMA_VERSION,
        "run_id": run_id,
        "created_at_utc": _utc_now_iso(),
        "source_sensor_root": str(Path(sensor_root)),
        "source_capture_root": str(capture_dir),
        "frame_count": len(frames),
        "source_successful_frame_count": len(source_records),
        "excluded_frame_count": len(excluded_records),
        "excluded_captures": [
            {
                "sequence": int(record["sequence"]),
                "custom_name": str(record["custom_name"]),
                "reason": "operator-specified incomplete capture",
            }
            for record in excluded_records
        ],
        "files_per_frame": files_per_frame(),
        "source_bytes": total_bytes,
        "camera_views": {
            view: {"sensor_dir": sensor_dir, "height": hw[0], "width": hw[1]}
            for sensor_dir, view, hw in CAMERAS
        },
        "modalities": [name for _, name in MODALITIES],
        "twinlite_views": [view for _, view, _ in CAMERAS],
        "source_deleted": False,
    }
    _write_json_atomic(run_root / "dataset.json", dataset_meta)
    _write_json_atomic(
        run_root / "_SUCCESS",
        {
            "completed_at_utc": _utc_now_iso(),
            "frame_count": len(frames),
            "file_count": len(tasks),
        },
    )
    return {
        "run_id": run_id,
        "run_root": str(run_root),
        "frame_count": len(frames),
        "source_successful_frame_count": len(source_records),
        "excluded_frame_count": len(excluded_records),
        "excluded_sequences": [
            int(record["sequence"]) for record in excluded_records
        ],
        "file_count": len(tasks),
        "copied_files": copied_files,
        "skipped_files": skipped_files,
        "copied_bytes": copied_bytes,
        "source_bytes": total_bytes,
    }
