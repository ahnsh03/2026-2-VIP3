# -*- coding: utf-8 -*-
"""데이터셋 레이아웃 writer: meta.yaml + manifest.jsonl + 채널별 파일."""

from __future__ import print_function

import json
import os
import subprocess
from datetime import datetime

import yaml

from data_collection import SCHEMA_VERSION


def default_vip3_data_root():
    if os.environ.get("VIP3_DATA"):
        return os.path.expanduser(os.environ["VIP3_DATA"])
    ws = os.environ.get("VIP3_WS_ROOT")
    if ws:
        return os.path.join(os.path.expanduser(ws), "data")
    return os.path.expanduser("~/projects/2026-2-Vertically Integrated Project 3/data")


def _git_sha(cwd=None):
    try:
        out = subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=cwd,
            stderr=subprocess.DEVNULL,
        )
        return out.decode("utf-8").strip()
    except Exception:
        return "unknown"


class DatasetWriter(object):
    def __init__(self, profile, run_id=None, data_root=None, repo_cwd=None):
        self.profile = profile
        self.data_root = os.path.expanduser(data_root or default_vip3_data_root())
        self.run_id = run_id or datetime.now().strftime("%Y%m%d_%H%M%S")
        self.root = os.path.join(self.data_root, "datasets", self.run_id)
        self._frame_idx = 0
        self._skipped = {"no_ego": 0, "channel_missing": 0}
        self._manifest = None
        self._ego_fp = None
        self.repo_cwd = repo_cwd
        self._prepare_dirs()
        self._write_meta()
        self._manifest = open(
            os.path.join(self.root, "manifest.jsonl"), "a", buffering=1
        )
        if profile["channels"].get("ego"):
            self._ego_fp = open(
                os.path.join(self.root, "state", "ego.jsonl"), "a", buffering=1
            )

    def _prepare_dirs(self):
        os.makedirs(self.root, exist_ok=True)
        for sub in (
            "sensors/cam_front",
            "sensors/cam_left",
            "sensors/cam_right",
            "sensors/cam_rear",
            "sensors/lidar",
            "state",
            "gt/objects",
            "gt/collision",
            "cmd",
        ):
            os.makedirs(os.path.join(self.root, sub), exist_ok=True)

    def _write_meta(self):
        meta = {
            "schema_version": SCHEMA_VERSION,
            "run_id": self.run_id,
            "profile": self.profile.get("name"),
            "description": self.profile.get("description"),
            "channels": self.profile.get("channels"),
            "topics": self.profile.get("topics"),
            "collect_hz": self.profile.get("collect_hz"),
            "mode": self.profile.get("mode"),
            "map": self.profile.get("map"),
            "vehicle": self.profile.get("vehicle"),
            "bridge_type": self.profile.get("bridge_type"),
            "git_sha": _git_sha(self.repo_cwd),
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "notes": self.profile.get("notes"),
            "morai_msgs_branch": "26.R1 @ 4c9be6f",
            "warning": "gt/ 는 학습·평가 전용이다. 주행 그래프에서 import 하지 말 것",
        }
        path = os.path.join(self.root, "meta.yaml")
        with open(path, "w") as f:
            yaml.safe_dump(meta, f, default_flow_style=False, allow_unicode=True)

    @property
    def frame_idx(self):
        return self._frame_idx

    def close(self):
        if self._manifest:
            self._manifest.close()
            self._manifest = None
        if self._ego_fp:
            self._ego_fp.close()
            self._ego_fp = None
        summary = {
            "frames_written": self._frame_idx,
            "skipped": self._skipped,
        }
        with open(os.path.join(self.root, "summary.json"), "w") as f:
            json.dump(summary, f, indent=2)

    def write_frame(self, payload):
        """
        payload keys (optional):
          sim_time, stamp_ns, ego,
          cams: {name: jpeg_bytes},
          lidar_xyzi: Nx4 float32 bytes or None,
          objects: dict/list,
          collision: dict,
          missing: [channel names skipped]
        """
        idx = self._frame_idx
        stem = "{:06d}".format(idx)
        paths = {}
        keys = {}
        missing = list(payload.get("missing") or [])

        if "ego" in payload and payload["ego"] is not None and self._ego_fp:
            row = {"frame_idx": idx, "sim_time": payload.get("sim_time"), "ego": payload["ego"]}
            self._ego_fp.write(json.dumps(row, ensure_ascii=False) + "\n")
            keys["ego_row"] = idx

        cams = payload.get("cams") or {}
        for name, blob in cams.items():
            if not blob:
                missing.append(name)
                continue
            rel = "sensors/{}/{}.jpg".format(name, stem)
            with open(os.path.join(self.root, rel), "wb") as f:
                f.write(blob)
            paths[name] = rel

        lidar_bytes = payload.get("lidar_xyzi")
        if lidar_bytes is not None:
            rel = "sensors/lidar/{}.bin".format(stem)
            with open(os.path.join(self.root, rel), "wb") as f:
                f.write(lidar_bytes)
            paths["lidar"] = rel
            keys["lidar_points"] = payload.get("lidar_n", 0)

        if payload.get("objects") is not None:
            rel = "gt/objects/{}.json".format(stem)
            with open(os.path.join(self.root, rel), "w") as f:
                json.dump(payload["objects"], f, ensure_ascii=False)
            keys["gt_objects"] = rel

        if payload.get("collision") is not None:
            rel = "gt/collision/{}.json".format(stem)
            with open(os.path.join(self.root, rel), "w") as f:
                json.dump(payload["collision"], f, ensure_ascii=False)
            keys["collision"] = rel

        if payload.get("gps") is not None:
            rel = "state/gps/{}.json".format(stem)
            os.makedirs(os.path.join(self.root, "state", "gps"), exist_ok=True)
            with open(os.path.join(self.root, rel), "w") as f:
                json.dump(payload["gps"], f, ensure_ascii=False)
            keys["gps"] = rel

        if payload.get("imu") is not None:
            rel = "state/imu/{}.json".format(stem)
            os.makedirs(os.path.join(self.root, "state", "imu"), exist_ok=True)
            with open(os.path.join(self.root, rel), "w") as f:
                json.dump(payload["imu"], f, ensure_ascii=False)
            keys["imu"] = rel

        if missing:
            self._skipped["channel_missing"] += 1

        rec = {
            "frame_idx": idx,
            "sim_time": payload.get("sim_time"),
            "stamp_ns": payload.get("stamp_ns"),
            "paths": paths,
            "keys": keys,
        }
        if missing:
            rec["missing"] = missing
        self._manifest.write(json.dumps(rec, ensure_ascii=False) + "\n")
        self._frame_idx += 1
        return idx
