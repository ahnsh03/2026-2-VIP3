#!/usr/bin/env python3
"""Evaluate a frozen 2-head TwinLite checkpoint without updating weights.

드라이버블 + (binary lane | 4-class road_marking) 두 task 를 모두 평가한다.
task 는 체크포인트에서 읽고 dataset manifest 와 일치하는지 확인한다.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import sys
import time
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Optional

import torch
from torch.utils.data import DataLoader


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "perception"))

from twinlite_morai import (  # noqa: E402
    MaskedTwinLiteLoss,
    MoraiTwinLiteDataset,
    TwinLiteTaskSpec,
    adapt_twinlite_outputs,
    configure_twinlite_task,
    load_external_twinlite,
    move_batch_to_device,
    task_from_checkpoint,
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-root", default=os.environ.get("VIP3_DATA", "/data"))
    parser.add_argument("--dataset-version", required=True)
    parser.add_argument("--split", action="append", choices=("train", "val", "test"), required=True)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--twinlite-root", default=os.environ.get("TWINLITE_ROOT"))
    parser.add_argument("--config", default="medium", choices=("nano", "small", "medium", "large"))
    parser.add_argument("--batch-size", type=int, default=12)
    parser.add_argument("--num-workers", type=int, default=6)
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--weights", default="auto", choices=("auto", "ema", "model"))
    parser.add_argument("--no-amp", action="store_true")
    parser.add_argument("--channels-last", action="store_true")
    parser.add_argument("--log-interval", type=int, default=50)
    parser.add_argument("--max-samples", type=int, help="Smoke-test limit per split")
    parser.add_argument(
        "--scenario-regex",
        help=(
            "run_id 에서 시나리오 라벨을 뽑는 정규식(첫 캡처 그룹). "
            "생략하면 run_id 의 마지막 '_' 토큰을 쓴다."
        ),
    )
    return parser.parse_args()


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _scenario(run_id: str, pattern: Optional["re.Pattern"]) -> str:
    """run_id 에서 조건 축(시나리오) 라벨을 뽑는다.

    ASMC 는 K-City 날씨/시각(`_foggy_`, `_sunny_`, `_sim11am_`)을 파싱했지만
    KATRI 주차 run_id 규약이 아직 없다. 기본값은 run_id 의 마지막 토큰이며,
    규약이 정해지면 `--scenario-regex` 로 첫 캡처 그룹을 라벨로 쓴다.
    TODO(VIP3): 주차 슬롯 위치 / 진입 방향 / 조도 축이 확정되면 기본값을 갱신.
    """
    if pattern is not None:
        match = pattern.search(run_id)
        if match:
            return match.group(1) if match.groups() else match.group(0)
        return "unmatched"
    tail = str(run_id).rsplit("_", 1)[-1]
    return tail or "unknown"


def _new_stats(task_spec: TwinLiteTaskSpec):
    return {
        head: {
            "class_intersection": [0] * task_spec.class_counts[head],
            "class_union": [0] * task_spec.class_counts[head],
            "intersection": 0,
            "union": 0,
            "prediction_positive": 0,
            "target_positive": 0,
            "valid_pixels": 0,
        }
        for head in task_spec.heads
    }


def _add(stats, head, prediction, target, valid, class_count):
    """클래스별 + foreground(클래스>0) 교집합/합집합을 누적한다.

    binary head 에서는 foreground 가 곧 클래스 1 이므로 기존 스칼라 필드
    (intersection/union/iou)의 의미가 그대로 유지된다.
    """
    entry = stats[head]
    for class_index in range(class_count):
        predicted_class = (prediction == class_index) & valid
        target_class = (target == class_index) & valid
        entry["class_intersection"][class_index] += int(
            (predicted_class & target_class).sum().item()
        )
        entry["class_union"][class_index] += int(
            (predicted_class | target_class).sum().item()
        )
    predicted_foreground = (prediction > 0) & valid
    target_foreground = (target > 0) & valid
    entry["intersection"] += int(
        (predicted_foreground & target_foreground).sum().item()
    )
    entry["union"] += int((predicted_foreground | target_foreground).sum().item())
    entry["prediction_positive"] += int(predicted_foreground.sum().item())
    entry["target_positive"] += int(target_foreground.sum().item())
    entry["valid_pixels"] += int(valid.sum().item())


def _head_score(values, class_names) -> float:
    """binary head 는 foreground IoU, 다클래스 head 는 mean foreground IoU."""
    if len(class_names) == 2:
        return float(values["iou"])
    return float(values["mean_foreground_iou"])


def _finalize(stats, task_spec: TwinLiteTaskSpec):
    result = {}
    head_scores = {}
    for head, values in stats.items():
        class_names = task_spec.class_names[head]
        values = dict(values)
        intersections = values["class_intersection"]
        unions = values["class_union"]
        values["class_iou"] = {
            name: intersections[index] / max(1, unions[index])
            for index, name in enumerate(class_names)
        }
        foreground_scores = [
            intersections[index] / unions[index]
            for index in range(1, len(class_names))
            if unions[index] > 0
        ]
        values["present_foreground_classes"] = [
            class_names[index]
            for index in range(1, len(class_names))
            if unions[index] > 0
        ]
        values["mean_foreground_iou"] = (
            sum(foreground_scores) / len(foreground_scores)
            if foreground_scores
            else 0.0
        )
        values["class_intersection"] = dict(zip(class_names, intersections))
        values["class_union"] = dict(zip(class_names, unions))
        values["iou"] = values["intersection"] / max(1, values["union"])
        head_scores[head] = _head_score(values, class_names)
        result[head] = values
    result["head_scores"] = head_scores
    result["mean_head_iou"] = sum(head_scores.values()) / len(head_scores)
    return result


def _write_json(path: Path, value: Any) -> None:
    temporary = path.with_name(path.name + ".part")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")
    os.replace(temporary, path)


def main():
    args = parse_args()
    if args.batch_size <= 0 or args.num_workers < 0:
        raise ValueError("batch-size must be positive and num-workers non-negative")
    device = torch.device(args.device)
    if device.type == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA was requested but is unavailable")
    amp_enabled = device.type == "cuda" and not args.no_amp
    checkpoint_path = Path(args.checkpoint).expanduser().resolve()
    data_root = Path(args.data_root).expanduser().resolve()
    dataset_root = data_root / "dataset_versions" / args.dataset_version
    dataset_metadata_path = dataset_root / "dataset.json"
    if not dataset_metadata_path.is_file():
        raise RuntimeError(f"dataset metadata not found: {dataset_metadata_path}")
    with dataset_metadata_path.open("r", encoding="utf-8") as stream:
        dataset_metadata = json.load(stream)
    split_manifest_sha256 = {
        split: _sha256(dataset_root / (split + ".jsonl"))
        for split in args.split
    }
    checkpoint = torch.load(checkpoint_path, map_location="cpu")
    task_spec = task_from_checkpoint(checkpoint)
    scenario_pattern = re.compile(args.scenario_regex) if args.scenario_regex else None
    use_ema = args.weights == "ema" or (
        args.weights == "auto" and checkpoint.get("ema_state_dict") is not None
    )
    if use_ema and checkpoint.get("ema_state_dict") is None:
        raise RuntimeError("checkpoint has no EMA weights")
    state_key = "ema_state_dict" if use_ema else "model_state_dict"

    model, upstream_commit = load_external_twinlite(
        config=args.config, root=args.twinlite_root
    )
    if checkpoint.get("upstream_commit") != upstream_commit:
        raise RuntimeError("checkpoint and loaded TwinLite source commits differ")
    # 4-class 체크포인트는 out_ll 을 2->4 로 바꾼 뒤에야 state_dict 가 들어간다.
    configure_twinlite_task(model, task_spec)
    model.load_state_dict(checkpoint[state_key])
    model = model.to(device).eval()
    if args.channels_last:
        model = model.to(memory_format=torch.channels_last)
    criterion = MaskedTwinLiteLoss(task_spec=task_spec).to(device)

    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    all_results = {}
    run_started = time.perf_counter()
    if device.type == "cuda":
        torch.cuda.reset_peak_memory_stats(device)

    for split in args.split:
        dataset = MoraiTwinLiteDataset(
            data_root=args.data_root,
            dataset_version=args.dataset_version,
            split=split,
            max_samples=args.max_samples,
        )
        if dataset.task_spec != task_spec:
            raise RuntimeError(
                "checkpoint task differs from dataset task: {} != {}".format(
                    task_spec.name, dataset.task_spec.name
                )
            )
        loader = DataLoader(
            dataset,
            batch_size=args.batch_size,
            shuffle=False,
            num_workers=args.num_workers,
            pin_memory=device.type == "cuda",
            persistent_workers=args.num_workers > 0,
        )
        def new_stats():
            return _new_stats(task_spec)

        aggregate = new_stats()
        by_run = defaultdict(new_stats)
        by_view = defaultdict(new_stats)
        by_scenario = defaultdict(new_stats)
        losses = defaultdict(float)
        per_sample = []
        split_started = time.perf_counter()
        sample_count = 0
        with torch.no_grad():
            for batch_index, host_batch in enumerate(loader, 1):
                batch = move_batch_to_device(host_batch, device)
                if args.channels_last:
                    batch["image"] = batch["image"].contiguous(memory_format=torch.channels_last)
                with torch.cuda.amp.autocast(enabled=amp_enabled):
                    outputs = adapt_twinlite_outputs(
                        model(batch["image"]), task_spec.secondary_head
                    )
                    result_losses = criterion(outputs, batch["targets"], batch["valid_masks"])
                batch_size = int(batch["image"].shape[0])
                sample_count += batch_size
                for name, value in result_losses.items():
                    losses[name] += float(value.item()) * batch_size
                predictions = {
                    head: (
                        output.argmax(dim=1)
                        if output.shape[1] > 1
                        else (output[:, 0] > 0)
                    ).long()
                    for head, output in outputs.items()
                }
                for index in range(batch_size):
                    run_id = str(batch["meta"]["run_id"][index])
                    view = str(batch["meta"]["view"][index])
                    frame_value = batch["meta"]["frame_id"][index]
                    frame_id = int(frame_value.item() if hasattr(frame_value, "item") else frame_value)
                    scenario = _scenario(run_id, scenario_pattern)
                    sample_metrics = {}
                    for head in task_spec.heads:
                        valid = batch["valid_masks"][head][index].bool()
                        prediction = predictions[head][index]
                        target = batch["targets"][head][index].long()
                        for stats in (
                            aggregate,
                            by_run[run_id],
                            by_view[view],
                            by_scenario[scenario],
                        ):
                            _add(
                                stats,
                                head,
                                prediction,
                                target,
                                valid,
                                task_spec.class_counts[head],
                            )
                        foreground_prediction = (prediction > 0) & valid
                        foreground_target = (target > 0) & valid
                        intersection = int(
                            (foreground_prediction & foreground_target).sum().item()
                        )
                        union = int(
                            (foreground_prediction | foreground_target).sum().item()
                        )
                        sample_metrics[head + "_iou"] = intersection / max(1, union)
                    per_sample.append(
                        {
                            "sample_id": host_batch["meta"]["sample_id"][index],
                            "run_id": run_id,
                            "frame_id": frame_id,
                            "view": view,
                            "scenario": scenario,
                            **sample_metrics,
                        }
                    )
                if args.log_interval and (
                    batch_index % args.log_interval == 0 or batch_index == len(loader)
                ):
                    print(
                        "eval split={} batch={}/{} samples={}".format(
                            split, batch_index, len(loader), sample_count
                        ),
                        flush=True,
                    )
        elapsed = time.perf_counter() - split_started
        per_sample_path = output_dir / (split + "_per_sample.jsonl")
        with per_sample_path.open("w", encoding="utf-8") as stream:
            for row in per_sample:
                stream.write(json.dumps(row, ensure_ascii=False, sort_keys=True) + "\n")
        all_results[split] = {
            "sample_count": sample_count,
            "elapsed_seconds": elapsed,
            "samples_per_second": sample_count / max(elapsed, 1e-9),
            "losses": {name: value / max(1, sample_count) for name, value in losses.items()},
            "overall": _finalize(aggregate, task_spec),
            "by_run": {
                name: _finalize(stats, task_spec)
                for name, stats in sorted(by_run.items())
            },
            "by_view": {
                name: _finalize(stats, task_spec)
                for name, stats in sorted(by_view.items())
            },
            "by_scenario": {
                name: _finalize(stats, task_spec)
                for name, stats in sorted(by_scenario.items())
            },
            "per_sample_manifest": str(per_sample_path),
        }

    report = {
        "schema_version": "twinlite-frozen-baseline-eval-1.1.0",
        "completed_at_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "dataset_version": args.dataset_version,
        "dataset_metadata_sha256": _sha256(dataset_metadata_path),
        "dataset_split_manifest_sha256": split_manifest_sha256,
        "dataset_target_policy_ids": dataset_metadata.get("target_policy_ids", []),
        "splits": list(args.split),
        "checkpoint": str(checkpoint_path),
        "checkpoint_sha256": _sha256(checkpoint_path),
        "checkpoint_epoch": checkpoint.get("epoch"),
        "task_spec": task_spec.as_dict(),
        "scenario_regex": args.scenario_regex,
        "weights": state_key,
        "upstream_commit": upstream_commit,
        "device": str(device),
        "gpu": torch.cuda.get_device_name(device) if device.type == "cuda" else None,
        "amp": amp_enabled,
        "batch_size": args.batch_size,
        "num_workers": args.num_workers,
        "elapsed_seconds": time.perf_counter() - run_started,
        "peak_cuda_memory_bytes": torch.cuda.max_memory_allocated(device) if device.type == "cuda" else 0,
        "training_performed": False,
        "results": all_results,
    }
    _write_json(output_dir / "metrics.json", report)
    print(json.dumps(report, ensure_ascii=False, sort_keys=True), flush=True)


if __name__ == "__main__":
    main()
