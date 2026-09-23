#!/usr/bin/env python3
"""Run a reproducible task-aware TwinLiteNet+ tiny-overfit check."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import random
import sys
import time
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict

import numpy as np
import torch
from torch.utils.data import DataLoader


REPO_ROOT = Path(__file__).resolve().parents[1]
PERCEPTION_SRC = REPO_ROOT / "src" / "perception"
sys.path.insert(0, str(PERCEPTION_SRC))

from twinlite_morai import (  # noqa: E402
    MaskedTwinLiteLoss,
    MoraiTwinLiteDataset,
    ROAD_MARKING_TASK,
    configure_twinlite_task,
    load_external_twinlite,
    load_task_compatible_state_dict,
    move_batch_to_device,
    task_from_checkpoint,
    twinlite_training_step,
)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", default=os.environ.get("VIP3_DATA", "/data"))
    parser.add_argument("--dataset-version", default="vip3_katri_parking_v1")
    parser.add_argument("--split", default="train", choices=("train", "val"))
    parser.add_argument(
        "--selection-file",
        help="JSON file containing dataset_version, split and an ordered sample_ids list",
    )
    parser.add_argument("--twinlite-root", default=os.environ.get("TWINLITE_ROOT"))
    parser.add_argument(
        "--config", default="medium", choices=("nano", "small", "medium", "large")
    )
    parser.add_argument("--max-samples", type=int, default=12)
    parser.add_argument("--epochs", type=int, default=60)
    parser.add_argument("--batch-size", type=int, default=3)
    parser.add_argument("--num-workers", type=int, default=0)
    parser.add_argument("--lr", type=float, default=5e-4)
    parser.add_argument("--weight-decay", type=float, default=5e-4)
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument(
        "--init-checkpoint",
        help="Initialize from a full-training checkpoint without restoring optimizer state",
    )
    parser.add_argument(
        "--init-weights", default="auto", choices=("auto", "ema", "model")
    )
    parser.add_argument(
        "--road-marking-class-weights",
        type=float,
        nargs=4,
        metavar=("BACKGROUND", "WHITE", "YELLOW", "STOPLINE"),
    )
    parser.add_argument(
        "--output-dir",
        default=str(REPO_ROOT / "artifacts/perception_eval/runs/twinlite_tiny_smoke"),
    )
    parser.add_argument(
        "--device", default="cuda" if torch.cuda.is_available() else "cpu"
    )
    parser.add_argument("--no-amp", action="store_true")
    parser.add_argument("--channels-last", action="store_true")
    return parser.parse_args()


def seed_everything(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _write_json(path: Path, value: Any) -> None:
    temporary = path.with_name(path.name + ".part")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")
    os.replace(temporary, path)


def _atomic_torch_save(value: Dict[str, Any], path: Path) -> None:
    temporary = path.with_suffix(path.suffix + ".part")
    torch.save(value, temporary)
    os.replace(temporary, path)


def _apply_selection(dataset, selection_path: Path, dataset_version: str, split: str):
    with selection_path.open("r", encoding="utf-8") as stream:
        selection = json.load(stream)
    if selection.get("dataset_version") != dataset_version:
        raise ValueError("selection file dataset_version does not match")
    if selection.get("split") != split:
        raise ValueError("selection file split does not match")
    sample_ids = [str(value) for value in selection.get("sample_ids", [])]
    if not sample_ids or len(sample_ids) != len(set(sample_ids)):
        raise ValueError("selection file needs unique, non-empty sample_ids")
    rows_by_id = {str(row["sample_id"]): row for row in dataset.rows}
    missing = [sample_id for sample_id in sample_ids if sample_id not in rows_by_id]
    if missing:
        raise ValueError("selection samples are absent from the dataset: {}".format(missing))
    dataset.rows = [rows_by_id[sample_id] for sample_id in sample_ids]
    return selection, sample_ids


def _new_counts(task_spec):
    return {
        head: {
            "intersection": [0] * task_spec.class_counts[head],
            "union": [0] * task_spec.class_counts[head],
            "prediction": [0] * task_spec.class_counts[head],
            "target": [0] * task_spec.class_counts[head],
            "valid_pixels": 0,
        }
        for head in task_spec.heads
    }


def _update_counts(counts, outputs, batch, task_spec):
    for head in task_spec.heads:
        logits = outputs[head]
        prediction = (
            logits.argmax(dim=1)
            if logits.shape[1] > 1
            else (logits[:, 0] > 0).long()
        )
        target = batch["targets"][head].long()
        valid = batch.get("valid_masks", {}).get(head, batch["valid_mask"]).bool()
        counts[head]["valid_pixels"] += int(valid.sum().item())
        for class_index in range(task_spec.class_counts[head]):
            predicted_class = (prediction == class_index) & valid
            target_class = (target == class_index) & valid
            counts[head]["intersection"][class_index] += int(
                (predicted_class & target_class).sum().item()
            )
            counts[head]["union"][class_index] += int(
                (predicted_class | target_class).sum().item()
            )
            counts[head]["prediction"][class_index] += int(
                predicted_class.sum().item()
            )
            counts[head]["target"][class_index] += int(target_class.sum().item())


def _finalize_counts(counts, task_spec):
    metrics = {}
    head_scores = {}
    for head in task_spec.heads:
        values = counts[head]
        names = task_spec.class_names[head]
        class_iou = {
            name: values["intersection"][index] / max(1, values["union"][index])
            for index, name in enumerate(names)
        }
        foreground_scores = [
            class_iou[names[index]]
            for index in range(1, len(names))
            if values["union"][index] > 0
        ]
        metrics[head] = {
            "class_iou": class_iou,
            "class_intersection": dict(zip(names, values["intersection"])),
            "class_union": dict(zip(names, values["union"])),
            "class_prediction_pixels": dict(zip(names, values["prediction"])),
            "class_target_pixels": dict(zip(names, values["target"])),
            "valid_pixels": values["valid_pixels"],
            "mean_foreground_iou": sum(foreground_scores)
            / max(1, len(foreground_scores)),
        }
        if len(names) == 2:
            metrics[head]["iou"] = class_iou[names[1]]
            head_scores[head] = metrics[head]["iou"]
        else:
            head_scores[head] = metrics[head]["mean_foreground_iou"]
    return metrics, head_scores


@torch.no_grad()
def evaluate(model, loader, criterion, device, task_spec, amp_enabled, channels_last):
    model.eval()
    totals = defaultdict(float)
    sample_count = 0
    counts = _new_counts(task_spec)
    for host_batch in loader:
        batch = move_batch_to_device(host_batch, device)
        if channels_last:
            batch["image"] = batch["image"].contiguous(
                memory_format=torch.channels_last
            )
        with torch.cuda.amp.autocast(enabled=amp_enabled):
            outputs, losses = twinlite_training_step(model, batch, criterion)
        batch_size = int(batch["image"].shape[0])
        sample_count += batch_size
        for name, value in losses.items():
            totals[name] += float(value.item()) * batch_size
        _update_counts(counts, outputs, batch, task_spec)
    metrics, head_scores = _finalize_counts(counts, task_spec)
    return {
        "sample_count": sample_count,
        "losses": {
            name: value / max(1, sample_count) for name, value in totals.items()
        },
        "metrics": metrics,
        "head_scores": head_scores,
        "mean_head_iou": sum(head_scores.values()) / len(head_scores),
    }


def train_epoch(
    model, loader, criterion, optimizer, scaler, device, amp_enabled, channels_last
):
    model.train()
    total_loss = 0.0
    sample_count = 0
    for host_batch in loader:
        batch = move_batch_to_device(host_batch, device)
        if channels_last:
            batch["image"] = batch["image"].contiguous(
                memory_format=torch.channels_last
            )
        optimizer.zero_grad(set_to_none=True)
        with torch.cuda.amp.autocast(enabled=amp_enabled):
            _, losses = twinlite_training_step(model, batch, criterion)
        scaler.scale(losses["loss"]).backward()
        scaler.step(optimizer)
        scaler.update()
        batch_size = int(batch["image"].shape[0])
        total_loss += float(losses["loss"].detach().item()) * batch_size
        sample_count += batch_size
    return total_loss / max(1, sample_count)


def _checkpoint_state(
    args, epoch, model, optimizer, scaler, task_spec, upstream_commit, result
):
    return {
        "schema_version": "twinlite-morai-tiny-checkpoint-1.1.0",
        "epoch": epoch,
        "args": vars(args),
        "task_spec": task_spec.as_dict(),
        "upstream_commit": upstream_commit,
        "model_state_dict": model.state_dict(),
        "optimizer_state_dict": optimizer.state_dict(),
        "scaler_state_dict": scaler.state_dict(),
        "result": result,
    }


def _format_metrics(prefix, metrics, task_spec):
    parts = [
        "{} loss={:.6f}".format(prefix, metrics["losses"]["loss"]),
        "DA={:.4f}".format(metrics["head_scores"]["drivable"]),
    ]
    secondary = task_spec.secondary_head
    parts.append("{}={:.4f}".format(secondary, metrics["head_scores"][secondary]))
    if secondary == "road_marking":
        values = metrics["metrics"][secondary]["class_iou"]
        parts.extend(
            "{}={:.4f}".format(name, values[name])
            for name in ("white_lane", "yellow_lane", "stopline")
        )
    return " ".join(parts)


def main():
    args = parse_args()
    if args.max_samples <= 0 or args.epochs <= 0 or args.batch_size <= 0:
        raise ValueError("max-samples, epochs and batch-size must be positive")
    if args.num_workers < 0:
        raise ValueError("num-workers must be non-negative")
    seed_everything(args.seed)
    device = torch.device(args.device)
    if device.type == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA was requested but is not available")
    amp_enabled = device.type == "cuda" and not args.no_amp

    selection_path = (
        Path(args.selection_file).expanduser().resolve()
        if args.selection_file
        else None
    )
    dataset = MoraiTwinLiteDataset(
        data_root=args.data_root,
        dataset_version=args.dataset_version,
        split=args.split,
        verify_files=False if selection_path else True,
        max_samples=None if selection_path else args.max_samples,
    )
    selection = None
    if selection_path:
        selection, sample_ids = _apply_selection(
            dataset, selection_path, args.dataset_version, args.split
        )
    else:
        sample_ids = [str(row["sample_id"]) for row in dataset.rows]
    if len(dataset) < args.batch_size:
        raise ValueError("tiny dataset must contain at least one complete batch")
    task_spec = dataset.task_spec
    if task_spec != ROAD_MARKING_TASK and args.road_marking_class_weights is not None:
        raise ValueError("road-marking class weights require a road-marking task")

    evaluation_loader = DataLoader(
        dataset,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.num_workers,
        pin_memory=device.type == "cuda",
    )
    generator = torch.Generator().manual_seed(args.seed)
    training_loader = DataLoader(
        dataset,
        batch_size=args.batch_size,
        shuffle=True,
        generator=generator,
        num_workers=args.num_workers,
        pin_memory=device.type == "cuda",
    )

    model, upstream_commit = load_external_twinlite(
        config=args.config, root=args.twinlite_root
    )
    reset_modules = configure_twinlite_task(model, task_spec)
    initialization = {"mode": "random", "reset_modules": list(reset_modules)}
    if args.init_checkpoint:
        init_path = Path(args.init_checkpoint).expanduser().resolve()
        checkpoint = torch.load(init_path, map_location="cpu")
        if checkpoint.get("upstream_commit") != upstream_commit:
            raise RuntimeError("initialization checkpoint uses another upstream commit")
        use_ema = args.init_weights == "ema" or (
            args.init_weights == "auto" and checkpoint.get("ema_state_dict") is not None
        )
        state_key = "ema_state_dict" if use_ema else "model_state_dict"
        if checkpoint.get(state_key) is None:
            raise RuntimeError("initialization checkpoint has no {}".format(state_key))
        source_task = task_from_checkpoint(checkpoint)
        transfer = load_task_compatible_state_dict(
            model,
            checkpoint[state_key],
            source_task,
            task_spec,
            reset_modules,
        )
        initialization = {
            "mode": "checkpoint_weights_only",
            "path": str(init_path),
            "sha256": _sha256(init_path),
            "weights": state_key,
            "source_task": source_task.as_dict(),
            "target_task": task_spec.as_dict(),
            **transfer,
        }

    model = model.to(device)
    if args.channels_last:
        model = model.to(memory_format=torch.channels_last)
    criterion = MaskedTwinLiteLoss(
        task_spec=task_spec,
        road_marking_class_weights=args.road_marking_class_weights,
    ).to(device)
    optimizer = torch.optim.AdamW(
        model.parameters(), lr=args.lr, weight_decay=args.weight_decay
    )
    scaler = torch.cuda.amp.GradScaler(enabled=amp_enabled)

    output_dir = Path(args.output_dir).expanduser().resolve()
    if output_dir.exists():
        raise FileExistsError("tiny-overfit output already exists: {}".format(output_dir))
    output_dir.mkdir(parents=True)
    config = {
        **vars(args),
        "created_at_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "task_spec": task_spec.as_dict(),
        "sample_ids": sample_ids,
        "selection": selection,
        "selection_sha256": _sha256(selection_path) if selection_path else None,
        "initialization": initialization,
        "upstream_commit": upstream_commit,
        "model_parameters": sum(parameter.numel() for parameter in model.parameters()),
        "device_resolved": str(device),
        "gpu": torch.cuda.get_device_name(device) if device.type == "cuda" else None,
        "amp": amp_enabled,
    }
    _write_json(output_dir / "config.json", config)

    started = time.perf_counter()
    before = evaluate(
        model,
        evaluation_loader,
        criterion,
        device,
        task_spec,
        amp_enabled,
        args.channels_last,
    )
    history = []
    print(_format_metrics("before", before, task_spec), flush=True)
    for epoch in range(1, args.epochs + 1):
        train_loss = train_epoch(
            model,
            training_loader,
            criterion,
            optimizer,
            scaler,
            device,
            amp_enabled,
            args.channels_last,
        )
        metrics = evaluate(
            model,
            evaluation_loader,
            criterion,
            device,
            task_spec,
            amp_enabled,
            args.channels_last,
        )
        row = {"epoch": epoch, "train_loss": train_loss, "evaluation": metrics}
        history.append(row)
        progress = {
            "kind": "tiny-overfit-progress",
            "epoch": epoch,
            "task_spec": task_spec.as_dict(),
            "before": before,
            "latest": metrics,
            "history": history,
        }
        _write_json(output_dir / "progress.json", progress)
        _atomic_torch_save(
            _checkpoint_state(
                args,
                epoch,
                model,
                optimizer,
                scaler,
                task_spec,
                upstream_commit,
                progress,
            ),
            output_dir / "latest.pt",
        )
        print(
            _format_metrics("epoch={:03d}".format(epoch), metrics, task_spec)
            + " train={:.6f}".format(train_loss),
            flush=True,
        )

    after = history[-1]["evaluation"]
    elapsed = time.perf_counter() - started
    loss_decreased = after["losses"]["loss"] < before["losses"]["loss"]
    secondary_improved = (
        after["head_scores"][task_spec.secondary_head]
        > before["head_scores"][task_spec.secondary_head]
    )
    foreground_coverage = {
        name: before["metrics"][task_spec.secondary_head]["class_target_pixels"][name]
        for name in task_spec.class_names[task_spec.secondary_head][1:]
    }
    all_foreground_present = all(value > 0 for value in foreground_coverage.values())
    result = {
        "kind": "tiny-overfit-smoke",
        "completed_at_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "training_performed": True,
        "dataset_version": args.dataset_version,
        "split": args.split,
        "sample_count": len(dataset),
        "sample_ids": sample_ids,
        "task_spec": task_spec.as_dict(),
        "config": args.config,
        "upstream_commit": upstream_commit,
        "device": str(device),
        "gpu": torch.cuda.get_device_name(device) if device.type == "cuda" else None,
        "amp": amp_enabled,
        "epochs": args.epochs,
        "batch_size": args.batch_size,
        "learning_rate": args.lr,
        "class_weights": args.road_marking_class_weights,
        "initialization": initialization,
        "foreground_target_pixels": foreground_coverage,
        "all_foreground_classes_present": all_foreground_present,
        "before": before,
        "after": after,
        "history": history,
        "elapsed_seconds": elapsed,
        "loss_decreased": loss_decreased,
        "secondary_score_improved": secondary_improved,
        "overfit_contract_passed": (
            loss_decreased and secondary_improved and all_foreground_present
        ),
    }
    _write_json(output_dir / "metrics.json", result)
    _atomic_torch_save(
        _checkpoint_state(
            args,
            args.epochs,
            model,
            optimizer,
            scaler,
            task_spec,
            upstream_commit,
            result,
        ),
        output_dir / "checkpoint.pt",
    )
    print(
        json.dumps(
            {
                "completed_at_utc": result["completed_at_utc"],
                "elapsed_seconds": result["elapsed_seconds"],
                "loss_decreased": result["loss_decreased"],
                "overfit_contract_passed": result["overfit_contract_passed"],
                "after_head_scores": result["after"]["head_scores"],
                "after_class_iou": result["after"]["metrics"]
                [task_spec.secondary_head]["class_iou"],
                "output_dir": str(output_dir),
            },
            ensure_ascii=False,
            sort_keys=True,
        ),
        flush=True,
    )
    if not result["overfit_contract_passed"]:
        raise RuntimeError("tiny-overfit contract did not pass")


if __name__ == "__main__":
    main()
