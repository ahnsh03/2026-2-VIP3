#!/usr/bin/env python3
"""Train the pinned TwinLiteNet+ baseline on MORAI version manifests."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
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
    TwinLiteTaskSpec,
    configure_twinlite_task,
    load_task_compatible_state_dict,
    load_external_twinlite,
    move_batch_to_device,
    task_from_checkpoint,
    twinlite_training_step,
)


# 학습/집계 대상 뷰. VIP3는 front/left/right/rear 4대를 공유 가중치로 학습한다.
# 카메라를 추가하면 여기 한 곳만 고치면 된다(config/vip3_topics.yaml 과 동기화할 것).
VIEWS = ("all", "front", "left", "right", "rear")


def _default_run_name() -> str:
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return f"twinlite_medium_katri_{timestamp}"


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", default=os.environ.get("VIP3_DATA", "/data"))
    parser.add_argument("--dataset-version", default="vip3_katri_parking_v1")
    parser.add_argument("--twinlite-root", default=os.environ.get("TWINLITE_ROOT"))
    parser.add_argument("--config", default="medium", choices=("nano", "small", "medium", "large"))
    parser.add_argument("--epochs", type=int, default=30)
    parser.add_argument("--batch-size", type=int, default=12)
    parser.add_argument("--num-workers", type=int, default=6)
    parser.add_argument("--lr", type=float, default=5e-4)
    parser.add_argument("--weight-decay", type=float, default=5e-4)
    parser.add_argument("--poly-power", type=float, default=1.5)
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--run-name", default=_default_run_name())
    parser.add_argument(
        "--output-root",
        default=str(REPO_ROOT / "artifacts/perception_eval/runs"),
    )
    parser.add_argument("--resume", help="Path to latest.pt from the same run")
    parser.add_argument(
        "--init-checkpoint",
        help="Load model weights only for a new run; optimizer/scheduler/EMA are reset",
    )
    parser.add_argument(
        "--init-weights",
        default="auto",
        choices=("auto", "ema", "model"),
        help="Weight state to load from --init-checkpoint",
    )
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--no-amp", action="store_true")
    parser.add_argument("--no-ema", action="store_true")
    parser.add_argument("--ema-decay", type=float, default=0.9999)
    parser.add_argument("--max-train-samples", type=int)
    parser.add_argument("--max-val-samples", type=int)
    parser.add_argument("--skip-file-check", action="store_true")
    parser.add_argument("--channels-last", action="store_true")
    parser.add_argument("--log-interval", type=int, default=50)
    parser.add_argument(
        "--road-marking-class-weights",
        type=float,
        nargs=4,
        metavar=("BACKGROUND", "WHITE", "YELLOW", "STOPLINE"),
        help=(
            "Optional positive loss weights for the four road-marking classes; "
            "omit for uniform weighting"
        ),
    )
    parser.add_argument(
        "--preflight-only",
        action="store_true",
        help="Load data/model/checkpoint and run one no-grad batch per split, then exit",
    )
    return parser.parse_args()


def seed_everything(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)


class ModelEMA:
    def __init__(self, model: torch.nn.Module, decay: float = 0.9999) -> None:
        self.model = copy.deepcopy(model).eval()
        self.decay = float(decay)
        self.updates = 0
        for parameter in self.model.parameters():
            parameter.requires_grad_(False)

    @torch.no_grad()
    def update(self, model: torch.nn.Module) -> None:
        self.updates += 1
        decay = self.decay * (1.0 - math.exp(-self.updates / 2000.0))
        source = model.state_dict()
        for name, value in self.model.state_dict().items():
            if value.dtype.is_floating_point:
                value.mul_(decay).add_(source[name].detach(), alpha=1.0 - decay)
            else:
                value.copy_(source[name])


def _atomic_torch_save(value: Dict[str, Any], path: Path) -> None:
    temporary = path.with_suffix(path.suffix + ".part")
    torch.save(value, temporary)
    os.replace(temporary, path)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _append_jsonl(path: Path, row: Dict[str, Any]) -> None:
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(row, ensure_ascii=False, sort_keys=True) + "\n")


def _new_confusion(task_spec: TwinLiteTaskSpec):
    return {
        head: {
            view: {
                "intersection": [0] * task_spec.class_counts[head],
                "union": [0] * task_spec.class_counts[head],
                "foreground": [0, 0],
            }
            for view in VIEWS
        }
        for head in task_spec.heads
    }


def _update_confusion(confusion, outputs, batch, task_spec: TwinLiteTaskSpec):
    views = batch["meta"]["view"]
    for head in task_spec.heads:
        valid = batch.get("valid_masks", {}).get(head, batch["valid_mask"]).bool()
        logits = outputs[head]
        prediction = (
            logits.argmax(dim=1)
            if logits.shape[1] > 1
            else (logits[:, 0] > 0).long()
        ).long()
        target = batch["targets"][head].long()
        for index, view in enumerate(views):
            view = str(view)
            selected_valid = valid[index]
            for class_index in range(task_spec.class_counts[head]):
                predicted_class = (prediction[index] == class_index) & selected_valid
                target_class = (target[index] == class_index) & selected_valid
                intersection = int((predicted_class & target_class).sum().item())
                union = int((predicted_class | target_class).sum().item())
                for group in ("all", view):
                    confusion[head][group]["intersection"][class_index] += intersection
                    confusion[head][group]["union"][class_index] += union

            predicted_foreground = (prediction[index] > 0) & selected_valid
            target_foreground = (target[index] > 0) & selected_valid
            foreground_intersection = int(
                (predicted_foreground & target_foreground).sum().item()
            )
            foreground_union = int(
                (predicted_foreground | target_foreground).sum().item()
            )
            for group in ("all", view):
                confusion[head][group]["foreground"][0] += foreground_intersection
                confusion[head][group]["foreground"][1] += foreground_union


def _confusion_to_metrics(confusion, task_spec: TwinLiteTaskSpec):
    result = {}
    for head in task_spec.heads:
        result[head] = {}
        class_names = task_spec.class_names[head]
        for view in VIEWS:
            entry = confusion[head][view]
            intersections = entry["intersection"]
            unions = entry["union"]
            class_iou = {
                name: intersections[index] / max(1, unions[index])
                for index, name in enumerate(class_names)
            }
            foreground_scores = [
                intersections[index] / unions[index]
                for index in range(1, len(class_names))
                if unions[index] > 0
            ]
            foreground_intersection, foreground_union = entry["foreground"]
            view_metrics = {
                "class_intersection": dict(zip(class_names, intersections)),
                "class_union": dict(zip(class_names, unions)),
                "class_iou": class_iou,
                "present_foreground_classes": [
                    class_names[index]
                    for index in range(1, len(class_names))
                    if unions[index] > 0
                ],
                "mean_foreground_iou": (
                    sum(foreground_scores) / len(foreground_scores)
                    if foreground_scores
                    else 0.0
                ),
                "foreground_intersection": foreground_intersection,
                "foreground_union": foreground_union,
                "foreground_union_iou": foreground_intersection
                / max(1, foreground_union),
            }
            if len(class_names) == 2:
                # Retain the legacy scalar fields consumed by existing reports.
                view_metrics.update(
                    {
                        "intersection": intersections[1],
                        "union": unions[1],
                        "iou": class_iou[class_names[1]],
                    }
                )
            result[head][view] = view_metrics
    return result


def _head_score(metrics, task_spec: TwinLiteTaskSpec, head: str) -> float:
    view_metrics = metrics[head]["all"]
    if task_spec.class_counts[head] == 2:
        return float(view_metrics["iou"])
    return float(view_metrics["mean_foreground_iou"])


def _prepare_batch(host_batch, device, channels_last):
    batch = move_batch_to_device(host_batch, device)
    if channels_last:
        batch["image"] = batch["image"].contiguous(
            memory_format=torch.channels_last
        )
    return batch


@torch.no_grad()
def validate(
    model,
    loader,
    criterion,
    task_spec,
    device,
    amp_enabled,
    channels_last=False,
    log_interval=0,
):
    model.eval()
    confusion = _new_confusion(task_spec)
    totals = defaultdict(float)
    sample_count = 0
    started = time.perf_counter()
    for batch_index, host_batch in enumerate(loader, 1):
        batch = _prepare_batch(host_batch, device, channels_last)
        with torch.cuda.amp.autocast(enabled=amp_enabled):
            outputs, losses = twinlite_training_step(model, batch, criterion)
        batch_size = int(batch["image"].shape[0])
        sample_count += batch_size
        for name, value in losses.items():
            totals[name] += float(value.item()) * batch_size
        _update_confusion(confusion, outputs, batch, task_spec)
        if log_interval > 0 and (
            batch_index % log_interval == 0 or batch_index == len(loader)
        ):
            elapsed = time.perf_counter() - started
            print(
                f"val batch={batch_index:04d}/{len(loader):04d} "
                f"samples={sample_count} sec={elapsed:.1f}",
                flush=True,
            )
    averaged_losses = {
        name: value / max(1, sample_count) for name, value in totals.items()
    }
    metrics = _confusion_to_metrics(confusion, task_spec)
    head_scores = {
        head: _head_score(metrics, task_spec, head) for head in task_spec.heads
    }
    mean_iou = sum(head_scores.values()) / len(head_scores)
    return {
        "sample_count": sample_count,
        "losses": averaged_losses,
        "metrics": metrics,
        "head_scores": head_scores,
        "mean_head_iou": mean_iou,
    }


def train_epoch(
    model,
    loader,
    criterion,
    optimizer,
    scaler,
    ema,
    device,
    amp_enabled,
    epoch,
    total_epochs,
    channels_last=False,
    log_interval=0,
):
    model.train()
    totals = defaultdict(float)
    sample_count = 0
    started = time.perf_counter()
    for batch_index, host_batch in enumerate(loader, 1):
        batch = _prepare_batch(host_batch, device, channels_last)
        optimizer.zero_grad(set_to_none=True)
        with torch.cuda.amp.autocast(enabled=amp_enabled):
            _, losses = twinlite_training_step(model, batch, criterion)
        scaler.scale(losses["loss"]).backward()
        scaler.step(optimizer)
        scaler.update()
        if ema is not None:
            ema.update(model)
        batch_size = int(batch["image"].shape[0])
        sample_count += batch_size
        for name, value in losses.items():
            totals[name] += float(value.detach().item()) * batch_size
        if log_interval > 0 and (
            batch_index % log_interval == 0 or batch_index == len(loader)
        ):
            elapsed = time.perf_counter() - started
            average_loss = totals["loss"] / max(1, sample_count)
            print(
                f"train epoch={epoch:03d}/{total_epochs} "
                f"batch={batch_index:04d}/{len(loader):04d} "
                f"samples={sample_count} loss={average_loss:.5f} sec={elapsed:.1f}",
                flush=True,
            )
    return {name: value / max(1, sample_count) for name, value in totals.items()}


def _checkpoint_state(
    args,
    epoch,
    model,
    ema,
    optimizer,
    scheduler,
    scaler,
    upstream_commit,
    best,
    validation,
    initialization,
    dataset_provenance,
    task_spec,
):
    return {
        "schema_version": "twinlite-morai-checkpoint-1.1.0",
        "epoch": epoch,
        "args": vars(args),
        "task_spec": task_spec.as_dict(),
        "upstream_commit": upstream_commit,
        "model_state_dict": model.state_dict(),
        "ema_state_dict": ema.model.state_dict() if ema is not None else None,
        "ema_updates": ema.updates if ema is not None else 0,
        "optimizer_state_dict": optimizer.state_dict(),
        "scheduler_state_dict": scheduler.state_dict(),
        "scaler_state_dict": scaler.state_dict(),
        "best": best,
        "validation": validation,
        "initialization": initialization,
        "dataset_provenance": dataset_provenance,
    }


@torch.no_grad()
def _preflight_split(model, loader, criterion, device, amp_enabled, channels_last):
    model.eval()
    host_batch = next(iter(loader))
    batch = _prepare_batch(host_batch, device, channels_last)
    with torch.cuda.amp.autocast(enabled=amp_enabled):
        outputs, losses = twinlite_training_step(model, batch, criterion)
    for name, value in losses.items():
        if not torch.isfinite(value):
            raise RuntimeError(f"non-finite preflight loss {name}: {value}")
    return {
        "sample_count": int(batch["image"].shape[0]),
        "image_shape": list(batch["image"].shape),
        "output_shapes": {
            head: list(output.shape) for head, output in outputs.items()
        },
        "losses": {name: float(value.item()) for name, value in losses.items()},
        "run_ids": sorted({str(value) for value in host_batch["meta"]["run_id"]}),
        "views": sorted({str(value) for value in host_batch["meta"]["view"]}),
    }


def main():
    args = parse_args()
    if (
        args.epochs <= 0
        or args.batch_size <= 0
        or args.num_workers < 0
        or args.log_interval < 0
    ):
        raise ValueError("epochs/batch-size must be positive and num-workers non-negative")
    if args.resume and args.init_checkpoint:
        raise ValueError("--resume and --init-checkpoint are mutually exclusive")
    seed_everything(args.seed)
    device = torch.device(args.device)
    if device.type == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA was requested but is not available")
    amp_enabled = device.type == "cuda" and not args.no_amp
    if device.type == "cuda":
        torch.backends.cudnn.benchmark = True

    output_dir = Path(args.output_root).expanduser().resolve() / args.run_name
    if output_dir.exists() and not args.resume:
        raise FileExistsError(
            f"output run already exists; choose a new --run-name: {output_dir}"
        )
    output_dir.mkdir(parents=True, exist_ok=True)
    history_path = output_dir / "history.jsonl"

    data_root = Path(args.data_root).expanduser().resolve()
    dataset_root = data_root / "dataset_versions" / args.dataset_version
    dataset_metadata_path = dataset_root / "dataset.json"
    dataset_provenance = {
        "dataset_version": args.dataset_version,
        "dataset_metadata_sha256": _sha256(dataset_metadata_path),
        "split_manifest_sha256": {
            split: _sha256(dataset_root / (split + ".jsonl"))
            for split in ("train", "val")
        },
    }

    train_dataset = MoraiTwinLiteDataset(
        data_root=args.data_root,
        dataset_version=args.dataset_version,
        split="train",
        verify_files=not args.skip_file_check,
        max_samples=args.max_train_samples,
    )
    val_dataset = MoraiTwinLiteDataset(
        data_root=args.data_root,
        dataset_version=args.dataset_version,
        split="val",
        verify_files=not args.skip_file_check,
        max_samples=args.max_val_samples,
    )
    if train_dataset.task_spec != val_dataset.task_spec:
        raise RuntimeError(
            "train/val task mismatch: "
            f"{train_dataset.task_spec.name} != {val_dataset.task_spec.name}"
        )
    task_spec = train_dataset.task_spec
    if (
        task_spec != ROAD_MARKING_TASK
        and args.road_marking_class_weights is not None
    ):
        raise ValueError(
            "--road-marking-class-weights requires a road-marking dataset task"
        )
    generator = torch.Generator().manual_seed(args.seed)
    loader_options = {
        "batch_size": args.batch_size,
        "num_workers": args.num_workers,
        "pin_memory": device.type == "cuda",
        "persistent_workers": args.num_workers > 0,
    }
    train_loader = DataLoader(
        train_dataset, shuffle=True, generator=generator, **loader_options
    )
    val_loader = DataLoader(val_dataset, shuffle=False, **loader_options)

    model, upstream_commit = load_external_twinlite(
        config=args.config, root=args.twinlite_root
    )
    reset_modules = configure_twinlite_task(model, task_spec)
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
    scheduler = torch.optim.lr_scheduler.LambdaLR(
        optimizer,
        lr_lambda=lambda epoch: max(0.0, 1.0 - epoch / args.epochs)
        ** args.poly_power,
    )
    scaler = torch.cuda.amp.GradScaler(enabled=amp_enabled)
    ema = None if args.no_ema else ModelEMA(model, decay=args.ema_decay)
    start_epoch = 1
    best = {"mean": -1.0, **{head: -1.0 for head in task_spec.heads}}
    initialization = {
        "mode": "random",
        "task": task_spec.as_dict(),
        "configured_reset_modules": list(reset_modules),
    }

    if args.resume:
        checkpoint = torch.load(args.resume, map_location=device)
        if checkpoint["upstream_commit"] != upstream_commit:
            raise RuntimeError("resume checkpoint uses a different upstream commit")
        checkpoint_task = task_from_checkpoint(checkpoint)
        if checkpoint_task != task_spec:
            raise RuntimeError(
                "resume checkpoint task differs from the dataset task: "
                f"{checkpoint_task.name} != {task_spec.name}"
            )
        model.load_state_dict(checkpoint["model_state_dict"])
        optimizer.load_state_dict(checkpoint["optimizer_state_dict"])
        scheduler.load_state_dict(checkpoint["scheduler_state_dict"])
        scaler.load_state_dict(checkpoint["scaler_state_dict"])
        best.update(checkpoint.get("best", {}))
        if ema is not None and checkpoint.get("ema_state_dict") is not None:
            ema.model.load_state_dict(checkpoint["ema_state_dict"])
            ema.updates = int(checkpoint.get("ema_updates", 0))
        start_epoch = int(checkpoint["epoch"]) + 1
        initialization = checkpoint.get(
            "initialization",
            {"mode": "resumed_legacy_checkpoint", "path": str(args.resume)},
        )
    elif args.init_checkpoint:
        initialization_path = Path(args.init_checkpoint).expanduser().resolve()
        checkpoint = torch.load(initialization_path, map_location="cpu")
        if checkpoint.get("upstream_commit") != upstream_commit:
            raise RuntimeError(
                "initialization checkpoint uses a different upstream commit"
            )
        use_ema = args.init_weights == "ema" or (
            args.init_weights == "auto"
            and checkpoint.get("ema_state_dict") is not None
        )
        if use_ema and checkpoint.get("ema_state_dict") is None:
            raise RuntimeError("initialization checkpoint has no EMA weights")
        state_key = "ema_state_dict" if use_ema else "model_state_dict"
        if checkpoint.get(state_key) is None:
            raise RuntimeError(
                f"initialization checkpoint has no {state_key}"
            )
        source_task = task_from_checkpoint(checkpoint)
        transfer = load_task_compatible_state_dict(
            model,
            checkpoint[state_key],
            source_task,
            task_spec,
            reset_modules,
        )
        if ema is not None:
            ema.model.load_state_dict(model.state_dict())
            ema.updates = 0
        source_args = checkpoint.get("args") or {}
        initialization = {
            "mode": "checkpoint_weights_only",
            "path": str(initialization_path),
            "sha256": _sha256(initialization_path),
            "checkpoint_epoch": checkpoint.get("epoch"),
            "weights": state_key,
            "source_dataset_version": source_args.get("dataset_version"),
            "source_task": source_task.as_dict(),
            "target_task": task_spec.as_dict(),
            **transfer,
            "optimizer_reset": True,
            "scheduler_reset": True,
            "ema_reset_from_loaded_model": ema is not None,
        }

    run_config = {
        **vars(args),
        "output_dir": str(output_dir),
        "device_resolved": str(device),
        "gpu": torch.cuda.get_device_name(device) if device.type == "cuda" else None,
        "torch_version": torch.__version__,
        "upstream_commit": upstream_commit,
        "model_parameters": sum(parameter.numel() for parameter in model.parameters()),
        "task_spec": task_spec.as_dict(),
        "train_samples": len(train_dataset),
        "val_samples": len(val_dataset),
        "spatial_weighting": "uniform within valid_mask; padding weight zero",
        "augmentation": "none (first MORAI baseline)",
        "initialization": initialization,
        "dataset_provenance": dataset_provenance,
    }
    with (output_dir / "config.json").open("w", encoding="utf-8") as stream:
        json.dump(run_config, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")

    print(json.dumps(run_config, ensure_ascii=False, sort_keys=True), flush=True)
    if args.preflight_only:
        report = {
            "schema_version": "twinlite-training-preflight-1.0.0",
            "completed_at_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "training_performed": False,
            "initialization": initialization,
            "task_spec": task_spec.as_dict(),
            "dataset_provenance": dataset_provenance,
            "train": _preflight_split(
                model,
                train_loader,
                criterion,
                device,
                amp_enabled,
                args.channels_last,
            ),
            "val": _preflight_split(
                model,
                val_loader,
                criterion,
                device,
                amp_enabled,
                args.channels_last,
            ),
        }
        with (output_dir / "preflight.json").open("w", encoding="utf-8") as stream:
            json.dump(report, stream, ensure_ascii=False, indent=2, sort_keys=True)
            stream.write("\n")
        print(json.dumps(report, ensure_ascii=False, sort_keys=True), flush=True)
        return
    if device.type == "cuda":
        torch.cuda.reset_peak_memory_stats(device)
    started = time.perf_counter()
    for epoch in range(start_epoch, args.epochs + 1):
        epoch_started = time.perf_counter()
        epoch_lr = optimizer.param_groups[0]["lr"]
        train_losses = train_epoch(
            model,
            train_loader,
            criterion,
            optimizer,
            scaler,
            ema,
            device,
            amp_enabled,
            epoch,
            args.epochs,
            channels_last=args.channels_last,
            log_interval=args.log_interval,
        )
        evaluation_model = ema.model if ema is not None else model
        validation = validate(
            evaluation_model,
            val_loader,
            criterion,
            task_spec,
            device,
            amp_enabled,
            channels_last=args.channels_last,
            log_interval=args.log_interval,
        )
        scheduler.step()
        head_scores = validation["head_scores"]
        da_iou = head_scores["drivable"]
        secondary_iou = head_scores[task_spec.secondary_head]
        mean_iou = validation["mean_head_iou"]
        row = {
            "epoch": epoch,
            "lr": epoch_lr,
            "train_losses": train_losses,
            "validation": validation,
            "epoch_seconds": time.perf_counter() - epoch_started,
        }
        _append_jsonl(history_path, row)
        candidates = {
            "mean": mean_iou,
            **head_scores,
        }
        improved = []
        for name, score in candidates.items():
            if score > best[name]:
                best[name] = score
                improved.append(name)
        state = _checkpoint_state(
            args,
            epoch,
            model,
            ema,
            optimizer,
            scheduler,
            scaler,
            upstream_commit,
            best,
            validation,
            initialization,
            dataset_provenance,
            task_spec,
        )
        _atomic_torch_save(state, output_dir / "latest.pt")
        for name in improved:
            _atomic_torch_save(state, output_dir / f"best_{name}.pt")
        print(
            f"epoch={epoch:03d}/{args.epochs} lr={row['lr']:.7f} "
            f"train={train_losses['loss']:.5f} val={validation['losses']['loss']:.5f} "
            f"DA={da_iou:.4f} {task_spec.secondary_head}={secondary_iou:.4f} "
            f"mean={mean_iou:.4f} "
            f"sec={row['epoch_seconds']:.1f}",
            flush=True,
        )

    result = {
        "completed_at_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "epochs_completed": args.epochs,
        "elapsed_seconds": time.perf_counter() - started,
        "best": best,
        "peak_cuda_memory_bytes": torch.cuda.max_memory_allocated(device)
        if device.type == "cuda"
        else 0,
        "output_dir": str(output_dir),
    }
    with (output_dir / "result.json").open("w", encoding="utf-8") as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps(result, ensure_ascii=False, sort_keys=True), flush=True)


if __name__ == "__main__":
    main()
