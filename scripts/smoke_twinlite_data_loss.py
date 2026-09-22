#!/usr/bin/env python3
"""Load real MORAI samples and verify the two-head loss can backpropagate."""

import argparse
import os
import sys
from pathlib import Path

import torch
from torch.utils.data import DataLoader


REPO_ROOT = Path(__file__).resolve().parents[1]
PERCEPTION_SRC = REPO_ROOT / "src" / "perception"
sys.path.insert(0, str(PERCEPTION_SRC))

from twinlite_morai import (  # noqa: E402
    MaskedTwinLiteLoss,
    MoraiTwinLiteDataset,
    move_batch_to_device,
    twinlite_training_step,
)


class SmokeTwin(torch.nn.Module):
    """Minimal two-head network used only to test the data/loss plumbing.

    Secondary head width follows the dataset task: 2 for binary lane,
    4 for the road-marking task.
    """

    def __init__(self, secondary_classes: int = 2):
        super().__init__()
        self.stem = torch.nn.Conv2d(3, 8, kernel_size=3, padding=1)
        self.drivable = torch.nn.Conv2d(8, 2, kernel_size=1)
        self.secondary = torch.nn.Conv2d(8, secondary_classes, kernel_size=1)

    def forward(self, image):
        features = torch.relu(self.stem(image))
        return self.drivable(features), self.secondary(features)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", default=os.environ.get("VIP3_DATA", "/data"))
    parser.add_argument("--dataset-version", default="vip3_twinlite_katri_v1")
    parser.add_argument("--split", default="train", choices=("train", "val"))
    parser.add_argument("--batch-size", type=int, default=2)
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    return parser.parse_args()


def main():
    args = parse_args()
    dataset = MoraiTwinLiteDataset(
        data_root=args.data_root,
        dataset_version=args.dataset_version,
        split=args.split,
        max_samples=max(1, args.batch_size),
    )
    host_batch = next(
        iter(DataLoader(dataset, batch_size=args.batch_size, num_workers=0))
    )
    device = torch.device(args.device)
    task_spec = dataset.task_spec
    secondary = task_spec.secondary_head
    model = SmokeTwin(task_spec.class_counts[secondary]).to(device)
    # move_batch_to_device 는 valid_masks(dict)까지 함께 옮긴다. 개별 필드만 옮기면
    # training.py 가 CPU 에 남은 valid_masks 를 우선 선택해 --device cuda 에서 죽는다.
    batch = move_batch_to_device(host_batch, device)
    criterion = MaskedTwinLiteLoss(task_spec=task_spec).to(device)
    outputs, losses = twinlite_training_step(model, batch, criterion)
    losses["loss"].backward()
    print(
        "OK device={} task={} samples={} image={} drivable={} {}={} valid={} loss={:.6f}".format(
            device,
            task_spec.name,
            len(dataset),
            tuple(batch["image"].shape),
            tuple(outputs["drivable"].shape),
            secondary,
            tuple(outputs[secondary].shape),
            int(batch["valid_mask"].sum().item()),
            float(losses["loss"].detach().cpu()),
        )
    )


if __name__ == "__main__":
    main()
