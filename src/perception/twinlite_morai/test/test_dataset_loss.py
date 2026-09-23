import json
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np
import torch
from torch.utils.data import DataLoader

from twinlite_morai.dataset import DEFAULT_DATASET_VERSION, MoraiTwinLiteDataset
from twinlite_morai.losses import MaskedTwinLiteLoss, adapt_twinlite_outputs
from twinlite_morai.tasks import ROAD_MARKING_TASK
from twinlite_morai.training import twinlite_training_step


# fixture 경로 이름. dataset 이름은 코드 기본값에서 끌어오고, cache 이름은
# data_collection 파이프라인(mask_baker.DEFAULT_OUTPUT_NAME)과 같은 문자열을 쓴다.
# 여기에 다른 이름을 적어 두면 테스트를 보고 따라 하는 사람이 엉뚱한 경로를 만든다.
DATASET_NAME = DEFAULT_DATASET_VERSION
CACHE_NAME = "vip3_twinlite_384x640_v1"


def write_png(path, image):
    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), image):
        raise RuntimeError("failed to write test PNG")


class TinyTwin(torch.nn.Module):
    def __init__(self):
        super().__init__()
        self.drivable = torch.nn.Conv2d(3, 2, 1)
        self.lane = torch.nn.Conv2d(3, 2, 1)

    def forward(self, image):
        return self.drivable(image), self.lane(image)


class TinyRoadMarkingTwin(torch.nn.Module):
    def __init__(self):
        super().__init__()
        self.drivable = torch.nn.Conv2d(3, 2, 1)
        self.road_marking = torch.nn.Conv2d(3, 4, 1)

    def forward(self, image):
        return self.drivable(image), self.road_marking(image)


class DatasetAndLossTest(unittest.TestCase):
    def make_dataset(self, root, view="front"):
        data_root = root / "data"
        version_root = data_root / "dataset_versions" / DATASET_NAME
        image_rel = f"datasets/run_001/frames/intensity/{view}/000000.png"
        image = np.zeros((6, 10, 4), dtype=np.uint8)
        image[:, :, :3] = (10, 20, 30)
        image[:, :, 3] = 255
        write_png(data_root / image_rel, image)

        target_hw = [8, 10]
        target_paths = {}
        for name in ("lane", "drivable"):
            relative = (
                f"datasets/run_001/derived/{CACHE_NAME}/"
                f"{name}_masks/{view}/000000.png"
            )
            mask = np.zeros(target_hw, dtype=np.uint8)
            mask[3:, 4:6] = 1
            write_png(data_root / relative, mask)
            target_paths[name] = relative
        valid_rel = (
            f"datasets/run_001/derived/{CACHE_NAME}/"
            f"valid_masks/{view}/000000.png"
        )
        valid = np.zeros(target_hw, dtype=np.uint8)
        valid[1:7] = 1
        write_png(data_root / valid_rel, valid)
        drivable_valid_rel = (
            f"datasets/run_001/derived/{CACHE_NAME}/"
            f"valid_masks/drivable/{view}/000000.png"
        )
        drivable_valid = valid.copy()
        drivable_valid[3:5, 4:6] = 0
        write_png(data_root / drivable_valid_rel, drivable_valid)

        geometry = {
            "original_hw": [6, 10],
            "target_hw": target_hw,
            "resized_hw": [6, 10],
            "pad_ltrb": [0, 1, 0, 1],
            "scale": 1.0,
            "scale_xy": [1.0, 1.0],
            "valid_roi_xyxy": [0, 1, 10, 7],
        }
        row = {
            "schema_version": "morai-perception-sample-1.0.0",
            "dataset_version": DATASET_NAME,
            "split": "train",
            "sample_id": f"run_001/{view}/000000",
            "run_id": "run_001",
            "frame_id": 0,
            "view": view,
            "image": image_rel,
            "targets": target_paths,
            "valid_mask": valid_rel,
            "valid_masks": {
                "lane": valid_rel,
                "drivable": drivable_valid_rel,
            },
            "geometry": geometry,
            "task_valid": {"lane": True, "drivable": True},
        }
        version_root.mkdir(parents=True)
        (version_root / "train.jsonl").write_text(
            json.dumps(row) + "\n", encoding="utf-8"
        )
        (version_root / "dataset.json").write_text(
            json.dumps(
                {
                    "schema_version": "morai-perception-dataset-1.0.0",
                    "target_hw": target_hw,
                    "training_heads": ["lane", "drivable"],
                }
            )
            + "\n",
            encoding="utf-8",
        )
        (version_root / "_SUCCESS").write_text("{}\n", encoding="utf-8")
        return data_root

    def test_dataset_applies_geometry_and_returns_named_targets(self):
        with tempfile.TemporaryDirectory() as tmp:
            dataset = MoraiTwinLiteDataset(self.make_dataset(Path(tmp)), split="train")
            sample = dataset[0]
            self.assertEqual(tuple(sample["image"].shape), (3, 8, 10))
            self.assertEqual(sample["image"].dtype, torch.float32)
            self.assertTrue(torch.allclose(sample["image"][:, 0], torch.full((3, 10), 114 / 255.0)))
            self.assertEqual(tuple(sample["targets"]["lane"].shape), (8, 10))
            self.assertEqual(sample["targets"]["lane"].dtype, torch.int64)
            self.assertEqual(sample["valid_mask"].dtype, torch.bool)
            self.assertFalse(bool(sample["valid_mask"][0, 0]))
            self.assertTrue(bool(sample["valid_masks"]["lane"][3, 4]))
            self.assertFalse(bool(sample["valid_masks"]["drivable"][3, 4]))
            self.assertEqual(sample["meta"]["view"], "front")

    def test_rear_view_sample_keeps_its_view_provenance(self):
        # VIP3 는 front/left/right/rear 4-view 를 공유 가중치로 학습한다.
        # rear 는 ASMC 에 없던 뷰라 provenance 회귀를 여기서 막는다.
        with tempfile.TemporaryDirectory() as tmp:
            dataset = MoraiTwinLiteDataset(
                self.make_dataset(Path(tmp), view="rear"), split="train"
            )
            sample = dataset[0]
            self.assertEqual(sample["meta"]["view"], "rear")
            self.assertEqual(sample["meta"]["sample_id"], "run_001/rear/000000")
            self.assertEqual(tuple(sample["image"].shape), (3, 8, 10))

    def test_rear_view_road_marking_batch_reaches_backward(self):
        with tempfile.TemporaryDirectory() as tmp:
            dataset = MoraiTwinLiteDataset(
                self.make_road_marking_dataset(Path(tmp), view="rear"), split="train"
            )
            batch = next(iter(DataLoader(dataset, batch_size=1)))
            self.assertEqual(batch["meta"]["view"], ["rear"])
            outputs, losses = twinlite_training_step(
                TinyRoadMarkingTwin(),
                batch,
                MaskedTwinLiteLoss(task_spec=ROAD_MARKING_TASK),
            )
            self.assertEqual(tuple(outputs["road_marking"].shape), (1, 4, 8, 10))
            losses["loss"].backward()
            self.assertTrue(torch.isfinite(losses["loss"]))

    def test_v11_manifest_ignores_stored_future_road_marking_for_two_head_training(self):
        with tempfile.TemporaryDirectory() as tmp:
            data_root = self.make_dataset(Path(tmp))
            version_root = data_root / "dataset_versions" / DATASET_NAME
            manifest_path = version_root / "train.jsonl"
            row = json.loads(manifest_path.read_text(encoding="utf-8"))
            marking_rel = (
                "datasets/run_001/derived/{CACHE_NAME}/"
                "road_marking_masks/front/000000.png"
            )
            marking = np.zeros((8, 10), dtype=np.uint8)
            marking[2, 2] = 1
            marking[3, 3] = 2
            marking[4, 4] = 3
            write_png(data_root / marking_rel, marking)
            row["schema_version"] = "morai-perception-sample-1.1.0"
            row["targets"]["road_marking"] = marking_rel
            row["valid_masks"]["road_marking"] = row["valid_mask"]
            row["target_encodings"] = {
                "road_marking": {
                    "type": "categorical_uint8",
                    "class_values": {
                        "background": 0,
                        "white_lane": 1,
                        "yellow_lane": 2,
                        "stopline": 3,
                    },
                }
            }
            manifest_path.write_text(json.dumps(row) + "\n", encoding="utf-8")
            metadata_path = version_root / "dataset.json"
            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
            metadata["schema_version"] = "morai-perception-dataset-1.1.0"
            metadata["stored_future_targets"] = ["road_marking", "stopline"]
            metadata_path.write_text(json.dumps(metadata) + "\n", encoding="utf-8")

            sample = MoraiTwinLiteDataset(data_root, split="train")[0]
            self.assertEqual(set(sample["targets"]), {"lane", "drivable"})

    def make_road_marking_dataset(self, root, view="front"):
        data_root = self.make_dataset(root, view=view)
        version_root = data_root / "dataset_versions" / DATASET_NAME
        manifest_path = version_root / "train.jsonl"
        row = json.loads(manifest_path.read_text(encoding="utf-8"))
        marking_rel = (
            f"datasets/run_001/derived/{CACHE_NAME}/"
            f"road_marking_masks/{view}/000000.png"
        )
        marking = np.zeros((8, 10), dtype=np.uint8)
        marking[2, 2:4] = 1
        marking[3, 3:5] = 2
        marking[4, 4:7] = 3
        write_png(data_root / marking_rel, marking)
        row["schema_version"] = "morai-perception-sample-1.1.0"
        row["targets"]["road_marking"] = marking_rel
        row["valid_masks"]["road_marking"] = row["valid_mask"]
        row["task_valid"]["road_marking"] = True
        row["target_encodings"] = {
            "road_marking": {
                "type": "categorical_uint8",
                "class_values": {
                    "background": 0,
                    "white_lane": 1,
                    "yellow_lane": 2,
                    "stopline": 3,
                },
            }
        }
        manifest_path.write_text(json.dumps(row) + "\n", encoding="utf-8")
        metadata_path = version_root / "dataset.json"
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        metadata["schema_version"] = "morai-perception-dataset-1.1.0"
        metadata["training_heads"] = ["drivable", "road_marking"]
        metadata_path.write_text(json.dumps(metadata) + "\n", encoding="utf-8")
        return data_root

    def test_dataset_loads_four_class_road_marking_target(self):
        with tempfile.TemporaryDirectory() as tmp:
            dataset = MoraiTwinLiteDataset(
                self.make_road_marking_dataset(Path(tmp)), split="train"
            )
            sample = dataset[0]
            self.assertEqual(set(sample["targets"]), {"drivable", "road_marking"})
            self.assertEqual(sample["targets"]["road_marking"].dtype, torch.int64)
            self.assertEqual(
                set(torch.unique(sample["targets"]["road_marking"]).tolist()),
                {0, 1, 2, 3},
            )
            self.assertEqual(dataset.task_spec, ROAD_MARKING_TASK)

    def test_four_class_output_and_loss_reach_backward(self):
        with tempfile.TemporaryDirectory() as tmp:
            dataset = MoraiTwinLiteDataset(
                self.make_road_marking_dataset(Path(tmp)), split="train"
            )
            batch = next(iter(DataLoader(dataset, batch_size=1)))
            model = TinyRoadMarkingTwin()
            criterion = MaskedTwinLiteLoss(task_spec=ROAD_MARKING_TASK)
            outputs, losses = twinlite_training_step(model, batch, criterion)
            self.assertEqual(tuple(outputs["road_marking"].shape), (1, 4, 8, 10))
            losses["loss"].backward()
            self.assertTrue(torch.isfinite(losses["loss"]))
            self.assertIsNotNone(model.road_marking.weight.grad)

    def test_tuple_output_order_matches_official_twinlite(self):
        drivable = torch.randn(1, 2, 4, 6)
        lane = torch.randn(1, 2, 4, 6)
        outputs = adapt_twinlite_outputs((drivable, lane))
        self.assertIs(outputs["drivable"], drivable)
        self.assertIs(outputs["lane"], lane)

    def test_invalid_padding_has_zero_gradient(self):
        criterion = MaskedTwinLiteLoss()
        drivable = torch.randn(1, 2, 4, 6, requires_grad=True)
        lane = torch.randn(1, 2, 4, 6, requires_grad=True)
        targets = {
            "drivable": torch.zeros(1, 4, 6, dtype=torch.long),
            "lane": torch.zeros(1, 4, 6, dtype=torch.long),
        }
        valid = torch.ones(1, 4, 6, dtype=torch.bool)
        valid[:, 0] = False
        losses = criterion((drivable, lane), targets, valid)
        losses["loss"].backward()
        self.assertTrue(torch.isfinite(losses["loss"]))
        self.assertEqual(int(torch.count_nonzero(drivable.grad[:, :, 0])), 0)
        self.assertEqual(int(torch.count_nonzero(lane.grad[:, :, 0])), 0)
        self.assertGreater(int(torch.count_nonzero(drivable.grad[:, :, 1:])), 0)

    def test_real_batch_contract_reaches_forward_and_backward(self):
        with tempfile.TemporaryDirectory() as tmp:
            dataset = MoraiTwinLiteDataset(self.make_dataset(Path(tmp)), split="train")
            batch = next(iter(DataLoader(dataset, batch_size=1)))
            model = TinyTwin()
            _, losses = twinlite_training_step(model, batch, MaskedTwinLiteLoss())
            losses["loss"].backward()
            self.assertTrue(torch.isfinite(losses["loss"]))
            self.assertIsNotNone(model.drivable.weight.grad)
            self.assertIsNotNone(model.lane.weight.grad)

    def test_head_specific_valid_masks_gate_each_heads_gradient(self):
        criterion = MaskedTwinLiteLoss()
        drivable = torch.randn(1, 2, 4, 6, requires_grad=True)
        lane = torch.randn(1, 2, 4, 6, requires_grad=True)
        targets = {
            "drivable": torch.zeros(1, 4, 6, dtype=torch.long),
            "lane": torch.zeros(1, 4, 6, dtype=torch.long),
        }
        valid = {
            "drivable": torch.ones(1, 4, 6, dtype=torch.bool),
            "lane": torch.ones(1, 4, 6, dtype=torch.bool),
        }
        valid["drivable"][:, :, :2] = False
        valid["lane"][:, :, 4:] = False
        losses = criterion((drivable, lane), targets, valid)
        losses["loss"].backward()
        self.assertEqual(int(torch.count_nonzero(drivable.grad[:, :, :, :2])), 0)
        self.assertGreater(int(torch.count_nonzero(drivable.grad[:, :, :, 4:])), 0)
        self.assertEqual(int(torch.count_nonzero(lane.grad[:, :, :, 4:])), 0)
        self.assertGreater(int(torch.count_nonzero(lane.grad[:, :, :, :2])), 0)


if __name__ == "__main__":
    unittest.main()
