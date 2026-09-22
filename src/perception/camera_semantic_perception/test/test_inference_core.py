#!/usr/bin/env python3

import threading
import time
import unittest

import numpy as np

from camera_semantic_perception.backend import (
    BackendInfo,
    CategoricalOutput,
    SemanticBackend,
)
from camera_semantic_perception.backends import create_backend
from camera_semantic_perception.frame_buffer import EncodedFrame, LatestFrameBuffer
from camera_semantic_perception.geometry import (
    compute_letterbox_geometry,
    letterbox_bgr,
    restore_categorical_probabilities,
    restore_class_ids,
    restore_probability,
)
from camera_semantic_perception.inference_engine import InferenceEngine


class DummyBackend(SemanticBackend):
    def __init__(self):
        self._info = BackendInfo(
            name="dummy",
            model_config="test",
            heads=("drivable", "lane"),
            input_hw=(384, 640),
            device="cpu",
            checkpoint_path="dummy.pt",
            checkpoint_sha256="0" * 64,
            checkpoint_epoch=0,
            weights="model",
            upstream_commit="dummy",
        )

    @property
    def info(self):
        return self._info

    def infer(self, batch):
        base = batch[:, 0].astype(np.float32, copy=False)
        return {"drivable": base, "lane": 1.0 - base}


class CategoricalDummyBackend(SemanticBackend):
    def __init__(self):
        self._info = BackendInfo(
            name="categorical-dummy",
            model_config="test",
            heads=("drivable", "road_marking"),
            input_hw=(384, 640),
            device="cpu",
            checkpoint_path="dummy.pt",
            checkpoint_sha256="0" * 64,
            checkpoint_epoch=0,
            weights="model",
            upstream_commit="dummy",
            categorical_classes={
                "road_marking": (
                    "background",
                    "white_lane",
                    "yellow_lane",
                    "stopline",
                )
            },
        )

    @property
    def info(self):
        return self._info

    def infer(self, batch):
        drivable = batch[:, 0].astype(np.float32, copy=False)
        marking = np.zeros(drivable.shape, dtype=np.uint8)
        marking[:, 100:200, 100:200] = 1
        marking[:, 200:300, 200:300] = 2
        marking[:, 300:350, 300:400] = 3
        confidence = np.full(marking.shape, 0.9, dtype=np.float32)
        return {
            "drivable": drivable,
            "road_marking": CategoricalOutput(marking, confidence),
        }


class GeometryTest(unittest.TestCase):
    def test_front_geometry_matches_training_export(self):
        geometry = compute_letterbox_geometry((900, 1600), (384, 640))
        self.assertEqual(geometry.resized_hw, (360, 640))
        self.assertEqual(geometry.pad_ltrb, (0, 12, 0, 12))

    def test_vip3_front_rear_geometry(self):
        # VIP3 front/rear: 1280x720 FOV 90. 두 view 의 letterbox 기하가 같다.
        geometry = compute_letterbox_geometry((720, 1280), (384, 640))
        self.assertEqual(geometry.resized_hw, (360, 640))
        self.assertEqual(geometry.pad_ltrb, (0, 12, 0, 12))

    def test_portrait_geometry_matches_training_export(self):
        geometry = compute_letterbox_geometry((1200, 1600), (384, 640))
        self.assertEqual(geometry.resized_hw, (384, 512))
        self.assertEqual(geometry.pad_ltrb, (64, 0, 64, 0))

    def test_letterbox_converts_bgr_to_normalized_rgb(self):
        image = np.zeros((900, 1600, 3), dtype=np.uint8)
        image[:, :] = (10, 20, 30)
        tensor, geometry = letterbox_bgr(image)
        self.assertEqual(tensor.shape, (3, 384, 640))
        self.assertEqual(tensor.dtype, np.float32)
        self.assertAlmostEqual(float(tensor[0, 12, 0]), 30.0 / 255.0)
        self.assertAlmostEqual(float(tensor[2, 12, 0]), 10.0 / 255.0)
        self.assertEqual(geometry.pad_ltrb, (0, 12, 0, 12))

    def test_restore_removes_padding_and_returns_original_shape(self):
        geometry = compute_letterbox_geometry((900, 1600), (384, 640))
        probability = np.zeros((384, 640), dtype=np.float32)
        probability[12:372] = 0.75
        restored = restore_probability(probability, geometry)
        self.assertEqual(restored.shape, (900, 1600))
        self.assertTrue(np.allclose(restored, 0.75))

    def test_restore_class_ids_uses_nearest_neighbor(self):
        geometry = compute_letterbox_geometry((900, 1600), (384, 640))
        class_ids = np.zeros((384, 640), dtype=np.uint8)
        class_ids[12:192] = 3
        restored = restore_class_ids(class_ids, geometry)
        self.assertEqual(restored.shape, (900, 1600))
        self.assertEqual(restored.dtype, np.uint8)
        self.assertEqual(set(np.unique(restored)), {0, 3})

    def test_restore_categorical_probabilities_renormalizes(self):
        geometry = compute_letterbox_geometry((900, 1600), (384, 640))
        probabilities = np.zeros((4, 384, 640), dtype=np.float32)
        probabilities[0] = 0.1
        probabilities[1] = 0.2
        probabilities[2] = 0.3
        probabilities[3] = 0.4
        restored = restore_categorical_probabilities(probabilities, geometry)
        self.assertEqual((4, 900, 1600), restored.shape)
        np.testing.assert_allclose(restored.sum(axis=0), 1.0, atol=1e-6)


    def test_vip3_side_geometry(self):
        # VIP3 left/right: 640x480 FOV 130.
        geometry = compute_letterbox_geometry((480, 640), (384, 640))
        self.assertEqual(geometry.resized_hw, (384, 512))
        self.assertEqual(geometry.pad_ltrb, (64, 0, 64, 0))


class InferenceEngineTest(unittest.TestCase):
    def test_backend_registry_rejects_unknown_model(self):
        with self.assertRaisesRegex(ValueError, "unsupported semantic backend"):
            create_backend("not-a-model")

    def test_multi_view_batch_restores_each_native_shape(self):
        # VIP3 실제 4-view 크기. front/rear 1280x720, left/right 640x480 이
        # 한 배치로 들어가고 각자 원본 크기로 복원돼야 한다.
        native_hw = {
            "front": (720, 1280),
            "left": (480, 640),
            "right": (480, 640),
            "rear": (720, 1280),
        }
        images = {
            view: np.full((hw[0], hw[1], 3), (0, 0, 255), dtype=np.uint8)
            for view, hw in native_hw.items()
        }
        prediction = InferenceEngine(DummyBackend()).infer(images)
        self.assertEqual(set(prediction.views), set(native_hw))
        for view, hw in native_hw.items():
            for head in ("drivable", "lane"):
                self.assertEqual(
                    prediction.views[view].probabilities[head].shape, hw
                )
        self.assertGreaterEqual(prediction.inference_ms, 0.0)

    def test_categorical_output_restores_class_and_confidence(self):
        image = np.full((900, 1600, 3), (0, 0, 255), dtype=np.uint8)
        prediction = InferenceEngine(CategoricalDummyBackend()).infer(
            {"front": image}
        )
        marking = prediction.views["front"].probabilities["road_marking"]
        confidence = prediction.views["front"].categorical_confidences[
            "road_marking"
        ]
        self.assertEqual(marking.shape, (900, 1600))
        self.assertEqual(marking.dtype, np.uint8)
        self.assertEqual(set(np.unique(marking)), {0, 1, 2, 3})
        self.assertEqual(confidence.shape, (900, 1600))
        self.assertEqual(confidence.dtype, np.float32)
        np.testing.assert_allclose(confidence, 0.9, atol=1e-6)
        model_marking = prediction.views["front"].model_grid_probabilities[
            "road_marking"
        ]
        model_confidence = prediction.views[
            "front"
        ].model_grid_categorical_confidences["road_marking"]
        self.assertEqual(model_marking.shape, (384, 640))
        self.assertEqual(model_marking.dtype, np.uint8)
        self.assertEqual(model_confidence.shape, (384, 640))
        self.assertEqual(model_confidence.dtype, np.float32)
        np.testing.assert_allclose(model_confidence, 0.9, atol=1e-6)

    def test_model_grid_copy_can_be_disabled_for_unsubscribed_runtime(self):
        image = np.zeros((720, 1280, 3), dtype=np.uint8)
        prediction = InferenceEngine(CategoricalDummyBackend()).infer(
            {"front": image}, preserve_model_grid=False
        )
        view = prediction.views["front"]
        self.assertEqual({}, view.model_grid_probabilities)
        self.assertEqual({}, view.model_grid_categorical_confidences)


class LatestFrameBufferTest(unittest.TestCase):
    @staticmethod
    def frame(view, value):
        return EncodedFrame(view, None, view, bytes([value]), time.monotonic())

    def test_replaces_unprocessed_frame(self):
        buffer = LatestFrameBuffer(("front",), batch_wait_ms=0)
        buffer.push(self.frame("front", 1))
        buffer.push(self.frame("front", 2))
        batch = buffer.pop_batch(timeout=0.01)
        self.assertEqual(batch[0].data, bytes([2]))
        self.assertEqual(buffer.replaced["front"], 1)

    def test_coalesces_views_in_declared_order(self):
        views = ("front", "left", "right", "rear")
        buffer = LatestFrameBuffer(views, batch_wait_ms=20)

        def delayed_push():
            for index, view in enumerate(("rear", "left", "right"), start=2):
                time.sleep(0.002)
                buffer.push(self.frame(view, index))

        buffer.push(self.frame("front", 1))
        thread = threading.Thread(target=delayed_push)
        thread.start()
        batch = buffer.pop_batch(timeout=0.2)
        thread.join()
        # 도착 순서가 아니라 선언 순서대로 나와야 한다.
        self.assertEqual([frame.view for frame in batch], list(views))


if __name__ == "__main__":
    unittest.main()
