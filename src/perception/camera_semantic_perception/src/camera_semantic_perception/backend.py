"""Backend contract shared by live segmentation models."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Mapping, Sequence, Tuple, Union

import numpy as np


@dataclass(frozen=True)
class CategoricalOutput:
    """Host-side categorical result reduced from an in-backend softmax."""

    class_ids: np.ndarray
    confidence: np.ndarray


@dataclass(frozen=True)
class BackendInfo:
    name: str
    model_config: str
    heads: Tuple[str, ...]
    input_hw: Tuple[int, int]
    device: str
    checkpoint_path: str
    checkpoint_sha256: str
    checkpoint_epoch: int
    weights: str
    upstream_commit: str
    categorical_classes: Mapping[str, Tuple[str, ...]] = field(default_factory=dict)


class SemanticBackend(ABC):
    """Convert normalized RGB NCHW batches to foreground probabilities."""

    @property
    @abstractmethod
    def info(self) -> BackendInfo:
        raise NotImplementedError

    @abstractmethod
    def infer(
        self, batch: np.ndarray
    ) -> Mapping[str, Union[np.ndarray, CategoricalOutput]]:
        """Return one probability array per head.

        Continuous heads are float32 ``[B,H,W]`` foreground probabilities.
        Heads listed in ``info.categorical_classes`` are ``CategoricalOutput``
        values. The backend computes softmax once, then transfers only the
        uint8 argmax and float32 maximum confidence as ``[B,H,W]`` arrays.
        """
        raise NotImplementedError


def validate_backend_output(
    outputs: Mapping[str, np.ndarray],
    heads: Sequence[str],
    batch_shape: Sequence[int],
    categorical_classes: Mapping[str, Sequence[str]] = None,
) -> None:
    categorical_classes = categorical_classes or {}
    expected_shape = (int(batch_shape[0]), int(batch_shape[2]), int(batch_shape[3]))
    if set(outputs) != set(heads):
        raise ValueError(
            "backend heads differ from contract: expected={} actual={}".format(
                tuple(heads), tuple(outputs)
            )
        )
    for head in heads:
        value = outputs[head]
        if head in categorical_classes:
            class_count = len(categorical_classes[head])
            if class_count <= 0:
                raise ValueError("{} has no categorical classes".format(head))
            if not isinstance(value, CategoricalOutput):
                raise TypeError("{} output must be CategoricalOutput".format(head))
            if value.class_ids.shape != expected_shape:
                raise ValueError(
                    "{} categorical class shape must be {}, got {}".format(
                        head, expected_shape, value.class_ids.shape
                    )
                )
            if value.confidence.shape != expected_shape:
                raise ValueError(
                    "{} categorical confidence shape must be {}, got {}".format(
                        head, expected_shape, value.confidence.shape
                    )
                )
            if value.class_ids.dtype != np.uint8:
                raise ValueError("{} categorical class IDs must be uint8".format(head))
            if value.confidence.dtype != np.float32:
                raise ValueError(
                    "{} categorical confidence must be float32".format(head)
                )
            if value.class_ids.size and int(value.class_ids.max()) >= class_count:
                raise ValueError(
                    "{} categorical class ID is outside configured classes".format(head)
                )
            if not np.all(np.isfinite(value.confidence)):
                raise ValueError(
                    "{} categorical confidence contains non-finite values".format(head)
                )
            if value.confidence.size and (
                float(value.confidence.min()) < 0.0
                or float(value.confidence.max()) > 1.0
            ):
                raise ValueError(
                    "{} categorical confidence lies outside [0,1]".format(head)
                )
        else:
            if not isinstance(value, np.ndarray):
                raise TypeError("{} output must be a numpy array".format(head))
            if value.shape != expected_shape:
                raise ValueError(
                    "{} output shape must be {}, got {}".format(
                        head, expected_shape, value.shape
                    )
                )
            if value.dtype != np.float32:
                raise ValueError("{} output must be float32".format(head))
            if not np.all(np.isfinite(value)):
                raise ValueError("{} output contains non-finite values".format(head))
            if value.size and (float(value.min()) < 0.0 or float(value.max()) > 1.0):
                raise ValueError("{} probability lies outside [0,1]".format(head))
