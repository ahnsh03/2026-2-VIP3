"""Model-independent camera semantic inference primitives."""

from .backend import BackendInfo, CategoricalOutput, SemanticBackend
from .frame_buffer import EncodedFrame, LatestFrameBuffer
from .geometry import (
    LetterboxGeometry,
    letterbox_bgr,
    restore_categorical_probabilities,
    restore_probability,
)
from .inference_engine import BatchPrediction, InferenceEngine, ViewPrediction

__all__ = [
    "BackendInfo",
    "BatchPrediction",
    "CategoricalOutput",
    "EncodedFrame",
    "InferenceEngine",
    "LatestFrameBuffer",
    "LetterboxGeometry",
    "SemanticBackend",
    "ViewPrediction",
    "letterbox_bgr",
    "restore_categorical_probabilities",
    "restore_probability",
]
