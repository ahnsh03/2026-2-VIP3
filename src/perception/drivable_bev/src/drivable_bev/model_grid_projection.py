"""Direct model-grid to continuous ``base_link`` ground projection."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Tuple

import numpy as np

from camera_semantic_perception.geometry import (
    LetterboxGeometry,
    compute_letterbox_geometry,
)

from .calibration import CameraCalibration


@dataclass(frozen=True)
class MetricEvidence:
    """Road-marking evidence in continuous vehicle coordinates."""

    xy: np.ndarray
    class_id: np.ndarray
    confidence: np.ndarray
    geometric_quality: np.ndarray
    source_mask: np.ndarray

    def __post_init__(self) -> None:
        count = len(self.xy)
        if np.asarray(self.xy).shape != (count, 2):
            raise ValueError("evidence xy must have shape [N,2]")
        for name in ("class_id", "confidence", "geometric_quality", "source_mask"):
            if np.asarray(getattr(self, name)).shape != (count,):
                raise ValueError("evidence {} must have shape [N]".format(name))

    @classmethod
    def empty(cls) -> "MetricEvidence":
        return cls(
            xy=np.empty((0, 2), dtype=np.float32),
            class_id=np.empty(0, dtype=np.uint8),
            confidence=np.empty(0, dtype=np.float32),
            geometric_quality=np.empty(0, dtype=np.float32),
            source_mask=np.empty(0, dtype=np.uint8),
        )

    @classmethod
    def concatenate(cls, values) -> "MetricEvidence":
        values = [value for value in values if len(value.xy)]
        if not values:
            return cls.empty()
        return cls(
            xy=np.ascontiguousarray(np.concatenate([value.xy for value in values])),
            class_id=np.ascontiguousarray(
                np.concatenate([value.class_id for value in values])
            ),
            confidence=np.ascontiguousarray(
                np.concatenate([value.confidence for value in values])
            ),
            geometric_quality=np.ascontiguousarray(
                np.concatenate([value.geometric_quality for value in values])
            ),
            source_mask=np.ascontiguousarray(
                np.concatenate([value.source_mask for value in values])
            ),
        )


class ModelGridGroundProjector:
    """Precompute a calibrated ground lookup for one semantic model grid."""

    def __init__(
        self,
        camera: CameraCalibration,
        model_hw: Tuple[int, int] = (384, 640),
        source_mask: int = 1,
        x_bounds_m: Tuple[float, float] = (-5.0, 20.0),
        y_bounds_m: Tuple[float, float] = (-8.0, 8.0),
        max_ground_range_m: float = 30.0,
        min_camera_depth_m: float = 0.10,
        edge_taper_fraction: float = 0.08,
        forward_fade_start_m: float = 15.0,
    ) -> None:
        if not 0 < int(source_mask) <= 255:
            raise ValueError("source_mask must be a positive uint8 value")
        if not 0.0 < edge_taper_fraction < 0.5:
            raise ValueError("edge_taper_fraction must lie in (0,0.5)")
        if max_ground_range_m <= 0.0 or min_camera_depth_m <= 0.0:
            raise ValueError("ground range and camera depth must be positive")
        self.camera = camera
        self.source_mask = int(source_mask)
        self.x_bounds_m = tuple(float(value) for value in x_bounds_m)
        self.y_bounds_m = tuple(float(value) for value in y_bounds_m)
        if self.x_bounds_m[1] <= self.x_bounds_m[0]:
            raise ValueError("x bounds are invalid")
        if self.y_bounds_m[1] <= self.y_bounds_m[0]:
            raise ValueError("y bounds are invalid")
        self.geometry: LetterboxGeometry = compute_letterbox_geometry(
            (camera.height, camera.width), model_hw
        )
        self.model_hw = self.geometry.target_hw

        height, width = self.model_hw
        rows, columns = np.indices((height, width), dtype=np.float64)
        left, top, _, _ = self.geometry.pad_ltrb
        scale_x, scale_y = self.geometry.scale_xy
        native_u = (columns - float(left)) / scale_x
        native_v = (rows - float(top)) / scale_y

        image_points = np.stack(
            [native_u, native_v, np.ones_like(native_u)], axis=-1
        ).reshape(-1, 3)
        image_from_ground = camera.ground_to_image_homography
        if abs(float(np.linalg.det(image_from_ground))) < 1e-12:
            raise ValueError("camera ground homography is singular")
        ground_h = (np.linalg.inv(image_from_ground) @ image_points.T).T
        denominator = ground_h[:, 2]
        safe = np.abs(denominator) > 1e-9
        ground = np.full((len(ground_h), 2), np.nan, dtype=np.float64)
        ground[safe] = ground_h[safe, :2] / denominator[safe, None]
        ground = ground.reshape(height, width, 2)

        resized_h, resized_w = self.geometry.resized_hw
        letterbox_valid = (
            (columns >= left)
            & (columns < left + resized_w)
            & (rows >= top)
            & (rows < top + resized_h)
        )
        flattened = ground.reshape(-1, 2)
        xyz = camera.ground_plane.xyz(flattened)
        optical_depth = camera.ground_to_optical(xyz)[:, 2].reshape(height, width)
        distance = np.linalg.norm(ground, axis=2)
        finite = np.isfinite(ground).all(axis=2)
        valid = (
            letterbox_valid
            & finite
            & (optical_depth > min_camera_depth_m)
            & (distance <= max_ground_range_m)
            & (ground[:, :, 0] >= self.x_bounds_m[0])
            & (ground[:, :, 0] <= self.x_bounds_m[1])
            & (ground[:, :, 1] >= self.y_bounds_m[0])
            & (ground[:, :, 1] <= self.y_bounds_m[1])
        )

        edge_distance = np.minimum.reduce(
            [
                columns - left,
                left + resized_w - 1.0 - columns,
                rows - top,
                top + resized_h - 1.0 - rows,
            ]
        )
        edge_scale = edge_taper_fraction * min(resized_h, resized_w)
        edge_quality = np.clip(edge_distance / max(edge_scale, 1e-6), 0.0, 1.0)

        safe_ground = ground.copy()
        safe_ground[~finite] = 0.0
        dx_du = np.gradient(safe_ground[:, :, 0], axis=1)
        dx_dv = np.gradient(safe_ground[:, :, 0], axis=0)
        dy_du = np.gradient(safe_ground[:, :, 1], axis=1)
        dy_dv = np.gradient(safe_ground[:, :, 1], axis=0)
        area_per_pixel = np.abs(dx_du * dy_dv - dx_dv * dy_du)
        density = np.zeros_like(area_per_pixel)
        density[valid] = 1.0 / np.sqrt(np.maximum(area_per_pixel[valid], 1e-9))
        if np.any(valid):
            density_scale = max(float(np.percentile(density[valid], 95.0)), 1e-9)
            density = np.clip(density / density_scale, 0.05, 1.0)

        forward_quality = np.ones((height, width), dtype=np.float64)
        beyond = ground[:, :, 0] > float(forward_fade_start_m)
        forward_quality[beyond] = np.clip(
            (self.x_bounds_m[1] - ground[:, :, 0][beyond])
            / max(self.x_bounds_m[1] - float(forward_fade_start_m), 1e-6),
            0.20,
            1.0,
        )
        quality = edge_quality * density * forward_quality
        quality[~valid] = 0.0
        self.ground_xy = np.ascontiguousarray(ground, dtype=np.float32)
        self.valid = np.ascontiguousarray(valid)
        self.geometric_quality = np.ascontiguousarray(quality, dtype=np.float32)

    def _selection_masks(
        self,
        class_id: np.ndarray,
        confidence: np.ndarray,
        confidence_by_class: Mapping[int, float],
        classes=(1, 2),
    ) -> tuple[np.ndarray, np.ndarray, dict]:
        classes_value = np.asarray(class_id)
        confidence_value = np.asarray(confidence)
        if classes_value.shape != self.model_hw or classes_value.dtype != np.uint8:
            raise ValueError(
                "class_id must be uint8 {}, got {} {}".format(
                    self.model_hw, classes_value.shape, classes_value.dtype
                )
            )
        if confidence_value.shape != self.model_hw:
            raise ValueError("confidence shape differs from the model grid")
        if confidence_value.dtype == np.uint8:
            normalized_confidence = confidence_value.astype(np.float32) / 255.0
        else:
            normalized_confidence = confidence_value.astype(np.float32, copy=False)
        if not np.isfinite(normalized_confidence).all():
            raise ValueError("confidence contains non-finite values")
        if normalized_confidence.size and (
            float(normalized_confidence.min()) < 0.0
            or float(normalized_confidence.max()) > 1.0
        ):
            raise ValueError("confidence lies outside [0,1]")

        class_selected = np.zeros(self.model_hw, dtype=bool)
        confidence_selected = np.zeros(self.model_hw, dtype=bool)
        thresholds = {}
        for value in classes:
            threshold = float(confidence_by_class.get(int(value), 0.5))
            if not 0.0 <= threshold <= 1.0:
                raise ValueError("class confidence threshold lies outside [0,1]")
            class_mask = classes_value == int(value)
            class_selected |= class_mask
            confidence_selected |= class_mask & (
                normalized_confidence >= threshold
            )
            thresholds[int(value)] = threshold
        selected = confidence_selected & self.valid
        return classes_value, normalized_confidence, {
            "class_selected": class_selected,
            "confidence_selected": confidence_selected,
            "selected": selected,
            "thresholds": thresholds,
        }

    def selection_audit(
        self,
        class_id: np.ndarray,
        confidence: np.ndarray,
        confidence_by_class: Mapping[int, float],
        classes=(1, 2),
    ) -> dict:
        """Explain projection/confidence loss without changing extraction."""
        classes_value, _, masks = self._selection_masks(
            class_id, confidence, confidence_by_class, classes
        )
        by_class = {}
        for value in classes:
            class_mask = classes_value == int(value)
            valid_class = class_mask & self.valid
            accepted = valid_class & masks["confidence_selected"]
            by_class[str(int(value))] = {
                "model_pixels": int(np.count_nonzero(class_mask)),
                "projection_invalid": int(np.count_nonzero(class_mask & ~self.valid)),
                "below_confidence": int(
                    np.count_nonzero(valid_class & ~masks["confidence_selected"])
                ),
                "accepted": int(np.count_nonzero(accepted)),
                "threshold": float(masks["thresholds"][int(value)]),
            }
        return {
            "model_marking_pixels": int(np.count_nonzero(masks["class_selected"])),
            "projection_invalid": int(
                np.count_nonzero(masks["class_selected"] & ~self.valid)
            ),
            "below_confidence": int(
                np.count_nonzero(
                    masks["class_selected"]
                    & self.valid
                    & ~masks["confidence_selected"]
                )
            ),
            "accepted": int(np.count_nonzero(masks["selected"])),
            "by_class": by_class,
        }

    def extract(
        self,
        class_id: np.ndarray,
        confidence: np.ndarray,
        confidence_by_class: Mapping[int, float],
        classes=(1, 2),
    ) -> MetricEvidence:
        classes_value, normalized_confidence, masks = self._selection_masks(
            class_id, confidence, confidence_by_class, classes
        )
        selected = masks["selected"]
        if not np.any(selected):
            return MetricEvidence.empty()
        count = int(np.count_nonzero(selected))
        return MetricEvidence(
            xy=np.ascontiguousarray(self.ground_xy[selected], dtype=np.float32),
            class_id=np.ascontiguousarray(classes_value[selected], dtype=np.uint8),
            confidence=np.ascontiguousarray(
                normalized_confidence[selected], dtype=np.float32
            ),
            geometric_quality=np.ascontiguousarray(
                self.geometric_quality[selected], dtype=np.float32
            ),
            source_mask=np.full(count, self.source_mask, dtype=np.uint8),
        )
