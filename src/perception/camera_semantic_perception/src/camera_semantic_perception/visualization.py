"""OpenCV rendering helpers for camera semantic debug images."""

from __future__ import annotations

from typing import Mapping, Sequence

import cv2
import numpy as np


DRIVABLE_COLOR_BGR = (40, 200, 40)
LANE_COLOR_BGR = (0, 230, 255)
WHITE_LANE_COLOR_BGR = (255, 255, 255)
YELLOW_LANE_COLOR_BGR = (0, 230, 255)
STOPLINE_COLOR_BGR = (0, 0, 255)
ROAD_MARKING_COLORS_BGR = {
    1: WHITE_LANE_COLOR_BGR,
    2: YELLOW_LANE_COLOR_BGR,
    3: STOPLINE_COLOR_BGR,
}


def normalize_probability(probability: np.ndarray) -> np.ndarray:
    """Return a contiguous float32 probability image in the [0, 1] range."""
    value = np.asarray(probability)
    if value.ndim != 2:
        raise ValueError("probability must be a two-dimensional array")
    if value.dtype == np.uint8:
        value = value.astype(np.float32) / 255.0
    else:
        value = value.astype(np.float32, copy=False)
    return np.ascontiguousarray(np.clip(value, 0.0, 1.0))


def hard_mask(probability: np.ndarray, threshold: float) -> np.ndarray:
    if not 0.0 <= threshold <= 1.0:
        raise ValueError("threshold must be between 0 and 1")
    return normalize_probability(probability) >= threshold


def render_overlay(
    image_bgr: np.ndarray,
    drivable_probability: np.ndarray,
    lane_probability: np.ndarray,
    drivable_threshold: float = 0.5,
    lane_threshold: float = 0.5,
    alpha: float = 0.45,
) -> np.ndarray:
    """Overlay thresholded drivable and lane predictions on a BGR image."""
    _validate_image(image_bgr)
    if not 0.0 <= alpha <= 1.0:
        raise ValueError("alpha must be between 0 and 1")
    drivable = hard_mask(drivable_probability, drivable_threshold)
    lane = hard_mask(lane_probability, lane_threshold)
    _validate_spatial_shape(image_bgr, drivable, "drivable")
    _validate_spatial_shape(image_bgr, lane, "lane")

    overlay = image_bgr.copy()
    color_layer = image_bgr.copy()
    color_layer[drivable] = DRIVABLE_COLOR_BGR
    color_layer[lane] = LANE_COLOR_BGR
    active = drivable | lane
    blended = cv2.addWeighted(image_bgr, 1.0 - alpha, color_layer, alpha, 0.0)
    overlay[active] = blended[active]
    return overlay


def render_panel(
    image_bgr: np.ndarray,
    drivable_probability: np.ndarray,
    secondary_output: np.ndarray,
    view: str,
    stamp_text: str = "",
    condition_label: str = "",
    drivable_threshold: float = 0.5,
    lane_threshold: float = 0.5,
    secondary_mode: str = "lane",
    panel_width: int = 640,
) -> np.ndarray:
    """Render original, drivable and lane/road-marking as a vertical column."""
    _validate_image(image_bgr)
    if panel_width < 320:
        raise ValueError("panel_width must be at least 320")
    if secondary_mode not in ("lane", "road_marking"):
        raise ValueError("secondary_mode must be lane or road_marking")

    drivable = hard_mask(drivable_probability, drivable_threshold)
    _validate_spatial_shape(image_bgr, drivable, "drivable")
    drivable_image = np.zeros_like(image_bgr)
    drivable_image[drivable] = DRIVABLE_COLOR_BGR
    secondary_image = np.zeros_like(image_bgr)
    if secondary_mode == "lane":
        lane = hard_mask(secondary_output, lane_threshold)
        _validate_spatial_shape(image_bgr, lane, "lane")
        secondary_image[lane] = LANE_COLOR_BGR
        secondary_label = "LANE >= {:.2f}".format(lane_threshold)
    else:
        road_marking = normalize_class_ids(secondary_output)
        _validate_spatial_shape(image_bgr, road_marking, "road_marking")
        for class_id, color in ROAD_MARKING_COLORS_BGR.items():
            secondary_image[road_marking == class_id] = color
        secondary_label = "ROAD MARKING | WHITE / YELLOW / STOP"

    tile_width = panel_width
    tile_height = max(1, int(round(image_bgr.shape[0] * tile_width / image_bgr.shape[1])))
    title_parts = [str(view).upper()]
    if condition_label:
        title_parts.append(condition_label)
    if stamp_text:
        title_parts.append(stamp_text)
    context = " | ".join(title_parts)
    tiles = (
        _label_tile(image_bgr, "ORIGINAL | " + context, tile_width, tile_height),
        _label_tile(
            drivable_image,
            "DRIVABLE >= {:.2f} | {}".format(
                drivable_threshold, str(view).upper()
            ),
            tile_width,
            tile_height,
        ),
        _label_tile(
            secondary_image,
            "{} | {}".format(secondary_label, str(view).upper()),
            tile_width,
            tile_height,
        ),
    )
    return np.ascontiguousarray(np.concatenate(tiles, axis=0))


def normalize_class_ids(class_ids: np.ndarray) -> np.ndarray:
    """Validate a road-marking class mask encoded as 0..3 uint8."""
    value = np.asarray(class_ids)
    if value.ndim != 2:
        raise ValueError("class IDs must be a two-dimensional array")
    if value.dtype != np.uint8:
        raise ValueError("class IDs must be uint8")
    if value.size and int(value.max()) > 3:
        raise ValueError("road-marking class ID must be in [0,3]")
    return np.ascontiguousarray(value)


def render_mosaic(
    panels: Mapping[str, np.ndarray],
    views: Sequence[str],
    per_view_width: int = 640,
) -> np.ndarray:
    """각 view의 3행 column을 가로로 이어 붙인다(N view, 종횡비 유지)."""
    if per_view_width < 160:
        raise ValueError("per_view_width must be at least 160")
    if not views:
        raise ValueError("views must not be empty")
    missing = [view for view in views if view not in panels]
    if missing:
        raise ValueError("missing panels for views: {}".format(missing))

    columns = []
    for view in views:
        panel = panels[view]
        _validate_image(panel)
        if panel.shape[0] % 3:
            raise ValueError("panel height must contain three equal rows")
        columns.append(np.split(panel, 3, axis=0))

    mosaic_rows = []
    for row_index in range(3):
        row_tiles = [
            _resize_to_width(column[row_index], per_view_width) for column in columns
        ]
        target_height = max(tile.shape[0] for tile in row_tiles)
        mosaic_rows.append(
            np.concatenate(
                [_pad_to_height(tile, target_height) for tile in row_tiles], axis=1
            )
        )
    return np.ascontiguousarray(np.concatenate(mosaic_rows, axis=0))


def _resize_to_width(image: np.ndarray, width: int) -> np.ndarray:
    height = max(1, int(round(image.shape[0] * width / image.shape[1])))
    interpolation = cv2.INTER_AREA if width < image.shape[1] else cv2.INTER_LINEAR
    return cv2.resize(image, (width, height), interpolation=interpolation)


def _pad_to_height(image: np.ndarray, height: int) -> np.ndarray:
    if image.shape[0] > height:
        raise ValueError("target height must not be smaller than image height")
    missing = height - image.shape[0]
    top = missing // 2
    bottom = missing - top
    return cv2.copyMakeBorder(
        image, top, bottom, 0, 0, cv2.BORDER_CONSTANT, value=(0, 0, 0)
    )


def _label_tile(
    image: np.ndarray, label: str, width: int, height: int
) -> np.ndarray:
    interpolation = cv2.INTER_AREA if width < image.shape[1] else cv2.INTER_LINEAR
    tile = cv2.resize(image, (width, height), interpolation=interpolation)
    bar_height = max(24, min(44, height // 10))
    cv2.rectangle(tile, (0, 0), (width, bar_height), (0, 0, 0), -1)
    font_scale = max(0.42, min(0.75, width / 900.0))
    cv2.putText(
        tile,
        label,
        (10, int(bar_height * 0.72)),
        cv2.FONT_HERSHEY_SIMPLEX,
        font_scale,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    return tile


def _validate_image(image: np.ndarray) -> None:
    if not isinstance(image, np.ndarray) or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("image_bgr must have shape (height, width, 3)")
    if image.dtype != np.uint8:
        raise ValueError("image_bgr must use uint8 pixels")


def _validate_spatial_shape(
    image: np.ndarray, probability: np.ndarray, name: str
) -> None:
    if probability.shape != image.shape[:2]:
        raise ValueError(
            "{} shape {} does not match image shape {}".format(
                name, probability.shape, image.shape[:2]
            )
        )
