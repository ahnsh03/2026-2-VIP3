"""OpenCV-only camera calibration and BEV debug visualization helpers."""

from __future__ import annotations

from typing import Mapping, Sequence, Tuple

import cv2
import numpy as np

from .calibration import CameraCalibration
from .grid import BevGridSpec


ROAD_MARKING_COLORS_BGR = {
    1: (255, 255, 255),
    2: (0, 255, 255),
    3: (0, 0, 255),
}

SHARED_FAMILY_STATE_COLORS_BGR = {
    "structure_inferred": (0, 165, 255),
    "invalid": (45, 45, 210),
}


def bev_crop_slices(source: BevGridSpec, display: BevGridSpec):
    """Return exact array slices for an aligned display grid inside source."""
    if not np.isclose(source.resolution_m, display.resolution_m, atol=1e-9):
        raise ValueError("display and source BEV resolutions must match")
    if (
        display.x_min_m < source.x_min_m
        or display.x_max_m > source.x_max_m
        or display.y_min_m < source.y_min_m
        or display.y_max_m > source.y_max_m
    ):
        raise ValueError("display BEV must lie inside the source grid")
    top_left = source.metric_to_pixel([[display.x_max_m, display.y_max_m]])[0]
    bottom_right = source.metric_to_pixel([[display.x_min_m, display.y_min_m]])[0]
    column_start, row_start = np.rint(top_left).astype(int)
    column_end, row_end = np.rint(bottom_right).astype(int)
    if (row_end - row_start, column_end - column_start) != display.shape:
        raise ValueError("display bounds do not align with source grid cells")
    return slice(row_start, row_end), slice(column_start, column_end)


def crop_bev_layers(layers: Mapping[str, np.ndarray], source, display):
    rows, columns = bev_crop_slices(source, display)
    result = {}
    for name, value in layers.items():
        array = np.asarray(value)
        if array.shape[:2] != source.shape:
            raise ValueError("{} layer does not match source BEV".format(name))
        result[name] = np.ascontiguousarray(array[rows, columns])
    return result


def colorize_drivable(probability: np.ndarray) -> np.ndarray:
    value = _mono8(probability, "drivable probability")
    canvas = np.zeros((*value.shape, 3), dtype=np.uint8)
    canvas[:, :, 1] = value
    return canvas


def colorize_road_marking(class_ids: np.ndarray) -> np.ndarray:
    value = _mono8(class_ids, "road-marking class IDs")
    if np.any(value > 3):
        raise ValueError("road-marking class IDs must be in [0,3]")
    canvas = np.zeros((*value.shape, 3), dtype=np.uint8)
    for class_id, color in ROAD_MARKING_COLORS_BGR.items():
        canvas[value == class_id] = color
    return canvas


def render_fused_bev(
    drivable_probability: np.ndarray,
    road_marking_class_id: np.ndarray,
    road_marking_confidence: np.ndarray,
    coverage: np.ndarray,
    source_count: np.ndarray,
    grid: BevGridSpec,
    label_prefix: str = "FUSED",
) -> np.ndarray:
    """Render one metric BEV with fused semantics and an unambiguous extent label."""
    drivable = _mono8(drivable_probability, "drivable probability")
    marking = _mono8(road_marking_class_id, "road-marking class IDs")
    confidence = _mono8(road_marking_confidence, "road-marking confidence")
    valid = _mono8(coverage, "coverage") > 0
    count = _mono8(source_count, "source count")
    for value in (drivable, marking, confidence, count):
        if value.shape != grid.shape:
            raise ValueError("fused BEV layer shape does not match grid")
    if np.any(marking > 3) or np.any(count > 3):
        raise ValueError("invalid fused class or source-count value")

    canvas = np.zeros((*grid.shape, 3), dtype=np.uint8)
    canvas[:, :, 1] = np.rint(drivable.astype(np.float32) * 0.72).astype(np.uint8)
    canvas[~valid] = 0
    for class_id, color in ROAD_MARKING_COLORS_BGR.items():
        selected = (marking == class_id) & valid
        if not np.any(selected):
            continue
        strength = 0.45 + 0.55 * confidence[selected].astype(np.float32) / 255.0
        canvas[selected] = np.rint(
            np.asarray(color, dtype=np.float32)[None, :] * strength[:, None]
        ).astype(np.uint8)

    draw_metric_grid(canvas, grid, major_step_m=5.0, minor_step_m=2.0)
    label = "{}  x[{:.0f},{:.0f}]m  y[{:.0f},{:.0f}]m  {:.2f}m/cell".format(
        str(label_prefix),
        grid.x_min_m,
        grid.x_max_m,
        grid.y_min_m,
        grid.y_max_m,
        grid.resolution_m,
    )
    _label(canvas, label)
    return canvas


def render_fused_quality(
    road_marking_class_id: np.ndarray,
    road_marking_confidence: np.ndarray,
    coverage: np.ndarray,
    source_count: np.ndarray,
    grid: BevGridSpec,
    label_prefix: str = "QUALITY",
) -> np.ndarray:
    """Render camera overlap and marking confidence without drivable fill.

    The background shade encodes how many cameras cover a cell.  Marking hue
    remains the semantic class while brightness encodes the selected class
    confidence.  This view is intentionally diagnostic: coverage is not an
    accuracy or planning-validity guarantee.
    """
    marking = _mono8(road_marking_class_id, "road-marking class IDs")
    confidence = _mono8(road_marking_confidence, "road-marking confidence")
    valid = _mono8(coverage, "coverage") > 0
    count = _mono8(source_count, "source count")
    for value in (marking, confidence, count):
        if value.shape != grid.shape:
            raise ValueError("fused quality layer shape does not match grid")
    if np.any(marking > 3) or np.any(count > 3):
        raise ValueError("invalid fused class or source-count value")

    canvas = np.zeros((*grid.shape, 3), dtype=np.uint8)
    overlap_colors = {
        1: (55, 22, 12),
        2: (75, 58, 18),
        3: (65, 100, 35),
    }
    for camera_count, color in overlap_colors.items():
        canvas[valid & (count == camera_count)] = color
    for class_id, color in ROAD_MARKING_COLORS_BGR.items():
        selected = (marking == class_id) & valid
        if not np.any(selected):
            continue
        strength = 0.20 + 0.80 * confidence[selected].astype(np.float32) / 255.0
        canvas[selected] = np.rint(
            np.asarray(color, dtype=np.float32)[None, :] * strength[:, None]
        ).astype(np.uint8)

    draw_metric_grid(canvas, grid, major_step_m=5.0, minor_step_m=2.0)
    label = (
        "{}  x[{:.0f},{:.0f}]m y[{:.0f},{:.0f}]m  "
        "bg=source count  line=confidence"
    ).format(
        str(label_prefix),
        grid.x_min_m,
        grid.x_max_m,
        grid.y_min_m,
        grid.y_max_m,
    )
    _label(canvas, label)
    return canvas


def draw_metric_evidence(canvas, points_xy, class_ids, grid, radius=1, colors=None):
    """Draw continuous white/yellow metric evidence on a BEV canvas."""
    points = np.asarray(points_xy, dtype=np.float64).reshape(-1, 2)
    classes = np.asarray(class_ids, dtype=np.uint8).reshape(-1)
    if len(points) != len(classes):
        raise ValueError("evidence point and class counts differ")
    pixels = np.rint(grid.metric_to_pixel(points)).astype(np.int32)
    palette = ROAD_MARKING_COLORS_BGR if colors is None else colors
    for class_id in (1, 2):
        selected = classes == class_id
        color = palette[class_id]
        for column, row in pixels[selected]:
            if 0 <= row < canvas.shape[0] and 0 <= column < canvas.shape[1]:
                cv2.circle(canvas, (int(column), int(row)), int(radius), color, -1)


def draw_metric_lanes(canvas, lanes, grid, thickness=2, colors=None):
    """Draw fitted lane objects or ``(class_id, points)`` pairs."""
    palette = ROAD_MARKING_COLORS_BGR if colors is None else colors
    for lane in lanes:
        if hasattr(lane, "sample_points"):
            class_id = int(lane.class_id)
            points = lane.sample_points
        else:
            class_id, points = lane
            class_id = int(class_id)
        if class_id not in (1, 2):
            continue
        pixels = np.rint(grid.metric_to_pixel(points)).astype(np.int32)
        valid = (
            (pixels[:, 0] >= 0)
            & (pixels[:, 0] < canvas.shape[1])
            & (pixels[:, 1] >= 0)
            & (pixels[:, 1] < canvas.shape[0])
        )
        start = None
        for index, is_valid in enumerate(valid):
            if is_valid and start is None:
                start = index
            at_end = index == len(valid) - 1
            if start is not None and ((not is_valid) or at_end):
                end = index + 1 if is_valid and at_end else index
                if end - start >= 2:
                    cv2.polylines(
                        canvas,
                        [pixels[start:end].reshape(-1, 1, 2)],
                        False,
                        palette[class_id],
                        int(thickness),
                        cv2.LINE_AA,
                    )
                start = None


def draw_stateful_lane_family(canvas, segments, grid, status_text=""):
    """Draw shared-family segments with observation validity as the primary cue.

    ``segments`` contains ``(role, state, class_id, points_xy)`` tuples.  The
    caller must split segments at state transitions so this renderer cannot
    accidentally connect an invalid interval to an observed one.
    """
    if canvas.shape[:2] != grid.shape:
        raise ValueError("canvas shape does not match BEV grid")
    valid_states = {"observed", "structure_inferred", "invalid"}
    for role, state, class_id, points in segments:
        del role  # Role is retained in the transport tuple and marker namespace.
        if state not in valid_states:
            raise ValueError("unknown shared-family visualization state")
        class_id = int(class_id)
        if class_id not in (1, 2):
            raise ValueError("shared-family class must be white or yellow")
        values = np.asarray(points, dtype=np.float64).reshape(-1, 2)
        if len(values) < 2:
            continue
        pixels = np.rint(grid.metric_to_pixel(values)).astype(np.int32)
        valid = (
            (pixels[:, 0] >= 0)
            & (pixels[:, 0] < canvas.shape[1])
            & (pixels[:, 1] >= 0)
            & (pixels[:, 1] < canvas.shape[0])
        )
        color = (
            ROAD_MARKING_COLORS_BGR[class_id]
            if state == "observed"
            else SHARED_FAMILY_STATE_COLORS_BGR[state]
        )
        thickness = (
            3
            if state == "observed"
            else 2
            if state == "structure_inferred"
            else 1
        )
        period = (
            1
            if state == "observed"
            else 2
            if state == "structure_inferred"
            else 3
        )
        for index in range(len(pixels) - 1):
            if index % period or not (valid[index] and valid[index + 1]):
                continue
            cv2.line(
                canvas,
                tuple(pixels[index]),
                tuple(pixels[index + 1]),
                color,
                thickness,
                cv2.LINE_AA,
            )

    legend = "SHARED  observed=lane color  inferred=orange dash  invalid=red dot"
    _label_at(canvas, legend, max(14, canvas.shape[0] - 25))
    if status_text:
        _label_at(canvas, str(status_text), max(14, canvas.shape[0] - 8))
    return canvas


def draw_lane_reprojection(image_bgr, lanes, camera, thickness=2):
    """Reproject metric lane polylines into a native camera debug image."""
    image = np.asarray(image_bgr)
    if image.shape != (camera.height, camera.width, 3) or image.dtype != np.uint8:
        raise ValueError("camera image differs from calibration")
    canvas = image.copy()
    for lane in lanes:
        class_id, points = lane
        if int(class_id) not in (1, 2):
            continue
        pixels, depth = camera.project_ground(points)
        _draw_projected_segments(
            canvas,
            pixels,
            depth,
            camera,
            ROAD_MARKING_COLORS_BGR[int(class_id)],
            int(thickness),
        )
    return canvas


def visualize_coverage(coverage: np.ndarray, grid: BevGridSpec) -> np.ndarray:
    value = _mono8(coverage, "coverage")
    canvas = np.zeros((*value.shape, 3), dtype=np.uint8)
    canvas[value > 0] = (85, 45, 20)
    draw_metric_grid(canvas, grid)
    return canvas


def draw_metric_grid(
    canvas: np.ndarray,
    grid: BevGridSpec,
    major_step_m: float = 10.0,
    minor_step_m: float = 5.0,
) -> None:
    if canvas.shape[:2] != grid.shape:
        raise ValueError("canvas shape does not match BEV grid")
    if minor_step_m <= 0.0 or major_step_m <= 0.0:
        return
    x_values = np.arange(
        np.ceil(grid.x_min_m / minor_step_m) * minor_step_m,
        grid.x_max_m + 0.5 * minor_step_m,
        minor_step_m,
    )
    y_values = np.arange(
        np.ceil(grid.y_min_m / minor_step_m) * minor_step_m,
        grid.y_max_m + 0.5 * minor_step_m,
        minor_step_m,
    )
    for x in x_values:
        endpoints = grid.metric_to_pixel([[x, grid.y_min_m], [x, grid.y_max_m]])
        major = np.isclose(x / major_step_m, round(x / major_step_m), atol=1e-6)
        color = (90, 90, 90) if major else (52, 52, 52)
        _line(canvas, endpoints[0], endpoints[1], color)
    for y in y_values:
        endpoints = grid.metric_to_pixel([[grid.x_min_m, y], [grid.x_max_m, y]])
        major = np.isclose(y / major_step_m, round(y / major_step_m), atol=1e-6)
        color = (90, 90, 90) if major else (52, 52, 52)
        _line(canvas, endpoints[0], endpoints[1], color)


def draw_ground_grid_overlay(
    image_bgr: np.ndarray,
    camera: CameraCalibration,
    x_min_m: float = -20.0,
    x_max_m: float = 60.0,
    y_min_m: float = -20.0,
    y_max_m: float = 20.0,
    longitudinal_step_m: float = 5.0,
    lateral_step_m: float = 2.0,
) -> np.ndarray:
    image = np.asarray(image_bgr)
    if image.shape != (camera.height, camera.width, 3) or image.dtype != np.uint8:
        raise ValueError(
            "camera image must be uint8 {}, got {} {}".format(
                (camera.height, camera.width, 3), image.shape, image.dtype
            )
        )
    canvas = image.copy()
    y_samples = np.linspace(y_min_m, y_max_m, 321)
    x_samples = np.linspace(x_min_m, x_max_m, 641)

    x_start = np.ceil(x_min_m / longitudinal_step_m) * longitudinal_step_m
    for x in np.arange(x_start, x_max_m + 0.5 * longitudinal_step_m, longitudinal_step_m):
        points = np.column_stack([np.full_like(y_samples, x), y_samples])
        pixels, depth = camera.project_ground(points)
        _draw_projected_segments(
            canvas, pixels, depth, camera, (80, 230, 255), thickness=1
        )

    y_start = np.ceil(y_min_m / lateral_step_m) * lateral_step_m
    for y in np.arange(y_start, y_max_m + 0.5 * lateral_step_m, lateral_step_m):
        points = np.column_stack([x_samples, np.full_like(x_samples, y)])
        pixels, depth = camera.project_ground(points)
        color = (255, 120, 220) if not np.isclose(y, 0.0) else (80, 255, 80)
        _draw_projected_segments(canvas, pixels, depth, camera, color, thickness=1)

    cv2.putText(
        canvas,
        "{}  HFOV {:.1f}  xyz={}  rpy={}".format(
            camera.name,
            camera.horizontal_fov_deg,
            tuple(round(x, 2) for x in camera.translation_m),
            tuple(round(x, 1) for x in camera.rotation_deg),
        ),
        (10, 22),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (0, 0, 0),
        3,
        cv2.LINE_AA,
    )
    cv2.putText(
        canvas,
        "{}  HFOV {:.1f}  xyz={}  rpy={}".format(
            camera.name,
            camera.horizontal_fov_deg,
            tuple(round(x, 2) for x in camera.translation_m),
            tuple(round(x, 1) for x in camera.rotation_deg),
        ),
        (10, 22),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    return canvas


def build_bev_mosaic(
    layers_by_view: Mapping[str, Mapping[str, np.ndarray]],
    grid: BevGridSpec,
    view_order: Sequence[str] = ("left", "front", "right", "rear"),
    tile_size: Tuple[int, int] = (300, 600),
    layer_names: Sequence[str] = ("drivable", "road_marking", "coverage"),
) -> np.ndarray:
    """모자이크는 layer_names 행 x view_order 열이다.

    secondary_head=lane 이면 두 번째 행을 "lane" 으로 준다. 없는 뷰/레이어는
    비우지 않고 "waiting" 타일을 그린다 — 죽은 뷰가 조용히 사라지지 않게 하려는
    의도적인 동작이다.
    """
    tile_width, tile_height = (int(tile_size[0]), int(tile_size[1]))
    if tile_width <= 0 or tile_height <= 0:
        raise ValueError("tile dimensions must be positive")
    rows = []
    for layer_name in layer_names:
        tiles = []
        for view in view_order:
            layers = layers_by_view.get(view)
            if layers is None or layer_name not in layers:
                tile = np.zeros((tile_height, tile_width, 3), dtype=np.uint8)
                label = "{} / {} / waiting".format(view, layer_name)
            else:
                value = layers[layer_name]
                if layer_name in ("drivable", "lane"):
                    color = colorize_drivable(value)
                elif layer_name == "road_marking":
                    color = colorize_road_marking(value)
                else:
                    color = visualize_coverage(value, grid)
                tile = cv2.resize(
                    color, (tile_width, tile_height), interpolation=cv2.INTER_NEAREST
                )
                label = "{} / {}".format(view, layer_name)
            _label(tile, label)
            tiles.append(tile)
        rows.append(np.concatenate(tiles, axis=1))
    return np.concatenate(rows, axis=0)


def _mono8(value: np.ndarray, name: str) -> np.ndarray:
    array = np.asarray(value)
    if array.ndim != 2 or array.dtype != np.uint8:
        raise ValueError("{} must be a two-dimensional uint8 array".format(name))
    return array


def _line(canvas, start, end, color, thickness=1) -> None:
    p0 = tuple(np.round(start).astype(int))
    p1 = tuple(np.round(end).astype(int))
    cv2.line(canvas, p0, p1, color, thickness, cv2.LINE_AA)


def _draw_projected_segments(
    canvas: np.ndarray,
    pixels: np.ndarray,
    depths: np.ndarray,
    camera: CameraCalibration,
    color,
    thickness: int,
) -> None:
    valid = (
        np.isfinite(pixels).all(axis=1)
        & (depths > 0.1)
        & (pixels[:, 0] >= 0.0)
        & (pixels[:, 0] < camera.width)
        & (pixels[:, 1] >= 0.0)
        & (pixels[:, 1] < camera.height)
    )
    start = None
    for index, is_valid in enumerate(valid):
        if is_valid and start is None:
            start = index
        at_end = index == len(valid) - 1
        if start is not None and ((not is_valid) or at_end):
            end = index + 1 if is_valid and at_end else index
            if end - start >= 2:
                points = np.round(pixels[start:end]).astype(np.int32).reshape(-1, 1, 2)
                cv2.polylines(canvas, [points], False, color, thickness, cv2.LINE_AA)
            start = None


def _label(tile: np.ndarray, text: str) -> None:
    font = cv2.FONT_HERSHEY_SIMPLEX
    base_scale = 0.5
    available_width = max(1, int(tile.shape[1]) - 16)
    text_width = cv2.getTextSize(text, font, base_scale, 1)[0][0]
    scale = min(base_scale, base_scale * available_width / max(1, text_width))
    baseline_y = max(12, int(round(8 + 28 * scale)))
    cv2.putText(
        tile, text, (8, baseline_y), font, scale, (0, 0, 0), 3, cv2.LINE_AA
    )
    cv2.putText(
        tile, text, (8, baseline_y), font, scale, (255, 255, 255), 1, cv2.LINE_AA
    )


def _label_at(tile: np.ndarray, text: str, baseline_y: int) -> None:
    font = cv2.FONT_HERSHEY_SIMPLEX
    base_scale = 0.38
    available_width = max(1, int(tile.shape[1]) - 16)
    text_width = cv2.getTextSize(text, font, base_scale, 1)[0][0]
    scale = min(base_scale, base_scale * available_width / max(1, text_width))
    y = int(np.clip(baseline_y, 10, max(10, tile.shape[0] - 2)))
    cv2.putText(tile, text, (8, y), font, scale, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(tile, text, (8, y), font, scale, (255, 255, 255), 1, cv2.LINE_AA)
