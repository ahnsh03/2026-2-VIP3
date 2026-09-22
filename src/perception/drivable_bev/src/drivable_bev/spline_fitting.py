"""Robust dependency-light parametric cubic B-spline lane fitting."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .lane_tracking import LaneTrack


@dataclass(frozen=True)
class FittedLane:
    lane_id: int
    class_id: int
    control_points: np.ndarray
    knots: np.ndarray
    degree: int
    sample_points: np.ndarray
    sample_observed: np.ndarray
    confidence: float
    source_mask: int
    valid_x_min_m: float
    valid_x_max_m: float
    observed_length_m: float
    bridged_length_ratio: float
    residual_p50_m: float
    residual_p95_m: float


def _open_uniform_knots(control_count: int, degree: int) -> np.ndarray:
    interior_count = control_count - degree - 1
    interior = (
        np.linspace(0.0, 1.0, interior_count + 2, dtype=np.float64)[1:-1]
        if interior_count > 0
        else np.empty(0, dtype=np.float64)
    )
    return np.concatenate(
        [np.zeros(degree + 1), interior, np.ones(degree + 1)]
    )


def bspline_basis(parameters, knots, degree, control_count):
    values = np.asarray(parameters, dtype=np.float64).reshape(-1)
    interval_count = len(knots) - 1
    basis = np.zeros((len(values), interval_count), dtype=np.float64)
    for index in range(interval_count):
        basis[:, index] = (
            (values >= knots[index]) & (values < knots[index + 1])
        ).astype(np.float64)
    for current_degree in range(1, degree + 1):
        next_count = len(knots) - current_degree - 1
        next_basis = np.zeros((len(values), next_count), dtype=np.float64)
        for index in range(next_count):
            left_denominator = knots[index + current_degree] - knots[index]
            if left_denominator > 0.0:
                next_basis[:, index] += (
                    (values - knots[index]) / left_denominator * basis[:, index]
                )
            right_denominator = (
                knots[index + current_degree + 1] - knots[index + 1]
            )
            if right_denominator > 0.0:
                next_basis[:, index] += (
                    (knots[index + current_degree + 1] - values)
                    / right_denominator
                    * basis[:, index + 1]
                )
        basis = next_basis
    result = basis[:, :control_count]
    result[values >= 1.0, :] = 0.0
    result[values >= 1.0, -1] = 1.0
    return result


def _fit_control_points(points, parameters, weights, control_count, degree, smoothing):
    knots = _open_uniform_knots(control_count, degree)
    basis = bspline_basis(parameters, knots, degree, control_count)
    root_weight = np.sqrt(np.maximum(weights, 1e-9))[:, None]
    design = basis * root_weight
    target = points * root_weight
    if control_count >= 3 and smoothing > 0.0:
        regularizer = np.diff(np.eye(control_count), n=2, axis=0)
        design = np.vstack([design, np.sqrt(smoothing) * regularizer])
        target = np.vstack([target, np.zeros((len(regularizer), 2))])
    control_points = np.linalg.lstsq(design, target, rcond=None)[0]
    return knots, control_points, basis


def _resample_curve(points, spacing_m):
    dense = np.asarray(points, dtype=np.float64)
    if len(dense) < 2:
        return dense.copy()
    distance = np.linalg.norm(np.diff(dense, axis=0), axis=1)
    cumulative = np.concatenate([[0.0], np.cumsum(distance)])
    if cumulative[-1] <= 1e-9:
        return dense[:1].copy()
    targets = np.arange(0.0, cumulative[-1], spacing_m)
    if not len(targets) or targets[-1] < cumulative[-1]:
        targets = np.append(targets, cumulative[-1])
    return np.column_stack(
        [np.interp(targets, cumulative, dense[:, axis]) for axis in range(2)]
    )


def fit_lane_track(
    track: LaneTrack,
    lane_id: int,
    smoothing: float = 0.05,
    sample_spacing_m: float = 0.25,
    robust_iterations: int = 3,
    outlier_scale: float = 2.5,
    control_spacing_m: float = 2.0,
) -> FittedLane:
    points = track.xy
    if len(points) < 4:
        raise ValueError("cubic lane fitting requires at least four nodes")
    if smoothing < 0.0 or sample_spacing_m <= 0.0 or control_spacing_m <= 0.0:
        raise ValueError("spline parameters are invalid")
    distance = np.linalg.norm(np.diff(points, axis=0), axis=1)
    cumulative = np.concatenate([[0.0], np.cumsum(distance)])
    if cumulative[-1] <= 1e-6:
        raise ValueError("lane track has no measurable length")
    parameters = cumulative / cumulative[-1]
    degree = min(3, len(points) - 1)
    control_count = int(np.clip(np.ceil(cumulative[-1] / control_spacing_m) + degree, 4, len(points)))
    base_weights = np.asarray(
        [
            max(1e-6, node.confidence * node.geometric_quality * np.sqrt(node.support))
            for node in track.nodes
        ],
        dtype=np.float64,
    )
    robust_weights = np.ones(len(points), dtype=np.float64)
    knots = None
    control_points = None
    basis = None
    residual = np.zeros(len(points), dtype=np.float64)
    for _ in range(max(1, int(robust_iterations))):
        knots, control_points, basis = _fit_control_points(
            points,
            parameters,
            base_weights * robust_weights,
            control_count,
            degree,
            smoothing,
        )
        fitted = basis @ control_points
        residual = np.linalg.norm(points - fitted, axis=1)
        median = float(np.median(residual))
        mad = float(np.median(np.abs(residual - median)))
        scale = max(1.4826 * mad, 0.02)
        normalized = residual / max(outlier_scale * scale, 1e-6)
        robust_weights = 1.0 / (1.0 + normalized * normalized)

    dense_parameter = np.linspace(0.0, 1.0, max(100, len(points) * 12))
    dense = bspline_basis(
        dense_parameter, knots, degree, control_count
    ) @ control_points
    samples = _resample_curve(dense, sample_spacing_m)
    # A smoothing B-spline is not required to interpolate its first and last
    # evidence nodes.  Advertise only the longitudinal interval covered by
    # both the evidence and the fitted curve; otherwise a downstream consumer
    # can receive a valid_x interval for which no public polyline exists.
    evidence_x_min = float(np.min(points[:, 0]))
    evidence_x_max = float(np.max(points[:, 0]))
    sample_x_min = float(np.min(samples[:, 0]))
    sample_x_max = float(np.max(samples[:, 0]))
    valid_x_min = max(evidence_x_min, sample_x_min)
    valid_x_max = min(evidence_x_max, sample_x_max)
    if valid_x_max < valid_x_min:
        raise ValueError("fitted lane has no longitudinal overlap with its evidence")
    dx = np.diff(points[:, 0])
    positive_dx = dx[dx > 1e-6]
    normal_step = float(np.median(positive_dx)) if len(positive_dx) else 0.0
    observed_gap_limit = max(0.60, 2.5 * normal_step)
    sample_observed = np.zeros(len(samples), dtype=bool)
    sample_observed[0] = sample_observed[-1] = True
    for start, end in zip(points[:-1], points[1:]):
        if end[0] - start[0] > observed_gap_limit:
            continue
        sample_observed |= (
            (samples[:, 0] >= min(start[0], end[0]) - 1e-6)
            & (samples[:, 0] <= max(start[0], end[0]) + 1e-6)
        )
    sample_distance = np.linalg.norm(np.diff(samples, axis=0), axis=1)
    observed_segments = sample_observed[:-1] & sample_observed[1:]
    observed_length = float(np.sum(sample_distance[observed_segments]))
    bridged_length = float(np.sum(sample_distance[~observed_segments]))
    bridged_ratio = bridged_length / max(observed_length + bridged_length, 1e-9)
    source_mask = int(
        np.bitwise_or.reduce(
            np.asarray([node.source_mask for node in track.nodes], dtype=np.uint8)
        )
    )
    confidence = float(
        np.average(
            [node.confidence for node in track.nodes],
            weights=np.maximum(base_weights, 1e-6),
        )
    )
    return FittedLane(
        lane_id=int(lane_id),
        class_id=track.class_id,
        control_points=np.ascontiguousarray(control_points, dtype=np.float64),
        knots=np.ascontiguousarray(knots, dtype=np.float64),
        degree=degree,
        sample_points=np.ascontiguousarray(samples, dtype=np.float64),
        sample_observed=np.ascontiguousarray(sample_observed),
        confidence=confidence,
        source_mask=source_mask,
        valid_x_min_m=valid_x_min,
        valid_x_max_m=valid_x_max,
        observed_length_m=observed_length,
        bridged_length_ratio=float(bridged_ratio),
        residual_p50_m=float(np.percentile(residual, 50)),
        residual_p95_m=float(np.percentile(residual, 95)),
    )


def fit_lane_tracks_with_audit(tracks, **kwargs) -> tuple[list, dict]:
    ordered = sorted(
        tracks,
        key=lambda track: (
            track.class_id,
            float(np.median(track.xy[:, 1])),
            float(track.xy[0, 0]),
        ),
    )
    result = []
    rejected = {
        "value_error": 0,
        "linear_algebra_error": 0,
    }
    for lane_id, track in enumerate(ordered, 1):
        try:
            result.append(fit_lane_track(track, lane_id=lane_id, **kwargs))
        except ValueError:
            rejected["value_error"] += 1
        except np.linalg.LinAlgError:
            rejected["linear_algebra_error"] += 1
    return result, {
        "input_tracks": int(len(ordered)),
        "accepted_lanes": int(len(result)),
        "rejected": rejected,
    }


def fit_lane_tracks(tracks, **kwargs) -> list:
    result, _ = fit_lane_tracks_with_audit(tracks, **kwargs)
    return result
