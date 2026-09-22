"""Reduce dense metric road-marking evidence to deterministic lane nodes."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .model_grid_projection import MetricEvidence


@dataclass(frozen=True)
class LaneNode:
    xy: np.ndarray
    class_id: int
    confidence: float
    geometric_quality: float
    source_mask: int
    support: int
    x_bin: int


def reduce_evidence_to_nodes(
    evidence: MetricEvidence,
    x_bin_m: float = 0.25,
    lateral_cluster_gap_m: float = 0.30,
    x_origin_m: float = -5.0,
) -> list:
    """Collapse line thickness and cross-view duplicates into metric nodes."""
    if x_bin_m <= 0.0 or lateral_cluster_gap_m <= 0.0:
        raise ValueError("node bin and lateral gap must be positive")
    if not len(evidence.xy):
        return []
    weights = np.asarray(evidence.confidence, dtype=np.float64) * np.asarray(
        evidence.geometric_quality, dtype=np.float64
    )
    weights = np.maximum(weights, 1e-6)
    bins = np.floor(
        (np.asarray(evidence.xy[:, 0], dtype=np.float64) - float(x_origin_m))
        / float(x_bin_m)
    ).astype(np.int64)
    # Sort once by class, longitudinal bin and lateral position.  The previous
    # implementation rescanned every class array for every bin, which made the
    # live cost grow sharply with thick segmentation masks.
    classes = np.asarray(evidence.class_id, dtype=np.uint8)
    xy = np.asarray(evidence.xy, dtype=np.float64)
    order = np.lexsort((xy[:, 1], bins, classes))
    ordered_class = classes[order]
    ordered_bin = bins[order]
    ordered_xy = xy[order]
    ordered_weight = weights[order]
    split = (
        (ordered_class[1:] != ordered_class[:-1])
        | (ordered_bin[1:] != ordered_bin[:-1])
        | (np.diff(ordered_xy[:, 1]) > lateral_cluster_gap_m)
    )
    starts = np.concatenate(([0], np.flatnonzero(split) + 1))
    ends = np.concatenate((starts[1:], [len(order)]))
    total = np.add.reduceat(ordered_weight, starts)
    weighted_x = np.add.reduceat(ordered_xy[:, 0] * ordered_weight, starts)
    weighted_y = np.add.reduceat(ordered_xy[:, 1] * ordered_weight, starts)
    confidence = np.add.reduceat(
        np.asarray(evidence.confidence, dtype=np.float64)[order] * ordered_weight,
        starts,
    ) / total
    quality = np.add.reduceat(
        np.asarray(evidence.geometric_quality, dtype=np.float64)[order]
        * ordered_weight,
        starts,
    ) / total
    source_mask = np.bitwise_or.reduceat(
        np.asarray(evidence.source_mask, dtype=np.uint8)[order], starts
    )
    result = [
        LaneNode(
            xy=np.asarray(
                [weighted_x[index] / total[index], weighted_y[index] / total[index]],
                dtype=np.float64,
            ),
            class_id=int(ordered_class[start]),
            confidence=float(confidence[index]),
            geometric_quality=float(quality[index]),
            source_mask=int(source_mask[index]),
            support=int(ends[index] - start),
            x_bin=int(ordered_bin[start]),
        )
        for index, start in enumerate(starts)
    ]
    return sorted(
        result,
        key=lambda node: (node.class_id, node.x_bin, float(node.xy[1])),
    )
