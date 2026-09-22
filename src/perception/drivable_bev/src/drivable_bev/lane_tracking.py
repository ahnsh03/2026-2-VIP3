"""Associate metric lane nodes into same-class longitudinal tracklets."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from .lane_evidence import LaneNode


@dataclass(frozen=True)
class LaneTrack:
    nodes: tuple

    @property
    def class_id(self) -> int:
        return int(self.nodes[0].class_id)

    @property
    def xy(self) -> np.ndarray:
        return np.asarray([node.xy for node in self.nodes], dtype=np.float64)


def _predict_y(nodes, x):
    if len(nodes) < 3:
        return float(nodes[-1].xy[1]), 0.0
    recent = np.asarray([node.xy for node in nodes[-6:]], dtype=np.float64)
    centered_x = recent[:, 0] - float(np.mean(recent[:, 0]))
    denominator = float(np.dot(centered_x, centered_x))
    slope = (
        0.0
        if denominator <= 1e-9
        else float(np.dot(centered_x, recent[:, 1] - np.mean(recent[:, 1])))
        / denominator
    )
    intercept = float(np.mean(recent[:, 1]) - slope * np.mean(recent[:, 0]))
    return slope * float(x) + intercept, slope


def associate_lane_nodes_with_audit(
    nodes,
    max_track_gap_m: float = 2.0,
    max_lateral_residual_m: float = 0.45,
    max_heading_change_deg: float = 20.0,
    min_track_nodes: int = 4,
    min_observed_span_m: float = 2.0,
) -> tuple[list, dict]:
    """Greedily associate nodes with deterministic one-to-one assignments."""
    if max_track_gap_m <= 0.0 or max_lateral_residual_m <= 0.0:
        raise ValueError("association distances must be positive")
    if not 0.0 < max_heading_change_deg < 90.0:
        raise ValueError("heading change must lie between zero and 90 degrees")
    if min_track_nodes < 2 or min_observed_span_m <= 0.0:
        raise ValueError("track support constraints are invalid")
    completed = []
    counters = {
        "input_nodes": int(len(nodes)),
        "candidate_pairs": 0,
        "rejected_nonforward_or_gap": 0,
        "rejected_lateral_residual": 0,
        "rejected_heading_change": 0,
        "assignment_conflicts": 0,
        "raw_tracklets": 0,
        "rejected_too_few_nodes": 0,
        "rejected_short_span": 0,
        "accepted_tracks": 0,
    }
    for class_id in sorted(set(int(node.class_id) for node in nodes)):
        values = [node for node in nodes if int(node.class_id) == class_id]
        by_bin = {}
        for node in values:
            by_bin.setdefault(int(node.x_bin), []).append(node)
        active = []
        for x_bin in sorted(by_bin):
            current = sorted(by_bin[x_bin], key=lambda node: float(node.xy[1]))
            x_value = float(np.mean([node.xy[0] for node in current]))
            still_active = []
            for track in active:
                if x_value - float(track[-1].xy[0]) <= max_track_gap_m:
                    still_active.append(track)
                else:
                    completed.append(track)
            active = still_active

            candidates = []
            for track_index, track in enumerate(active):
                predicted_y, previous_slope = _predict_y(track, x_value)
                for node_index, node in enumerate(current):
                    counters["candidate_pairs"] += 1
                    dx = float(node.xy[0] - track[-1].xy[0])
                    if dx <= 1e-6 or dx > max_track_gap_m:
                        counters["rejected_nonforward_or_gap"] += 1
                        continue
                    residual = abs(float(node.xy[1]) - predicted_y)
                    if residual > max_lateral_residual_m:
                        counters["rejected_lateral_residual"] += 1
                        continue
                    recent = np.asarray(
                        [value.xy for value in track[-5:]] + [node.xy],
                        dtype=np.float64,
                    )
                    centered_x = recent[:, 0] - float(np.mean(recent[:, 0]))
                    denominator = float(np.dot(centered_x, centered_x))
                    new_slope = (
                        previous_slope
                        if denominator <= 1e-9
                        else float(
                            np.dot(
                                centered_x,
                                recent[:, 1] - np.mean(recent[:, 1]),
                            )
                            / denominator
                        )
                    )
                    heading_change = abs(
                        math.degrees(math.atan(new_slope) - math.atan(previous_slope))
                    )
                    if heading_change > max_heading_change_deg:
                        counters["rejected_heading_change"] += 1
                        continue
                    candidates.append((residual, heading_change, track_index, node_index))
            assigned_tracks = set()
            assigned_nodes = set()
            for _, _, track_index, node_index in sorted(candidates):
                if track_index in assigned_tracks or node_index in assigned_nodes:
                    counters["assignment_conflicts"] += 1
                    continue
                active[track_index].append(current[node_index])
                assigned_tracks.add(track_index)
                assigned_nodes.add(node_index)
            for node_index, node in enumerate(current):
                if node_index not in assigned_nodes:
                    active.append([node])
        completed.extend(active)

    result = []
    counters["raw_tracklets"] = int(len(completed))
    for track in completed:
        ordered = tuple(sorted(track, key=lambda node: float(node.xy[0])))
        span = float(ordered[-1].xy[0] - ordered[0].xy[0])
        if len(ordered) < min_track_nodes:
            counters["rejected_too_few_nodes"] += 1
            continue
        if span < min_observed_span_m:
            counters["rejected_short_span"] += 1
            continue
        result.append(LaneTrack(ordered))
    result = sorted(
        result,
        key=lambda track: (
            track.class_id,
            float(track.nodes[0].xy[1]),
            float(track.nodes[0].xy[0]),
        ),
    )
    counters["accepted_tracks"] = int(len(result))
    counters["by_class"] = {
        str(class_id): {
            "input_nodes": int(sum(int(node.class_id) == class_id for node in nodes)),
            "accepted_tracks": int(sum(track.class_id == class_id for track in result)),
        }
        for class_id in sorted(set(int(node.class_id) for node in nodes))
    }
    return result, counters


def associate_lane_nodes(
    nodes,
    max_track_gap_m: float = 2.0,
    max_lateral_residual_m: float = 0.45,
    max_heading_change_deg: float = 20.0,
    min_track_nodes: int = 4,
    min_observed_span_m: float = 2.0,
) -> list:
    result, _ = associate_lane_nodes_with_audit(
        nodes,
        max_track_gap_m=max_track_gap_m,
        max_lateral_residual_m=max_lateral_residual_m,
        max_heading_change_deg=max_heading_change_deg,
        min_track_nodes=min_track_nodes,
        min_observed_span_m=min_observed_span_m,
    )
    return result
