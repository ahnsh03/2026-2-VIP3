"""Thread-safe per-view counters and bounded latency statistics."""

from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, Mapping, Union

import numpy as np


def source_time_gate(stamp_ns: int, next_output_ns: int, period_ns: int):
    """Return (publish, next deadline) without accumulating source-time drift."""
    if stamp_ns <= 0 or next_output_ns < 0 or period_ns <= 0:
        raise ValueError("source timestamp and period must be positive")
    if next_output_ns and stamp_ns < next_output_ns:
        return False, next_output_ns
    if not next_output_ns:
        return True, stamp_ns + period_ns
    periods = max(1, (stamp_ns - next_output_ns) // period_ns + 1)
    return True, next_output_ns + periods * period_ns


def source_time_gate_with_reset(
    stamp_ns: int,
    last_input_ns: int,
    next_output_ns: int,
    period_ns: int,
    reset_regression_ns: int = None,
):
    """Rate gate that starts a new epoch after a substantial stamp regression.

    Small out-of-order or duplicate samples remain rejected. A regression of
    at least one output period (or an explicit threshold) represents simulator
    reset or sequential rosbag replay and resets only the source-time deadline.
    """
    if stamp_ns <= 0 or last_input_ns < 0 or next_output_ns < 0 or period_ns <= 0:
        raise ValueError("source timestamp and period must be positive")
    threshold = period_ns if reset_regression_ns is None else int(reset_regression_ns)
    if threshold <= 0:
        raise ValueError("reset regression threshold must be positive")
    reset = bool(last_input_ns and stamp_ns + threshold <= last_input_ns)
    if last_input_ns and stamp_ns <= last_input_ns and not reset:
        return False, next_output_ns, False
    if reset:
        next_output_ns = 0
    publish, deadline = source_time_gate(stamp_ns, next_output_ns, period_ns)
    return publish, deadline, reset


@dataclass
class _ViewStats:
    input_pairs: int = 0
    outputs: int = 0
    throttled: int = 0
    errors: int = 0
    last_input_stamp_ns: int = 0
    last_output_source_stamp_ns: int = 0
    last_result_stamp_ns: int = 0
    projection_ms: Deque[float] = field(default_factory=lambda: deque(maxlen=512))
    publish_ms: Deque[float] = field(default_factory=lambda: deque(maxlen=512))
    total_ms: Deque[float] = field(default_factory=lambda: deque(maxlen=512))
    output_times: Deque[float] = field(default_factory=lambda: deque(maxlen=256))


class PerformanceMonitor:
    def __init__(self, views) -> None:
        self._stats: Dict[str, _ViewStats] = {str(view): _ViewStats() for view in views}
        self._lock = threading.Lock()

    def count_input(self, view: str, source_stamp_ns: int) -> None:
        with self._lock:
            stats = self._stats[view]
            stats.input_pairs += 1
            stats.last_input_stamp_ns = int(source_stamp_ns)

    def count_throttled(self, view: str) -> None:
        with self._lock:
            self._stats[view].throttled += 1

    def count_error(self, view: str) -> None:
        with self._lock:
            self._stats[view].errors += 1

    def count_output(
        self,
        view: str,
        projection_ms: float,
        publish_ms: float,
        source_stamp_ns: int,
        result_stamp_ns: int,
    ) -> None:
        with self._lock:
            stats = self._stats[view]
            stats.outputs += 1
            stats.projection_ms.append(float(projection_ms))
            stats.publish_ms.append(float(publish_ms))
            stats.total_ms.append(float(projection_ms) + float(publish_ms))
            stats.output_times.append(time.monotonic())
            stats.last_output_source_stamp_ns = int(source_stamp_ns)
            stats.last_result_stamp_ns = int(result_stamp_ns)

    def snapshot(self) -> Mapping[str, Mapping[str, Union[int, float]]]:
        with self._lock:
            return {name: self._snapshot_one(stats) for name, stats in self._stats.items()}

    @staticmethod
    def _snapshot_one(stats: _ViewStats) -> Mapping[str, Union[int, float]]:
        projection = np.asarray(stats.projection_ms, dtype=np.float64)
        publish = np.asarray(stats.publish_ms, dtype=np.float64)
        total = np.asarray(stats.total_ms, dtype=np.float64)
        fps = 0.0
        if len(stats.output_times) >= 2:
            elapsed = stats.output_times[-1] - stats.output_times[0]
            if elapsed > 0.0:
                fps = (len(stats.output_times) - 1) / elapsed

        def value(array: np.ndarray, percentile: float = 0.0) -> float:
            if array.size == 0:
                return 0.0
            return float(np.percentile(array, percentile)) if percentile else float(array.mean())

        return {
            "input_pairs": stats.input_pairs,
            "outputs": stats.outputs,
            "throttled": stats.throttled,
            "errors": stats.errors,
            "last_input_stamp_ns": stats.last_input_stamp_ns,
            "last_output_source_stamp_ns": stats.last_output_source_stamp_ns,
            "last_result_stamp_ns": stats.last_result_stamp_ns,
            "processing_fps": fps,
            "projection_ms_mean": value(projection),
            "projection_ms_p95": value(projection, 95.0),
            "publish_ms_mean": value(publish),
            "publish_ms_p95": value(publish, 95.0),
            "total_ms_mean": value(total),
            "total_ms_p95": value(total, 95.0),
        }
