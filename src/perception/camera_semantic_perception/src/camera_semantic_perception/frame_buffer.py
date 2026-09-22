"""Bounded latest-only multi-camera frame buffer."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional


@dataclass(frozen=True)
class EncodedFrame:
    view: str
    stamp: object
    frame_id: str
    data: bytes
    received_monotonic: float
    received_ros_ns: int = 0


class LatestFrameBuffer:
    def __init__(self, views: Iterable[str], batch_wait_ms: float = 4.0):
        self.views = tuple(views)
        if not self.views or len(set(self.views)) != len(self.views):
            raise ValueError("views must be non-empty and unique")
        if batch_wait_ms < 0:
            raise ValueError("batch_wait_ms cannot be negative")
        self.batch_wait_seconds = float(batch_wait_ms) / 1000.0
        self._condition = threading.Condition()
        self._pending: Dict[str, EncodedFrame] = {}
        self._closed = False
        self.received = {view: 0 for view in self.views}
        self.replaced = {view: 0 for view in self.views}

    def push(self, frame: EncodedFrame) -> None:
        if frame.view not in self.received:
            raise ValueError("unknown view: {}".format(frame.view))
        with self._condition:
            if self._closed:
                return
            self.received[frame.view] += 1
            if frame.view in self._pending:
                self.replaced[frame.view] += 1
            self._pending[frame.view] = frame
            self._condition.notify()

    def pop_batch(self, timeout: Optional[float] = None) -> List[EncodedFrame]:
        with self._condition:
            deadline = None if timeout is None else time.monotonic() + timeout
            while not self._pending and not self._closed:
                remaining = None if deadline is None else deadline - time.monotonic()
                if remaining is not None and remaining <= 0:
                    return []
                self._condition.wait(remaining)
            if self._closed:
                return []

            coalesce_deadline = time.monotonic() + self.batch_wait_seconds
            while len(self._pending) < len(self.views) and not self._closed:
                remaining = coalesce_deadline - time.monotonic()
                if remaining <= 0:
                    break
                self._condition.wait(remaining)
            batch = [
                self._pending.pop(view)
                for view in self.views
                if view in self._pending
            ]
            return batch

    def close(self) -> None:
        with self._condition:
            self._closed = True
            self._pending.clear()
            self._condition.notify_all()
