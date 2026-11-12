"""Timestamp selection and bounded JPEG decoding, independent of ROS."""
from dataclasses import dataclass
import cv2
import numpy as np

VIEWS = ("front", "rear", "left", "right")

@dataclass
class CameraFrame:
    image: np.ndarray
    stamp: float
    arrival: float
    frame_id: str


def select_frames(buffers, now, max_age, slop):
    """Select a fresh synchronized set; never keep displaying stale cameras.

    Header timestamps are compared only to each other (simulation clocks may
    differ from wall time). Zero timestamps use monotonic arrival times and
    are reported explicitly. A negative/expired arrival is never accepted.
    """
    fresh = {view: [f for f in buffers[view] if 0 <= now - f.arrival <= max_age]
             for view in VIEWS}
    if any(not frames for frames in fresh.values()):
        return None, "waiting/stale", None
    use_stamp = all(frames[-1].stamp > 0 for frames in fresh.values())
    clock = (lambda f: f.stamp) if use_stamp else (lambda f: f.arrival)
    anchor_view = min(VIEWS, key=lambda v: clock(fresh[v][-1]))
    for anchor in reversed(fresh[anchor_view]):
        selected = {v: min(fresh[v], key=lambda f: abs(clock(f) - clock(anchor))) for v in VIEWS}
        values = [clock(f) for f in selected.values()]
        span = max(values) - min(values)
        if span <= slop:
            return selected, "header" if use_stamp else "arrival (zero header)", span
    return None, "unsynchronized", None


@dataclass(frozen=True)
class EncodedCameraFrame:
    data: bytes
    stamp: float
    arrival: float
    frame_id: str


class SelectedFrameDecoder:
    """Single-worker cache: at most one decoded image per camera, never a backlog."""

    def __init__(self, cameras):
        self.cameras = cameras
        self.cache = {}
        self.decoded_count = 0

    def decode(self, frames):
        images = {}
        for view in VIEWS:
            frame = frames[view]
            cached = self.cache.get(view)
            if cached is not None and cached[0] is frame:
                images[view] = cached[1]
                continue
            try:
                image = cv2.imdecode(np.frombuffer(frame.data, np.uint8), cv2.IMREAD_COLOR)
            except cv2.error as exc:
                raise ValueError('{} invalid JPEG'.format(view)) from exc
            camera = self.cameras[view]
            if image is None or image.shape != (camera['height'], camera['width'], 3):
                raise ValueError('{} invalid JPEG or dimensions'.format(view))
            if frame.frame_id != camera['frame_id']:
                raise ValueError('{} frame mismatch'.format(view))
            self.cache[view] = (frame, image)
            self.decoded_count += 1
            images[view] = image
        return images
