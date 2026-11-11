"""Keep JPEGs compressed until a synchronized SVM frame set is selected."""
from dataclasses import dataclass

import cv2
import numpy as np

from .fisheye_svm import VIEWS


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
