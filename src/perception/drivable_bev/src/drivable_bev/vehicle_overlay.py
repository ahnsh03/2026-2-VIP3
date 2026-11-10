"""Metric, front-up vehicle artwork for display only; never alters SVM validity."""
import cv2
import numpy as np


class VehicleOverlay:
    def __init__(self, image_path, config):
        sprite = cv2.imread(str(image_path), cv2.IMREAD_UNCHANGED)
        if sprite is None:
            raise ValueError("cannot read vehicle overlay: {}".format(image_path))
        if sprite.ndim != 3 or sprite.shape[2] != 4 or sprite.dtype != np.uint8:
            raise ValueError("vehicle overlay must be an 8-bit BGRA image")
        # Ignore nearly invisible export residue when finding the vehicle bounds.
        rows, cols = np.where(sprite[:, :, 3] > 4)
        if not len(rows):
            raise ValueError("vehicle overlay has no visible pixels")
        x0, x1 = int(cols.min()), int(cols.max()) + 1
        y0, y1 = int(rows.min()), int(rows.max()) + 1
        sprite = sprite[y0:y1, x0:x1].astype(np.float32)
        spec = config["vehicle_geometry"]["documented_spec"]
        length = float(spec["length_m"])
        center_x = length / 2 - float(spec["rear_overhang_m"])
        self.size = int(config["output_size"])
        pixels_per_m = self.size / float(config["extent_m"])
        height = max(1, int(round(length * pixels_per_m)))
        # Keep mirrors and the source aspect ratio instead of squeezing to body width.
        width = max(1, int(round(height * sprite.shape[1] / sprite.shape[0])))
        alpha = sprite[:, :, 3:4] / 255.0
        premultiplied = np.concatenate((sprite[:, :, :3] * alpha, alpha), axis=2)
        interpolation = cv2.INTER_AREA if height < sprite.shape[0] else cv2.INTER_LINEAR
        resized = cv2.resize(premultiplied, (width, height), interpolation=interpolation)
        center_row = (float(config["center_x_m"]) + float(config["extent_m"]) / 2
                      - center_x) * pixels_per_m
        top = int(round(center_row - height / 2))
        left = int(round((self.size - width) / 2))
        bottom, right = min(self.size, top + height), min(self.size, left + width)
        dest_top, dest_left = max(0, top), max(0, left)
        if dest_top >= bottom or dest_left >= right:
            raise ValueError("vehicle overlay lies outside the SVM extent")
        self.roi = (slice(dest_top, bottom), slice(dest_left, right))
        clipped = resized[dest_top-top:bottom-top, dest_left-left:right-left]
        self.color, self.alpha = clipped[:, :, :3], clipped[:, :, 3:4]
        self.metadata = {
            "image": str(image_path), "visualization_only": True,
            "source_crop_xyxy": [x0, y0, x1, y1],
            "length_m": length, "center_x_m": center_x,
            "width_with_mirrors_m": length * sprite.shape[1] / sprite.shape[0],
            "orientation": "front_up", "size_px": [width, height],
        }

    def apply(self, image):
        """Return a composited copy; input pixels and the geometric mask stay intact."""
        if image.shape != (self.size, self.size, 3) or image.dtype != np.uint8:
            raise ValueError("vehicle overlay requires an SVM-sized uint8 BGR image")
        result = image.copy()
        result[self.roi] = np.clip(np.rint(self.color + image[self.roi] * (1 - self.alpha)),
                                   0, 255).astype(np.uint8)
        return result
