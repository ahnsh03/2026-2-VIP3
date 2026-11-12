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
        height = length * pixels_per_m
        # Keep mirrors and the source aspect ratio instead of squeezing to body width.
        scale = height / sprite.shape[0]
        width = scale * sprite.shape[1]
        alpha = sprite[:, :, 3:4] / 255.0
        premultiplied = np.concatenate((sprite[:, :, :3] * alpha, alpha), axis=2)
        center_row = (float(config["center_x_m"]) + float(config["extent_m"]) / 2
                      - center_x) * pixels_per_m
        top = center_row - height / 2
        left = (self.size - width) / 2
        # Include the interpolation footprint, including for small sprites scaled up.
        support = scale / 2
        bottom = min(self.size, int(np.ceil(top + height + support)))
        right = min(self.size, int(np.ceil(left + width + support)))
        dest_top = max(0, int(np.floor(top - support)))
        dest_left = max(0, int(np.floor(left - support)))
        if dest_top >= bottom or dest_left >= right:
            raise ValueError("vehicle overlay lies outside the SVM extent")
        self.roi = (slice(dest_top, bottom), slice(dest_left, right))
        # Map source pixel centres to metric pixel boundaries without rounding the
        # vehicle size or centre. Supersampling retains fractional boundary coverage.
        # The ROI is cached once; per-frame apply() is still a small alpha blend.
        supersample = 4
        transform = np.array([
            [scale, 0, left - dest_left + scale / 2],
            [0, scale, top - dest_top + scale / 2]], dtype=np.float32) * supersample
        transform[:, 2] -= .5
        high_resolution = cv2.warpAffine(
            premultiplied, transform,
            ((right - dest_left) * supersample, (bottom - dest_top) * supersample),
            flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        clipped = cv2.resize(high_resolution, (right - dest_left, bottom - dest_top),
                             interpolation=cv2.INTER_AREA)
        self.color, self.alpha = clipped[:, :, :3], clipped[:, :, 3:4]
        self.metadata = {
            "image": str(image_path), "visualization_only": True,
            "source_crop_xyxy": [x0, y0, x1, y1],
            "length_m": length, "center_x_m": center_x,
            "width_with_mirrors_m": length * sprite.shape[1] / sprite.shape[0],
            "orientation": "front_up", "size_px": [width, height],
            "placement": "metric_subpixel_supersampled",
            "bounds_edge_px": [left, top, left + width, top + height],
            "front_x_m": center_x + length / 2,
            "rear_x_m": center_x - length / 2,
        }

    def apply(self, image):
        """Return a composited copy; input pixels and the geometric mask stay intact."""
        if image.shape != (self.size, self.size, 3) or image.dtype != np.uint8:
            raise ValueError("vehicle overlay requires an SVM-sized uint8 BGR image")
        result = image.copy()
        result[self.roi] = np.clip(np.rint(self.color + image[self.roi] * (1 - self.alpha)),
                                   0, 255).astype(np.uint8)
        return result


def draw_grid(image, size, extent, center_x):
    result = image.copy()
    for distance in range(-5, 6):
        row = int((center_x + extent / 2 - distance) * size / extent)
        col = int((extent / 2 - distance) * size / extent)
        if 0 <= row < size:
            cv2.line(result, (0, row), (size - 1, row), (75, 75, 75), 1)
        if 0 <= col < size:
            cv2.line(result, (col, 0), (col, size - 1), (75, 75, 75), 1)
    cv2.putText(result, "FRONT", (size // 2 - 28, 24), cv2.FONT_HERSHEY_SIMPLEX,
                .5, (0, 220, 255), 1, cv2.LINE_AA)
    return result


def decorate(projection, svm, overlay, compensated, lens_label, show_vehicle_mask, grid=False):
    image = draw_grid(projection, svm.size, svm.extent, svm.center_x) if grid else projection
    image = overlay.apply(image) if overlay else image.copy()
    cv2.putText(image, 'SVM PREVIEW | ' + ('POSE OK' if compensated else 'STATIC'),
                (8, svm.size - 12), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 180, 255), 1, cv2.LINE_AA)
    cv2.putText(image, lens_label, (8, svm.size - 52),
                cv2.FONT_HERSHEY_SIMPLEX, .48, (0, 180, 255), 1, cv2.LINE_AA)
    if not show_vehicle_mask:
        cv2.putText(image, 'BODY MASK OFF | DEBUG', (8, svm.size - 34),
                    cv2.FONT_HERSHEY_SIMPLEX, .48, (0, 180, 255), 1, cv2.LINE_AA)
    return image
