"""Shared BEV pixel-centre <-> base_link XY contract (front up, left left).

Pixels are (row, column), not OpenCV (u, v). Integer coordinates denote pixel
centres. The image covers a square extent centred at (center_x_m, 0).
"""
import numpy as np


def _validate(size, extent_m):
    if int(size) != size or size <= 0 or not np.isfinite(extent_m) or extent_m <= 0:
        raise ValueError('positive integer size and finite positive extent required')


def pixel_to_vehicle(row, col, size, extent_m, center_x_m):
    _validate(size, extent_m)
    row, col = np.asarray(row), np.asarray(col)
    return (center_x_m + extent_m / 2 - (row + .5) * extent_m / size,
            extent_m / 2 - (col + .5) * extent_m / size)


def vehicle_to_pixel(x, y, size, extent_m, center_x_m):
    _validate(size, extent_m)
    x, y = np.asarray(x), np.asarray(y)
    return ((center_x_m + extent_m / 2 - x) * size / extent_m - .5,
            (extent_m / 2 - y) * size / extent_m - .5)
