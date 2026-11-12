"""One vehicle reference for camera heights, ground projection and body masking."""
import copy
import math
from pathlib import Path

import yaml

from dataclasses import dataclass
from typing import Dict, Tuple
import numpy as np

@dataclass(frozen=True)
class GroundPlane:
    """Planar road surface expressed in ``base_link`` coordinates.

    The MORAI vehicle reference is at the rear-axle centre, not on the road
    surface.  Keeping that vertical offset explicit prevents camera mounts
    from being silently reinterpreted as heights above the road.
    """

    frame_id: str
    z_at_origin_m: float
    dz_dx: float
    dz_dy: float
    source: str
    model: str = "plane"

    def __post_init__(self) -> None:
        if self.frame_id != "base_link":
            raise ValueError("ground plane frame_id must be base_link")
        if self.model != "plane":
            raise ValueError("ground plane model must be plane")
        coefficients = np.asarray(self.coefficients, dtype=np.float64)
        if not np.isfinite(coefficients).all():
            raise ValueError("ground plane coefficients must be finite")
        if not self.source:
            raise ValueError("ground plane source must not be empty")

    @property
    def coefficients(self) -> Tuple[float, float, float]:
        """Return ``(a, b, c)`` for ``z = a*x + b*y + c``."""
        return (float(self.dz_dx), float(self.dz_dy), float(self.z_at_origin_m))

    def xyz(self, points_xy: np.ndarray) -> np.ndarray:
        points = np.asarray(points_xy, dtype=np.float64).reshape(-1, 2)
        a, b, c = self.coefficients
        z = a * points[:, 0] + b * points[:, 1] + c
        return np.column_stack([points, z])

    def to_mapping(self) -> Dict[str, object]:
        return {
            "frame_id": self.frame_id,
            "model": self.model,
            "z_at_origin_m": float(self.z_at_origin_m),
            "dz_dx": float(self.dz_dx),
            "dz_dy": float(self.dz_dy),
            "source": self.source,
        }


def vehicle_geometry(document):
    if document['frame_id'] != 'base_link' or document['origin'] != 'rear_axle_midpoint':
        raise ValueError('vehicle geometry requires the rear-axle base_link origin')
    spec = document['documented_spec']
    values = [float(spec[key]) for key in
              ('length_m', 'width_m', 'wheelbase_m', 'front_overhang_m', 'rear_overhang_m')]
    length, width, wheelbase, front_overhang, rear_overhang = values
    if not all(math.isfinite(v) and v > 0 for v in values):
        raise ValueError('vehicle dimensions must be finite and positive')
    if not math.isclose(length, wheelbase + front_overhang + rear_overhang, abs_tol=1e-5):
        raise ValueError('length differs from wheelbase + front/rear overhangs')
    ground = document['ground_reference']
    height = float(ground['base_link_height_m'])
    if not math.isfinite(height) or height <= 0:
        raise ValueError('base_link height must be finite and positive')
    plane = GroundPlane('base_link', -height, float(ground['dz_dx']),
                        float(ground['dz_dy']), str(ground['source']))
    margin = float(document['mask']['margin_m'])
    if not math.isfinite(margin) or margin < 0:
        raise ValueError('body mask margin must be finite and nonnegative')
    front = wheelbase + front_overhang
    return {
        'center_x_m': (front - rear_overhang) / 2,
        'ground_z_m': plane.z_at_origin_m,
        'ground_plane': plane.to_mapping(),
        'vehicle_mask': dict(x_min_m=-rear_overhang-margin, x_max_m=front+margin,
                             y_min_m=-width/2-margin, y_max_m=width/2+margin),
    }


def resolve_svm_geometry(config):
    result = copy.deepcopy(config)
    if 'vehicle_geometry' in result:
        result.update(vehicle_geometry(result['vehicle_geometry']))
    elif 'vehicle_geometry_file' in result:
        raise ValueError('vehicle geometry was not loaded; use load_svm_config or svm.launch')
    # Historical captures already contain their original flat geometry values.
    return result


def load_svm_config(path):
    path = Path(path)
    config = yaml.safe_load(path.read_text())
    if 'vehicle_geometry_file' in config and 'vehicle_geometry' not in config:
        config['vehicle_geometry'] = yaml.safe_load(
            (path.parent / config['vehicle_geometry_file']).read_text())
    return resolve_svm_geometry(config)
