"""One vehicle reference for camera heights, ground projection and body masking."""
import copy
import math
from pathlib import Path

import yaml

from .calibration import GroundPlane


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
        raise ValueError('vehicle geometry was not loaded; use load_svm_config or fisheye_svm.launch')
    # Historical captures already contain their original flat geometry values.
    return result


def load_svm_config(path):
    path = Path(path)
    config = yaml.safe_load(path.read_text())
    if 'vehicle_geometry_file' in config and 'vehicle_geometry' not in config:
        config['vehicle_geometry'] = yaml.safe_load(
            (path.parent / config['vehicle_geometry_file']).read_text())
    return resolve_svm_geometry(config)
