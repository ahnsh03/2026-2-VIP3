#!/usr/bin/env python3
"""Reproduce a capture with its exact plane/motion; export individual camera warps."""
import argparse
import json
from pathlib import Path
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/vip3_perception/src'))
from vip3_perception.svm.projection import FisheyeSvm
from vip3_perception.svm.frames import VIEWS
from vip3_perception.svm.vehicle_geometry import load_svm_config
from vip3_perception.svm.pose import offset_ground_plane


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture_dir', type=Path)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--config', help='optional calibrated YAML instead of capture config')
    parser.add_argument('--static', action='store_true', help='compare without attitude/time compensation')
    args = parser.parse_args()
    if args.output_dir.exists():
        parser.error('use a new output directory to preserve measurements')
    captured_config = load_svm_config(args.capture_dir / 'config.yaml')
    config = load_svm_config(args.config) if args.config else captured_config
    old_convention = captured_config.get('pose_compensation', {}).get('quaternion_convention', 'ros_xyzw')
    new_convention = config.get('pose_compensation', {}).get('quaternion_convention', 'ros_xyzw')
    if args.config and not args.static and old_convention != new_convention:
        parser.error('pose convention differs from capture; record new inputs with the new profile. '
                     '--static only compares lenses without pose compensation')
    svm = FisheyeSvm(config)
    report = json.loads((args.capture_dir / 'capture.json').read_text())
    state = report.get('projection_state')
    if not args.static:
        if state is None:
            parser.error('capture predates pose metadata; explicitly use --static')
        motion = {v: (np.array(m['rotation']), np.array(m['translation'])) for v, m in state['motion'].items()}
        # Assumed planes include the old paint offset. Observed planes already
        # describe the visible surface and must not receive another offset.
        observed_ground = report.get('ground_refinement', {}).get('source', '').startswith('vision')
        delta = (0. if observed_ground else
                 float(config.get('projection_surface_offset_m', 0.))
                 - float(captured_config.get('projection_surface_offset_m', 0.)))
        svm.set_projection(offset_ground_plane(state['plane'], delta), motion)
    images = {v: cv2.imread(str(args.capture_dir / (v+'.png'))) for v in VIEWS}
    projection = svm.render(images)
    args.output_dir.mkdir(parents=True)
    cv2.imwrite(str(args.output_dir / 'svm_projection.png'), projection)
    cv2.imwrite(str(args.output_dir / 'valid_mask.png'), svm.valid_mask)
    for view in VIEWS:
        mx, my, weight = svm.maps[view]
        warped = cv2.remap(images[view], mx, my, cv2.INTER_LINEAR)
        warped[weight <= 1e-6] = 0
        cv2.imwrite(str(args.output_dir / (view+'_warped.png')), warped)
    (args.output_dir / 'replay.json').write_text(json.dumps(dict(
        source=str(args.capture_dir), static=args.static, plane=svm.active_plane,
        calibration_status=config['calibration_status'],
        calibration_profile=config.get('calibration_profile', 'unspecified'),
        ground_refinement=report.get('ground_refinement') if not args.static else None,
        projection_surface_offset_m=config.get('projection_surface_offset_m', 0.)), indent=2)+'\n')


if __name__ == '__main__':
    main()
