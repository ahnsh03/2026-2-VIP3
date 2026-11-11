#!/usr/bin/env python3
"""Fit inspected ground anchors; write a candidate only if held-out checks pass."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import yaml

for package in ('drivable_bev', 'camera_semantic_perception'):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/perception' / package / 'src'))
from drivable_bev.svm_calibration import fit_ground_calibration
from drivable_bev.vehicle_geometry import load_svm_config


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default='src/perception/drivable_bev/config/fisheye_svm.yaml')
    parser.add_argument('--observations', required=True, help='JSON with ground correspondences')
    parser.add_argument('--output', required=True, help='new YAML; never overwrite the input')
    parser.add_argument('--report', required=True, help='JSON metrics, including rejected fits')
    args = parser.parse_args()
    paths = [Path(p).resolve() for p in (args.config, args.observations, args.output, args.report)]
    if len(set(paths)) != 4 or paths[2].exists() or paths[3].exists():
        parser.error('use distinct paths and new output/report files')
    observations = json.loads(paths[1].read_text())
    candidate, report = fit_ground_calibration(load_svm_config(paths[0]), observations['observations'])
    report['observations_sha256'] = hashlib.sha256(paths[1].read_bytes()).hexdigest()
    report['configuration_sha256'] = hashlib.sha256(paths[0].read_bytes()).hexdigest()
    paths[3].parent.mkdir(parents=True, exist_ok=True)
    paths[3].write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    if not report['accepted']:
        return 2
    candidate['calibration_evidence'] = report
    paths[2].parent.mkdir(parents=True, exist_ok=True)
    paths[2].write_text(yaml.safe_dump(candidate, sort_keys=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
