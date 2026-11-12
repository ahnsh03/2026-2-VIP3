#!/usr/bin/env python3
"""CPU SVM benchmark and numerical comparison against a saved reference module.

Uses all output pixels and changing poses. This measures local compute, not
MORAI rendering, network transport or GUI latency. Optional PNG captures supply
real textures; otherwise a deterministic synthetic image is used.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import platform
import sys
import time

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/vip3_perception/src'))
from vip3_perception.svm.projection import FisheyeSvm
from vip3_perception.svm.frames import VIEWS
from vip3_perception.svm.pose import (
    PoseSample, ground_plane_from_pose, offset_ground_plane, relative_pose
)
from vip3_perception.svm.vehicle_geometry import load_svm_config


def statistics(values):
    return dict(p50_ms=float(np.median(values)), p95_ms=float(np.percentile(values, 95)),
                mean_ms=float(np.mean(values)))


def projection(index, offset):
    phase = index*.13
    reference = PoseSample(100., 1., [10.+index*.015, 20., 2.],
                           [.02*np.sin(phase), .01*np.cos(phase), .3, 1.])
    plane = offset_ground_plane(ground_plane_from_pose(reference, .35), offset)
    motion = {}
    for i, view in enumerate(VIEWS):
        # 0..30ms capture-time offsets while the vehicle translates and rotates.
        sample = PoseSample(100.-i*.01, 1., reference.position+[-i*.007, i*.001, 0.],
                            reference.quaternion + [i*.00007, -i*.00003, -i*.0001, 0.])
        motion[view] = relative_pose(reference, sample)
    return plane, motion


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT/'src/vip3_perception/config/svm.yaml')
    parser.add_argument('--capture-dir', type=Path)
    parser.add_argument('--reference-module', type=Path, help='old fisheye_svm.py, exported with git show')
    parser.add_argument('--frames', type=int, default=50)
    parser.add_argument('--threads', type=int, default=4)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.frames < 10 or args.threads < 1:
        parser.error('use at least 10 frames and one OpenCV thread')
    if args.output.exists():
        parser.error('choose a new output path to preserve earlier measurements')
    cv2.setNumThreads(args.threads)
    config = load_svm_config(args.config)
    images = {}
    for i, v in enumerate(VIEWS):
        camera = config['cameras'][v]
        if args.capture_dir:
            image = cv2.imread(str(args.capture_dir/(v+'.png')))
            if image is None or image.shape != (camera['height'], camera['width'], 3):
                parser.error('capture must provide four images matching the camera dimensions')
        else:
            y, x = np.indices((camera['height'], camera['width']))
            image = np.stack(((x+i*30)%256, (y+i*50)%256, ((x//9+y//11)%2)*255), axis=-1).astype(np.uint8)
        images[v] = image
    implementations = {'optimized': FisheyeSvm(config)}
    if args.reference_module:
        spec = importlib.util.spec_from_file_location('vip3_perception.svm._benchmark_reference', args.reference_module)
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        # Historical reference core only needs this renamed import. Its math
        # remains unchanged; retain the original file hash in the report.
        source = args.reference_module.read_text().replace('from .svm_pose import', 'from .pose import')
        exec(compile(source, str(args.reference_module), 'exec'), module.__dict__)
        implementations['reference'] = module.FisheyeSvm(config)
    measurements = {name: {'maps': [], 'warp': [], 'maps_and_warp': []} for name in implementations}
    agreement = dict(max_remap_error_px=0., max_weight_error=0., mask_changed_pixels=0,
                     max_output_channel_difference=0, output_changed_pixels=0)
    for index in range(args.frames+5):
        plane, motion = projection(index, float(config.get('projection_surface_offset_m', 0.)))
        rendered = {}
        order = list(implementations)
        if index % 2:
            order.reverse()
        for name in order:
            svm = implementations[name]
            t = time.perf_counter()
            svm.set_projection(plane, motion)
            mapped = time.perf_counter()
            rendered[name] = svm.render(images)
            finished = time.perf_counter()
            if index >= 5:
                for stage, value in [('maps', mapped-t), ('warp', finished-mapped), ('maps_and_warp', finished-t)]:
                    measurements[name][stage].append(value*1000)
        if 'reference' in implementations:
            current, old = implementations['optimized'], implementations['reference']
            for view in VIEWS:
                for dimension in (0, 1):
                    error = np.max(np.abs(current.maps[view][dimension]-old.maps[view][dimension]))
                    agreement['max_remap_error_px'] = max(agreement['max_remap_error_px'], float(error))
                agreement['max_weight_error'] = max(agreement['max_weight_error'],
                    float(np.max(np.abs(current.maps[view][2]-old.maps[view][2]))))
            agreement['mask_changed_pixels'] += int(np.count_nonzero(current.valid_mask != old.valid_mask))
            difference = abs(rendered['optimized'].astype(int)-rendered['reference'].astype(int))
            agreement['max_output_channel_difference'] = max(agreement['max_output_channel_difference'], int(difference.max()))
            agreement['output_changed_pixels'] += int(np.count_nonzero(np.any(difference, axis=2)))
    report = dict(opencv=cv2.__version__, numpy=np.__version__, python=platform.python_version(),
                  opencv_threads=cv2.getNumThreads(), available_logical_cpus=cv2.getNumberOfCPUs(),
                  measured_frames=args.frames, warmup_frames=5, output_size=config['output_size'],
                  camera_size=[config['cameras']['front']['width'], config['cameras']['front']['height']],
                  capture=str(args.capture_dir) if args.capture_dir else 'synthetic',
                  calibration_profile=config.get('calibration_profile'),
                  scope='offline CPU maps+warp, varying poses; excludes network, JPEG and GUI',
                  timings={n: {k: statistics(v) for k, v in m.items()} for n, m in measurements.items()})
    if args.reference_module:
        report['reference_sha256'] = hashlib.sha256(args.reference_module.read_bytes()).hexdigest()
        report['agreement'] = agreement
        report['agreement_passed'] = (agreement['max_remap_error_px'] <= .001
                                      and agreement['max_weight_error'] <= 1e-5
                                      and agreement['mask_changed_pixels'] == 0
                                      and agreement['max_output_channel_difference'] <= 1)
        report['agreement_tolerance'] = dict(remap_px=.001, weight=1e-5,
                                             mask_changed_pixels=0, output_channel_difference=1)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    if report.get('agreement_passed') is False:
        raise SystemExit(2)


if __name__ == '__main__':
    main()
