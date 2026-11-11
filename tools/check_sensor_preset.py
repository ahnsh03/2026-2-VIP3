#!/usr/bin/env python3
"""Check the active sensor JSON, ROS contract and SVM poses for configuration drift."""
import json
import math
import re
from datetime import datetime
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml


def main():
    root = Path(__file__).resolve().parents[1]
    contract = yaml.safe_load((root / 'config/ros/topics.yaml').read_text())
    versions = []
    for key, kind in (('sensors', 'Sensor'), ('network', 'Network')):
        path = Path(contract['presets'][key])
        match = re.fullmatch(r'(\d{6})_VIP3_' + kind + r'_(.+)\.json', path.name)
        assert match, 'preset filename must be YYMMDD_VIP3_' + kind + '_MORAI-version.json'
        datetime.strptime(match.group(1), '%y%m%d')
        versions.append(match.group(2))
    assert versions[0] == versions[1], 'Sensor/Network MORAI versions must match; export dates may differ'
    preset = json.loads((root / contract['presets']['sensors']).read_text())
    svm_path = root / 'src/perception/drivable_bev/config/fisheye_svm.yaml'
    svm = yaml.safe_load(svm_path.read_text())
    assert svm['source_sensor_preset'] == contract['presets']['sensors']
    vehicle_path = (svm_path.parent / svm['vehicle_geometry_file']).resolve()
    assert vehicle_path == (root / contract['frames']['vehicle_geometry_file']).resolve()
    vehicle = yaml.safe_load(vehicle_path.read_text())
    spec = vehicle['documented_spec']
    assert vehicle['origin'] == 'rear_axle_midpoint'
    assert math.isclose(spec['length_m'], spec['wheelbase_m'] + spec['front_overhang_m'] + spec['rear_overhang_m'])
    assert vehicle['ground_reference']['base_link_height_m'] > 0
    assert not {'ground_z_m', 'center_x_m', 'vehicle_mask'} & set(svm), 'derive geometry from shared vehicle file'
    cameras = {item['m_SensorUniqueID']: item for item in preset['cameraList']}
    assert len(cameras) == len(preset['cameraList']) == 4, 'four distinct camera IDs required'
    topics, frames = [], []
    transforms = {node.get('name'): node.get('args').split()
                  for node in ET.parse(root / 'src/vip3_bringup/launch/static_tf.launch').getroot().findall('node')}
    for name, expected in contract['cameras'].items():
        sensor = cameras[expected['sensor_id']]
        block, preview = sensor['cc'], svm['cameras'][name]
        assert block['commType'] == 3 and block['modelType'] == 2
        assert block['rosConfig']['RosMessageOnOff']
        assert block['rosConfig']['RosBridgePort'] == contract['bridge']['port'] == 9090
        assert block['rosConfig']['RosBridgeServerUrl'] == contract['bridge']['url']
        for target in (expected, preview):
            assert target['topic'] == block['rosConfig']['Topic'], name + ' topic drift'
            assert target['frame_id'] == block['rosConfig']['frameID'], name + ' frame drift'
            assert (target['width'], target['height']) == (block['cameraResWidth'], block['cameraResHeight'])
            np.testing.assert_allclose(target['translation_m'], [float(sensor['pos'][k]) for k in ('x','y','z')])
            np.testing.assert_allclose(target['rotation_deg'], [float(sensor['rot'][k]) for k in ('roll','pitch','yaw')])
        np.testing.assert_allclose(expected['period_s'], block['sensorPeriod'])
        assert expected['horizontal_fov_deg'] == block['cameraFOV']
        transform = transforms['tf_' + expected['frame_id']]
        np.testing.assert_allclose([float(v) for v in transform[:3]], expected['translation_m'])
        roll, pitch, yaw = expected['rotation_deg']
        np.testing.assert_allclose([float(v) for v in transform[3:6]],
                                   [math.radians((yaw + 180) % 360 - 180), math.radians(pitch), math.radians(roll)], atol=1e-6)
        assert transform[6:] == ['base_link', expected['frame_id']]
        topics.append(expected['topic']); frames.append(expected['frame_id'])
    assert len(set(topics)) == len(set(frames)) == 4, 'camera topics/frames must be unique'
    assert not any(preset.get(k) for k in ('Lidar3DList', 'GPSList', 'IMUList')), 'update active sensors contract'
    assert not contract['sensors'], 'runtime must not require removed sensors'
    network = json.loads((root / contract['presets']['network']).read_text())
    controls = [item['EgoCtrlConfig'] for ego in network['egoNetworkData']
                for item in ego['listEgoNetworkInfo'] if item['PUBSUB_TYPE'] == 771]
    assert len(controls) == 1 and controls[0]['commType'] == 3
    assert controls[0]['rosConfig']['Topic'] == contract['network']['publish']['ctrl_cmd']['topic']
    assert controls[0]['rosConfig']['RosBridgePort'] == 9090
    print('PASS: four fisheye poses, dimensions, IDs, unique topics/frames and CtrlCmd match')
    print('Lens calibration and live connection are separate gates; not validated by this check.')


if __name__ == '__main__':
    main()
