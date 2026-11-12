"""The snapshot and bag paths must consume the same four camera streams."""
import os
from pathlib import Path
import unittest
from unittest.mock import patch

from vip3_data.profile_loader import CAMERA_CHANNELS, load_profile
from vip3_data.bag_profile import load_bag_profile


class CollectionContractTest(unittest.TestCase):
    def setUp(self):
        self.config = Path(__file__).resolve().parents[1] / 'config'
        self.environment = patch.dict(os.environ, {
            'VIP3_COLLECT_ENABLE': '', 'VIP3_COLLECT_DISABLE': ''})
        self.environment.start()
        self.addCleanup(self.environment.stop)

    def test_parking_snapshots_match_raw_bag_camera_inputs(self):
        snapshot = load_profile(str(self.config / 'snapshot_profiles/parking_core.yaml'))
        bag = load_bag_profile(str(self.config / 'bag_profiles'), 'parking_raw')
        camera_topics = {snapshot['topics'][v] for v in CAMERA_CHANNELS}
        self.assertEqual(len(camera_topics), 4)
        self.assertTrue(all(snapshot['channels'][v] for v in CAMERA_CHANNELS))
        self.assertTrue(camera_topics.issubset(set(bag['required_topics'])))
        self.assertTrue(snapshot['channels']['ego'])
        self.assertFalse(any(snapshot['channels'][v] for v in ('lidar', 'gps', 'imu')))
        self.assertEqual(snapshot['map'], 'R_KR_PR_K-city_2025')

    def test_first_connection_profile_only_requires_ego(self):
        profile = load_profile(str(self.config / 'snapshot_profiles/dry_run.yaml'))
        self.assertEqual([v for v, enabled in profile['channels'].items() if enabled], ['ego'])
        self.assertTrue(profile['require_ego'])

    def test_channel_overrides_do_not_modify_the_saved_profile(self):
        path = str(self.config / 'snapshot_profiles/parking_core.yaml')
        with patch.dict(os.environ, {'VIP3_COLLECT_DISABLE': 'cam_rear,objects'}):
            self.assertFalse(load_profile(path)['channels']['cam_rear'])
        self.assertTrue(load_profile(path)['channels']['cam_rear'])


if __name__ == '__main__':
    unittest.main()
