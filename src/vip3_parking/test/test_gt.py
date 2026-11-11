import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from vip3_parking.gt import load_spaces
from vip3_parking.evaluation import slot_from_points


class TestParkingGt(unittest.TestCase):
    def test_supplied_bytes_and_candidate_ids_survive_package_move(self):
        path = Path(__file__).resolve().parents[1]/'data/katri/parking_space_set.json'
        self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(),
                         'd31405670ebf7ec84668412a6ce90f9cd102d0cda602072b940ee633df4dc3b1')
        spaces = load_spaces(path)
        self.assertEqual(len(spaces), 31)
        for number in (2, 8, 9, 12, 39):
            self.assertIn('A5256W{:06d}'.format(number), spaces)
        slot = spaces['A5256W000008']
        self.assertEqual(slot['angle'], 90)
        self.assertIsNone(slot['parking_direction'])
        self.assertAlmostEqual(slot_from_points(slot['points'])['angle'], 150.3, delta=.5)

    def test_invalid_records_are_rejected_instead_of_becoming_targets(self):
        good = dict(idx='p1', points=[[0, 0, 0], [5, 0, 0], [5, 2.5, 0], [0, 2.5, 0]])
        bad_inputs = [{}, [None], [good, good], [dict(good, idx='')],
                      [dict(good, points=[[0, 0, 0]]*4)],
                      [dict(good, points=[[0, 0, 0], [float('nan'), 2, 0], [5, 0, 0]])]]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'gt.json'
            for data in bad_inputs:
                with self.subTest(data=data):
                    path.write_text(json.dumps(data))
                    with self.assertRaises(ValueError):
                        load_spaces(path)


if __name__ == '__main__':
    unittest.main()
