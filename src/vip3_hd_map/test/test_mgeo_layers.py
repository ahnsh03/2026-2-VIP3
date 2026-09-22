#!/usr/bin/env python3
"""MGeo 레이어 로더 단위 테스트. ROS 없이 돌고, 실제 맵 없이도 돈다."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from vip3_hd_map.mgeo_layers import (
    EMPTY_FILE_SHA256,
    EMPTY_LIST_SHA256,
    KatriMGeo,
    as_int,
    bounds_of,
    lane_type_label,
    normalise_parking_space,
    parking_space_polygon,
    parse_file_hash_manifest,
    points_xyz,
    surface_marking_label,
)


class FieldTypeTest(unittest.TestCase):
    """MGeo 의 필드 타입은 일정하지 않다. 로더가 방어적이어야 한다."""

    def test_lane_type_may_be_a_list(self):
        self.assertEqual(505, as_int([505]))
        self.assertEqual(505, as_int(505))
        self.assertEqual(505, as_int("505"))

    def test_unknown_values_fall_back(self):
        self.assertIsNone(as_int(None))
        self.assertEqual(-1, as_int(None, -1))
        self.assertEqual(-1, as_int("Driving", -1))

    def test_points_drop_malformed_features(self):
        self.assertEqual(0, len(points_xyz({})))
        self.assertEqual(0, len(points_xyz({"points": []})))
        self.assertEqual(2, len(points_xyz({"points": [[0, 0, 0], [1, 1, 1]]})))

    def test_bounds_of_empty_is_none(self):
        self.assertIsNone(bounds_of([]))


class LabelTest(unittest.TestCase):
    def test_known_lane_types_have_names(self):
        for code in (501, 503, 505, 530):
            slug, korean = lane_type_label(code)
            self.assertFalse(slug.startswith("UNVERIFIED"), code)
            self.assertTrue(korean)

    def test_unknown_lane_type_is_marked_unverified(self):
        slug, _ = lane_type_label(9999)
        self.assertTrue(slug.startswith("UNVERIFIED"))

    def test_lane_type_535_stays_unverified(self):
        """링도 주차 규격도 아니다. 코드표 원문으로 확인하기 전까지 단정하지 않는다."""
        slug, _ = lane_type_label(535)
        self.assertTrue(slug.startswith("UNVERIFIED"), slug)

    def test_surface_marking_label_accepts_string_sub_type(self):
        slug, _ = surface_marking_label("5371")
        self.assertEqual(slug, surface_marking_label(5371)[0])


class ManifestTest(unittest.TestCase):
    def test_empty_sentinel_hashes_are_correct(self):
        import hashlib

        self.assertEqual(EMPTY_LIST_SHA256, hashlib.sha256(b"[]").hexdigest())
        self.assertEqual(EMPTY_FILE_SHA256, hashlib.sha256(b"").hexdigest())

    def test_manifest_is_parsed_from_the_python_repr_string(self):
        info = {"mgeo_file_hash": "{'a.json': 'aa', 'b.json.meta': 'bb'}"}
        parsed = parse_file_hash_manifest(info)
        self.assertEqual("aa", parsed["a.json"])

    def test_missing_nonempty_is_distinguished_from_missing_empty(self):
        with tempfile.TemporaryDirectory() as root:
            directory = Path(root)
            (directory / "global_info.json").write_text(
                json.dumps(
                    {
                        "global_coordinate_system": "+proj=utm +zone=52",
                        "local_origin_in_global": [1.0, 2.0, 3.0],
                        "mgeo_file_hash": str(
                            {
                                "empty_upstream.json": EMPTY_LIST_SHA256,
                                "real_but_missing.json": "0d410f76" + "0" * 56,
                            }
                        ),
                    }
                ),
                encoding="utf-8",
            )
            mgeo = KatriMGeo(directory)
            statuses = {row["file"]: row["status"] for row in mgeo.manifest_report()}
            self.assertEqual("missing_empty", statuses["empty_upstream.json"])
            self.assertEqual("missing_nonempty", statuses["real_but_missing.json"])
            self.assertEqual(
                ["real_but_missing.json"],
                [row["file"] for row in mgeo.missing_nonempty_files()],
            )


class ParkingSpaceTest(unittest.TestCase):
    """parking_space_set.json 이 도착하면 코드 수정 없이 켜져야 한다."""

    def test_defaults_are_filled(self):
        space = normalise_parking_space(
            {"idx": "P1", "points": [[0, 0], [5, 0], [5, 2.5], [0, 2.5]]}
        )
        self.assertEqual(2.5, space["width"])
        self.assertEqual(5.0, space["length"])
        self.assertEqual(90.0, space["angle"])
        # parking_space_polygon 은 LINE_STRIP 에 바로 쓰도록 **닫힌** 폴리곤을 준다.
        polygon = parking_space_polygon(space)
        self.assertEqual(5, len(polygon))
        np.testing.assert_allclose(polygon[0], polygon[-1])

    def test_center_point_is_derived_when_absent(self):
        space = normalise_parking_space(
            {"idx": "P1", "points": [[0, 0], [4, 0], [4, 2], [0, 2]]}
        )
        center = np.asarray(space["center_point"], dtype=float)
        self.assertAlmostEqual(2.0, float(center[0]))
        self.assertAlmostEqual(1.0, float(center[1]))

    def test_manual_fallback_is_used_when_layer_is_absent(self):
        with tempfile.TemporaryDirectory() as root:
            directory = Path(root)
            (directory / "global_info.json").write_text(
                json.dumps(
                    {
                        "global_coordinate_system": "+proj=utm +zone=52",
                        "local_origin_in_global": [0.0, 0.0, 0.0],
                        "mgeo_file_hash": "{}",
                    }
                ),
                encoding="utf-8",
            )
            manual = directory / "manual.json"
            manual.write_text(
                json.dumps([{"idx": "P1", "points": [[0, 0], [5, 0], [5, 2.5], [0, 2.5]]}]),
                encoding="utf-8",
            )
            mgeo = KatriMGeo(directory, manual_parking_path=manual)
            spaces = mgeo.parking_spaces()
            self.assertEqual(1, len(spaces))
            self.assertEqual("manual", mgeo.parking_space_source)

    def test_no_source_reports_none(self):
        with tempfile.TemporaryDirectory() as root:
            directory = Path(root)
            (directory / "global_info.json").write_text(
                json.dumps(
                    {
                        "global_coordinate_system": "+proj=utm +zone=52",
                        "local_origin_in_global": [0.0, 0.0, 0.0],
                        "mgeo_file_hash": "{}",
                    }
                ),
                encoding="utf-8",
            )
            mgeo = KatriMGeo(directory)
            self.assertEqual([], mgeo.parking_spaces())
            self.assertEqual("none", mgeo.parking_space_source)


if __name__ == "__main__":
    unittest.main()
