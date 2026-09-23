#!/usr/bin/env python3
"""뷰 이름 계약이 파이프라인 전 단계에서 일치하는지 지킨다.

**왜 이 파일이 있나.** ASMC 에서 이식할 때 수집 단계(`capture_sync.CAMERAS`)는 4뷰인데
그 뒤 단계(curation / mask_baker / perception_dataset)가 각자 `("front","left","right")`
를 들고 있었다. 결과적으로 **후방 카메라 데이터를 수집해 놓고 학습 직전에 조용히
버리는** 상태였다. 카메라를 더 달면 같은 일이 또 일어난다.

정본은 `capture_sync.CAMERAS` 하나다. 나머지는 전부 거기서 import 한다.
"""

from __future__ import annotations

import json
import unittest
from pathlib import Path

from data_collection.capture_sync import CAMERAS, MODALITIES, VIEWS
from data_collection.mask_baker import TWINLITE_VIEWS
from data_collection.perception_curation import VIEWS as CURATION_VIEWS
from data_collection.perception_dataset import EXPECTED_VIEWS

REPO_ROOT = Path(__file__).resolve().parents[3]


class ViewContractTest(unittest.TestCase):
    def test_every_stage_uses_the_same_views(self):
        self.assertEqual(VIEWS, TWINLITE_VIEWS, "mask_baker 가 정본과 다르다")
        self.assertEqual(VIEWS, CURATION_VIEWS, "perception_curation 이 정본과 다르다")
        self.assertEqual(VIEWS, EXPECTED_VIEWS, "perception_dataset 가 정본과 다르다")

    def test_rear_is_a_training_view(self):
        """ASMC 의 4번 카메라는 학습에서 제외됐다. VIP3 후방은 후진 주차의 핵심 뷰다."""
        self.assertIn("rear", VIEWS)
        self.assertIn("rear", TWINLITE_VIEWS)
        self.assertIn("rear", EXPECTED_VIEWS)

    def test_views_are_unique_and_derived_from_cameras(self):
        self.assertEqual(len(VIEWS), len(set(VIEWS)))
        self.assertEqual(VIEWS, tuple(view for _, view, _ in CAMERAS))

    def test_files_per_frame_matches_cameras_and_modalities(self):
        """sync preflight 가 프레임당 세는 파일 수. 뷰가 늘면 자동으로 늘어야 한다."""
        expected = len(CAMERAS) * len(MODALITIES) + 2  # + gps, imu
        self.assertEqual(18, expected, "4 뷰 x 4 모달리티 + gps + imu = 18")

    def test_target_policy_views_match_the_code(self):
        """config 의 target policy 가 코드 뷰와 어긋나면 bake 가 런타임에 터진다."""
        path = (
            REPO_ROOT / "config" / "perception" / "target_policy_v1_katri_4view.json"
        )
        policy = json.loads(path.read_text(encoding="utf-8"))
        self.assertEqual(sorted(VIEWS), sorted(policy["views"]))
        for name, target in (policy.get("targets") or {}).items():
            if isinstance(target, dict) and "valid_views" in target:
                unknown = set(target["valid_views"]) - set(VIEWS)
                self.assertFalse(
                    unknown, "target {} 의 valid_views 에 모르는 뷰: {}".format(name, unknown)
                )

    def test_pipeline_names_are_defined_once(self):
        """cache 이름이 두 곳에 따로 적혀 있으면 build 가 만든 걸 train 이 못 찾는다.

        실제로 그랬다 — 코드 기본값(twinlite_morai_v1 / twinlite_384x640_v2),
        학습 스크립트(vip3_twinlite_katri_v1), README(vip3_katri_parking_v1) 가
        전부 달라서 기본값만 쓰면 파이프라인이 끊겼다.
        """
        from data_collection.mask_baker import DEFAULT_OUTPUT_NAME
        from data_collection.perception_dataset import (
            DEFAULT_CACHE_NAME, DEFAULT_DATASET_VERSION,
        )

        # cache 를 만드는 쪽과 읽는 쪽이 같은 이름이어야 한다.
        self.assertIs(DEFAULT_OUTPUT_NAME, DEFAULT_CACHE_NAME)
        # 학습 adapter 가 기대하는 dataset 이름과 build 가 만드는 이름이 같아야 한다.
        twinlite = REPO_ROOT / "src" / "perception" / "twinlite_morai" / "dataset.py"
        text = twinlite.read_text(encoding="utf-8")
        self.assertIn(
            'DEFAULT_DATASET_VERSION = "{}"'.format(DEFAULT_DATASET_VERSION), text,
            "twinlite_morai 와 data_collection 의 기본 dataset 이름이 다르다",
        )
        # 학습 테스트의 fixture 경로도 같은 cache 이름을 쓴다. 여기가 어긋나면
        # 테스트를 보고 경로를 따라 만드는 사람이 bake 결과를 못 찾는다.
        fixture = (
            REPO_ROOT / "src" / "perception" / "twinlite_morai"
            / "test" / "test_dataset_loss.py"
        )
        self.assertIn(
            'CACHE_NAME = "{}"'.format(DEFAULT_OUTPUT_NAME),
            fixture.read_text(encoding="utf-8"),
            "twinlite_morai 학습 테스트의 fixture cache 이름이 파이프라인과 다르다",
        )

    def test_training_scripts_use_the_same_dataset_default(self):
        from data_collection.perception_dataset import DEFAULT_DATASET_VERSION

        for name in ("train_twinlite.py", "train_twinlite_tiny.py",
                     "smoke_twinlite_data_loss.py", "render_twinlite_predictions.py"):
            path = REPO_ROOT / "scripts" / name
            if not path.is_file():
                continue
            text = path.read_text(encoding="utf-8")
            if "--dataset-version" not in text:
                continue
            self.assertIn(
                'default="{}"'.format(DEFAULT_DATASET_VERSION), text,
                "{} 의 --dataset-version 기본값이 파이프라인과 다르다".format(name),
            )

    def test_collector_profiles_cover_every_camera_view(self):
        """수집 프로파일의 카메라 채널이 뷰 전체를 덮는지."""
        from data_collection.profile_loader import CAMERA_CHANNELS

        self.assertEqual(
            sorted(CAMERA_CHANNELS),
            sorted("cam_{}".format(view) for view in VIEWS),
            "profile_loader 의 카메라 채널이 뷰와 어긋난다",
        )


if __name__ == "__main__":
    unittest.main()
