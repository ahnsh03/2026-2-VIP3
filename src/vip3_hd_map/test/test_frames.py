import numpy as np
import unittest

from vip3_hd_map.frames import LocalMapFrame, origin_translation, transform_points


class FrameTest(unittest.TestCase):
    def test_declared_origins_define_legacy_to_current_translation(self):
        legacy = LocalMapFrame(
            "+proj=utm +zone=52 +ellps=WGS84 +units=m +no_defs",
            (302459.942, 4122635.537, 28.991),
        )
        current = LocalMapFrame(
            "+proj=utm +zone=52 +datum=WGS84 +units=m +no_defs",
            (302595.0, 4124145.0, 0.0),
        )
        np.testing.assert_allclose(
            origin_translation(legacy, current),
            [-135.058, -1509.463, 28.991],
            atol=1e-9,
        )
        np.testing.assert_allclose(
            transform_points([[0.0, 0.0, 0.0]], legacy, current)[0],
            [-135.058, -1509.463, 28.991],
            atol=1e-9,
        )

    def test_incompatible_coordinate_system_is_rejected(self):
        source = LocalMapFrame("+proj=utm +zone=52 +datum=WGS84", (0.0, 0.0, 0.0))
        target = LocalMapFrame("+proj=utm +zone=51 +datum=WGS84", (0.0, 0.0, 0.0))
        with self.assertRaisesRegex(ValueError, "incompatible map CRS"):
            origin_translation(source, target)


if __name__ == "__main__":
    unittest.main()
