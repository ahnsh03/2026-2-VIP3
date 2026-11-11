import unittest

import cv2
import numpy as np

from drivable_bev.fisheye_svm import VIEWS, select_frames
from drivable_bev.svm_input import EncodedCameraFrame, SelectedFrameDecoder


class TestSelectedFrameDecoder(unittest.TestCase):
    def setUp(self):
        self.cameras = {v: dict(width=32, height=24, frame_id='cam_'+v) for v in VIEWS}
        self.decoder = SelectedFrameDecoder(self.cameras)

    @staticmethod
    def frame(view, value=100, stamp=100., arrival=10., width=32):
        data = cv2.imencode('.jpg', np.full((24, width, 3), value, np.uint8))[1].tobytes()
        return EncodedCameraFrame(data, stamp, arrival, 'cam_'+view)

    def test_discarded_old_frames_never_need_valid_jpeg_or_decoding(self):
        buffers = {v: [EncodedCameraFrame(b'not a jpeg', 99., 9.8, 'cam_'+v), self.frame(v)]
                   for v in VIEWS}
        selected, _, _ = select_frames(buffers, 10.1, .5, .06)
        images = self.decoder.decode(selected)
        self.assertEqual(self.decoder.decoded_count, 4)
        self.assertTrue(all(np.all(im == 100) for im in images.values()))

    def test_reused_frame_decodes_once_but_new_frame_with_same_stamp_is_not_stale(self):
        frames = {v: self.frame(v) for v in VIEWS}
        a = self.decoder.decode(frames)
        b = self.decoder.decode(frames)
        self.assertEqual(self.decoder.decoded_count, 4)
        self.assertIs(a['rear'], b['rear'])
        frames['front'] = self.frame('front', value=200)
        c = self.decoder.decode(frames)
        self.assertEqual(self.decoder.decoded_count, 5)
        self.assertTrue(np.all(c['front'] == 200))
        self.assertEqual(len(self.decoder.cache), 4)

    def test_invalid_selected_input_does_not_return_old_cached_image(self):
        frames = {v: self.frame(v) for v in VIEWS}
        self.decoder.decode(frames)
        for bad in (EncodedCameraFrame(b'', 101., 11., 'cam_front'),
                    EncodedCameraFrame(b'bad', 101., 11., 'cam_front'),
                    self.frame('front', width=16), self.frame('wrong_frame')):
            frames['front'] = bad
            with self.assertRaises(ValueError):
                self.decoder.decode(frames)


if __name__ == '__main__':
    unittest.main()
