"""Height limit, coverage and missing depth regression tests."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np

path = Path(__file__).resolve().parents[2] / 'src/ax_description/scripts/cargo_height_guard.py'
spec = importlib.util.spec_from_file_location('cargo_height_guard', path)
guard = importlib.util.module_from_spec(spec)
spec.loader.exec_module(guard)


class CargoHeightTest(unittest.TestCase):
    def measure(self, depth):
        return guard.measure_height(depth, (268.5, 268.5, 320., 240.),
                                    1.026, .5, .72, .72, .95)

    def test_empty_single_double_and_limit(self):
        for height, blocked in ((0., False), (.25, False), (.49, False), (.5, True), (.55, True)):
            with self.subTest(height=height):
                measured = self.measure(np.full((480, 640), 1.026-height))
                self.assertAlmostEqual(measured, height)
                self.assertEqual(guard.loading_blocked(measured, .5), blocked)

    def test_small_high_box_is_not_averaged_away(self):
        depth = np.full((480, 640), 1.026)
        depth[235:245, 315:325] = .476
        self.assertTrue(guard.loading_blocked(self.measure(depth), .5))

    def test_rendered_pixel_centers_exclude_frame_just_outside_deck(self):
        depth = np.full((480, 640), 1.026)
        # Gazebo에서 관측한 빈 선반의 경계 픽셀: 실제 X는 0.3616m로 판 바깥이다.
        depth[128, 432] = .86306214
        k = (268.5118819767296, 268.5118819767296, 320., 240.)
        self.assertAlmostEqual(guard.measure_height(depth,k,1.026,.5,.72,.72,.95,.5),0.)
        self.assertGreater(guard.measure_height(depth,k,1.026,.5,.72,.72,.95,0.),.15)

    def test_invalid_depth_and_insufficient_fov(self):
        for value in (float('nan'), float('inf'), 0.):
            with self.assertRaises(ValueError):
                self.measure(np.full((480, 640), value))
        with self.assertRaises(ValueError):
            guard.measure_height(np.ones((480, 640)), (554., 554., 320., 240.), 1.026, .5, .72, .72, .95)
        self.assertTrue(guard.loading_blocked(None, .5))

    def test_next_box_headroom(self):
        self.assertTrue(guard.loading_blocked(.4, .5, .2))
        self.assertFalse(guard.loading_blocked(.2, .5, .2))

    def test_depth_padding_units_and_endian(self):
        for encoding, dtype, values in (('16UC1', '>u2', [500, 1000]), ('32FC1', '>f4', [.5, 1.])):
            row = np.array(values, dtype=dtype).tobytes() + b'padding!'
            msg = SimpleNamespace(encoding=encoding, is_bigendian=True,
                width=2, height=2, step=len(row), data=row+row)
            np.testing.assert_allclose(guard.decode_depth(msg), [[.5, 1.], [.5, 1.]])


if __name__ == '__main__':
    unittest.main()
