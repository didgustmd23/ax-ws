"""B1 거리·좌표·무효 깊이 진단을 모의 센서 입력으로 검증한다."""
import importlib.util
from pathlib import Path
from collections import deque
from types import SimpleNamespace
import unittest

import numpy as np
from sensor_msgs.msg import CameraInfo, Image

SOURCE = Path(__file__).resolve().parents[2] / 'src/ax_perception/scripts/sensor_processing.py'
SPEC = importlib.util.spec_from_file_location('ax_sensor_processing_test', SOURCE)
sensor = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(sensor)


def images(values):
    depth = Image(height=1, width=2, encoding='32FC1', step=8,
                  data=np.asarray(values, dtype='<f4').tobytes())
    depth.header.stamp.sec = 1
    depth.header.frame_id = 'robot_1/front_rgbd_optical_frame'
    rgb = Image(header=depth.header, height=1, width=2, encoding='rgb8', step=6,
                data=bytes([255, 0, 0, 0, 255, 0]))
    info = CameraInfo(header=depth.header, height=1, width=2,
                      k=[2., 0., 1., 0., 2., 0., 0., 0., 1.],
                      p=[2., 0., 1., 0., 0., 2., 0., 0., 0., 0., 1., 0.])
    return depth, rgb, info


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class SensorProcessingTests(unittest.TestCase):
    def test_depth_limits_preserve_input(self):
        depth, _, _ = images([0.01, 6.])
        result = sensor.cleaned_depth(depth, .05, 5.)
        self.assertTrue(np.isnan(np.frombuffer(result.data, '<f4')).all())
        np.testing.assert_allclose(np.frombuffer(depth.data, '<f4'), [.01, 6.])

    def test_optical_axes_and_frame(self):
        depth, rgb, info = images([2., 2.])
        cloud = sensor.optical_cloud(depth, info, rgb)
        xyz = np.frombuffer(cloud.data, '<f4').reshape(2, 4)[:, :3]
        np.testing.assert_allclose(xyz, [[-1., 0., 2.], [0., 0., 2.]])
        self.assertEqual(cloud.header, depth.header)

    def test_empty_depth_reports_error_and_recovers(self):
        name = 'front_rgbd'
        node = SimpleNamespace(
            cfg={'sync_slop_s': .03, 'max_age_s': .5, 'depth_min_m': .05, 'depth_max_m': 5.},
            buffers={name: {key: deque() for key in ('image', 'depth_image', 'camera_info')}},
            outputs={(name, key): Publisher() for key in ('image', 'depth_image', 'camera_info', 'points')},
            health={name + '/synchronized': {}},
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1_000_000_000)))
        for values, expected in (([float('nan'), float('inf')], False), ([2., float('nan')], True)):
            depth, rgb, info = images(values)
            for key, message in (('image', rgb), ('depth_image', depth), ('camera_info', info)):
                sensor.SensorProcessing.rgbd(node, name, key, message)
            self.assertEqual(node.health[name + '/synchronized']['ok'], expected)
            self.assertEqual(node.outputs[name, 'points'].messages[-1].is_dense, False)
        self.assertEqual(len(node.outputs[name, 'points'].messages), 2)


if __name__ == '__main__':
    unittest.main()
