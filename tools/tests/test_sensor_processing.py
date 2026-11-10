"""B1 거리·좌표·무효 깊이 진단을 모의 센서 입력으로 검증한다."""
import importlib.util
from pathlib import Path
from collections import deque
from copy import deepcopy
from types import SimpleNamespace
import unittest

import numpy as np
from sensor_msgs.msg import CameraInfo, Image
from sensor_msgs.msg import Range
from unittest.mock import Mock
from unittest.mock import patch
import math
import xacro
from scipy.spatial.transform import Rotation

SOURCE = Path(__file__).resolve().parents[2] / 'src/ax_perception/scripts/sensor_processing.py'
SPEC = importlib.util.spec_from_file_location('ax_sensor_processing_test', SOURCE)
sensor = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(sensor)


def images(values):
    depth = Image(height=1, width=2, encoding='32FC1', step=8,
                  data=np.asarray(values, dtype='<f4').tobytes())
    depth.header.stamp.sec = 1
    depth.header.frame_id = 'robot_1/front_rgbd_optical_frame'
    rgb = Image(header=deepcopy(depth.header), height=1, width=2, encoding='rgb8', step=6,
                data=bytes([255, 0, 0, 0, 255, 0]))
    info = CameraInfo(header=deepcopy(depth.header), height=1, width=2,
                      k=[2., 0., 1., 0., 2., 0., 0., 0., 1.],
                      p=[2., 0., 1., 0., 0., 2., 0., 0., 0., 0., 1., 0.])
    return depth, rgb, info


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class SensorProcessingTests(unittest.TestCase):
    def test_receive_timeout_counts_suspend_with_frozen_ros_and_monotonic_clocks(self):
        from diagnostic_msgs.msg import DiagnosticStatus
        node, receive, _ = self.receiver()
        _, message, _ = images([1., 1.])
        message.header.frame_id = 'robot_1/front_rgb_optical_frame'
        message.header.stamp.sec = 10
        # 절전 중 ROS/monotonic은 멈추고 BOOTTIME만 진행하는 조건.
        with patch.object(sensor.time, 'monotonic', return_value=100.), \
                patch.object(sensor.time, 'clock_gettime', return_value=100.) as clock:
            receive(message)
            clock.assert_called_with(sensor.time.CLOCK_BOOTTIME)
        with patch.object(sensor.time, 'monotonic', return_value=100.), \
                patch.object(sensor.time, 'clock_gettime', return_value=110.) as clock:
            sensor.SensorProcessing.report(node)
            self.assertEqual(node.diagnostics.messages[-1].status[0].level, DiagnosticStatus.STALE)
            clock.assert_called_with(sensor.time.CLOCK_BOOTTIME)

    def pending_node(self):
        node, _, _ = self.receiver()
        node.cfg.update(tof_body_filter=True, tof_tf_wait_s=.15)
        node.body_mesh = np.zeros((1, 3, 3))
        node.tof_pending = deque()
        node.health['tool_tof/raw'] = {'received': 100., 'ok': False, 'reason': 'No data'}
        node.tf_buffer = SimpleNamespace(can_transform=Mock(return_value=False))
        node.tof = Mock()
        return node

    def test_tof_waits_for_exact_tf_and_preserves_order(self):
        node = self.pending_node()
        first = Range();first.header.stamp.sec = 10
        second = Range();second.header.stamp.sec = 10;second.header.stamp.nanosec = 1
        for msg in (first, second):
            with patch.object(sensor, 'boottime', return_value=100.):
                sensor.SensorProcessing.queue_tof(node, msg)
        with patch.object(sensor, 'boottime', return_value=100.05):
            sensor.SensorProcessing.drain_tof(node)
        node.tof.assert_not_called()
        self.assertEqual(len(node.tof_pending), 2)
        node.tf_buffer.can_transform.return_value = True
        with patch.object(sensor, 'boottime', return_value=100.08):
            sensor.SensorProcessing.drain_tof(node)
        self.assertEqual([call.args[0] for call in node.tof.call_args_list], [first, second])
        self.assertFalse(node.tof_pending)
        self.assertTrue(node.health['tool_tof/raw']['ok'])

    def test_tof_wait_timeout_stale_and_clock_reset(self):
        node = self.pending_node()
        message = Range();message.header.stamp.sec = 10
        with patch.object(sensor, 'boottime', return_value=100.):
            sensor.SensorProcessing.queue_tof(node, message)
        node.tof.side_effect = ValueError('TF unavailable')
        with patch.object(sensor, 'boottime', return_value=100.16):
            sensor.SensorProcessing.drain_tof(node)
        self.assertFalse(node.health['tool_tof/raw']['ok'])
        self.assertFalse(node.tof_pending)
        node.tof.reset_mock()
        with patch.object(sensor, 'boottime', return_value=101.):
            sensor.SensorProcessing.queue_tof(node, message)
        node.now_ns = 11_000_000_000
        sensor.SensorProcessing.drain_tof(node)
        node.tof.assert_not_called()
        self.assertIn('stale', node.health['tool_tof/raw']['reason'])
        with patch.object(sensor, 'boottime', return_value=102.):
            sensor.SensorProcessing.queue_tof(node, message)
        node.now_ns = 1_000_000_000
        sensor.SensorProcessing.drain_tof(node)
        self.assertFalse(node.tof_pending)

    def receiver(self):
        node = SimpleNamespace(cfg={'max_age_s': .5, 'future_tolerance_s': .05,
                                   'missing_timeout_s': 2.}, robot='robot_1',
            health={}, last_stamp={}, buffers={}, clock_previous=0., owned_subs=[],
            now_ns=10_000_000_000, diagnostics=Publisher())
        node.get_clock = lambda: SimpleNamespace(now=lambda: sensor.Time(nanoseconds=node.now_ns))
        node.observe_clock = lambda now: sensor.SensorProcessing.observe_clock(node, now)
        callbacks = {}
        node.create_subscription = lambda cls, topic, callback, qos: callbacks.setdefault(topic, callback)
        output = Publisher()
        sensor.SensorProcessing.subscribe(node, Image, 'front_rgb/image_raw', 'front_rgb_optical_frame', output.publish)
        return node, callbacks['sensors/front_rgb/image_raw'], output

    def test_stale_future_reversed_and_wrong_frame_rejected(self):
        node, receive, output = self.receiver()
        _, message, _ = images([1., 1.])
        message.header.frame_id = 'robot_1/front_rgb_optical_frame'
        def send(seconds, frame=None):
            message.header.stamp = sensor.Time(nanoseconds=int(seconds*1e9)).to_msg()
            message.header.frame_id = frame or 'robot_1/front_rgb_optical_frame'
            receive(message)
        for value in (0., 9., 10.1):
            send(value)
            self.assertFalse(node.health['front_rgb/image_raw']['ok'])
        send(9.9)
        self.assertEqual(len(output.messages), 1)
        for value in (9.9, 9.8):
            send(value)
            self.assertFalse(node.health['front_rgb/image_raw']['ok'])
        send(9.95, 'robot_2/front_rgb_optical_frame')
        self.assertEqual(len(output.messages), 1)
        send(9.99)
        self.assertTrue(node.health['front_rgb/image_raw']['ok'])
        self.assertEqual(len(output.messages), 2)

    def test_clock_reset_clears_buffers_and_other_sensor_health(self):
        node, receive, _ = self.receiver()
        node.clock_previous = 10.
        node.last_stamp = {'old': 9.9}
        node.buffers = {'rgbd': {'image': deque([object()])}}
        node.health['rgbd/synchronized'] = {'received': 1., 'ok': True, 'reason': 'Valid'}
        node.now_ns = 1_000_000_000
        # 센서가 아직 오지 않아도 진단 타이머가 시간 역행을 감지한다.
        sensor.SensorProcessing.report(node)
        self.assertFalse(node.health['rgbd/synchronized']['ok'])
        self.assertIsNone(node.health['rgbd/synchronized']['received'])
        self.assertFalse(node.last_stamp)
        self.assertFalse(node.buffers['rgbd']['image'])
        _, message, _ = images([1., 1.])
        message.header.frame_id = 'robot_1/front_rgb_optical_frame'
        receive(message)
        self.assertTrue(node.health['front_rgb/image_raw']['ok'])

    def test_missing_sensor_diagnostic_and_recovery(self):
        from diagnostic_msgs.msg import DiagnosticStatus
        node, receive, _ = self.receiver()
        _, message, _ = images([1., 1.])
        message.header.frame_id = 'robot_1/front_rgb_optical_frame'
        message.header.stamp.sec = 10
        with patch.object(sensor, 'boottime', return_value=100.):
            receive(message)
        with patch.object(sensor, 'boottime', return_value=102.1):
            sensor.SensorProcessing.report(node)
            self.assertEqual(node.diagnostics.messages[-1].status[0].level, DiagnosticStatus.STALE)
            message.header.stamp.nanosec = 1
            receive(message)
            sensor.SensorProcessing.report(node)
            self.assertEqual(node.diagnostics.messages[-1].status[0].level, DiagnosticStatus.OK)

    def test_rgbd_sync_rejects_skew_and_recovers_without_reusing_frames(self):
        name = 'front_rgbd'
        node = SimpleNamespace(
            cfg={'sync_slop_s': .03, 'max_age_s': .5, 'depth_min_m': .05, 'depth_max_m': 5.},
            buffers={name: {key: deque(maxlen=10) for key in ('image', 'depth_image', 'camera_info')}},
            outputs={(name, key): Publisher() for key in ('image', 'depth_image', 'camera_info', 'points')},
            health={name + '/synchronized': {}},
            get_clock=lambda: SimpleNamespace(now=lambda: sensor.Time(nanoseconds=1_100_000_000)))
        depth, rgb, info = images([1., 2.])
        info.header = type(info.header)()
        info.header.frame_id = rgb.header.frame_id
        info.header.stamp.sec = 1
        info.header.stamp.nanosec = 40_000_000
        for part, msg in (('image', rgb), ('depth_image', depth), ('camera_info', info)):
            sensor.SensorProcessing.rgbd(node, name, part, msg)
        self.assertFalse(node.outputs[name, 'points'].messages)
        for part, msg in (('image', rgb), ('depth_image', depth)):
            msg = deepcopy(msg)
            msg.header.stamp.nanosec = 50_000_000
            sensor.SensorProcessing.rgbd(node, name, part, msg)
        self.assertEqual(len(node.outputs[name, 'points'].messages), 1)
        self.assertEqual(node.outputs[name, 'points'].messages[0].header.stamp.nanosec, 50_000_000)
        info.header.stamp.nanosec = 60_000_000
        sensor.SensorProcessing.rgbd(node, name, 'camera_info', info)
        self.assertEqual(len(node.outputs[name, 'points'].messages), 1)


    def test_invalid_geometry_and_transform_are_rejected(self):
        for description in ('<robot/>', '<robot><link name="base_link"/></robot>'):
            with self.assertRaises(ValueError):
                sensor.body_triangles(description)
        with self.assertRaises(ValueError):
            sensor.rotation_matrix(0, 0, 0, 0)
        with self.assertRaises(ValueError):
            sensor.ray_body_distance(np.array([math.nan, 0, 0]), np.array([1, 0, 0]), np.zeros((1, 3, 3)))
        self.assertTrue(sensor.body_blocks_measurement(.364, .365, .4, .005))
        self.assertFalse(sensor.body_blocks_measurement(.359, .365, .4, .005))

    def test_disabled_filter_preserves_range_and_header(self):
        node = SimpleNamespace(cfg={'tof_body_filter': False}, tof_pub=Publisher())
        message = Range(min_range=.02, max_range=.4, range=.365)
        message.header.frame_id = 'robot_1/tool_tof_link'
        message.header.stamp.sec = 12
        sensor.SensorProcessing.tof(node, message)
        self.assertEqual(node.tof_pub.messages[-1], message)

    def test_body_ray_and_closer_obstacle(self):
        source = SOURCE.parents[2] / 'ax_description/urdf/ax.urdf.xacro'
        description = xacro.process_file(str(source), mappings={'gripper_model': 'epick'}).toxml()
        triangles = sensor.body_triangles(description)
        import xml.etree.ElementTree as ET
        robot = ET.fromstring(description)
        frames = {'base_link': np.eye(4)}
        pending = robot.findall('joint')
        positions = dict(zip(('column_yaw_joint', 'shoulder_joint', 'elbow_joint', 'wrist_joint', 'gripper_rotation_joint'),
                             [0., .4, -.8, .4, 0.]))
        while pending:
            count = len(pending)
            for joint in pending[:]:
                parent = joint.find('parent').get('link')
                if parent not in frames:
                    continue
                element = joint.find('origin')
                t = np.eye(4)
                if element is not None:
                    t[:3, 3] = np.fromstring(element.get('xyz', '0 0 0'), sep=' ')
                    t[:3, :3] = Rotation.from_euler('xyz', np.fromstring(element.get('rpy', '0 0 0'), sep=' ')).as_matrix()
                if joint.get('name') in positions:
                    axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
                    t[:3, :3] = t[:3, :3] @ Rotation.from_rotvec(axis*positions[joint.get('name')]).as_matrix()
                frames[joint.find('child').get('link')] = frames[parent] @ t
                pending.remove(joint)
            self.assertLess(len(pending), count)
        t = frames['epick_tof_link']
        expected = sensor.ray_body_distance(t[:3, 3], t[:3, 0], triangles)
        self.assertAlmostEqual(expected, .3774024, places=4)
        measured_body = expected + .0004
        self.assertTrue(sensor.body_blocks_measurement(measured_body, expected, .4, .005))
        self.assertTrue(sensor.body_blocks_measurement(math.inf, expected, .4, .005))
        self.assertFalse(sensor.body_blocks_measurement(.1, expected, .4, .005))
        self.assertFalse(sensor.body_blocks_measurement(-math.inf, expected, .4, .005))
        self.assertFalse(sensor.body_blocks_measurement(.365, math.inf, .4, .005))
        self.assertFalse(sensor.body_blocks_measurement(.365, .5, .4, .005))
        # 전처리 출력만 무효화하며 원본은 보존한다.
        q = Rotation.from_matrix(t[:3, :3]).as_quat()
        transform = SimpleNamespace(translation=SimpleNamespace(**dict(zip('xyz', t[:3, 3]))),
                                    rotation=SimpleNamespace(**dict(zip('xyzw', q))))
        node = SimpleNamespace(cfg={'tof_body_filter': True, 'tof_body_tolerance_m': .005},
            robot='robot_1', body_mesh=triangles, body_error='', tof_pub=Publisher(),
            tf_buffer=SimpleNamespace(lookup_transform=Mock(return_value=SimpleNamespace(transform=transform))))
        message = Range(min_range=.02, max_range=.4, range=measured_body)
        message.header.frame_id = 'robot_1/tool_tof_link'
        message.header.stamp.sec = 12
        with self.assertRaisesRegex(ValueError, 'blocked by robot body'):
            sensor.SensorProcessing.tof(node, message)
        self.assertTrue(math.isnan(node.tof_pub.messages[-1].range))
        self.assertAlmostEqual(message.range, measured_body, places=5)
        message.range = .1
        sensor.SensorProcessing.tof(node, message)
        self.assertAlmostEqual(node.tof_pub.messages[-1].range, .1, places=6)
        self.assertEqual(node.tf_buffer.lookup_transform.call_args.args[2].nanoseconds, 12_000_000_000)

    def test_missing_model_or_tf_does_not_publish_trusted_distance(self):
        from tf2_ros import LookupException
        node = SimpleNamespace(cfg={'tof_body_filter': True, 'tof_body_tolerance_m': .005},
            robot='robot_1', body_mesh=None, body_error='Waiting for robot_description', tof_pub=Publisher())
        message = Range(min_range=.02, max_range=.4, range=.2)
        with self.assertRaisesRegex(ValueError, 'Waiting for robot_description'):
            sensor.SensorProcessing.tof(node, message)
        self.assertTrue(math.isnan(node.tof_pub.messages[-1].range))
        node.body_mesh = np.zeros((1, 3, 3))
        node.tf_buffer = SimpleNamespace(lookup_transform=Mock(side_effect=LookupException('missing transform')))
        with self.assertRaisesRegex(ValueError, 'missing transform'):
            sensor.SensorProcessing.tof(node, message)
        self.assertTrue(math.isnan(node.tof_pub.messages[-1].range))

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
        for values, expected, fraction in (([float('nan'), float('inf')], False, 0.),
                                          ([2., float('nan')], True, .5), ([1., 2.], True, 1.)):
            depth, rgb, info = images(values)
            for key, message in (('image', rgb), ('depth_image', depth), ('camera_info', info)):
                sensor.SensorProcessing.rgbd(node, name, key, message)
            self.assertTrue(node.health[name + '/synchronized']['ok'])
            self.assertEqual(node.health[name + '/depth_valid']['ok'], expected)
            self.assertEqual(node.health[name + '/depth_valid']['valid_depth_fraction'], fraction)
            self.assertEqual(node.outputs[name, 'points'].messages[-1].is_dense, fraction == 1.)
        self.assertEqual(len(node.outputs[name, 'points'].messages), 3)


if __name__ == '__main__':
    unittest.main()
