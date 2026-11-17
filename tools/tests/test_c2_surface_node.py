"""C2 ROS 입력 노드(c2_surface_node.py)를 실제 ROS 메시지 객체로 검사한다.

[검사 범위] rclpy 노드를 만들고 구독 콜백을 직접 호출한다(같은 프로세스, DDS 통신 시험 아님).
ROS 시각(정수 ns)·감독 시각은 시험이 정한다. 깊이·검출은 합성 fixture이며 실제 C1 추론·Gazebo 검증이 아니다.
설치본 실행·프로세스 종료는 test_c2_surface_node_process.py가 검사한다.
"""
from itertools import permutations
import errno
import json
import math
import os
from pathlib import Path
import random
import re
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/ax_perception/scripts'))
sys.path.insert(0, str(ROOT / 'tools/tests'))

import ros_test_domain  # noqa: E402

# 같은 PC의 다른 시험(20~95 무작위, 100, 101~140 배정)·팀 격리(141~231)와 겹치지 않는 범위.
C2_NODE_DOMAINS = range(96, 100)

FRAME = 'robot_1/front_rgbd_optical_frame'
OTHER_FRAME = 'robot_1/front_rgb_optical_frame'
TOPIC = 'perception/rgbd_detections'
NS = 1_000_000_000
EPOCH = 1_794_923_490 * NS + 123_456_789

# 시험 설정(fixture 값, 운영 승인값 아님). 만료 0.5 s는 C1·B1 max_age_s 기본값과 같게 둔 시험값이다.
# 해상도·바이트는 시험 장면(640×480, 깊이 4 B/px)에 맞춘 값, 나머지 개수·길이 상한은 시험을 위한 작은 값이다.
BASE_PARAMETERS = {
    'robot_id': 'robot_1', 'camera': 'front_rgbd', 'camera_frame': FRAME, 'detection_topic': TOPIC,
    'time_source': 'sim', 'settings_source': 'tools/tests/test_c2_surface.py TEST_SETTINGS(시험값)',
    'target_frame': '', 'bundle_capacity': 3, 'part_capacity': 3, 'detection_capacity': 3,
    'outcome_history': 16, 'expected_width': 640, 'expected_height': 480, 'max_image_bytes': 640 * 480 * 4,
    'max_detections_per_array': 4, 'max_hypotheses_per_detection': 2, 'max_text_len': 64,
    'max_distortion_len': 8, 'diagnostic_rows_per_reason': 3, 'max_record_bytes': 4 * 1024 * 1024,
    'max_age_s': 0.5, 'future_tolerance_s': 0.05, 'assembly_wait_s': 0.3, 'detection_wait_s': 0.4,
    'poll_period_s': 0.02, 'state_period_s': 1.0,
}
# TF를 켤 때 추가하는 시험값
TF_PARAMETERS = {'target_frame': 'robot_1/base_link', 'tf_wait_s': 0.2, 'tf_cache_s': 10.0,
                 'tf_pending_capacity': 4}


# ROS 메시지·노드 모듈을 전역으로 불러온다(domain 설정 뒤 호출). 설치본 실행 시험도 이 도우미를 쓴다
def load_ros():
    global rclpy, node_module, link, se, scene, face_boxes, TEST_SETTINGS, ns
    global CameraInfo, Image, Detection2D, Detection2DArray, ObjectHypothesisWithPose, TransformStamped, Parameter

    import rclpy
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import CameraInfo, Image
    from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
    from geometry_msgs.msg import TransformStamped

    import c2_surface_node as node_module
    import c2_detection_link as link
    import surface_estimation as se
    from test_c2_detection_link import face_boxes, ns, scene
    from test_c2_surface import TEST_SETTINGS


# 시험 domain을 잡고 ROS를 초기화한다
def setUpModule():
    global DOMAIN_LOCK

    domain, DOMAIN_LOCK = ros_test_domain.reserve_domain(C2_NODE_DOMAINS)
    os.environ.update(ROS_DOMAIN_ID=str(domain), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    load_ros()
    rclpy.init()


def tearDownModule():
    rclpy.shutdown()
    ros_test_domain.release(DOMAIN_LOCK)


# 정수 ns → header
def set_stamp(header, stamp, frame):
    header.stamp.sec = stamp // NS
    header.stamp.nanosec = stamp % NS
    header.frame_id = frame


# 합성 장면 깊이 메시지(32FC1)
def depth_message(stamp, frame=FRAME, depth=None, encoding='32FC1'):
    array = (scene() if depth is None else depth).astype(np.float32)
    message = Image(height=array.shape[0], width=array.shape[1], encoding=encoding, is_bigendian=False,
                    step=array.shape[1] * 4, data=array.tobytes())
    set_stamp(message.header, stamp, frame)

    return message


# 컬러 메시지(내용은 C2 계산에 쓰이지 않음: 크기·encoding·frame·stamp·지문만 검사). fill로 내용을 바꾼다
def color_message(stamp, frame=FRAME, width=640, height=480, fill=0, step=None):
    step = width * 3 if step is None else step
    message = Image(height=height, width=width, encoding='rgb8', step=step, data=bytes([fill]) * (step * height))
    set_stamp(message.header, stamp, frame)

    return message


# 무왜곡 CameraInfo(test_c2_surface 시험 카메라와 같은 값)
def info_message(stamp, frame=FRAME, distortion=0.0, d_length=5, model='plumb_bob', width=640):
    f = 554.256
    message = CameraInfo(width=width, height=480, distortion_model=model, d=[distortion] + [0.0] * (d_length - 1),
                         k=[f, 0.0, 320.0, 0.0, f, 240.0, 0.0, 0.0, 1.0],
                         p=[f, 0.0, 320.0, 0.0, 0.0, f, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0])
    set_stamp(message.header, stamp, frame)

    return message


# 실제 vision_msgs 검출 배열. boxes: ((u0, v0, u1, v1), class_id, score) 목록
def detections(stamp, boxes, frame=FRAME):
    message = Detection2DArray()
    set_stamp(message.header, stamp, frame)

    for (u0, v0, u1, v1), class_id, score in boxes:
        item = Detection2D()
        item.header = message.header
        item.bbox.center.position.x, item.bbox.center.position.y = (u0 + u1) / 2, (v0 + v1) / 2
        item.bbox.size_x, item.bbox.size_y = float(u1 - u0), float(v1 - v0)
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id, hypothesis.hypothesis.score = class_id, score
        item.results.append(hypothesis)
        message.detections.append(item)

    return message


# 장면 두 면의 2 px 안쪽 bbox
def inner_boxes():
    return [(float(u0 + 2), float(v0 + 2), float(u1 - 2), float(v1 - 2)) for u0, v0, u1, v1 in face_boxes()]


class NodeCase(unittest.TestCase):
    # 노드 하나와 시험 시계(정수 ns). 기록은 임시 폴더 JSON Lines
    def make(self, **changes):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        values = dict(BASE_PARAMETERS)
        values.update(changes)
        values.setdefault('record_path', os.path.join(self.folder.name, 'record.jsonl'))

        for name, value in TEST_SETTINGS.__dict__.items():
            values.setdefault('estimation.' + name, value)

        overrides = [Parameter(name, value=value) for name, value in values.items()]
        overrides.append(Parameter('use_sim_time', value=values['time_source'] == 'sim'))
        self.now, self.wall = ns(10.0), 1000.0
        node = node_module.C2SurfaceNode(parameter_overrides=overrides)
        node.now_ns = lambda: self.now
        node.wall_s = lambda: self.wall
        self.node = node
        self.addCleanup(self.cleanup, node)

        return node

    def cleanup(self, node):
        node.close()
        node.destroy_node()

    # 기록 줄(event로 거름)
    def rows(self, event=None):
        lines = Path(self.node.cfg.record_path).read_text().splitlines()
        rows = [json.loads(line) for line in lines]

        return [row for row in rows if event is None or row['event'] == event]

    # 한 stamp의 부품 세 개를 주어진 순서로 넣는다
    def parts(self, stamp, order=('image', 'depth_image', 'camera_info'), **makers):
        made = {'image': makers.get('image', color_message(stamp)),
                'depth_image': makers.get('depth_image', depth_message(stamp)),
                'camera_info': makers.get('camera_info', info_message(stamp))}

        for part in order:
            self.node.on_part(part, made[part])

    def boxes(self):
        return [(box, 'box', 0.9) for box in inner_boxes()]

    def reasons(self):
        return [row['reasons'] for row in self.rows('detection_array_rejected')]


class Configuration(NodeCase):
    def test_missing_or_invalid_settings_are_rejected_at_start(self):
        values = dict(BASE_PARAMETERS, record_path='/tmp/x.jsonl')
        values.update({'estimation.' + key: value for key, value in TEST_SETTINGS.__dict__.items()})
        node_module.parse_config(values, True)

        for change, expected in ((dict(detection_topic=''), 'detection_topic'),
                                 (dict(bundle_capacity=0), 'bundle_capacity'),
                                 (dict(max_age_s=math.nan), 'max_age_s'),
                                 (dict(time_source='system'), 'time_source/use_sim_time'),
                                 (dict(time_source='wall'), 'time_source'),
                                 (dict(target_frame='robot_1/base_link'), 'tf_wait_s'),
                                 (dict(TF_PARAMETERS, tf_pending_capacity=3), 'tf_pending_capacity<max_det'),
                                 (dict(record_path='relative.jsonl'), 'record_path'),
                                 (dict(outcome_history=0), 'outcome_history'),
                                 (dict(max_detections_per_array=True), 'max_detections_per_array'),
                                 (dict(max_image_bytes=640 * 480 * 3), 'max_image_bytes<4*width*height'),
                                 (dict(max_record_bytes=4096), 'max_record_bytes'),
                                 (dict(max_text_len=8), 'camera_frame>max_text_len'),
                                 ({'estimation.ransac_iterations': 0}, 'estimation('),
                                 ({'estimation.min_valid_points': math.nan}, 'estimation.min_valid_points')):
            with self.subTest(change=change):
                with self.assertRaises(ValueError) as caught:
                    node_module.parse_config(dict(values, **change), True)

                self.assertIn(expected, str(caught.exception))

        # TF를 켜면 TF 상한 3개가 모두 필요하다.
        node_module.parse_config(dict(values, **TF_PARAMETERS), True)

        # 노드도 같은 이유로 시작을 거부한다(기본 운영값 없음).
        with self.assertRaises(ValueError):
            self.make(detection_topic='')

    def test_existing_record_is_not_overwritten(self):
        node = self.make()
        path = node.cfg.record_path

        with self.assertRaises(FileExistsError):
            node_module.C2SurfaceNode(parameter_overrides=[
                Parameter(name, value=node.get_parameter(name).value) for name in node.parameter_names]
                + [Parameter('use_sim_time', value=True)])

        self.assertEqual(self.rows('start')[0]['config']['record_path'], path)


class Assembly(NodeCase):
    def test_any_part_order_links_every_bbox_with_exact_stamp(self):
        node = self.make(bundle_capacity=2)

        for index, order in enumerate(permutations(node_module.PARTS)):
            stamp = ns(10.0) + index * 50_000_000
            self.now = stamp + 10_000_000
            self.parts(stamp, order)
            node.on_detections(detections(stamp, self.boxes()))

        results = self.rows('detection_result')
        self.assertEqual(len(results), 12)
        self.assertEqual({row['link_status'] for row in results}, {'LINKED'})

        # 두 bbox는 같은 묶음 깊이에서 서로 다른 면(1.0 m, 1.2 m)을 얻는다. stamp_ns는 그대로.
        first = [row for row in results if row['stamp_ns'] == ns(10.0)]
        self.assertEqual([round(row['plane_fit']['center'][2], 2) for row in first], [1.0, 1.2])
        self.assertEqual({row['frame_id'] for row in results}, {FRAME})
        self.assertEqual(first[0]['source'], 'robot_1/sensors/front_rgbd/processed')
        self.assertEqual((first[0]['class_id'], round(first[0]['score'], 3)), ('box', 0.9))
        self.assertEqual(first[0]['plane_fit']['validity_scope'], 'plane_fit')
        self.assertEqual(first[0]['plane_fit']['stamp_origin'], 'header')
        self.assertEqual(first[0]['plane_fit']['roi_origin'], 'c1_detection')
        self.assertEqual(first[0]['transform'], {'status': 'DISABLED'})

        # 대상 판정·흡착 관련 값은 만들지 않는다.
        for key in ('object_id', 'confidence', 'mass_kg', 'suction_suitable', 'association'):
            self.assertNotIn(key, json.dumps(first[0]).replace('normal_convention', ''))

        # 처리한 묶음은 남기지 않는다.
        self.assertEqual(node.state()['linker']['buffered'], 0)

    def test_epoch_one_nanosecond_apart_is_not_linked(self):
        node = self.make(time_source='system')
        self.now = EPOCH + 2
        self.parts(EPOCH)
        self.parts(EPOCH + 2)
        node.on_detections(detections(EPOCH + 1, self.boxes()))
        self.assertEqual(len(node.pending_detections), 1)
        node.on_detections(detections(EPOCH + 2, self.boxes()))
        results = self.rows('detection_result')
        self.assertEqual([row['stamp_ns'] for row in results], [EPOCH + 2, EPOCH + 2])
        self.assertEqual({row['plane_fit']['stamp_ns'] for row in results}, {EPOCH + 2})
        node.poll()
        # EPOCH+1 검출은 EPOCH·EPOCH+2 어느 묶음에도 결합되지 않는다(float 초로는 셋이 같다).
        self.assertEqual(self.reasons(), [['OUT_OF_ORDER_DETECTION_ARRAY']])

    def test_detection_before_parts_waits_bounded(self):
        node = self.make()
        node.on_detections(detections(ns(10.0), self.boxes()))
        self.assertEqual(len(node.pending_detections), 1)
        self.wall += 0.1
        self.parts(ns(10.0), ('camera_info', 'image', 'depth_image'))
        rows = self.rows('detection_result')
        self.assertEqual([row['link_status'] for row in rows], ['LINKED', 'LINKED'])
        self.assertAlmostEqual(rows[0]['wait_wall_s'], 0.1, places=6)

        # 묶음이 끝내 오지 않으면 감독 시계 상한 뒤 대기 시간 초과로 거부한다(만료와 구분).
        self.now = ns(10.2)
        node.on_detections(detections(ns(10.2), self.boxes()))
        self.wall += 0.39
        node.poll()
        self.assertEqual(self.rows('detection_array_rejected'), [])
        self.wall += 0.02
        node.poll()
        self.assertEqual(self.reasons()[-1], ['NO_MATCHING_FRAME_TIMEOUT:missing=all'])

    def test_partial_parts_time_out_and_detection_gets_reason(self):
        node = self.make()
        node.on_part('image', color_message(ns(10.0)))
        node.on_part('camera_info', info_message(ns(10.0)))
        node.on_detections(detections(ns(10.0), self.boxes()))
        self.wall += 0.31
        node.poll()
        self.assertEqual(self.reasons()[0], ['FRAME_ASSEMBLY_TIMEOUT:depth_image'])

        # 늦게 온 깊이는 그 stamp에 다시 묶이지 않는다(거부된 stamp 뒤 도착).
        node.on_part('depth_image', depth_message(ns(10.0)))
        self.assertEqual(node.assembler.counts['late_after_rejected'], 1)
        self.assertEqual(node.state()['linker']['buffered'], 0)

    def test_latest_depth_is_not_substituted(self):
        node = self.make()
        self.parts(ns(10.0))
        self.now = ns(10.1)
        self.parts(ns(10.1))
        node.on_detections(detections(ns(10.05), self.boxes()))
        self.wall += 0.41
        node.poll()
        self.assertEqual(self.rows('detection_result'), [])
        self.assertEqual(self.reasons()[0], ['NO_MATCHING_FRAME_TIMEOUT:missing=all'])

    def test_mismatch_invalid_and_duplicate_inputs(self):
        node = self.make(detection_capacity=8)
        self.now = ns(10.31)

        # 다른 카메라 검출은 받지 않는다.
        node.on_detections(detections(ns(10.0), self.boxes(), frame=OTHER_FRAME))
        # 같은 카메라 부품의 형식 문제는 그 stamp를 거부하고 검출에 사유를 남긴다.
        node.on_part('depth_image', depth_message(ns(10.1), encoding='16UC1'))
        node.on_detections(detections(ns(10.1), self.boxes()))
        node.on_part('camera_info', info_message(ns(10.15), distortion=0.1))
        node.on_detections(detections(ns(10.15), self.boxes()))
        node.on_part('image', color_message(ns(10.2), width=320, height=240))
        node.on_detections(detections(ns(10.2), self.boxes()))
        # 같은 stamp 부품 중복(같은 내용 / 다른 내용)은 stamp 전체 거부
        node.on_part('image', color_message(ns(10.25)))
        node.on_part('image', color_message(ns(10.25)))
        node.on_detections(detections(ns(10.25), self.boxes()))
        node.on_part('image', color_message(ns(10.27)))
        node.on_part('image', color_message(ns(10.27), fill=9))
        node.on_detections(detections(ns(10.27), self.boxes()))
        # 정상 묶음 + 같은 검출 배열 두 번
        self.parts(ns(10.3))
        node.on_detections(detections(ns(10.3), self.boxes()))
        node.on_detections(detections(ns(10.3), self.boxes()))

        self.assertEqual(self.reasons(), [
            ['DETECTION_FRAME_MISMATCH'], ['FRAME_PART_INVALID:depth_image:ENCODING_UNSUPPORTED'],
            ['FRAME_PART_INVALID:camera_info:CAMERA_MODEL_UNSUPPORTED'],
            ['FRAME_PART_INVALID:image:IMAGE_SIZE_UNEXPECTED'], ['FRAME_DUPLICATE_PART:image'],
            ['FRAME_CONFLICTING_PART:image'], ['DUPLICATE_DETECTION_ARRAY']])
        self.assertEqual(len(self.rows('detection_result')), 2)
        # 다른 카메라 frame 문자열은 기록에 남기지 않는다.
        self.assertIsNone(self.rows('detection_array_rejected')[0]['frame_id'])

    def test_other_frame_part_does_not_disturb_valid_stamp(self):
        node = self.make()
        node.on_part('image', color_message(ns(10.0)))
        # 같은 stamp의 다른 카메라 부품은 버리기만 하고 정상 조립을 망치지 않는다.
        node.on_part('image', color_message(ns(10.0), frame=OTHER_FRAME))
        node.on_part('depth_image', depth_message(ns(10.0)))
        node.on_part('camera_info', info_message(ns(10.0)))
        node.on_detections(detections(ns(10.0), self.boxes()))
        self.assertEqual(len(self.rows('detection_result')), 2)
        self.assertEqual(node.counts['DROP_image_FRAME_MISMATCH'], 1)

    def test_image_and_camera_info_limits_before_copy(self):
        node = self.make(detection_capacity=8)
        self.now = ns(10.5)
        padded = color_message(ns(10.0), step=640 * 3 + 4)
        short = color_message(ns(10.05))
        short.data = bytes(100)
        cases = [('image', padded, 'IMAGE_STEP_INVALID'), ('image', short, 'IMAGE_LAYOUT_INVALID'),
                 ('camera_info', info_message(ns(10.1), d_length=9), 'CAMERA_INFO_LIMIT'),
                 ('camera_info', info_message(ns(10.15), model='m' * 65), 'CAMERA_INFO_LIMIT'),
                 ('camera_info', info_message(ns(10.2), width=320), 'CAMERA_INFO_SIZE_UNEXPECTED')]

        expected = {}

        with patch.object(node_module, 'depth_array', side_effect=AssertionError('copied')) as copied:
            for part, message, code in cases:
                node.on_part(part, message)
                key = f'DROP_{part}_{code}'
                expected[key] = expected.get(key, 0) + 1
                self.assertEqual(node.counts[key], expected[key], code)

            bad_depth = depth_message(ns(10.25))
            bad_depth.step = 640 * 4 + 8
            node.on_part('depth_image', bad_depth)
            self.assertEqual(copied.call_count, 0)

        self.assertEqual(node.counts['DROP_depth_image_IMAGE_STEP_INVALID'], 1)

    def test_stale_future_and_empty_arrays(self):
        node = self.make()
        self.now = ns(11.0)
        node.on_part('image', color_message(ns(10.4)))
        self.assertEqual(node.counts['DROP_image_STALE'], 1)
        node.on_detections(detections(ns(10.4), self.boxes()))
        node.on_detections(detections(ns(11.2), self.boxes()))
        self.parts(ns(10.9))
        node.on_detections(detections(ns(10.9), []))

        self.assertEqual(self.reasons(), [['STALE_DETECTION'], ['FUTURE_DETECTION']])
        empty = self.rows('detection_array_empty')
        self.assertEqual([row['stamp_ns'] for row in empty], [ns(10.9)])
        self.assertEqual(node.state()['linker']['buffered'], 0)

        # 검출 대기 중 sim 시각이 지나 만료되면 STALE_DETECTION(대기 시간 초과와 다른 사유)
        node.on_detections(detections(ns(11.0), self.boxes()))
        self.now = ns(11.6)
        self.parts(ns(11.0))
        self.assertEqual(self.reasons()[-1], ['STALE_DETECTION'])

    def test_buffers_are_bounded(self):
        node = self.make(bundle_capacity=2, part_capacity=2, detection_capacity=2)

        for index in range(4):
            self.parts(ns(10.0) + index * 10_000_000)

        node.on_detections(detections(ns(10.0), self.boxes()))
        self.assertEqual(self.reasons()[0], ['FRAME_EVICTED_BEFORE_DETECTION'])
        queues = node.state()['queues']
        self.assertEqual((queues['bundles']['size'], queues['bundles']['evicted'], queues['bundles']['high_water']),
                         (2, 2, 2))
        self.assertLessEqual(node.state()['linker']['buffered_bytes'], 2 * 640 * 480 * 4)

        # 미완성 stamp 상한: 세 번째 미완성 stamp가 오면 가장 오래된 것을 거부
        self.now = ns(10.12)

        for stamp in (ns(10.1), ns(10.11), ns(10.12)):
            node.on_part('image', color_message(stamp))

        self.assertEqual(node.assembler.outcome(ns(10.1)), 'ASSEMBLY_OVERFLOW:depth_image+camera_info')
        self.assertEqual((len(node.assembler.pending), node.assembler.high_water['pending']), (2, 2))

        # 검출 대기 상한
        self.now = ns(10.22)

        for stamp in (ns(10.2), ns(10.21), ns(10.22)):
            node.on_detections(detections(stamp, self.boxes()))

        self.assertEqual(self.reasons()[-1], ['DETECTION_QUEUE_OVERFLOW'])
        self.assertEqual((len(node.pending_detections), node.high_water['pending_detections']), (2, 2))

    def test_outcome_history_is_bounded_including_eviction_path(self):
        node = self.make(bundle_capacity=2, outcome_history=3)

        for index in range(10):
            self.now = ns(10.0) + index * 10_000_000
            self.parts(self.now)
            self.assertLessEqual(len(node.assembler.outcomes), 3)

        self.assertEqual(node.assembler.high_water['outcomes'], 3)
        # 이력에서 밀려난 stamp의 늦은 부품은 판단 불가로만 센다.
        node.on_part('image', color_message(ns(10.0) + 50_000_000))
        self.assertEqual(node.assembler.counts['late_history_evicted'], 1)

    def test_too_many_detections_is_atomic_and_identity_first(self):
        node = self.make()
        self.parts(ns(10.0))
        boxes = self.boxes() * 3   # 6개 > 상한 4

        # 다른 카메라·미래 stamp의 큰 배열은 정상 묶음·순서 기록을 바꾸지 않는다.
        node.on_detections(detections(ns(10.0), boxes, frame=OTHER_FRAME))
        node.on_detections(detections(ns(10.2), boxes))
        self.assertEqual((node.state()['linker']['buffered'], node.linker.last_array_stamp), (1, None))

        # 신원이 유효하면 배열 전체를 거부(결과 0개)하고 그 묶음을 해제한다.
        node.on_detections(detections(ns(10.0), boxes))
        self.assertEqual(self.reasons(), [['DETECTION_FRAME_MISMATCH'], ['FUTURE_DETECTION'],
                                          ['DETECTION_COUNT_EXCEEDED']])
        self.assertEqual(self.rows('detection_result'), [])
        self.assertEqual(node.state()['linker']['buffered'], 0)
        node.on_detections(detections(ns(10.0), self.boxes()))
        self.assertEqual(self.reasons()[-1], ['DUPLICATE_DETECTION_ARRAY'])

        # 정확 묶음이 없는 큰 배열은 대기하지 않고 거부하며 이후 정상 연결을 막지 않는다.
        self.now = ns(10.1)
        node.on_detections(detections(ns(10.1), boxes))
        self.assertEqual((len(node.pending_detections), self.reasons()[-1]), (0, ['DETECTION_COUNT_EXCEEDED']))
        self.parts(ns(10.1))
        node.on_detections(detections(ns(10.1), self.boxes()))
        self.assertEqual(len(self.rows('detection_result')), 2)


class LateParts(NodeCase):
    def test_late_identical_conflicting_and_limited_diagnostics(self):
        node = self.make()
        self.parts(ns(10.0))

        # 조립 뒤 같은 내용(동일)·다른 내용(상충)
        node.on_part('image', color_message(ns(10.0)))

        for fill in range(1, 6):
            node.on_part('image', color_message(ns(10.0), fill=fill))

        counts = node.assembler.counts
        self.assertEqual((counts['late_identical'], counts['late_conflicting']), (1, 5))
        # 상세 진단 행은 사유별 상한(3)까지만, 나머지는 카운터
        diagnostics = [row for row in self.rows('diagnostic') if row['code'] == 'LATE_CONFLICTING']
        self.assertEqual(len(diagnostics), 3)
        self.assertEqual(node.counts['LATE_CONFLICTING'], 5)
        self.assertGreaterEqual(node.counts['diagnostic_suppressed'], 2)

        # 검출 사용 뒤(RELEASED)에도 상충은 진단일 뿐, 이미 낸 결과를 바꾸지 않는다.
        node.on_detections(detections(ns(10.0), self.boxes()))
        results = self.rows('detection_result')
        self.assertEqual(len(results), 2)
        node.on_part('depth_image', depth_message(ns(10.0), depth=np.ones((480, 640), np.float32)))
        self.assertEqual(node.assembler.outcome(ns(10.0)), 'RELEASED')
        self.assertEqual(counts['late_conflicting'], 6)
        self.assertEqual(self.rows('detection_result'), results)


class Clock(NodeCase):
    def test_clock_zero_and_reset(self):
        node = self.make()
        self.now = 0
        self.parts(ns(10.0))
        node.on_detections(detections(ns(10.0), self.boxes()))
        self.assertEqual(node.counts['CLOCK_NOT_STARTED'], 3)
        self.assertEqual(self.reasons()[0], ['CLOCK_NOT_STARTED'])

        self.now = ns(10.1)
        self.parts(ns(10.0))
        node.on_detections(detections(ns(10.1), self.boxes()))
        self.assertEqual(len(node.pending_detections), 1)

        # sim 시각 역행: 대기 검출은 사유와 함께 기록하고 모든 버퍼를 비운다.
        self.now = ns(1.0)
        node.on_part('image', color_message(ns(1.0)))
        reset = self.rows('clock_reset')[0]
        self.assertEqual((reset['dropped']['buffered'], reset['previous_ns'], reset['now_ns']),
                         (1, ns(10.1), ns(1.0)))
        self.assertEqual(self.reasons()[-1], ['CLOCK_RESET'])
        self.assertEqual(node.state()['linker']['buffered'], 0)

        # 새 시각으로 다시 연결된다.
        node.on_part('depth_image', depth_message(ns(1.0)))
        node.on_part('camera_info', info_message(ns(1.0)))
        node.on_detections(detections(ns(1.0), self.boxes()))
        self.assertEqual([row['link_status'] for row in self.rows('detection_result')], ['LINKED', 'LINKED'])


class Transform(NodeCase):
    # optical → base_link 변환(robot_1 front_rgbd 장착과 같은 축 배치의 시험값)
    def transform(self, stamp):
        message = TransformStamped()
        set_stamp(message.header, stamp, 'robot_1/base_link')
        message.child_frame_id = FRAME
        message.transform.translation.x, message.transform.translation.y = 0.73, -0.1
        message.transform.translation.z = 0.2
        # base ← optical: optical Z→base X, optical X→base −Y, optical Y→base −Z
        rotation = message.transform.rotation
        rotation.x, rotation.y, rotation.z, rotation.w = (-0.5, 0.5, -0.5, 0.5)

        return message

    def test_transform_uses_observation_stamp_only(self):
        node = self.make(**TF_PARAMETERS)

        # TF 없음: 상한 동안 기다린 뒤 실패 사유를 남긴다(최신 TF 대체 없음). optical 관측은 그대로 기록.
        self.parts(ns(10.0))
        node.on_detections(detections(ns(10.0), self.boxes()[:1]))
        self.assertEqual(self.rows('detection_result'), [])
        self.wall += 0.21
        node.poll()
        failed = self.rows('detection_result')[0]
        self.assertEqual(failed['transform']['status'], 'FAILED')
        self.assertEqual(failed['transform']['reason'], 'TF_UNAVAILABLE_AT_STAMP')
        self.assertEqual(failed['plane_fit']['status'], 'VALID')

        # 다른 시각(10.5)의 동적 TF 하나만 있으면 관측 시각(10.1)은 조회하지 않는다.
        node.tf_buffer.set_transform(self.transform(ns(10.5)), 'test')
        self.now = ns(10.1)
        self.parts(ns(10.1))
        node.on_detections(detections(ns(10.1), self.boxes()[:1]))
        self.wall += 0.21
        node.poll()
        self.assertEqual(self.rows('detection_result')[-1]['transform']['reason'], 'TF_UNAVAILABLE_AT_STAMP')

        # 정적 장착 TF가 있으면 관측 stamp_ns로 변환한다.
        node.tf_buffer.set_transform_static(self.transform(0), 'test')
        self.now = ns(10.2)
        self.parts(ns(10.2))
        node.on_detections(detections(ns(10.2), self.boxes()[:1]))
        moved = self.rows('detection_result')[-1]
        self.assertEqual(moved['transform']['status'], 'OK', moved['transform'])
        self.assertEqual(moved['transform']['transform_stamp_ns'], moved['stamp_ns'])
        center = moved['plane_fit']['center']
        expected = [0.73 + center[2], -0.1 - center[0], 0.2 - center[1]]
        np.testing.assert_allclose(moved['transform']['observation']['center'], expected, atol=1e-9)
        self.assertEqual(moved['transform']['observation']['observed_frame_id'], FRAME)
        self.assertEqual(moved['transform']['lineage'], [[FRAME, 'robot_1/base_link', moved['stamp_ns']]])

    def test_system_time_epoch_transform_is_exact(self):
        # 보완 전에는 epoch float 왕복 오차로 TRANSFORM_STAMP_MISMATCH가 날 수 있었다.
        node = self.make(time_source='system', **TF_PARAMETERS)
        node.tf_buffer.set_transform_static(self.transform(0), 'test')
        stamps = [EPOCH + offset for offset in (0, 1, 7, 999)]

        for stamp in stamps:
            self.now = stamp
            self.parts(stamp)
            node.on_detections(detections(stamp, self.boxes()[:1]))

        rows = self.rows('detection_result')
        self.assertEqual([row['transform']['status'] for row in rows], ['OK'] * 4)
        self.assertEqual([row['transform']['transform_stamp_ns'] for row in rows], stamps)

    def test_transform_header_ns_must_match(self):
        node = self.make(**TF_PARAMETERS)
        node.tf_buffer.set_transform_static(self.transform(0), 'test')
        original = node.tf_buffer.lookup_transform

        # 응답 header가 1 ns 다르면 변환하지 않는다(최신·근사 시각 대체 금지).
        def shifted(target, source, when):
            result = original(target, source, when)
            result.header.stamp.nanosec += 1

            return result

        node.tf_buffer.lookup_transform = shifted
        self.parts(ns(10.0))
        node.on_detections(detections(ns(10.0), self.boxes()[:1]))
        self.assertEqual(self.rows('detection_result')[0]['transform']['reason'], 'TF_STAMP_MISMATCH')

    def test_tf_queue_full_is_array_atomic(self):
        node = self.make(**TF_PARAMETERS)   # TF 없음: 결과가 대기 큐에 머문다. 상한 4
        self.parts(ns(10.0))
        node.on_detections(detections(ns(10.0), self.boxes()))
        self.assertEqual(len(node.pending_tf), 2)
        self.now = ns(10.1)
        self.parts(ns(10.1))
        node.on_detections(detections(ns(10.1), self.boxes() + self.boxes()[:1]))   # 유효 3개, 2+3 > 4

        rows = [row for row in self.rows('detection_result') if row['stamp_ns'] == ns(10.1)]
        self.assertEqual([row['transform']['reason'] for row in rows], ['TF_QUEUE_FULL'] * 3)
        self.assertEqual({row['plane_fit']['status'] for row in rows}, {'VALID'})
        self.assertEqual((len(node.pending_tf), node.high_water['pending_tf']), (2, 2))


class Record(NodeCase):
    def test_record_limit_stops_without_rewriting(self):
        probe = self.make()
        start = len(Path(probe.cfg.record_path).read_bytes())

        limit = max(start + node_module.MARKER_RESERVE + 200, 4 * node_module.MARKER_RESERVE + 1)
        node = self.make(max_record_bytes=limit)

        with self.assertRaises(node_module.RecordStop) as caught:
            for index in range(20):
                self.now = ns(10.0) + index * 10_000_000
                self.parts(self.now)
                node.on_detections(detections(self.now, self.boxes()))

        self.assertEqual(caught.exception.code, node_module.EXIT_RECORD_LIMIT)
        rows = self.rows()
        self.assertEqual(rows[-1]['event'], 'record_limit_reached')
        self.assertEqual(sum(row['event'] == 'record_limit_reached' for row in rows), 1)
        self.assertLessEqual(len(Path(node.cfg.record_path).read_bytes()), node.cfg.max_record_bytes)
        count = len(rows)

        # 이후 입력·종료에서도 다시 쓰지 않고, 종료 코드는 상한 도달(정상 완료 아님)이다.
        self.now = ns(11.0)
        self.parts(ns(11.0))
        node.on_detections(detections(ns(11.0), self.boxes()))
        node.close()
        self.assertEqual(len(self.rows()), count)
        self.assertNotIn('shutdown', [row['event'] for row in self.rows()])
        self.assertEqual(node.exit_code, node_module.EXIT_RECORD_LIMIT)

    def test_write_error_fails_once_with_code(self):
        node = self.make()
        real = node.record_file

        class Full:
            # 디스크 가득 참을 흉내 내는 파일(쓰기 시도 수를 센다)
            calls = 0
            closed = False

            def write(self, data):
                Full.calls += 1
                raise OSError(errno.ENOSPC, 'No space left on device')

            def close(self):
                real.close()

        node.record_file = Full()
        self.parts(ns(10.0))

        with self.assertRaises(node_module.RecordStop) as caught:
            node.on_detections(detections(ns(10.0), self.boxes()))

        self.assertEqual(caught.exception.code, node_module.EXIT_RECORD_FAILED)
        self.assertIn('RECORD_WRITE_FAILED', caught.exception.reason)
        node.on_detections(detections(ns(10.05), self.boxes()))
        node.close()
        self.assertEqual((Full.calls, node.exit_code, node.record_state),
                         (1, node_module.EXIT_RECORD_FAILED, 'FAILED'))


class Fuzz(NodeCase):
    def test_random_inputs_keep_every_queue_bounded(self):
        node = self.make(bundle_capacity=2, part_capacity=3, detection_capacity=2, outcome_history=6,
                         **TF_PARAMETERS)
        rng = random.Random(20261117)
        cfg = node.cfg
        boxes = self.boxes()
        depth_bytes = scene().astype(np.float32).tobytes()

        # 큰 메시지를 실제로 할당하지 않는다(최대 640×480 시험 영상). 순서·중복·잘못된 입력을 섞는다.
        for step in range(1000):
            self.wall += rng.choice((0.0, 0.01, 0.2))
            # 시각은 단조 증가(가끔 역행), stamp는 현재 주변 ±60 ms(미래·만료 포함)
            base = ns(10.0) + step * 2_000_000
            self.now = base + 60_000_000

            if rng.random() < 0.01:
                self.now = ns(1.0)   # 시계 역행

            # 20 ms 격자 stamp(같은 stamp가 여러 step에 걸쳐 나와 조립·연결이 실제로 일어나게 한다)
            grid = (self.now // 20_000_000) * 20_000_000
            stamp = grid - rng.randrange(4) * 20_000_000 + (60_000_000 if rng.random() < 0.05 else 0)
            kind = rng.random()
            frame = OTHER_FRAME if rng.random() < 0.05 else FRAME

            if kind < 0.45:
                part = rng.choice(node_module.PARTS)

                if part == 'image':
                    message = color_message(stamp, frame=frame, fill=rng.randrange(3))
                elif part == 'depth_image':
                    message = Image(height=480, width=640, encoding='32FC1', step=640 * 4, data=depth_bytes)
                    set_stamp(message.header, stamp, frame)
                else:
                    message = info_message(stamp, frame=frame, d_length=rng.choice((5, 5, 9)))

                node.on_part(part, message)
            elif kind < 0.8:
                chosen = rng.choice(([], boxes[:1], boxes, boxes * 3, [((-5.0, -5.0, -1.0, -1.0), 'x', 0.1)]))
                node.on_detections(detections(stamp, chosen, frame=frame))
            else:
                node.poll()

            queues = node.state()['queues']

            for name, queue in queues.items():
                self.assertLessEqual(queue['size'], queue['capacity'], (step, name))
                self.assertLessEqual(queue['high_water'], queue['capacity'], (step, name))

            self.assertLessEqual(len(node.linker.bundles), cfg.bundle_capacity)

        # 카운터 키는 유한 코드 형식이고, 상세 진단 행은 사유별 상한 이하다.
        pattern = re.compile(r'^[A-Za-z][A-Za-z0-9_]{0,99}$')
        keys = set(node.counts) | set(node.assembler.counts)
        self.assertTrue(all(pattern.match(key) for key in keys), keys)
        self.assertLess(len(keys), 120)
        per_code = {}

        for row in self.rows('diagnostic'):
            per_code[row['code']] = per_code.get(row['code'], 0) + 1

        self.assertTrue(all(count <= cfg.diagnostic_rows_per_reason for count in per_code.values()), per_code)
        # 시험이 실제로 여러 경로를 거쳤는지(연결·거부·늦은 부품·상한) 확인한다.
        self.assertGreater(node.counts['arrays_linked'], 0)
        self.assertGreater(node.counts['arrays_rejected'], 0)


class Shutdown(NodeCase):
    def test_close_records_pending_and_state(self):
        node = self.make()
        node.on_detections(detections(ns(10.0), self.boxes()))
        self.now = ns(10.05)
        node.on_part('image', color_message(ns(10.05)))
        node.close()
        rows = [json.loads(line) for line in Path(node.cfg.record_path).read_text().splitlines()]
        self.assertEqual(rows[-2]['reasons'], ['SHUTDOWN_PENDING'])
        self.assertEqual(rows[-1]['event'], 'shutdown')
        self.assertEqual(rows[-1]['queues']['pending_parts']['size'], 1)
        self.assertTrue(node.record_file.closed)
        self.assertEqual(node.exit_code, 0)


if __name__ == '__main__':
    unittest.main()
