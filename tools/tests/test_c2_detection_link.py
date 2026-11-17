"""C1 검출 ↔ 같은 RGB-D 깊이 연결(c2_detection_link.py)을 합성 입력으로 검사한다. ROS 노드·Gazebo 없이 실행한다."""
# [검사 범위] 합성 검출·깊이 fixture로 연결 규칙만 검사한다. 실제 C1 추론(모델·가중치) 검증이 아니다.
from decimal import Decimal
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/ax_perception/scripts'))
sys.path.insert(0, str(ROOT / 'tools/tests'))

import c2_detection_link as link  # noqa: E402
import surface_estimation as se  # noqa: E402
from test_c2_surface import (  # noqa: E402
    blank, draw_rectangle, FRAME, intrinsics, SOURCE, TEST_SETTINGS)

# 시험용 연결 설정(fixture, 운영 승인값 아님). 만료 0.5 s는 C1 max_age_s 기본 0.5와 같게 둔 시험값.
# 검출 수 4·가설 3·class_id 32자는 이 시험 장면에 맞춘 시험값이다.
SETTINGS = link.LinkSettings(camera_frame=FRAME, source=SOURCE, capacity=3, max_age_s=0.5, future_tolerance_s=0.05,
                             max_detections=4, max_hypotheses=3, max_text_len=32)


# 시험 문자 리터럴 초 → 정수 ns(10진 리터럴을 그대로 옮긴다. 측정값 float 복원이 아니다)
def ns(seconds):
    return int(Decimal(str(seconds)) * se.NS_PER_S)


# 두 상자 면이 보이는 합성 깊이(왼쪽 0.20×0.30 m @1.0 m, 오른쪽 0.20×0.30 m @1.2 m)
def scene():
    depth = draw_rectangle(blank(), (-0.15, 0.0, 1.0), (0, 0, -1), (1, 0, 0), 0.20, 0.30)

    return draw_rectangle(depth, (0.20, 0.0, 1.2), (0, 0, -1), (1, 0, 0), 0.20, 0.30)


# stamp_ns 묶음
def bundle(stamp, depth=None, frame=FRAME, source=SOURCE, color_size=(640, 480), origin=se.HEADER):
    depth = scene() if depth is None else depth
    camera = intrinsics(stamp=stamp, frame=frame, source=source, origin=origin)
    frame_depth = se.DepthFrame(depth, '32FC1', frame, stamp, origin, source)

    return link.RgbdBundle(stamp, frame, source, color_size, frame_depth, camera)


# 그린 면 하나를 감싸는 검출(픽셀 중심·크기). 두 면의 픽셀 범위는 시험에서 계산
def detection(index, u0, v0, u1, v1, class_id='box', score=0.9):
    return link.Detection(index, (u0 + u1) / 2, (v0 + v1) / 2, float(u1 - u0), float(v1 - v0), class_id, score, False)


# 장면 두 면의 픽셀 경계(유한 깊이 영역의 왼쪽·오른쪽 덩어리)
def face_boxes():
    depth = scene()
    columns = np.nonzero(np.isfinite(depth).any(axis=0))[0]
    rows = np.nonzero(np.isfinite(depth).any(axis=1))[0]
    split = columns[np.argmax(np.diff(columns))] + 1
    left = (columns.min(), rows.min(), split, rows.max() + 1)
    right = (columns[columns >= split].min(), rows.min(), columns.max() + 1, rows.max() + 1)
    # 오른쪽 면은 멀어서 행 범위가 더 좁다.
    right_rows = np.nonzero(np.isfinite(depth[:, right[0]:right[2]]).any(axis=1))[0]

    return [left, (right[0], right_rows.min(), right[2], right_rows.max() + 1)]


# 여유 2 px 안쪽 검출
def inner(index, box, margin=2, **kwargs):
    u0, v0, u1, v1 = box

    return detection(index, float(u0 + margin), float(v0 + margin), float(u1 - margin), float(v1 - margin), **kwargs)


# vision_msgs Detection2DArray와 같은 필드 구조(실제 메시지가 있으면 그것을 쓴다). stamp는 정수 ns
def message(stamp, frame, boxes, hypotheses=1, class_id=None):
    try:
        from builtin_interfaces.msg import Time
        from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
    except ImportError:
        return None

    result = Detection2DArray()
    result.header.frame_id = frame
    result.header.stamp = Time(sec=stamp // se.NS_PER_S, nanosec=stamp % se.NS_PER_S)

    for (u, v, w, h), name, score in boxes:
        item = Detection2D()
        item.header = result.header
        item.bbox.center.position.x, item.bbox.center.position.y = u, v
        item.bbox.size_x, item.bbox.size_y = w, h

        for index in range(hypotheses):
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = class_id or (name if index == 0 else 'other')
            hypothesis.hypothesis.score = score / (index + 1)
            item.results.append(hypothesis)

        result.detections.append(item)

    return result


class ExactLink(unittest.TestCase):
    def setUp(self):
        self.linker = link.DetectionDepthLinker(SETTINGS)
        self.boxes = face_boxes()

    def consume(self, array, now):
        return self.linker.consume(array, ns(now), TEST_SETTINGS, 'robot_1')

    def add(self, stamp, now=None, **kwargs):
        return self.linker.add_bundle(bundle(ns(stamp), **kwargs), ns(stamp if now is None else now))

    def array(self, stamp, detections, frame=FRAME):
        return link.DetectionArray(ns(stamp), frame, tuple(detections))

    def test_exact_stamp_and_frame_link_each_bbox(self):
        self.assertEqual(self.add(10.0, 10.05), (True, []))
        problems, results = self.consume(self.array(10.0, (inner(0, self.boxes[0]), inner(1, self.boxes[1]))), 10.3)
        self.assertEqual(problems, [])
        self.assertEqual([result.status for result in results], ['LINKED', 'LINKED'])

        # 두 bbox가 같은 묶음 깊이를 각각 썼다(두 번째도 지워지지 않음): 서로 다른 면 거리.
        centers = [result.observation.center[2] for result in results]
        self.assertAlmostEqual(centers[0], 1.0, delta=0.005)
        self.assertAlmostEqual(centers[1], 1.2, delta=0.005)

        # 검출 출처만 전달, 관측 stamp는 원 컬러 stamp_ns, object_id 없음
        self.assertEqual(results[0].class_id, 'box')
        self.assertEqual(results[0].observation.stamp_ns, ns(10.0))
        self.assertTrue(results[0].observation.stamp_exact)
        self.assertEqual(results[0].observation.roi_origin, 'c1_detection')
        self.assertFalse(hasattr(results[0], 'object_id'))
        self.assertEqual(results[0].observation.validity_scope, 'plane_fit')

        # 처리한 묶음은 버린다.
        self.assertEqual(self.linker.state()['buffered'], 0)

    def test_epoch_one_nanosecond_is_not_the_same_frame(self):
        epoch = 1_794_923_490 * se.NS_PER_S + 123_456_789
        self.linker.add_bundle(bundle(epoch), epoch)
        self.linker.add_bundle(bundle(epoch + 2), epoch + 2)
        # epoch+1 검출: float 초로는 epoch와 같지만 정확 묶음이 아니다.
        array = link.DetectionArray(epoch + 1, FRAME, (inner(0, self.boxes[0]),))
        self.assertEqual(self.linker.consume(array, epoch + 3, TEST_SETTINGS, 'robot_1'), (['NO_MATCHING_FRAME'], []))
        problems, results = self.linker.consume(link.DetectionArray(epoch + 2, FRAME, (inner(0, self.boxes[0]),)),
                                                epoch + 3, TEST_SETTINGS, 'robot_1')
        self.assertEqual((problems, results[0].observation.stamp_ns), ([], epoch + 2))

    def test_old_bbox_is_not_joined_with_latest_depth(self):
        self.add(10.0)
        self.add(10.1)
        # 10.05 stamp 검출: 정확 묶음이 없으므로 최신(10.1) 깊이로 대체하지 않는다.
        problems, results = self.consume(self.array(10.05, (inner(0, self.boxes[0]),)), 10.2)
        self.assertEqual((problems, results), (['NO_MATCHING_FRAME'], []))
        self.assertEqual(self.linker.state()['buffered'], 2)

        # 10.1 묶음을 쓴 뒤에는 그 이전 10.0 검출이 와도 묶음이 없다(이미 버려짐 + 역행).
        self.consume(self.array(10.1, (inner(0, self.boxes[0]),)), 10.2)
        problems, _ = self.consume(self.array(10.0, (inner(0, self.boxes[0]),)), 10.25)
        self.assertEqual(problems, ['OUT_OF_ORDER_DETECTION_ARRAY'])

    def test_late_detection_expires_and_buffer_is_bounded(self):
        for index in range(6):
            accepted, _ = self.add(10.0 + index / 10)
            self.assertTrue(accepted)

        state = self.linker.state()
        self.assertEqual((state['buffered'], state['evicted'], state['high_water']), (3, 3, 3))
        self.assertLessEqual(state['buffered_bytes'], 3 * 640 * 480 * 4)

        # 밀려난 묶음 stamp는 연결 불가
        problems, _ = self.consume(self.array(10.1, (inner(0, self.boxes[0]),)), 10.5)
        self.assertEqual(problems, ['NO_MATCHING_FRAME'])

        # 버퍼에 있어도 만료(현재 − stamp > 0.5 s)면 거부
        problems, _ = self.consume(self.array(10.3, (inner(0, self.boxes[0]),)), 10.81)
        self.assertEqual(problems, ['STALE_DETECTION'])

    def test_duplicate_reverse_future_and_clock_reset(self):
        self.add(10.0)
        self.assertEqual(self.add(10.0, 10.01), (False, ['DUPLICATE_BUNDLE']))
        self.add(10.2)
        self.assertEqual(self.add(10.1, 10.21), (False, ['OUT_OF_ORDER_BUNDLE']))

        array = self.array(10.2, (inner(0, self.boxes[0]),))
        self.assertEqual(self.consume(array, 10.3)[0], [])
        self.assertEqual(self.consume(array, 10.31)[0], ['DUPLICATE_DETECTION_ARRAY'])
        self.assertEqual(self.consume(self.array(10.5, ()), 10.4)[0], ['FUTURE_DETECTION'])

        # sim 시계 역행: 버퍼·순서 기록을 비우고 새로 시작한다.
        self.add(10.4)
        self.assertTrue(self.linker.observe_clock(ns(1.0)))
        self.assertEqual(self.linker.state()['buffered'], 0)
        self.assertEqual(self.linker.state()['resets'], 1)
        self.assertEqual(self.add(1.0), (True, []))
        self.assertEqual(self.consume(self.array(1.0, (inner(0, self.boxes[0]),)), 1.1)[0], [])

    def test_empty_invalid_and_other_camera(self):
        self.add(10.0)
        problems, results = self.consume(self.array(10.0, ()), 10.1)
        self.assertEqual((problems, results), ([], []))
        self.assertEqual(self.linker.state()['buffered'], 0)

        self.add(10.1)
        bad = (link.Detection(0, 300.0, 200.0, 0.0, 10.0, 'box', 0.9, False),
               link.Detection(1, float('nan'), 200.0, 10.0, 10.0, 'box', 0.9, False),
               link.Detection(2, 900.0, 900.0, 20.0, 20.0, 'box', 0.9, False))
        problems, results = self.consume(self.array(10.1, bad), 10.2)
        self.assertEqual(problems, [])
        self.assertEqual([r.reasons for r in results], [('BBOX_INVALID',), ('BBOX_INVALID',), ('ROI_INVALID',)])

        # front_rgb 검출(다른 카메라)은 받지 않는다.
        self.add(10.2)
        other = self.array(10.2, (inner(0, self.boxes[0]),), frame='robot_1/front_rgb_optical_frame')
        self.assertEqual(self.consume(other, 10.3), (['DETECTION_FRAME_MISMATCH'], []))

        # 다른 카메라·처리 단계·크기·비정확 시각의 묶음은 버퍼에 넣지 않는다.
        rgb = bundle(ns(10.3), frame='robot_1/front_rgb_optical_frame')
        self.assertIn('BUNDLE_FRAME_MISMATCH', self.linker.add_bundle(rgb, ns(10.3))[1])
        raw = bundle(ns(10.3), source='robot_1/sensors/front_rgbd/raw')
        self.assertIn('BUNDLE_SOURCE_MISMATCH', self.linker.add_bundle(raw, ns(10.3))[1])
        self.assertIn('BUNDLE_SIZE_MISMATCH', self.add(10.3, color_size=(320, 240))[1])
        self.assertIn('BUNDLE_STAMP_NOT_EXACT', self.add(10.3, origin=se.LEGACY_FLOAT)[1])

        mixed = link.RgbdBundle(ns(10.3), FRAME, SOURCE, (640, 480),
                                se.DepthFrame(scene(), '32FC1', FRAME, ns(10.3) - 1, se.HEADER, SOURCE),
                                intrinsics(stamp=ns(10.3)))
        self.assertIn('BUNDLE_PART_STAMP_MISMATCH', self.linker.add_bundle(mixed, ns(10.3))[1])

    def test_background_mixed_bbox_is_kept_as_rejection(self):
        # 실제 bbox처럼 두 면을 함께 감싸면 C2가 거부하며, 그 결과를 그대로 남긴다.
        self.add(10.0)
        (u0, v0, _, v1), (_, _, u1, _) = self.boxes
        problems, results = self.consume(
            self.array(10.0, (detection(0, float(u0), float(v0), float(u1), float(v1)),)), 10.1)
        self.assertEqual(problems, [])
        self.assertEqual(results[0].status, 'REJECTED')
        self.assertIn('MULTIPLE_SURFACES', results[0].reasons)

    def test_settings_have_no_defaults(self):
        with self.assertRaises(TypeError):
            link.LinkSettings()

        for bad in (dict(capacity=0), dict(max_age_s=0.0), dict(camera_frame=''), dict(capacity=True),
                    dict(future_tolerance_s=float('nan')), dict(max_detections=0), dict(max_hypotheses=1.5),
                    dict(max_text_len=False)):
            values = dict(SETTINGS.__dict__)
            values.update(bad)

            with self.assertRaises(ValueError):
                link.DetectionDepthLinker(link.LinkSettings(**values))


class SizeLimits(unittest.TestCase):
    # 크기 상한은 신원·순서를 확인한 뒤에만 적용하고, 원자적으로 거부한다.
    def setUp(self):
        self.linker = link.DetectionDepthLinker(SETTINGS)
        self.box = ((100.0, 100.0, 10.0, 10.0), 'box', 0.9)

    def admit(self, msg, now):
        if msg is None:
            self.skipTest('vision_msgs 없음(ROS 환경 필요)')

        return self.linker.admit(msg, ns(now))

    def test_too_many_detections_with_exact_bundle_is_atomic(self):
        self.linker.add_bundle(bundle(ns(10.0)), ns(10.0))
        array, reasons = self.admit(message(ns(10.0), FRAME, [self.box] * 5), 10.1)
        self.assertEqual((array, reasons), (None, ['DETECTION_COUNT_EXCEEDED']))
        # 처리 완료로 표시: 묶음 해제, 같은 stamp 재전송은 중복
        self.assertEqual(self.linker.state()['buffered'], 0)
        _, reasons = self.admit(message(ns(10.0), FRAME, [self.box]), 10.1)
        self.assertEqual(reasons, ['DUPLICATE_DETECTION_ARRAY'])

        # 상한과 같은 개수는 받는다.
        self.linker.add_bundle(bundle(ns(10.1)), ns(10.1))
        array, reasons = self.admit(message(ns(10.1), FRAME, [self.box] * 4), 10.15)
        self.assertEqual((len(array.detections), reasons), (4, []))

    def test_hypothesis_and_text_limits(self):
        self.linker.add_bundle(bundle(ns(10.0)), ns(10.0))
        _, reasons = self.admit(message(ns(10.0), FRAME, [self.box], hypotheses=4), 10.1)
        self.assertEqual(reasons, ['HYPOTHESIS_COUNT_EXCEEDED'])
        self.linker.add_bundle(bundle(ns(10.1)), ns(10.1))
        _, reasons = self.admit(message(ns(10.1), FRAME, [self.box], class_id='c' * 33), 10.15)
        self.assertEqual(reasons, ['TEXT_LIMIT'])

        # 순수 값 경로도 같은 상한을 지킨다.
        self.linker.add_bundle(bundle(ns(10.2)), ns(10.2))
        many = link.DetectionArray(ns(10.2), FRAME, tuple(detection(i, 0, 0, 5, 5) for i in range(5)))
        self.assertEqual(self.linker.consume(many, ns(10.25), TEST_SETTINGS, 'robot_1'),
                         (['DETECTION_COUNT_EXCEEDED'], []))
        self.assertEqual(self.linker.state()['buffered'], 0)

    def test_oversize_without_valid_identity_does_not_touch_state(self):
        self.linker.add_bundle(bundle(ns(10.0)), ns(10.0))
        self.linker.add_bundle(bundle(ns(10.1)), ns(10.1))
        too_many = [self.box] * 5
        cases = {
            'DETECTION_FRAME_MISMATCH': message(ns(10.1), 'robot_1/front_rgb_optical_frame', too_many),
            'FUTURE_DETECTION': message(ns(10.3), FRAME, too_many),
            'STALE_DETECTION': message(ns(9.5), FRAME, too_many),
            'DETECTION_STAMP_INVALID': message(0, FRAME, too_many),
        }

        for reason, msg in cases.items():
            _, reasons = self.admit(msg, 10.15)
            self.assertEqual(reasons, [reason])
            self.assertEqual(self.linker.last_array_stamp, None, reason)
            self.assertEqual(list(self.linker.bundles), [ns(10.0), ns(10.1)], reason)

        # 정확 묶음이 없는 개수 초과도 상태를 바꾸지 않는다.
        _, reasons = self.admit(message(ns(10.05), FRAME, too_many), 10.15)
        self.assertEqual(reasons, ['DETECTION_COUNT_EXCEEDED'])
        self.assertEqual((self.linker.last_array_stamp, len(self.linker.bundles)), (None, 2))

        # 정상 배열은 그대로 연결된다.
        array, reasons = self.admit(message(ns(10.1), FRAME, [self.box]), 10.15)
        problems, results = self.linker.consume(array, ns(10.15), TEST_SETTINGS, 'robot_1')
        self.assertEqual((reasons, problems, len(results)), ([], [], 1))

        # 처리 뒤 역행·중복 배열(개수 초과 포함)도 순서 기록을 바꾸지 않는다.
        for stamp, reason in ((10.0, 'OUT_OF_ORDER_DETECTION_ARRAY'), (10.1, 'DUPLICATE_DETECTION_ARRAY')):
            _, reasons = self.admit(message(ns(stamp), FRAME, too_many), 10.15)
            self.assertEqual(reasons, [reason])
            self.assertEqual(self.linker.last_array_stamp, ns(10.1))


class MessageAdapter(unittest.TestCase):
    def test_real_detection2d_fields(self):
        msg = message(ns(10.5), FRAME, [((300.0, 200.0, 40.0, 30.0), 'box', 0.8),
                                        ((100.0, 50.0, 10.0, 12.0), 'x', 0.3)], hypotheses=2)

        if msg is None:
            self.skipTest('vision_msgs 없음(ROS 환경 필요)')

        array = link.from_message(msg, ns(10.5))
        self.assertEqual((array.stamp_ns, array.frame_id, len(array.detections)), (ns(10.5), FRAME, 2))
        first = array.detections[0]
        self.assertEqual((first.center_u, first.center_v, first.size_u, first.size_v), (300.0, 200.0, 40.0, 30.0))
        self.assertEqual((first.class_id, first.has_detection_id), ('box', False))
        self.assertAlmostEqual(first.score, 0.8)
        roi = link.bbox_roi(first, FRAME, ns(10.5))
        self.assertEqual((roi.u_min, roi.v_min, roi.u_max, roi.v_max), (280, 185, 320, 215))
        self.assertEqual((roi.stamp_ns, roi.stamp_origin), (ns(10.5), se.HEADER))

        empty = link.from_message(message(ns(11.0), FRAME, []), ns(11.0))
        self.assertEqual(empty.detections, ())

    def test_header_stamp_is_integer_exact(self):
        header = SimpleNamespace(stamp=SimpleNamespace(sec=1_794_923_490, nanosec=123_456_789), frame_id=FRAME)
        self.assertEqual(link.header_stamp(header), (1_794_923_490_123_456_789, None))
        bad = SimpleNamespace(stamp=SimpleNamespace(sec=3, nanosec=se.NS_PER_S), frame_id=FRAME)
        self.assertEqual(link.header_stamp(bad), (None, 'DETECTION_STAMP_INVALID'))

    def test_duck_typed_message_without_ros(self):
        hypothesis = SimpleNamespace(hypothesis=SimpleNamespace(class_id='box', score=0.7))
        item = SimpleNamespace(bbox=SimpleNamespace(center=SimpleNamespace(position=SimpleNamespace(x=10, y=20)),
                                                    size_x=4, size_y=6), results=[hypothesis], id='track-7')
        header = SimpleNamespace(stamp=SimpleNamespace(sec=3, nanosec=250000000), frame_id=FRAME)
        array = link.from_message(SimpleNamespace(header=header, detections=[item]), 3_250_000_000)
        self.assertEqual(array.stamp_ns, 3_250_000_000)
        self.assertEqual(array.detections[0].size_v, 6.0)
        # Detection2D.id 값은 보존하지 않고 있었는지만 남긴다.
        self.assertIs(array.detections[0].has_detection_id, True)
        self.assertNotIn('track-7', repr(array))


if __name__ == '__main__':
    unittest.main()
