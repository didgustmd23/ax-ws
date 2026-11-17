#!/usr/bin/env python3
"""C1 검출(Detection2DArray) ↔ 같은 RGB-D 카메라 깊이 연결(순수 함수·클래스, ROS 노드 없음).

전제: C1은 front_rgbd/processed/image를 명시 구성으로 입력받아 검출 header에 원 컬러 관측 stamp·frame을 복사한다
(object_detection.py detection_message). 기본 front_rgb 구성의 bbox는 다른 카메라 픽셀이라 여기서 받지 않는다.

연결 규칙(최소안)
  - 권위 시각은 header 정수 stamp_ns(sec×10^9 + nanosec)다. 검출과 RGB-D 묶음(컬러·깊이·CameraInfo)의 stamp_ns가
    정수로 정확히 같아야 한다. float 초는 비교에 쓰지 않는다. B1 sync_slop 0.03 s 계약을 바꾸지 않는다.
  - 검출 frame·묶음 frame·설정 frame이 같아야 한다. 묶음 source도 설정과 같아야 한다.
  - 유한 버퍼(용량·최대 나이·미래 허용은 명시 설정). 최신 깊이에 과거 bbox를 결합하지 않는다.
  - 한 Detection2DArray의 여러 bbox는 같은 묶음을 각각 쓴다. 배열 처리 뒤 그 묶음과 이전 묶음을 버린다.
  - 중복 묶음·역행 묶음·이미 처리한 stamp의 검출 배열은 거부한다. sim 시계 역행이면 버퍼를 비운다.
  - 입력 상한(검출 수·가설 수·class_id 길이)은 frame·stamp·미래·만료·중복·역행 검사를 모두 통과한 뒤 본다.
    넘으면 배열 전체를 거부한다(결과 0개). 정확 stamp 묶음이 있을 때만 그 배열을 처리 완료로 보고 묶음을 해제하며,
    없으면 연결기 상태를 바꾸지 않는다(무효·비순서 입력이 정상 묶음·순서 기록을 오염시키지 않게).
  - class_id·score는 검출 출처로만 남긴다. Detection2D.id는 보존하지 않고 있었는지만 남긴다. D object_id로 채우지 않는다.
  - 결과 관측의 VALID는 C2 plane_fit 유효일 뿐 목표 물체 일치가 아니다. TargetAssociation은 만들지 않는다.
"""

from collections import OrderedDict
from dataclasses import dataclass
import math

import numpy as np

import surface_estimation as se


@dataclass(frozen=True)
class LinkSettings:
    # 연결 설정. 기본값 없음(호출자가 근거와 함께 정한다). 운영 승인값 아님.
    camera_frame: str            # 받을 optical frame(예: robot_1/front_rgbd_optical_frame)
    source: str                  # 받을 처리 단계 출처(예: robot_1/sensors/front_rgbd/processed)
    capacity: int                # 버퍼에 둘 RGB-D 묶음 최대 수
    max_age_s: float             # 검출 stamp가 현재 시각보다 이만큼(s) 넘게 오래되면 만료
    future_tolerance_s: float    # 검출 stamp가 현재 시각보다 이만큼(s) 넘게 앞서면 미래로 거부
    max_detections: int          # 배열 하나의 검출 수 상한
    max_hypotheses: int          # 검출 하나의 가설(results) 수 상한
    max_text_len: int            # class_id 길이 상한(문자 수)


@dataclass(frozen=True)
class Detection:
    # Detection2D 한 개에 대응하는 순수 값. bbox는 픽셀 중심·크기(vision_msgs BoundingBox2D와 같은 뜻).
    index: int
    center_u: float
    center_v: float
    size_u: float
    size_v: float
    class_id: str
    score: float
    has_detection_id: bool       # Detection2D.id가 비어 있지 않았는지(값은 보존하지 않음)


@dataclass(frozen=True)
class DetectionArray:
    # Detection2DArray에 대응. stamp_ns는 원 컬러 관측 header의 정수 ns, frame은 header.frame_id.
    stamp_ns: int
    frame_id: str
    detections: tuple

    # 표시용 초(파생값)
    @property
    def stamp_s(self):
        return self.stamp_ns / se.NS_PER_S


@dataclass(frozen=True)
class RgbdBundle:
    # 같은 stamp_ns의 processed 컬러 크기·깊이·내부 파라미터 묶음
    stamp_ns: int
    frame_id: str
    source: str
    color_size: tuple        # (width, height)
    depth: object            # se.DepthFrame
    intrinsics: object       # se.CameraIntrinsics


@dataclass(frozen=True)
class LinkResult:
    # 검출 1개의 연결 결과. observation은 C2 관측(연결 실패면 None)
    detection_index: int
    status: str              # LINKED / REJECTED
    reasons: tuple
    class_id: str
    score: float
    observation: object = None


# 초 설정값 → 정수 ns(설정 변환. 시각 식별자 아님)
def seconds_to_ns(value):
    return int(round(value * se.NS_PER_S))


# ROS header → (stamp_ns, 문제 사유 또는 None)
def header_stamp(header):
    try:
        return se.stamp_ns_from_header(header.stamp.sec, header.stamp.nanosec), None
    except ValueError:
        return None, 'DETECTION_STAMP_INVALID'


# 배열 크기 상한 검사(목록 복사·가설 순회 전에 길이만 본다). 문제 사유 또는 None
def size_problem(message, settings):
    detections = message.detections

    if len(detections) > settings.max_detections:
        return 'DETECTION_COUNT_EXCEEDED'

    for item in detections:
        if len(item.results) > settings.max_hypotheses:
            return 'HYPOTHESIS_COUNT_EXCEEDED'

        for result in item.results:
            class_id = result.hypothesis.class_id

            if not isinstance(class_id, str) or len(class_id) > settings.max_text_len:
                return 'TEXT_LIMIT'

    return None


# 검사를 통과한 vision_msgs Detection2DArray(또는 같은 필드를 가진 객체)를 순수 값으로 바꾼다
def from_message(message, stamp_ns):
    detections = []

    for index, item in enumerate(message.detections):
        best = max(item.results, key=lambda result: result.hypothesis.score) if item.results else None
        detections.append(Detection(
            index, float(item.bbox.center.position.x), float(item.bbox.center.position.y),
            float(item.bbox.size_x), float(item.bbox.size_y),
            best.hypothesis.class_id if best else '', float(best.hypothesis.score) if best else float('nan'),
            bool(item.id),
        ))

    return DetectionArray(stamp_ns, message.header.frame_id, tuple(detections))


# bbox 중심·크기를 정수 픽셀 ROI로 바꾼다. 유한·양수 크기가 아니면 None
def bbox_roi(detection, frame_id, stamp_ns):
    values = (detection.center_u, detection.center_v, detection.size_u, detection.size_v)

    if not all(isinstance(value, float) and math.isfinite(value) for value in values):
        return None

    if detection.size_u <= 0 or detection.size_v <= 0:
        return None

    u_min = int(math.floor(detection.center_u - detection.size_u / 2))
    v_min = int(math.floor(detection.center_v - detection.size_v / 2))
    u_max = int(math.ceil(detection.center_u + detection.size_u / 2))
    v_max = int(math.ceil(detection.center_v + detection.size_v / 2))

    return se.Roi(u_min, v_min, u_max, v_max, frame_id, stamp_ns, se.HEADER, origin='c1_detection')


class DetectionDepthLinker:
    # 같은 카메라 RGB-D 묶음 버퍼와 검출 배열 연결기. 스레드 안전하지 않다(호출자가 직렬화). 시각 인자는 정수 ns.
    def __init__(self, settings):
        counts = (settings.capacity, settings.max_detections, settings.max_hypotheses, settings.max_text_len)
        times = (settings.max_age_s, settings.future_tolerance_s)

        if (not settings.camera_frame or not settings.source
                or not all(se.is_integer(value) and value >= 1 for value in counts)
                or not all(isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
                           and value > 0 for value in times)):
            raise ValueError('LINK_SETTINGS_INVALID')

        self.settings = settings
        self.max_age_ns = seconds_to_ns(settings.max_age_s)
        self.future_ns = seconds_to_ns(settings.future_tolerance_s)
        self.bundles = OrderedDict()   # stamp_ns → RgbdBundle(오름차순)
        self.last_bundle_stamp = None
        self.last_array_stamp = None
        self.last_now = None
        self.evicted = 0
        self.resets = 0
        self.high_water = 0

    # 시계 역행이면 버퍼와 순서 기록을 비운다. 비웠으면 True
    def observe_clock(self, now_ns):
        if self.last_now is not None and now_ns < self.last_now:
            self.bundles.clear()
            self.last_bundle_stamp = self.last_array_stamp = None
            self.resets += 1
            self.last_now = now_ns

            return True

        self.last_now = now_ns

        return False

    # RGB-D 묶음을 넣는다. (수락 여부, 사유 목록)
    def add_bundle(self, bundle, now_ns):
        self.observe_clock(now_ns)
        reasons = []

        if bundle.frame_id != self.settings.camera_frame:
            reasons.append('BUNDLE_FRAME_MISMATCH')

        if bundle.source != self.settings.source:
            reasons.append('BUNDLE_SOURCE_MISMATCH')

        depth, intrinsics = bundle.depth, bundle.intrinsics

        if len({bundle.frame_id, depth.frame_id, intrinsics.frame_id}) != 1:
            reasons.append('BUNDLE_PART_FRAME_MISMATCH')

        if not se.is_integer(bundle.stamp_ns) or len({bundle.stamp_ns, depth.stamp_ns, intrinsics.stamp_ns}) != 1:
            reasons.append('BUNDLE_PART_STAMP_MISMATCH')

        # 연결기는 정확 시각(header) 묶음만 받는다.
        if {depth.stamp_origin, intrinsics.stamp_origin} != {se.HEADER}:
            reasons.append('BUNDLE_STAMP_NOT_EXACT')

        if len({bundle.source, depth.source, intrinsics.source}) != 1:
            reasons.append('BUNDLE_PART_SOURCE_MISMATCH')

        shape = getattr(depth.depth, 'shape', None)

        if (tuple(bundle.color_size) != (intrinsics.width, intrinsics.height)
                or shape != (intrinsics.height, intrinsics.width)):
            reasons.append('BUNDLE_SIZE_MISMATCH')

        if self.last_bundle_stamp is not None:
            if bundle.stamp_ns == self.last_bundle_stamp or bundle.stamp_ns in self.bundles:
                reasons.append('DUPLICATE_BUNDLE')
            elif bundle.stamp_ns < self.last_bundle_stamp:
                reasons.append('OUT_OF_ORDER_BUNDLE')

        if reasons:
            return False, reasons

        # 넣기 전에 비워 상한을 넘는 순간이 없게 한다.
        while len(self.bundles) >= self.settings.capacity:
            self.bundles.popitem(last=False)
            self.evicted += 1

        self.bundles[bundle.stamp_ns] = bundle
        self.last_bundle_stamp = bundle.stamp_ns
        self.high_water = max(self.high_water, len(self.bundles))

        return True, []

    # 배열 신원·순서 검사(상태를 바꾸지 않음). 문제 사유 또는 None
    def identity_problem(self, stamp_ns, frame_id, now_ns):
        if frame_id != self.settings.camera_frame:
            return 'DETECTION_FRAME_MISMATCH'

        if not se.is_integer(stamp_ns) or stamp_ns <= 0:
            return 'DETECTION_STAMP_INVALID'

        if stamp_ns - now_ns > self.future_ns:
            return 'FUTURE_DETECTION'

        if self.last_array_stamp is not None:
            if stamp_ns == self.last_array_stamp:
                return 'DUPLICATE_DETECTION_ARRAY'

            if stamp_ns < self.last_array_stamp:
                return 'OUT_OF_ORDER_DETECTION_ARRAY'

        if now_ns - stamp_ns > self.max_age_ns:
            return 'STALE_DETECTION'

        return None

    # 처리 완료 표시: 순서 기록 갱신, 이 stamp 이하 묶음 해제
    def finish(self, stamp_ns):
        self.last_array_stamp = stamp_ns

        for old in [key for key in self.bundles if key <= stamp_ns]:
            del self.bundles[old]

    # ROS 메시지를 받아들인다. (DetectionArray 또는 None, 사유 목록).
    # 신원·순서를 먼저 보고, 통과한 뒤에만 크기 상한을 본다. 크기 초과는 정확 묶음이 있을 때만 처리 완료로 표시한다.
    def admit(self, message, now_ns):
        self.observe_clock(now_ns)
        frame_id = message.header.frame_id

        if frame_id != self.settings.camera_frame:
            return None, ['DETECTION_FRAME_MISMATCH']

        stamp_ns, problem = header_stamp(message.header)
        problem = problem or self.identity_problem(stamp_ns, frame_id, now_ns)

        if problem:
            return None, [problem]

        problem = size_problem(message, self.settings)

        if problem:
            if stamp_ns in self.bundles:
                self.finish(stamp_ns)

            return None, [problem]

        return from_message(message, stamp_ns), []

    # 검출 배열 하나를 연결한다. 배열 단위 거부면 (사유 목록, []), 아니면 ([], 검출별 LinkResult 목록)
    def consume(self, array, now_ns, estimation_settings, robot_id):
        self.observe_clock(now_ns)
        stamp = array.stamp_ns
        problem = self.identity_problem(stamp, array.frame_id, now_ns)

        if problem:
            return [problem], []

        # 순수 값으로 들어온 배열도 같은 상한을 지킨다(admit을 거치지 않은 호출자).
        if len(array.detections) > self.settings.max_detections or any(
                len(detection.class_id) > self.settings.max_text_len for detection in array.detections):
            if stamp in self.bundles:
                self.finish(stamp)

            return ['DETECTION_COUNT_EXCEEDED' if len(array.detections) > self.settings.max_detections
                    else 'TEXT_LIMIT'], []

        bundle = self.bundles.get(stamp)

        if bundle is None:
            # 정확 stamp 묶음이 없으면 다른(최신) 묶음으로 대체하지 않는다.
            return ['NO_MATCHING_FRAME'], []

        results = []

        for detection in array.detections:
            base = dict(detection_index=detection.index, class_id=detection.class_id, score=detection.score)
            roi = bbox_roi(detection, array.frame_id, stamp)

            if roi is None:
                results.append(LinkResult(status='REJECTED', reasons=('BBOX_INVALID',), **base))
                continue

            observation = se.estimate_surface(robot_id, bundle.depth, bundle.intrinsics, roi, estimation_settings)
            status = 'LINKED' if observation.valid else 'REJECTED'
            results.append(LinkResult(status=status, reasons=tuple(observation.reasons),
                                      observation=observation, **base))

        # 배열의 모든 bbox를 처리한 뒤에만 이 묶음과 이전 묶음을 버린다(재사용·과거 결합 방지).
        self.finish(stamp)

        return [], results

    # 버퍼 상태(메모리 상한 확인용)
    def state(self):
        return {'buffered': len(self.bundles), 'capacity': self.settings.capacity, 'high_water': self.high_water,
                'evicted': self.evicted, 'resets': self.resets,
                'buffered_bytes': int(sum(np.asarray(b.depth.depth).nbytes for b in self.bundles.values()))}
