#!/usr/bin/env python3
"""C2 ROS 입력 노드: 같은 RGB-D 카메라 processed 컬러·깊이·CameraInfo와 명시 검출 토픽을 연결해 표면 관측을 기록한다.

범위
  - 입력: <namespace>/sensors/<camera>/processed/{image,depth_image,camera_info}(B1 출력)와 detection_topic
    (vision_msgs/Detection2DArray). 모두 SENSOR QoS. detection_topic은 기본값이 없으며 C1 기본 출력(front_rgb)을 쓰지 않는다.
  - 출력: record_path의 JSON Lines 로컬 기록뿐이다. 공개 Surface 메시지·토픽을 만들지 않는다.
  - 계산: c2_detection_link.DetectionDepthLinker·surface_estimation을 쓴다.
    VALID는 plane_fit 유효일 뿐 목표 물체 일치가 아니다. object_id·MATCH·confidence·질량·흡착 적합성을 만들지 않는다.

시각
  - 권위 시각은 header 정수 stamp_ns다. 조립·버퍼·만료·중복·순서·TF 조회·기록을 정수 ns로 한다. stamp를 다시 쓰지 않는다.
  - ROS 시계(time_source=sim이면 /clock)의 정수 ns로 만료·미래를 잰다. 부품 조립·검출 선도착·TF 대기는 CLOCK_BOOTTIME 상한.
  - ROS 시각 0(clock 미수신)이면 입력을 받지 않는다. 시각 역행이면 모든 버퍼를 비운다.

자원 상한(모두 필수 설정, 기본값 없음)
  - 입력 경계: 영상 해상도·step·data 길이·바이트, CameraInfo d·문자열 길이, 검출·가설 수·class_id 길이를
    보존·복사 전에 검사한다. 검출 수는 신원·순서 검사 뒤, 목록 복사 전에 본다.
  - 보존: 미완성 부품·묶음·검출 대기·TF 대기·조립 결과 이력·진단 상세 행·기록 파일 바이트.
  - 한계: rclpy가 콜백 전에 메시지 하나를 역직렬화하는 메모리와 tf2 캐시(시간 상한 tf_cache_s만, 개수 상한 없음)는
    이 노드가 개수로 제한하지 못한다.

종료 코드: 0 정상, 2 시작 거부(설정·기록 파일), 3 기록 쓰기 실패, 4 기록 바이트 상한 도달(정상 완료 아님).
"""

from collections import Counter, OrderedDict, deque
from dataclasses import dataclass
import hashlib
import json
import math
import os
import re
import time

import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data as SENSOR
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection2DArray

import c2_detection_link as link
import surface_estimation as se

PARTS = ('image', 'depth_image', 'camera_info')
COLOR_ENCODINGS = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}
TIME_SOURCES = ('sim', 'system')
EXIT_START, EXIT_RECORD_FAILED, EXIT_RECORD_LIMIT = 2, 3, 4
# 상한 도달 표식 1행을 쓰기 위해 남겨 두는 바이트(표식 행 길이 상한. 내부 구현값)
MARKER_RESERVE = 1024
# 진단·카운터 키로 허용하는 코드 형식(동적 문자열이 키가 되지 않게)
CODE = re.compile(r'^[A-Za-z][A-Za-z0-9_]{0,79}$')
# 조립 결과 중 묶음이 실제로 쓰였거나 쓰일 수 있었던 것(늦은 부품 지문 비교 대상)
USED_OUTCOMES = ('ASSEMBLED', 'RELEASED', 'EVICTED_BEFORE_DETECTION')

# 문자열 설정(빈 값이면 미정). target_frame은 선택
TEXT_KEYS = ('robot_id', 'camera', 'camera_frame', 'detection_topic', 'time_source', 'record_path',
             'settings_source', 'target_frame')
# 정수 설정(0이면 미정). TF 관련은 target_frame이 있을 때만 필수
INT_KEYS = ('bundle_capacity', 'part_capacity', 'detection_capacity', 'outcome_history', 'expected_width',
            'expected_height', 'max_image_bytes', 'max_detections_per_array', 'max_hypotheses_per_detection',
            'max_text_len', 'max_distortion_len', 'diagnostic_rows_per_reason', 'max_record_bytes',
            'tf_pending_capacity')
# 실수 설정(NaN이면 미정)
FLOAT_KEYS = ('max_age_s', 'future_tolerance_s', 'assembly_wait_s', 'detection_wait_s', 'poll_period_s',
              'state_period_s', 'tf_wait_s', 'tf_cache_s')
TF_KEYS = ('tf_wait_s', 'tf_cache_s', 'tf_pending_capacity')
ESTIMATION_INT = ('min_valid_points', 'ransac_iterations', 'ransac_seed')
# 기록에 남길 품질 항목(관측 기하 설명값. 성공 확률 아님)
QUALITY_KEYS = ('roi_pixel_count', 'valid_point_count', 'valid_fraction', 'edge_clipped', 'roi_used',
                'inlier_fraction', 'second_plane_fraction', 'component_fraction', 'plane_rms_m', 'plane_p95_m',
                'surface_point_count', 'incidence_deg')


@dataclass(frozen=True)
class NodeConfig:
    # 검사를 통과한 노드 설정
    robot_id: str
    camera: str
    camera_frame: str
    source: str
    detection_topic: str
    time_source: str
    record_path: str
    settings_source: str
    target_frame: str
    bundle_capacity: int
    part_capacity: int
    detection_capacity: int
    outcome_history: int
    expected_width: int
    expected_height: int
    max_image_bytes: int
    max_detections_per_array: int
    max_hypotheses_per_detection: int
    max_text_len: int
    max_distortion_len: int
    diagnostic_rows_per_reason: int
    max_record_bytes: int
    tf_pending_capacity: int
    max_age_s: float
    future_tolerance_s: float
    assembly_wait_s: float
    detection_wait_s: float
    poll_period_s: float
    state_period_s: float
    tf_wait_s: float
    tf_cache_s: float
    estimation: object


class RecordStop(Exception):
    # 기록을 더 쓸 수 없어 노드를 멈춘다(code: 종료 코드)
    def __init__(self, code, reason):
        super().__init__(reason)
        self.code = code
        self.reason = reason


# 감독 경과 시간(절전 포함). 측정 stamp가 아니라 대기 상한에만 쓴다
def boottime():
    return time.clock_gettime(time.CLOCK_BOOTTIME)


# 사유 문자열 → 유한 코드(첫 ':' 앞). 형식이 맞지 않으면 UNCLASSIFIED
def code_of(text):
    head = str(text).split(':', 1)[0]

    return head if CODE.match(head) else 'UNCLASSIFIED'


# 기록용 값 정리: 비유한 실수는 null, numpy 값은 파이썬 값
def clean(value):
    if isinstance(value, dict):
        return {str(key): clean(item) for key, item in value.items()}

    if isinstance(value, (list, tuple)):
        return [clean(item) for item in value]

    if isinstance(value, (bool, np.bool_)):
        return bool(value)

    if isinstance(value, (int, np.integer)):
        return int(value)

    if isinstance(value, (float, np.floating)):
        return float(value) if math.isfinite(value) else None

    return value


# 파라미터 값 사전으로 설정을 만든다. 모든 문제를 모아 CONFIG_INVALID ValueError로 거부한다
def parse_config(values, use_sim_time):
    problems = []
    texts = {}

    for name in TEXT_KEYS:
        value = values.get(name)

        if not isinstance(value, str) or (name != 'target_frame' and not value.strip()):
            problems.append(name)
            value = ''

        texts[name] = value.strip()

    tf = bool(texts['target_frame'])

    if texts['time_source'] and texts['time_source'] not in TIME_SOURCES:
        problems.append('time_source')

    # 시각 출처 선언과 실제 use_sim_time이 다르면 거부한다(암묵 시계 사용 금지).
    if texts['time_source'] in TIME_SOURCES and (texts['time_source'] == 'sim') != bool(use_sim_time):
        problems.append('time_source/use_sim_time')

    ints = {}

    for name in INT_KEYS:
        value = values.get(name)
        needed = name not in TF_KEYS or tf

        if needed and (not se.is_integer(value) or value < 1):
            problems.append(name)
            value = 0

        ints[name] = int(value) if se.is_integer(value) else 0

    floats = {}

    for name in FLOAT_KEYS:
        value = values.get(name)
        needed = name not in TF_KEYS or tf

        if needed and (not isinstance(value, float) or not math.isfinite(value) or value <= 0):
            problems.append(name)
            value = math.nan

        floats[name] = value if isinstance(value, float) else math.nan

    # 상호관계: 깊이(4 바이트/픽셀) 영상이 바이트 상한 안에 들어가야 한다.
    if ints['max_image_bytes'] and ints['max_image_bytes'] < 4 * ints['expected_width'] * ints['expected_height']:
        problems.append('max_image_bytes<4*width*height')

    # 한 배열의 결과가 TF 대기 큐에 한 번에 들어갈 수 있어야 한다(원자 처리).
    if tf and ints['tf_pending_capacity'] and ints['tf_pending_capacity'] < ints['max_detections_per_array']:
        problems.append('tf_pending_capacity<max_detections_per_array')

    if ints['max_record_bytes'] and ints['max_record_bytes'] <= 4 * MARKER_RESERVE:
        problems.append('max_record_bytes')

    if tf and texts['target_frame'] == texts['camera_frame']:
        problems.append('target_frame')

    if texts['record_path'] and not os.path.isabs(texts['record_path']):
        problems.append('record_path')

    # 기록에 남는 설정 문자열 길이
    if ints['max_text_len']:
        for name in ('robot_id', 'camera', 'camera_frame', 'target_frame', 'detection_topic', 'settings_source'):
            if len(texts[name]) > ints['max_text_len']:
                problems.append(name + '>max_text_len')

    estimation = None
    fields = {}

    for name in se.EstimationSettings.__dataclass_fields__:
        value = values.get('estimation.' + name)
        valid = se.is_integer(value) if name in ESTIMATION_INT else isinstance(value, float) and math.isfinite(value)

        if not valid:
            problems.append('estimation.' + name)

        fields[name] = value

    if not any(name.startswith('estimation.') for name in problems):
        estimation = se.EstimationSettings(**fields)

        try:
            se.check_settings(estimation)
        except ValueError as error:
            problems.append(f'estimation({error})')

    if problems:
        raise ValueError('CONFIG_INVALID: ' + ', '.join(problems))

    source = f'{texts["robot_id"]}/sensors/{texts["camera"]}/processed'

    return NodeConfig(source=source, estimation=estimation, **texts, **ints, **floats)


# 영상 메시지 경계 검사(보존·복사 전). 문제 코드 또는 None
def image_problem(message, depth, cfg):
    if depth:
        pixel = 4 if message.encoding == '32FC1' else None
    else:
        pixel = COLOR_ENCODINGS.get(message.encoding)

    if pixel is None:
        return 'ENCODING_UNSUPPORTED'

    if (message.width, message.height) != (cfg.expected_width, cfg.expected_height):
        return 'IMAGE_SIZE_UNEXPECTED'

    # 행 패딩 정책이 정해지기 전까지 step은 정확히 width × 픽셀 바이트만 받는다.
    if message.step != message.width * pixel:
        return 'IMAGE_STEP_INVALID'

    size = len(message.data)

    if size != message.step * message.height:
        return 'IMAGE_LAYOUT_INVALID'

    if size > cfg.max_image_bytes:
        return 'IMAGE_BYTES_EXCEEDED'

    return None


# CameraInfo 경계 검사(보존 전). 문제 코드 또는 None
def camera_info_problem(message, cfg):
    if (message.width, message.height) != (cfg.expected_width, cfg.expected_height):
        return 'CAMERA_INFO_SIZE_UNEXPECTED'

    if len(message.d) > cfg.max_distortion_len or len(message.distortion_model) > cfg.max_text_len:
        return 'CAMERA_INFO_LIMIT'

    if len(message.k) != 9 or len(message.p) != 12:
        return 'CAMERA_INFO_LIMIT'

    return None


# 32FC1 깊이 메시지 → (H, W) float32 배열(경계 검사 통과 뒤 복사본 하나만 만든다)
def depth_array(message):
    dtype = ('>' if message.is_bigendian else '<') + 'f4'
    view = np.frombuffer(memoryview(message.data), dtype=dtype).reshape(message.height, message.width)

    return view.astype(np.float32, copy=True)


# 부품 내용 지문(raw 바이트를 memoryview로 참조, 추가 대형 복사 없음) + 형식 메타
def part_digest(part, message):
    digest = hashlib.sha256()

    if part == 'camera_info':
        meta = (message.width, message.height, message.distortion_model, [float(v) for v in message.d],
                [float(v) for v in message.k], [float(v) for v in message.p])
        digest.update(json.dumps(meta).encode())
    else:
        meta = (message.encoding, message.width, message.height, message.step, bool(message.is_bigendian))
        digest.update(json.dumps(meta).encode())
        digest.update(memoryview(message.data))

    return digest.hexdigest()[:16]


# TransformStamped의 회전·이동 → 4×4 행렬(정규화 안 함: 비강체면 C2 rigid 검사가 거부)
def transform_matrix(transform):
    q, t = transform.rotation, transform.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    matrix = np.eye(4)
    matrix[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    matrix[:3, 3] = [t.x, t.y, t.z]

    return matrix


class BundleAssembler:
    # 같은 stamp_ns의 컬러·깊이·CameraInfo를 모은다. 도착 순서는 상관없고, 대기는 감독 시계로 제한한다.
    # outcomes: stamp_ns별 결과(사유 문자열)와 부품 지문. 유한 이력이며 모든 변경은 store()로만 한다.
    def __init__(self, part_capacity, wait_s, history):
        self.part_capacity = part_capacity
        self.wait_s = wait_s
        self.history = history
        self.pending = OrderedDict()     # stamp_ns → {'parts': {part: message}, 'digests': {}, 'first': 감독 시각}
        self.outcomes = OrderedDict()    # stamp_ns → (결과, 지문 사전)
        self.forgotten = None            # 이력에서 밀려난 가장 큰 stamp_ns(그 이하 늦은 부품은 판단 불가)
        self.counts = Counter()
        self.high_water = {'pending': 0, 'outcomes': 0}

    # 결과 이력 단일 변경 경로(넣고 바로 상한까지 자른다)
    def store(self, stamp, outcome, digests=None):
        previous = self.outcomes.pop(stamp, None)

        if digests is None and previous is not None:
            digests = previous[1]

        self.outcomes[stamp] = (outcome, digests or {})

        while len(self.outcomes) > self.history:
            old, _ = self.outcomes.popitem(last=False)
            self.forgotten = old if self.forgotten is None else max(self.forgotten, old)

        self.high_water['outcomes'] = max(self.high_water['outcomes'], len(self.outcomes))

    # stamp 결과를 확정한다(미완성 목록에서 빼고 이력에 남긴다)
    def settle(self, stamp, outcome):
        entry = self.pending.pop(stamp, None)
        self.store(stamp, outcome, entry['digests'] if entry else None)

        if outcome not in USED_OUTCOMES:
            self.counts['stamp_' + code_of(outcome)] += 1

    # 결과 조회(없으면 None)
    def outcome(self, stamp):
        item = self.outcomes.get(stamp)

        return item[0] if item else None

    # 늦은 부품 분류(이미 결과가 있거나 이력에서 밀려난 stamp). 분류 코드 또는 None(새 stamp)
    def late_kind(self, part, stamp, digest):
        item = self.outcomes.get(stamp)

        if item is None:
            if self.forgotten is not None and stamp <= self.forgotten:
                return 'late_history_evicted'

            return None

        outcome, digests = item

        if outcome not in USED_OUTCOMES:
            return 'late_after_rejected'

        known = digests.get(part)

        if known is None:
            return 'late_history_evicted'

        return 'late_identical' if known == digest else 'late_conflicting'

    # 부품 하나를 넣는다. (완성 부품 사전 또는 None, 늦은 부품 분류 코드 또는 None)
    def add(self, part, stamp, message, digest, wall):
        late = self.late_kind(part, stamp, digest)

        if late:
            self.counts[late] += 1
            return None, late

        entry = self.pending.get(stamp)

        if entry is None:
            # 넣기 전에 상한을 맞춘다(가장 오래된 미완성 stamp 거부).
            while len(self.pending) >= self.part_capacity:
                oldest = next(iter(self.pending))
                self.settle(oldest, 'ASSEMBLY_OVERFLOW:' + self.missing(oldest))

            entry = self.pending[stamp] = {'parts': {}, 'digests': {}, 'first': wall}
            self.high_water['pending'] = max(self.high_water['pending'], len(self.pending))

        if part in entry['parts']:
            # 조립 전 같은 부품 중복: 어느 쪽이 맞는지 알 수 없어 stamp 전체를 거부한다.
            kind = 'DUPLICATE_PART' if entry['digests'][part] == digest else 'CONFLICTING_PART'
            self.settle(stamp, f'{kind}:{part}')
            return None, None

        entry['parts'][part] = message
        entry['digests'][part] = digest

        if len(entry['parts']) < len(PARTS):
            return None, None

        return entry['parts'], None

    # 아직 오지 않은 부품 이름(+로 연결)
    def missing(self, stamp):
        entry = self.pending.get(stamp)
        have = entry['parts'] if entry else {}

        return '+'.join(part for part in PARTS if part not in have) or 'none'

    # 대기 상한을 넘은 미완성 stamp를 거부한다. 거부한 stamp 목록
    def expire(self, wall):
        expired = [stamp for stamp, entry in self.pending.items() if wall - entry['first'] > self.wait_s]

        for stamp in expired:
            self.settle(stamp, 'ASSEMBLY_TIMEOUT:' + self.missing(stamp))

        return expired

    # 시계 역행 등으로 모두 비운다
    def reset(self):
        self.pending.clear()
        self.outcomes.clear()
        self.forgotten = None

    # 미완성 부품 메모리(byte)
    def pending_bytes(self):
        return int(sum(len(getattr(message, 'data', b'')) for entry in self.pending.values()
                       for message in entry['parts'].values()))


class C2SurfaceNode(Node):
    # C2 입력 노드. 단일 스레드 실행기에서 콜백이 직렬로 실행된다고 가정한다(linker가 스레드 안전하지 않음).
    def __init__(self, **kwargs):
        super().__init__('c2_surface', **kwargs)
        self.declare_inputs()
        values = {name: self.get_parameter(name).value for name in self.parameter_names}
        self.cfg = parse_config(values, self.get_parameter('use_sim_time').value)
        cfg = self.cfg

        # 이전 기록·다른 파일을 덮어쓰지 않는다(이미 있으면 시작 거부). 쓰기 오류를 바로 알도록 버퍼 없이 쓴다.
        self.record_file = open(cfg.record_path, 'xb', buffering=0)
        self.record_bytes = 0
        self.record_state = 'OK'
        self.exit_code = 0
        self.linker = link.DetectionDepthLinker(link.LinkSettings(
            camera_frame=cfg.camera_frame, source=cfg.source, capacity=cfg.bundle_capacity,
            max_age_s=cfg.max_age_s, future_tolerance_s=cfg.future_tolerance_s,
            max_detections=cfg.max_detections_per_array, max_hypotheses=cfg.max_hypotheses_per_detection,
            max_text_len=cfg.max_text_len))
        self.future_ns = link.seconds_to_ns(cfg.future_tolerance_s)
        self.max_age_ns = link.seconds_to_ns(cfg.max_age_s)
        self.assembler = BundleAssembler(cfg.part_capacity, cfg.assembly_wait_s, cfg.outcome_history)
        self.pending_detections = deque()   # (DetectionArray, 수신 감독 시각)
        self.pending_tf = deque()           # (기록 사전, 관측, 수신 감독 시각)
        self.counts = Counter()
        self.diagnostic_rows = Counter()
        self.high_water = {'pending_detections': 0, 'pending_tf': 0}
        self.last_now = None
        self.closed = False
        self.tf_buffer = None

        if cfg.target_frame:
            from tf2_ros import Buffer, TransformListener

            # 관측 stamp_ns의 TF만 조회한다. 최신 TF로 대체하지 않는다. 캐시는 시간 상한만 있다.
            self.tf_buffer = Buffer(cache_time=Duration(nanoseconds=link.seconds_to_ns(cfg.tf_cache_s)), node=self)
            self.tf_listener = TransformListener(self.tf_buffer, self)

        for part, kind in (('image', Image), ('depth_image', Image), ('camera_info', CameraInfo)):
            self.create_subscription(kind, f'sensors/{cfg.camera}/processed/{part}',
                                     lambda message, part=part: self.on_part(part, message), SENSOR)

        self.create_subscription(Detection2DArray, cfg.detection_topic, self.on_detections, SENSOR)
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(cfg.poll_period_s, self.poll, clock=steady)
        self.create_timer(cfg.state_period_s, self.write_state, clock=steady)
        self.write('start', config={key: value for key, value in vars(cfg).items() if key != 'estimation'},
                   estimation=vars(cfg.estimation),
                   topics={part: self.resolve_topic_name(f'sensors/{cfg.camera}/processed/{part}') for part in PARTS},
                   detection_topic=self.resolve_topic_name(cfg.detection_topic),
                   use_sim_time=self.get_parameter('use_sim_time').value, pid=os.getpid(),
                   modules={'node': os.path.realpath(__file__), 'surface_estimation': os.path.realpath(se.__file__),
                            'c2_detection_link': os.path.realpath(link.__file__)})

    # 모든 설정을 미정 표식(빈 문자열·0·NaN) 기본으로 선언한다
    def declare_inputs(self):
        self.parameter_names = list(TEXT_KEYS) + list(INT_KEYS) + list(FLOAT_KEYS)

        for name in TEXT_KEYS:
            self.declare_parameter(name, '')

        for name in INT_KEYS:
            self.declare_parameter(name, 0)

        for name in FLOAT_KEYS:
            self.declare_parameter(name, math.nan)

        for name in se.EstimationSettings.__dataclass_fields__:
            self.declare_parameter('estimation.' + name, 0 if name in ESTIMATION_INT else math.nan)
            self.parameter_names.append('estimation.' + name)

    # ROS 시각(정수 ns). 시험에서 바꿔 끼울 수 있게 메서드로 둔다
    def now_ns(self):
        return self.get_clock().now().nanoseconds

    # 감독 경과 시각(초)
    def wall_s(self):
        return boottime()

    # 기록 한 줄. 바이트 상한·쓰기 오류면 상태를 바꾸고 RecordStop(같은 실패를 반복 기록하지 않음)
    def write(self, event, **fields):
        if self.record_state != 'OK':
            return

        row = {'event': event, 'wall_boottime_s': self.wall_s(), 'ros_now_ns': self.now_ns()}
        row.update(fields)
        data = (json.dumps(clean(row), ensure_ascii=False, separators=(',', ':')) + '\n').encode()

        if self.record_bytes + len(data) > self.cfg.max_record_bytes - MARKER_RESERVE:
            self.record_state = 'LIMIT_REACHED'
            marker = {'event': 'record_limit_reached', 'record_bytes': self.record_bytes,
                      'max_record_bytes': self.cfg.max_record_bytes, 'dropped_event': code_of(event),
                      'note': '기록 상한 도달: 이후 결과 미기록. 정상 완료 아님'}
            self.raw_write((json.dumps(marker, ensure_ascii=False) + '\n').encode()[:MARKER_RESERVE])
            raise RecordStop(EXIT_RECORD_LIMIT, 'RECORD_LIMIT_REACHED')

        self.raw_write(data)

    # 파일에 바로 쓴다. 실패하면 FAILED 상태로 두고 RecordStop
    def raw_write(self, data):
        try:
            written = self.record_file.write(data)

            if written != len(data):
                raise OSError('short write')
        except OSError as error:
            self.record_state = 'FAILED'
            raise RecordStop(EXIT_RECORD_FAILED, f'RECORD_WRITE_FAILED: {type(error).__name__} {error.errno}')

        self.record_bytes += len(data)

    # 사유별 카운터 + 사유별 제한된 상세 진단 행
    def note(self, code, **fields):
        code = code_of(code)
        self.counts[code] += 1

        if self.diagnostic_rows[code] < self.cfg.diagnostic_rows_per_reason:
            self.diagnostic_rows[code] += 1
            self.write('diagnostic', code=code, **fields)
        else:
            self.counts['diagnostic_suppressed'] += 1

    # 큐 실행 중 최대치 갱신
    def track(self):
        self.high_water['pending_detections'] = max(self.high_water['pending_detections'],
                                                    len(self.pending_detections))
        self.high_water['pending_tf'] = max(self.high_water['pending_tf'], len(self.pending_tf))

    # 시각 확인: 시작 전이면 False. 역행이면 모든 버퍼를 비우고 기록한다
    def tick(self):
        now = self.now_ns()

        if now <= 0:
            return False, now

        if self.last_now is not None and now < self.last_now:
            dropped = {'pending_parts': len(self.assembler.pending), 'buffered': len(self.linker.bundles),
                       'pending_detections': len(self.pending_detections), 'pending_tf': len(self.pending_tf)}
            previous = self.last_now
            waiting, queued = list(self.pending_detections), list(self.pending_tf)
            self.pending_detections.clear()
            self.pending_tf.clear()
            self.assembler.reset()
            self.linker.observe_clock(now)

            if self.tf_buffer is not None:
                self.tf_buffer.clear()

            self.counts['clock_reset'] += 1
            self.last_now = now
            self.write('clock_reset', previous_ns=previous, now_ns=now, dropped=dropped)

            # 대기 중이던 검출·TF 결과는 사라지지 않게 사유와 함께 기록한다.
            for array, _ in waiting:
                self.reject_array(array.stamp_ns, array.frame_id, len(array.detections), ['CLOCK_RESET'])

            for row, observation, _ in queued:
                self.finish_tf(row, observation, None, 'CLOCK_RESET')

        self.last_now = now
        self.linker.observe_clock(now)

        return True, now

    # 부품 공통 신원 검사(frame → stamp → 미래 → 만료). (stamp_ns 또는 None, 문제 코드 또는 None)
    def part_identity(self, message, now):
        if message.header.frame_id != self.cfg.camera_frame:
            return None, 'FRAME_MISMATCH'

        try:
            stamp = se.stamp_ns_from_header(message.header.stamp.sec, message.header.stamp.nanosec)
        except ValueError:
            return None, 'STAMP_INVALID'

        if stamp <= 0:
            return None, 'STAMP_INVALID'

        if stamp - now > self.future_ns:
            return stamp, 'FUTURE'

        if now - stamp > self.max_age_ns:
            return stamp, 'STALE'

        return stamp, None

    # 부품 수신
    def on_part(self, part, message):
        started, now = self.tick()
        self.counts['received_' + part] += 1

        if not started:
            self.note('CLOCK_NOT_STARTED', part=part)
            return

        stamp, problem = self.part_identity(message, now)

        if problem:
            # 다른 frame·무효·미래·만료 부품은 정상 stamp의 조립 상태를 바꾸지 않고 버린다.
            self.note(f'DROP_{part}_{problem}', stamp_ns=stamp)
            self.process_pending()
            return

        problem = (camera_info_problem(message, self.cfg) if part == 'camera_info'
                   else image_problem(message, part == 'depth_image', self.cfg))

        if problem is None and part == 'camera_info':
            problem = self.camera_problem(message, stamp)

        if problem:
            # 같은 카메라·같은 stamp의 형식 문제: 다른 부품과 묶이지 않도록 그 stamp를 거부한다.
            self.note(f'DROP_{part}_{problem}', stamp_ns=stamp)

            if self.assembler.outcome(stamp) is None:
                self.assembler.settle(stamp, f'PART_INVALID:{part}:{problem}')

            self.process_pending()
            return

        parts, late = self.assembler.add(part, stamp, message, part_digest(part, message), self.wall_s())

        if late:
            self.note(late.upper(), part=part, stamp_ns=stamp)
        elif parts is not None:
            self.add_bundle(stamp, parts, now)

        self.process_pending()

    # CameraInfo 내부 파라미터 문제(정상이면 None)
    def camera_problem(self, message, stamp):
        try:
            se.CameraIntrinsics.from_camera_info(message.width, message.height, message.k, message.p, message.d,
                                                 message.header.frame_id, stamp, se.HEADER, self.cfg.source)
        except ValueError as error:
            return code_of(error)

        return None

    # 완성된 부품으로 RgbdBundle을 만들어 연결기에 넣는다
    def add_bundle(self, stamp, parts, now):
        cfg = self.cfg
        color, depth_message, info = parts['image'], parts['depth_image'], parts['camera_info']
        intrinsics = se.CameraIntrinsics.from_camera_info(info.width, info.height, info.k, info.p, info.d,
                                                          info.header.frame_id, stamp, se.HEADER, cfg.source)
        depth = se.DepthFrame(depth_array(depth_message), '32FC1', depth_message.header.frame_id, stamp, se.HEADER,
                              cfg.source)
        bundle = link.RgbdBundle(stamp, color.header.frame_id, cfg.source, (color.width, color.height),
                                 depth, intrinsics)
        before = set(self.linker.bundles)
        accepted, reasons = self.linker.add_bundle(bundle, now)

        if not accepted:
            self.assembler.settle(stamp, 'BUNDLE_REJECTED:' + '+'.join(reasons))
            return

        self.assembler.settle(stamp, 'ASSEMBLED')
        self.counts['bundles_buffered'] += 1

        # 용량 초과로 밀려난 묶음은 그 stamp 검출이 오면 바로 사유를 알 수 있게 이력에 남긴다.
        for old in before - set(self.linker.bundles):
            self.assembler.store(old, 'EVICTED_BEFORE_DETECTION')
            self.counts['bundles_evicted'] += 1

    # 검출 배열 수신
    def on_detections(self, message):
        started, now = self.tick()
        self.counts['received_detection_arrays'] += 1
        count = len(message.detections)

        if not started:
            self.reject_array(None, None, count, ['CLOCK_NOT_STARTED'])
            return

        # 신원·순서 검사 뒤 크기 상한, 통과하면 순수 값으로 바꾼다(연결기 admit).
        array, reasons = self.linker.admit(message, now)

        if reasons:
            stamp, _ = link.header_stamp(message.header)
            frame = message.header.frame_id if message.header.frame_id == self.cfg.camera_frame else None
            self.reject_array(stamp, frame, count, reasons)
            return

        self.counts['received_detections'] += count

        if not self.try_consume(array, self.wall_s(), now):
            # 넣기 전에 상한을 맞춘다(가장 오래된 대기 검출 거부).
            while len(self.pending_detections) >= self.cfg.detection_capacity:
                old, _ = self.pending_detections.popleft()
                self.reject_array(old.stamp_ns, old.frame_id, len(old.detections), ['DETECTION_QUEUE_OVERFLOW'])

            self.pending_detections.append((array, self.wall_s()))
            self.track()

    # 검출 배열 하나 연결 시도. 끝났으면(연결·거부) True, 묶음 대기가 필요하면 False
    def try_consume(self, array, received_wall, now):
        start = time.perf_counter()
        reasons, results = self.linker.consume(array, now, self.cfg.estimation, self.cfg.robot_id)
        compute_s = time.perf_counter() - start

        if reasons == ['NO_MATCHING_FRAME']:
            outcome = self.assembler.outcome(array.stamp_ns)

            # 그 stamp 묶음이 이미 거부·만료·밀려났으면 기다리지 않고 그 사유로 거부한다.
            if outcome is not None and outcome != 'ASSEMBLED':
                self.reject_array(array.stamp_ns, array.frame_id, len(array.detections), ['FRAME_' + outcome])
                return True

            return False

        if reasons:
            self.reject_array(array.stamp_ns, array.frame_id, len(array.detections), reasons)
            return True

        # 정확 stamp 묶음을 소비했다. 그 묶음과 이전 묶음은 연결기가 버렸다.
        for stamp, (outcome, _) in list(self.assembler.outcomes.items()):
            if stamp <= array.stamp_ns and outcome == 'ASSEMBLED':
                self.assembler.store(stamp, 'RELEASED')

        self.counts['arrays_linked'] += 1

        if not results:
            self.counts['arrays_empty'] += 1
            self.write('detection_array_empty', stamp_ns=array.stamp_ns, stamp_s=array.stamp_s,
                       frame_id=array.frame_id, wait_wall_s=self.wall_s() - received_wall, compute_s=compute_s)
            return True

        rows = []

        for result in results:
            self.counts['results_' + result.status] += 1
            rows.append((self.result_row(array, result, received_wall, compute_s), result.observation))

        if self.tf_buffer is None:
            for row, _ in rows:
                row['transform'] = {'status': 'DISABLED'}
                self.emit_result(row)

            return True

        waiting = [(row, observation) for row, observation in rows if observation is not None and observation.valid]

        # 배열 단위 원자 처리: 결과 전부가 TF 대기에 들어갈 수 없으면 하나도 넣지 않는다.
        full = len(self.pending_tf) + len(waiting) > self.cfg.tf_pending_capacity

        for row, observation in rows:
            if observation is None or not observation.valid:
                row['transform'] = {'status': 'NOT_ATTEMPTED', 'reason': 'OBSERVATION_NOT_VALID'}
                self.emit_result(row)
            elif full:
                self.finish_tf(row, observation, None, 'TF_QUEUE_FULL')
            else:
                self.pending_tf.append((row, observation, self.wall_s()))

        self.track()
        self.process_tf()

        return True

    # 검출 결과 기록 사전(원본 배열 전체는 남기지 않는다)
    def result_row(self, array, result, received_wall, compute_s):
        observation = result.observation
        row = {'stamp_ns': array.stamp_ns, 'stamp_s': array.stamp_s, 'frame_id': array.frame_id,
               'source': self.cfg.source, 'robot_id': self.cfg.robot_id, 'n_detections': len(array.detections),
               'detection_index': result.detection_index, 'class_id': result.class_id, 'score': result.score,
               'link_status': result.status, 'reasons': list(result.reasons),
               'received_wall_s': received_wall, 'array_compute_s': compute_s}

        if observation is not None:
            row['plane_fit'] = self.observation_fields(observation)

        return row

    # 관측 표면 필드(plane_fit 범위)
    def observation_fields(self, observation):
        return {'status': observation.status, 'reasons': list(observation.reasons),
                'validity_scope': observation.validity_scope, 'observation_id': observation.observation_id,
                'stamp_ns': observation.stamp_ns, 'stamp_origin': observation.stamp_origin,
                'frame_id': observation.frame_id, 'observed_frame_id': observation.observed_frame_id,
                'roi_origin': observation.roi_origin, 'normal_convention': observation.normal_convention,
                'center': observation.center, 'normal': observation.normal, 'tangent': observation.tangent,
                'width_m': observation.width_m, 'depth_m': observation.depth_m,
                'axis_ambiguous': observation.axis_ambiguous,
                'quality': {key: observation.quality.get(key) for key in QUALITY_KEYS if key in observation.quality}}

    # 결과 한 줄 기록
    def emit_result(self, row):
        row['wait_wall_s'] = self.wall_s() - row.pop('received_wall_s')
        self.write('detection_result', **row)

    # 배열 단위 거부 기록(배열마다 1행. 다른 카메라 frame 문자열은 남기지 않는다)
    def reject_array(self, stamp_ns, frame_id, count, reasons):
        self.counts['arrays_rejected'] += 1

        for reason in reasons:
            self.counts['array_' + code_of(reason)] += 1

        self.write('detection_array_rejected', stamp_ns=stamp_ns,
                   stamp_s=None if stamp_ns is None else stamp_ns / se.NS_PER_S,
                   frame_id=frame_id, n_detections=count, reasons=list(reasons))

    # 대기 중 검출 재시도(오래된 stamp부터)
    def process_pending(self):
        if not self.pending_detections:
            return

        started, now = self.tick()

        if not started:
            return

        waiting = deque()

        for array, received in sorted(self.pending_detections, key=lambda item: item[0].stamp_ns):
            if not self.try_consume(array, received, now):
                waiting.append((array, received))

        self.pending_detections = waiting

    # TF 대기 결과 처리(관측 stamp_ns TF만, 응답 header ns도 정수로 대조)
    def process_tf(self):
        if self.tf_buffer is None or not self.pending_tf:
            return

        from tf2_ros import TransformException

        waiting = deque()

        for row, observation, queued in self.pending_tf:
            when = Time(nanoseconds=observation.stamp_ns, clock_type=self.get_clock().clock_type)

            if self.tf_buffer.can_transform(self.cfg.target_frame, observation.frame_id, when):
                try:
                    transform = self.tf_buffer.lookup_transform(self.cfg.target_frame, observation.frame_id, when)
                except TransformException as error:
                    self.finish_tf(row, observation, None,
                                   'TF_LOOKUP_FAILED: ' + str(error)[:self.cfg.max_text_len])
                    continue

                self.finish_tf(row, observation, transform, None)
            elif self.wall_s() - queued > self.cfg.tf_wait_s:
                self.finish_tf(row, observation, None, 'TF_UNAVAILABLE_AT_STAMP')
            else:
                waiting.append((row, observation, queued))

        self.pending_tf = waiting

    # 변환 결과를 기록에 붙인다
    def finish_tf(self, row, observation, transform, failure):
        if failure is None:
            try:
                transform_ns = se.stamp_ns_from_header(transform.header.stamp.sec, transform.header.stamp.nanosec)
            except ValueError:
                transform_ns = None

            if transform_ns != observation.stamp_ns:
                failure = 'TF_STAMP_MISMATCH'

        if failure is not None:
            self.counts['tf_' + code_of(failure)] += 1
            row['transform'] = {'status': 'FAILED', 'reason': failure, 'target_frame': self.cfg.target_frame}
            self.emit_result(row)
            return

        moved = se.transform_observation(observation, transform_matrix(transform.transform), self.cfg.target_frame,
                                         observation.frame_id, transform_ns)
        status = 'OK' if moved.valid else 'REJECTED'
        self.counts['tf_' + status] += 1
        row['transform'] = {'status': status, 'target_frame': self.cfg.target_frame,
                            'transform_stamp_ns': transform_ns,
                            'observation': self.observation_fields(moved), 'lineage': list(moved.transforms)}
        self.emit_result(row)

    # 주기 처리: 조립 만료·검출 대기 만료·TF 대기
    def poll(self):
        started, now = self.tick()
        wall = self.wall_s()
        self.assembler.expire(wall)

        if started:
            self.process_pending()

        waiting = deque()

        for array, received in self.pending_detections:
            if wall - received > self.cfg.detection_wait_s:
                missing = (self.assembler.missing(array.stamp_ns) if array.stamp_ns in self.assembler.pending
                           else 'all')
                self.reject_array(array.stamp_ns, array.frame_id, len(array.detections),
                                  [f'NO_MATCHING_FRAME_TIMEOUT:missing={missing}'])
            else:
                waiting.append((array, received))

        self.pending_detections = waiting

        if started:
            self.process_tf()

    # 버퍼·드롭·상한 상태
    def state(self):
        cfg = self.cfg
        queues = {
            'pending_parts': {'size': len(self.assembler.pending), 'capacity': cfg.part_capacity,
                              'high_water': self.assembler.high_water['pending'],
                              'bytes': self.assembler.pending_bytes()},
            'bundles': {'size': len(self.linker.bundles), 'capacity': cfg.bundle_capacity,
                        'high_water': self.linker.high_water, 'evicted': self.linker.evicted},
            'pending_detections': {'size': len(self.pending_detections), 'capacity': cfg.detection_capacity,
                                   'high_water': self.high_water['pending_detections']},
            'pending_tf': {'size': len(self.pending_tf), 'capacity': cfg.tf_pending_capacity,
                           'high_water': self.high_water['pending_tf']},
            'outcomes': {'size': len(self.assembler.outcomes), 'capacity': cfg.outcome_history,
                         'high_water': self.assembler.high_water['outcomes']},
        }

        return {'counts': dict(self.counts), 'assembler': dict(self.assembler.counts),
                'linker': self.linker.state(), 'queues': queues, 'record_bytes': self.record_bytes}

    # 주기 상태 기록
    def write_state(self):
        self.write('state', **self.state())

    # 종료: 대기 중 항목을 사유와 함께 남기고 기록을 닫는다(기록 실패·상한 상태면 더 쓰지 않는다)
    def close(self):
        if self.closed:
            return

        self.closed = True

        try:
            for array, _ in self.pending_detections:
                self.reject_array(array.stamp_ns, array.frame_id, len(array.detections), ['SHUTDOWN_PENDING'])

            for row, observation, _ in self.pending_tf:
                self.finish_tf(row, observation, None, 'SHUTDOWN_PENDING')

            self.pending_detections.clear()
            self.pending_tf.clear()
            self.write('shutdown', **self.state())
        except RecordStop as stop:
            self.exit_code = self.exit_code or stop.code

        if self.record_state == 'FAILED':
            self.exit_code = EXIT_RECORD_FAILED
        elif self.record_state == 'LIMIT_REACHED':
            self.exit_code = self.exit_code or EXIT_RECORD_LIMIT

        try:
            self.record_file.close()
        except OSError:
            self.exit_code = EXIT_RECORD_FAILED


def main():
    rclpy.init()

    try:
        node = C2SurfaceNode()
    except (ValueError, OSError, RecordStop) as error:
        # 설정 오류·기존 기록 파일·기록 폴더 문제·시작 기록 실패는 시작 거부로 명확히 알린다.
        print(f'c2_surface: 시작 거부: {error}', flush=True)

        if rclpy.ok():
            rclpy.shutdown()

        return error.code if isinstance(error, RecordStop) else EXIT_START

    code = 0

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RecordStop as stop:
        print(f'c2_surface: 기록 중단으로 종료: {stop.reason}', flush=True)
        code = stop.code
    finally:
        node.close()
        code = code or node.exit_code
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    return code


if __name__ == '__main__':
    raise SystemExit(main())
