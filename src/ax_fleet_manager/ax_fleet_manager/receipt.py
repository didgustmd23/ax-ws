"""D2 W4 자동 입고 규칙(순수, ROS·원장 무의존).

- 입고 관측(ReceiptObservation 필드)을 검증하고, 원장에서 읽은 같은 물체의 입고 사건과 대조해
  신규 입고·반복 관측·식별 불명·재입고·구역 충돌을 판정한다. 기록과 작업 생성은 intake.py가 한다.
- 작업 등록 요청(RegisterTask 필드)의 형식·멱등 digest·목적지 규칙을 정한다.
- 임계값(식별 신뢰 0.8, 관측 나이 2.0 s)은 개발 가정값이며 운영 기준이 아니다.
"""
import hashlib
import json
import math
import uuid
from dataclasses import dataclass, field

from . import group_model as gm

# ReceiptEvent.state(공개 값 그대로)
RECEIPT_PENDING_IDENTIFICATION = 0
RECEIPT_TASK_CREATED = 1
RECEIPT_DUPLICATE = 2
RECEIPT_REJECTED = 3

# ReceiptObservation.source(공개 값 그대로)
SOURCE_UNKNOWN = 0
SOURCE_SENSOR = 1
SOURCE_GROUND_TRUTH = 2
PROVENANCE_BY_SOURCE = {SOURCE_SENSOR: 'SENSOR', SOURCE_GROUND_TRUTH: 'GROUND_TRUTH'}

# 내부 등록 요청 ID(uuid5)의 고정 namespace. 바꾸면 재시작 전후 같은 사건의 요청 ID가 달라진다
RECEIPT_REQUEST_NAMESPACE = uuid.UUID('5f96e85c-52b2-5215-8fd5-4232de1a4c72')

# 사유 코드(D2 내부). ROS 연결 시 계약 D-13 목록에 반영해야 한다
DUPLICATE_REQUEST = 'DUPLICATE_REQUEST'  # D-13
RERECEIPT_UNCONFIRMED = 'RERECEIPT_UNCONFIRMED'
NO_DESTINATION_RULE = 'NO_DESTINATION_RULE'
SOURCE_MISMATCH = 'SOURCE_MISMATCH'
ZONE_NOT_RECEIPT = 'ZONE_NOT_RECEIPT'
OBJECT_BUSY = 'OBJECT_BUSY'
RECEIPT_ZONE_CONFLICT = 'RECEIPT_ZONE_CONFLICT'

OBSERVATION_KEYS = ('observation_id', 'zone_id', 'object_id', 'object_class',
                    'identity_confidence', 'source', 'source_id', 'valid', 'stamp_sim_s')
REQUEST_KEYS = ('request_id', 'receipt_event_id', 'object_id', 'source_location_id',
                'destination_location_id')


@dataclass(frozen=True)
class ReceiptConfig:
    """입고 설정. 구역·목적지 표는 호출자가 명시하는 개발 자료이며 등록 지점 원장 값이 아니다."""

    receipt_zones: tuple
    destination_rules: dict = field(default_factory=dict)
    min_identity_confidence: float = 0.8
    max_observation_age_s: float = 2.0
    max_observation_ids: int = 64
    allow_ground_truth: bool = True
    team_id: str = 'team_dev'
    priority: int = 5


@dataclass(frozen=True)
class ReceiptObservation:
    """검증된 입고 관측. pose는 D 판정에 쓰지 않아 받지 않는다(집기 pose는 C가 계산)."""

    observation_id: str
    zone_id: str
    object_id: str
    object_class: str
    identity_confidence: float
    source: int
    source_id: str
    valid: bool
    stamp_sim_s: float


@dataclass(frozen=True)
class ObservationVerdict:
    """관측 판정. action: REJECT·PENDING·DUPLICATE·NEW·RERECEIPT."""

    action: str
    reason_code: str = ''
    receipt_event_id: str = ''
    receipt_round: int = 0


# 설정 검증: 구역·목적지 표가 등록 지점에 있어야 하고 임계값은 범위 안이어야 한다
def validate_config(config, locations):
    if not config.receipt_zones:
        raise gm.SpecError('INVALID_CONFIG', 'receipt_zones 비어 있음')

    for zone in config.receipt_zones:
        gm._check_id('receipt_zone', zone)
        if zone not in locations:
            raise gm.SpecError('UNKNOWN_LOCATION', zone)

    for zone, destination in config.destination_rules.items():
        if zone not in config.receipt_zones:
            raise gm.SpecError('INVALID_CONFIG', f'입고 구역이 아닌 규칙: {zone}')
        if destination not in locations or destination == zone:
            raise gm.SpecError('INVALID_CONFIG', f'목적지 규칙: {zone}->{destination}')

    if not 0.0 < config.min_identity_confidence <= 1.0:
        raise gm.SpecError('INVALID_CONFIG', 'min_identity_confidence')

    if not 0.0 < config.max_observation_age_s:
        raise gm.SpecError('INVALID_CONFIG', 'max_observation_age_s')

    gm._check_id('team_id', config.team_id)


# 정렬 JSON digest(멱등 대조용)
def digest(data):
    encoded = json.dumps(data, sort_keys=True, separators=(',', ':'), ensure_ascii=False)

    return hashlib.sha256(encoded.encode('utf-8')).hexdigest()


# 사전 입력의 키 집합이 정확히 맞는지 확인한다
def _exact(name, data, keys):
    if not isinstance(data, dict) or set(data) != set(keys):
        raise gm.SpecError('INVALID_SCHEMA', name)


# 빈 문자열을 허용하는 식별자 검사
def _check_optional_id(name, value):
    if value == '':
        return

    gm._check_id(name, value)


# 관측 사전을 검증된 ReceiptObservation으로 바꾼다(형식 오류는 SpecError)
def parse_observation(data):
    _exact('receipt_observation', data, OBSERVATION_KEYS)

    gm._check_id('observation_id', data['observation_id'])
    gm._check_id('zone_id', data['zone_id'])
    _check_optional_id('object_id', data['object_id'])
    gm._check_id('source_id', data['source_id'])

    if type(data['object_class']) is not str or len(data['object_class']) > gm.MAX_ID_LENGTH:
        raise gm.SpecError('INVALID_SCHEMA', 'object_class')

    confidence = data['identity_confidence']
    if type(confidence) not in (int, float) or not math.isfinite(confidence) \
            or not 0.0 <= confidence <= 1.0:
        raise gm.SpecError('INVALID_NUMBER', 'identity_confidence')

    stamp = data['stamp_sim_s']
    if type(stamp) not in (int, float) or not math.isfinite(stamp):
        raise gm.SpecError('INVALID_NUMBER', 'stamp_sim_s')

    if type(data['source']) is not int or type(data['valid']) is not bool:
        raise gm.SpecError('INVALID_SCHEMA', 'source/valid')

    return ReceiptObservation(**{k: data[k] for k in OBSERVATION_KEYS})


# 관측 내용 digest(같은 observation_id 재전송 대조)
def observation_digest(obs):
    return digest({k: getattr(obs, k) for k in OBSERVATION_KEYS})


# 관측 자체의 수용 판정(원장 무관): 유효성·출처·구역·시각. 통과하면 None
def observation_rejection(obs, now, config, mode=gm.MODE_DEVELOPMENT_MOCK):
    if not now.time_valid:
        return 'TIME_INVALID'

    if not obs.valid:
        return 'OBSERVATION_INVALID'

    if obs.source not in PROVENANCE_BY_SOURCE:
        return 'SOURCE_UNKNOWN'

    # 정답 기반 관측은 개발 모드에서만 받는다(운영 모드는 현재 생성 단계에서 거부됨)
    if obs.source == SOURCE_GROUND_TRUTH and (
            mode != gm.MODE_DEVELOPMENT_MOCK or not config.allow_ground_truth):
        return 'PROVENANCE_REJECTED'

    if obs.zone_id not in config.receipt_zones:
        return ZONE_NOT_RECEIPT

    age = now.sim_time_s - obs.stamp_sim_s
    if age < 0.0:
        return 'OBSERVATION_FROM_FUTURE'

    if age > config.max_observation_age_s:
        return 'OBSERVATION_STALE'

    return None


# 식별 완료 여부: object_id가 있고 신뢰도가 임계값 이상
def identified(obs, config):
    return obs.object_id != '' and obs.identity_confidence >= config.min_identity_confidence


# 입고 사건이 아직 열려 있는지. 작업이 진행 중인 TASK_CREATED, 작업 없이 남은 REJECTED(같은 물품의
# 반복 관측을 새 사건으로 만들지 않기 위해 열린 것으로 본다). 작업이 종료된 사건은 닫힌 것이다
def event_open(event, task_states):
    if event['state'] == RECEIPT_REJECTED:
        return True

    if event['state'] != RECEIPT_TASK_CREATED:
        return False

    state = task_states.get(event['task_id'])

    return state is None or state.task_state not in gm.TERMINAL_TASK_STATES


# 식별된 관측을 같은 물체의 기존 사건과 대조한다. events는 회차 순 목록
def classify_identified(obs, events, task_states):
    open_events = [e for e in events if event_open(e, task_states)]

    if open_events:
        current = open_events[-1]
        if current['zone_id'] != obs.zone_id:
            return ObservationVerdict('REJECT', RECEIPT_ZONE_CONFLICT,
                                      current['receipt_event_id'])

        return ObservationVerdict('DUPLICATE', '', current['receipt_event_id'],
                                  current['receipt_round'])

    next_round = 1 + max((e['receipt_round'] for e in events), default=0)

    # 이전 사건의 작업이 종료된 물품이 다시 보임: 재고(W5) 없이 실제 출고·재입고를 확인할 수 없다
    if events:
        return ObservationVerdict('RERECEIPT', RERECEIPT_UNCONFIRMED, '', next_round)

    return ObservationVerdict('NEW', '', '', next_round)


# 입고 사건의 내부 등록 요청 ID(재시작·재시도에도 같은 값)
def receipt_request_id(receipt_event_id):
    return f'req-{uuid.uuid5(RECEIPT_REQUEST_NAMESPACE, receipt_event_id)}'


# 등록 요청 사전 검증(형식 오류는 SpecError)
def parse_request(data):
    _exact('register_task', data, REQUEST_KEYS)

    gm._check_id('request_id', data['request_id'])
    for key in REQUEST_KEYS[1:]:
        if type(data[key]) is not str:
            raise gm.SpecError('INVALID_SCHEMA', key)
        _check_optional_id(key, data[key])

    if data['object_id'] == '':
        raise gm.SpecError('INVALID_ID', 'object_id 비어 있음')

    return dict(data)


# 출발·목적지 결정. 입고 사건 작업은 출발 = 입고 구역, 목적지가 비면 구역별 규칙 표
def resolve_locations(request, event, config):
    source = request['source_location_id']
    if event is not None and source != event['zone_id']:
        return None, None, SOURCE_MISMATCH

    if source == '':
        return None, None, SOURCE_MISMATCH

    destination = request['destination_location_id']
    if destination == '':
        destination = config.destination_rules.get(source, '')
        if destination == '':
            return None, None, NO_DESTINATION_RULE

    return source, destination, ''


# ReceiptEvent 메시지 필드(관제·후속 ROS 연결용)
def receipt_event_fields(event):
    return {
        'receipt_event_id': event['receipt_event_id'],
        'object_id': event['object_id'],
        'zone_id': event['zone_id'],
        'observation_ids': list(event['observation_ids']),
        'state': event['state'],
        'task_id': event['task_id'],
    }
