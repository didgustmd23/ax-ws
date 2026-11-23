"""D2 그룹 작업의 입력 schema·계획 검증·내부 자료형.

ROS에 의존하지 않는다. 공개 enum 숫자는 ax_interfaces의 GroupTaskStatus·ResultStatus·
LoadState·ToolHoldState·ActuationState·ExecuteLocalTask 정의를 그대로 옮긴 값이며 새 값을
만들지 않는다. 내부 단계 상태·실행 억제 사유는 공개 enum이 아니다.
"""
import copy
import hashlib
import json
import re
from dataclasses import dataclass, field

from .execution_ids import DERIVE_RULE_VERSION, make_execution_id

# GroupTaskStatus 작업 상태(공개 값 그대로)
TASK_UNSPECIFIED = 0
TASK_QUEUED = 1
TASK_ASSIGNED = 2
TASK_RUNNING = 3
TASK_COMPLETED = 4
TASK_FAILED = 5
TASK_CANCELED = 6
TASK_RETRY_PENDING = 7  # W3는 자동 재시도를 하지 않아 이 상태를 만들지 않는다
TERMINAL_TASK_STATES = (TASK_COMPLETED, TASK_FAILED, TASK_CANCELED)

# GroupTaskStatus 인계 상태
TRANSFER_UNKNOWN = 0  # 잠금 요청 예약 뒤 LOCKED 확인 전에도 쓴다(두 로봇 주행 금지)
TRANSFER_NONE = 1
TRANSFER_LOCKED = 2
TRANSFER_IN_PROGRESS = 3
TRANSFER_UNLOCKED = 4
RELEASED_TRANSFER_STATES = (TRANSFER_NONE, TRANSFER_UNLOCKED)

# ResultStatus.outcome
OUTCOME_UNSPECIFIED = 0
OUTCOME_SUCCEEDED = 1
OUTCOME_FAILED = 2
OUTCOME_CANCELED = 3
OUTCOME_TIMED_OUT = 4
OUTCOME_UNCERTAIN = 5
OUTCOMES = (0, 1, 2, 3, 4, 5)

# ToolHoldState.state / LoadState.state / ActuationState.state
HOLD_UNKNOWN, HOLD_HELD, HOLD_RELEASED = 0, 1, 2
LOAD_UNKNOWN, LOAD_STOWED, LOAD_EMPTY, LOAD_LOST = 0, 1, 2, 3
ACTUATION_STOPPED = 4

# InventoryEntry.state(공개 값 그대로). 그룹 물체 상태
INV_UNKNOWN = 0
INV_AT_SOURCE = 1
INV_HELD_BY_MANIPULATOR = 2
INV_STOWED_ON_TRANSPORTER = 3
INV_AT_DESTINATION = 4
INV_LOST = 5

# ExecuteLocalTask.task_kind(D-10)
LOCAL_TASK_KINDS = {
    'LOCK_HANDOFF': 1,
    'UNLOCK_HANDOFF': 2,
    'PICK_AND_STOW': 3,
    'UNLOAD_AND_PLACE': 4,
    'CONFIRM_LOAD': 5,
    'RETRACT_ARM': 6,
}

# D-11 계획 허용 operation. W3 엔진은 통로 요청·대기를 아직 지원하지 않는다
PLAN_OPERATIONS = ('NAVIGATE_TO', 'REQUEST_CORRIDOR', 'WAIT') + tuple(LOCAL_TASK_KINDS)
W3_SUPPORTED_OPERATIONS = ('NAVIGATE_TO',) + tuple(LOCAL_TASK_KINDS)

MANIPULATOR = 'MANIPULATOR'
TRANSPORTER = 'TRANSPORTER'
ROLES = (MANIPULATOR, TRANSPORTER)

# 역할마다 맡을 수 있는 operation. 조작 역할만 집기·내려놓기·팔 후퇴, 운송 역할만 적재 확인
ROLE_OPERATIONS = {
    MANIPULATOR: {'NAVIGATE_TO', 'LOCK_HANDOFF', 'UNLOCK_HANDOFF',
                  'PICK_AND_STOW', 'UNLOAD_AND_PLACE', 'RETRACT_ARM'},
    TRANSPORTER: {'NAVIGATE_TO', 'LOCK_HANDOFF', 'UNLOCK_HANDOFF', 'CONFIRM_LOAD'},
}

SOURCE = 'SOURCE'
DESTINATION = 'DESTINATION'
PHASES = (SOURCE, DESTINATION)

# 입력 상한. 가변 입력이 메모리·원장을 무한히 키우지 않게 한다
MAX_ID_LENGTH = 128
MAX_PLAN_STEPS = 32
MAX_DEPENDS = 8
ID_PATTERN = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.:/-]{0,127}$')
ROBOT_ID_PATTERN = re.compile(r'^[a-z][a-z0-9_]{0,63}$')

# 개발 모드만 지원한다. 운영 모드는 선행 기능이 없어 생성 단계에서 거부한다
MODE_DEVELOPMENT_MOCK = 'DEVELOPMENT_MOCK'
MODE_OPERATIONAL = 'OPERATIONAL'
PROVENANCE_MOCK = 'MOCK'
PROVENANCE_LIVE = 'LIVE'


class SpecError(ValueError):
    """입력 schema·계획 검증 실패. code는 기계가 읽는 사유 코드다."""

    # 사유 코드와 사람이 읽는 설명을 함께 보관한다
    def __init__(self, code, detail=''):
        super().__init__(f'{code}: {detail}' if detail else code)
        self.code = code
        self.detail = detail


@dataclass(frozen=True)
class PlanStep:
    """고정 단계 1개. location은 SOURCE·DESTINATION 중 하나로 작업의 등록 지점을 가리킨다."""

    step_key: str
    operation: str
    robot_role: str
    location: str
    depends_on: tuple = ()
    terminal: bool = False


@dataclass(frozen=True)
class GroupTaskSpec:
    """D2가 수용하는 명시 작업·계획. 좌표·자동 선택값은 넣지 않는다."""

    team_id: str
    task_id: str
    object_id: str
    inventory_revision: int
    source_location_id: str
    source_location_revision: int
    destination_location_id: str
    destination_location_revision: int
    map_revision: int
    root_execution_id: str
    task_revision: int
    schedule_revision: int
    priority: int
    source_transfer_id: str
    destination_transfer_id: str
    input_source_id: str
    steps: tuple


@dataclass(frozen=True)
class Assignment:
    """명시 역할 배정 쌍. robot_id 값을 코드에 고정하지 않는다."""

    assignment_id: str
    assignment_generation: int
    manipulator_robot_id: str
    transporter_robot_id: str

    # 역할 이름으로 배정된 robot_id를 돌려준다
    def robot_for(self, role):
        return self.manipulator_robot_id if role == MANIPULATOR else self.transporter_robot_id

    # 배정된 두 robot_id
    def robots(self):
        return (self.manipulator_robot_id, self.transporter_robot_id)


# 표준 운반 계획 템플릿: 출발 정차 → 인계 잠금 → 적재·확인·후퇴 → 해제 → 운송 → 목적지 잠금 →
# 하역·후퇴·EMPTY 확인 → 해제. W3 엔진은 이 구조만 실행한다(다른 구조는 PLAN_SHAPE_UNSUPPORTED)
def standard_transfer_plan():
    def step(key, op, role, loc, deps=(), terminal=False):
        return PlanStep(key, op, role, loc, tuple(deps), terminal)

    src_nav = ('src_nav_manipulator', 'src_nav_transporter')
    src_lock = ('src_lock_manipulator', 'src_lock_transporter')
    src_unlock = ('src_unlock_manipulator', 'src_unlock_transporter')
    dst_nav = ('dst_nav_manipulator', 'dst_nav_transporter')
    dst_lock = ('dst_lock_manipulator', 'dst_lock_transporter')

    return (
        step(src_nav[0], 'NAVIGATE_TO', MANIPULATOR, SOURCE),
        step(src_nav[1], 'NAVIGATE_TO', TRANSPORTER, SOURCE),
        step(src_lock[0], 'LOCK_HANDOFF', MANIPULATOR, SOURCE, src_nav),
        step(src_lock[1], 'LOCK_HANDOFF', TRANSPORTER, SOURCE, src_nav),
        step('load_pick_and_stow', 'PICK_AND_STOW', MANIPULATOR, SOURCE, src_lock),
        step('load_confirm', 'CONFIRM_LOAD', TRANSPORTER, SOURCE, ('load_pick_and_stow',)),
        step('load_retract', 'RETRACT_ARM', MANIPULATOR, SOURCE, ('load_confirm',)),
        step(src_unlock[0], 'UNLOCK_HANDOFF', MANIPULATOR, SOURCE, ('load_retract',)),
        step(src_unlock[1], 'UNLOCK_HANDOFF', TRANSPORTER, SOURCE, ('load_retract',)),
        step(dst_nav[0], 'NAVIGATE_TO', MANIPULATOR, DESTINATION, src_unlock),
        step(dst_nav[1], 'NAVIGATE_TO', TRANSPORTER, DESTINATION, src_unlock),
        step(dst_lock[0], 'LOCK_HANDOFF', MANIPULATOR, DESTINATION, dst_nav),
        step(dst_lock[1], 'LOCK_HANDOFF', TRANSPORTER, DESTINATION, dst_nav),
        step('unload_place', 'UNLOAD_AND_PLACE', MANIPULATOR, DESTINATION, dst_lock),
        step('unload_retract', 'RETRACT_ARM', MANIPULATOR, DESTINATION, ('unload_place',)),
        step('unload_confirm_empty', 'CONFIRM_LOAD', TRANSPORTER, DESTINATION,
             ('unload_retract',)),
        step('dst_unlock_manipulator', 'UNLOCK_HANDOFF', MANIPULATOR, DESTINATION,
             ('unload_confirm_empty',), True),
        step('dst_unlock_transporter', 'UNLOCK_HANDOFF', TRANSPORTER, DESTINATION,
             ('unload_confirm_empty',), True),
    )


# 식별자 문자열 형식 검사
def _check_id(name, value, pattern=ID_PATTERN):
    if type(value) is not str or not pattern.match(value):
        raise SpecError('INVALID_ID', f'{name}={value!r}')


# 음이 아닌 uint64 정수 검사. bool은 정수로 받지 않는다
def _check_uint(name, value, minimum=0):
    if type(value) is not int or not minimum <= value < 2**64:
        raise SpecError('INVALID_NUMBER', f'{name}={value!r}')


# 계획 자체의 일반 검증: 단계 키·operation·역할·선행 단계·순환·종료 단계
def validate_plan(steps):
    if not isinstance(steps, tuple) or not steps:
        raise SpecError('PLAN_EMPTY')

    if len(steps) > MAX_PLAN_STEPS:
        raise SpecError('PLAN_TOO_LARGE', f'{len(steps)} > {MAX_PLAN_STEPS}')

    keys = [s.step_key for s in steps]
    if len(set(keys)) != len(keys):
        raise SpecError('DUPLICATE_STEP_KEY')

    for s in steps:
        _check_id('step_key', s.step_key)

        if s.operation not in PLAN_OPERATIONS:
            raise SpecError('OPERATION_NOT_ALLOWED', s.operation)

        if s.operation not in W3_SUPPORTED_OPERATIONS:
            raise SpecError('OPERATION_UNSUPPORTED', s.operation)

        if s.robot_role not in ROLES:
            raise SpecError('INVALID_ROLE', s.robot_role)

        if s.operation not in ROLE_OPERATIONS[s.robot_role]:
            raise SpecError('ROLE_OPERATION_MISMATCH', f'{s.robot_role}/{s.operation}')

        if s.location not in PHASES:
            raise SpecError('INVALID_LOCATION_REF', s.location)

        if len(s.depends_on) > MAX_DEPENDS or len(set(s.depends_on)) != len(s.depends_on):
            raise SpecError('INVALID_DEPENDS', s.step_key)

        for dep in s.depends_on:
            if dep not in keys:
                raise SpecError('UNKNOWN_DEPENDENCY', f'{s.step_key}->{dep}')

    # 선행 관계 순환 검사(위상 정렬)
    remaining = {s.step_key: set(s.depends_on) for s in steps}
    while remaining:
        ready = [k for k, deps in remaining.items() if not deps]
        if not ready:
            raise SpecError('PLAN_CYCLE', ','.join(sorted(remaining)))

        for key in ready:
            del remaining[key]

        for deps in remaining.values():
            deps.difference_update(ready)

    if not any(s.terminal for s in steps):
        raise SpecError('NO_TERMINAL_STEP')

    # W3 엔진이 단계별 근거 조건을 아는 구조는 표준 템플릿뿐이다
    if steps != standard_transfer_plan():
        raise SpecError('PLAN_SHAPE_UNSUPPORTED')


SPEC_KEYS = (
    'team_id', 'task_id', 'object_id', 'inventory_revision',
    'source_location_id', 'source_location_revision',
    'destination_location_id', 'destination_location_revision',
    'map_revision', 'root_execution_id', 'task_revision', 'schedule_revision',
    'priority', 'source_transfer_id', 'destination_transfer_id',
    'input_source_id', 'steps',
)
STEP_KEYS = ('step_key', 'operation', 'robot_role', 'location', 'depends_on', 'terminal')
ASSIGNMENT_KEYS = ('assignment_id', 'assignment_generation',
                   'manipulator_robot_id', 'transporter_robot_id')


# 사전 입력의 키 집합이 정확히 맞는지 확인한다(빠짐·추가 모두 거부)
def _exact_keys(name, data, keys):
    if not isinstance(data, dict):
        raise SpecError('INVALID_SCHEMA', f'{name}는 사전이어야 한다')

    missing = sorted(set(keys) - set(data))
    extra = sorted(set(data) - set(keys))
    if missing or extra:
        raise SpecError('INVALID_SCHEMA', f'{name} 빠짐={missing} 추가={extra}')


# 단계 사전 하나를 PlanStep으로 바꾼다
def parse_step(data):
    _exact_keys('step', data, STEP_KEYS)

    deps = data['depends_on']
    if not isinstance(deps, list) or not all(type(d) is str for d in deps):
        raise SpecError('INVALID_SCHEMA', 'depends_on은 문자열 목록이어야 한다')

    if type(data['terminal']) is not bool:
        raise SpecError('INVALID_SCHEMA', 'terminal은 bool이어야 한다')

    for key in ('step_key', 'operation', 'robot_role', 'location'):
        if type(data[key]) is not str:
            raise SpecError('INVALID_SCHEMA', f'{key}는 문자열이어야 한다')

    return PlanStep(data['step_key'], data['operation'], data['robot_role'],
                    data['location'], tuple(deps), data['terminal'])


# 작업 사전 하나를 검증된 GroupTaskSpec으로 바꾼다
def parse_spec(data):
    _exact_keys('task', data, SPEC_KEYS)

    if not isinstance(data['steps'], list):
        raise SpecError('INVALID_SCHEMA', 'steps는 목록이어야 한다')

    values = dict(data)
    values['steps'] = tuple(parse_step(s) for s in data['steps'])
    spec = GroupTaskSpec(**values)
    validate_spec(spec)

    return spec


# 작업 정의 자체의 검증. 여러 작업 사이 충돌은 GroupRegistry가 검사한다
def validate_spec(spec):
    for name in ('team_id', 'task_id', 'object_id', 'source_location_id',
                 'destination_location_id', 'root_execution_id',
                 'source_transfer_id', 'destination_transfer_id', 'input_source_id'):
        _check_id(name, getattr(spec, name))

    for name in ('inventory_revision', 'source_location_revision',
                 'destination_location_revision', 'map_revision', 'schedule_revision'):
        _check_uint(name, getattr(spec, name))

    # task_revision 0은 "발행 전"과 구분할 수 없어 1부터 받는다
    _check_uint('task_revision', spec.task_revision, minimum=1)

    if type(spec.priority) is not int or not 1 <= spec.priority <= 9:
        raise SpecError('INVALID_PRIORITY', repr(spec.priority))

    if spec.source_location_id == spec.destination_location_id:
        raise SpecError('SAME_SOURCE_DESTINATION')

    if spec.source_transfer_id == spec.destination_transfer_id:
        raise SpecError('DUPLICATE_TRANSFER_ID')

    validate_plan(spec.steps)


# 배정 사전을 검증된 Assignment로 바꾼다
def parse_assignment(data):
    _exact_keys('assignment', data, ASSIGNMENT_KEYS)
    assignment = Assignment(**data)
    validate_assignment(assignment)

    return assignment


# 배정 쌍 검증: 두 역할 모두 명시, 같은 로봇 중복 금지, 세대 1 이상
def validate_assignment(assignment):
    _check_id('assignment_id', assignment.assignment_id)
    _check_uint('assignment_generation', assignment.assignment_generation, minimum=1)
    _check_id('manipulator_robot_id', assignment.manipulator_robot_id, ROBOT_ID_PATTERN)
    _check_id('transporter_robot_id', assignment.transporter_robot_id, ROBOT_ID_PATTERN)

    if assignment.manipulator_robot_id == assignment.transporter_robot_id:
        raise SpecError('SAME_ROBOT_FOR_BOTH_ROLES')


# 단계 정의를 key로 찾는다
def step_by_key(spec, step_key):
    return step_by_key_in(spec.steps, step_key)


# 단계 목록에서 key로 찾는다
def step_by_key_in(steps, step_key):
    for s in steps:
        if s.step_key == step_key:
            return s

    raise KeyError(step_key)


# 단계가 속한 인계 ID(출발·목적지). 주행 단계는 인계와 무관해 빈 문자열
def transfer_id_for(spec, plan_step):
    if plan_step.operation == 'NAVIGATE_TO':
        return ''

    if plan_step.location == SOURCE:
        return spec.source_transfer_id

    return spec.destination_transfer_id


# 단계가 가리키는 등록 지점 ID·revision
def location_for(spec, plan_step):
    if plan_step.location == SOURCE:
        return spec.source_location_id, spec.source_location_revision

    return spec.destination_location_id, spec.destination_location_revision


# 불변 인자를 정렬 JSON으로 고정하고 그 digest를 만든다
def args_digest(args):
    encoded = json.dumps(args, sort_keys=True, separators=(',', ':'), ensure_ascii=False)

    return hashlib.sha256(encoded.encode('utf-8')).hexdigest()


# 한 단계 실행에 필요한 ExecutionRef 필드·불변 인자를 만든다(W3: attempt_index는 항상 0)
def build_execution(spec, assignment, plan_step, attempt_index=0):
    execution_id = make_execution_id(spec.root_execution_id, plan_step.step_key,
                                     plan_step.operation, attempt_index)
    robot_id = assignment.robot_for(plan_step.robot_role)
    peer_role = TRANSPORTER if plan_step.robot_role == MANIPULATOR else MANIPULATOR
    peer_robot_id = '' if plan_step.operation == 'NAVIGATE_TO' else assignment.robot_for(peer_role)
    location_id, location_revision = location_for(spec, plan_step)

    execution = {
        'team_id': spec.team_id,
        'task_id': spec.task_id,
        'task_revision': spec.task_revision,
        'assignment_id': assignment.assignment_id,
        'assignment_generation': assignment.assignment_generation,
        'transfer_id': transfer_id_for(spec, plan_step),
        'root_execution_id': spec.root_execution_id,
        'parent_execution_id': '',
        'execution_id': execution_id,
        'derive_rule_version': DERIVE_RULE_VERSION,
        'step_key': plan_step.step_key,
        'operation': plan_step.operation,
        'attempt_index': attempt_index,
    }
    args = {
        'robot_id': robot_id,
        'peer_robot_id': peer_robot_id,
        'object_id': spec.object_id,
        'inventory_revision': spec.inventory_revision,
        'location_id': location_id,
        'location_revision': location_revision,
        'source_location_id': spec.source_location_id,
        'source_location_revision': spec.source_location_revision,
        'destination_location_id': spec.destination_location_id,
        'destination_location_revision': spec.destination_location_revision,
        'map_revision': spec.map_revision,
        'task_kind': LOCAL_TASK_KINDS.get(plan_step.operation, 0),
    }

    return execution, args


@dataclass(frozen=True)
class ClockSnapshot:
    """엔진에 주입하는 시각. boottime은 이 프로세스의 CLOCK_BOOTTIME, sim은 ROS sim time."""

    boottime_s: float
    sim_time_s: float
    time_valid: bool = True


# 관측 종류별 만료(D-05). ARM_RETRACTED·PLACEMENT는 생산자·계약이 아직 없어 개발 가정값이다
OBSERVATION_EXPIRY_S = {
    'ACTUATION': 0.3,
    'HOLD': 0.5,
    'LOAD': 0.5,
    'ARM_RETRACTED': 0.5,
    'PLACEMENT': 3.0,
}


@dataclass(frozen=True)
class Observation:
    """상대 소유 상태의 관측. 사실값과 출처·세션·revision·시각을 함께 보존한다.

    ACTUATION의 value는 ActuationState.state(물리 정지 판단)만 쓴다. drive_allowed 같은 허가는
    permits에 따로 담고 D2 단계 근거로 쓰지 않는다.
    """

    kind: str
    robot_id: str
    object_id: str
    value: int
    source_id: str
    session_id: str
    revision: int
    valid: bool
    stamp_sim_s: float
    received_boottime_s: float
    provenance: str
    permits: tuple = ()


@dataclass
class StepRun:
    """한 단계의 실행 기록(내부 상태, 공개 enum 아님)."""

    step_key: str
    status: str = 'PENDING'
    execution_id: str = ''
    attempt_index: int = 0
    goal_uuid: str = ''
    sent_boottime_s: float = -1.0
    finished_boottime_s: float = -1.0
    outcome: int = -1
    error_code: str = ''
    object_hold_state: int = -1
    load_state: int = -1
    status_phase: str = ''
    transfer_id: str = ''
    assignment_generation: int = 0


@dataclass
class GroupState:
    """그룹 작업 하나의 D2 상태. apply_event는 복사본을 바꿔 다음 상태를 만든다."""

    spec: GroupTaskSpec
    mode: str = MODE_DEVELOPMENT_MOCK
    assignment: Assignment = None
    task_state: int = TASK_QUEUED
    transfer_state: int = TRANSFER_NONE
    transfer_id: str = ''
    steps: dict = field(default_factory=dict)
    inhibit: list = field(default_factory=list)
    hold_ids: list = field(default_factory=list)
    observations: dict = field(default_factory=dict)
    retired_sessions: dict = field(default_factory=dict)
    last_sim_time_s: float = -1.0
    cancel_requested: bool = False
    cancel_requested_boottime_s: float = -1.0
    failure_event_id: str = ''
    revision: int = 0
    # W5 그룹 물체 상태(재고 항목에 같은 트랜잭션으로 반영). 확정 근거 없이 승격하지 않는다
    object_state: int = INV_AT_SOURCE
    object_location_id: str = ''
    object_carrier_robot_id: str = ''
    object_assignment_generation: int = 0
    object_isolated: bool = False
    physical_started: bool = False

    # 깊은 복사본(입력 상태를 바꾸지 않는다)
    def copy(self):
        return copy.deepcopy(self)


# 실행 모드 검사. 운영 모드는 선행 기능(D1 실행 API·실 A3/C4·D3/D4 연결)이 없어 거부한다
def check_mode(mode):
    if mode == MODE_OPERATIONAL:
        raise SpecError('OPERATIONAL_MODE_UNSUPPORTED')

    if mode != MODE_DEVELOPMENT_MOCK:
        raise SpecError('UNKNOWN_MODE', repr(mode))


# 모드별로 받을 수 있는 근거 출처. 운영 모드는 MOCK을 실제 근거로 받지 않는다
def provenance_acceptable(mode, provenance):
    if mode == MODE_OPERATIONAL:
        return provenance == PROVENANCE_LIVE

    return mode == MODE_DEVELOPMENT_MOCK and provenance == PROVENANCE_MOCK


# 새 작업 상태를 만든다. 단계 기록은 전부 PENDING
def new_group_state(spec, mode=MODE_DEVELOPMENT_MOCK):
    check_mode(mode)
    validate_spec(spec)

    return GroupState(spec=spec, mode=mode,
                      steps={s.step_key: StepRun(s.step_key) for s in spec.steps},
                      object_location_id=spec.source_location_id)
