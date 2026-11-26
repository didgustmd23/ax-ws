"""D2 W5 live 연결 어댑터 초안(기본 비활성). 실제 운영 연결이 아니다.

- D1NavigationPort: D1 NavigationClient(같은 프로세스 Python 객체, submit·cancel)를 감싼다. 주행 goal
  UUID는 D1이 만들고(계약 ID 표 "Action 클라이언트"), D2와 D1의 결합 키는 execution_id(D-03)다.
  D2는 D1이 돌려준 UUID를 참고 기록만 한다. D1 공개 조회 API(execution_id → 상태)가 없어 결과는
  호출자가 D1 원장 행을 넘길 때만 변환한다(없으면 D2 쪽 결과 대기 시간초과로 미확정).
- 등록 지점 → 정차 자세는 개발 카탈로그(stop_poses)에서 고른다. 지점 원장 소유·pose 원천은 미정이다.
- actuation_observation: A4 ActuationState 필드 → D2 관측. 판단에는 state(STOPPED)만 쓰고 허가 필드는
  permits에 따로 둔다.
- hold_observation: C4 ToolHoldState 필드 → D2 관측(kind='HOLD'). 형식·모의 근거 거절만 하고
  valid·만료·물체·앵커 판정은 group_machine이 한다. 구독·노드는 만들지 않는다.
- 기본 enabled=False다. live 출처(LIVE)는 개발 모드 판정 근거로 받지 않으므로(provenance_acceptable)
  runtime 연결 전에 출처 정책 결정이 필요하다. src/ax_navigation 파일은 수정하지 않는다.
"""
from . import group_model as gm
from .group_events import StepResult
from .ports import PortError, PortUnsupported

# D1 원장 상태 → ResultStatus.outcome. ARRIVED는 Nav2 도착일 뿐이며 정지 근거는 A4 관측으로 따로 본다
D1_TERMINAL_OUTCOMES = {
    'ARRIVED': gm.OUTCOME_SUCCEEDED,
    'CANCELED': gm.OUTCOME_CANCELED,
    'ABORTED': gm.OUTCOME_FAILED,
    'REJECTED': gm.OUTCOME_FAILED,
    'UNKNOWN': gm.OUTCOME_UNCERTAIN,
}


# 주행 단계의 역할(표준 계획에서 찾는다)
def _role_of(step_key):
    return gm.step_by_key_in(gm.standard_transfer_plan(), step_key).robot_role


class D1NavigationPort:
    """D1 NavigationClient 어댑터 초안. clients는 {robot_id: submit·cancel을 가진 객체}."""

    name = 'D1_NAVIGATION_CLIENT'
    provenance = gm.PROVENANCE_LIVE
    generates_goal_uuid = True

    # D1 클라이언트·카탈로그를 받는다. enabled=False면 지원하지 않는다(runtime이 보내지 않음)
    def __init__(self, clients, catalog, enabled=False):
        self.clients = dict(clients)
        self.catalog = catalog
        self.enabled = enabled
        self.submitted = {}

    # 명시적으로 켠 개발 시험에서만 사용한다. 운영 모드는 지원하지 않는다
    def supported(self, mode):
        return self.enabled and mode == gm.MODE_DEVELOPMENT_MOCK

    # 등록 지점 revision 대조 후 역할별 정차 자세(map 기준 x, y, yaw)
    def stop_pose(self, intent):
        args = intent.args
        info = self.catalog.get(args['location_id'])
        if info is None:
            raise PortError('UNKNOWN_LOCATION', uncertain=False)

        if (info['location_revision'] != args['location_revision']
                or self.catalog.map_revision != args['map_revision']):
            raise PortError('LOCATION_REVISION_MISMATCH', uncertain=False)

        pose = info['stop_poses'].get(_role_of(intent.step_key))
        if pose is None:
            raise PortError('STOP_POSE_MISSING', uncertain=False)

        return pose

    # D1 submit(같은 execution_id). D2 UUID는 받지 않고 D1 goal UUID를 돌려준다.
    # D1 예외는 예약 전 거절과 전송 실패를 밖에서 구분할 수 없어 모두 미확정으로 본다(보수적)
    def dispatch(self, intent, goal_uuid=None):
        if goal_uuid:
            raise PortError('D2_GOAL_UUID_NOT_ALLOWED', uncertain=False)

        client = self.clients.get(intent.robot_id)
        if client is None:
            raise PortUnsupported(f'D1_CLIENT_MISSING:{intent.robot_id}')

        pose = self.stop_pose(intent)
        execution = intent.execution

        try:
            row = client.submit(execution_id=execution['execution_id'],
                                assignment_id=execution['assignment_id'],
                                assignment_generation=execution['assignment_generation'],
                                x=pose['x'], y=pose['y'], yaw=pose['yaw'])
        except Exception as error:  # noqa: BLE001 - D1 예외 종류를 계약으로 정하지 않았다
            raise PortError(f'D1_SUBMIT_FAILED:{type(error).__name__}', uncertain=True) from error

        made = row.get('goal_uuid') if isinstance(row, dict) else None
        if type(made) is not str or not made:
            raise PortError('D1_GOAL_UUID_MISSING', uncertain=True)

        self.submitted[execution['execution_id']] = made

        return made

    # D1 cancel(execution_id). 살아 있는 goal handle이 없으면 D1이 거부하며, 취소 ACK는 종료가 아니다
    def cancel(self, intent):
        client = self.clients.get(intent.robot_id)
        if client is None:
            raise PortUnsupported(f'D1_CLIENT_MISSING:{intent.robot_id}')

        try:
            client.cancel(intent.execution['execution_id'])
        except Exception as error:  # noqa: BLE001
            raise PortError(f'D1_CANCEL_FAILED:{type(error).__name__}', uncertain=True) from error


# D1 원장 행(execution_id·robot·state·goal_uuid) → D2 결과 사건. 종료 전 상태면 None
def d1_result_event(row, robot_id, provenance=gm.PROVENANCE_LIVE):
    outcome = D1_TERMINAL_OUTCOMES.get(row.get('state'))
    if outcome is None:
        return None

    return StepResult(goal_uuid=row.get('goal_uuid', ''), robot_id=robot_id, outcome=outcome,
                      provenance=provenance, error_code=row.get('detail', ''),
                      execution_id=row['execution_id'])


# A4 ActuationState 필드 사전 → D2 관측. stamp는 초, 수신 시각은 호출자의 CLOCK_BOOTTIME
def actuation_observation(fields, received_boottime_s, provenance=gm.PROVENANCE_LIVE):
    meta = fields['meta']

    return gm.Observation(
        kind='ACTUATION', robot_id=fields['robot_id'], object_id='', value=fields['state'],
        source_id=meta['source_id'], session_id=meta['source_session_id'],
        revision=meta['revision'], valid=meta['valid'], stamp_sim_s=meta['stamp_s'],
        received_boottime_s=received_boottime_s, provenance=provenance,
        permits=(('drive_allowed', fields['drive_allowed']),
                 ('arm_allowed', fields['arm_allowed'])))


# C4 ToolHoldState.state 값(계약). UNKNOWN은 미보유가 아니다
HOLD_STATES = (gm.HOLD_UNKNOWN, gm.HOLD_HELD, gm.HOLD_RELEASED)

# 모의 판정 근거 접두어. C4 mock_execution은 valid=true로 'MOCK_ATTACH_STATE'를 낸다.
# C4가 meta·출처로 모의 판정을 구분하기 전까지 쓰는 임시 규칙이다
MOCK_EVIDENCE_PREFIX = 'MOCK'


# ToolHoldState 메시지(duck-typing) → 필드 사전. rclpy 없이 시험할 수 있게 속성만 읽는다
def hold_fields(msg):
    meta = msg.meta
    stamp = meta.header.stamp

    return {
        'robot_id': msg.robot_id,
        'object_id': msg.object_id,
        'state': int(msg.state),
        'evidence': msg.evidence,
        'meta': {
            'source_id': meta.source_id,
            'source_session_id': meta.source_session_id,
            'revision': int(meta.revision),
            'valid': bool(meta.valid),
            'stamp_s': stamp.sec + stamp.nanosec * 1e-9,
        },
    }


# 비어 있지 않은 문자열인지
def _filled(value):
    return type(value) is str and value != ''


# 보유 판정 필드의 형식 거절 사유(통과하면 None). receipt.observation_rejection과 같은 관례다.
# valid=false·만료·물체 불일치·앵커 이전 수신은 여기서 보지 않는다(group_machine._fresh가 거른다)
def hold_rejection(fields):
    meta = fields['meta']

    if type(fields['state']) is not int or fields['state'] not in HOLD_STATES:
        return 'HOLD_STATE_UNKNOWN_VALUE'

    if not _filled(fields['robot_id']):
        return 'HOLD_ROBOT_ID_EMPTY'

    if not _filled(fields['object_id']):
        return 'HOLD_OBJECT_ID_EMPTY'

    if not _filled(meta['source_id']) or not _filled(meta['source_session_id']):
        return 'HOLD_SOURCE_EMPTY'

    if type(meta['revision']) is not int or meta['revision'] <= 0:
        return 'HOLD_REVISION_UNSET'

    evidence = fields['evidence']
    if type(evidence) is not str:
        return 'HOLD_EVIDENCE_INVALID'

    if evidence.startswith(MOCK_EVIDENCE_PREFIX):
        return 'HOLD_MOCK_EVIDENCE'

    return None


# C4 ToolHoldState 필드 → (D2 관측, None) 또는 (None, 거절 사유). 수신 시각은 호출자의 CLOCK_BOOTTIME.
# 기본 출처 LIVE는 개발 모드 판정 근거가 아니다(provenance_acceptable, 정책 변경 없음)
def hold_observation(fields, received_boottime_s, provenance=gm.PROVENANCE_LIVE):
    reason = hold_rejection(fields)
    if reason is not None:
        return None, reason

    meta = fields['meta']

    obs = gm.Observation(
        kind='HOLD', robot_id=fields['robot_id'], object_id=fields['object_id'],
        value=fields['state'], source_id=meta['source_id'],
        session_id=meta['source_session_id'], revision=meta['revision'],
        valid=meta['valid'], stamp_sim_s=meta['stamp_s'],
        received_boottime_s=received_boottime_s, provenance=provenance)

    return obs, None
