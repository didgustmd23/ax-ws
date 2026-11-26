"""상대 기능 포트: 실제(live) 포트는 W3에서 모두 미지원, 개발 모의 포트만 동작한다.

- LocalTaskPort: A3 ExecuteLocalTask Goal 전송·취소. live 연결(실 A3)은 W3 범위 밖이라 미지원.
- NavigationPort: D1의 실행 ID·goal UUID·취소를 제공하는 실행 API가 아직 없어 미지원.
  주행 포트는 goal UUID를 스스로 만든다(generates_goal_uuid, W5). dispatch(intent, None)이 D1의
  goal UUID를 돌려주고, D2는 그 값을 참고 기록한다. 결합 키는 execution_id다.
- 모의 포트는 프로세스·인스턴스가 드러나는 source_id와 provenance=MOCK을 가진다. 받은 요청을
  기록만 하며 ROS Action·속도·팔·공구·안전 해제를 호출하거나 발행하지 않는다.
"""
import itertools
import os
import uuid

from .group_model import (
    DEVELOPMENT_MODES, EVIDENCE_LOCAL_TASK, LIVE_KIND_NAVIGATION, PROVENANCE_MOCK,
)

# W3에서 연결하지 않은 상대 기능. 운영 실행 전 각 소유자의 구현·계약 확정이 필요하다
UNSUPPORTED_EXTERNAL_PORTS = {
    'A3_LIVE_EXECUTE_LOCAL_TASK': '실 A3 ExecuteLocalTask 서버 연결(실제 조작 dispatcher 미완료)',
    'D1_NAVIGATION_EXECUTION_API': 'D1 실행 ID·goal UUID·취소를 제공하는 주행 실행 API 미정',
    'C4_MANIPULATION': 'C4 실제 조작 서버·대상 대응 판정기 미완료',
    'D3_CORRIDOR_PERMIT': 'D3 통로 허가 연결(W3 계획은 REQUEST_CORRIDOR 미지원)',
    'D4_HOLD_LIFECYCLE': 'D4 보류 생명주기(9.6) 전체. 보류 해제 미지원',
    'D4_ASSIGNMENT_CHECK': 'D4 배정 확인 응답·운영 승인 연결',
    'REGISTERED_LOCATION_METADATA': '등록 지점·물품 원장 pose·revision 조회',
    'PLACEMENT_EVIDENCE_PRODUCER': '목적지 배치 확인 생산자 미정',
    'ARM_RETRACT_EVIDENCE_PRODUCER': '팔 후퇴 위치 확인 생산자 미정',
    'A4_ACTUATION_LIVE_ADAPTER': 'ActuationState schema 전환 확정 전. 실 A4 수락 정책 미정',
}

_INSTANCE = itertools.count(1)


class PortError(Exception):
    """포트 호출 실패. uncertain=True면 상대가 받았는지 알 수 없다."""

    # 실패 사유와 불확실 여부를 보관한다
    def __init__(self, reason, uncertain):
        super().__init__(reason)
        self.reason = reason
        self.uncertain = uncertain


class PortUnsupported(PortError):
    """연결하지 않은 포트. 전송하지 않았음이 확실하다."""

    # 미지원 사유를 확정 미전송으로 만든다
    def __init__(self, reason):
        super().__init__(reason, uncertain=False)


class LocalTaskPort:
    """실 A3 로컬 작업 포트(W3 미지원)."""

    name = 'A3_LIVE_EXECUTE_LOCAL_TASK'
    provenance = 'LIVE'

    # 이 모드·live 허용 종류에서 사용할 수 있는지
    def supported(self, mode, live_kinds=()):
        return False

    # Goal 전송(미지원)
    def dispatch(self, intent, goal_uuid):
        raise PortUnsupported(f'PORT_UNSUPPORTED:{self.name}')

    # 취소 전송(미지원)
    def cancel(self, intent):
        raise PortUnsupported(f'PORT_UNSUPPORTED:{self.name}')


class NavigationPort(LocalTaskPort):
    """D1 주행 실행 포트(실행 API 미정으로 W3 미지원)."""

    name = 'D1_NAVIGATION_EXECUTION_API'
    generates_goal_uuid = True


# ExecuteLocalTask Goal 필드 사전(msg 필드 이름 그대로). pose는 원장 조회 포트가 없어 None
def local_task_goal_fields(intent):
    args = intent.args

    return {
        'execution': dict(intent.execution),
        'task_kind': args['task_kind'],
        'peer_robot_id': args['peer_robot_id'],
        'object_id': args['object_id'],
        'inventory_revision': args['inventory_revision'],
        'source_location_id': args['source_location_id'],
        'source_location_revision': args['source_location_revision'],
        'destination_location_id': args['destination_location_id'],
        'destination_location_revision': args['destination_location_revision'],
        'pick_pose': None,
        'place_pose': None,
        'map_revision': args['map_revision'],
        'budgets': [],
        'has_candidate_preference': False,
        'candidate_preference': None,
    }


# D1 주행 의도 사전(내부 계약 초안, 공개 API 아님): 대상 등록 지점과 실행 식별자만 넘긴다
def navigation_request_fields(intent):
    args = intent.args

    return {
        'execution': dict(intent.execution),
        'robot_id': intent.robot_id,
        'location_id': args['location_id'],
        'location_revision': args['location_revision'],
        'map_revision': args['map_revision'],
    }


class MockLocalTaskPort(LocalTaskPort):
    """개발 모의 A3 포트. 요청을 기록만 한다(ROS 호출 없음)."""

    name = 'MOCK_LOCAL_TASK'
    provenance = PROVENANCE_MOCK
    # 이 포트가 내는 결과의 근거 종류. LOCAL_TASK는 열 수 없으므로 항상 MOCK 대상이다
    evidence_kind = EVIDENCE_LOCAL_TASK
    convert = staticmethod(local_task_goal_fields)

    # 프로세스 ID·인스턴스 번호로 출처를 구분한다
    def __init__(self, label='mock'):
        self.source_id = f'd2_{self.name.lower()}:{os.getpid()}:{next(_INSTANCE)}:{label}'
        self.requests = []
        self.cancels = []
        self.fail_next = None

    # 두 개발 모드(모의·혼합)에서 사용 가능. 단, 담당 근거 종류를 live로 열었으면 MOCK 결과는
    # 받을 수 없으므로 거절한다(종류마다 출처 하나, 설정 실수를 전송 전에 PORT_UNSUPPORTED로 드러냄)
    def supported(self, mode, live_kinds=()):
        return mode in DEVELOPMENT_MODES and self.evidence_kind not in live_kinds

    # 요청 기록. fail_next에 PortError를 넣으면 한 번 실패한다
    def dispatch(self, intent, goal_uuid):
        if self.fail_next is not None:
            error, self.fail_next = self.fail_next, None
            raise error

        self.requests.append({'goal_uuid': goal_uuid, 'robot_id': intent.robot_id,
                              'step_key': intent.step_key, 'operation': intent.operation,
                              'fields': self.convert(intent)})

    # 취소 요청 기록
    def cancel(self, intent):
        self.cancels.append({'goal_uuid': intent.goal_uuid, 'robot_id': intent.robot_id,
                             'step_key': intent.step_key})


class MockNavigationPort(MockLocalTaskPort):
    """개발 모의 주행 포트. Nav2·cmd_vel을 사용하지 않는다.

    D1 방식을 흉내 낸다: goal UUID는 포트(D1)가 execution_id마다 한 번 만들고 같은 execution_id
    재호출에는 같은 UUID를 돌려준다(D1 ExecutionLedger.reserve와 같은 멱등).
    """

    name = 'MOCK_NAVIGATION'
    evidence_kind = LIVE_KIND_NAVIGATION
    convert = staticmethod(navigation_request_fields)
    generates_goal_uuid = True

    # 출처와 execution_id별 UUID 기록을 준비한다
    def __init__(self, label='mock'):
        super().__init__(label)
        self.goal_uuids = {}

    # 요청 기록 후 D1처럼 만든 goal UUID를 돌려준다(D2가 준 값은 쓰지 않는다)
    def dispatch(self, intent, goal_uuid=None):
        execution_id = intent.execution['execution_id']
        made = self.goal_uuids.setdefault(execution_id, uuid.uuid4().hex)
        super().dispatch(intent, made)

        return made
