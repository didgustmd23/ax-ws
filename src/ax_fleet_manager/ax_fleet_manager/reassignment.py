"""예비 재배정 사전 검사(설계 9.10, 순수 함수). 재배정을 실행하거나 원장·상태를 바꾸지 않는다.

- D2에는 로봇 풀이 없다. 후보는 D4 FleetSnapshot의 RobotSummary 필드 사전으로 받는다.
- 기존 로봇의 실제 구동 차단은 A4 ActuationState 필드 사전으로 받는다. 없거나 확인할 수 없으면 거절한다.
- 정책이 미정인 항목(예산 승계, 기존 권한 철회·새 배정 세대 부여 절차, 재시작한 고장 로봇의 과거 명령
  재실행 방지 검증)은 판정하지 않는다. 허용 결과는 "사전 조건 충족"일 뿐 재배정 승인이 아니다.
"""
from . import group_model as gm
from .registry import _occupies

# 후보로 받는 가용 상태(계약: 4종, 빈 문자열은 가용이 아님)
CANDIDATE_AVAILABILITY = ('AVAILABLE', 'STANDBY_SPARE')

# RobotSummary 영역 meta 만료(D-05: 구동 상태 0.3 s, 안전 0.5 s, 위치 신뢰성 1.0 s, 적재 0.5 s)
SUMMARY_META_EXPIRY_S = {
    'actuation': 0.3,
    'safety': 0.5,
    'localization': 1.0,
    'load': 0.5,
}

# A4 ActuationState 만료(D-05 구동 상태 0.3 s)
ACTUATION_EXPIRY_S = 0.3

# 기존 로봇 단계가 끝나지 않은 상태(의도·예약·전송·수락)
ACTIVE_STEP_STATUSES = ('INTENDED', 'RESERVED', 'SENT', 'ACCEPTED')

# 물품을 들거나 싣고 있어 검증된 회수·인계 절차 없이는 예비를 투입할 수 없는 재고 상태
HELD_OBJECT_STATES = (gm.INV_HELD_BY_MANIPULATOR, gm.INV_STOWED_ON_TRANSPORTER)

# 물품 상태를 모르는 재고 상태(보유 여부를 배제할 수 없다)
UNKNOWN_OBJECT_STATES = (gm.INV_UNKNOWN, gm.INV_LOST)


# 상태 meta(사전)가 유효·revision 있음·최신(sim 나이 0~만료)인지
def _meta_fresh(meta, expiry_s, now):
    if not isinstance(meta, dict):
        return False

    if meta.get('valid') is not True:
        return False

    revision = meta.get('revision')
    if type(revision) is not int or revision <= 0:
        return False

    stamp = meta.get('stamp_s')
    if not isinstance(stamp, (int, float)) or isinstance(stamp, bool):
        return False

    age = now.sim_time_s - stamp

    return 0.0 <= age <= expiry_s


# 후보 RobotSummary의 역할·능력·가용·배정·영역 meta 검사 사유
def _candidate_reasons(candidate, role, required_capabilities, now):
    reasons = []

    # 다른 역할끼리 대체하지 않는다
    if candidate.get('role') != role:
        reasons.append('CANDIDATE_ROLE_MISMATCH')

    capabilities = candidate.get('capabilities')
    if not isinstance(capabilities, (list, tuple)) or not set(required_capabilities) <= set(
            capabilities):
        reasons.append('CANDIDATE_CAPABILITY_MISSING')

    if candidate.get('availability') not in CANDIDATE_AVAILABILITY:
        reasons.append('CANDIDATE_NOT_AVAILABLE')

    if candidate.get('assignment_id', '') != '':
        reasons.append('CANDIDATE_ALREADY_ASSIGNED')

    # 영역 meta: 구동·안전·위치는 항상, 적재는 적재 대상(운송 역할)일 때만 본다
    areas = ['actuation', 'safety', 'localization']
    if candidate.get('load_applicable') is True:
        areas.append('load')

    for area in areas:
        if not _meta_fresh(candidate.get(f'{area}_meta'), SUMMARY_META_EXPIRY_S[area], now):
            reasons.append(f'CANDIDATE_META_STALE:{area}')

    return reasons


# 기존 로봇의 실제 구동 차단 확인: 최신·유효 ActuationState가 STOPPED이고 주행·팔 허가가 모두 거짓
def _departing_stopped(actuation, robot_id, now):
    if not isinstance(actuation, dict) or actuation.get('robot_id') != robot_id:
        return False

    if not _meta_fresh(actuation.get('meta'), ACTUATION_EXPIRY_S, now):
        return False

    return (actuation.get('state') == gm.ACTUATION_STOPPED
            and actuation.get('drive_allowed') is False
            and actuation.get('arm_allowed') is False)


# 기존 로봇 단계·작업 인계·물품 상태 검사 사유
def _task_reasons(state, departing):
    reasons = []
    plan = {step.step_key: step for step in state.spec.steps}
    runs = [run for key, run in state.steps.items()
            if state.assignment.robot_for(plan[key].robot_role) == departing]

    if any(run.status in ACTIVE_STEP_STATUSES for run in runs):
        reasons.append('DEPARTING_STEP_ACTIVE')

    if any(run.status == 'UNCERTAIN' for run in runs):
        reasons.append('DEPARTING_STEP_UNCERTAIN')

    # 인계 잠금이 남았거나 불명이면 물품 보유·인계 중일 수 있다
    if state.transfer_state not in gm.RELEASED_TRANSFER_STATES:
        reasons.append('TRANSFER_NOT_RELEASED')

    if state.object_state in HELD_OBJECT_STATES:
        reasons.append('OBJECT_HELD_NO_RECOVERY_PROCEDURE')
    elif state.object_state in UNKNOWN_OBJECT_STATES:
        reasons.append('OBJECT_STATE_UNKNOWN')

    return reasons


# 예비 재배정 사전 검사. state는 대상 작업, role은 교체할 역할, candidate는 RobotSummary 필드 사전,
# departing_actuation은 기존 로봇의 ActuationState 필드 사전(없으면 None), other_states는 다른 작업 상태.
# 결과는 (허용 여부, 사유 코드 tuple). 사유를 모두 모으며 하나라도 있으면 거절이다
def check_spare_reassignment(state, role, candidate, departing_actuation, required_capabilities,
                             now, other_states=()):
    if role not in gm.ROLES:
        return False, ('INVALID_ROLE',)

    if state.assignment is None:
        return False, ('TASK_NOT_ASSIGNED',)

    if not isinstance(candidate, dict):
        return False, ('CANDIDATE_INVALID',)

    reasons = []
    departing = state.assignment.robot_for(role)
    partner = state.assignment.robot_for(
        gm.TRANSPORTER if role == gm.MANIPULATOR else gm.MANIPULATOR)
    candidate_id = candidate.get('robot_id')

    # 실패·취소·완료 작업은 RUNNING으로 되돌리지 않는다(후속 작업 승인 절차 몫)
    if state.task_state in gm.TERMINAL_TASK_STATES:
        reasons.append('TASK_TERMINAL')

    # 후보 식별: 비어 있거나 기존 로봇·상대 로봇이면 거절
    if type(candidate_id) is not str or not gm.ROBOT_ID_PATTERN.match(candidate_id):
        reasons.append('CANDIDATE_INVALID')
    elif candidate_id == departing:
        reasons.append('CANDIDATE_IS_DEPARTING_ROBOT')
    elif candidate_id == partner:
        reasons.append('CANDIDATE_IS_PARTNER')

    reasons += _candidate_reasons(candidate, role, required_capabilities, now)

    # 다른 진행 중·잠금 잔존 작업이 후보 로봇을 쓰고 있으면 거절(작업 간 상호 배제)
    for other in other_states:
        if (other.spec.task_id != state.spec.task_id and other.assignment is not None
                and _occupies(other) and candidate_id in other.assignment.robots()):
            reasons.append('CANDIDATE_IN_ACTIVE_TASK')
            break

    reasons += _task_reasons(state, departing)

    if not _departing_stopped(departing_actuation, departing, now):
        reasons.append('DEPARTING_ACTUATION_UNCONFIRMED')

    return not reasons, tuple(reasons)
