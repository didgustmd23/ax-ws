"""D2 예비 재배정 사전 검사(설계 9.10) 시험(ROS 없음, 순수 함수).

실행: .venv/bin/python -m pytest -q -p no:launch_testing -p no:launch_ros \
      tools/tests/test_d2_reassignment.py
"""
import sys
from dataclasses import replace
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src' / 'ax_fleet_manager'))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from ax_fleet_manager import group_model as gm  # noqa: E402
from ax_fleet_manager.dev_fixtures import clock, make_assignment, make_spec  # noqa: E402
from ax_fleet_manager.group_events import AssignRoles  # noqa: E402
from ax_fleet_manager.group_machine import apply_event  # noqa: E402
from ax_fleet_manager.reassignment import check_spare_reassignment  # noqa: E402
from test_d2_group_machine import Flow  # noqa: E402

NOW = clock(100.0)
NEED = ('SUCTION_PICK',)


# 최신·유효 영역 meta
def meta(**changes):
    data = {'source_id': 'd4', 'source_session_id': 's1', 'revision': 3, 'valid': True,
            'stamp_s': 100.0}
    data.update(changes)

    return data


# 조작 역할 예비 후보(RobotSummary 필드 사전)
def candidate(**changes):
    data = {'robot_id': 'robot_m9', 'role': gm.MANIPULATOR,
            'capabilities': ['SUCTION_PICK', 'ARM_5DOF'], 'availability': 'STANDBY_SPARE',
            'assignment_id': '', 'assignment_generation': 0, 'location_id': '',
            'safety_meta': meta(), 'safety_level': 0,
            'actuation_meta': meta(), 'actuation_state': gm.ACTUATION_STOPPED,
            'localization_meta': meta(), 'localization_state': 0,
            'load_applicable': False, 'load_meta': meta(valid=False), 'load_state': 0}
    data.update(changes)

    return data


# 기존 조작 로봇의 실제 구동 차단(ActuationState 필드 사전)
def stopped(**changes):
    data = {'robot_id': 'robot_m7', 'state': gm.ACTUATION_STOPPED, 'drive_allowed': False,
            'arm_allowed': False, 'meta': meta()}
    data.update(changes)

    return data


# 배정만 된(전송 전) 작업 상태
def assigned_state():
    state = gm.new_group_state(make_spec())

    return apply_event(state, AssignRoles(make_assignment('robot_m7', 'robot_t3')),
                       clock(99.0)).next_state


# 판정 호출
def check(state=None, role=gm.MANIPULATOR, cand=None, actuation='default', others=()):
    return check_spare_reassignment(
        state or assigned_state(), role, cand if cand is not None else candidate(),
        stopped() if actuation == 'default' else actuation, NEED, NOW, others)


# 허용: 같은 역할·능력, STANDBY_SPARE·AVAILABLE, 영역 meta 최신, 기존 단계 없음, 기존 로봇 정지
@pytest.mark.parametrize('availability', ['STANDBY_SPARE', 'AVAILABLE'])
def test_allowed(availability):
    assert check(cand=candidate(availability=availability)) == (True, ())


# 운송 역할 후보는 적재 meta도 본다
def test_transporter_requires_load_meta():
    cand = candidate(robot_id='robot_t9', role=gm.TRANSPORTER, capabilities=['CARRY'],
                     load_applicable=True)
    actuation = stopped(robot_id='robot_t3')
    allowed, reasons = check_spare_reassignment(assigned_state(), gm.TRANSPORTER, cand,
                                                actuation, ('CARRY',), NOW)
    assert (allowed, reasons) == (False, ('CANDIDATE_META_STALE:load',))

    cand['load_meta'] = meta()
    assert check_spare_reassignment(assigned_state(), gm.TRANSPORTER, cand, actuation,
                                    ('CARRY',), NOW) == (True, ())


# 후보 검사 거절 사유
@pytest.mark.parametrize('changes, reason', [
    ({'role': gm.TRANSPORTER}, 'CANDIDATE_ROLE_MISMATCH'),
    ({'capabilities': ['ARM_5DOF']}, 'CANDIDATE_CAPABILITY_MISSING'),
    ({'capabilities': 'SUCTION_PICK'}, 'CANDIDATE_CAPABILITY_MISSING'),
    ({'availability': ''}, 'CANDIDATE_NOT_AVAILABLE'),
    ({'availability': 'FAULT'}, 'CANDIDATE_NOT_AVAILABLE'),
    ({'availability': 'BUSY'}, 'CANDIDATE_NOT_AVAILABLE'),
    ({'assignment_id': 'asg_other'}, 'CANDIDATE_ALREADY_ASSIGNED'),
    ({'actuation_meta': meta(valid=False)}, 'CANDIDATE_META_STALE:actuation'),
    ({'actuation_meta': meta(revision=0)}, 'CANDIDATE_META_STALE:actuation'),
    ({'actuation_meta': meta(stamp_s=99.6)}, 'CANDIDATE_META_STALE:actuation'),
    ({'safety_meta': meta(stamp_s=99.4)}, 'CANDIDATE_META_STALE:safety'),
    ({'localization_meta': meta(stamp_s=100.5)}, 'CANDIDATE_META_STALE:localization'),
    ({'localization_meta': None}, 'CANDIDATE_META_STALE:localization'),
    ({'robot_id': ''}, 'CANDIDATE_INVALID'),
    ({'robot_id': 'robot_m7'}, 'CANDIDATE_IS_DEPARTING_ROBOT'),
    ({'robot_id': 'robot_t3'}, 'CANDIDATE_IS_PARTNER'),
])
def test_candidate_rejections(changes, reason):
    allowed, reasons = check(cand=candidate(**changes))

    assert not allowed
    assert reasons == (reason,)


# 영역 meta 만료 경계: 구동 0.3 s는 받고 0.31 s는 거절
def test_meta_expiry_boundary():
    assert check(cand=candidate(actuation_meta=meta(stamp_s=99.7)))[0]
    assert not check(cand=candidate(actuation_meta=meta(stamp_s=99.69)))[0]


# 기존 로봇 구동 차단을 확인할 수 없으면 거절
@pytest.mark.parametrize('actuation', [
    None,
    stopped(robot_id='robot_t3'),
    stopped(state=1),
    stopped(drive_allowed=True),
    stopped(arm_allowed=True),
    stopped(meta=meta(valid=False)),
    stopped(meta=meta(stamp_s=99.0)),
    stopped(meta=meta(revision=0)),
])
def test_departing_actuation_unconfirmed(actuation):
    assert check(actuation=actuation) == (False, ('DEPARTING_ACTUATION_UNCONFIRMED',))


# 기존 로봇 단계가 진행 중이거나 미확정이면 거절. 상대 로봇 단계는 보지 않는다
def test_departing_steps():
    flow = Flow()
    flow.start()
    allowed, reasons = check(state=flow.state)
    assert not allowed and reasons == ('DEPARTING_STEP_ACTIVE',)

    flow.send_all()
    flow.finish('src_nav_manipulator', outcome=gm.OUTCOME_FAILED)
    assert flow.state.steps['src_nav_transporter'].status == 'ACCEPTED'
    assert check(state=flow.state) == (True, ())

    uncertain = flow.state.copy()
    uncertain.steps['src_nav_manipulator'].status = 'UNCERTAIN'
    assert check(state=uncertain) == (False, ('DEPARTING_STEP_UNCERTAIN',))


# 인계 상태가 NONE·UNLOCKED가 아니면 거절
@pytest.mark.parametrize('transfer, ok', [
    (gm.TRANSFER_NONE, True), (gm.TRANSFER_UNLOCKED, True), (gm.TRANSFER_UNKNOWN, False),
    (gm.TRANSFER_LOCKED, False), (gm.TRANSFER_IN_PROGRESS, False),
])
def test_transfer_state(transfer, ok):
    state = replace(assigned_state(), transfer_state=transfer)

    assert check(state=state) == ((True, ()) if ok else (False, ('TRANSFER_NOT_RELEASED',)))


# 물품 보유·적재 중이면 회수·인계 절차가 없어 거절. 상태 불명도 보수적으로 거절
@pytest.mark.parametrize('object_state, reason', [
    (gm.INV_AT_SOURCE, None), (gm.INV_AT_DESTINATION, None),
    (gm.INV_HELD_BY_MANIPULATOR, 'OBJECT_HELD_NO_RECOVERY_PROCEDURE'),
    (gm.INV_STOWED_ON_TRANSPORTER, 'OBJECT_HELD_NO_RECOVERY_PROCEDURE'),
    (gm.INV_UNKNOWN, 'OBJECT_STATE_UNKNOWN'), (gm.INV_LOST, 'OBJECT_STATE_UNKNOWN'),
])
def test_object_state(object_state, reason):
    state = replace(assigned_state(), object_state=object_state)

    assert check(state=state) == ((True, ()) if reason is None else (False, (reason,)))


# 작업 배정 없음·종료 작업·역할 이름 오류·후보 형식 오류
def test_task_level_rejections():
    unassigned = gm.new_group_state(make_spec())
    assert check(state=unassigned) == (False, ('TASK_NOT_ASSIGNED',))

    done = replace(assigned_state(), task_state=gm.TASK_FAILED)
    assert check(state=done) == (False, ('TASK_TERMINAL',))

    assert check(role='DRIVER') == (False, ('INVALID_ROLE',))
    assert check_spare_reassignment(assigned_state(), gm.MANIPULATOR, 'robot_m9', stopped(),
                                    NEED, NOW) == (False, ('CANDIDATE_INVALID',))


# 다른 진행 중 작업이 후보 로봇을 쓰면 거절
def test_candidate_in_other_active_task():
    other_spec = make_spec('task_other', 'box_other', root_execution_id='root_other',
                           source_transfer_id='xfer_o1', destination_transfer_id='xfer_o2')
    other = apply_event(gm.new_group_state(other_spec),
                        AssignRoles(make_assignment('robot_m9', 'robot_t8')),
                        clock(99.0)).next_state

    assert check(others=(other,)) == (False, ('CANDIDATE_IN_ACTIVE_TASK',))

    finished = replace(other, task_state=gm.TASK_COMPLETED)
    assert check(others=(finished,)) == (True, ())


# 순수 함수: 입력 상태를 바꾸지 않는다
def test_does_not_mutate_state():
    state = assigned_state()
    before = state.copy()
    check(state=state)

    assert state == before


# 여러 사유를 모두 모은다
def test_collects_all_reasons():
    state = replace(assigned_state(), transfer_state=gm.TRANSFER_LOCKED,
                    object_state=gm.INV_HELD_BY_MANIPULATOR)
    allowed, reasons = check(state=state, cand=candidate(availability='BUSY'), actuation=None)

    assert not allowed
    assert set(reasons) == {'CANDIDATE_NOT_AVAILABLE', 'TRANSFER_NOT_RELEASED',
                            'OBJECT_HELD_NO_RECOVERY_PROCEDURE',
                            'DEPARTING_ACTUATION_UNCONFIRMED'}
