"""D2 포트·fixture 형식이 기존 ax_interfaces 정의·계약과 맞는지 확인(ROS 없음).

실행: .venv/bin/python -m pytest -q -p no:launch_testing -p no:launch_ros \
      tools/tests/test_d2_ports.py
"""
import os
import re
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src' / 'ax_fleet_manager'))
IFACE = ROOT / 'src' / 'ax_interfaces'

from ax_fleet_manager import group_model as gm  # noqa: E402
from ax_fleet_manager.dev_fixtures import clock, make_assignment, make_spec  # noqa: E402
from ax_fleet_manager.group_events import AssignRoles, StartTask  # noqa: E402
from ax_fleet_manager.group_machine import apply_event  # noqa: E402
from ax_fleet_manager.ports import (  # noqa: E402
    UNSUPPORTED_EXTERNAL_PORTS, LocalTaskPort, MockLocalTaskPort, MockNavigationPort,
    NavigationPort, PortUnsupported, local_task_goal_fields,
)
from ax_fleet_manager.snapshot import GROUP_TASK_STATUS_FIELDS, task_status_fields  # noqa: E402


# .msg·.action 한 구역의 필드 이름과 상수 값을 읽는다
def parse_fields(text):
    fields, constants = [], {}
    for line in text.splitlines():
        line = line.split('#', 1)[0].strip()
        if not line:
            continue
        match = re.match(r'^(\S+)\s+([A-Za-z_][A-Za-z0-9_]*)\s*=\s*(\S+)$', line)
        if match:
            constants[match.group(2)] = int(match.group(3))
            continue
        fields.append(line.split()[1])

    return fields, constants


# msg 파일 읽기
def msg(name):
    return parse_fields((IFACE / 'msg' / f'{name}.msg').read_text(encoding='utf-8'))


# action Goal 구역 읽기
def action_goal(name):
    text = (IFACE / 'action' / f'{name}.action').read_text(encoding='utf-8')
    return parse_fields(text.split('---')[0])


# 첫 의도 하나를 만든다
def first_intent(operation='LOCK_HANDOFF'):
    spec = make_spec()
    state = gm.new_group_state(spec)
    state.assignment = make_assignment('robot_m1', 'robot_t9')
    plan_step = [s for s in spec.steps if s.operation == operation][0]
    from ax_fleet_manager.group_events import CommandIntent
    execution, args = gm.build_execution(spec, state.assignment, plan_step)
    return CommandIntent('LOCAL_TASK', spec.task_id, plan_step.step_key, operation,
                         args['robot_id'], args['peer_robot_id'], execution, args,
                         gm.args_digest({'execution': execution, 'args': args}))


# ExecuteLocalTask Goal 필드 이름과 같다
def test_local_task_goal_fields_match_action():
    fields, constants = action_goal('ExecuteLocalTask')
    goal = local_task_goal_fields(first_intent())
    assert list(goal) == fields
    assert {k: v for k, v in constants.items() if k != 'UNSPECIFIED'} == gm.LOCAL_TASK_KINDS


# ExecutionRef 필드 이름과 같다
def test_execution_fields_match_execution_ref():
    fields, _ = msg('ExecutionRef')
    assert list(first_intent().execution) == fields


# GroupTaskStatus 필드·상수와 같다(새 공개 값 없음)
def test_group_task_status_fields_and_constants():
    fields, constants = msg('GroupTaskStatus')
    assert list(GROUP_TASK_STATUS_FIELDS) == fields
    state = gm.new_group_state(make_spec())
    assert list(task_status_fields(state)) == fields

    expected = {
        'UNSPECIFIED': gm.TASK_UNSPECIFIED, 'QUEUED': gm.TASK_QUEUED,
        'ASSIGNED': gm.TASK_ASSIGNED, 'RUNNING': gm.TASK_RUNNING,
        'COMPLETED': gm.TASK_COMPLETED, 'FAILED': gm.TASK_FAILED,
        'CANCELED': gm.TASK_CANCELED, 'RETRY_PENDING': gm.TASK_RETRY_PENDING,
        'TRANSFER_UNKNOWN': gm.TRANSFER_UNKNOWN, 'TRANSFER_NONE': gm.TRANSFER_NONE,
        'TRANSFER_LOCKED': gm.TRANSFER_LOCKED, 'TRANSFER_IN_PROGRESS': gm.TRANSFER_IN_PROGRESS,
        'TRANSFER_UNLOCKED': gm.TRANSFER_UNLOCKED,
    }
    assert constants == expected


# 결과·보유·적재·구동 상수가 msg 정의와 같다
def test_result_and_state_constants():
    _, result = msg('ResultStatus')
    assert result == {'UNSPECIFIED': 0, 'SUCCEEDED': gm.OUTCOME_SUCCEEDED,
                      'FAILED': gm.OUTCOME_FAILED, 'CANCELED': gm.OUTCOME_CANCELED,
                      'TIMED_OUT': gm.OUTCOME_TIMED_OUT, 'UNCERTAIN': gm.OUTCOME_UNCERTAIN}
    _, hold = msg('ToolHoldState')
    assert (hold['UNKNOWN'], hold['HELD'], hold['RELEASED']) == (
        gm.HOLD_UNKNOWN, gm.HOLD_HELD, gm.HOLD_RELEASED)
    _, load = msg('LoadState')
    assert (load['UNKNOWN'], load['STOWED'], load['EMPTY'], load['LOST']) == (
        gm.LOAD_UNKNOWN, gm.LOAD_STOWED, gm.LOAD_EMPTY, gm.LOAD_LOST)
    _, actuation = msg('ActuationState')
    assert actuation['STOPPED'] == gm.ACTUATION_STOPPED


# 계획 operation·역할 이름이 계약 D-11·PlanStep 주석과 같다
def test_plan_operations_match_contract():
    text = (ROOT / 'docs' / 'design' / 'INTERFACE_CONTRACT.md').read_text(encoding='utf-8')
    line = [x for x in text.splitlines() if x.startswith('| D-11 ')][0]
    named = set(re.findall(r'`([A-Z_]+)`', line))
    d10 = [x for x in text.splitlines() if x.startswith('| D-10 ')][0]
    named |= set(re.findall(r'`([A-Z_]+)`', d10))
    assert set(gm.PLAN_OPERATIONS) == named

    plan_step = (IFACE / 'msg' / 'PlanStep.msg').read_text(encoding='utf-8')
    assert 'MANIPULATOR, TRANSPORTER' in plan_step
    assert set(gm.ROLES) == {'MANIPULATOR', 'TRANSPORTER'}


# 주행 단계 operation 이름은 D-03 규칙대로 NAVIGATE_TO, 인계 ID 없음
def test_navigation_execution_ref():
    intent = first_intent('NAVIGATE_TO')
    assert intent.execution['operation'] == 'NAVIGATE_TO'
    assert intent.execution['transfer_id'] == ''
    assert intent.peer_robot_id == ''
    assert intent.args['task_kind'] == 0


# 적재·하역 pose는 원장 조회 포트가 없어 만들지 않는다(정답 좌표로 꾸미지 않음)
def test_goal_poses_not_fabricated():
    goal = local_task_goal_fields(first_intent('PICK_AND_STOW'))
    assert goal['pick_pose'] is None and goal['place_pose'] is None
    assert goal['budgets'] == [] and goal['has_candidate_preference'] is False


# live 포트는 미지원이며 전송하지 않았음이 확실한 실패를 낸다
@pytest.mark.parametrize('port', [LocalTaskPort(), NavigationPort()])
def test_live_ports_unsupported(port):
    assert not port.supported(gm.MODE_DEVELOPMENT_MOCK)
    assert not port.supported(gm.MODE_OPERATIONAL)
    with pytest.raises(PortUnsupported) as error:
        port.dispatch(first_intent(), 'uuid')
    assert error.value.uncertain is False


# 모의 포트: 출처에 프로세스·인스턴스 구분, MOCK 출처, 개발 모드에서만 사용
def test_mock_ports_identify_source():
    a, b = MockLocalTaskPort('a'), MockNavigationPort('b')
    assert a.provenance == b.provenance == gm.PROVENANCE_MOCK
    assert str(os.getpid()) in a.source_id and a.source_id != b.source_id
    assert a.supported(gm.MODE_DEVELOPMENT_MOCK)
    assert not a.supported(gm.MODE_OPERATIONAL)
    a.dispatch(first_intent(), 'uuid_1')
    assert a.requests[0]['fields']['execution']['execution_id']


# 미구현 상대 기능 목록(운영 실행 차단 근거)
def test_unsupported_external_ports_listed():
    for key in ('A3_LIVE_EXECUTE_LOCAL_TASK', 'D1_NAVIGATION_EXECUTION_API',
                'C4_MANIPULATION', 'D3_CORRIDOR_PERMIT', 'D4_HOLD_LIFECYCLE',
                'PLACEMENT_EVIDENCE_PRODUCER', 'A4_ACTUATION_LIVE_ADAPTER'):
        assert key in UNSUPPORTED_EXTERNAL_PORTS


# 엔진 의도는 실제 구동 명령 종류를 만들지 않는다(주행·로컬 작업·취소 의도만)
def test_intent_kinds_are_not_actuation():
    state = gm.new_group_state(make_spec())
    d = apply_event(state, AssignRoles(make_assignment('robot_m1', 'robot_t9')), clock(1.0))
    d = apply_event(d.next_state, StartTask(), clock(1.1))
    assert {e.kind for e in d.effects} == {'NAVIGATION'}
    for e in d.effects:
        assert 'cmd_vel' not in repr(e) and 'joint' not in repr(e).lower()
