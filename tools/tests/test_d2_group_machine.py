"""D2 W3 순수 그룹 단계 엔진 시험(ROS 없음, 모의 관측).

실행: .venv/bin/python -m pytest -q -p no:launch_testing -p no:launch_ros \
      tools/tests/test_d2_group_machine.py
"""
import sys
from dataclasses import replace
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src' / 'ax_fleet_manager'))

from ax_fleet_manager import group_model as gm  # noqa: E402
from ax_fleet_manager.dev_fixtures import (  # noqa: E402
    MockObservationSource, clock, make_assignment, make_spec, spec_dict,
)
from ax_fleet_manager.execution_ids import make_execution_id  # noqa: E402
from ax_fleet_manager.group_events import (  # noqa: E402
    AssignRoles, CancelAcknowledged, CancelRequested, DispatchFailed, DispatchReserved,
    DispatchSent, FailTerminate, GoalResponse, HoldApplied, HoldReleaseRequested,
    ObservationReceived, StartTask, StepResult, StoreFault, TaskStatusSeen, Tick,
)
from ax_fleet_manager.group_machine import apply_event, restore_state  # noqa: E402
from ax_fleet_manager.registry import GroupRegistry  # noqa: E402

MOCK = gm.PROVENANCE_MOCK


class Flow:
    """엔진만 쓰는 시험 구동기. 모의 상대 역할(결과·관측)을 시험 코드가 직접 넣는다."""

    # 작업·배정·시각을 준비한다
    def __init__(self, manipulator='robot_m7', transporter='robot_t3', spec=None):
        self.spec = spec or make_spec()
        self.assignment = make_assignment(manipulator, transporter)
        self.state = gm.new_group_state(self.spec)
        self.t = 100.0
        self.source = MockObservationSource()
        self.intents = []
        self.decisions = []

    # 현재 시각
    def now(self):
        return clock(self.t)

    # 시간을 진행한다
    def wait(self, dt):
        self.t += dt

    # 사건 하나를 적용하고 의도를 모은다
    def apply(self, event):
        decision = apply_event(self.state, event, self.now())
        self.state = decision.next_state
        self.intents.extend(decision.effects)
        self.decisions.append(decision)

        return decision

    # 배정 후 실행 시작
    def start(self):
        self.apply(AssignRoles(self.assignment))
        return self.apply(StartTask())

    # 아직 전송하지 않은 의도
    def unsent(self):
        return [i for i in self.intents
                if i.kind != 'CANCEL' and self.state.steps[i.step_key].status == 'INTENDED']

    # 단계 의도의 robot_id
    def robot(self, step_key):
        plan_step = gm.step_by_key(self.spec, step_key)
        return self.assignment.robot_for(plan_step.robot_role)

    # 의도 하나를 원장 기록·전송·수락까지 진행한다
    def send(self, intent):
        uuid = f'uuid_{intent.step_key}'
        self.apply(DispatchReserved(intent.execution['execution_id'], uuid))
        self.apply(DispatchSent(intent.execution['execution_id']))
        self.apply(GoalResponse(uuid, True, MOCK))

        return uuid

    # 전송 안 된 의도를 모두 전송한다
    def send_all(self):
        return [self.send(i) for i in self.unsent()]

    # 단계 결과를 넣는다
    def finish(self, step_key, outcome=gm.OUTCOME_SUCCEEDED, hold=0, load=0, robot=None):
        return self.apply(StepResult(f'uuid_{step_key}', robot or self.robot(step_key),
                                     outcome, MOCK, '', hold, load))

    # 관측 하나를 넣는다
    def observe(self, kind, robot, value, **kwargs):
        return self.apply(ObservationReceived(
            self.source.make(kind, robot, value, self.now(), **kwargs)))

    # 이름으로 묶은 관측을 넣는다
    def evidence(self, *names):
        m, t = self.assignment.robots()
        obj = self.spec.object_id
        table = {
            'stopped': [('ACTUATION', m, gm.ACTUATION_STOPPED, ''),
                        ('ACTUATION', t, gm.ACTUATION_STOPPED, '')],
            'stowed': [('LOAD', t, gm.LOAD_STOWED, obj)],
            'empty': [('LOAD', t, gm.LOAD_EMPTY, obj)],
            'released': [('HOLD', m, gm.HOLD_RELEASED, obj)],
            'held': [('HOLD', m, gm.HOLD_HELD, obj)],
            'retracted': [('ARM_RETRACTED', m, 1, '')],
            'placed': [('PLACEMENT', m, 1, obj)],
        }
        for name in names:
            for kind, robot, value, object_id in table[name]:
                self.observe(kind, robot, value, object_id=object_id)

    # 의도 operation 순서
    def operations(self):
        return [(i.step_key, i.operation) for i in self.intents if i.kind != 'CANCEL']

    # 지정 단계 의도가 나왔는지
    def intended(self, step_key):
        return any(i.step_key == step_key for i in self.intents)


# 지정 단계까지 정상 흐름을 진행한다
def run_until(flow, stop_after):
    plan = [
        ('src_nav', ['src_nav_manipulator', 'src_nav_transporter'], {}, ['stopped']),
        ('src_lock', ['src_lock_manipulator', 'src_lock_transporter'], {}, ['stopped']),
        ('pick', ['load_pick_and_stow'], {}, ['stowed']),
        ('confirm', ['load_confirm'], {'load': gm.LOAD_STOWED}, ['stowed']),
        ('retract', ['load_retract'], {},
         ['stowed', 'released', 'retracted', 'stopped']),
        ('src_unlock', ['src_unlock_manipulator', 'src_unlock_transporter'], {},
         ['stowed', 'released', 'retracted', 'stopped']),
        ('dst_nav', ['dst_nav_manipulator', 'dst_nav_transporter'], {}, ['stopped']),
        ('dst_lock', ['dst_lock_manipulator', 'dst_lock_transporter'], {},
         ['stopped', 'stowed']),
        ('unload', ['unload_place'], {}, ['placed']),
        ('unload_retract', ['unload_retract'], {}, ['placed', 'empty']),
        ('confirm_empty', ['unload_confirm_empty'], {'load': gm.LOAD_EMPTY},
         ['empty', 'placed', 'released', 'retracted', 'stopped']),
        ('dst_unlock', ['dst_unlock_manipulator', 'dst_unlock_transporter'], {},
         ['empty', 'placed', 'released', 'retracted', 'stopped']),
    ]
    flow.start()
    for name, keys, result, evidence in plan:
        assert sorted(i.step_key for i in flow.unsent()) == sorted(keys), name
        flow.send_all()
        flow.wait(0.05)
        flow.evidence(*evidence)
        for key in keys:
            flow.finish(key, **result)

        if name == stop_after:
            return


# 정상 운반 fixture: 고정 순서, 실행 ID 규칙, 인계 상태, 완료
def test_normal_flow_order_and_completion():
    flow = Flow()
    run_until(flow, 'dst_unlock')

    ops = flow.operations()
    assert [op for _, op in ops] == [
        'NAVIGATE_TO', 'NAVIGATE_TO', 'LOCK_HANDOFF', 'LOCK_HANDOFF', 'PICK_AND_STOW',
        'CONFIRM_LOAD', 'RETRACT_ARM', 'UNLOCK_HANDOFF', 'UNLOCK_HANDOFF',
        'NAVIGATE_TO', 'NAVIGATE_TO', 'LOCK_HANDOFF', 'LOCK_HANDOFF', 'UNLOAD_AND_PLACE',
        'RETRACT_ARM', 'CONFIRM_LOAD', 'UNLOCK_HANDOFF', 'UNLOCK_HANDOFF']
    assert flow.state.task_state == gm.TASK_COMPLETED
    assert flow.state.transfer_state == gm.TRANSFER_UNLOCKED
    assert flow.state.transfer_id == flow.spec.destination_transfer_id
    assert flow.state.inhibit == []

    # 실행 ID는 D-03 규칙, attempt_index 0, 진행으로 task_revision이 바뀌지 않음
    for intent in flow.intents:
        ex = intent.execution
        assert ex['execution_id'] == make_execution_id(
            flow.spec.root_execution_id, intent.step_key, intent.operation, 0)
        assert ex['attempt_index'] == 0
        assert ex['task_revision'] == flow.spec.task_revision
        assert ex['derive_rule_version'] == 'derive-v1'

    assert flow.state.revision > 10


# 인계 상태 전이: 잠금 확인 → 적재 중 → 해제 → 재잠금
def test_transfer_state_sequence():
    flow = Flow()
    seen = []
    original = flow.apply

    # 사건마다 인계 상태 변화를 기록한다
    def tracking(event):
        decision = original(event)
        pair = (decision.next_state.transfer_state, decision.next_state.transfer_id)
        if not seen or seen[-1] != pair:
            seen.append(pair)
        return decision

    flow.apply = tracking
    run_until(flow, 'dst_unlock')
    src, dst = flow.spec.source_transfer_id, flow.spec.destination_transfer_id
    # 잠금 확인과 적재 의도가 같은 사건에서 일어나므로 사건 단위로는 LOCKED가 바로 IN_PROGRESS가 된다
    assert seen == [
        (gm.TRANSFER_NONE, ''),
        (gm.TRANSFER_IN_PROGRESS, src), (gm.TRANSFER_UNLOCKED, src),
        (gm.TRANSFER_IN_PROGRESS, dst), (gm.TRANSFER_UNLOCKED, dst)]


# 두 잠금 결과·정지 확인 뒤 LOCKED. 보류 중이면 적재 의도 없이 LOCKED로 남는다
def test_locked_state_visible_when_pick_blocked():
    flow = Flow()
    run_until(flow, 'src_nav')
    flow.send_all()
    flow.apply(HoldApplied('hold_lock'))
    flow.evidence('stopped')
    flow.finish('src_lock_manipulator')
    assert flow.state.transfer_state == gm.TRANSFER_NONE

    flow.finish('src_lock_transporter')
    assert flow.state.transfer_state == gm.TRANSFER_LOCKED
    assert flow.state.transfer_id == flow.spec.source_transfer_id
    assert not flow.intended('load_pick_and_stow')


# robot_id를 코드에 고정하지 않는다: 임의 이름으로도 같은 흐름
def test_robot_ids_not_hardcoded():
    flow = Flow('robot_zeta', 'robot_alpha')
    run_until(flow, 'dst_unlock')
    assert flow.state.task_state == gm.TASK_COMPLETED

    lock = [i for i in flow.intents if i.step_key == 'src_lock_transporter'][0]
    assert lock.robot_id == 'robot_alpha'
    assert lock.peer_robot_id == 'robot_zeta'


# 주행 결과만으로 잠금 의도를 내지 않는다: 두 로봇 실제 정지 필요
def test_lock_requires_both_stopped():
    flow = Flow()
    flow.start()
    flow.send_all()
    flow.finish('src_nav_manipulator')
    flow.finish('src_nav_transporter')
    assert not flow.intended('src_lock_manipulator')

    m, _ = flow.assignment.robots()
    flow.observe('ACTUATION', m, gm.ACTUATION_STOPPED)
    assert not flow.intended('src_lock_manipulator')

    flow.evidence('stopped')
    assert flow.intended('src_lock_manipulator')
    assert flow.intended('src_lock_transporter')


# 정지 관측의 허가 필드는 근거가 아니다: STOPPED가 아닌 상태는 drive_allowed와 무관하게 불충분
def test_actuation_permit_is_not_stop_evidence():
    flow = Flow()
    flow.start()
    flow.send_all()
    flow.finish('src_nav_manipulator')
    flow.finish('src_nav_transporter')
    m, t = flow.assignment.robots()
    for robot in (m, t):
        flow.apply(ObservationReceived(replace(
            flow.source.make('ACTUATION', robot, 1, flow.now()),
            permits=(('drive_allowed', False), ('arm_allowed', False)))))
    assert not flow.intended('src_lock_manipulator')


# 적재 STOWED여도 해제·후퇴·정지 근거가 없으면 해제·운송 의도 없음
@pytest.mark.parametrize('missing', ['released', 'retracted', 'stopped'])
def test_stowed_without_release_retract_stop_has_no_transport(missing):
    flow = Flow()
    run_until(flow, 'confirm')
    flow.send_all()
    flow.wait(0.05)
    present = [n for n in ('stowed', 'released', 'retracted', 'stopped') if n != missing]
    flow.evidence(*present)
    flow.finish('load_retract')

    assert not flow.intended('src_unlock_manipulator')
    assert not any(op == 'NAVIGATE_TO' and k.startswith('dst') for k, op in flow.operations())

    # 보내지 않은 해제 단계 결과를 넣어도 결합되지 않는다
    decision = flow.finish('src_unlock_manipulator')
    assert 'UNBOUND_RESULT' in decision.reasons
    assert flow.state.transfer_state == gm.TRANSFER_IN_PROGRESS


# 해제 결과 SUCCEEDED만으로 UNLOCKED가 되지 않는다: 그 시점의 최신 근거 필요
def test_unlock_results_with_stale_evidence_do_not_unlock():
    flow = Flow()
    run_until(flow, 'retract')
    flow.send_all()
    flow.wait(1.0)
    flow.finish('src_unlock_manipulator')
    flow.finish('src_unlock_transporter')
    assert flow.state.transfer_state == gm.TRANSFER_IN_PROGRESS
    assert not flow.intended('dst_nav_transporter')

    flow.evidence('stowed', 'released', 'retracted', 'stopped')
    assert flow.state.transfer_state == gm.TRANSFER_UNLOCKED
    assert flow.intended('dst_nav_transporter')


# 목적지 배치·EMPTY 근거가 없으면 완료하지 않는다
@pytest.mark.parametrize('missing', ['placed', 'empty', 'released', 'retracted'])
def test_no_completion_without_destination_evidence(missing):
    flow = Flow()
    run_until(flow, 'unload_retract')
    flow.send_all()
    flow.wait(0.05)
    flow.finish('unload_confirm_empty', load=gm.LOAD_EMPTY)
    present = [n for n in ('empty', 'placed', 'released', 'retracted', 'stopped')
               if n != missing]
    flow.evidence(*present)
    if flow.unsent():
        flow.send_all()
        flow.finish('dst_unlock_manipulator')
        flow.finish('dst_unlock_transporter')

    assert flow.state.task_state == gm.TASK_RUNNING
    assert flow.state.transfer_state != gm.TRANSFER_UNLOCKED


# CONFIRM_LOAD SUCCEEDED라도 결과 load_state가 STOWED가 아니면 진행 금지
@pytest.mark.parametrize('load,reason', [(gm.LOAD_UNKNOWN, 'LOAD_UNKNOWN'),
                                         (gm.LOAD_LOST, 'LOAD_LOST'),
                                         (gm.LOAD_EMPTY, 'EVIDENCE_CONFLICT')])
def test_confirm_load_success_without_stowed(load, reason):
    flow = Flow()
    run_until(flow, 'pick')
    flow.send_all()
    flow.evidence('stowed')
    flow.finish('load_confirm', load=load)
    assert reason in flow.state.inhibit
    assert not flow.intended('load_retract')


# 결과는 STOWED지만 관측이 EMPTY면 모순으로 막는다
def test_confirm_result_conflicts_with_observation():
    flow = Flow()
    run_until(flow, 'pick')
    flow.send_all()
    flow.evidence('empty')
    flow.finish('load_confirm', load=gm.LOAD_STOWED)
    assert 'EVIDENCE_CONFLICT' in flow.state.inhibit
    assert not flow.intended('load_retract')


# 보유 HELD와 적재 STOWED가 동시에 최신이면 모순
def test_held_and_stowed_conflict():
    flow = Flow()
    run_until(flow, 'confirm')
    flow.send_all()
    flow.evidence('stowed', 'held', 'retracted', 'stopped')
    flow.finish('load_retract')
    assert 'EVIDENCE_CONFLICT' in flow.state.inhibit
    assert not flow.intended('src_unlock_manipulator')


# 운송 중 적재 LOST·UNKNOWN은 보류하고 목적지 잠금을 막는다
@pytest.mark.parametrize('value,reason', [(gm.LOAD_LOST, 'LOAD_LOST'),
                                          (gm.LOAD_UNKNOWN, 'LOAD_UNKNOWN')])
def test_load_lost_or_unknown_during_transport(value, reason):
    flow = Flow()
    run_until(flow, 'src_unlock')
    _, t = flow.assignment.robots()
    flow.observe('LOAD', t, value, object_id=flow.spec.object_id)
    assert reason in flow.state.inhibit
    flow.send_all()
    flow.finish('dst_nav_manipulator')
    flow.finish('dst_nav_transporter')
    flow.evidence('stopped')
    assert not flow.intended('dst_lock_manipulator')


# 무효·만료·미래·다른 물체·미배정 로봇·출처 거부 관측은 근거가 아니다
@pytest.mark.parametrize('kwargs,reason', [
    ({'valid': False}, None),
    ({'age_s': 0.5}, None),
    ({'age_s': -0.2}, 'OBSERVATION_FROM_FUTURE'),
    ({'provenance': gm.PROVENANCE_LIVE}, 'PROVENANCE_REJECTED'),
])
def test_invalid_observations_are_not_evidence(kwargs, reason):
    flow = Flow()
    flow.start()
    flow.send_all()
    flow.finish('src_nav_manipulator')
    flow.finish('src_nav_transporter')
    m, t = flow.assignment.robots()
    for robot in (m, t):
        decision = flow.observe('ACTUATION', robot, gm.ACTUATION_STOPPED, **kwargs)
        if reason:
            assert reason in decision.reasons
    assert not flow.intended('src_lock_manipulator')


# 다른 물체·배정 밖 로봇의 관측은 버린다
def test_other_object_and_unassigned_robot():
    flow = Flow()
    run_until(flow, 'pick')
    flow.send_all()
    flow.wait(0.05)
    _, t = flow.assignment.robots()
    d1 = flow.observe('LOAD', t, gm.LOAD_STOWED, object_id='box_other')
    d2 = flow.observe('ACTUATION', 'robot_unassigned', gm.ACTUATION_STOPPED)
    assert 'OBSERVATION_OBJECT_MISMATCH' in d1.reasons
    assert 'OBSERVATION_ROBOT_NOT_ASSIGNED' in d2.reasons
    flow.finish('load_confirm', load=gm.LOAD_STOWED)
    assert not flow.intended('load_retract')


# 오래된 관측 시간 경과로 만료되면 근거가 사라진다
def test_observation_expiry_by_receive_time():
    flow = Flow()
    flow.start()
    flow.send_all()
    flow.evidence('stopped')
    flow.wait(0.31)
    flow.finish('src_nav_manipulator')
    flow.finish('src_nav_transporter')
    assert not flow.intended('src_lock_manipulator')


# 같은 세션 revision 감소·은퇴한 세션 관측은 버린다
def test_session_and_revision_rules():
    flow = Flow()
    flow.start()
    flow.send_all()
    flow.finish('src_nav_manipulator')
    flow.finish('src_nav_transporter')
    m, t = flow.assignment.robots()

    flow.observe('ACTUATION', m, 1, revision=5)
    d = flow.observe('ACTUATION', m, gm.ACTUATION_STOPPED, revision=4)
    assert 'OBSERVATION_REVISION_REGRESSED' in d.reasons

    flow.observe('ACTUATION', m, 1, session_id='mock_session_2', revision=1)
    d = flow.observe('ACTUATION', m, gm.ACTUATION_STOPPED, revision=9)
    assert 'OBSERVATION_SESSION_RETIRED' in d.reasons

    flow.observe('ACTUATION', t, gm.ACTUATION_STOPPED)
    assert not flow.intended('src_lock_manipulator')


# 중복 결과는 새 효과가 없고, 다른 결과의 중복은 모순으로 막는다
def test_duplicate_and_conflicting_results():
    flow = Flow()
    flow.start()
    flow.send_all()
    flow.evidence('stopped')
    flow.finish('src_nav_manipulator')
    flow.finish('src_nav_transporter')
    count = len(flow.intents)

    d = flow.finish('src_nav_manipulator')
    assert 'DUPLICATE_RESULT' in d.reasons
    assert d.effects == ()

    d = flow.finish('src_nav_manipulator', outcome=gm.OUTCOME_FAILED)
    assert 'EVIDENCE_CONFLICT' in flow.state.inhibit
    assert d.effects == ()
    assert len(flow.intents) == count


# 다른 로봇이 보낸 결과·결합되지 않은 UUID는 버린다
def test_result_from_wrong_robot_or_unbound_uuid():
    flow = Flow()
    flow.start()
    flow.send_all()
    _, t = flow.assignment.robots()
    d = flow.finish('src_nav_manipulator', robot=t)
    assert 'RESULT_ROBOT_MISMATCH' in d.reasons
    assert flow.state.steps['src_nav_manipulator'].status == 'ACCEPTED'

    d = flow.apply(StepResult('uuid_unknown', t, gm.OUTCOME_SUCCEEDED, MOCK))
    assert 'UNBOUND_RESULT' in d.reasons


# 순서 역전: 수락 응답 전에 결과가 와도 UUID로 결합하고, 늦은 수락은 되돌리지 않는다
def test_result_before_goal_response():
    flow = Flow()
    flow.start()
    intent = flow.unsent()[0]
    uuid = f'uuid_{intent.step_key}'
    flow.apply(DispatchReserved(intent.execution['execution_id'], uuid))
    flow.apply(DispatchSent(intent.execution['execution_id']))
    flow.finish(intent.step_key)
    assert flow.state.steps[intent.step_key].status == 'SUCCEEDED'
    d = flow.apply(GoalResponse(uuid, True, MOCK))
    assert 'LATE_GOAL_RESPONSE' in d.reasons
    assert flow.state.steps[intent.step_key].status == 'SUCCEEDED'


# RobotTaskStatus: 이전 배정 세대는 무시, 같은 실행의 실패 보고는 진행 금지
def test_task_status_binding():
    flow = Flow()
    flow.start()
    flow.send_all()
    intent = [i for i in flow.intents if i.step_key == 'src_nav_manipulator'][0]
    m, _ = flow.assignment.robots()

    old = dict(intent.execution, assignment_generation=0)
    d = flow.apply(TaskStatusSeen(m, 's1', 1, old, 'NAVIGATING', gm.OUTCOME_FAILED, MOCK))
    assert 'STATUS_STALE_ASSIGNMENT' in d.reasons
    assert flow.state.inhibit == []

    other = dict(intent.execution, execution_id='0' * 32)
    d = flow.apply(TaskStatusSeen(m, 's1', 2, other, 'X', gm.OUTCOME_FAILED, MOCK))
    assert 'STATUS_UNKNOWN_EXECUTION' in d.reasons

    # 상태 보고의 SUCCEEDED는 Action 결과를 대신하지 않는다
    d = flow.apply(TaskStatusSeen(m, 's1', 3, intent.execution, 'DONE',
                                  gm.OUTCOME_SUCCEEDED, MOCK))
    assert flow.state.steps['src_nav_manipulator'].status == 'ACCEPTED'

    d = flow.apply(TaskStatusSeen(m, 's1', 4, intent.execution, 'X',
                                  gm.OUTCOME_FAILED, MOCK))
    assert flow.state.steps['src_nav_manipulator'].status == 'UNCERTAIN'
    assert 'STATUS_REPORTS_FAILURE' in flow.state.inhibit


# 배정 세대는 증가해야 하고, 실행 시작 뒤 재배정은 W3에서 지원하지 않는다
def test_assignment_generation_rules():
    flow = Flow()
    flow.apply(AssignRoles(flow.assignment))
    d = flow.apply(AssignRoles(make_assignment('robot_m7', 'robot_t3', generation=1,
                                               assignment_id='asg_other')))
    assert 'ASSIGNMENT_GENERATION_NOT_INCREASED' in d.reasons

    newer = make_assignment('robot_m8', 'robot_t3', generation=2)
    flow.apply(AssignRoles(newer))
    assert flow.state.assignment == newer
    flow.assignment = newer
    flow.apply(StartTask())
    d = flow.apply(AssignRoles(make_assignment('robot_m9', 'robot_t3', generation=3)))
    assert 'REASSIGNMENT_UNSUPPORTED' in d.reasons
    assert flow.state.assignment == newer


# 실패는 잠금 해제·재시도가 아니다. 명시 실패 종료 뒤에도 인계 상태 유지
def test_failure_keeps_lock_and_does_not_retry():
    flow = Flow()
    run_until(flow, 'src_lock')
    flow.send_all()
    assert flow.state.transfer_state == gm.TRANSFER_IN_PROGRESS
    flow.finish('load_pick_and_stow', outcome=gm.OUTCOME_FAILED)

    assert 'STEP_FAILED' in flow.state.inhibit
    assert flow.state.task_state == gm.TASK_RUNNING
    assert flow.state.transfer_state == gm.TRANSFER_IN_PROGRESS
    count = len(flow.intents)
    flow.evidence('stowed', 'stopped')
    flow.apply(Tick())
    assert len(flow.intents) == count
    assert flow.state.steps['load_pick_and_stow'].attempt_index == 0

    d = flow.apply(FailTerminate('fail_evt_1'))
    assert flow.state.task_state == gm.TASK_FAILED
    assert flow.state.transfer_state == gm.TRANSFER_IN_PROGRESS
    assert flow.state.failure_event_id == 'fail_evt_1'
    assert d.effects == ()


# 진행 중 단계가 있으면 실패 종료를 거부한다(먼저 종료 확인 필요)
def test_fail_terminate_rejected_with_inflight():
    flow = Flow()
    flow.start()
    flow.send_all()
    d = flow.apply(FailTerminate('fail_evt_2'))
    assert 'INFLIGHT_STEPS' in d.reasons
    assert flow.state.task_state == gm.TASK_RUNNING


# 취소 ACK만으로 CANCELED가 아니다. 하위 종료 후 CANCELED, 인계 잠금은 유지
def test_cancel_requires_terminal_results_and_keeps_lock():
    flow = Flow()
    run_until(flow, 'src_lock')
    flow.send_all()
    d = flow.apply(CancelRequested())
    cancels = [e for e in d.effects if e.kind == 'CANCEL']
    assert [c.step_key for c in cancels] == ['load_pick_and_stow']
    assert cancels[0].goal_uuid == 'uuid_load_pick_and_stow'

    flow.apply(CancelAcknowledged('uuid_load_pick_and_stow'))
    assert flow.state.task_state == gm.TASK_RUNNING

    flow.finish('load_pick_and_stow', outcome=gm.OUTCOME_CANCELED)
    assert flow.state.task_state == gm.TASK_RUNNING
    flow.wait(0.05)
    flow.evidence('stopped')
    assert flow.state.task_state == gm.TASK_CANCELED
    assert flow.state.transfer_state == gm.TRANSFER_IN_PROGRESS
    assert not flow.intended('load_confirm')


# 전송이 전혀 없던 작업의 취소는 물리 근거 없이 CANCELED(근거를 만들지 않음)
@pytest.mark.parametrize('started', [False, True])
def test_cancel_without_any_dispatch(started):
    flow = Flow()
    flow.apply(AssignRoles(flow.assignment))
    if started:
        flow.apply(StartTask())
        assert flow.unsent()
    flow.apply(CancelRequested())
    assert flow.state.task_state == gm.TASK_CANCELED
    assert flow.state.observations == {}
    assert all(run.goal_uuid == '' for run in flow.state.steps.values())


# 전송 후 취소: 하위 종료 뒤 받은 두 로봇 최신 STOPPED가 있어야 CANCELED
@pytest.mark.parametrize('case', ['no_observation', 'moving', 'expired', 'stopped_before_cancel',
                                  'one_robot', 'uncertain_left', 'invalid', 'provenance_rejected',
                                  'same_instant_as_result'])
def test_cancel_after_dispatch_requires_fresh_stop(case):
    flow = Flow()
    flow.start()
    flow.send_all()
    flow.apply(HoldApplied('hold_c'))
    m, t = flow.assignment.robots()
    if case == 'stopped_before_cancel':
        flow.evidence('stopped')
    flow.wait(0.05)
    flow.apply(CancelRequested())
    flow.finish('src_nav_manipulator', outcome=gm.OUTCOME_CANCELED)
    if case == 'uncertain_left':
        flow.wait(gm_limits().result_timeout_s + 1.0)
        flow.apply(Tick())
    else:
        flow.finish('src_nav_transporter', outcome=gm.OUTCOME_CANCELED)
    if case != 'same_instant_as_result':
        flow.wait(0.05)

    if case == 'moving':
        for robot in (m, t):
            flow.observe('ACTUATION', robot, 1)
    elif case == 'expired':
        for robot in (m, t):
            flow.observe('ACTUATION', robot, gm.ACTUATION_STOPPED, age_s=0.5)
    elif case == 'one_robot':
        flow.observe('ACTUATION', m, gm.ACTUATION_STOPPED)
    elif case in ('uncertain_left', 'same_instant_as_result'):
        flow.evidence('stopped')
    elif case == 'invalid':
        for robot in (m, t):
            flow.observe('ACTUATION', robot, gm.ACTUATION_STOPPED, valid=False)
    elif case == 'provenance_rejected':
        for robot in (m, t):
            flow.observe('ACTUATION', robot, gm.ACTUATION_STOPPED,
                         provenance=gm.PROVENANCE_LIVE)
    flow.apply(Tick())

    assert flow.state.task_state == gm.TASK_RUNNING
    assert flow.state.cancel_requested
    assert flow.state.hold_ids == ['hold_c']


# 하위가 모두 끝난 뒤(실패) 받은 정지라도 취소 요청 전이면 근거가 아니다. 요청 뒤 정지 필요
def test_cancel_needs_stop_after_request_when_steps_already_ended():
    flow = Flow()
    run_until(flow, 'src_lock')
    flow.send_all()
    flow.finish('load_pick_and_stow', outcome=gm.OUTCOME_FAILED)
    flow.wait(0.05)
    flow.evidence('stopped')
    flow.wait(0.05)
    flow.apply(CancelRequested())
    assert flow.state.task_state == gm.TASK_RUNNING

    flow.wait(0.05)
    flow.evidence('stopped')
    assert flow.state.task_state == gm.TASK_CANCELED
    assert flow.state.transfer_state == gm.TRANSFER_IN_PROGRESS


# 종료 뒤 양쪽 최신 정지면 CANCELED. inhibit·hold·인계 상태는 그대로 보존
def test_cancel_completes_with_fresh_stop_preserving_state():
    flow = Flow()
    run_until(flow, 'src_lock')
    flow.send_all()
    flow.apply(HoldApplied('hold_c'))
    flow.apply(CancelRequested())
    flow.finish('load_pick_and_stow', outcome=gm.OUTCOME_FAILED)
    assert flow.state.task_state == gm.TASK_RUNNING
    flow.wait(0.05)
    flow.evidence('stopped')
    assert flow.state.task_state == gm.TASK_CANCELED
    assert flow.state.transfer_state == gm.TRANSFER_IN_PROGRESS
    assert flow.state.hold_ids == ['hold_c']
    assert 'STEP_FAILED' in flow.state.inhibit


# 결과 미확정 단계가 있으면 취소 완료로 보지 않는다
def test_cancel_with_uncertain_step_stays_running():
    flow = Flow()
    flow.start()
    flow.send_all()
    flow.wait(1000.0)
    flow.apply(Tick())
    flow.apply(CancelRequested())
    assert flow.state.task_state == gm.TASK_RUNNING


# 결과 시간초과는 미확정이며 재전송하지 않는다. 늦은 성공도 진행 근거가 아니다
def test_result_timeout_is_uncertain_without_resend():
    flow = Flow()
    flow.start()
    flow.send_all()
    count = len(flow.intents)
    flow.wait(gm_limits().result_timeout_s + 1.0)
    d = flow.apply(Tick())
    assert flow.state.steps['src_nav_manipulator'].status == 'UNCERTAIN'
    assert 'RESULT_TIMEOUT' in flow.state.inhibit
    assert d.effects == ()

    flow.evidence('stopped')
    d = flow.finish('src_nav_manipulator')
    flow.finish('src_nav_transporter')
    assert 'LATE_RESULT_RECORDED' in d.reasons
    assert len(flow.intents) == count


# 전송 실패: 불확실이면 UNCERTAIN, 확실한 미전송이면 NOT_SENT. 모두 재전송 없음
@pytest.mark.parametrize('uncertain,status,reason', [
    (True, 'UNCERTAIN', 'DISPATCH_UNCERTAIN'),
    (False, 'NOT_SENT', 'DISPATCH_FAILED')])
def test_dispatch_failure(uncertain, status, reason):
    flow = Flow()
    flow.start()
    intent = flow.unsent()[0]
    flow.apply(DispatchReserved(intent.execution['execution_id'], 'uuid_x'))
    d = flow.apply(DispatchFailed(intent.execution['execution_id'], uncertain))
    assert flow.state.steps[intent.step_key].status == status
    assert reason in flow.state.inhibit
    assert d.effects == ()


# Goal 거절은 진행 금지
def test_goal_rejected():
    flow = Flow()
    flow.start()
    intent = flow.unsent()[0]
    flow.apply(DispatchReserved(intent.execution['execution_id'], 'uuid_r'))
    flow.apply(GoalResponse('uuid_r', False, MOCK))
    assert flow.state.steps[intent.step_key].status == 'REJECTED'
    assert 'GOAL_REJECTED' in flow.state.inhibit


# 보류 ID는 새 의도를 막고, 해제는 미지원이다
def test_hold_blocks_and_release_unsupported():
    flow = Flow()
    flow.start()
    flow.send_all()
    flow.apply(HoldApplied('hold_1'))
    flow.evidence('stopped')
    flow.finish('src_nav_manipulator')
    flow.finish('src_nav_transporter')
    assert not flow.intended('src_lock_manipulator')

    d = flow.apply(HoldReleaseRequested('hold_1'))
    assert 'HOLD_RELEASE_UNSUPPORTED' in d.reasons
    assert flow.state.hold_ids == ['hold_1']


# 원장 쓰기 실패는 새 명령을 막는다
def test_store_fault_blocks():
    flow = Flow()
    flow.apply(AssignRoles(flow.assignment))
    flow.apply(StoreFault('WRITE_FAILED'))
    d = flow.apply(StartTask())
    assert 'STORE_FAULT' in flow.state.inhibit
    assert d.effects == ()


# 시간 유효성 없음은 새 의도를 막고, sim time 역행은 지속 차단
def test_clock_invalid_and_reversed():
    flow = Flow()
    flow.apply(AssignRoles(flow.assignment))
    d = apply_event(flow.state, StartTask(), clock(flow.t, time_valid=False))
    assert d.effects == ()
    assert 'TIME_INVALID' in d.reasons

    flow.state = d.next_state
    d = apply_event(flow.state, Tick(), clock(flow.t + 1, sim_time_s=flow.t - 10))
    assert 'CLOCK_REVERSED' in d.next_state.inhibit
    flow.state = d.next_state
    d = flow.apply(Tick())
    assert d.effects == ()


# 복원 상태는 관측을 버리고 진행 중 단계를 미확정으로 두며 새 명령을 막는다
def test_restore_blocks_until_reconciled():
    flow = Flow()
    run_until(flow, 'src_lock')
    flow.send_all()
    restored = restore_state(flow.state)
    assert restored.observations == {}
    assert restored.steps['load_pick_and_stow'].status == 'UNCERTAIN'
    assert 'RESTORED_UNRECONCILED' in restored.inhibit
    assert restored.transfer_state == gm.TRANSFER_IN_PROGRESS

    flow.state = restored
    flow.evidence('stowed', 'stopped')
    flow.finish('load_pick_and_stow')
    assert not flow.intended('load_confirm')


# 운영 모드는 생성 단계에서 거부하고, 운영 모드에서는 MOCK 근거를 받지 않는다
def test_operational_mode_unsupported():
    with pytest.raises(gm.SpecError) as error:
        gm.new_group_state(make_spec(), mode=gm.MODE_OPERATIONAL)
    assert error.value.code == 'OPERATIONAL_MODE_UNSUPPORTED'
    assert not gm.provenance_acceptable(gm.MODE_OPERATIONAL, gm.PROVENANCE_MOCK)
    assert gm.provenance_acceptable(gm.MODE_DEVELOPMENT_MOCK, gm.PROVENANCE_MOCK)
    assert not gm.provenance_acceptable(gm.MODE_DEVELOPMENT_MOCK, gm.PROVENANCE_LIVE)


# 입력 상태를 바꾸지 않는다(순수 함수)
def test_apply_event_does_not_mutate_input():
    flow = Flow()
    before = flow.state.copy()
    apply_event(flow.state, AssignRoles(flow.assignment), flow.now())
    assert flow.state == before


# 단계 계획 변형 헬퍼
def mutated_steps(change):
    steps = spec_dict()['steps']
    change(steps)
    return steps


# 입력 schema·계획 검증: 빠짐·추가·순환·미지원 operation·역할 불일치 등
@pytest.mark.parametrize('overrides,code', [
    ({'priority': 0}, 'INVALID_PRIORITY'),
    ({'task_revision': 0}, 'INVALID_NUMBER'),
    ({'map_revision': -1}, 'INVALID_NUMBER'),
    ({'map_revision': True}, 'INVALID_NUMBER'),
    ({'object_id': ''}, 'INVALID_ID'),
    ({'destination_location_id': 'station_src'}, 'SAME_SOURCE_DESTINATION'),
    ({'destination_transfer_id': 'task_dev_1_xfer_src'}, 'DUPLICATE_TRANSFER_ID'),
    ({'steps': []}, 'PLAN_EMPTY'),
    ({'steps': mutated_steps(lambda s: s[0].update(operation='FLY'))},
     'OPERATION_NOT_ALLOWED'),
    ({'steps': mutated_steps(lambda s: s[0].update(operation='REQUEST_CORRIDOR'))},
     'OPERATION_UNSUPPORTED'),
    ({'steps': mutated_steps(lambda s: s[4].update(robot_role='TRANSPORTER'))},
     'ROLE_OPERATION_MISMATCH'),
    ({'steps': mutated_steps(lambda s: s[0].update(depends_on=['nope']))},
     'UNKNOWN_DEPENDENCY'),
    ({'steps': mutated_steps(lambda s: s[0].update(depends_on=['load_confirm']))},
     'PLAN_CYCLE'),
    ({'steps': mutated_steps(lambda s: s[1].update(step_key=s[0]['step_key']))},
     'DUPLICATE_STEP_KEY'),
    ({'steps': mutated_steps(lambda s: [x.update(terminal=False) for x in s])},
     'NO_TERMINAL_STEP'),
    ({'steps': mutated_steps(lambda s: s[5].update(depends_on=[]))},
     'PLAN_SHAPE_UNSUPPORTED'),
    ({'steps': mutated_steps(lambda s: s.extend([dict(s[0], step_key=f'x{i}')
                                                 for i in range(40)]))},
     'PLAN_TOO_LARGE'),
])
def test_spec_validation(overrides, code):
    with pytest.raises(gm.SpecError) as error:
        gm.parse_spec(spec_dict(**overrides))
    assert error.value.code == code


# 입력 사전 키 빠짐·추가 거부
def test_spec_schema_exact_keys():
    data = spec_dict()
    del data['map_revision']
    with pytest.raises(gm.SpecError, match='INVALID_SCHEMA'):
        gm.parse_spec(data)

    data = spec_dict(target_pose=[0, 0, 0])
    with pytest.raises(gm.SpecError, match='INVALID_SCHEMA'):
        gm.parse_spec(data)


# 배정 검증: 같은 로봇 두 역할 금지, 세대 0 금지, 형식
def test_assignment_validation():
    with pytest.raises(gm.SpecError, match='SAME_ROBOT_FOR_BOTH_ROLES'):
        make_assignment('robot_a', 'robot_a')

    with pytest.raises(gm.SpecError, match='INVALID_NUMBER'):
        make_assignment('robot_a', 'robot_b', generation=0)

    with pytest.raises(gm.SpecError, match='INVALID_ID'):
        make_assignment('Robot A', 'robot_b')

    with pytest.raises(gm.SpecError, match='INVALID_SCHEMA'):
        gm.parse_assignment({'assignment_id': 'a'})


# 여러 작업 사이: 중복 ID·물체·로봇·배정 ID·지도/지점 revision 불일치 거부
def test_registry_admission_and_mutual_exclusion():
    reg = GroupRegistry(map_revision=1, locations={'station_src': 1, 'station_dst': 1,
                                                   'station_b': 1})
    now = clock(10.0)
    reg.admit(make_spec('task_a', 'box_a'))

    for spec, code in [
        (make_spec('task_a', 'box_z'), 'DUPLICATE_TASK_ID'),
        (make_spec('task_b', 'box_a'), 'OBJECT_IN_ACTIVE_TASK'),
        (make_spec('task_c', 'box_c', root_execution_id='root_task_a'),
         'DUPLICATE_ROOT_EXECUTION_ID'),
        (make_spec('task_d', 'box_d', source_transfer_id='task_a_xfer_src'),
         'DUPLICATE_TRANSFER_ID'),
        (make_spec('task_e', 'box_e', map_revision=2), 'MAP_REVISION_MISMATCH'),
        (make_spec('task_f', 'box_f', source_location_revision=3),
         'LOCATION_REVISION_MISMATCH'),
        (make_spec('task_g', 'box_g', destination_location_id='station_unknown'),
         'UNKNOWN_LOCATION'),
    ]:
        with pytest.raises(gm.SpecError) as error:
            reg.admit(spec)
        assert error.value.code == code, spec.task_id

    reg.admit(make_spec('task_b', 'box_b', destination_location_id='station_b'))
    reg.apply('task_a', AssignRoles(make_assignment('robot_p', 'robot_q')), now)

    d = reg.apply('task_b', AssignRoles(make_assignment('robot_r', 'robot_q')), now)
    assert 'ROBOT_IN_ACTIVE_TASK' in d.reasons
    assert reg.states['task_b'].assignment is None

    d = reg.apply('task_b', AssignRoles(make_assignment(
        'robot_r', 'robot_s', assignment_id='asg_robot_p_robot_q_1')), now)
    assert 'DUPLICATE_ASSIGNMENT_ID' in d.reasons


# 실패 종료됐어도 잠금이 남은 작업의 로봇은 다른 작업에 배정하지 않는다
def test_registry_locked_terminal_task_keeps_robots_busy():
    reg = GroupRegistry(map_revision=1, locations={'station_src': 1, 'station_dst': 1})
    flow = Flow('robot_p', 'robot_q', spec=make_spec('task_a', 'box_a'))
    run_until(flow, 'src_lock')
    flow.send_all()
    flow.finish('load_pick_and_stow', outcome=gm.OUTCOME_FAILED)
    flow.apply(FailTerminate('fail_1'))
    reg.restore([flow.state])

    reg.admit(make_spec('task_b', 'box_b'))
    d = reg.apply('task_b', AssignRoles(make_assignment('robot_p', 'robot_z')), clock(500.0))
    assert 'ROBOT_IN_ACTIVE_TASK' in d.reasons

    # 잠금이 남은 실패 작업의 물체도 새 작업에 쓰지 않는다
    with pytest.raises(gm.SpecError, match='OBJECT_IN_ACTIVE_TASK'):
        reg.admit(make_spec('task_c', 'box_a'))


# 작업 상한
def test_registry_task_limit():
    reg = GroupRegistry(map_revision=1, locations={'station_src': 1, 'station_dst': 1},
                        max_tasks=2)
    reg.admit(make_spec('task_a', 'box_a'))
    reg.admit(make_spec('task_b', 'box_b'))
    with pytest.raises(gm.SpecError, match='TASK_LIMIT'):
        reg.admit(make_spec('task_c', 'box_c'))


# 시험 설정 상한(엔진 기본값)
def gm_limits():
    from ax_fleet_manager.group_machine import EngineLimits
    return EngineLimits()
