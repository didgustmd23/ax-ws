"""D2 runtime + 모의 상대 포트 시나리오: 정상 운반, 동시 작업, 부분 성공, 취소, 재시작.

모의 상대(MockCounterpart)는 시험 코드 안의 개발 흉내이며 실제 A3·C4·D1 동작이 아니다.
실행: .venv/bin/python -m pytest -q -p no:launch_testing -p no:launch_ros \
      tools/tests/test_d2_workflow_scenarios.py
"""
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src' / 'ax_fleet_manager'))

from ax_fleet_manager import group_model as gm  # noqa: E402
from ax_fleet_manager.dev_fixtures import (  # noqa: E402
    MockObservationSource, clock, make_assignment, make_spec,
)
from ax_fleet_manager.group_events import (  # noqa: E402
    AssignRoles, CancelRequested, FailTerminate, GoalResponse, ObservationReceived,
    StartTask, StepResult,
)
from ax_fleet_manager.ports import MockLocalTaskPort, MockNavigationPort  # noqa: E402
from ax_fleet_manager.runtime import GroupRuntime  # noqa: E402
from ax_fleet_manager.snapshot import task_list_fields  # noqa: E402
from ax_fleet_manager.task_store import TaskStore  # noqa: E402

MOCK = gm.PROVENANCE_MOCK
LOCATIONS = {'station_src': 1, 'station_dst': 1, 'station_b': 1, 'station_c': 1}

# 단계별 모의 상대가 결과 전에 내는 관측(이름은 MockCounterpart.publish 참조)
EVIDENCE = {
    'src_nav_manipulator': ['stopped'], 'src_nav_transporter': ['stopped'],
    'src_lock_manipulator': ['stopped'], 'src_lock_transporter': ['stopped'],
    'load_pick_and_stow': ['stowed'], 'load_confirm': ['stowed'],
    'load_retract': ['stowed', 'released', 'retracted', 'stopped'],
    'src_unlock_manipulator': ['stowed', 'released', 'retracted', 'stopped'],
    'src_unlock_transporter': ['stowed', 'released', 'retracted', 'stopped'],
    'dst_nav_manipulator': ['stopped', 'stowed'], 'dst_nav_transporter': ['stopped', 'stowed'],
    'dst_lock_manipulator': ['stopped', 'stowed'], 'dst_lock_transporter': ['stopped', 'stowed'],
    'unload_place': ['placed'], 'unload_retract': ['placed', 'empty'],
    'unload_confirm_empty': ['empty', 'placed', 'released', 'retracted', 'stopped'],
    'dst_unlock_manipulator': ['empty', 'placed', 'released', 'retracted', 'stopped'],
    'dst_unlock_transporter': ['empty', 'placed', 'released', 'retracted', 'stopped'],
}
LOAD_RESULT = {'load_confirm': gm.LOAD_STOWED, 'unload_confirm_empty': gm.LOAD_EMPTY}


class Clock:
    """주입 시각(테스트가 진행시킨다)."""

    # 시작 시각
    def __init__(self):
        self.t = 200.0

    # 현재 시각
    def __call__(self):
        return clock(self.t)


class MockCounterpart:
    """모의 A3·D1 상대: 받은 요청에 수락·관측·결과를 돌려준다."""

    # runtime과 포트를 연결한다
    def __init__(self, runtime, local, nav, clk):
        self.runtime = runtime
        self.ports = (local, nav)
        self.clock = clk
        self.done = set()
        self.source = MockObservationSource('d2_mock_counterpart')
        self.overrides = {}

    # 작업의 배정 로봇·물체로 관측을 낸다
    def publish(self, task_id, names):
        state = self.runtime.registry.states[task_id]
        m, t = state.assignment.robots()
        obj = state.spec.object_id
        table = {
            'stopped': [('ACTUATION', m, gm.ACTUATION_STOPPED, ''),
                        ('ACTUATION', t, gm.ACTUATION_STOPPED, '')],
            'stowed': [('LOAD', t, gm.LOAD_STOWED, obj)],
            'empty': [('LOAD', t, gm.LOAD_EMPTY, obj)],
            'released': [('HOLD', m, gm.HOLD_RELEASED, obj)],
            'retracted': [('ARM_RETRACTED', m, 1, '')],
            'placed': [('PLACEMENT', m, 1, obj)],
        }
        for name in names:
            for kind, robot, value, object_id in table[name]:
                self.runtime.handle(task_id, ObservationReceived(self.source.make(
                    kind, robot, value, self.clock(), object_id=object_id)))

    # 아직 답하지 않은 요청에 응답한다. 한 번 호출로 지금 있는 요청만 처리한다
    def respond(self):
        pending = [r for port in self.ports for r in port.requests
                   if r['goal_uuid'] not in self.done]
        for request in pending:
            self.done.add(request['goal_uuid'])
            task_id = request['fields']['execution']['task_id']
            key = request['step_key']
            outcome = self.overrides.get((task_id, key), gm.OUTCOME_SUCCEEDED)
            self.clock.t += 0.02
            self.runtime.handle(task_id, GoalResponse(request['goal_uuid'], True, MOCK))
            if outcome is None:
                continue
            self.publish(task_id, EVIDENCE[key])
            self.runtime.handle(task_id, StepResult(
                request['goal_uuid'], request['robot_id'], outcome, MOCK, '',
                gm.HOLD_RELEASED if key == 'load_retract' else 0, LOAD_RESULT.get(key, 0)))

        return len(pending)

    # 새 요청이 없을 때까지 응답한다
    def run(self, limit=100):
        for _ in range(limit):
            if not self.respond():
                return
        raise AssertionError('응답 반복 상한 초과')


# runtime·모의 상대 묶음
def make_scenario(tmp_path, name='d2.sqlite'):
    clk = Clock()
    local, nav = MockLocalTaskPort('scenario'), MockNavigationPort('scenario')
    store = TaskStore(str(tmp_path / name))
    runtime = GroupRuntime(store, {'LOCAL_TASK': local, 'NAVIGATION': nav}, clk,
                           map_revision=1, locations=LOCATIONS)
    return runtime, MockCounterpart(runtime, local, nav, clk), local, nav


# 작업 시작
def begin(runtime, task_id, object_id, manipulator, transporter, **spec_kw):
    runtime.admit(make_spec(task_id, object_id, **spec_kw))
    runtime.handle(task_id, AssignRoles(make_assignment(manipulator, transporter)))
    runtime.handle(task_id, StartTask())


# 정상 운반 흐름: runtime·원장·모의 포트 끝까지
def test_normal_transfer_through_runtime(tmp_path):
    runtime, mock, local, nav = make_scenario(tmp_path)
    begin(runtime, 'task_a', 'box_a', 'robot_m', 'robot_t')
    mock.run()

    state = runtime.registry.states['task_a']
    assert state.task_state == gm.TASK_COMPLETED
    assert state.transfer_state == gm.TRANSFER_UNLOCKED
    assert len(nav.requests) == 4 and len(local.requests) == 14
    assert local.cancels == [] and nav.cancels == []
    rows = runtime.store.dispatch_rows('task_a')
    assert len(rows) == 18 and {r['status'] for r in rows} == {'SUCCEEDED'}
    assert len({r['goal_uuid'] for r in rows}) == 18
    assert task_list_fields(runtime.states()) == []


# 동시 작업 두 개(로봇 겹치지 않음)는 서로 섞이지 않고 각각 완료, 겹치는 배정은 거부
def test_two_concurrent_tasks_mutually_exclusive(tmp_path):
    runtime, mock, local, _ = make_scenario(tmp_path)
    begin(runtime, 'task_a', 'box_a', 'robot_m', 'robot_t')
    begin(runtime, 'task_b', 'box_b', 'robot_n', 'robot_u',
          source_location_id='station_b', destination_location_id='station_c')

    runtime.admit(make_spec('task_c', 'box_c'))
    d = runtime.handle('task_c', AssignRoles(make_assignment('robot_x', 'robot_t')))
    assert 'ROBOT_IN_ACTIVE_TASK' in d.reasons

    mock.run()
    for task_id in ('task_a', 'task_b'):
        assert runtime.registry.states[task_id].task_state == gm.TASK_COMPLETED

    for request in local.requests:
        task_id = request['fields']['execution']['task_id']
        robots = runtime.registry.states[task_id].assignment.robots()
        assert request['robot_id'] in robots


# 부분 성공: 한쪽 잠금만 성공하면 LOCKED가 아니고 적재 의도 없음
def test_partial_lock_success(tmp_path):
    runtime, mock, local, _ = make_scenario(tmp_path)
    mock.overrides[('task_a', 'src_lock_transporter')] = gm.OUTCOME_FAILED
    begin(runtime, 'task_a', 'box_a', 'robot_m', 'robot_t')
    mock.run()

    state = runtime.registry.states['task_a']
    # W4: 한쪽 잠금이 이미 걸렸을 수 있으므로 NONE으로 돌아가지 않고 UNKNOWN(두 로봇 주행 금지)을
    # 유지한다. W3 기대값 NONE에서 강화(A4 HANDOFF_LOCK이 걸리도록)
    assert state.transfer_state == gm.TRANSFER_UNKNOWN
    assert state.transfer_id == state.spec.source_transfer_id
    assert 'STEP_FAILED' in state.inhibit
    assert not any(r['operation'] == 'PICK_AND_STOW' for r in local.requests)


# 적재 중 실패 → 실패 종료 뒤에도 잠금 작업은 목록에 남는다(A4가 계속 막음)
def test_failed_locked_task_stays_in_list(tmp_path):
    runtime, mock, _, _ = make_scenario(tmp_path)
    mock.overrides[('task_a', 'load_pick_and_stow')] = gm.OUTCOME_FAILED
    begin(runtime, 'task_a', 'box_a', 'robot_m', 'robot_t')
    mock.run()
    runtime.handle('task_a', FailTerminate('fail_evt_a'))

    listed = task_list_fields(runtime.states())
    assert len(listed) == 1
    item = listed[0]
    assert item['state'] == gm.TASK_FAILED
    assert item['transfer_state'] == gm.TRANSFER_IN_PROGRESS
    assert item['failure_event_id'] == 'fail_evt_a'
    assert (item['manipulator_robot_id'], item['transporter_robot_id']) == ('robot_m', 'robot_t')


# 결과가 오지 않은 단계(응답 실종) 중 취소: 취소 의도는 보내지만 결과 전에는 CANCELED 아님
def test_cancel_during_transport(tmp_path):
    runtime, mock, local, nav = make_scenario(tmp_path)
    mock.overrides[('task_a', 'dst_nav_transporter')] = None
    begin(runtime, 'task_a', 'box_a', 'robot_m', 'robot_t')
    mock.run()

    runtime.handle('task_a', CancelRequested())
    assert [c['step_key'] for c in nav.cancels] == ['dst_nav_transporter']
    assert runtime.registry.states['task_a'].task_state == gm.TASK_RUNNING

    uuid = nav.cancels[0]['goal_uuid']
    runtime.handle('task_a', StepResult(uuid, 'robot_t', gm.OUTCOME_CANCELED, MOCK))
    assert runtime.registry.states['task_a'].task_state == gm.TASK_RUNNING
    mock.clock.t += 0.05
    mock.publish('task_a', ['stopped'])
    state = runtime.registry.states['task_a']
    assert state.task_state == gm.TASK_CANCELED
    assert state.transfer_state == gm.TRANSFER_UNLOCKED
    assert len(nav.cancels) == 1


# 취소 요청·하위 종료 후 정지 확인 전 재시작: 취소 요청·시각이 복원되고 취소 재전송 없음,
# 복원 뒤 최신 두 로봇 정지로만 CANCELED, 복원 차단·인계 상태 보존.
# same_host: 같은 호스트(BOOTTIME 계속 증가), rewound: 재부팅처럼 시계가 취소 시각보다 앞이면 완료 불가
@pytest.mark.parametrize('clock_case', ['same_host', 'rewound'])
def test_restart_after_cancel_before_stop(tmp_path, clock_case):
    runtime, mock, _, nav = make_scenario(tmp_path)
    mock.overrides[('task_a', 'dst_nav_transporter')] = None
    begin(runtime, 'task_a', 'box_a', 'robot_m', 'robot_t')
    mock.run()
    runtime.handle('task_a', CancelRequested())
    runtime.handle('task_a', StepResult(nav.cancels[0]['goal_uuid'], 'robot_t',
                                        gm.OUTCOME_CANCELED, MOCK))
    assert runtime.registry.states['task_a'].task_state == gm.TASK_RUNNING
    runtime.store.close()

    last_t = mock.clock.t
    restarted, mock2, local2, nav2 = make_scenario(tmp_path)
    if clock_case == 'same_host':
        mock2.clock.t = last_t
    state = restarted.registry.states['task_a']
    assert state.cancel_requested and state.cancel_requested_boottime_s > 0
    assert state.task_state == gm.TASK_RUNNING
    assert nav2.cancels == [] and nav2.requests == [] and local2.requests == []

    mock2.clock.t += 0.05
    mock2.publish('task_a', ['stopped'])
    state = restarted.registry.states['task_a']
    if clock_case == 'rewound':
        assert state.task_state == gm.TASK_RUNNING
        return
    assert state.task_state == gm.TASK_CANCELED
    assert 'RESTORED_UNRECONCILED' in state.inhibit
    assert state.transfer_state == gm.TRANSFER_UNLOCKED
    assert local2.requests == [] and nav2.requests == []


# 진행 중 재시작: 잠금 상태 유지·목록 유지·새 전송 없음
def test_restart_mid_handoff_keeps_lock(tmp_path):
    runtime, mock, _, _ = make_scenario(tmp_path)
    mock.overrides[('task_a', 'load_pick_and_stow')] = None
    begin(runtime, 'task_a', 'box_a', 'robot_m', 'robot_t')
    mock.run()
    assert runtime.registry.states['task_a'].transfer_state == gm.TRANSFER_IN_PROGRESS
    runtime.store.close()

    restarted, mock2, local2, nav2 = make_scenario(tmp_path)
    state = restarted.registry.states['task_a']
    assert state.steps['load_pick_and_stow'].status == 'UNCERTAIN'
    assert 'RESTORED_UNRECONCILED' in state.inhibit
    mock2.publish('task_a', ['stowed', 'stopped'])
    assert local2.requests == [] and nav2.requests == []

    listed = task_list_fields(restarted.states())
    assert listed[0]['transfer_state'] == gm.TRANSFER_IN_PROGRESS


# W4: 잠금 요청 포트가 없어 예약 전에 실패하면 A3에 보낸 것이 없으므로 NONE을 유지한다(영구 차단 없음)
def test_lock_not_reserved_keeps_none(tmp_path):
    clk = Clock()
    nav = MockNavigationPort('scenario')
    store = TaskStore(str(tmp_path / 'd2.sqlite'))
    runtime = GroupRuntime(store, {'NAVIGATION': nav}, clk, map_revision=1, locations=LOCATIONS)
    mock = MockCounterpart(runtime, MockLocalTaskPort('unused'), nav, clk)
    begin(runtime, 'task_a', 'box_a', 'robot_m', 'robot_t')
    mock.run()

    state = runtime.registry.states['task_a']
    for key in ('src_lock_manipulator', 'src_lock_transporter'):
        run = state.steps[key]
        assert run.status == 'NOT_SENT' and not run.goal_uuid
        assert store.dispatch_row(run.execution_id)['status'] == 'INTENDED'

    assert 'DISPATCH_FAILED' in state.inhibit
    assert state.transfer_state == gm.TRANSFER_NONE
    assert state.transfer_id == ''


# W4: 예약 기록 직후 상태 기록 전에 멈춘 경우: 재시작 복원이 예약 표로 UNKNOWN을 세우고 재전송하지 않는다.
# 개발 출력 노드의 읽기 전용 복원도 같은 상태를 낸다
def test_restart_after_lock_reserve_before_record(tmp_path):
    from ax_fleet_manager.node_config import load_store_states
    from ax_fleet_manager.task_store import StoreError

    runtime, mock, local, _ = make_scenario(tmp_path)
    original_reserve = runtime.store.reserve_dispatch
    original_record = runtime.store.record_decision
    flags = {}

    # 잠금 요청 예약 뒤 다음 상태 기록을 실패시킨다
    def reserve(execution_id):
        goal_uuid = original_reserve(execution_id)
        if 'src_lock' in runtime.store.dispatch_row(execution_id)['step_key']:
            flags['fail_next'] = True
        return goal_uuid

    # 표시된 다음 기록 하나만 실패
    def record(*args, **kwargs):
        if flags.pop('fail_next', False):
            raise StoreError('시험 주입: 예약 뒤 상태 기록 실패')
        return original_record(*args, **kwargs)

    runtime.store.reserve_dispatch = reserve
    runtime.store.record_decision = record
    begin(runtime, 'task_a', 'box_a', 'robot_m', 'robot_t')
    mock.run()
    assert runtime.fault
    assert not any(r['operation'] == 'LOCK_HANDOFF' for r in local.requests)
    runtime.store.close()

    restarted, mock2, local2, nav2 = make_scenario(tmp_path)
    state = restarted.registry.states['task_a']
    assert state.transfer_state == gm.TRANSFER_UNKNOWN
    assert state.transfer_id == state.spec.source_transfer_id
    assert 'RESTORED_UNRECONCILED' in state.inhibit
    mock2.publish('task_a', ['stopped'])
    assert local2.requests == [] and nav2.requests == []
    restarted.store.close()

    (read_state,) = load_store_states(tmp_path / 'd2.sqlite')
    assert read_state.transfer_state == gm.TRANSFER_UNKNOWN
    listed = task_list_fields([read_state])
    assert listed[0]['transfer_state'] == gm.TRANSFER_UNKNOWN


# W4: 한쪽 잠금 실패 뒤 재시작해도 UNKNOWN·목록 유지, 이동 요청 0건
def test_restart_after_partial_lock_keeps_unknown(tmp_path):
    runtime, mock, _, nav = make_scenario(tmp_path)
    mock.overrides[('task_a', 'src_lock_transporter')] = gm.OUTCOME_FAILED
    begin(runtime, 'task_a', 'box_a', 'robot_m', 'robot_t')
    mock.run()
    navs = len(nav.requests)
    assert runtime.registry.states['task_a'].transfer_state == gm.TRANSFER_UNKNOWN
    runtime.store.close()

    restarted, mock2, local2, nav2 = make_scenario(tmp_path)
    mock2.publish('task_a', ['stopped'])
    state = restarted.registry.states['task_a']
    assert state.transfer_state == gm.TRANSFER_UNKNOWN
    assert local2.requests == [] and nav2.requests == []
    assert navs == 2
    assert task_list_fields(restarted.states())[0]['transfer_state'] == gm.TRANSFER_UNKNOWN
