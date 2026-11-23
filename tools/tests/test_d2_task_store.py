"""D2 원장·실행 결합 시험: 기록 후 전송, 중복 예약 거부, 쓰기 실패·중단·손상·복원.

실행: .venv/bin/python -m pytest -q -p no:launch_testing -p no:launch_ros \
      tools/tests/test_d2_task_store.py
"""
import json
import sqlite3
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
    AssignRoles, Decision, ObservationReceived, StartTask, StepResult, Tick,
)
from ax_fleet_manager.ports import (  # noqa: E402
    LocalTaskPort, MockLocalTaskPort, MockNavigationPort, NavigationPort, PortError,
)
from ax_fleet_manager.runtime import GroupRuntime  # noqa: E402
from ax_fleet_manager.task_store import (  # noqa: E402
    DispatchAlreadyReserved, StoreConflict, StoreCorrupt, TaskStore, state_from_dict,
    state_to_dict,
)

LOCATIONS = {'station_src': 1, 'station_dst': 1}


class SimulatedCrash(BaseException):
    """프로세스 중단 흉내(일반 예외 처리에 잡히지 않음)."""


class FailingConnection:
    """지정 SQL에서 실패하는 연결(쓰기 실패 주입)."""

    # 실제 연결과 실패 조건을 감싼다
    def __init__(self, db, state):
        self.db = db
        self.state = state

    # 실패 플래그가 켜지면 쓰기 문장에서 오류를 낸다
    def execute(self, sql, *args):
        if self.state.get('fail') and sql.split()[0] in ('UPDATE', 'INSERT', 'BEGIN'):
            raise sqlite3.OperationalError('disk I/O error (주입)')
        return self.db.execute(sql, *args)

    # 연결 닫기
    def close(self):
        self.db.close()


class Clock:
    """주입 시각."""

    # 시작 시각
    def __init__(self):
        self.t = 50.0

    # 현재 시각
    def __call__(self):
        return clock(self.t)


# 개발 runtime을 만든다
def make_runtime(path, local=None, nav=None, connect=sqlite3.connect, clk=None):
    store = TaskStore(str(path), connect=connect)
    ports = {'LOCAL_TASK': local or MockLocalTaskPort(),
             'NAVIGATION': nav or MockNavigationPort()}
    runtime = GroupRuntime(store, ports, clk or Clock(), map_revision=1, locations=LOCATIONS)

    return runtime


# 작업을 수용·배정·시작한다
def start_task(runtime, task_id='task_dev_1', object_id='box_dev_1'):
    runtime.admit(make_spec(task_id, object_id))
    runtime.handle(task_id, AssignRoles(make_assignment('robot_m1', 'robot_t9')))
    return runtime.handle(task_id, StartTask())


# 상태 JSON 왕복
def test_state_roundtrip():
    state = gm.new_group_state(make_spec())
    state.assignment = make_assignment('robot_a', 'robot_b')
    state.inhibit.append('X')
    assert state_from_dict(json.loads(json.dumps(state_to_dict(state)))) == state


# 시작 의도는 원장에 기록되고 전송 시작(SUBMITTING)이 기록된 뒤에만 전송된다.
# W5(사용자 결정): 주행 goal UUID는 D1이 만들므로 전송 순간 D2 원장에는 UUID가 없고(이전 기대값:
# RESERVED+D2 UUID), D1이 돌려준 UUID를 전송 뒤 기록한다. 로컬 작업의 예약 선기록은 test_d2_inventory에서 본다
def test_intent_recorded_and_reserved_before_send(tmp_path):
    order = []
    nav = MockNavigationPort()
    runtime = make_runtime(tmp_path / 'd2.sqlite', nav=nav)
    original = nav.dispatch

    # 전송 순간의 원장 기록 상태를 확인한다
    def checking(intent, goal_uuid):
        row = runtime.store.dispatch_row(intent.execution['execution_id'])
        order.append((row['status'], row['goal_uuid'], goal_uuid))
        return original(intent, goal_uuid)

    nav.dispatch = checking
    start_task(runtime)
    assert order == [('SUBMITTING', None, None), ('SUBMITTING', None, None)]
    rows = runtime.store.dispatch_rows('task_dev_1')
    assert [r['status'] for r in rows] == ['SENT', 'SENT']
    assert {r['goal_uuid'] for r in rows} == set(nav.goal_uuids.values())
    assert {r['step_key'] for r in rows} == {'src_nav_manipulator', 'src_nav_transporter'}
    assert [r['robot_id'] for r in nav.requests] == ['robot_m1', 'robot_t9']


# 같은 실행 ID는 다시 예약·전송하지 않는다
def test_reserve_twice_is_rejected(tmp_path):
    runtime = make_runtime(tmp_path / 'd2.sqlite')
    start_task(runtime)
    execution_id = runtime.store.dispatch_rows('task_dev_1')[0]['execution_id']
    with pytest.raises(DispatchAlreadyReserved):
        runtime.store.reserve_dispatch(execution_id)


# 같은 실행 ID에 다른 인자는 충돌로 거부한다
def test_conflicting_args_for_same_execution_id(tmp_path):
    runtime = make_runtime(tmp_path / 'd2.sqlite')
    decision = start_task(runtime)
    intent = runtime.store.dispatch_rows('task_dev_1')[0]
    store = runtime.store
    state = runtime.registry.states['task_dev_1']

    from ax_fleet_manager.group_events import CommandIntent
    fake = CommandIntent('NAVIGATION', 'task_dev_1', intent['step_key'], intent['operation'],
                         intent['robot_id'], '', {'execution_id': intent['execution_id']},
                         {}, 'different_digest')
    with pytest.raises(StoreConflict):
        store.record_decision('task_dev_1', state.revision,
                              Decision(state, (fake,)), 'Fake', {})
    assert decision.effects


# expected_revision 불일치는 기록 거부(동시 갱신 방지)
def test_revision_conflict(tmp_path):
    runtime = make_runtime(tmp_path / 'd2.sqlite')
    start_task(runtime)
    state = runtime.registry.states['task_dev_1']
    with pytest.raises(StoreConflict):
        runtime.store.record_decision('task_dev_1', state.revision - 1, Decision(state),
                                      'Stale', {})


# 쓰기 실패: 반영·전송 없음, runtime 고장 상태로 이후 명령 차단
def test_write_failure_blocks_without_send(tmp_path):
    flags = {}
    nav = MockNavigationPort()

    # 실패 주입 연결
    def connect(path, **kwargs):
        return FailingConnection(sqlite3.connect(path, **kwargs), flags)

    runtime = make_runtime(tmp_path / 'd2.sqlite', nav=nav, connect=connect)
    runtime.admit(make_spec())
    runtime.handle('task_dev_1', AssignRoles(make_assignment('robot_m1', 'robot_t9')))
    flags['fail'] = True
    decision = runtime.handle('task_dev_1', StartTask())

    assert 'STORE_WRITE_FAILED' in decision.reasons
    assert nav.requests == []
    assert runtime.fault
    assert 'STORE_FAULT' in runtime.registry.states['task_dev_1'].inhibit

    flags['fail'] = False
    decision = runtime.handle('task_dev_1', Tick())
    assert decision.reasons == ('RUNTIME_STORE_FAULT',)
    assert nav.requests == []


# 예약 기록 실패: 전송하지 않는다
def test_reserve_failure_blocks_send(tmp_path):
    flags = {}
    nav = MockNavigationPort()

    # 실패 주입 연결
    def connect(path, **kwargs):
        return FailingConnection(sqlite3.connect(path, **kwargs), flags)

    runtime = make_runtime(tmp_path / 'd2.sqlite', nav=nav, connect=connect)
    runtime.admit(make_spec())
    runtime.handle('task_dev_1', AssignRoles(make_assignment('robot_m1', 'robot_t9')))
    # W5: 주행 전송 시작 기록은 reserve_dispatch 대신 begin_submit이다(주행 UUID는 D1 생성)
    original = runtime.store.begin_submit

    # 예약(전송 시작 기록) 직전에 실패를 켠다
    def failing_reserve(execution_id):
        flags['fail'] = True
        return original(execution_id)

    runtime.store.begin_submit = failing_reserve
    runtime.handle('task_dev_1', StartTask())
    assert nav.requests == []
    assert runtime.fault


# 예약 후 중단 → 재시작 시 미확정, 자동 재전송 없음, 새 명령 차단
# reserved_only: 예약 기록 직후(상태에는 아직 INTENDED), before_send: 전송 직전, after_send: 전송 직후
@pytest.mark.parametrize('crash_point', ['reserved_only', 'before_send', 'after_send'])
def test_crash_between_reserve_and_send_is_uncertain(tmp_path, crash_point):
    path = tmp_path / 'd2.sqlite'
    nav = MockNavigationPort()

    # 전송 전 또는 전송 직후에 프로세스가 멈춘 것처럼 만든다
    def crashing(intent, goal_uuid):
        if crash_point == 'after_send':
            nav.requests.append(goal_uuid)
        raise SimulatedCrash()

    nav.dispatch = crashing
    runtime = make_runtime(path, nav=nav)
    if crash_point == 'reserved_only':
        # W5: 주행은 begin_submit(SUBMITTING)이 예약 기록에 해당한다(주행 UUID는 D1 생성)
        original = runtime.store.begin_submit

        # 전송 시작 기록은 끝났지만 상태 기록·전송 전에 멈춘다
        def reserve_then_crash(execution_id):
            original(execution_id)
            raise SimulatedCrash()

        runtime.store.begin_submit = reserve_then_crash

    with pytest.raises(SimulatedCrash):
        start_task(runtime)
    runtime.store.close()

    nav2 = MockNavigationPort()
    restarted = make_runtime(path, nav=nav2)
    state = restarted.registry.states['task_dev_1']
    statuses = {k: v.status for k, v in state.steps.items() if v.status != 'PENDING'}
    assert 'UNCERTAIN' in statuses.values()
    assert 'RESTORED_UNRECONCILED' in state.inhibit
    assert nav2.requests == []

    # 늦은 결과·최신 관측이 와도 진행하지 않는다
    source = MockObservationSource()
    now = restarted.clock()
    for robot in ('robot_m1', 'robot_t9'):
        restarted.handle('task_dev_1', ObservationReceived(
            source.make('ACTUATION', robot, gm.ACTUATION_STOPPED, now)))
    run = [v for v in state.steps.values() if v.status == 'UNCERTAIN'][0]
    if run.goal_uuid:
        restarted.handle('task_dev_1', StepResult(run.goal_uuid, 'robot_m1',
                                                  gm.OUTCOME_SUCCEEDED, gm.PROVENANCE_MOCK))
    restarted.handle('task_dev_1', Tick())
    assert nav2.requests == []
    # W5: 주행 전송 시작 상태 SUBMITTING 추가
    assert restarted.store.dispatch_rows('task_dev_1')[0]['status'] in ('SUBMITTING', 'RESERVED',
                                                                        'SENT', 'UNCERTAIN')


# 정상 재시작도 복원 상태를 원장에 남기고 새 명령을 막는다(과거 세션·관측 복원 없음)
def test_restart_restores_blocked_state(tmp_path):
    path = tmp_path / 'd2.sqlite'
    runtime = make_runtime(path)
    start_task(runtime)
    runtime.store.close()

    restarted = make_runtime(path)
    state = restarted.registry.states['task_dev_1']
    assert state.observations == {}
    assert 'RESTORED_UNRECONCILED' in state.inhibit
    assert restarted.store.events('task_dev_1')[-1][0] == 'Restore'

    restarted.store.close()
    again = make_runtime(path)
    assert again.registry.states['task_dev_1'].inhibit.count('RESTORED_UNRECONCILED') == 1


# 원장 손상: 무결성·schema·JSON 이상은 열기를 거부한다
def test_corrupt_store_rejected(tmp_path):
    garbage = tmp_path / 'garbage.sqlite'
    garbage.write_bytes(b'not a database' * 100)
    with pytest.raises(StoreCorrupt):
        TaskStore(str(garbage))

    path = tmp_path / 'd2.sqlite'
    runtime = make_runtime(path)
    runtime.admit(make_spec())
    runtime.store.db.execute("UPDATE tasks SET state_json='{broken'")
    runtime.store.close()
    with pytest.raises(StoreCorrupt):
        make_runtime(path)

    other = tmp_path / 'other.sqlite'
    TaskStore(str(other)).close()
    db = sqlite3.connect(str(other))
    db.execute("UPDATE meta SET value='999' WHERE key='schema_version'")
    db.commit()
    db.close()
    with pytest.raises(StoreCorrupt):
        TaskStore(str(other))

    foreign = tmp_path / 'foreign.sqlite'
    db = sqlite3.connect(str(foreign))
    db.execute('CREATE TABLE status_ledger (x TEXT)')
    db.commit()
    db.close()
    with pytest.raises(StoreCorrupt):
        TaskStore(str(foreign))


# 저장 단계 기록이 계획과 다르면 손상
def test_tampered_steps_rejected(tmp_path):
    path = tmp_path / 'd2.sqlite'
    runtime = make_runtime(path)
    runtime.admit(make_spec())
    row = runtime.store.db.execute('SELECT state_json FROM tasks').fetchone()[0]
    data = json.loads(row)
    data['steps'].pop('load_confirm')
    runtime.store.db.execute('UPDATE tasks SET state_json=?', (json.dumps(data),))
    runtime.store.close()
    with pytest.raises(StoreCorrupt):
        make_runtime(path)


# 사건 기록 상한
def test_event_log_bounded(tmp_path):
    store = TaskStore(str(tmp_path / 'd2.sqlite'), max_events_per_task=5)
    runtime = GroupRuntime(store, {'LOCAL_TASK': MockLocalTaskPort(),
                                   'NAVIGATION': MockNavigationPort()},
                           Clock(), map_revision=1, locations=LOCATIONS)
    runtime.admit(make_spec())
    for _ in range(20):
        runtime.handle('task_dev_1', Tick())
    assert len(store.events('task_dev_1')) == 5


# 미지원(live) 포트는 예약·전송 없이 NOT_SENT와 차단
def test_unsupported_live_ports_do_not_send(tmp_path):
    runtime = make_runtime(tmp_path / 'd2.sqlite', local=LocalTaskPort(), nav=NavigationPort())
    start_task(runtime)
    state = runtime.registry.states['task_dev_1']
    assert {state.steps[k].status for k in ('src_nav_manipulator', 'src_nav_transporter')} \
        == {'NOT_SENT'}
    assert 'DISPATCH_FAILED' in state.inhibit
    assert {r['status'] for r in runtime.store.dispatch_rows('task_dev_1')} == {'INTENDED'}


# 포트 전송 오류(불확실)는 미확정으로 기록, 재전송 없음
def test_port_uncertain_error_recorded(tmp_path):
    nav = MockNavigationPort()
    nav.fail_next = PortError('LINK_LOST', uncertain=True)
    runtime = make_runtime(tmp_path / 'd2.sqlite', nav=nav)
    start_task(runtime)
    rows = runtime.store.dispatch_rows('task_dev_1')
    assert rows[0]['status'] == 'UNCERTAIN'
    assert 'DISPATCH_UNCERTAIN' in runtime.registry.states['task_dev_1'].inhibit
    assert len(nav.requests) == 0


# 이 원장은 D2 전용 파일이다: D4 원장 표를 만들거나 열지 않는다
def test_store_is_d2_only(tmp_path):
    store = TaskStore(str(tmp_path / 'd2.sqlite'))
    tables = {r[0] for r in store.db.execute("SELECT name FROM sqlite_master WHERE type='table'")}
    # W4 schema 2(K9 승인)는 입고 표 3개, W5 schema 3은 재고·지점·예약 표 3개를 더한다.
    # 정확한 표 집합 비교는 유지한다
    assert tables == {'meta', 'tasks', 'dispatches', 'events', 'sqlite_sequence',
                      'receipt_events', 'receipt_observations', 'register_requests',
                      'inventory', 'locations', 'reservations'}
