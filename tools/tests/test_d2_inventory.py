"""D2 W5 재고 갱신·완료 판정·예약·주행 UUID 주체·live 어댑터 초안 시험(ROS 없음, 모의 상대).

실행: .venv/bin/python -m pytest -q -p no:launch_testing -p no:launch_ros \
      tools/tests/test_d2_inventory.py
"""
import importlib.util
import itertools
import json
import sqlite3
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src' / 'ax_fleet_manager'))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from ax_fleet_manager import group_model as gm  # noqa: E402
from ax_fleet_manager import inventory as inv  # noqa: E402
from ax_fleet_manager import receipt as rc  # noqa: E402
from ax_fleet_manager.dev_fixtures import (  # noqa: E402
    catalog_dict, make_assignment, make_spec,
)
from ax_fleet_manager.group_events import (  # noqa: E402
    AssignRoles, FailTerminate, ObservationReceived, StartTask, StepResult,
    Tick,
)
from ax_fleet_manager.group_machine import apply_event, restore_state  # noqa: E402
from ax_fleet_manager.intake import ReceiptIntake  # noqa: E402
from ax_fleet_manager.live_adapters import (  # noqa: E402
    D1NavigationPort, actuation_observation, d1_result_event, hold_fields,
    hold_observation, hold_rejection,
)
from ax_fleet_manager.ports import MockLocalTaskPort, MockNavigationPort, PortError  # noqa: E402
from ax_fleet_manager.runtime import GroupRuntime  # noqa: E402
from ax_fleet_manager.task_store import (  # noqa: E402
    RECEIPT_DDL, V1_DDL, V3_TABLES, StoreCorrupt, TaskStore, state_to_dict,
)
import test_d2_workflow_scenarios as scenarios  # noqa: E402
from test_d2_group_machine import Flow, run_until  # noqa: E402

MOCK = gm.PROVENANCE_MOCK
PURPOSES = {'zone_in': 'RECEIPT', 'station_dst': 'STORAGE', 'station_big': 'STORAGE',
            'station_off': 'STORAGE', 'station_src': 'STATION'}
CONFIG = rc.ReceiptConfig(receipt_zones=('zone_in',),
                          destination_rules={'zone_in': 'station_dst'})


class SimulatedCrash(BaseException):
    """프로세스 중단 흉내."""


class FailingConnection:
    """지정 SQL 접두어에서 실패하는 연결."""

    # 실제 연결과 실패 조건을 감싼다
    def __init__(self, db, state):
        self.db = db
        self.state = state

    # 실패 접두어와 맞는 문장에서 오류를 낸다
    def execute(self, sql, *args):
        prefix = self.state.get('fail_prefix')
        if prefix and sql.startswith(prefix):
            raise sqlite3.OperationalError('disk I/O error (주입)')
        return self.db.execute(sql, *args)

    # 연결 닫기
    def close(self):
        self.db.close()


# 시험 카탈로그: station_dst는 용량 1·box_small만, station_big은 제한 없음, station_off는 사용 중지
def make_catalog():
    data = catalog_dict(PURPOSES)
    data['locations']['station_dst'].update(capacity=1, allowed_object_classes=['box_small'])
    data['locations']['station_off']['active'] = False

    return inv.parse_catalog(data)


CATALOG = make_catalog()


# runtime·접수기·모의 상대 묶음
def make_world(tmp_path, connect=None, nav=None, uuid_calls=None):
    clk = scenarios.Clock()
    local = MockLocalTaskPort('inv')
    nav = nav or MockNavigationPort('inv')
    counter = itertools.count(1)

    # D2 goal UUID 발급 횟수를 센다(주행 단계에서 쓰이면 안 됨)
    def d2_uuid():
        if uuid_calls is not None:
            uuid_calls.append(1)
        return f'd2uuid{next(counter):04d}'

    kwargs = {'connect': connect} if connect else {}
    store = TaskStore(str(tmp_path / 'd2.sqlite'), uuid_factory=d2_uuid, **kwargs)
    runtime = GroupRuntime(store, {'LOCAL_TASK': local, 'NAVIGATION': nav}, clk,
                           map_revision=CATALOG.map_revision, locations=CATALOG.revisions())
    intake = ReceiptIntake(runtime, CONFIG, CATALOG)
    mock = scenarios.MockCounterpart(runtime, local, nav, clk)

    return runtime, intake, mock, local, nav


# 입고 관측 사전
def obs(observation_id, object_id='obj-box-1', object_class='box_small', stamp=200.0):
    return {'observation_id': observation_id, 'zone_id': 'zone_in', 'object_id': object_id,
            'object_class': object_class, 'identity_confidence': 0.95,
            'source': rc.SOURCE_SENSOR, 'source_id': 'c1_mock', 'valid': True,
            'stamp_sim_s': stamp}


# 입고 → 배정 → 시작
def receive_and_start(runtime, intake, object_id='obj-box-1'):
    result = intake.observe(obs(f'ob-{object_id}', object_id))
    assert result.decision == 'TASK_CREATED', result
    runtime.handle(result.task_id, AssignRoles(make_assignment('robot_m', 'robot_t')))
    runtime.handle(result.task_id, StartTask())

    return result.task_id


# 작업의 예약 상태
def reservation_state(runtime, task_id):
    return [r['state'] for r in runtime.store.reservations() if r['task_id'] == task_id]


# ---- 엔진: 그룹 물체 상태 전이 ----

# 표준 계획을 단계별로 진행하며 물체 상태를 기록한다. 집기·하역 단계에 보유(HELD) 관측을 넣는다
def test_object_state_progression_with_hold_evidence():
    flow = Flow()
    seen = []
    original = flow.apply

    # 사건마다 물체 상태 변화를 기록한다
    def tracking(event):
        decision = original(event)
        value = decision.next_state.object_state
        if not seen or seen[-1] != value:
            seen.append(value)
        return decision

    flow.apply = tracking
    plan = [
        (['src_nav_manipulator', 'src_nav_transporter'], {}, ['stopped']),
        (['src_lock_manipulator', 'src_lock_transporter'], {}, ['stopped']),
        (['load_pick_and_stow'], {}, ['held']),
        (['load_confirm'], {'load': gm.LOAD_STOWED}, ['stowed']),
        (['load_retract'], {}, ['stowed', 'released', 'retracted', 'stopped']),
        (['src_unlock_manipulator', 'src_unlock_transporter'], {},
         ['stowed', 'released', 'retracted', 'stopped']),
        (['dst_nav_manipulator', 'dst_nav_transporter'], {}, ['stopped']),
        (['dst_lock_manipulator', 'dst_lock_transporter'], {}, ['stopped', 'stowed']),
        (['unload_place'], {}, ['held', 'placed']),
        (['unload_retract'], {}, ['placed', 'empty']),
        (['unload_confirm_empty'], {'load': gm.LOAD_EMPTY},
         ['empty', 'placed', 'released', 'retracted', 'stopped']),
        (['dst_unlock_manipulator', 'dst_unlock_transporter'], {},
         ['empty', 'placed', 'released', 'retracted', 'stopped']),
    ]
    flow.start()
    for keys, result, evidence in plan:
        assert sorted(i.step_key for i in flow.unsent()) == sorted(keys)
        flow.send_all()
        flow.wait(0.05)
        flow.evidence(*evidence)
        for key in keys:
            flow.finish(key, **result)

    assert flow.state.task_state == gm.TASK_COMPLETED
    assert seen == [gm.INV_AT_SOURCE, gm.INV_HELD_BY_MANIPULATOR, gm.INV_STOWED_ON_TRANSPORTER,
                    gm.INV_HELD_BY_MANIPULATOR, gm.INV_AT_DESTINATION]
    assert flow.state.object_location_id == flow.spec.destination_location_id
    assert flow.state.object_carrier_robot_id == ''


# 보유 관측이 없으면 HELD를 건너뛰고 적재 확인 근거로 STOWED가 된다(K1). 보유 관측도 예약 전이면 무시
def test_object_state_without_hold_and_hold_before_pick_ignored():
    early = Flow()
    run_until(early, 'src_lock')
    early.evidence('held')
    assert early.state.object_state == gm.INV_AT_SOURCE
    assert not early.state.physical_started

    stowed = Flow()
    run_until(stowed, 'confirm')
    assert stowed.state.object_state == gm.INV_STOWED_ON_TRANSPORTER
    assert stowed.state.object_carrier_robot_id == stowed.assignment.robots()[1]
    assert stowed.state.object_assignment_generation == 1


# 집기 요청 예약 전 실패는 출발 지점 유지, 예약 뒤 실패·미확정은 UNKNOWN·격리, LOAD_LOST는 LOST
def test_object_failure_states():
    before = Flow()
    run_until(before, 'src_nav')
    before.send_all()
    before.finish('src_lock_manipulator', outcome=gm.OUTCOME_FAILED)
    assert before.state.object_state == gm.INV_AT_SOURCE and not before.state.object_isolated

    after = Flow()
    run_until(after, 'src_lock')
    after.send_all()
    after.finish('load_pick_and_stow', outcome=gm.OUTCOME_FAILED)
    assert after.state.object_state == gm.INV_UNKNOWN
    assert after.state.object_isolated and after.state.object_location_id == ''

    lost = Flow()
    run_until(lost, 'src_unlock')
    lost.observe('LOAD', lost.assignment.robots()[1], gm.LOAD_LOST,
                 object_id=lost.spec.object_id)
    assert lost.state.object_state == gm.INV_LOST and lost.state.object_isolated


# 복원 미대조 중에는 물체 상태를 바꾸지 않는다(승격·강등 없음)
def test_object_state_frozen_while_restored():
    flow = Flow()
    run_until(flow, 'confirm')
    flow.state = restore_state(flow.state)
    flow.finish('load_retract', outcome=gm.OUTCOME_FAILED)
    assert flow.state.object_state == gm.INV_STOWED_ON_TRANSPORTER


# ---- runtime + 원장: 재고 갱신·완료 판정(E-31) ----

# (a) 전 흐름: 입고 → 등록·예약 → 배정·실행 → 완료 → 재고 AT_DESTINATION·예약 소진·지점 OCCUPIED
def test_full_flow_receipt_to_inventory(tmp_path):
    runtime, intake, mock, local, nav = make_world(tmp_path)
    task_id = receive_and_start(runtime, intake)
    entry = runtime.store.inventory_entry('obj-box-1')
    assert (entry['state'], entry['location_id'], entry['revision']) == (
        gm.INV_AT_SOURCE, 'zone_in', 1)
    assert runtime.registry.states[task_id].spec.inventory_revision == 1
    assert reservation_state(runtime, task_id) == ['ACTIVE']

    mock.run()
    state = runtime.registry.states[task_id]
    assert state.task_state == gm.TASK_COMPLETED
    entry = runtime.store.inventory_entry('obj-box-1')
    assert (entry['state'], entry['location_id'], entry['carrier_robot_id']) == (
        gm.INV_AT_DESTINATION, 'station_dst', '')
    assert entry['receipt_event_id'].startswith('rcpt-') and entry['object_class'] == 'box_small'
    assert entry['revision'] == 3  # AT_SOURCE(1) → STOWED(2) → AT_DESTINATION(3)
    assert reservation_state(runtime, task_id) == ['CONSUMED']

    snapshot = inv.inventory_snapshot_fields(runtime.store, CATALOG)
    dst = [loc for loc in snapshot['locations'] if loc['location_id'] == 'station_dst'][0]
    assert dst['state'] == inv.LOCATION_OCCUPIED and dst['reservation_id'] == ''
    assert [i['state'] for i in snapshot['items']] == [gm.INV_AT_DESTINATION]

    # 용량 1 지점이 물품으로 찼으므로 다음 입고는 LOCATION_FULL
    second = intake.observe(obs('ob-2', 'obj-box-2'))
    assert (second.decision, second.reason_code) == ('REJECTED', inv.LOCATION_FULL)


# E-31 ①: 배치 관측이 없으면 완료·AT_DESTINATION 없음, 예약 유지
def test_e31_no_placement_no_destination(tmp_path, monkeypatch):
    evidence = {k: [n for n in v if n != 'placed'] for k, v in scenarios.EVIDENCE.items()}
    monkeypatch.setattr(scenarios, 'EVIDENCE', evidence)
    runtime, intake, mock, _, _ = make_world(tmp_path)
    task_id = receive_and_start(runtime, intake)
    mock.run()

    assert runtime.registry.states[task_id].task_state == gm.TASK_RUNNING
    assert runtime.store.inventory_entry('obj-box-1')['state'] != gm.INV_AT_DESTINATION
    assert reservation_state(runtime, task_id) == ['ACTIVE']


# E-31 ②: 해제 결과 유실(시간초과 미확정) → UNKNOWN·격리, AT_DESTINATION 아님, 예약 유지
def test_e31_result_lost_is_unknown(tmp_path):
    runtime, intake, mock, _, _ = make_world(tmp_path)
    task_id = receive_and_start(runtime, intake)
    mock.overrides[(task_id, 'dst_unlock_transporter')] = None
    mock.run()
    assert runtime.registry.states[task_id].task_state == gm.TASK_RUNNING

    mock.clock.t += 61.0
    runtime.handle(task_id, Tick())
    entry = runtime.store.inventory_entry('obj-box-1')
    assert entry['state'] == gm.INV_UNKNOWN and entry['isolated']
    assert reservation_state(runtime, task_id) == ['ACTIVE']


# E-31 ③: 하역 중 재시작 → 재고 그대로(승격 없음), 이후 관측에도 AT_DESTINATION 없음, 예약 유지
def test_e31_restart_before_completion(tmp_path):
    runtime, intake, mock, _, _ = make_world(tmp_path)
    task_id = receive_and_start(runtime, intake)
    mock.overrides[(task_id, 'unload_place')] = None
    mock.run()
    before = runtime.store.inventory_entry('obj-box-1')
    assert before['state'] == gm.INV_STOWED_ON_TRANSPORTER
    runtime.store.close()

    runtime2, _, mock2, local2, nav2 = make_world(tmp_path)
    mock2.publish(task_id, ['placed', 'empty', 'released', 'retracted', 'stopped'])
    after = runtime2.store.inventory_entry('obj-box-1')
    assert after == before
    assert reservation_state(runtime2, task_id) == ['ACTIVE']
    assert local2.requests == [] and nav2.requests == []


# 물리 시작 전 실패 종료는 예약 해제·출발 지점 유지, 시작 뒤 실패는 예약 유지·UNKNOWN
@pytest.mark.parametrize('failed_step,started', [('src_lock_transporter', False),
                                                 ('load_pick_and_stow', True)])
def test_failure_reservation_and_inventory(tmp_path, failed_step, started):
    runtime, intake, mock, _, _ = make_world(tmp_path)
    task_id = receive_and_start(runtime, intake)
    mock.overrides[(task_id, failed_step)] = gm.OUTCOME_FAILED
    mock.run()
    runtime.handle(task_id, FailTerminate('fail-inv-1'))

    entry = runtime.store.inventory_entry('obj-box-1')
    if started:
        assert entry['state'] == gm.INV_UNKNOWN and entry['isolated']
        assert reservation_state(runtime, task_id) == ['ACTIVE']
    else:
        assert entry['state'] == gm.INV_AT_SOURCE and not entry['isolated']
        assert entry['location_id'] == 'zone_in'
        assert reservation_state(runtime, task_id) == ['RELEASED']


# 운반 중 적재 이탈 확인 → LOST
def test_load_lost_marks_inventory_lost(tmp_path):
    runtime, intake, mock, _, _ = make_world(tmp_path)
    task_id = receive_and_start(runtime, intake)
    mock.overrides[(task_id, 'dst_nav_transporter')] = None
    mock.run()
    state = runtime.registry.states[task_id]
    runtime.handle(task_id, ObservationReceived(mock.source.make(
        'LOAD', 'robot_t', gm.LOAD_LOST, mock.clock(), object_id=state.spec.object_id)))

    entry = runtime.store.inventory_entry('obj-box-1')
    assert entry['state'] == gm.INV_LOST and entry['isolated']
    assert entry['carrier_robot_id'] == 'robot_t'


# ---- 등록·예약 원자성, 용량·종류, 수동 등록(K7) ----

# 목적지 종류·사용 중지·용량 검사
def test_destination_checks(tmp_path):
    runtime, intake, _, _, _ = make_world(tmp_path)
    other = intake.observe(obs('ob-1', 'obj-a', object_class='pallet'))
    assert other.reason_code == inv.OBJECT_CLASS_NOT_ALLOWED

    manual_off = intake.register_task({
        'request_id': 'req-off', 'receipt_event_id': '', 'object_id': 'obj-m',
        'source_location_id': 'station_src', 'destination_location_id': 'station_off'})
    assert manual_off['reason_code'] == inv.LOCATION_INACTIVE_CODE

    first = intake.observe(obs('ob-2', 'obj-b'))
    second = intake.observe(obs('ob-3', 'obj-c'))
    assert first.decision == 'TASK_CREATED'
    assert (second.decision, second.reason_code) == ('REJECTED', inv.LOCATION_FULL)
    assert runtime.store.inventory_entry('obj-c') is None
    assert len(runtime.store.reservations()) == 1


# 예약 원자성: 예약·재고·작업 행 중 하나라도 실패하면 모두 되돌리고 registry도 그대로
@pytest.mark.parametrize('prefix', ['INSERT INTO reservations', 'INSERT OR REPLACE INTO inventory',
                                    'INSERT INTO tasks'])
def test_reservation_atomicity(tmp_path, prefix):
    flags = {}

    # 실패 주입 연결
    def connect(path, **kwargs):
        return FailingConnection(sqlite3.connect(path, **kwargs), flags)

    runtime, intake, _, _, _ = make_world(tmp_path, connect=connect)
    flags['fail_prefix'] = prefix
    result = intake.observe(obs('ob-1'))
    assert result.reason_code == 'STORE_WRITE_FAILED' and runtime.fault
    assert runtime.registry.states == {}
    runtime.store.close()

    store = TaskStore(str(tmp_path / 'd2.sqlite'))
    assert store.inventory_entries() == [] and store.reservations() == []
    assert store.load_states() == [] and store.receipt_events() == []
    store.close()


# 수동 등록(K7): 도착 물품의 재이동은 출발 = 현재 위치일 때만, 격리·위치 불일치는 INVENTORY_MISMATCH
def test_manual_register_inventory_rules(tmp_path):
    runtime, intake, mock, _, _ = make_world(tmp_path)
    receive_and_start(runtime, intake)
    mock.run()
    assert runtime.store.inventory_entry('obj-box-1')['state'] == gm.INV_AT_DESTINATION

    # 수동 요청 사전
    def manual(request_id, source, destination='station_big', object_id='obj-box-1'):
        return intake.register_task({
            'request_id': request_id, 'receipt_event_id': '', 'object_id': object_id,
            'source_location_id': source, 'destination_location_id': destination})

    assert manual('req-1', 'station_src')['reason_code'] == inv.INVENTORY_MISMATCH
    moved = manual('req-2', 'station_dst')
    assert moved['accepted']
    entry = runtime.store.inventory_entry('obj-box-1')
    assert (entry['state'], entry['task_id'], entry['location_id']) == (
        gm.INV_AT_SOURCE, moved['task_id'], 'station_dst')
    assert runtime.registry.states[moved['task_id']].spec.inventory_revision == entry['revision']
    assert manual('req-3', 'station_dst')['reason_code'] == rc.OBJECT_BUSY

    # 격리된 물품(UNKNOWN)은 수동 등록 불가. 출발 인계 해제 뒤 운송 주행이 실패해 잠금은 풀렸지만
    # 물품 위치를 확정할 수 없는 경우(잠금이 남으면 OBJECT_BUSY가 먼저다)
    runtime.handle(moved['task_id'], AssignRoles(make_assignment('robot_m', 'robot_t',
                                                                 assignment_id='asg-k7')))
    runtime.handle(moved['task_id'], StartTask())
    assert runtime.registry.states[moved['task_id']].task_state == gm.TASK_RUNNING
    mock.overrides[(moved['task_id'], 'dst_nav_transporter')] = gm.OUTCOME_FAILED
    mock.run()
    runtime.handle(moved['task_id'], FailTerminate('fail-k7'))
    failed = runtime.registry.states[moved['task_id']]
    assert failed.task_state == gm.TASK_FAILED
    assert failed.transfer_state == gm.TRANSFER_UNLOCKED
    entry = runtime.store.inventory_entry('obj-box-1')
    assert entry['isolated'] and entry['state'] == gm.INV_UNKNOWN
    assert manual('req-4', 'station_dst')['reason_code'] == inv.INVENTORY_MISMATCH

    # 재고 항목이 없는 새 물체의 수동 요청은 AT_SOURCE 항목을 만든다(종류 없음 → "*" 지점만)
    fresh = manual('req-5', 'station_src', object_id='obj-new')
    assert fresh['accepted']
    assert runtime.store.inventory_entry('obj-new')['state'] == gm.INV_AT_SOURCE
    assert manual('req-6', 'station_src', 'station_dst', 'obj-new2')['reason_code'] == \
        inv.OBJECT_CLASS_NOT_ALLOWED


# 재고 revision: 항목 revision은 변경마다 +1, snapshot revision은 단조 증가·재시작 뒤 유지
def test_inventory_revisions_persist(tmp_path):
    runtime, intake, mock, _, _ = make_world(tmp_path)
    start_revision = runtime.store.inventory_revision()
    receive_and_start(runtime, intake)
    mock.run()
    end_revision = runtime.store.inventory_revision()
    assert end_revision > start_revision
    runtime.store.close()

    runtime2, _, _, _, _ = make_world(tmp_path)
    assert runtime2.store.inventory_revision() == end_revision


# ---- 원장 이주 v1·v2 → v3 ----

# v1·v2 원장 만들기(진행 중 작업 1, 종료 작업 1)
def make_old_store(path, version):
    db = sqlite3.connect(path, isolation_level=None)
    for ddl in V1_DDL + (RECEIPT_DDL if version == 2 else ()):
        db.execute(ddl)
    db.execute("INSERT INTO meta VALUES ('schema_version', ?)", (str(version),))

    running = gm.new_group_state(make_spec('task_run', 'obj-run'))
    done = gm.new_group_state(make_spec('task_done', 'obj-done'))
    done.task_state = gm.TASK_FAILED
    for state in (running, done):
        data = state_to_dict(state)
        for key in ('object_state', 'object_location_id', 'object_carrier_robot_id',
                    'object_assignment_generation', 'object_isolated', 'physical_started'):
            del data[key]
        db.execute('INSERT INTO tasks VALUES (?, ?, ?)',
                   (state.spec.task_id, state.revision, json.dumps(data)))
    db.close()


# 표 이름·버전
def schema_of(path):
    db = sqlite3.connect(path)
    tables = {r[0] for r in db.execute("SELECT name FROM sqlite_master WHERE type='table'")}
    version = db.execute("SELECT value FROM meta WHERE key='schema_version'").fetchone()[0]
    columns = [r[1] for r in db.execute('PRAGMA table_info(receipt_events)')]
    db.close()

    return tables, version, columns


# 이주: 표·열 추가, 진행 중 작업만 UNKNOWN 백필, 작업 보존
@pytest.mark.parametrize('version', [1, 2])
def test_migration_to_v3(tmp_path, version):
    path = str(tmp_path / 'd2.sqlite')
    make_old_store(path, version)
    store = TaskStore(path)
    assert store.schema_version == 3
    assert sorted(s.spec.task_id for s in store.load_states()) == ['task_done', 'task_run']
    entries = store.inventory_entries()
    assert [(e['object_id'], e['state'], e['isolated'], e['task_id']) for e in entries] == [
        ('obj-run', gm.INV_UNKNOWN, False, 'task_run')]
    store.close()

    tables, schema, columns = schema_of(path)
    assert V3_TABLES <= tables and schema == '3' and 'object_class' in columns


# 이주 실패: 원장 불변(버전·표), 읽기 전용 열기는 이주 없이 읽는다
@pytest.mark.parametrize('version', [1, 2])
@pytest.mark.parametrize('prefix', ['CREATE TABLE reservations', 'INSERT INTO inventory',
                                    'UPDATE meta'])
def test_migration_failure_leaves_store(tmp_path, version, prefix):
    path = str(tmp_path / 'd2.sqlite')
    make_old_store(path, version)
    before = schema_of(path)

    # 실패 주입 연결
    def connect(p, **kwargs):
        return FailingConnection(sqlite3.connect(p, **kwargs), {'fail_prefix': prefix})

    with pytest.raises(StoreCorrupt):
        TaskStore(path, connect=connect)
    assert schema_of(path) == before

    uri = Path(path).resolve().as_uri() + '?mode=ro'
    store = TaskStore(path, read_only=True,
                      connect=lambda _p, **kw: sqlite3.connect(uri, uri=True, **kw))
    assert store.schema_version == version and len(store.load_states()) == 2
    store.close()


# ---- 주행 goal UUID 주체(D1) ----

# 주행 단계에서는 D2 UUID를 만들지 않고 D1(포트) UUID를 기록한다. 로컬 작업은 D2 UUID를 먼저 예약
def test_navigation_uuid_made_by_d1(tmp_path):
    calls = []
    runtime, intake, mock, local, nav = make_world(tmp_path, uuid_calls=calls)
    task_id = receive_and_start(runtime, intake)
    assert calls == []
    rows = runtime.store.dispatch_rows(task_id)
    assert {r['goal_uuid'] for r in rows} == set(nav.goal_uuids.values())
    assert all(r['status'] == 'SENT' for r in rows)

    order = []
    original = local.dispatch

    # 로컬 작업 전송 순간의 원장 기록
    def checking(intent, goal_uuid):
        row = runtime.store.dispatch_row(intent.execution['execution_id'])
        order.append((row['status'], row['goal_uuid'] == goal_uuid, goal_uuid.startswith('d2')))
        return original(intent, goal_uuid)

    local.dispatch = checking
    mock.run()
    assert order and set(order) == {('RESERVED', True, True)}
    nav_rows = [r for r in runtime.store.dispatch_rows(task_id) if r['operation'] == 'NAVIGATE_TO']
    assert len(calls) == len(order) and {r['goal_uuid'] for r in nav_rows} == \
        set(nav.goal_uuids.values())


# D1 submit 뒤 UUID 기록 전 중단: 재시작에서 재전송 없이 미확정, UUID 기록 없음
def test_navigation_crash_before_uuid_record(tmp_path):
    runtime, intake, _, _, nav = make_world(tmp_path)
    result = intake.observe(obs('ob-1'))
    runtime.handle(result.task_id, AssignRoles(make_assignment('robot_m', 'robot_t')))

    # D1 전송은 끝났지만 D2가 UUID를 기록하기 전에 멈춘다
    def crash(execution_id, goal_uuid):
        raise SimulatedCrash()

    runtime.store.record_submitted = crash
    with pytest.raises(SimulatedCrash):
        runtime.handle(result.task_id, StartTask())
    assert len(nav.requests) == 1
    runtime.store.close()

    runtime2, _, _, local2, nav2 = make_world(tmp_path)
    state = runtime2.registry.states[result.task_id]
    run = [r for r in state.steps.values() if r.status == 'UNCERTAIN']
    assert len(run) == 1 and run[0].goal_uuid == ''
    assert runtime2.store.dispatch_row(run[0].execution_id)['status'] == 'SUBMITTING'
    runtime2.handle(result.task_id, Tick())
    assert nav2.requests == [] and local2.requests == []


# 주행 결과는 execution_id로도 결합한다. 로컬 작업 결과는 execution_id만으로 결합하지 않는다
def test_navigation_result_by_execution_id():
    flow = Flow()
    flow.start()
    nav = flow.unsent()[0]
    flow.send(nav)
    decision = flow.apply(StepResult('', nav.robot_id, gm.OUTCOME_SUCCEEDED, MOCK,
                                     execution_id=nav.execution['execution_id']))
    assert flow.state.steps[nav.step_key].status == 'SUCCEEDED'
    assert 'UNBOUND_RESULT' not in decision.reasons

    local_flow = Flow()
    run_until(local_flow, 'src_nav')
    lock = local_flow.unsent()[0]
    local_flow.send(lock)
    decision = local_flow.apply(StepResult('', lock.robot_id, gm.OUTCOME_SUCCEEDED, MOCK,
                                           execution_id=lock.execution['execution_id']))
    assert 'UNBOUND_RESULT' in decision.reasons


# 실제 D1 실행 원장 모듈(src/ax_navigation, 수정 없이 읽기만). D1 폴더에 바이트코드 캐시를 쓰지 않는다
def load_d1_execution():
    path = ROOT / 'src' / 'ax_navigation' / 'scripts' / 'navigation_execution.py'
    spec = importlib.util.spec_from_file_location('d1_navigation_execution', path)
    module = importlib.util.module_from_spec(spec)
    previous = sys.dont_write_bytecode
    sys.dont_write_bytecode = True
    try:
        spec.loader.exec_module(module)
    finally:
        sys.dont_write_bytecode = previous

    return module


class FakeD1Client:
    """NavigationClient.submit과 같은 인자로 실제 D1 NavigationExecution을 쓴다(Nav2 없음)."""

    # D1 원장과 실행 관리자
    def __init__(self, module, path, robot):
        self.ledger = module.ExecutionLedger(path)
        self.manager = module.NavigationExecution(self.ledger)
        self.robot = robot
        self.sent = []

    # NavigationClient.submit 시그니처
    def submit(self, *, execution_id, assignment_id, assignment_generation, x, y, yaw):
        return self.manager.submit(self.sent.append, execution_id=execution_id, robot=self.robot,
                                   assignment_id=assignment_id,
                                   assignment_generation=assignment_generation, x=x, y=y, yaw=yaw)

    # 살아 있는 goal handle 없음
    def cancel(self, execution_id):
        raise ValueError('No live owned goal handle')


# 주행 의도 하나(robot_1 조작 역할, station_dst 정차)
def nav_intent():
    spec = make_spec('task_d1', 'obj-d1', source_location_id='station_src',
                     destination_location_id='station_dst')
    state = gm.new_group_state(spec)
    flow = Flow(manipulator='robot_1', transporter='robot_2', spec=spec)
    flow.state = state
    flow.start()

    return [i for i in flow.unsent() if i.robot_id == 'robot_1'][0]


# D1 어댑터: D2 UUID 거부, 같은 execution_id 재호출은 같은 D1 UUID·D1 전송 1회, 지점 revision 대조
def test_d1_adapter_with_real_d1_ledger(tmp_path):
    module = load_d1_execution()
    client = FakeD1Client(module, str(tmp_path / 'd1.sqlite'), 'robot_1')
    port = D1NavigationPort({'robot_1': client}, CATALOG)
    assert not port.supported(gm.MODE_DEVELOPMENT_MOCK)
    assert D1NavigationPort({}, CATALOG, enabled=True).supported(gm.MODE_DEVELOPMENT_MOCK)

    intent = nav_intent()
    with pytest.raises(PortError) as error:
        port.dispatch(intent, 'd2-made-uuid')
    assert error.value.uncertain is False

    first = port.dispatch(intent)
    again = port.dispatch(intent)
    assert first == again and len(client.sent) == 1
    row = client.ledger.get(intent.execution['execution_id'])
    assert row['goal_uuid'] == first
    request = json.loads(row['request'])
    pose = CATALOG.get('station_src')['stop_poses']['MANIPULATOR']
    assert (request['x'], request['y']) == (pose['x'], pose['y'])

    with pytest.raises(PortError) as error:
        port.cancel(intent)
    assert error.value.uncertain is True

    stale = intent.__class__(**dict(vars(intent), args=dict(intent.args, location_revision=9)))
    with pytest.raises(PortError) as error:
        port.dispatch(stale)
    assert error.value.reason == 'LOCATION_REVISION_MISMATCH'


# D1 원장 상태 → D2 결과 사건(execution_id 결합), 종료 전 상태는 결과 아님
def test_d1_result_mapping():
    row = {'execution_id': 'e1', 'goal_uuid': 'g1', 'state': 'ARRIVED', 'detail': ''}
    event = d1_result_event(row, 'robot_1')
    assert (event.outcome, event.execution_id, event.provenance) == (
        gm.OUTCOME_SUCCEEDED, 'e1', gm.PROVENANCE_LIVE)
    assert d1_result_event(dict(row, state='ACTIVE'), 'robot_1') is None
    assert d1_result_event(dict(row, state='UNKNOWN'), 'robot_1').outcome == \
        gm.OUTCOME_UNCERTAIN


# A4 ActuationState 변환: state만 정지 근거, 허가 필드는 permits. LIVE 출처는 개발 모드 근거가 아니다
def test_actuation_observation_adapter():
    fields = {'robot_id': 'robot_m7', 'state': 1, 'drive_allowed': False, 'arm_allowed': False,
              'meta': {'source_id': 'a4', 'source_session_id': 's1', 'revision': 3,
                       'valid': True, 'stamp_s': 100.0}}
    obs_running = actuation_observation(fields, 100.0, provenance=MOCK)
    assert obs_running.value == 1 and ('drive_allowed', False) in obs_running.permits

    flow = Flow()
    flow.start()
    flow.send_all()
    flow.finish('src_nav_manipulator')
    flow.finish('src_nav_transporter')
    for robot in flow.assignment.robots():
        flow.apply(ObservationReceived(actuation_observation(
            dict(fields, robot_id=robot), flow.t, provenance=MOCK)))
    assert not flow.intended('src_lock_manipulator')

    live = actuation_observation(dict(fields, state=gm.ACTUATION_STOPPED), flow.t)
    decision = apply_event(flow.state, ObservationReceived(live), flow.now())
    assert 'PROVENANCE_REJECTED' in decision.reasons


# C4 ToolHoldState 필드 시험용 사전(정상 HELD). 물체는 make_spec 기본값과 같다
def _hold_fields(**changes):
    meta = {'source_id': 'c4_manipulate_object_server', 'source_session_id': 'c4_s1',
            'revision': 5, 'valid': True, 'stamp_s': 100.0}
    meta.update(changes.pop('meta', {}))
    fields = {'robot_id': 'robot_m7', 'object_id': 'box_dev_1', 'state': gm.HOLD_HELD,
              'evidence': 'ATTACH_SIGNAL', 'meta': meta}
    fields.update(changes)

    return fields


# 정상 변환: HELD·RELEASED·UNKNOWN 모두 관측이 되고 UNKNOWN은 HELD·RELEASED가 아니다. 기본 출처는 LIVE
@pytest.mark.parametrize('state', [gm.HOLD_HELD, gm.HOLD_RELEASED, gm.HOLD_UNKNOWN])
def test_hold_observation_states(state):
    obs, reason = hold_observation(_hold_fields(state=state), 100.2)

    assert reason is None
    assert (obs.kind, obs.robot_id, obs.object_id, obs.value) == (
        'HOLD', 'robot_m7', 'box_dev_1', state)
    assert (obs.source_id, obs.session_id, obs.revision, obs.valid) == (
        'c4_manipulate_object_server', 'c4_s1', 5, True)
    assert (obs.stamp_sim_s, obs.received_boottime_s) == (100.0, 100.2)
    assert obs.provenance == gm.PROVENANCE_LIVE
    assert obs.permits == ()


# 형식 거절: 관측을 만들지 않고 사유 코드를 돌려준다
@pytest.mark.parametrize('changes, reason', [
    ({'state': 3}, 'HOLD_STATE_UNKNOWN_VALUE'),
    ({'state': True}, 'HOLD_STATE_UNKNOWN_VALUE'),
    ({'robot_id': ''}, 'HOLD_ROBOT_ID_EMPTY'),
    ({'object_id': ''}, 'HOLD_OBJECT_ID_EMPTY'),
    ({'meta': {'source_id': ''}}, 'HOLD_SOURCE_EMPTY'),
    ({'meta': {'source_session_id': ''}}, 'HOLD_SOURCE_EMPTY'),
    ({'meta': {'revision': 0}}, 'HOLD_REVISION_UNSET'),
    ({'evidence': None}, 'HOLD_EVIDENCE_INVALID'),
    ({'evidence': 'MOCK_ATTACH_STATE'}, 'HOLD_MOCK_EVIDENCE'),
    ({'evidence': 'MOCK'}, 'HOLD_MOCK_EVIDENCE'),
])
def test_hold_observation_rejections(changes, reason):
    fields = _hold_fields(**changes)

    assert hold_rejection(fields) == reason
    assert hold_observation(fields, 100.0) == (None, reason)


# valid=false·빈 근거는 형식 거절이 아니다(관측으로 넘기고 group_machine이 판정한다)
def test_hold_observation_passes_invalid_through():
    obs, reason = hold_observation(_hold_fields(evidence='', meta={'valid': False}), 100.0)

    assert reason is None and obs.valid is False


# ROS 메시지(duck-typing) → 필드 사전. stamp는 sec + nanosec
def test_hold_fields_from_message():
    from types import SimpleNamespace as NS

    stamp = NS(sec=100, nanosec=250_000_000)
    meta = NS(header=NS(stamp=stamp), source_id='c4', source_session_id='s1',
              revision=7, valid=True)
    msg = NS(meta=meta, robot_id='robot_m7', object_id='box_dev_1', state=2,
             evidence='ATTACH_SIGNAL')

    fields = hold_fields(msg)

    assert fields == {'robot_id': 'robot_m7', 'object_id': 'box_dev_1', 'state': 2,
                      'evidence': 'ATTACH_SIGNAL',
                      'meta': {'source_id': 'c4', 'source_session_id': 's1', 'revision': 7,
                               'valid': True, 'stamp_s': 100.25}}
    assert hold_rejection(fields) is None


# 출발지 해제 직전까지 진행하고 팔 보유 외 근거(적재·후퇴·정지)를 넣는다
def _flow_before_source_release():
    flow = Flow()
    run_until(flow, 'confirm')
    flow.send_all()
    flow.wait(0.05)
    flow.evidence('stowed', 'retracted', 'stopped')

    return flow


# 어댑터 RELEASED 관측(모의 출처로 주입)이 group_machine 판정 경로를 거친다. 정상 관측만 해제로 이어진다
@pytest.mark.parametrize('case', ['ok', 'invalid', 'stale', 'other_object', 'before_anchor',
                                  'unknown'])
def test_hold_adapter_through_group_machine(case):
    flow = _flow_before_source_release()
    meta = {'stamp_s': flow.t, 'valid': case != 'invalid'}
    if case == 'stale':
        meta['stamp_s'] = flow.t - 1.0
    changes = {'state': gm.HOLD_UNKNOWN if case == 'unknown' else gm.HOLD_RELEASED,
               'meta': meta}
    if case == 'other_object':
        changes['object_id'] = 'box_other'
    received = flow.t - 0.01 if case == 'before_anchor' else flow.t

    obs, reason = hold_observation(_hold_fields(**changes), received, provenance=MOCK)
    assert reason is None
    decision = flow.apply(ObservationReceived(obs))
    if case == 'other_object':
        assert 'OBSERVATION_OBJECT_MISMATCH' in decision.reasons
    flow.finish('load_retract')

    assert flow.intended('src_unlock_manipulator') is (case == 'ok')


# 개발 모드에서 LIVE 출처 보유 관측은 판정 근거가 아니다(PROVENANCE_REJECTED, 해제 의도 없음)
def test_hold_adapter_live_provenance_not_used_in_development():
    flow = _flow_before_source_release()
    obs, _ = hold_observation(_hold_fields(state=gm.HOLD_RELEASED,
                                           meta={'stamp_s': flow.t}), flow.t)

    decision = flow.apply(ObservationReceived(obs))
    flow.finish('load_retract')

    assert 'PROVENANCE_REJECTED' in decision.reasons
    assert ('HOLD', 'robot_m7') not in flow.state.observations
    assert not flow.intended('src_unlock_manipulator')


# 카탈로그 검증
@pytest.mark.parametrize('change', [
    lambda d: d['locations']['zone_in'].update(purpose='DOCK'),
    lambda d: d['locations']['zone_in'].update(capacity=-1),
    lambda d: d['locations']['zone_in'].update(wait_location_id='nowhere'),
    lambda d: d['locations']['zone_in']['stop_poses'].update(DRIVER={'x': 0, 'y': 0, 'yaw': 0}),
    lambda d: d['locations']['zone_in']['pose'].update(x=float('nan')),
    lambda d: d.update(extra=1),
])
def test_catalog_validation(change):
    data = catalog_dict(PURPOSES)
    change(data)
    with pytest.raises(gm.SpecError):
        inv.parse_catalog(data)


# 입고 구역 용도가 RECEIPT가 아니거나 카탈로그 revision이 registry와 다르면 접수기 생성 거부
def test_intake_catalog_mismatch(tmp_path):
    store = TaskStore(str(tmp_path / 'd2.sqlite'))
    runtime = GroupRuntime(store, {}, scenarios.Clock(), map_revision=1,
                           locations=dict(CATALOG.revisions(), zone_in=2))
    with pytest.raises(gm.SpecError):
        ReceiptIntake(runtime, CONFIG, CATALOG)

    data = catalog_dict(dict(PURPOSES, zone_in='STORAGE'))
    runtime2 = GroupRuntime(store, {}, scenarios.Clock(), map_revision=1,
                            locations=inv.parse_catalog(data).revisions())
    with pytest.raises(gm.SpecError):
        ReceiptIntake(runtime2, CONFIG, inv.parse_catalog(data))
    store.close()
