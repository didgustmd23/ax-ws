"""D2 W4 자동 입고 등록 시험(ROS 없음, 모의 관측).

반복 관측·식별 불명·재전송 멱등·재입고·구역 충돌·목적지 규칙·트랜잭션 실패·schema 1 이주를 본다.
실행: .venv/bin/python -m pytest -q -p no:launch_testing -p no:launch_ros \
      tools/tests/test_d2_receipt.py
"""
import itertools
import json
import sqlite3
import sys
import uuid
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src' / 'ax_fleet_manager'))

from ax_fleet_manager import group_model as gm  # noqa: E402
from ax_fleet_manager import receipt as rc  # noqa: E402
from ax_fleet_manager.dev_fixtures import (  # noqa: E402
    catalog_dict, clock, make_assignment, make_spec,
)
from ax_fleet_manager.inventory import parse_catalog  # noqa: E402
from ax_fleet_manager.group_events import (  # noqa: E402
    AssignRoles, CancelRequested, FailTerminate, StartTask,
)
from ax_fleet_manager.intake import ReceiptIntake  # noqa: E402
from ax_fleet_manager.ports import MockLocalTaskPort, MockNavigationPort  # noqa: E402
from ax_fleet_manager.runtime import GroupRuntime  # noqa: E402
from ax_fleet_manager.task_store import (  # noqa: E402
    RECEIPT_TABLES, V1_DDL, StoreCorrupt, TaskStore, state_to_dict,
)

LOCATIONS = {'zone_in_a': 1, 'zone_in_b': 1, 'station_dst': 1, 'station_src': 1}
CONFIG = rc.ReceiptConfig(receipt_zones=('zone_in_a', 'zone_in_b'),
                          destination_rules={'zone_in_a': 'station_dst'})
# W5: 접수기는 등록 지점 카탈로그를 받는다. W4 시험은 용량·종류 제한이 없는 지점으로 기존 판정을 본다
CATALOG = parse_catalog(catalog_dict({'zone_in_a': 'RECEIPT', 'zone_in_b': 'RECEIPT',
                                      'station_dst': 'STORAGE', 'station_src': 'STATION'}))


class Clock:
    """주입 시각."""

    # 시작 시각
    def __init__(self):
        self.t = 300.0
        self.valid = True

    # 현재 시각
    def __call__(self):
        return clock(self.t, time_valid=self.valid)


class FailingConnection:
    """지정 SQL 앞부분에서 실패하는 연결(쓰기 실패 주입)."""

    # 실제 연결과 실패 조건(SQL 접두어)을 감싼다
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


# 결정적 UUID 발급기(시험 재현용)
def counter_uuid():
    counter = itertools.count(1)
    return lambda: str(uuid.UUID(int=next(counter)))


# runtime·접수기 묶음
def make_intake(tmp_path, config=CONFIG, connect=None, clk=None):
    clk = clk or Clock()
    kwargs = {'connect': connect} if connect else {}
    store = TaskStore(str(tmp_path / 'd2.sqlite'), **kwargs)
    local, nav = MockLocalTaskPort('receipt'), MockNavigationPort('receipt')
    runtime = GroupRuntime(store, {'LOCAL_TASK': local, 'NAVIGATION': nav}, clk,
                           map_revision=1, locations=LOCATIONS)

    return ReceiptIntake(runtime, config, CATALOG, uuid_factory=counter_uuid()), runtime, clk


# 입고 관측 사전(ReceiptObservation 필드)
def obs(observation_id, object_id='obj-box-1', zone='zone_in_a', confidence=0.95,
        source=rc.SOURCE_SENSOR, valid=True, stamp=300.0):
    return {
        'observation_id': observation_id, 'zone_id': zone, 'object_id': object_id,
        'object_class': 'box_small', 'identity_confidence': confidence, 'source': source,
        'source_id': 'c1_mock', 'valid': valid, 'stamp_sim_s': stamp,
    }


# 등록 요청 사전(RegisterTask 필드)
def request(request_id, object_id='obj-box-9', source='zone_in_a', destination='',
            receipt_event_id=''):
    return {'request_id': request_id, 'receipt_event_id': receipt_event_id,
            'object_id': object_id, 'source_location_id': source,
            'destination_location_id': destination}


# 원장에 기록된 작업 ID 목록
def task_ids(runtime):
    return sorted(s.spec.task_id for s in runtime.store.load_states())


# 반복 관측: 사건 1개·작업 1개, 관측 ID는 같은 사건에 덧붙는다
def test_repeated_observation_creates_one_task(tmp_path):
    intake, runtime, _ = make_intake(tmp_path)
    results = [intake.observe(obs(f'ob-{i}')) for i in range(5)]

    assert [r.decision for r in results] == ['TASK_CREATED'] + ['DUPLICATE'] * 4
    assert len({r.receipt_event_id for r in results}) == 1
    assert len(task_ids(runtime)) == 1

    (event,) = runtime.store.receipt_events()
    assert event['state'] == rc.RECEIPT_TASK_CREATED
    assert event['observation_ids'] == [f'ob-{i}' for i in range(5)]
    assert event['task_id'] == results[0].task_id
    assert event['receipt_event_id'].startswith('rcpt-')

    state = runtime.registry.states[event['task_id']]
    assert state.task_state == gm.TASK_QUEUED
    assert state.spec.source_location_id == 'zone_in_a'
    assert state.spec.destination_location_id == 'station_dst'
    assert state.spec.input_source_id == f'd2_receipt:{event["receipt_event_id"]}'
    assert state.spec.source_transfer_id.startswith('xfer-')
    assert state.spec.source_transfer_id != state.spec.destination_transfer_id
    assert state.spec.steps == gm.standard_transfer_plan()


# 식별 불명: 확인 대기 사건만, 작업 0. 사용자 결정(해석 2 "나")에 따라 같은 구역의 식별 불명은
# 대기 사건 1개에 관측 ID로 묶고(이전 기대값: 관측마다 사건 1개), 다른 구역은 별도 사건이다.
# 후보 ID가 있어도 object_id는 빈 값으로 두어 물체 ID와 합치지 않는다
def test_unidentified_observation_is_pending_and_not_merged(tmp_path):
    intake, runtime, _ = make_intake(tmp_path)
    first = intake.observe(obs('ob-1', object_id=''))
    low = intake.observe(obs('ob-2', object_id='obj-box-1', confidence=0.79))
    low2 = intake.observe(obs('ob-3', object_id='obj-box-1', confidence=0.5))
    other_zone = intake.observe(obs('ob-4', object_id='', zone='zone_in_b'))

    assert [r.decision for r in (first, low, low2, other_zone)] == ['PENDING'] * 4
    assert first.receipt_event_id == low.receipt_event_id == low2.receipt_event_id
    assert other_zone.receipt_event_id != first.receipt_event_id
    assert task_ids(runtime) == []

    events = {e['receipt_event_id']: e for e in runtime.store.receipt_events()}
    assert len(events) == 2
    assert events[first.receipt_event_id]['observation_ids'] == ['ob-1', 'ob-2', 'ob-3']
    assert events[other_zone.receipt_event_id]['zone_id'] == 'zone_in_b'
    assert all(e['object_id'] == '' and e['state'] == rc.RECEIPT_PENDING_IDENTIFICATION
               for e in events.values())

    # 임계값 이상으로 식별되면 확인 대기 사건과 합치지 않고 새 입고가 된다
    ok = intake.observe(obs('ob-5', object_id='obj-box-1', confidence=0.8))
    assert ok.decision == 'TASK_CREATED'
    assert ok.receipt_event_id not in events


# 대기 사건의 관측 ID 상한: 넘은 관측은 관측 기록에만 남고 사건은 그대로(식별 사건 DUPLICATE와 같음).
# 이전 시험(전역 대기 사건 상한 256·PENDING_IDENTIFICATION_LIMIT)은 구역별 1개 규칙으로 필요 없어져 교체
def test_pending_observation_id_cap(tmp_path):
    config = rc.ReceiptConfig(receipt_zones=('zone_in_a',),
                              destination_rules={'zone_in_a': 'station_dst'},
                              max_observation_ids=2)
    intake, runtime, _ = make_intake(tmp_path, config=config)
    results = [intake.observe(obs(f'ob-{i}', object_id='')) for i in range(3)]

    assert [r.decision for r in results] == ['PENDING'] * 3
    assert len({r.receipt_event_id for r in results}) == 1
    (event,) = runtime.store.receipt_events()
    assert event['observation_ids'] == ['ob-0', 'ob-1']
    assert runtime.store.receipt_observation('ob-2')['receipt_event_id'] == \
        event['receipt_event_id']


# 같은 구역 식별 불명 300회: 대기 사건 1개, 거부 없음. 이후 같은 구역 식별 관측은 정상 신규 입고
def test_many_unidentified_stay_one_pending_event(tmp_path):
    intake, runtime, _ = make_intake(tmp_path)
    results = [intake.observe(obs(f'ob-{i}', object_id='')) for i in range(300)]

    assert {r.decision for r in results} == {'PENDING'}
    assert len({r.receipt_event_id for r in results}) == 1
    (pending,) = runtime.store.receipt_events()
    assert len(pending['observation_ids']) == CONFIG.max_observation_ids

    created = intake.observe(obs('ob-id-1', object_id='obj-box-7'))
    assert created.decision == 'TASK_CREATED'
    assert created.receipt_event_id != pending['receipt_event_id']
    assert len(task_ids(runtime)) == 1
    assert runtime.store.receipt_event(pending['receipt_event_id'])['observation_ids'] == \
        pending['observation_ids']


# 재시작 뒤 같은 구역 식별 불명은 기존 대기 사건에 붙는다
def test_restart_unidentified_joins_pending_event(tmp_path):
    intake, runtime, _ = make_intake(tmp_path)
    first = intake.observe(obs('ob-1', object_id=''))
    runtime.store.close()

    intake2, runtime2, _ = make_intake(tmp_path)
    again = intake2.observe(obs('ob-2', object_id=''))
    assert again.decision == 'PENDING'
    assert again.receipt_event_id == first.receipt_event_id
    (event,) = runtime2.store.receipt_events()
    assert event['observation_ids'] == ['ob-1', 'ob-2']


# 같은 관측 ID 재전송: 같은 내용은 처음 판정 재현, 다른 내용은 DUPLICATE_REQUEST. 작업은 1개
def test_observation_retransmit_idempotent(tmp_path):
    intake, runtime, _ = make_intake(tmp_path)
    first = intake.observe(obs('ob-1'))
    again = intake.observe(obs('ob-1'))
    changed = intake.observe(obs('ob-1', confidence=0.9))

    assert again.replayed and (again.decision, again.receipt_event_id, again.task_id) == (
        first.decision, first.receipt_event_id, first.task_id)
    assert (changed.decision, changed.reason_code) == ('REJECT', rc.DUPLICATE_REQUEST)
    assert len(task_ids(runtime)) == 1
    assert runtime.store.receipt_events()[0]['observation_ids'] == ['ob-1']


# 등록 요청 멱등: 같은 ID·같은 내용은 같은 응답, 같은 ID·다른 내용과 같은 사건의 다른 ID는 거절
def test_register_request_idempotent(tmp_path):
    intake, runtime, _ = make_intake(tmp_path)
    accepted = intake.register_task(request('req-a'))
    assert accepted['accepted'] and accepted['task_id'].startswith('task-')
    assert intake.register_task(request('req-a')) == accepted
    assert intake.register_task(request('req-a', destination='station_src')) == {
        'accepted': False, 'task_id': '', 'reason_code': rc.DUPLICATE_REQUEST}
    assert len(task_ids(runtime)) == 1

    created = intake.observe(obs('ob-1'))
    other = intake.register_task(request('req-b', object_id='obj-box-1',
                                         receipt_event_id=created.receipt_event_id))
    assert other == {'accepted': False, 'task_id': created.task_id,
                     'reason_code': rc.DUPLICATE_REQUEST}

    internal = intake.register_task({
        'request_id': rc.receipt_request_id(created.receipt_event_id),
        'receipt_event_id': created.receipt_event_id, 'object_id': 'obj-box-1',
        'source_location_id': 'zone_in_a', 'destination_location_id': ''})
    assert internal == {'accepted': True, 'task_id': created.task_id, 'reason_code': ''}
    assert len(task_ids(runtime)) == 2

    unknown = intake.register_task(request('req-c', receipt_event_id='rcpt-none'))
    assert unknown['reason_code'] == 'RECEIPT_EVENT_UNKNOWN'


# 재시작 뒤에도 같은 관측·같은 물품 관측은 새 작업을 만들지 않는다
def test_restart_keeps_one_task_per_receipt(tmp_path):
    intake, runtime, _ = make_intake(tmp_path)
    first = intake.observe(obs('ob-1'))
    runtime.store.close()

    intake2, runtime2, _ = make_intake(tmp_path)
    replay = intake2.observe(obs('ob-1'))
    repeat = intake2.observe(obs('ob-2'))
    assert replay.replayed and replay.task_id == first.task_id
    assert repeat.decision == 'DUPLICATE' and repeat.receipt_event_id == first.receipt_event_id
    assert task_ids(runtime2) == [first.task_id]
    assert 'RESTORED_UNRECONCILED' in runtime2.registry.states[first.task_id].inhibit


# 재입고: 이전 사건 작업이 종료되면 새 회차 사건을 REJECTED로만 남기고 작업 0, 그 뒤 관측은 DUPLICATE
@pytest.mark.parametrize('terminal', ['failed', 'canceled'])
def test_rereceipt_unconfirmed(tmp_path, terminal):
    intake, runtime, _ = make_intake(tmp_path)
    first = intake.observe(obs('ob-1'))
    if terminal == 'failed':
        runtime.handle(first.task_id, FailTerminate('fail-rr-1'))
    else:
        runtime.handle(first.task_id, CancelRequested())
    assert runtime.registry.states[first.task_id].task_state in gm.TERMINAL_TASK_STATES

    again = intake.observe(obs('ob-2'))
    assert (again.decision, again.reason_code) == ('REJECTED', rc.RERECEIPT_UNCONFIRMED)
    assert again.task_id == '' and again.receipt_event_id != first.receipt_event_id
    event = runtime.store.receipt_event(again.receipt_event_id)
    assert (event['receipt_round'], event['state']) == (2, rc.RECEIPT_REJECTED)
    assert len(task_ids(runtime)) == 1

    later = intake.observe(obs('ob-3'))
    assert later.decision == 'DUPLICATE' and later.receipt_event_id == again.receipt_event_id
    assert len(runtime.store.receipt_events()) == 2


# 열린 사건이 있는 물체가 다른 입고 구역에서 보이면 합치지 않고 충돌로 거부한다
def test_zone_conflict_rejected(tmp_path):
    intake, runtime, _ = make_intake(tmp_path)
    first = intake.observe(obs('ob-1', zone='zone_in_a'))
    other = intake.observe(obs('ob-2', zone='zone_in_b'))

    assert (other.decision, other.reason_code) == ('REJECT', rc.RECEIPT_ZONE_CONFLICT)
    assert runtime.store.receipt_event(first.receipt_event_id)['observation_ids'] == ['ob-1']
    assert len(runtime.store.receipt_events()) == 1
    assert len(task_ids(runtime)) == 1


# 관측 자체의 거부: 무효·출처 불명·입고 구역 아님·만료·미래. 정답 기반은 개발 모드에서만, 출처 기록
@pytest.mark.parametrize('changes,reason', [
    ({'valid': False}, 'OBSERVATION_INVALID'),
    ({'source': rc.SOURCE_UNKNOWN}, 'SOURCE_UNKNOWN'),
    ({'zone': 'station_dst'}, rc.ZONE_NOT_RECEIPT),
    ({'stamp': 297.9}, 'OBSERVATION_STALE'),
    ({'stamp': 300.1}, 'OBSERVATION_FROM_FUTURE'),
])
def test_observation_rejections(tmp_path, changes, reason):
    intake, runtime, _ = make_intake(tmp_path)
    result = intake.observe(obs('ob-1', **changes))

    assert (result.decision, result.reason_code) == ('REJECT', reason)
    assert runtime.store.receipt_events() == []
    assert task_ids(runtime) == []


# 경계값: 나이 2.0 s는 받고, 정답 기반 관측은 출처를 사건에 남긴다
def test_age_boundary_and_ground_truth(tmp_path):
    intake, runtime, _ = make_intake(tmp_path)
    result = intake.observe(obs('ob-1', stamp=298.0, source=rc.SOURCE_GROUND_TRUTH))
    assert result.decision == 'TASK_CREATED'
    assert runtime.store.receipt_events()[0]['provenance'] == 'GROUND_TRUTH'

    strict = rc.ReceiptConfig(receipt_zones=('zone_in_a',), allow_ground_truth=False)
    other = tmp_path / 'strict'
    other.mkdir()
    intake2, _, _ = make_intake(other, config=strict)
    rejected = intake2.observe(obs('ob-2', source=rc.SOURCE_GROUND_TRUTH))
    assert rejected.reason_code == 'PROVENANCE_REJECTED'


# 시각 무효면 기록하지 않아 회복 뒤 같은 관측을 다시 판정한다
def test_time_invalid_not_recorded(tmp_path):
    intake, runtime, clk = make_intake(tmp_path)
    clk.valid = False
    assert intake.observe(obs('ob-1')).reason_code == 'TIME_INVALID'
    clk.valid = True
    assert intake.observe(obs('ob-1')).decision == 'TASK_CREATED'


# 형식 오류 관측은 기록하지 않고 거부한다
@pytest.mark.parametrize('changes', [
    {'observation_id': ''}, {'identity_confidence': 1.5},
    {'identity_confidence': float('nan')}, {'extra': 1},
])
def test_malformed_observation(tmp_path, changes):
    intake, runtime, _ = make_intake(tmp_path)
    data = dict(obs('ob-1'), **changes)
    assert intake.observe(data).decision == 'REJECT'
    assert runtime.store.receipt_events() == []


# 목적지 규칙이 없는 입고 구역: 사건은 REJECTED·NO_DESTINATION_RULE, 작업 0
def test_no_destination_rule(tmp_path):
    intake, runtime, _ = make_intake(tmp_path)
    result = intake.observe(obs('ob-1', zone='zone_in_b'))

    assert (result.decision, result.reason_code) == ('REJECTED', rc.NO_DESTINATION_RULE)
    event = runtime.store.receipt_event(result.receipt_event_id)
    assert event['state'] == rc.RECEIPT_REJECTED and event['task_id'] == ''
    assert task_ids(runtime) == []
    assert intake.observe(obs('ob-2', zone='zone_in_b')).decision == 'DUPLICATE'


# 출발 지점 규칙: 입고 사건 작업은 출발 = 입고 구역, 수동 요청은 출발 필수
def test_source_mismatch(tmp_path):
    event = {'zone_id': 'zone_in_a'}
    assert rc.resolve_locations(request('r', source='station_src'), event, CONFIG)[2] == \
        rc.SOURCE_MISMATCH
    assert rc.resolve_locations(request('r', source='zone_in_a'), event, CONFIG) == \
        ('zone_in_a', 'station_dst', '')

    intake, runtime, _ = make_intake(tmp_path)
    assert intake.register_task(request('req-a', source=''))['reason_code'] == \
        rc.SOURCE_MISMATCH
    assert intake.register_task(request('req-b', source='station_src'))['reason_code'] == \
        rc.NO_DESTINATION_RULE
    manual = intake.register_task(request('req-c', source='station_src',
                                          destination='station_dst'))
    assert manual['accepted']
    spec = runtime.registry.states[manual['task_id']].spec
    assert spec.input_source_id == 'd2_register:req-c'


# 진행 중 작업이 쓰는 물체의 입고 관측: OBJECT_BUSY로 거절, 작업 추가 없음
def test_object_busy(tmp_path):
    intake, runtime, _ = make_intake(tmp_path)
    manual = intake.register_task(request('req-a', object_id='obj-box-1'))
    assert manual['accepted']

    result = intake.observe(obs('ob-1', object_id='obj-box-1'))
    assert (result.decision, result.reason_code) == ('REJECTED', rc.OBJECT_BUSY)
    assert task_ids(runtime) == [manual['task_id']]


# 등록된 작업은 QUEUED이며 배정은 명시 입력이다. 배정·시작 뒤 기존 엔진 흐름으로 첫 주행 의도가 나온다
def test_registered_task_runs_through_engine(tmp_path):
    intake, runtime, _ = make_intake(tmp_path)
    created = intake.observe(obs('ob-1'))
    nav = runtime.ports['NAVIGATION']
    assert nav.requests == []

    runtime.handle(created.task_id, AssignRoles(make_assignment('robot_m', 'robot_t')))
    runtime.handle(created.task_id, StartTask())
    assert sorted(r['step_key'] for r in nav.requests) == [
        'src_nav_manipulator', 'src_nav_transporter']


# 트랜잭션 원자성: 작업 행 기록이 실패하면 관측·사건·요청도 남지 않고 registry도 그대로, runtime 고장
@pytest.mark.parametrize('prefix', ['INSERT INTO tasks', 'INSERT INTO register_requests',
                                    'INSERT INTO receipt_observations'])
def test_intake_transaction_failure_leaves_nothing(tmp_path, prefix):
    flags = {}

    # 실패 주입 연결
    def connect(path, **kwargs):
        return FailingConnection(sqlite3.connect(path, **kwargs), flags)

    intake, runtime, _ = make_intake(tmp_path, connect=connect)
    flags['fail_prefix'] = prefix
    result = intake.observe(obs('ob-1'))

    assert (result.decision, result.reason_code) == ('REJECT', 'STORE_WRITE_FAILED')
    assert runtime.fault
    assert runtime.registry.states == {}
    assert intake.observe(obs('ob-2')).reason_code == 'RUNTIME_STORE_FAULT'
    runtime.store.close()

    store = TaskStore(str(tmp_path / 'd2.sqlite'))
    assert store.receipt_events() == []
    assert store.receipt_observation('ob-1') is None
    assert store.load_states() == []
    assert store.db.execute('SELECT COUNT(*) FROM register_requests').fetchone()[0] == 0
    store.close()


# 등록 요청 단독 경로의 트랜잭션 실패
def test_register_transaction_failure(tmp_path):
    flags = {}

    # 실패 주입 연결
    def connect(path, **kwargs):
        return FailingConnection(sqlite3.connect(path, **kwargs), flags)

    intake, runtime, _ = make_intake(tmp_path, connect=connect)
    flags['fail_prefix'] = 'INSERT INTO register_requests'
    assert intake.register_task(request('req-a'))['reason_code'] == 'STORE_WRITE_FAILED'
    assert runtime.registry.states == {}
    runtime.store.close()

    store = TaskStore(str(tmp_path / 'd2.sqlite'))
    assert store.load_states() == [] and store.register_request('req-a') is None
    store.close()


# schema 1 원장 만들기(W3 형식)
def make_v1(path):
    db = sqlite3.connect(path, isolation_level=None)
    for ddl in V1_DDL:
        db.execute(ddl)
    db.execute("INSERT INTO meta VALUES ('schema_version', '1')")
    db.close()


# 원장의 표 이름과 schema 버전
def schema_of(path):
    db = sqlite3.connect(path)
    tables = {r[0] for r in db.execute("SELECT name FROM sqlite_master WHERE type='table'")}
    version = db.execute("SELECT value FROM meta WHERE key='schema_version'").fetchone()[0]
    db.close()

    return tables, version


# schema 1 이주: 기존 작업 보존, 입고 표 추가, 버전 2
def test_v1_migration_preserves_tasks(tmp_path):
    path = str(tmp_path / 'd2.sqlite')
    make_v1(path)
    db = sqlite3.connect(path, isolation_level=None)
    state = gm.new_group_state(make_spec())
    db.execute('INSERT INTO tasks VALUES (?, ?, ?)',
               (state.spec.task_id, state.revision, json.dumps(state_to_dict(state))))
    db.close()

    store = TaskStore(path)
    # W5: schema 3으로 이주(이전 기대값 2)
    assert store.schema_version == 4  # W6: schema 4
    assert [s.spec.task_id for s in store.load_states()] == ['task_dev_1']
    assert store.receipt_events() == []
    store.close()

    tables, version = schema_of(path)
    assert RECEIPT_TABLES <= tables and version == '4'  # W6: schema 4


# 이주 실패: 원장 불변(버전 1, 입고 표 없음)으로 거부하고, 읽기 전용 열기는 이주 없이 읽는다
@pytest.mark.parametrize('prefix', ['CREATE TABLE register_requests', 'UPDATE meta'])
def test_v1_migration_failure_leaves_store_unchanged(tmp_path, prefix):
    path = str(tmp_path / 'd2.sqlite')
    make_v1(path)

    # 실패 주입 연결
    def connect(p, **kwargs):
        return FailingConnection(sqlite3.connect(p, **kwargs), {'fail_prefix': prefix})

    with pytest.raises(StoreCorrupt):
        TaskStore(path, connect=connect)

    tables, version = schema_of(path)
    assert version == '1' and not tables & RECEIPT_TABLES

    uri = Path(path).resolve().as_uri() + '?mode=ro'
    store = TaskStore(path, read_only=True,
                      connect=lambda _p, **kw: sqlite3.connect(uri, uri=True, **kw))
    assert store.schema_version == 1 and store.load_states() == []
    store.close()
    assert schema_of(path)[1] == '1'


# uuid5 namespace는 고정 상수다: 같은 사건 ID는 재시작 전후 같은 요청 ID
def test_request_namespace_fixed():
    assert rc.RECEIPT_REQUEST_NAMESPACE == uuid.UUID('5f96e85c-52b2-5215-8fd5-4232de1a4c72')
    assert rc.receipt_request_id('rcpt-a') == rc.receipt_request_id('rcpt-a')
    assert rc.receipt_request_id('rcpt-a') == (
        f'req-{uuid.uuid5(rc.RECEIPT_REQUEST_NAMESPACE, "rcpt-a")}')
    assert rc.receipt_request_id('rcpt-a') != rc.receipt_request_id('rcpt-b')


# 설정 검증: 등록 지점에 없는 구역·규칙은 거부
@pytest.mark.parametrize('config', [
    rc.ReceiptConfig(receipt_zones=('zone_unknown',)),
    rc.ReceiptConfig(receipt_zones=('zone_in_a',), destination_rules={'zone_in_a': 'nowhere'}),
    rc.ReceiptConfig(receipt_zones=('zone_in_a',),
                     destination_rules={'station_src': 'station_dst'}),
    rc.ReceiptConfig(receipt_zones=('zone_in_a',), min_identity_confidence=0.0),
])
def test_config_validation(tmp_path, config):
    with pytest.raises(gm.SpecError):
        make_intake(tmp_path, config=config)
