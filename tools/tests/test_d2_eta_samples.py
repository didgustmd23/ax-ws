"""D2 W6 ETA 주행 표본(schema 4)·내보내기 시험(ROS 없음, 모의 상대).

실행: .venv/bin/python -m pytest -q -p no:launch_testing -p no:launch_ros \
      tools/tests/test_d2_eta_samples.py
"""
import csv
import json
import sqlite3
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src' / 'ax_fleet_manager'))
sys.path.insert(0, str(ROOT / 'tools'))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import export_d2_eta_samples as export  # noqa: E402
from ax_fleet_manager import group_model as gm  # noqa: E402
from ax_fleet_manager.dev_fixtures import make_assignment, make_spec  # noqa: E402
from ax_fleet_manager.group_events import AssignRoles, StartTask, Tick  # noqa: E402
from ax_fleet_manager.ports import MockLocalTaskPort, MockNavigationPort  # noqa: E402
from ax_fleet_manager.runtime import GroupRuntime  # noqa: E402
from ax_fleet_manager.task_store import (  # noqa: E402
    ETA_SAMPLE_FIELDS, RECEIPT_DDL, V1_DDL, V3_DDL, StoreCorrupt, StoreError, TaskStore,
    state_to_dict,
)
from test_d2_group_machine import Flow, run_until  # noqa: E402
from test_d2_inventory import FailingConnection  # noqa: E402
from test_d2_mixed_mode import MixedFlow  # noqa: E402
from test_d2_workflow_scenarios import LOCATIONS, Clock  # noqa: E402

NAV_STEPS = ('dst_nav_manipulator', 'dst_nav_transporter', 'src_nav_manipulator',
             'src_nav_transporter')
MISSING = ('path_length_m', 'turn_count', 'corridor_wait_s', 'obstacle_density')


# 다른 작업 ID·물체·실행 ID를 쓰는 작업 정의
def other_spec(name):
    return make_spec(f'task_{name}', f'box_{name}', root_execution_id=f'root_{name}',
                     source_transfer_id=f'xs_{name}', destination_transfer_id=f'xd_{name}')


# 흐름의 모든 판정을 원장에 차례로 기록한다(runtime과 같은 record_decision 경로)
def record_all(store, flow, start):
    store.create_task(start)
    previous = start

    for decision in flow.decisions:
        store.record_decision(flow.spec.task_id, previous.revision, decision, 'Event', {})
        previous = decision.next_state


# 흐름을 원장에 기록하기 위해 시작 상태를 함께 만든다
def flow_with_start(factory):
    flow = factory()
    return flow, flow.state.copy()


# v3 원장 만들기(작업 1개: 출발 주행이 이미 끝난 상태. 이주 시 표본을 백필하지 않는지 본다)
def make_v3_store(path):
    flow = Flow()
    run_until(flow, 'src_nav')
    db = sqlite3.connect(path, isolation_level=None)
    for ddl in V1_DDL + RECEIPT_DDL + V3_DDL:
        db.execute(ddl)
    db.execute("INSERT INTO meta VALUES ('schema_version', '3')")
    db.execute("INSERT INTO meta VALUES ('inventory_revision', '0')")
    db.execute('INSERT INTO tasks VALUES (?, ?, ?)',
               (flow.spec.task_id, flow.state.revision, json.dumps(state_to_dict(flow.state))))
    db.close()


# 표 이름·버전
def schema_of(path):
    db = sqlite3.connect(path)
    tables = {r[0] for r in db.execute("SELECT name FROM sqlite_master WHERE type='table'")}
    version = db.execute("SELECT value FROM meta WHERE key='schema_version'").fetchone()[0]
    db.close()

    return tables, version


# v3 → v4 이주: 표 추가·버전 4·작업 보존·과거 단계 표본 백필 없음
def test_migration_v3_to_v4(tmp_path):
    path = str(tmp_path / 'd2.sqlite')
    make_v3_store(path)

    store = TaskStore(path)
    assert store.schema_version == 4
    assert [s.spec.task_id for s in store.load_states()] == ['task_dev_1']
    assert store.eta_samples() == []
    store.close()

    tables, version = schema_of(path)
    assert 'eta_samples' in tables and version == '4'


# v1·v2 원장도 한 번에 v4로 이주한다
@pytest.mark.parametrize('version', [1, 2])
def test_migration_v1_v2_to_v4(tmp_path, version):
    from test_d2_inventory import make_old_store

    path = str(tmp_path / 'd2.sqlite')
    make_old_store(path, version)
    store = TaskStore(path)

    assert store.schema_version == 4 and store.eta_samples() == []
    assert len(store.inventory_entries()) == 1
    store.close()


# 이주 실패는 원장 불변, 읽기 전용 열기는 이주하지 않는다
@pytest.mark.parametrize('prefix', ['CREATE TABLE eta_samples', 'UPDATE meta'])
def test_migration_v3_failure_leaves_store(tmp_path, prefix):
    path = str(tmp_path / 'd2.sqlite')
    make_v3_store(path)
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
    assert store.schema_version == 3
    with pytest.raises(StoreError):
        store.eta_samples()
    store.close()


# 손상 거부: 버전과 표 구성이 맞지 않으면 열지 않는다
@pytest.mark.parametrize('case', ['v4_without_table', 'v3_with_table'])
def test_schema_table_mismatch_rejected(tmp_path, case):
    path = str(tmp_path / 'd2.sqlite')
    if case == 'v4_without_table':
        make_v3_store(path)
        db = sqlite3.connect(path, isolation_level=None)
        db.execute("UPDATE meta SET value='4' WHERE key='schema_version'")
    else:
        make_v3_store(path)
        db = sqlite3.connect(path, isolation_level=None)
        db.execute('CREATE TABLE eta_samples (execution_id TEXT PRIMARY KEY)')
    db.close()

    with pytest.raises(StoreCorrupt):
        TaskStore(path)


# 정상 흐름: 주행 4단계 표본, 결측 열 NULL, 시각·적재 상태
def test_samples_for_normal_flow(tmp_path):
    flow, start = flow_with_start(Flow)
    run_until(flow, 'dst_unlock')
    store = TaskStore(str(tmp_path / 'd2.sqlite'))
    record_all(store, flow, start)

    rows = {row['step_key']: row for row in store.eta_samples()}
    assert sorted(rows) == sorted(NAV_STEPS)

    for key, row in rows.items():
        run = flow.state.steps[key]
        assert row['execution_id'] == run.execution_id
        assert (row['status'], row['outcome'], row['provenance']) == (
            'SUCCEEDED', gm.OUTCOME_SUCCEEDED, 'MOCK')
        assert (row['mode'], row['live_kinds']) == (gm.MODE_DEVELOPMENT_MOCK, '[]')
        assert all(row[name] is None for name in MISSING)
        assert row['duration_s'] == pytest.approx(
            run.finished_boottime_s - run.sent_boottime_s)
        assert row['duration_sim_s'] == pytest.approx(run.finished_sim_s - run.sent_sim_s)
        assert row['sent_boottime_s'] <= row['accepted_boottime_s'] <= row['finished_boottime_s']
        assert row['map_revision'] == flow.spec.map_revision

    assert rows['src_nav_manipulator']['from_location_id'] is None
    assert rows['src_nav_manipulator']['to_location_id'] == flow.spec.source_location_id
    assert rows['dst_nav_transporter']['from_location_id'] == flow.spec.source_location_id
    assert rows['dst_nav_transporter']['to_location_id'] == flow.spec.destination_location_id
    assert rows['dst_nav_transporter']['role'] == gm.TRANSPORTER

    # 운송 로봇은 목적지 주행 때 물체를 싣고 있다. 조작 로봇은 아니다. 출발 주행은 둘 다 아니다
    assert rows['dst_nav_transporter']['carrying_object'] == 1
    assert rows['dst_nav_manipulator']['carrying_object'] == 0
    assert rows['src_nav_transporter']['carrying_object'] == 0
    store.close()


# 혼합 모드: 주행 결과 출처 LIVE와 live_kinds 기록
def test_samples_mixed_live(tmp_path):
    flow, start = flow_with_start(MixedFlow)
    run_until(flow, 'src_nav')
    store = TaskStore(str(tmp_path / 'd2.sqlite'))
    record_all(store, flow, start)

    rows = store.eta_samples()
    assert len(rows) == 2
    assert all((r['mode'], r['provenance'], r['live_kinds']) == (
        gm.MODE_DEVELOPMENT_MIXED, 'LIVE', '["ACTUATION", "NAVIGATION"]') for r in rows)
    store.close()


# 중복 결과는 표본을 하나만 남긴다. 실패 결과도 표본이다
def test_duplicate_result_single_sample(tmp_path):
    flow, start = flow_with_start(Flow)
    flow.start()
    flow.send_all()
    flow.finish('src_nav_manipulator', outcome=gm.OUTCOME_FAILED)
    duplicate = flow.finish('src_nav_manipulator', outcome=gm.OUTCOME_FAILED)
    assert 'DUPLICATE_RESULT' in duplicate.reasons

    store = TaskStore(str(tmp_path / 'd2.sqlite'))
    record_all(store, flow, start)
    rows = store.eta_samples()

    assert [(r['step_key'], r['status']) for r in rows] == [('src_nav_manipulator', 'FAILED')]
    store.close()


# 결과 대기 시간초과로 미확정: 종료 시각·출처 NULL
def test_timeout_uncertain_sample(tmp_path):
    flow, start = flow_with_start(Flow)
    flow.start()
    flow.send_all()
    flow.wait(61.0)
    flow.apply(Tick())

    store = TaskStore(str(tmp_path / 'd2.sqlite'))
    record_all(store, flow, start)
    rows = store.eta_samples()

    assert sorted(r['step_key'] for r in rows) == ['src_nav_manipulator', 'src_nav_transporter']
    for row in rows:
        assert row['status'] == 'UNCERTAIN'
        assert row['provenance'] is None and row['outcome'] is None
        assert row['finished_boottime_s'] is None and row['duration_s'] is None
    store.close()


# 표본은 판정 기록과 같은 트랜잭션: 표본 쓰기가 실패하면 작업 상태도 바뀌지 않는다
def test_sample_in_same_transaction(tmp_path):
    flow, start = flow_with_start(Flow)
    flow.start()
    flow.send_all()
    flow.finish('src_nav_manipulator')
    path = str(tmp_path / 'd2.sqlite')
    control = {}

    # 실패 주입 연결(나중에 켠다)
    def connect(p, **kwargs):
        return FailingConnection(sqlite3.connect(p, **kwargs), control)

    store = TaskStore(path, connect=connect)
    store.create_task(start)
    previous = start
    for decision in flow.decisions[:-1]:
        store.record_decision(flow.spec.task_id, previous.revision, decision, 'Event', {})
        previous = decision.next_state

    control['fail_prefix'] = 'INSERT INTO eta_samples'
    with pytest.raises(StoreError):
        store.record_decision(flow.spec.task_id, previous.revision, flow.decisions[-1],
                              'StepResult', {})
    control.clear()

    revision = store.db.execute('SELECT revision FROM tasks').fetchone()[0]
    assert revision == previous.revision
    assert store.eta_samples() == []
    store.close()


# 재시작 복원으로 진행 중 주행이 미확정이 되면 표본을 남긴다(종료 시각 NULL)
def test_restore_uncertain_sample(tmp_path):
    path = str(tmp_path / 'd2.sqlite')
    ports = {'LOCAL_TASK': MockLocalTaskPort(), 'NAVIGATION': MockNavigationPort()}
    runtime = GroupRuntime(TaskStore(path), ports, Clock(), map_revision=1, locations=LOCATIONS)
    runtime.admit(make_spec())
    runtime.handle('task_dev_1', AssignRoles(make_assignment('robot_m7', 'robot_t3')))
    runtime.handle('task_dev_1', StartTask())
    assert runtime.store.eta_samples() == []
    runtime.store.close()

    store = TaskStore(path)
    GroupRuntime(store, ports, Clock(), map_revision=1, locations=LOCATIONS)
    rows = store.eta_samples()

    assert sorted(r['step_key'] for r in rows) == ['src_nav_manipulator', 'src_nav_transporter']
    assert all(r['status'] == 'UNCERTAIN' and r['finished_boottime_s'] is None for r in rows)
    store.close()


# 내보내기용 원장: 혼합 LIVE 성공 2, 혼합 LIVE 실패 1, 모의 성공 2, 비정상 시간 행 3
def make_export_store(path):
    store = TaskStore(path)

    live, start = flow_with_start(MixedFlow)
    run_until(live, 'src_nav')
    record_all(store, live, start)

    failed, start = flow_with_start(lambda: MixedFlow(spec=other_spec('fail')))
    failed.start()
    failed.send_all()
    failed.finish('src_nav_manipulator', outcome=gm.OUTCOME_FAILED)
    record_all(store, failed, start)

    mock, start = flow_with_start(lambda: Flow(spec=other_spec('mock')))
    run_until(mock, 'src_nav')
    record_all(store, mock, start)

    # 비정상 시간 행(시험용 직접 삽입): 음수·무한 시간, 종료 시각 없음
    template = [r for r in store.eta_samples() if r['provenance'] == 'LIVE'
                and r['status'] == 'SUCCEEDED'][0]
    bad = [dict(template, execution_id='bad_negative', duration_s=-1.0),
           dict(template, execution_id='bad_inf', duration_s=float('inf')),
           dict(template, execution_id='bad_sim', duration_sim_s=-0.5),
           dict(template, execution_id='bad_unfinished', finished_boottime_s=None)]
    with store.transaction():
        for row in bad:
            store.db.execute(
                f'INSERT INTO eta_samples ({", ".join(ETA_SAMPLE_FIELDS)}) VALUES '
                f'({", ".join("?" for _ in ETA_SAMPLE_FIELDS)})',
                tuple(row[n] for n in ETA_SAMPLE_FIELDS))
    store.close()


# CSV 읽기(첫 줄 열 이름)
def read_csv(path):
    with open(path, encoding='utf-8') as stream:
        rows = list(csv.reader(stream))

    return rows[0], [dict(zip(rows[0], r)) for r in rows[1:]]


# 내보내기 필터: 기본은 혼합·LIVE·SUCCEEDED만, 비정상 시간 행은 항상 제외
@pytest.mark.parametrize('flags, expected', [
    ([], {('LIVE', 'SUCCEEDED', 'DEVELOPMENT_MIXED'): 2}),
    (['--include-failed'], {('LIVE', 'SUCCEEDED', 'DEVELOPMENT_MIXED'): 2,
                            ('LIVE', 'FAILED', 'DEVELOPMENT_MIXED'): 1}),
    (['--include-mock'], {('LIVE', 'SUCCEEDED', 'DEVELOPMENT_MIXED'): 2,
                          ('MOCK', 'SUCCEEDED', 'DEVELOPMENT_MOCK'): 2}),
    (['--include-mock', '--include-failed'], {('LIVE', 'SUCCEEDED', 'DEVELOPMENT_MIXED'): 2,
                                              ('LIVE', 'FAILED', 'DEVELOPMENT_MIXED'): 1,
                                              ('MOCK', 'SUCCEEDED', 'DEVELOPMENT_MOCK'): 2}),
])
def test_export_filters(tmp_path, flags, expected):
    path = str(tmp_path / 'd2.sqlite')
    make_export_store(path)
    out = str(tmp_path / 'eta.csv')

    assert export.main([path, '-o', out] + flags) == 0
    header, rows = read_csv(out)

    assert tuple(header) == ETA_SAMPLE_FIELDS
    counts = {}
    for row in rows:
        key = (row['provenance'], row['status'], row['mode'])
        counts[key] = counts.get(key, 0) + 1
    assert counts == expected
    assert not any(r['execution_id'].startswith('bad_') for r in rows)
    assert all(r[name] == '' for r in rows for name in MISSING)


# 내보내기는 원장을 바꾸지 않고, schema 4 미만·없는 파일은 종료 코드 2
def test_export_read_only_and_old_schema(tmp_path):
    path = str(tmp_path / 'd2.sqlite')
    make_export_store(path)
    before = Path(path).read_bytes()
    assert export.main([path, '-o', str(tmp_path / 'a.csv')]) == 0
    assert Path(path).read_bytes() == before

    old = str(tmp_path / 'old.sqlite')
    make_v3_store(old)
    assert export.main([old, '-o', str(tmp_path / 'b.csv')]) == 2
    assert schema_of(old)[1] == '3'
    assert export.main([str(tmp_path / 'none.sqlite')]) == 2
