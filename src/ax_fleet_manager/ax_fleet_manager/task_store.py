"""D2 전용 SQLite 원장: 작업 정의·배정 세대·상태·단계 의도·실행 ID·goal UUID·결과.

- D4 원장 DB와 다른 파일이며 D4 원장을 열거나 바꾸지 않는다.
- 상태·의도는 같은 트랜잭션으로 기록한다(expected_revision으로 동시 갱신 거부).
- 전송 전 reserve_dispatch로 goal UUID를 먼저 기록한다. 이미 예약된 실행은 다시 예약하지 않으며
  호출자는 그 실행을 재전송하지 않는다(ROS Action exactly-once를 가정하지 않음).
- 열 때 무결성·schema 버전·JSON 형식을 확인하고, 이상이 있으면 StoreCorrupt로 거부한다.
"""
import json
import sqlite3
import uuid
from dataclasses import asdict

from . import group_model as gm

SCHEMA_VERSION = 1
MAX_EVENT_PAYLOAD_BYTES = 4096


class StoreError(Exception):
    """원장 쓰기·읽기 실패. 호출자는 새 일반 명령을 막는다."""


class StoreCorrupt(StoreError):
    """원장 손상·schema 불일치."""


class StoreConflict(StoreError):
    """revision 불일치 또는 같은 실행 ID의 다른 인자."""


class DispatchAlreadyReserved(StoreError):
    """이미 예약·전송된 실행. 재전송하지 않는다."""


# 상태를 JSON 사전으로 바꾼다(관측 버퍼는 저장하지 않는다)
def state_to_dict(state):
    return {
        'spec': asdict(state.spec),
        'mode': state.mode,
        'assignment': asdict(state.assignment) if state.assignment else None,
        'task_state': state.task_state,
        'transfer_state': state.transfer_state,
        'transfer_id': state.transfer_id,
        'steps': {k: asdict(v) for k, v in state.steps.items()},
        'inhibit': list(state.inhibit),
        'hold_ids': list(state.hold_ids),
        'last_sim_time_s': state.last_sim_time_s,
        'cancel_requested': state.cancel_requested,
        'cancel_requested_boottime_s': state.cancel_requested_boottime_s,
        'failure_event_id': state.failure_event_id,
        'revision': state.revision,
    }


# JSON 사전을 상태로 되돌린다. 작업 정의는 다시 검증한다
def state_from_dict(data):
    spec_data = dict(data['spec'])
    spec_data['steps'] = tuple(
        gm.PlanStep(s['step_key'], s['operation'], s['robot_role'], s['location'],
                    tuple(s['depends_on']), s['terminal'])
        for s in spec_data['steps'])
    spec = gm.GroupTaskSpec(**spec_data)
    gm.validate_spec(spec)

    assignment = gm.Assignment(**data['assignment']) if data['assignment'] else None
    if assignment is not None:
        gm.validate_assignment(assignment)

    steps = {k: gm.StepRun(**v) for k, v in data['steps'].items()}
    if set(steps) != {s.step_key for s in spec.steps}:
        raise StoreCorrupt('단계 기록과 계획 단계가 다르다')

    return gm.GroupState(
        spec=spec, mode=data['mode'], assignment=assignment,
        task_state=data['task_state'], transfer_state=data['transfer_state'],
        transfer_id=data['transfer_id'], steps=steps, inhibit=list(data['inhibit']),
        hold_ids=list(data['hold_ids']), last_sim_time_s=data['last_sim_time_s'],
        cancel_requested=data['cancel_requested'],
        cancel_requested_boottime_s=data['cancel_requested_boottime_s'],
        failure_event_id=data['failure_event_id'], revision=data['revision'])


class TaskStore:
    """D2 원장 파일 하나. connect는 시험에서 실패를 주입할 수 있게 받는다."""

    # 원장을 열고 무결성·schema를 확인한다
    def __init__(self, path, max_events_per_task=500, busy_timeout_s=2.0,
                 connect=sqlite3.connect, uuid_factory=None):
        self.max_events_per_task = max_events_per_task
        self.uuid_factory = uuid_factory or (lambda: str(uuid.uuid4()))

        try:
            self.db = connect(path, timeout=busy_timeout_s, isolation_level=None)
            self.db.execute('PRAGMA synchronous=FULL')
            check = self.db.execute('PRAGMA integrity_check').fetchone()
        except sqlite3.DatabaseError as error:
            raise StoreCorrupt(f'원장 열기 실패: {error}') from error

        if check is None or check[0] != 'ok':
            raise StoreCorrupt(f'무결성 검사 실패: {check}')

        self._init_schema()

    # 새 원장이면 표를 만들고, 기존 원장이면 schema 버전을 확인한다
    def _init_schema(self):
        tables = {r[0] for r in self.db.execute(
            "SELECT name FROM sqlite_master WHERE type='table'")}

        if not tables:
            with self._tx():
                self.db.execute('CREATE TABLE meta (key TEXT PRIMARY KEY, value TEXT NOT NULL)')
                self.db.execute(
                    'CREATE TABLE tasks (task_id TEXT PRIMARY KEY, revision INTEGER NOT NULL,'
                    ' state_json TEXT NOT NULL)')
                self.db.execute(
                    'CREATE TABLE dispatches (execution_id TEXT PRIMARY KEY,'
                    ' task_id TEXT NOT NULL, step_key TEXT NOT NULL, operation TEXT NOT NULL,'
                    ' robot_id TEXT NOT NULL, args_digest TEXT NOT NULL,'
                    ' intent_json TEXT NOT NULL, goal_uuid TEXT, status TEXT NOT NULL,'
                    ' result_json TEXT)')
                self.db.execute(
                    'CREATE TABLE events (id INTEGER PRIMARY KEY AUTOINCREMENT,'
                    ' task_id TEXT NOT NULL, kind TEXT NOT NULL, payload_json TEXT NOT NULL)')
                self.db.execute("INSERT INTO meta VALUES ('schema_version', ?)",
                                (str(SCHEMA_VERSION),))
            return

        expected = {'meta', 'tasks', 'dispatches', 'events', 'sqlite_sequence'}
        if not tables <= expected or not {'meta', 'tasks', 'dispatches', 'events'} <= tables:
            raise StoreCorrupt(f'예상하지 않은 표 구성: {sorted(tables)}')

        row = self.db.execute("SELECT value FROM meta WHERE key='schema_version'").fetchone()
        if row is None or row[0] != str(SCHEMA_VERSION):
            raise StoreCorrupt(f'schema 버전 불일치: {row}')

    # 즉시 쓰기 잠금 트랜잭션. 실패하면 되돌리고 StoreError로 바꾼다
    def _tx(self):
        store = self

        class _Tx:
            def __enter__(self):
                try:
                    store.db.execute('BEGIN IMMEDIATE')
                except sqlite3.Error as error:
                    raise StoreError(f'트랜잭션 시작 실패: {error}') from error

            def __exit__(self, exc_type, exc, tb):
                if exc_type is None:
                    try:
                        store.db.execute('COMMIT')
                        return False
                    except sqlite3.Error as error:
                        exc = error
                try:
                    store.db.execute('ROLLBACK')
                except sqlite3.Error:
                    pass
                if isinstance(exc, StoreError):
                    return False
                if isinstance(exc, (sqlite3.Error, ValueError, TypeError)):
                    raise StoreError(f'원장 쓰기 실패: {exc}') from exc
                return False

        return _Tx()

    # 사건 기록(작업별 상한 초과분은 오래된 것부터 지운다)
    def _append_event(self, task_id, kind, payload):
        text = json.dumps(payload, sort_keys=True, ensure_ascii=False, default=str)
        if len(text.encode('utf-8')) > MAX_EVENT_PAYLOAD_BYTES:
            text = json.dumps({'truncated': True, 'kind': kind})

        self.db.execute('INSERT INTO events (task_id, kind, payload_json) VALUES (?, ?, ?)',
                        (task_id, kind, text))
        self.db.execute(
            'DELETE FROM events WHERE task_id=? AND id NOT IN ('
            ' SELECT id FROM events WHERE task_id=? ORDER BY id DESC LIMIT ?)',
            (task_id, task_id, self.max_events_per_task))

    # 새 작업 기록. 같은 작업 ID가 있으면 거부한다
    def create_task(self, state):
        with self._tx():
            if self.db.execute('SELECT 1 FROM tasks WHERE task_id=?',
                               (state.spec.task_id,)).fetchone():
                raise StoreConflict(f'이미 있는 작업: {state.spec.task_id}')

            self.db.execute('INSERT INTO tasks VALUES (?, ?, ?)',
                            (state.spec.task_id, state.revision,
                             json.dumps(state_to_dict(state), sort_keys=True)))
            self._append_event(state.spec.task_id, 'Admit',
                               {'input_source_id': state.spec.input_source_id})

    # 판정 하나(상태·의도·사건)를 한 트랜잭션으로 기록한다
    def record_decision(self, task_id, expected_revision, decision, event_kind, payload):
        state = decision.next_state

        with self._tx():
            row = self.db.execute('SELECT revision FROM tasks WHERE task_id=?',
                                  (task_id,)).fetchone()
            if row is None or row[0] != expected_revision:
                raise StoreConflict(f'revision 불일치: {row} != {expected_revision}')

            self.db.execute('UPDATE tasks SET revision=?, state_json=? WHERE task_id=?',
                            (state.revision, json.dumps(state_to_dict(state), sort_keys=True),
                             task_id))

            for intent in decision.effects:
                if intent.kind == 'CANCEL':
                    continue
                self._insert_intent(task_id, intent)

            self._append_event(task_id, event_kind, {
                'payload': payload, 'reasons': list(decision.reasons),
                'effects': [(e.kind, e.step_key, e.execution['execution_id'])
                            for e in decision.effects]})

    # 의도 기록. 같은 실행 ID가 이미 있으면 인자가 같아야 한다(다르면 충돌)
    def _insert_intent(self, task_id, intent):
        execution_id = intent.execution['execution_id']
        row = self.db.execute('SELECT args_digest FROM dispatches WHERE execution_id=?',
                              (execution_id,)).fetchone()
        if row is not None:
            if row[0] != intent.args_digest:
                raise StoreConflict(f'같은 실행 ID의 다른 인자: {execution_id}')
            return

        self.db.execute(
            'INSERT INTO dispatches VALUES (?, ?, ?, ?, ?, ?, ?, NULL, ?, NULL)',
            (execution_id, task_id, intent.step_key, intent.operation, intent.robot_id,
             intent.args_digest,
             json.dumps({'execution': intent.execution, 'args': intent.args},
                        sort_keys=True),
             'INTENDED'))

    # 전송 직전 goal UUID 예약. INTENDED가 아니면 예약·전송하지 않는다
    def reserve_dispatch(self, execution_id):
        with self._tx():
            row = self.db.execute('SELECT status FROM dispatches WHERE execution_id=?',
                                  (execution_id,)).fetchone()
            if row is None:
                raise StoreConflict(f'기록되지 않은 실행: {execution_id}')
            if row[0] != 'INTENDED':
                raise DispatchAlreadyReserved(f'{execution_id}: {row[0]}')

            goal_uuid = self.uuid_factory()
            self.db.execute(
                "UPDATE dispatches SET status='RESERVED', goal_uuid=? WHERE execution_id=?",
                (goal_uuid, execution_id))

        return goal_uuid

    # 실행 기록 상태·결과 갱신(전송 완료·실패·결과)
    def update_dispatch(self, execution_id, status, result=None):
        with self._tx():
            cursor = self.db.execute(
                'UPDATE dispatches SET status=?, result_json=COALESCE(?, result_json)'
                ' WHERE execution_id=?',
                (status, json.dumps(result, sort_keys=True) if result is not None else None,
                 execution_id))
            if cursor.rowcount != 1:
                raise StoreConflict(f'기록되지 않은 실행: {execution_id}')

    # 실행 기록 하나
    def dispatch_row(self, execution_id):
        cursor = self.db.execute('SELECT * FROM dispatches WHERE execution_id=?',
                                 (execution_id,))
        row = cursor.fetchone()
        if row is None:
            return None

        return dict(zip([c[0] for c in cursor.description], row))

    # 작업의 실행 기록 목록
    def dispatch_rows(self, task_id):
        cursor = self.db.execute('SELECT * FROM dispatches WHERE task_id=? ORDER BY rowid',
                                 (task_id,))
        names = [c[0] for c in cursor.description]

        return [dict(zip(names, row)) for row in cursor.fetchall()]

    # 작업의 사건 기록
    def events(self, task_id):
        return [(kind, json.loads(payload)) for kind, payload in self.db.execute(
            'SELECT kind, payload_json FROM events WHERE task_id=? ORDER BY id', (task_id,))]

    # 저장된 상태 전체(복원 규칙은 호출자가 적용). 형식 오류는 손상으로 본다
    def load_states(self):
        states = []
        try:
            rows = self.db.execute(
                'SELECT task_id, revision, state_json FROM tasks ORDER BY task_id').fetchall()
            for task_id, revision, text in rows:
                state = state_from_dict(json.loads(text))
                if state.spec.task_id != task_id or state.revision != revision:
                    raise StoreCorrupt(f'작업 행 불일치: {task_id}')
                states.append(state)
        except (sqlite3.Error, ValueError, KeyError, TypeError) as error:
            raise StoreCorrupt(f'원장 상태 읽기 실패: {error}') from error

        return states

    # 연결 닫기
    def close(self):
        self.db.close()
