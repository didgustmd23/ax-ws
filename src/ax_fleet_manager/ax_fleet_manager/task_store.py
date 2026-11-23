"""D2 전용 SQLite 원장: 작업 정의·배정 세대·상태·단계 의도·실행 ID·goal UUID·결과.

- D4 원장 DB와 다른 파일이며 D4 원장을 열거나 바꾸지 않는다.
- 상태·의도는 같은 트랜잭션으로 기록한다(expected_revision으로 동시 갱신 거부).
- 전송 전 reserve_dispatch로 goal UUID를 먼저 기록한다. 이미 예약된 실행은 다시 예약하지 않으며
  호출자는 그 실행을 재전송하지 않는다(ROS Action exactly-once를 가정하지 않음).
- 열 때 무결성·schema 버전·JSON 형식을 확인하고, 이상이 있으면 StoreCorrupt로 거부한다.
- schema 2(W4)는 입고 사건·입고 관측·작업 등록 요청 표를, schema 3(W5)은 재고 항목·등록 지점·목적지
  예약 표를 더한다. schema 1·2 원장은 쓰기 가능하게 열 때 한 트랜잭션으로 이주하고, 이주가 실패하면
  원장을 바꾸지 않은 채 거부한다. 읽기 전용 열기는 이주하지 않는다.
- 작업 판정 기록(record_decision)과 같은 트랜잭션에서 그 작업 물체의 재고 항목·예약을 맞춘다.
- 주행 단계는 goal UUID를 D1이 만든다: begin_submit(SUBMITTING) → D1 submit → record_submitted.
"""
import json
import sqlite3
import uuid
from dataclasses import asdict

from . import group_model as gm

SCHEMA_VERSION = 3
V1_TABLES = frozenset({'meta', 'tasks', 'dispatches', 'events'})
RECEIPT_TABLES = frozenset({'receipt_events', 'receipt_observations', 'register_requests'})
V3_TABLES = frozenset({'inventory', 'locations', 'reservations'})

# schema 1 표(W3)
V1_DDL = (
    'CREATE TABLE meta (key TEXT PRIMARY KEY, value TEXT NOT NULL)',
    'CREATE TABLE tasks (task_id TEXT PRIMARY KEY, revision INTEGER NOT NULL,'
    ' state_json TEXT NOT NULL)',
    'CREATE TABLE dispatches (execution_id TEXT PRIMARY KEY,'
    ' task_id TEXT NOT NULL, step_key TEXT NOT NULL, operation TEXT NOT NULL,'
    ' robot_id TEXT NOT NULL, args_digest TEXT NOT NULL,'
    ' intent_json TEXT NOT NULL, goal_uuid TEXT, status TEXT NOT NULL,'
    ' result_json TEXT)',
    'CREATE TABLE events (id INTEGER PRIMARY KEY AUTOINCREMENT,'
    ' task_id TEXT NOT NULL, kind TEXT NOT NULL, payload_json TEXT NOT NULL)',
)

# schema 2에서 더하는 입고 표. 식별된 물체는 회차마다 사건 1개, 작업은 사건마다 1개
RECEIPT_DDL = (
    'CREATE TABLE receipt_events (receipt_event_id TEXT PRIMARY KEY,'
    ' object_id TEXT NOT NULL, zone_id TEXT NOT NULL, receipt_round INTEGER NOT NULL,'
    ' state INTEGER NOT NULL, task_id TEXT NOT NULL, observation_ids_json TEXT NOT NULL,'
    ' provenance TEXT NOT NULL, reason_code TEXT NOT NULL)',
    'CREATE UNIQUE INDEX receipt_object_round ON receipt_events(object_id, receipt_round)'
    " WHERE object_id != ''",
    "CREATE UNIQUE INDEX receipt_task ON receipt_events(task_id) WHERE task_id != ''",
    # 식별 불명 관측은 입고 구역마다 대기 사건 1개로 묶는다(state 0 = PENDING_IDENTIFICATION)
    'CREATE UNIQUE INDEX receipt_pending_zone ON receipt_events(zone_id) WHERE state = 0',
    'CREATE TABLE receipt_observations (observation_id TEXT PRIMARY KEY,'
    ' digest TEXT NOT NULL, receipt_event_id TEXT NOT NULL, decision TEXT NOT NULL,'
    ' reason_code TEXT NOT NULL)',
    'CREATE TABLE register_requests (request_id TEXT PRIMARY KEY,'
    ' request_digest TEXT NOT NULL, response_json TEXT NOT NULL)',
)

# schema 3(W5): 재고 항목(물체 단위, revision 영속), 등록 지점 설정 기록, 목적지 예약(작업당 1개)
V3_DDL = (
    "ALTER TABLE receipt_events ADD COLUMN object_class TEXT NOT NULL DEFAULT ''",
    'CREATE TABLE inventory (object_id TEXT PRIMARY KEY, revision INTEGER NOT NULL,'
    ' receipt_event_id TEXT NOT NULL, object_class TEXT NOT NULL, location_id TEXT NOT NULL,'
    ' carrier_robot_id TEXT NOT NULL, assignment_generation INTEGER NOT NULL,'
    ' state INTEGER NOT NULL, isolated INTEGER NOT NULL, task_id TEXT NOT NULL)',
    'CREATE TABLE locations (location_id TEXT PRIMARY KEY, location_revision INTEGER NOT NULL,'
    ' map_revision INTEGER NOT NULL, info_json TEXT NOT NULL)',
    'CREATE TABLE reservations (reservation_id TEXT PRIMARY KEY, location_id TEXT NOT NULL,'
    ' task_id TEXT NOT NULL UNIQUE, object_id TEXT NOT NULL, state TEXT NOT NULL)',
)
INVENTORY_FIELDS = ('object_id', 'revision', 'receipt_event_id', 'object_class', 'location_id',
                    'carrier_robot_id', 'assignment_generation', 'state', 'isolated', 'task_id')
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
        'object_state': state.object_state,
        'object_location_id': state.object_location_id,
        'object_carrier_robot_id': state.object_carrier_robot_id,
        'object_assignment_generation': state.object_assignment_generation,
        'object_isolated': state.object_isolated,
        'physical_started': state.physical_started,
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
        failure_event_id=data['failure_event_id'], revision=data['revision'],
        object_state=data.get('object_state', gm.INV_UNKNOWN),
        object_location_id=data.get('object_location_id', ''),
        object_carrier_robot_id=data.get('object_carrier_robot_id', ''),
        object_assignment_generation=data.get('object_assignment_generation', 0),
        object_isolated=data.get('object_isolated', False),
        physical_started=data.get('physical_started', False))


class TaskStore:
    """D2 원장 파일 하나. connect는 시험에서 실패를 주입할 수 있게 받는다."""

    # 원장을 열고 무결성·schema를 확인한다
    def __init__(self, path, max_events_per_task=500, busy_timeout_s=2.0,
                 connect=sqlite3.connect, uuid_factory=None, read_only=False):
        self.max_events_per_task = max_events_per_task
        self.read_only = read_only
        self.schema_version = SCHEMA_VERSION
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

    # 새 원장이면 표를 만들고, 기존 원장이면 schema 버전을 확인한다(schema 1·2는 한 트랜잭션 이주)
    def _init_schema(self):
        tables = {r[0] for r in self.db.execute(
            "SELECT name FROM sqlite_master WHERE type='table'")}

        if not tables:
            if self.read_only:
                raise StoreCorrupt('읽기 전용으로 빈 원장을 열 수 없다')

            with self._tx():
                for ddl in V1_DDL + RECEIPT_DDL + V3_DDL:
                    self.db.execute(ddl)
                self.db.execute("INSERT INTO meta VALUES ('schema_version', ?)",
                                (str(SCHEMA_VERSION),))
                self.db.execute("INSERT INTO meta VALUES ('inventory_revision', '0')")
            return

        known = V1_TABLES | RECEIPT_TABLES | V3_TABLES | {'sqlite_sequence'}
        if not tables <= known or not V1_TABLES <= tables:
            raise StoreCorrupt(f'예상하지 않은 표 구성: {sorted(tables)}')

        row = self.db.execute("SELECT value FROM meta WHERE key='schema_version'").fetchone()
        version = row[0] if row is not None else None
        expected = {'1': V1_TABLES, '2': V1_TABLES | RECEIPT_TABLES,
                    '3': V1_TABLES | RECEIPT_TABLES | V3_TABLES}

        if version not in expected or tables - {'sqlite_sequence'} != expected[version]:
            raise StoreCorrupt(f'schema 버전·표 불일치: {row} {sorted(tables)}')

        self.schema_version = int(version)
        if self.schema_version == SCHEMA_VERSION:
            return

        # 이전 schema: 읽기 전용이면 그대로 읽고, 쓰기 가능이면 한 트랜잭션으로 이주한다
        if self.read_only:
            return

        self._migrate(self.schema_version)
        self.schema_version = SCHEMA_VERSION

    # schema 1·2 → 3 이주: 표 추가·열 추가·재고 백필·버전 갱신을 한 트랜잭션으로. 실패하면 되돌리고 거부
    def _migrate(self, version):
        try:
            with self._tx():
                if version == 1:
                    for ddl in RECEIPT_DDL:
                        self.db.execute(ddl)

                for ddl in V3_DDL:
                    self.db.execute(ddl)

                self._backfill_inventory()
                self.db.execute("UPDATE meta SET value=? WHERE key='schema_version'",
                                (str(SCHEMA_VERSION),))
                self.db.execute("INSERT INTO meta VALUES ('inventory_revision', '0')")
        except StoreError as error:
            raise StoreCorrupt(f'schema {version}→{SCHEMA_VERSION} 이주 실패(원장 불변): {error}') \
                from error

    # 이주 백필: 종료 전 작업의 물체는 근거가 없으므로 UNKNOWN 항목(격리 아님), 종료 작업은 만들지 않음
    def _backfill_inventory(self):
        for task_id, text in self.db.execute('SELECT task_id, state_json FROM tasks').fetchall():
            data = json.loads(text)
            if data['task_state'] in gm.TERMINAL_TASK_STATES:
                continue

            object_id = data['spec']['object_id']
            if self.db.execute('SELECT 1 FROM inventory WHERE object_id=?',
                               (object_id,)).fetchone():
                continue

            self.db.execute(
                'INSERT INTO inventory VALUES (?, 1, ?, ?, ?, ?, ?, ?, 0, ?)',
                (object_id, '', '', '', '', 0, gm.INV_UNKNOWN, task_id))

    # 호출자가 여러 기록을 한 트랜잭션으로 묶을 때 쓴다(입고 사건·요청·작업 생성)
    def transaction(self):
        return self._tx()

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
            self.insert_task_row(state)

    # 작업 행 추가(호출자의 트랜잭션 안에서). 같은 작업 ID가 있으면 거부한다
    def insert_task_row(self, state):
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

            self._sync_inventory(state)

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

    # 주행 단계 전송 시작 기록(goal UUID 없음, D1이 만든다). INTENDED가 아니면 보내지 않는다
    def begin_submit(self, execution_id):
        with self._tx():
            row = self.db.execute('SELECT status FROM dispatches WHERE execution_id=?',
                                  (execution_id,)).fetchone()
            if row is None:
                raise StoreConflict(f'기록되지 않은 실행: {execution_id}')
            if row[0] != 'INTENDED':
                raise DispatchAlreadyReserved(f'{execution_id}: {row[0]}')

            self.db.execute(
                "UPDATE dispatches SET status='SUBMITTING' WHERE execution_id=?",
                (execution_id,))

    # D1이 돌려준 goal UUID 기록(참고·결합용). SUBMITTING에서 한 번만, 같은 값 재기록은 허용
    def record_submitted(self, execution_id, goal_uuid):
        if type(goal_uuid) is not str or not goal_uuid:
            raise StoreConflict(f'D1 goal UUID 없음: {execution_id}')

        with self._tx():
            row = self.db.execute('SELECT status, goal_uuid FROM dispatches WHERE execution_id=?',
                                  (execution_id,)).fetchone()
            if row is None:
                raise StoreConflict(f'기록되지 않은 실행: {execution_id}')
            if row[0] != 'SUBMITTING':
                if row[1] == goal_uuid:
                    return
                raise StoreConflict(f'{execution_id}: {row[0]} {row[1]} != {goal_uuid}')

            self.db.execute(
                "UPDATE dispatches SET status='RESERVED', goal_uuid=? WHERE execution_id=?",
                (goal_uuid, execution_id))

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

    # ---- 입고 표(schema 2). 쓰기는 호출자의 transaction() 안에서 부른다 ----

    # 질의 결과를 열 이름 사전 목록으로
    def _rows(self, sql, args=()):
        cursor = self.db.execute(sql, args)
        names = [c[0] for c in cursor.description]

        return [dict(zip(names, row)) for row in cursor.fetchall()]

    # 입고 표가 없는(schema 1 읽기 전용) 원장에서는 입고 기능을 쓰지 않는다
    def _require_receipt_tables(self):
        if self.schema_version < 2:
            raise StoreError('schema 1 원장에는 입고 표가 없다')

    # 입고 관측 기록 하나(없으면 None)
    def receipt_observation(self, observation_id):
        self._require_receipt_tables()
        rows = self._rows('SELECT * FROM receipt_observations WHERE observation_id=?',
                          (observation_id,))

        return rows[0] if rows else None

    # 입고 관측 기록 추가
    def insert_receipt_observation(self, observation_id, digest, receipt_event_id, decision,
                                   reason_code):
        self._require_receipt_tables()
        self.db.execute('INSERT INTO receipt_observations VALUES (?, ?, ?, ?, ?)',
                        (observation_id, digest, receipt_event_id, decision, reason_code))

    # 입고 사건 하나(없으면 None). observation_ids는 목록으로 풀어 준다
    def receipt_event(self, receipt_event_id):
        self._require_receipt_tables()
        rows = self._rows('SELECT * FROM receipt_events WHERE receipt_event_id=?',
                          (receipt_event_id,))

        return _decode_event(rows[0]) if rows else None

    # 물체의 입고 사건 목록(회차 순)
    def receipt_events_for_object(self, object_id):
        self._require_receipt_tables()
        rows = self._rows('SELECT * FROM receipt_events WHERE object_id=?'
                          ' ORDER BY receipt_round', (object_id,))

        return [_decode_event(r) for r in rows]

    # 전체 입고 사건(시험·관제용)
    def receipt_events(self):
        self._require_receipt_tables()

        return [_decode_event(r) for r in self._rows(
            'SELECT * FROM receipt_events ORDER BY rowid')]

    # 입고 구역의 식별 대기 사건(없으면 None). 구역마다 많아야 1개다
    def pending_receipt_event(self, zone_id, pending_state):
        self._require_receipt_tables()
        rows = self._rows('SELECT * FROM receipt_events WHERE zone_id=? AND state=?',
                          (zone_id, pending_state))

        return _decode_event(rows[0]) if rows else None

    # 입고 사건 추가 또는 갱신(같은 ID면 전체 필드를 바꾼다)
    def put_receipt_event(self, event):
        self._require_receipt_tables()
        self.db.execute(
            'INSERT INTO receipt_events (receipt_event_id, object_id, zone_id, receipt_round,'
            ' state, task_id, observation_ids_json, provenance, reason_code, object_class)'
            ' VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)'
            ' ON CONFLICT(receipt_event_id) DO UPDATE SET object_id=excluded.object_id,'
            ' zone_id=excluded.zone_id, receipt_round=excluded.receipt_round,'
            ' state=excluded.state, task_id=excluded.task_id,'
            ' observation_ids_json=excluded.observation_ids_json,'
            ' provenance=excluded.provenance, reason_code=excluded.reason_code,'
            ' object_class=excluded.object_class',
            (event['receipt_event_id'], event['object_id'], event['zone_id'],
             event['receipt_round'], event['state'], event['task_id'],
             json.dumps(list(event['observation_ids'])), event['provenance'],
             event['reason_code'], event.get('object_class', '')))

    # 작업 등록 요청 기록 하나(없으면 None). 응답은 사전으로 풀어 준다
    def register_request(self, request_id):
        self._require_receipt_tables()
        rows = self._rows('SELECT * FROM register_requests WHERE request_id=?', (request_id,))
        if not rows:
            return None

        row = rows[0]
        row['response'] = json.loads(row.pop('response_json'))

        return row

    # 작업 등록 요청과 응답 기록
    def insert_register_request(self, request_id, digest, response):
        self._require_receipt_tables()
        self.db.execute('INSERT INTO register_requests VALUES (?, ?, ?)',
                        (request_id, digest, json.dumps(response, sort_keys=True)))

    # ---- 재고·지점·예약(schema 3). 쓰기는 호출자의 transaction() 또는 record_decision 안 ----

    # 재고 표가 없는(이전 schema 읽기 전용) 원장에서는 재고 기능을 쓰지 않는다
    def _require_inventory_tables(self):
        if self.schema_version < 3:
            raise StoreError('schema 3 이전 원장에는 재고 표가 없다')

    # 재고 항목 하나(없으면 None). isolated는 bool
    def inventory_entry(self, object_id):
        self._require_inventory_tables()
        rows = self._rows('SELECT * FROM inventory WHERE object_id=?', (object_id,))
        if not rows:
            return None

        rows[0]['isolated'] = bool(rows[0]['isolated'])

        return rows[0]

    # 전체 재고 항목
    def inventory_entries(self):
        self._require_inventory_tables()
        rows = self._rows('SELECT * FROM inventory ORDER BY object_id')
        for row in rows:
            row['isolated'] = bool(row['isolated'])

        return rows

    # 재고 항목 추가·갱신. 바뀐 필드가 있으면 항목 revision +1, 재고 snapshot revision +1
    def put_inventory_entry(self, entry):
        self._require_inventory_tables()
        current = self.inventory_entry(entry['object_id'])
        fields = {k: entry[k] for k in INVENTORY_FIELDS if k != 'revision'}
        if current is not None:
            if all(current[k] == fields[k] for k in fields):
                return current['revision']
            revision = current['revision'] + 1
        else:
            revision = 1

        self.db.execute(
            'INSERT OR REPLACE INTO inventory VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)',
            (fields['object_id'], revision, fields['receipt_event_id'], fields['object_class'],
             fields['location_id'], fields['carrier_robot_id'], fields['assignment_generation'],
             fields['state'], int(fields['isolated']), fields['task_id']))
        self.bump_inventory_revision()

        return revision

    # 재고 snapshot revision(InventorySnapshot.meta.revision) +1, 원장 영속
    def bump_inventory_revision(self):
        self.db.execute('UPDATE meta SET value=CAST(value AS INTEGER) + 1'
                        " WHERE key='inventory_revision'")

    # 재고 snapshot revision
    def inventory_revision(self):
        self._require_inventory_tables()
        row = self.db.execute("SELECT value FROM meta WHERE key='inventory_revision'").fetchone()

        return int(row[0])

    # 작업 판정과 같은 트랜잭션: 그 작업이 소유한 재고 항목에 그룹 물체 상태를 반영하고 예약을 맞춘다.
    # 재고 항목이 없는 작업(명시 작업 입력·이전 원장)은 바꾸지 않는다
    def _sync_inventory(self, state):
        if self.schema_version < 3:
            return

        entry = self.inventory_entry(state.spec.object_id)
        if entry is not None and entry['task_id'] == state.spec.task_id:
            updated = dict(entry)
            updated.update({
                'state': state.object_state,
                'location_id': state.object_location_id,
                'carrier_robot_id': state.object_carrier_robot_id,
                'assignment_generation': state.object_assignment_generation,
                'isolated': state.object_isolated,
            })
            self.put_inventory_entry(updated)

        # 완료면 예약 소진, 물리 동작 시작 전의 실패 종료·취소면 예약 해제, 그 밖에는 유지
        if state.task_state == gm.TASK_COMPLETED:
            self._set_reservation_state(state.spec.task_id, 'CONSUMED')
        elif state.task_state in (gm.TASK_FAILED, gm.TASK_CANCELED) and not state.physical_started:
            self._set_reservation_state(state.spec.task_id, 'RELEASED')

    # 활성 예약 상태 변경(ACTIVE에서만)
    def _set_reservation_state(self, task_id, new_state):
        cursor = self.db.execute(
            "UPDATE reservations SET state=? WHERE task_id=? AND state='ACTIVE'",
            (new_state, task_id))
        if cursor.rowcount:
            self.bump_inventory_revision()

    # 예약 추가(작업당 1개)
    def insert_reservation(self, reservation_id, location_id, task_id, object_id):
        self._require_inventory_tables()
        self.db.execute("INSERT INTO reservations VALUES (?, ?, ?, ?, 'ACTIVE')",
                        (reservation_id, location_id, task_id, object_id))
        self.bump_inventory_revision()

    # 예약 목록
    def reservations(self):
        self._require_inventory_tables()

        return self._rows('SELECT * FROM reservations ORDER BY rowid')

    # 지점의 점유 수(도착 지점 물품 + 활성 예약)
    def location_load(self, location_id):
        self._require_inventory_tables()
        items = self.db.execute(
            'SELECT COUNT(*) FROM inventory WHERE location_id=? AND state=?',
            (location_id, gm.INV_AT_DESTINATION)).fetchone()[0]
        active = self.db.execute(
            "SELECT COUNT(*) FROM reservations WHERE location_id=? AND state='ACTIVE'",
            (location_id,)).fetchone()[0]

        return items, active

    # 등록 지점 설정 기록(시작 때 설정 반영). revision이 같은데 내용이 다르면 거부
    def sync_locations(self, map_revision, locations):
        self._require_inventory_tables()

        with self._tx():
            for location_id, info in sorted(locations.items()):
                text = json.dumps(info, sort_keys=True)
                row = self.db.execute(
                    'SELECT location_revision, map_revision, info_json FROM locations'
                    ' WHERE location_id=?', (location_id,)).fetchone()
                if row is not None and row[0] == info['location_revision'] \
                        and row[1] == map_revision and row[2] != text:
                    raise StoreConflict(f'같은 revision의 다른 지점 정보: {location_id}')
                if row is None or tuple(row) != (info['location_revision'], map_revision, text):
                    self.db.execute('INSERT OR REPLACE INTO locations VALUES (?, ?, ?, ?)',
                                    (location_id, info['location_revision'], map_revision,
                                     text))
                    self.bump_inventory_revision()

    # 연결 닫기
    def close(self):
        self.db.close()


# 입고 사건 행의 JSON 열을 푼다. 형식 오류는 손상으로 본다
def _decode_event(row):
    try:
        ids = json.loads(row.pop('observation_ids_json'))
    except ValueError as error:
        raise StoreCorrupt(f'입고 사건 형식 오류: {row.get("receipt_event_id")}') from error

    if not isinstance(ids, list):
        raise StoreCorrupt(f'입고 사건 형식 오류: {row.get("receipt_event_id")}')

    row['observation_ids'] = ids

    return row
