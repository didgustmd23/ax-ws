"""통로 원장과 요청 이력을 보존하며 정책 미합의 상태에서는 새 허가를 보류합니다."""
from contextlib import contextmanager
import hashlib
import json
import math
from pathlib import Path
import sqlite3
import threading
import time
import uuid

FREE, RESERVED, OCCUPIED, BLOCKED_UNKNOWN = range(4)
OUTSIDE, APPROACHING, INSIDE, UNKNOWN = range(4)

# 아래 신규 사유 6종은 D-13 추가 제안 상태입니다. 소비자 대응 초안은
# docs/guides/A3_C3_D3_INTEGRATION_REVIEW_DRAFT.md의 D-01~D-06에 정리했습니다.


def boottime():
    # 역할: 절전 중에도 흐르는 시간을 초 단위로 반환합니다.
    return time.clock_gettime(time.CLOCK_BOOTTIME)


class CorridorManager:
    """기존 허가를 보존하고 정책 합의 전에는 새 허가를 발급하지 않습니다."""

    # D3는 통로 허가·점유만 관리합니다. 작업 배정·실제 주행·안전 정지는 상대 파트 책임입니다.

    def __init__(self, database, corridor_ids, robot_ids, *, enabled=False, clock=boottime,
                 grant_policy_approved=False):
        # 역할: 설정과 내부 상태를 준비합니다.
        if (not corridor_ids or len(set(corridor_ids)) != len(corridor_ids)
                or not robot_ids or len(set(robot_ids)) != len(robot_ids)
                or any(not isinstance(item, str) or not item
                       for item in (*corridor_ids, *robot_ids))):
            raise ValueError('unique nonempty corridor and robot IDs required')
        self.corridor_ids = tuple(corridor_ids)
        self.robot_ids = tuple(robot_ids)
        self.enabled = enabled
        # 영역 승인과 허가 정책 합의는 별개입니다. 합의 전에는 기존 원장만 보존합니다.
        # 기본값 False는 발급 보류입니다. True를 쓰는 회귀 시험은 실제 팀 합의를 대신하지 않습니다.
        self.grant_policy_approved = grant_policy_approved
        self.clock = clock
        self.lock = threading.RLock()
        self.observations = {}
        self.assignments = {}
        self.stops = {}
        self.observation_serial = 0
        Path(database).parent.mkdir(parents=True, exist_ok=True)
        self.db = sqlite3.connect(str(database), check_same_thread=False, isolation_level=None)
        self.db.row_factory = sqlite3.Row
        self.db.execute('PRAGMA journal_mode=WAL')
        self.db.execute('PRAGMA synchronous=FULL')
        self.db.executescript("""
            CREATE TABLE IF NOT EXISTS corridors (
                id TEXT PRIMARY KEY, state INTEGER NOT NULL, owner TEXT NOT NULL,
                grant_id TEXT NOT NULL, revision INTEGER NOT NULL, entered INTEGER NOT NULL,
                grant_serial INTEGER NOT NULL);
            CREATE TABLE IF NOT EXISTS metadata (key TEXT PRIMARY KEY, value INTEGER NOT NULL);
            INSERT OR IGNORE INTO metadata VALUES ('revision', 1);
            CREATE TABLE IF NOT EXISTS responses (
                method TEXT, request_id TEXT, fingerprint TEXT, response TEXT,
                PRIMARY KEY(method, request_id));
            CREATE TABLE IF NOT EXISTS waiters (
                ticket INTEGER PRIMARY KEY AUTOINCREMENT, corridor TEXT, robot TEXT,
                deadline REAL, UNIQUE(corridor, robot));
        """)
        with self._atomic():
            existing = {row['id'] for row in self.db.execute('SELECT id FROM corridors')}
            if existing and existing != set(corridor_ids):
                raise ValueError('corridor IDs differ from ledger; do not discard old grants')
            for corridor in corridor_ids:
                self.db.execute('INSERT OR IGNORE INTO corridors '
                                'VALUES (?, ?, ?, ?, 1, 0, 0)',
                                (corridor, BLOCKED_UNKNOWN, '', ''))
                # 재시작은 점유 해제가 아닙니다. owner/grant를 보존하고 새 관측을 기다립니다.
                self._state(corridor, BLOCKED_UNKNOWN)
            self.db.execute('UPDATE corridors SET grant_serial=0')
            self.db.execute('DELETE FROM waiters')

    @contextmanager
    def _atomic(self):
        # 허가 판단과 장부 저장을 한 묶음으로 처리해 동시 요청에 이중 허가가 나가지 않게 합니다.
        # 역할: 판정과 원장 저장을 한 트랜잭션으로 묶고 실패하면 되돌립니다.
        with self.lock:
            self.db.execute('BEGIN IMMEDIATE')
            try:
                yield
                self.db.execute('COMMIT')
            except BaseException:
                self.db.execute('ROLLBACK')
                raise

    def _row(self, corridor):
        # 역할: 해당 통로의 현재 원장 행을 읽습니다.
        return self.db.execute('SELECT * FROM corridors WHERE id=?', (corridor,)).fetchone()

    def _state(self, corridor, state, *, owner=None, grant=None, entered=None, serial=None):
        # 실제 상태가 달라질 때만 영속 revision을 올립니다. 단순 재발행은 새 상태가 아닙니다.
        # 역할: 통로 내용이 바뀔 때만 상태와 영속 수정 번호를 갱신합니다.
        row = self._row(corridor)
        values = (state, row['owner'] if owner is None else owner,
                  row['grant_id'] if grant is None else grant,
                  row['entered'] if entered is None else int(entered),
                  row['grant_serial'] if serial is None else serial)
        previous = (row['state'], row['owner'], row['grant_id'],
                    row['entered'], row['grant_serial'])
        if values == previous:
            return
        self.db.execute("""UPDATE corridors SET state=?, owner=?, grant_id=?, entered=?,
                           grant_serial=?, revision=revision+1 WHERE id=?""", (*values, corridor))
        self.db.execute("UPDATE metadata SET value=value+1 WHERE key='revision'")

    def observe(self, robot, states, valid_for_s=1.0):
        """전체 통로 관측을 반영하고 누락되거나 무효인 관측을 불명으로 처리합니다."""
        # 전체 통로 관측이 필요합니다. 누락·무효·만료를 OUTSIDE로 추측하지 않습니다.
        if robot not in self.robot_ids:
            return
        with self._atomic():
            self.observation_serial += 1
            duration = min(1.0, valid_for_s) if math.isfinite(valid_for_s) else 0.0
            complete = set(states) == set(self.corridor_ids) and duration > 0
            for corridor in self.corridor_ids:
                state, valid = states.get(corridor, (UNKNOWN, False))
                if not complete or not valid or state not in (OUTSIDE, APPROACHING, INSIDE):
                    state = UNKNOWN
                self.observations[robot, corridor] = (
                    state, self.clock() + max(0.0, duration), self.observation_serial)
            self._refresh()

    def assign(self, robots, valid_for_s=3.0):
        """상위 배정에서 실행 가능한 로봇과 그 유효기한만 받아 보관합니다."""
        # D2가 배정한 로봇 목록을 받아 유효기간만 관리합니다. D3가 새 작업을 배정하지 않습니다.
        with self.lock:
            duration = min(3.0, valid_for_s) if math.isfinite(valid_for_s) else 0.0
            self.assignments = {robot: self.clock() + max(0.0, duration) for robot in robots
                                if robot in self.robot_ids}

    def stopped(self, robot, confirmed, valid_for_s=0.3):
        # 역할: 로봇의 정지 확인 여부와 확인 기한을 보관합니다.
        with self.lock:
            duration = min(0.3, valid_for_s) if math.isfinite(valid_for_s) else 0.0
            self.stops[robot] = (bool(confirmed), self.clock() + max(0.0, duration))

    def _observation(self, robot, corridor):
        # 역할: 기한이 남은 통로 관측과 관측 순번을 읽습니다.
        state, deadline, serial = self.observations.get((robot, corridor), (UNKNOWN, 0.0, 0))
        return (state if self.clock() < deadline else UNKNOWN), serial

    def _refresh(self):
        # 대기표의 만료와 물리 점유 해제는 다릅니다. 관측이 끊겨도 기존 owner/grant는 보존합니다.
        # 역할: 대기표와 관측 만료를 반영하되 물리 이탈 없이 기존 허가를 지우지 않습니다.
        self.db.execute('DELETE FROM waiters WHERE deadline<=?', (self.clock(),))
        for corridor in self.corridor_ids:
            row = self._row(corridor)
            observed = {robot: self._observation(robot, corridor)[0] for robot in self.robot_ids}
            if not self.enabled or not self.grant_policy_approved or UNKNOWN in observed.values():
                # 발급 보류를 FREE로 표현하면 소비자가 진입 가능으로 오해할 수 있어 불명으로 둡니다.
                self._state(corridor, BLOCKED_UNKNOWN)
            elif row['owner']:
                others_inside = any(state == INSIDE for robot, state in observed.items()
                                    if robot != row['owner'])
                if others_inside:
                    self._state(corridor, BLOCKED_UNKNOWN)
                elif observed[row['owner']] == INSIDE:
                    self._state(corridor, OCCUPIED, entered=True)
                else:
                    # 밖으로 보이더라도 grant를 임의 해제하지 않습니다. ReleaseCorridor로 확인합니다.
                    self._state(corridor, OCCUPIED if row['entered'] else RESERVED)
            else:
                self._state(corridor, BLOCKED_UNKNOWN if INSIDE in observed.values() else FREE)

    def snapshot(self):
        # 역할: 최신 관측을 반영한 전체 상태와 수정 번호를 반환합니다.
        with self._atomic():
            self._refresh()
            revision = self.db.execute(
                "SELECT value FROM metadata WHERE key='revision'").fetchone()[0]
            return revision, [dict(self._row(corridor)) for corridor in self.corridor_ids]

    def _cached(self, method, request_id, arguments):
        # 같은 요청 번호는 같은 응답을 돌려줍니다. 번호는 같은데 내용이 달라지면 거절합니다.
        # 역할: 요청 식별자의 기존 응답과 인자 불일치를 확인합니다.
        fingerprint = hashlib.sha256(json.dumps(arguments, sort_keys=True).encode()).hexdigest()
        row = self.db.execute('SELECT * FROM responses WHERE method=? AND request_id=?',
                              (method, request_id)).fetchone()
        if row and row['fingerprint'] != fingerprint:
            return fingerprint, {'reason_code': 'DUPLICATE_REQUEST'}
        return fingerprint, json.loads(row['response']) if row else None

    def _save(self, method, request_id, fingerprint, response):
        # 역할: 요청 인자 지문과 응답을 원장에 함께 저장합니다.
        self.db.execute('INSERT INTO responses VALUES (?, ?, ?, ?)',
                        (method, request_id, fingerprint, json.dumps(response)))
        return response

    def request(self, corridor, robot, request_id):
        """과거 요청은 재현하고 새 요청은 정책 합의와 배정 및 정지 조건을 검사합니다."""
        # 운영 기본값은 발급 보류입니다. 아래 발급 알고리즘은 합의를 가정한 회귀 시험에 남깁니다.
        with self._atomic():
            self._refresh()
            fingerprint, cached = self._cached('entry', request_id, [corridor, robot])
            if cached is not None:
                # 과거 응답의 재전송입니다. 현재 진입에는 최신 상태와 안전 게이트도 확인해야 합니다.
                # 보류 중에도 과거 응답을 지우지 않습니다. granted=True 재현은 새 허가 발급이 아닙니다.
                return dict({'granted': False, 'grant_id': '', 'corridor_revision': 0}, **cached)
            reason = ''
            row = self._row(corridor)
            if not request_id or row is None or robot not in self.robot_ids:
                reason = 'AUTHORITY_INVALID'
            elif not self.enabled:
                reason = 'REGIONS_NOT_APPROVED'
            elif not self.grant_policy_approved:
                # 미합의 정책으로 새 허가를 발급하거나 기존 허가를 재승인하지 않습니다.
                # 새 사유 코드를 임의로 추가하지 않고 기존 AUTHORITY_INVALID를 사용합니다.
                reason = 'AUTHORITY_INVALID'
            elif self.assignments.get(robot, 0.0) <= self.clock():
                reason = 'ASSIGNMENT_STALE'
            elif (self.stops.get(robot, (False, 0))[0] is not True
                  or self.stops.get(robot, (False, 0))[1] <= self.clock()):
                # D-01: 정지 요청을 보냈다는 사실이 아니라 실제 정지 확인이 필요합니다.
                reason = 'STOP_NOT_CONFIRMED'
            elif row['owner'] == robot:
                # 기존 소유자도 새 요청에서는 위의 배정·정지 검사를 생략하지 않습니다.
                if row['state'] == BLOCKED_UNKNOWN:
                    reason = 'CORRIDOR_BLOCKED'
                else:
                    return self._save('entry', request_id, fingerprint, {
                        'granted': True, 'grant_id': row['grant_id'], 'reason_code': '',
                        'corridor_revision': row['revision']})
            else:
                self.db.execute("""INSERT INTO waiters(corridor, robot, deadline) VALUES (?, ?, ?)
                    ON CONFLICT(corridor, robot) DO UPDATE SET deadline=excluded.deadline""",
                                (corridor, robot, self.clock() + 5.0))
                head = self.db.execute(
                    'SELECT robot FROM waiters WHERE corridor=? ORDER BY ticket LIMIT 1',
                    (corridor,)).fetchone()
                if row['state'] != FREE or row['owner']:
                    reason = 'CORRIDOR_BLOCKED'
                elif head['robot'] != robot:
                    # D-02: 이 개발안에서는 순서 대기입니다. 전체 작업 실패 결정은 소비자 몫입니다.
                    reason = 'WAITING_TURN'
            if reason:
                return self._save('entry', request_id, fingerprint, {
                    'granted': False, 'grant_id': '', 'reason_code': reason,
                    'corridor_revision': row['revision'] if row else 0})
            grant = 'grant-' + uuid.uuid4().hex
            # 허가 당시의 관측 순번을 기억해, 그보다 이전 OUTSIDE로 곧바로 해제하지 못하게 합니다.
            serial = self._observation(robot, corridor)[1]
            self._state(corridor, RESERVED, owner=robot, grant=grant, entered=False, serial=serial)
            self.db.execute('DELETE FROM waiters WHERE corridor=? AND robot=?', (corridor, robot))
            return self._save('entry', request_id, fingerprint, {
                'granted': True, 'grant_id': grant, 'reason_code': '',
                'corridor_revision': self._row(corridor)['revision']})

    def release(self, corridor, robot, grant, request_id):
        # 보류는 기존 허가 삭제가 아닙니다. 소유권과 완전 이탈 증거가 맞는 정상 해제는 허용합니다.
        """현재 소유권과 이탈 증거를 확인하거나 실행 중 선점을 해제합니다."""
        # 현재 owner/grant와 새 완전 이탈 관측을 확인합니다. 시간초과만으로 해제하지 않습니다.
        with self._atomic():
            self._refresh()
            fingerprint, cached = self._cached('release', request_id, [corridor, robot, grant])
            if cached is not None:
                return dict({'accepted': False, 'corridor_revision': 0}, **cached)
            row = self._row(corridor)
            reason = ''
            if (not request_id or row is None or not grant
                    or row['owner'] != robot or row['grant_id'] != grant):
                # D-03: 잘못된 소유권을 임의로 교정하거나 현재 허가를 삭제하지 않습니다.
                reason = 'GRANT_MISMATCH'
            elif not self.enabled:
                reason = 'REGIONS_NOT_APPROVED'
            else:
                state, serial = self._observation(robot, corridor)
                others = [self._observation(peer, corridor)[0]
                          for peer in self.robot_ids if peer != robot]
                if state != OUTSIDE or serial <= row['grant_serial']:
                    reason = 'EXIT_NOT_CONFIRMED'
                elif any(state in (UNKNOWN, INSIDE) for state in others):
                    # 내 로봇만 나가도 통로 전체가 빈 것은 아닙니다. 관측 회복 후 재요청합니다.
                    reason = 'CORRIDOR_BLOCKED'
            if not reason:
                self._state(corridor, FREE, owner='', grant='', entered=False, serial=0)
            return self._save('release', request_id, fingerprint, {
                'accepted': not reason, 'reason_code': reason,
                'corridor_revision': self._row(corridor)['revision'] if row else 0})

    def close(self):
        # 역할: 보유한 원장과 내부 자원을 안전하게 닫습니다.
        with self.lock:
            self.db.close()
