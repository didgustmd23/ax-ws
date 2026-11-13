"""ROS-independent corridor state machine with SQLite grant/response records."""
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


def boottime():
    return time.clock_gettime(time.CLOCK_BOOTTIME)


class CorridorManager:
    """Keep one owner per corridor; a timeout never proves physical exit."""

    # D3는 통로 허가·점유만 관리합니다. 작업 배정·실제 주행·안전 정지는 상대 파트 책임입니다.

    def __init__(self, database, corridor_ids, robot_ids, *, enabled=False, clock=boottime):
        if (not corridor_ids or len(set(corridor_ids)) != len(corridor_ids)
                or not robot_ids or len(set(robot_ids)) != len(robot_ids)
                or any(not isinstance(item, str) or not item
                       for item in (*corridor_ids, *robot_ids))):
            raise ValueError('unique nonempty corridor and robot IDs required')
        self.corridor_ids = tuple(corridor_ids)
        self.robot_ids = tuple(robot_ids)
        self.enabled = enabled
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
        with self.lock:
            self.db.execute('BEGIN IMMEDIATE')
            try:
                yield
                self.db.execute('COMMIT')
            except BaseException:
                self.db.execute('ROLLBACK')
                raise

    def _row(self, corridor):
        return self.db.execute('SELECT * FROM corridors WHERE id=?', (corridor,)).fetchone()

    def _state(self, corridor, state, *, owner=None, grant=None, entered=None, serial=None):
        # 실제 상태가 달라질 때만 영속 revision을 올립니다. 단순 재발행은 새 상태가 아닙니다.
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
        """Accept a validated full B3 snapshot; missing/invalid entries become UNKNOWN."""
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
        """Consume D2 eligibility only; never allocate tasks in D3."""
        # D2가 배정한 로봇 목록을 받아 유효기간만 관리합니다. D3가 새 작업을 배정하지 않습니다.
        with self.lock:
            duration = min(3.0, valid_for_s) if math.isfinite(valid_for_s) else 0.0
            self.assignments = {robot: self.clock() + max(0.0, duration) for robot in robots
                                if robot in self.robot_ids}

    def stopped(self, robot, confirmed, valid_for_s=0.3):
        with self.lock:
            duration = min(0.3, valid_for_s) if math.isfinite(valid_for_s) else 0.0
            self.stops[robot] = (bool(confirmed), self.clock() + max(0.0, duration))

    def _observation(self, robot, corridor):
        state, deadline, serial = self.observations.get((robot, corridor), (UNKNOWN, 0.0, 0))
        return (state if self.clock() < deadline else UNKNOWN), serial

    def _refresh(self):
        # 대기표의 만료와 물리 점유 해제는 다릅니다. 관측이 끊겨도 기존 owner/grant는 보존합니다.
        self.db.execute('DELETE FROM waiters WHERE deadline<=?', (self.clock(),))
        for corridor in self.corridor_ids:
            row = self._row(corridor)
            observed = {robot: self._observation(robot, corridor)[0] for robot in self.robot_ids}
            if not self.enabled or UNKNOWN in observed.values():
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
        with self._atomic():
            self._refresh()
            revision = self.db.execute(
                "SELECT value FROM metadata WHERE key='revision'").fetchone()[0]
            return revision, [dict(self._row(corridor)) for corridor in self.corridor_ids]

    def _cached(self, method, request_id, arguments):
        # 같은 요청 번호는 같은 응답을 돌려줍니다. 번호는 같은데 내용이 달라지면 거절합니다.
        fingerprint = hashlib.sha256(json.dumps(arguments, sort_keys=True).encode()).hexdigest()
        row = self.db.execute('SELECT * FROM responses WHERE method=? AND request_id=?',
                              (method, request_id)).fetchone()
        if row and row['fingerprint'] != fingerprint:
            return fingerprint, {'reason_code': 'DUPLICATE_REQUEST'}
        return fingerprint, json.loads(row['response']) if row else None

    def _save(self, method, request_id, fingerprint, response):
        self.db.execute('INSERT INTO responses VALUES (?, ?, ?, ?)',
                        (method, request_id, fingerprint, json.dumps(response)))
        return response

    def request(self, corridor, robot, request_id):
        """Issue one durable grant; callers wait/retry with new request IDs."""
        # 새 진입에는 유효한 배정·실제 정지·통로 상태와 대기 순서가 모두 맞아야 합니다.
        with self._atomic():
            self._refresh()
            fingerprint, cached = self._cached('entry', request_id, [corridor, robot])
            if cached is not None:
                return dict({'granted': False, 'grant_id': '', 'corridor_revision': 0}, **cached)
            reason = ''
            row = self._row(corridor)
            if not request_id or row is None or robot not in self.robot_ids:
                reason = 'AUTHORITY_INVALID'
            elif not self.enabled:
                reason = 'REGIONS_NOT_APPROVED'
            elif row['owner'] == robot:
                return self._save('entry', request_id, fingerprint, {
                    'granted': True, 'grant_id': row['grant_id'], 'reason_code': '',
                    'corridor_revision': row['revision']})
            elif self.assignments.get(robot, 0.0) <= self.clock():
                reason = 'ASSIGNMENT_STALE'
            elif (self.stops.get(robot, (False, 0))[0] is not True
                  or self.stops.get(robot, (False, 0))[1] <= self.clock()):
                reason = 'STOP_NOT_CONFIRMED'
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
        """Release only on fresh full-exit evidence newer than the grant."""
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
                    reason = 'CORRIDOR_BLOCKED'
            if not reason:
                self._state(corridor, FREE, owner='', grant='', entered=False, serial=0)
            return self._save('release', request_id, fingerprint, {
                'accepted': not reason, 'reason_code': reason,
                'corridor_revision': self._row(corridor)['revision'] if row else 0})

    def close(self):
        with self.lock:
            self.db.close()
