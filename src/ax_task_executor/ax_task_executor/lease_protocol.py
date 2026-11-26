"""C4 lease 토픽과 A2/A4 명령 확인이 같은 권한·수락 원장을 사용하게 합니다."""
import hashlib
import json
import math
import re
import sqlite3


class LeaseProtocol:
    """현재 배정·활성 Goal·위임 예산에 한해 세대를 영속 발급합니다. 외부 요청은 Goal을 만들지 못합니다."""

    def __init__(self, authority, ledger):
        """서비스의 원장을 공유하며 프로세스 재시작 시 권한은 자동 복원하지 않습니다."""
        self.authority, self.ledger = authority, ledger
        self.budgets = {}
        self.failed = False
        self.notified_revoke = None
        with ledger.lock:
            ledger.db.execute('CREATE TABLE IF NOT EXISTS lease_generations ('
                              'robot TEXT, execution TEXT, generation INTEGER, goal TEXT, '
                              'session TEXT, revoked INTEGER, PRIMARY KEY(robot, execution))')
            ledger.db.execute('CREATE TABLE IF NOT EXISTS lease_responses ('
                              'robot TEXT, request TEXT, digest TEXT, response TEXT, '
                              'PRIMARY KEY(robot, request))')
            ledger.db.execute('CREATE TABLE IF NOT EXISTS lease_revocations ('
                              'robot TEXT, execution TEXT, generation INTEGER, '
                              'goal TEXT, reason TEXT, '
                              'PRIMARY KEY(robot, execution, generation))')
            ledger.db.execute('CREATE TABLE IF NOT EXISTS lease_goals ('
                              'robot TEXT, execution TEXT, goal TEXT, session TEXT, budgets TEXT, '
                              'resyncs INTEGER NOT NULL DEFAULT 0, '
                              'PRIMARY KEY(robot, execution, goal, session))')

    def register_budgets(self, execution_id, goal_id, budgets):
        """A3 내부가 기록한 현재 Goal의 예산만 등록합니다. 요청 토픽에서 호출하지 않습니다."""
        with self.authority.lock:
            ctx = self.authority.context
            if ctx is None or (ctx.execution_id, ctx.goal_id) != (execution_id, goal_id):
                raise ValueError('matching active goal required')
            valid = {}
            now = self.authority.clock()
            for budget in budgets:
                if (budget.kind == 'GRASP_ATTEMPT' and budget.granted_attempts > 0
                        and execution_id in budget.allowed_execution_ids
                        and budget.budget_grant_id and math.isfinite(budget.time_budget_s)
                        and budget.time_budget_s > 0):
                    if budget.budget_grant_id in valid:
                        raise ValueError('duplicate budget ID')
                    valid[budget.budget_grant_id] = now + budget.time_budget_s
            if not valid:
                raise ValueError('delegated GRASP_ATTEMPT budget required')
            key = (execution_id, goal_id)
            if key in self.budgets:
                raise ValueError('budget registration cannot extend an existing goal')
            with self.ledger.lock:
                self.ledger.db.execute(
                    'INSERT INTO lease_goals (robot, execution, goal, session, budgets) '
                    'VALUES (?, ?, ?, ?, ?)',
                    (self.authority.robot_id, *key, self.authority.session_id, json.dumps(valid)))
            self.budgets = {key: valid}

    @staticmethod
    def fields(request):
        """계약 필드만 복사해 요청 지문을 만듭니다."""
        return {key: getattr(request, key) for key in (
            'kind', 'request_id', 'execution_id', 'goal_id', 'c_session_id',
            'budget_grant_id', 'lease_generation')}

    @staticmethod
    def reply(fields, reason='', generation=0, duration=0.0):
        """응답에 요청의 실행·부모 Goal 식별자를 그대로 돌려줍니다."""
        # Q11: reason_code도 공유 메시지의 계약 표면이다. 내부 예외 문자열을 넣거나
        # 기존 코드의 의미를 임의로 바꾸지 않는다. 새 운영 사유는 D-13 검토 대상으로 정리한다.
        return {'request_id': fields['request_id'], 'execution_id': fields['execution_id'],
                'goal_id': fields['goal_id'], 'lease_generation': generation,
                'granted': not bool(reason), 'reason_code': reason, 'lease_s': duration}

    def handle(self, request):
        """지문 검사를 권한 재검사보다 먼저 하고, 거절도 저장해 같은 ID의 답을 고정합니다."""
        fields = self.fields(request)
        digest = hashlib.sha256(json.dumps(fields, sort_keys=True).encode()).hexdigest()
        robot = self.authority.robot_id
        events = []
        # 서비스와 동일하게 RuntimeAuthority → LeaseLedger 순서로 잠근다. 수락·철회와 경합하지 않는다.
        with self.authority.lock, self.ledger.lock:
            self.resync_unverified = False
            try:
                db = self.ledger.db
                db.execute('BEGIN IMMEDIATE')
                cached = db.execute(
                    'SELECT digest, response FROM lease_responses WHERE robot=? AND request=?',
                    (robot, fields['request_id'])).fetchone()
                if cached:
                    db.execute('COMMIT')
                    return (json.loads(cached[1]) if cached[0] == digest else
                            self.reply(fields, 'DUPLICATE_REQUEST')), events
                # 장애 고정 뒤에도 읽을 수 있는 기존 응답은 바꾸지 않는다. 새 발급만 막는다.
                if self.failed:
                    result, install = self.reply(fields, 'A3_RECORD_FAILED'), None
                else:
                    result, events, install = self._decide(fields)
                db.execute('INSERT INTO lease_responses VALUES (?, ?, ?, ?)',
                           (robot, fields['request_id'], digest, json.dumps(result)))
                # FULL 커밋이 성공하기 전에는 grant를 발행하거나 메모리 권한을 허용하지 않는다.
                db.execute('COMMIT')
                if install is not None:
                    ctx = self.authority.context
                    ctx.lease_generation, ctx.lease_deadline = install
                return result, events
            except (sqlite3.Error, ValueError, OverflowError):
                try:
                    self.ledger.db.execute('ROLLBACK')
                except sqlite3.Error:
                    pass
                self.failed = True
                self.authority.invalidate_assignment()
                reason = 'LEASE_RESYNC_UNCERTAIN' if self.resync_unverified else 'A3_RECORD_FAILED'
                return self.reply(fields, reason), events

    def _decide(self, fields):
        """최초 발급·갱신·재동기화를 구분합니다. 호출자는 두 잠금과 트랜잭션을 보유합니다."""
        def deny(reason, events=None):
            return self.reply(fields, reason), events or [], None

        identifier = r'[A-Za-z0-9_][A-Za-z0-9_.:\-]{0,127}'
        if (fields['kind'] not in (1, 2)
                or re.fullmatch(r'[0-9a-f]{32}', fields['goal_id']) is None
                or any(re.fullmatch(identifier, fields[key]) is None for key in
                       ('request_id', 'execution_id', 'c_session_id', 'budget_grant_id'))):
            return deny('AUTHORITY_INVALID')
        authority = self.authority
        authority.expire_assignment()
        ctx, now = authority.context, authority.clock()
        if authority.assignment is None or now >= authority.assignment_deadline:
            return deny('ASSIGNMENT_STALE')
        if (ctx is None or ctx.local_task_kind or (ctx.execution_id, ctx.goal_id) !=
                (fields['execution_id'], fields['goal_id'])):
            return deny('GOAL_MISMATCH')
        if authority.assignment != (ctx.assignment_id, ctx.assignment_generation):
            return deny('ASSIGNMENT_STALE')
        if ctx.revoked:
            return deny('LEASE_REVOKED')
        budget_end = self.budgets.get((ctx.execution_id, ctx.goal_id), {}).get(
            fields['budget_grant_id'], 0)
        if now >= budget_end:
            return deny('BUDGET_EXHAUSTED')
        row = self.ledger.db.execute(
            'SELECT generation, goal, session, revoked FROM lease_generations '
            'WHERE robot=? AND execution=?', (authority.robot_id, ctx.execution_id)).fetchone()
        events = []
        if fields['kind'] == 2:
            if (row is None or row[3] or row[2] != authority.session_id
                    or row[1] != ctx.goal_id or fields['lease_generation'] != row[0]
                    or ctx.lease_generation != row[0]):
                return deny('LEASE_REVOKED')
            if now >= ctx.lease_deadline:
                return deny('LEASE_EXPIRED')
            generation = row[0]
        else:
            self.resync_unverified = row is not None
            if row is not None:
                # 재동기화는 이전 세대를 먼저 차단한다. 기록 확인 실패나 ACCEPT가 있으면 새 세대를 주지 않는다.
                ctx.lease_deadline = 0.0
                self.ledger.db.execute(
                    'UPDATE lease_generations SET revoked=1 WHERE robot=? AND execution=?',
                    (authority.robot_id, ctx.execution_id))
                self.ledger.db.execute(
                    'INSERT OR IGNORE INTO lease_revocations VALUES (?, ?, ?, ?, ?)',
                    (authority.robot_id, ctx.execution_id, row[0], row[1], 'LEASE_REVOKED'))
                # FIRST 재동기화는 같은 Goal을 유지한다. 이전 세대는 원장에서 차단하되
                # 현재 Goal 전체를 종료시키는 revoke/cancel 통지는 명시적 철회 경로에서만 보낸다.
                key = (authority.robot_id, ctx.execution_id, ctx.goal_id, authority.session_id)
                count = self.ledger.db.execute(
                    'SELECT resyncs FROM lease_goals WHERE '
                    'robot=? AND execution=? AND goal=? AND session=?', key).fetchone()
                if count is None or count[0] >= 2:
                    ctx.revoked = True
                    return deny('LEASE_EXPIRED', events)
                self.ledger.db.execute('UPDATE lease_goals SET resyncs=resyncs+1 WHERE '
                                       'robot=? AND execution=? AND goal=? AND session=?', key)
            # 서비스가 남긴 같은 execution_id의 물리 명령 수락 기록을 직접 확인한다.
            try:
                accepted = self.ledger.db.execute(
                    'SELECT 1 FROM accepts WHERE robot=? '
                    "AND json_extract(command, '$.execution_id')=? LIMIT 1",
                    (authority.robot_id, ctx.execution_id)).fetchone()
            except sqlite3.Error:
                self.resync_unverified = True
                raise
            if accepted:
                ctx.revoked = True
                return deny('LEASE_RESYNC_UNCERTAIN', events)
            generation = (row[0] if row else 0) + 1
        duration = math.floor(min(1.5, authority.assignment_deadline - now,
                                  budget_end - now) * 1000) / 1000
        if duration <= 0:
            return deny('LEASE_EXPIRED', events)
        self.ledger.db.execute(
            'INSERT INTO lease_generations VALUES (?, ?, ?, ?, ?, 0) '
            'ON CONFLICT(robot, execution) DO UPDATE SET generation=excluded.generation, '
            'goal=excluded.goal, session=excluded.session, revoked=0',
            (authority.robot_id, ctx.execution_id, generation, ctx.goal_id, authority.session_id))
        return (self.reply(fields, generation=generation, duration=duration),
                events, (generation, now + duration))

    def revoke(self, reason='LEASE_REVOKED'):
        """명령 확인을 먼저 차단한 뒤 철회 발행·Action cancel에 필요한 값을 돌려줍니다."""
        with self.authority.lock:
            ctx = self.authority.context
            if ctx is None or not ctx.lease_generation:
                return []
            key = (ctx.execution_id, ctx.goal_id, ctx.lease_generation)
            if self.notified_revoke == key:
                return []
            self.authority.revoke(ctx.execution_id, ctx.goal_id, ctx.lease_generation)
            try:
                with self.ledger.lock:
                    self.ledger.db.execute(
                        'UPDATE lease_generations SET revoked=1 WHERE robot=? AND execution=?',
                        (self.authority.robot_id, ctx.execution_id))
                    self.ledger.db.execute(
                        'INSERT OR IGNORE INTO lease_revocations VALUES (?, ?, ?, ?, ?)',
                        (self.authority.robot_id, ctx.execution_id, ctx.lease_generation,
                         ctx.goal_id, reason))
            except sqlite3.Error:
                # 저장 장애도 실제 정지 통지를 막지 않는다. 새 grant는 장애 고정으로 계속 거절한다.
                self.failed = True
            self.notified_revoke = key
            return [{'execution_id': ctx.execution_id, 'goal_id': ctx.goal_id,
                     'lease_generation': ctx.lease_generation,
                     'reason_code': reason, 'cancel': True}]
