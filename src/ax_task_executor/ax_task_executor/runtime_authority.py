"""배정과 활성 작업 및 실행 권한을 관리합니다. 외부 명령 확인 요청은 권한을 만들지 않습니다."""
from contextlib import contextmanager
from dataclasses import dataclass
import math
import sqlite3
import threading
import time
import uuid
import weakref


def boottime():
    """절전 중에도 흐르는 시간을 초 단위로 반환합니다."""
    # 절전 중에도 흐른 시간을 셉니다. 컴퓨터가 잠들었다고 권한 기한이 늘어나면 안 됩니다.
    return time.clock_gettime(time.CLOCK_BOOTTIME)


@dataclass
class ActiveCommandContext:
    """현재 배정에 연결된 활성 작업과 실행 권한을 보관합니다."""

    execution_id: str
    # 배정 세대와 lease 세대는 다른 번호입니다. 담당 변경과 실행 권한 교체를 각각 구별합니다.
    assignment_id: str
    assignment_generation: int
    goal_id: str
    local_task_kind: int = 0
    lease_generation: int = 0
    lease_deadline: float = 0.0
    revoked: bool = False


class RuntimeAuthority:
    """권한 변경과 명령 수락 기록을 직렬화합니다."""

    policy = 'operational'
    marks = ('', 'accept-')
    CHECK_LIFETIME_S = 2.0
    MAX_PENDING_CHECKS = 64

    def __init__(self, robot_id, clock=boottime):
        # 재시작마다 새 세션을 만듭니다. 예전 응답·장부만으로 실행 권한을 복원하지 않습니다.
        # 역할: 설정과 내부 상태를 준비합니다.
        self.robot_id = robot_id
        self.clock = clock
        self.lock = threading.RLock()
        self.session_id = uuid.uuid4().hex
        self.assignment = None
        self.assignment_deadline = 0.0
        self.context = None
        self.pending = {}
        # 평생 중복 방지 이력은 임시 디스크에 저장해 메모리가 계속 증가하지 않게 합니다.
        # SQLite는 원장을 닫으면 임시 파일을 제거합니다.
        # 새 프로세스는 새 A 세션을 사용하므로 예전 응답을 사용할 수 없습니다.
        self._check_history = sqlite3.connect('', check_same_thread=False, isolation_level=None)
        # 중복 방지 이력은 임시 파일에 남기고, 메모리에는 아직 응답을 기다릴 요청만 둡니다.
        self._check_history.execute('PRAGMA cache_size=-256')
        self._check_history.execute("""CREATE TABLE sent_checks (
            request_id TEXT PRIMARY KEY, assignment_id TEXT NOT NULL,
            generation INTEGER NOT NULL, sent_at REAL NOT NULL)""")
        self._close_history = weakref.finalize(self, self._check_history.close)
        self.d_session_id = None
        self.sequence = -1
        self.requested_assignment = None

    def begin_assignment_check(self, request_id, assignment_id, generation):
        """배정 확인 발신 시각을 기록하고 중복이나 과도한 대기를 거절합니다."""
        # D4에 보내기 전에 최초 발신 시각을 기록합니다. 재전송해도 이 시각은 바꾸지 않습니다.
        if not request_id or not assignment_id or generation <= 0:
            raise ValueError('assignment ID, request ID and positive generation required')
        with self.lock:
            now = self.clock()
            self._prune_checks(now)
            value = (assignment_id, generation)
            previous = self._check_history.execute(
                'SELECT assignment_id, generation, sent_at FROM sent_checks WHERE request_id=?',
                (request_id,)).fetchone()
            if previous is not None:
                if previous[:2] != value:
                    raise ValueError('request_id reused for another assignment')
                return
            # 현재 확인 중인 배정만 이후 응답으로 확정할 수 있습니다.
            if self.assignment is not None and self.assignment != value:
                self._invalidate()
            if self.requested_assignment != value:
                self.pending.clear()
            if len(self.pending) >= self.MAX_PENDING_CHECKS:
                raise ValueError('too many outstanding assignment checks')
            # 발행 전에 기록하며 완료되거나 만료된 요청 식별자를 다시 등록하지 않습니다.
            # 메모리의 대기 목록에서 제거돼도 중복 방지 이력은 유지합니다.
            self._check_history.execute('INSERT INTO sent_checks VALUES (?, ?, ?, ?)',
                                        (request_id, assignment_id, generation, now))
            self.requested_assignment = value
            self.pending[request_id] = (*value, now)

    def _prune_checks(self, now):
        """응답이 와도 권한을 줄 수 없는 만료된 대기 요청을 제거합니다."""
        # 최대 유효시간이 지난 요청은 대기 목록에서 제거하되, 중복 ID 이력은 유지합니다.
        expired = [key for key, value in self.pending.items()
                   if now >= value[2] + self.CHECK_LIFETIME_S]
        for key in expired:
            del self.pending[key]

    def close(self):
        """보유한 원장과 내부 자원을 안전하게 닫습니다."""
        with self.lock:
            self.invalidate_assignment()
            self._close_history()

    def confirm_assignment(self, reply):
        """요청과 세션 및 세대가 맞는 응답만 배정 확인으로 반영합니다."""
        # 요청 ID → A 세션 → 배정 → D 세션·순번을 대조한 뒤에만 응답을 반영합니다.
        with self.lock:
            self.expire_assignment()
            pending = self.pending.get(reply.request_id)
            if pending is None or reply.a_session_id != self.session_id:
                return False
            assignment_id, generation, sent_at = pending
            if (assignment_id, generation) != self.requested_assignment:
                return False
            if not reply.d_session_id:
                return False
            if self.d_session_id == reply.d_session_id and reply.sequence <= self.sequence:
                return False
            # 허용 응답은 요청 세대와 일치해야 합니다. 신선한 거절 응답은
            # 상위 원장의 새 현재 세대를 알릴 수 있으므로 요청과 연결해 검사합니다.
            # 그 거절을 무시하지 않고 이전 권한을 폐기합니다.
            if reply.valid and reply.assignment_generation != generation:
                # 허용 응답은 요청 세대와 같아야 합니다. 명시적 거절은 새 세대여도 아래에서 처리합니다.
                return False
            # 응답은 한 번만 반영하며 중복 응답으로 권한 기한을 연장하지 않습니다.
            del self.pending[reply.request_id]
            if self.d_session_id not in (None, reply.d_session_id):
                self._invalidate()
                self.pending.clear()
            self.d_session_id = reply.d_session_id
            self.sequence = reply.sequence
            duration = float(reply.validity_s)
            if (not reply.valid or not math.isfinite(duration) or duration <= 0
                    or sent_at + min(duration, self.CHECK_LIFETIME_S) <= self.clock()):
                # 현재 D4의 거절은 즉시 반영하고, 다른 지연 응답이 권한을 되살리지 못하게 합니다.
                self.invalidate_assignment()
                return False
            # 콜백 지연이나 절전 중 배정이 만료됐다면 늦은 확인 응답으로
            # 그 배정에 연결된 이전 작업을 되살리지 않습니다.
            self.expire_assignment()
            # 이전 배정의 지연 응답은 현재 권한을 복원하지 못합니다.
            if self.assignment is not None and self.assignment != (assignment_id, generation):
                self._invalidate()
            self.assignment = (assignment_id, generation)
            self.assignment_deadline = sent_at + min(duration, self.CHECK_LIFETIME_S)
            # 기한은 응답을 받은 시각이 아니라 최초 요청 시각부터 계산합니다.
            return self.assignment_deadline > self.clock()

    def _invalidate(self):
        # 배정이 끊기면 기존 Goal도 철회합니다. D4가 복구돼도 옛 동작을 자동 재개하지 않습니다.
        # 역할: 현재 배정을 지우고 기존 작업 권한을 철회합니다.
        self.assignment = None
        self.assignment_deadline = 0.0
        if self.context is not None:
            self.context.revoked = True

    def invalidate_assignment(self):
        """배정 권한과 대기 중인 확인 요청을 함께 무효화합니다."""
        with self.lock:
            self._invalidate()
            self.pending.clear()
            self.requested_assignment = None

    def expire_assignment(self):
        """배정 기한이 끝났다면 기존 작업이 되살아나지 않도록 무효화합니다."""
        with self.lock:
            self._prune_checks(self.clock())
            if self.assignment is not None and self.clock() >= self.assignment_deadline:
                self._invalidate()

    def activate_goal(self, execution, goal_id='', local_task_kind=0):
        """실제로 기록한 작업과 현재 배정에 연결된 활성 요청을 등록합니다."""
        import re

        # 실제 작업을 기록한 A3 내부에서만 호출합니다. 외부 권한 확인 요청은 Goal을 만들 수 없습니다.
        if not execution.execution_id:
            raise ValueError('execution_id required')
        if local_task_kind:
            if local_task_kind != 6 or goal_id:
                raise ValueError('only RETRACT_ARM has local authority')
        elif re.fullmatch('[0-9a-f]{32}', goal_id) is None:
            raise ValueError('parent ManipulateObject UUID required')
        with self.lock:
            assignment = (execution.assignment_id, execution.assignment_generation)
            if assignment != self.assignment or self.clock() >= self.assignment_deadline:
                raise ValueError('assignment is not confirmed')
            if self.context is not None:
                raise ValueError('finish previous goal before activating another')
            self.context = ActiveCommandContext(
                execution.execution_id, *assignment, goal_id, local_task_kind)

    def set_lease(self, execution_id, goal_id, generation, deadline):
        """현재 작업의 실행 권한 세대와 기한을 등록합니다."""
        with self.lock:
            # 발급기가 기록한 권한만 등록합니다. 세대 영속 발급·재동기화 전체를 대신하는 함수는 아닙니다.
            ctx = self.context
            now = self.clock()
            if (ctx is None or ctx.local_task_kind or ctx.execution_id != execution_id
                    or ctx.goal_id != goal_id or ctx.revoked):
                raise ValueError('no matching active manipulation goal')
            if (self.assignment != (ctx.assignment_id, ctx.assignment_generation)
                    or now >= self.assignment_deadline):
                raise ValueError('assignment is not confirmed')
            if (not math.isfinite(deadline) or not now < deadline <= min(
                    self.assignment_deadline, now + 1.5)):
                raise ValueError('lease exceeds assignment deadline or 1.5 s limit')
            if ctx.lease_generation:
                if generation != ctx.lease_generation or now >= ctx.lease_deadline:
                    raise ValueError('expired or different lease generation')
            elif generation <= 0:
                raise ValueError('positive lease generation required')
            ctx.lease_generation = generation
            ctx.lease_deadline = deadline

    def revoke(self, execution_id, goal_id, generation):
        """현재 실행 권한과 일치하는 철회를 반영합니다."""
        # 철회 메시지를 보내기 전에 먼저 차단 표시를 합니다. 예전 Goal의 철회는 현재 작업에 적용하지 않습니다.
        with self.lock:
            ctx = self.context
            if ctx is None or (ctx.execution_id, ctx.goal_id, ctx.lease_generation) != (
                    execution_id, goal_id, generation):
                return False
            ctx.revoked = True
            return True

    def finish_goal(self, execution_id, goal_id=''):
        """현재 작업이 끝나면 연결된 실행 권한도 종료합니다."""
        with self.lock:
            if (self.context is not None and
                    (self.context.execution_id, self.context.goal_id) == (execution_id, goal_id)):
                self.context = None

    def decide(self, fields, state=None):
        """요청 형식이나 현재 정책 및 권한을 검사해 거절 사유를 반환합니다."""
        # 배정·활성 Goal·세대·기한/철회를 검사합니다. 빈 문자열만 통과이며 나머지는 거절 사유입니다.
        with self.lock:
            ctx = self.context
            self.expire_assignment()
            if self.assignment is None or self.clock() >= self.assignment_deadline:
                return 'ASSIGNMENT_STALE'
            if ctx is None or fields['execution_id'] != ctx.execution_id:
                return 'GOAL_MISMATCH'
            if self.assignment != (ctx.assignment_id, ctx.assignment_generation):
                return 'ASSIGNMENT_STALE'
            if fields['robot_id'] != self.robot_id:
                return 'AUTHORITY_INVALID'
            if ctx.revoked:
                return 'LEASE_REVOKED'
            if fields['authority'] == 2:
                # A3 일반 후퇴는 현재 RETRACT_ARM 작업만 허용합니다. C 조작 lease와 혼용하지 않습니다.
                if not (ctx.local_task_kind == fields['local_task_kind'] == 6
                        and not fields['goal_id'] and fields['lease_generation'] == 0
                        and fields['command_source'] == 2):
                    return 'AUTHORITY_INVALID'
            else:
                if ctx.local_task_kind or fields['goal_id'] != ctx.goal_id:
                    return 'GOAL_MISMATCH'
                if fields['lease_generation'] != ctx.lease_generation:
                    return 'LEASE_REVOKED'
                if not ctx.lease_generation or self.clock() >= ctx.lease_deadline:
                    return 'LEASE_EXPIRED'
            return ''

    def remaining(self, fields):
        """입력이나 권한의 유효기한까지 남은 시간을 계산합니다."""
        # 배정과 lease 중 먼저 끝나는 기한까지만 허용합니다. 호출자는 권한 잠금을 잡고 사용합니다.
        deadline = self.assignment_deadline
        if fields['authority'] == 1 and self.context is not None:
            deadline = min(deadline, self.context.lease_deadline)
        return max(0.0, deadline - self.clock())

    @contextmanager
    def command_guard(self):
        """판정과 내구성 있는 수락 저장이 철회와 섞이지 않도록 잠급니다."""
        # 검사와 수락 장부 커밋을 철회와 직렬화합니다. 단순한 상태 읽기용 잠금이 아닙니다.
        with self.lock:
            yield
