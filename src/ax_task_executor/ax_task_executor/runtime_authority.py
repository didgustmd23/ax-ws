"""
Trusted A3 lifecycle state for command authorization, independent of ROS.

Only the A3 dispatcher may register goals and leases. CheckCommandLease requests
never create authority. Restart deliberately discards all live permissions.
"""
from contextlib import contextmanager
from dataclasses import dataclass
import math
import sqlite3
import threading
import time
import uuid
import weakref


def boottime():
    """Use a clock that includes host suspend time."""
    return time.clock_gettime(time.CLOCK_BOOTTIME)


@dataclass
class ActiveCommandContext:
    """A goal actually dispatched by A3, bound to its assignment."""

    execution_id: str
    assignment_id: str
    assignment_generation: int
    goal_id: str
    local_task_kind: int = 0
    lease_generation: int = 0
    lease_deadline: float = 0.0
    revoked: bool = False


class RuntimeAuthority:
    """Serialize permission transitions with command acceptance records."""

    policy = 'operational'
    marks = ('', 'accept-')
    CHECK_LIFETIME_S = 2.0
    MAX_PENDING_CHECKS = 64

    def __init__(self, robot_id, clock=boottime):
        self.robot_id = robot_id
        self.clock = clock
        self.lock = threading.RLock()
        self.session_id = uuid.uuid4().hex
        self.assignment = None
        self.assignment_deadline = 0.0
        self.context = None
        self.pending = {}
        # Keep lifetime duplicate protection on temporary disk, not an
        # ever-growing Python dictionary. SQLite removes this DB on close;
        # a new process gets a new A session and cannot use its old replies.
        self._check_history = sqlite3.connect('', check_same_thread=False, isolation_level=None)
        self._check_history.execute('PRAGMA cache_size=-256')
        self._check_history.execute("""CREATE TABLE sent_checks (
            request_id TEXT PRIMARY KEY, assignment_id TEXT NOT NULL,
            generation INTEGER NOT NULL, sent_at REAL NOT NULL)""")
        self._close_history = weakref.finalize(self, self._check_history.close)
        self.d_session_id = None
        self.sequence = -1
        self.requested_assignment = None

    def begin_assignment_check(self, request_id, assignment_id, generation):
        """Register tA before publishing an A3 -> D4 check (no retransmit reset)."""
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
            # Only the current pending assignment may subsequently be confirmed.
            if self.assignment is not None and self.assignment != value:
                self._invalidate()
            if self.requested_assignment != value:
                self.pending.clear()
            if len(self.pending) >= self.MAX_PENDING_CHECKS:
                raise ValueError('too many outstanding assignment checks')
            # Record before publishing. A completed/expired request ID is never
            # re-enrolled, even after it has been removed from pending memory.
            self._check_history.execute('INSERT INTO sent_checks VALUES (?, ?, ?, ?)',
                                        (request_id, assignment_id, generation, now))
            self.requested_assignment = value
            self.pending[request_id] = (*value, now)

    def _prune_checks(self, now):
        """Keep only responses that can still grant authority in memory."""
        expired = [key for key, value in self.pending.items()
                   if now >= value[2] + self.CHECK_LIFETIME_S]
        for key in expired:
            del self.pending[key]

    def close(self):
        """Invalidate permissions and delete the temporary duplicate history."""
        with self.lock:
            self.invalidate_assignment()
            self._close_history()

    def confirm_assignment(self, reply):
        """Correlate D4 response; authority deadline is tA + H, never receive + H."""
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
            # A positive answer must match the requested generation. A fresh,
            # correlated negative answer may report D4's NEW current generation:
            # discard the old permission instead of ignoring that rejection.
            if reply.valid and reply.assignment_generation != generation:
                return False
            # Consume once: duplicates cannot extend a permission.
            del self.pending[reply.request_id]
            if self.d_session_id not in (None, reply.d_session_id):
                self._invalidate()
                self.pending.clear()
            self.d_session_id = reply.d_session_id
            self.sequence = reply.sequence
            duration = float(reply.validity_s)
            if (not reply.valid or not math.isfinite(duration) or duration <= 0
                    or sent_at + min(duration, self.CHECK_LIFETIME_S) <= self.clock()):
                self.invalidate_assignment()
                return False
            # A delayed heartbeat must not resurrect a goal whose assignment
            # expired while ROS callbacks were paused or the host suspended.
            self.expire_assignment()
            # Late responses from a prior assignment must not restore it.
            if self.assignment is not None and self.assignment != (assignment_id, generation):
                self._invalidate()
            self.assignment = (assignment_id, generation)
            self.assignment_deadline = sent_at + min(duration, self.CHECK_LIFETIME_S)
            return self.assignment_deadline > self.clock()

    def _invalidate(self):
        self.assignment = None
        self.assignment_deadline = 0.0
        if self.context is not None:
            self.context.revoked = True

    def invalidate_assignment(self):
        """Invalidate on reassignment, hold, cancellation or lost authority."""
        with self.lock:
            self._invalidate()
            self.pending.clear()
            self.requested_assignment = None

    def expire_assignment(self):
        """Latch expiry on the active goal even if a new confirmation follows."""
        with self.lock:
            self._prune_checks(self.clock())
            if self.assignment is not None and self.clock() >= self.assignment_deadline:
                self._invalidate()

    def activate_goal(self, execution, goal_id='', local_task_kind=0):
        """
        Register the dispatcher's recorded and accepted real goal.

        C goals use the ManipulateObject UUID. Local authority is RETRACT_ARM=6
        only and deliberately has no C goal UUID. Never call for a mock result.
        """
        import re

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
        """
        Install an A3-issued lease; deadline uses CLOCK_BOOTTIME.

        Generation must be persisted by the issuer before this call. Resync of
        an existing generation is refused here: the issuer must first inspect
        durable acceptance records under the contract's resync protocol.
        """
        with self.lock:
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
        """Mark revoked before publishing revoke/cancel; ignore unrelated goals."""
        with self.lock:
            ctx = self.context
            if ctx is None or (ctx.execution_id, ctx.goal_id, ctx.lease_generation) != (
                    execution_id, goal_id, generation):
                return False
            ctx.revoked = True
            return True

    def finish_goal(self, execution_id, goal_id=''):
        """End the matching goal; late completion cannot erase a new goal."""
        with self.lock:
            if (self.context is not None and
                    (self.context.execution_id, self.context.goal_id) == (execution_id, goal_id)):
                self.context = None

    def decide(self, fields, state=None):
        """Return contract error or empty string; caller holds command_guard."""
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
        """Never advertise a duration beyond the actual permission deadline."""
        deadline = self.assignment_deadline
        if fields['authority'] == 1 and self.context is not None:
            deadline = min(deadline, self.context.lease_deadline)
        return max(0.0, deadline - self.clock())

    @contextmanager
    def command_guard(self):
        """Keep decision + durable acceptance atomic relative to revocation."""
        with self.lock:
            yield
