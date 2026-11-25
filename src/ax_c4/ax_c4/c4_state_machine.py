"""C4 ManipulateObject의 ROS 독립 실행 상태 머신.

이 파일은 rclpy를 직접 import하지 않는다. 그래서 ROS2가 설치되지 않은
환경에서도 Goal 검증, 중복 방지, lease 만료, 시도 예산, 취소 결과를
단위 테스트할 수 있다. 실제 ROS Action·Service 호출은
``manipulate_object_server.py``가 이 상태 머신의 결과를 사용한다.
"""

from dataclasses import dataclass, field
from enum import Enum
from hashlib import sha256
import math
import re
from typing import Iterable, Optional


class Phase(str, Enum):
    """C4가 외부 Feedback으로 알릴 수 있는 실행 단계."""

    CHECKING = "CHECKING"
    REQUESTING_LEASE = "REQUESTING_LEASE"
    PICKING = "PICKING"
    ATTACHING = "ATTACHING"
    VERIFYING_PICK = "VERIFYING_PICK"
    PLACING = "PLACING"
    RELEASING = "RELEASING"
    VERIFYING_PLACE = "VERIFYING_PLACE"
    CANCELING = "CANCELING"
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"
    CANCELED = "CANCELED"
    UNCERTAIN = "UNCERTAIN"


@dataclass(frozen=True)
class Budget:
    """Goal에 포함된 기존 ``BudgetGrant``의 검증에 필요한 값."""

    budget_grant_id: str
    kind: str
    granted_attempts: int
    allowed_execution_ids: tuple[str, ...]
    time_budget_s: float


@dataclass(frozen=True)
class GoalSnapshot:
    """ROS Action Goal에서 C4가 안전 판단에 사용하는 최소 복사본."""

    goal_id: str
    robot_id: str
    execution_id: str
    execution_operation: str
    assignment_id: str
    task_revision: int
    root_execution_id: str
    step_key: str
    derive_rule_version: str
    attempt_index: int
    task_id: str
    assignment_generation: int
    object_id: str
    operation: int
    peer_robot_id: str
    target_frame: str
    target_values_finite: bool
    target_stamp_valid: bool
    peer_verified: bool
    budgets: tuple[Budget, ...]
    has_candidate_preference: bool = False


@dataclass(frozen=True)
class LeaseGrant:
    """A3가 보내는 ``ExecutionLeaseGrant``의 안전 판단용 값."""

    request_id: str
    goal_id: str
    execution_id: str
    lease_generation: int
    granted: bool
    lease_s: float
    reason_code: str = ""


@dataclass(frozen=True)
class Attempt:
    """환불할 수 없는 물리 시도 1회의 기록."""

    attempt_index: int
    budget_grant_id: str
    execution_id: str
    kind: str
    state: str = "STARTED"
    started_at_s: float = 0.0
    started_at_ros_s: float = 0.0
    elapsed_s: float = 0.0
    failure_code: str = ""


@dataclass(frozen=True)
class ExecutionResult:
    """공통 ``ResultStatus``로 옮길 수 있는 상태 머신 결과."""

    outcome: str
    error_code: str = ""
    detail: str = ""
    hold_state: int = 0
    attempts: tuple[Attempt, ...] = ()
    need_reobservation_move: bool = False
    observation_reason: str = ""


@dataclass
class Execution:
    """현재 실행 하나의 내부 상태."""

    goal: GoalSnapshot
    fingerprint: str
    phase: Phase = Phase.CHECKING
    attempt_count: int = 0
    attempts: list[Attempt] = field(default_factory=list)
    lease: Optional[LeaseGrant] = None
    lease_started_at: float = 0.0
    cancel_requested: bool = False
    revoke_reason: str = ""
    budget_deadline_s: float = 0.0
    resync_attempts: int = 0


@dataclass(frozen=True)
class Validation:
    """Goal 검증 결과."""

    ok: bool
    reason_code: str = ""
    detail: str = ""


class C4StateMachine:
    """중복·lease·예산·취소를 결정하는 C4 핵심 상태 머신."""

    PICK = 1
    PLACE = 2
    UNKNOWN_HOLD = 0
    HELD = 1
    RELEASED = 2

    _GOAL_ID = re.compile(r"^[0-9a-f]{32}$")

    def __init__(self, robot_id: str, *, max_resync_attempts: int = 2):
        self.robot_id = robot_id
        self.max_resync_attempts = max_resync_attempts
        self.active: dict[str, Execution] = {}
        self.completed: dict[str, tuple[str, ExecutionResult]] = {}

    @staticmethod
    def fingerprint(goal: GoalSnapshot) -> str:
        """물리적으로 중요한 Goal 인자를 안정적인 digest로 만든다."""

        value = "|".join(
            (
                goal.robot_id,
                goal.execution_id,
                goal.task_id,
                str(goal.assignment_generation),
                goal.object_id,
                str(goal.operation),
                goal.peer_robot_id,
                goal.target_frame,
            )
        )
        return sha256(value.encode("utf-8")).hexdigest()

    def validate(self, goal: GoalSnapshot) -> Validation:
        """Goal의 형식·로봇·실행·예산 조건을 검사한다.

        실패하면 물리 명령을 만들지 않는다. 새로운 공통 reason code를
        추가하지 않고, 계약에 이미 있는 D-12/D-13 계열 문자열만 사용한다.
        """

        if goal.robot_id != self.robot_id:
            return Validation(False, "ROBOT_ID_MISMATCH", "Goal robot_id is not this C4 robot")
        if not goal.goal_id or not self._GOAL_ID.fullmatch(goal.goal_id):
            return Validation(False, "AUTHORITY_INVALID", "goal_id must be 32 lowercase hexadecimal characters")
        if not goal.execution_id or not goal.task_id or not goal.object_id:
            return Validation(False, "AUTHORITY_INVALID", "execution, task and object identifiers are required")
        if (not goal.execution_operation or not goal.assignment_id
                or not goal.root_execution_id or not goal.step_key
                or not goal.derive_rule_version):
            return Validation(False, "AUTHORITY_INVALID", "ExecutionRef fields are incomplete")
        expected_operation = {self.PICK: "PICK", self.PLACE: "PLACE"}.get(goal.operation)
        if goal.execution_operation != expected_operation:
            return Validation(False, "AUTHORITY_INVALID", "ExecutionRef.operation does not match the Goal operation")
        if goal.assignment_generation <= 0 or goal.task_revision < 0 or goal.attempt_index < 0:
            return Validation(False, "ASSIGNMENT_STALE", "ExecutionRef generation or revision is invalid")
        if goal.operation not in (self.PICK, self.PLACE):
            return Validation(False, "AUTHORITY_INVALID", "operation must be PICK or PLACE")
        if not goal.target_frame:
            return Validation(False, "PRECONDITION_FAILED", "target pose frame is required")
        if not goal.target_values_finite:
            return Validation(False, "PRECONDITION_FAILED", "target pose contains NaN or infinity")
        if not goal.target_stamp_valid:
            return Validation(False, "TF_STALE", "target pose timestamp is invalid")
        if goal.peer_robot_id == self.robot_id:
            return Validation(False, "PRECONDITION_FAILED", "peer robot cannot equal the executing robot")
        if goal.peer_robot_id and not goal.peer_verified:
            return Validation(False, "PRECONDITION_FAILED", "peer assignment was not verified by D2 task state")

        # 현재 공통 계약에서 물리 시도 예산의 이름은 GRASP_ATTEMPT이다.
        # C4는 새로운 예산 종류를 만들어 정책을 우회하지 않고, 이 목록에
        # execution_id가 실제로 들어 있는 경우에만 물리 시도를 시작한다.
        physical_budgets = [b for b in goal.budgets if b.kind == "GRASP_ATTEMPT"]
        if not physical_budgets:
            return Validation(False, "BUDGET_EXHAUSTED", "GRASP_ATTEMPT budget is required")
        for budget in physical_budgets:
            if not budget.budget_grant_id or budget.granted_attempts <= 0:
                return Validation(False, "BUDGET_EXHAUSTED", "budget grant is empty or exhausted")
            if goal.execution_id not in budget.allowed_execution_ids:
                return Validation(False, "PERMISSION_DENIED", "execution_id is not allowed by budget")
            if not math.isfinite(budget.time_budget_s) or budget.time_budget_s <= 0:
                return Validation(False, "BUDGET_EXHAUSTED", "budget time is invalid")
        return Validation(True)

    @staticmethod
    def validate_attach_evidence(
        *,
        robot_id: str,
        object_id: str,
        message_robot_id: str,
        message_object_id: str,
        attached: bool,
        want_attached: bool,
        meta_valid: bool,
        revision: int,
        source_session_id: str,
        stamp_s: float,
        now_s: float,
        ttl_s: float,
    ) -> Validation:
        """A2 원시 부착 신호가 결과 확정에 충분한지 검사한다.

        ``accepted=true``는 명령 접수일 뿐이므로, 이 함수가 통과하기 전에는
        PICK/PLACE 성공을 발행하지 않는다.
        """

        if message_robot_id != robot_id or message_object_id != object_id:
            return Validation(False, "ATTACH_STATE_MISMATCH", "robot or object does not match")
        if not meta_valid or revision <= 0 or not source_session_id:
            return Validation(False, "ATTACH_STATE_INVALID", "AttachState metadata is invalid")
        if (not math.isfinite(stamp_s) or stamp_s <= 0 or
                not math.isfinite(now_s) or now_s < stamp_s or now_s - stamp_s > ttl_s):
            return Validation(False, "ATTACH_STATE_STALE", "AttachState timestamp is stale")
        if attached != want_attached:
            return Validation(False, "ATTACH_STATE_MISMATCH", "AttachState does not prove the requested result")
        return Validation(True)

    def accept(self, goal: GoalSnapshot) -> tuple[Optional[Execution], Optional[ExecutionResult], Validation]:
        """새 Goal을 등록하거나 멱등 결과를 반환한다.

        같은 execution_id가 실행 중이면 중복 실행을 만들지 않는다.
        이미 끝난 동일 Goal이면 저장된 결과를 재사용하고, 인자가 다르면
        ``DUPLICATE_REQUEST``로 거절한다.
        """

        validation = self.validate(goal)
        if not validation.ok:
            return None, None, validation
        fingerprint = self.fingerprint(goal)
        active = self.active.get(goal.execution_id)
        if active is not None:
            if active.fingerprint == fingerprint:
                return None, ExecutionResult("FAILED", "DUPLICATE_REQUEST", "execution is already active"), validation
            return None, ExecutionResult("FAILED", "DUPLICATE_REQUEST", "execution_id has different arguments"), validation
        completed = self.completed.get(goal.execution_id)
        if completed is not None:
            old_fingerprint, result = completed
            if old_fingerprint == fingerprint:
                return None, result, validation
            return None, ExecutionResult("FAILED", "DUPLICATE_REQUEST", "completed execution has different arguments"), validation
        execution = Execution(goal=goal, fingerprint=fingerprint)
        self.active[goal.execution_id] = execution
        return execution, None, validation

    def set_lease(self, execution_id: str, grant: LeaseGrant, elapsed_s: float) -> Validation:
        """A3 grant가 현재 Goal에 속하고 아직 유효한지 확인한다."""

        execution = self.active.get(execution_id)
        if execution is None:
            return Validation(False, "GOAL_MISMATCH", "execution is not active")
        if execution.cancel_requested:
            return Validation(False, "CANCELED", "cancel was already requested")
        if not grant.granted:
            return Validation(False, grant.reason_code or "LEASE_EXPIRED", "A3 did not grant the lease")
        if grant.goal_id != execution.goal.goal_id or grant.execution_id != execution.goal.execution_id:
            return Validation(False, "GOAL_MISMATCH", "lease does not belong to this Goal")
        if grant.lease_generation <= 0 or not math.isfinite(grant.lease_s) or grant.lease_s <= elapsed_s:
            return Validation(False, "LEASE_EXPIRED", "lease is empty or already expired")
        execution.lease = grant
        execution.lease_started_at = elapsed_s
        return Validation(True)

    def renew_lease(self, execution_id: str, grant: LeaseGrant, elapsed_s: float) -> Validation:
        """현재 세대의 lease만 갱신한다.

        RENEW 응답이 이전 세대이거나 다른 Goal에 속하면 물리 명령을 계속하지 않는다.
        응답이 늦어 이미 만료된 경우도 보수적으로 거부한다.
        """

        execution = self.active.get(execution_id)
        if execution is None or execution.lease is None:
            return Validation(False, "GOAL_MISMATCH", "execution has no active lease")
        if grant.goal_id != execution.goal.goal_id or grant.execution_id != execution_id:
            return Validation(False, "GOAL_MISMATCH", "renewed lease does not belong to this Goal")
        if grant.lease_generation != execution.lease.lease_generation:
            return Validation(False, "LEASE_REVOKED", "lease generation changed unexpectedly")
        if (not grant.granted or not math.isfinite(grant.lease_s)
                or grant.lease_s <= 0):
            return Validation(False, grant.reason_code or "LEASE_EXPIRED", "renewal was not granted")
        execution.lease = grant
        execution.lease_started_at = elapsed_s
        return Validation(True)

    def check_runtime(self, execution_id: str, now_s: float) -> Validation:
        """물리 명령 직전에 lease와 실행 시간 예산을 다시 확인한다."""

        execution = self.active.get(execution_id)
        if execution is None:
            return Validation(False, "GOAL_MISMATCH", "execution is not active")
        if execution.cancel_requested:
            return Validation(False, "CANCELED", "cancel was requested")
        if execution.revoke_reason:
            return Validation(False, execution.revoke_reason, "lease was revoked")
        if execution.lease is None or now_s - execution.lease_started_at >= execution.lease.lease_s:
            return Validation(False, "LEASE_EXPIRED", "lease expired before the command")
        if execution.budget_deadline_s and now_s >= execution.budget_deadline_s:
            return Validation(False, "TIMED_OUT", "physical attempt exceeded its time budget")
        return Validation(True)

    def start_attempt(self, execution_id: str, now_s: float | None = None) -> Validation:
        """예산을 확인한 뒤 환불 불가능한 물리 시도를 시작한다."""

        execution = self.active.get(execution_id)
        if execution is None:
            return Validation(False, "GOAL_MISMATCH", "execution is not active")
        if execution.lease is None:
            return Validation(False, "LEASE_EXPIRED", "a valid lease is required")
        if now_s is not None:
            lease_check = self.check_runtime(execution_id, now_s)
            if not lease_check.ok:
                return lease_check
        if execution.revoke_reason:
            return Validation(False, execution.revoke_reason, "lease was revoked")
        if execution.cancel_requested:
            return Validation(False, "CANCELED", "cancel was requested before attempt")
        budget = next(
            (b for b in execution.goal.budgets
             if b.kind == "GRASP_ATTEMPT"
             and execution.goal.execution_id in b.allowed_execution_ids
             and execution.attempt_count < b.granted_attempts),
            None,
        )
        if budget is None:
            return Validation(False, "BUDGET_EXHAUSTED", "no physical attempt remains")
        attempt = Attempt(
            attempt_index=execution.attempt_count,
            budget_grant_id=budget.budget_grant_id,
            execution_id=execution.goal.execution_id,
            kind=budget.kind,
            started_at_s=now_s if now_s is not None else execution.lease_started_at,
        )
        execution.attempts.append(attempt)
        execution.attempt_count += 1
        execution.budget_deadline_s = ((now_s if now_s is not None else execution.lease_started_at)
                                       + budget.time_budget_s)
        return Validation(True)

    def finish_attempt(self, execution_id: str, state: str, now_s: float,
                       failure_code: str = "") -> Validation:
        """현재 물리 시도의 종료 상태와 사용 시간을 기록한다."""

        execution = self.active.get(execution_id)
        if execution is None or not execution.attempts:
            return Validation(False, "GOAL_MISMATCH", "no active attempt exists")
        attempt = execution.attempts[-1]
        if attempt.state != "STARTED":
            return Validation(False, "GOAL_MISMATCH", "attempt is already finished")
        started = attempt.started_at_s
        execution.attempts[-1] = Attempt(
            attempt_index=attempt.attempt_index,
            budget_grant_id=attempt.budget_grant_id,
            execution_id=attempt.execution_id,
            kind=attempt.kind,
            state=state,
            started_at_s=started,
            started_at_ros_s=attempt.started_at_ros_s,
            elapsed_s=max(0.0, now_s - started),
            failure_code=failure_code,
        )
        return Validation(True)

    def request_cancel(self, execution_id: str) -> Validation:
        """취소 요청을 기록한다. 실행 중인 하위 명령은 호출자가 중단해야 한다."""

        execution = self.active.get(execution_id)
        if execution is None:
            return Validation(False, "GOAL_MISMATCH", "execution is not active")
        execution.cancel_requested = True
        execution.phase = Phase.CANCELING
        return Validation(True)

    def revoke(self, execution_id: str, goal_id: str, generation: int, reason: str) -> Validation:
        """A3 lease revoke를 현재 실행에만 적용한다."""

        execution = self.active.get(execution_id)
        if execution is None:
            return Validation(False, "GOAL_MISMATCH", "execution is not active")
        if (execution.goal.goal_id != goal_id or execution.lease is None
                or execution.lease.lease_generation != generation):
            return Validation(False, "GOAL_MISMATCH", "revoke does not match current lease")
        execution.revoke_reason = reason or "LEASE_REVOKED"
        return Validation(True)

    def finish(self, execution_id: str, result: ExecutionResult) -> ExecutionResult:
        """실행을 종료하고 같은 execution_id 재전송에 사용할 결과를 보존한다."""

        execution = self.active.pop(execution_id, None)
        if execution is None:
            return result
        final = ExecutionResult(
            outcome=result.outcome,
            error_code=result.error_code,
            detail=result.detail,
            hold_state=result.hold_state,
            attempts=tuple(execution.attempts),
            need_reobservation_move=result.need_reobservation_move,
            observation_reason=result.observation_reason,
        )
        self.completed[execution_id] = (execution.fingerprint, final)
        return final

