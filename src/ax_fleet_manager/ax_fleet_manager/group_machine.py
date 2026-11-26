"""D2 W3 순수 그룹 단계 엔진: apply_event(state, event, now) -> Decision.

- 입력 상태를 바꾸지 않고 복사본으로 다음 상태를 만든다. 시간·관측·결과는 모두 사건으로 주입한다.
- 효과는 명령 의도뿐이다. 실제 전송·원장 기록은 runtime·포트가 하고, 원장 기록 전에는 보내지 않는다.
- 단계 전이는 하위 Action 결과와 그 결과 이후에 받은 최신 관측을 함께 요구한다. SUCCEEDED 하나로
  적재·해제·후퇴·정지·배치를 채우지 않는다.
- 실패·취소·시간초과·미확정은 실행 억제(inhibit)로 남기며 잠금 해제·재시도·attempt 증가를 하지 않는다.
"""
from dataclasses import dataclass, replace

from . import group_model as gm
from .group_events import (
    AssignRoles, CancelAcknowledged, CancelRequested, CommandIntent, Decision, DispatchFailed,
    DispatchReserved, DispatchSent, FailTerminate, GoalResponse, HoldApplied,
    HoldReleaseRequested, ObservationReceived, StartTask, StepResult, StoreFault,
    TaskStatusSeen, Tick,
)


@dataclass(frozen=True)
class EngineLimits:
    """엔진 상한. 결과 대기 시간은 개발 가정값이며 운영 기준이 아니다."""

    result_timeout_s: float = 60.0
    max_hold_ids: int = 16
    max_retired_sessions: int = 8


INFLIGHT = ('RESERVED', 'SENT', 'ACCEPTED')
OUTCOME_STATUS = {
    gm.OUTCOME_SUCCEEDED: 'SUCCEEDED',
    gm.OUTCOME_FAILED: 'FAILED',
    gm.OUTCOME_CANCELED: 'CANCELED',
    gm.OUTCOME_TIMED_OUT: 'TIMED_OUT',
    gm.OUTCOME_UNCERTAIN: 'UNCERTAIN',
    gm.OUTCOME_UNSPECIFIED: 'UNCERTAIN',
}
OBJECT_KINDS = ('HOLD', 'LOAD', 'PLACEMENT')

SRC_NAV = ('src_nav_manipulator', 'src_nav_transporter')
SRC_LOCK = ('src_lock_manipulator', 'src_lock_transporter')
SRC_UNLOCK = ('src_unlock_manipulator', 'src_unlock_transporter')
DST_NAV = ('dst_nav_manipulator', 'dst_nav_transporter')
DST_LOCK = ('dst_lock_manipulator', 'dst_lock_transporter')
DST_UNLOCK = ('dst_unlock_manipulator', 'dst_unlock_transporter')


class _Context:
    """사건 하나를 처리하는 동안의 작업 공간(다음 상태·효과·사유)."""

    # 다음 상태 복사본과 시각을 보관한다
    def __init__(self, state, now, limits):
        self.s = state
        self.now = now
        self.limits = limits
        self.effects = []
        self.reasons = []

    # 판정 사유 코드 추가
    def reason(self, code):
        if code not in self.reasons:
            self.reasons.append(code)

    # 실행 억제 사유 추가(지속). 해제 사건은 W3에 없다
    def inhibit(self, code):
        if code not in self.s.inhibit:
            self.s.inhibit.append(code)
        self.reason(code)


# 원장·발행에 남는 상태만 비교용으로 뽑는다(관측 버퍼 제외)
def persistent_view(state):
    return (state.assignment, state.task_state, state.transfer_state, state.transfer_id,
            {k: vars(v) for k, v in state.steps.items()}, tuple(state.inhibit),
            tuple(state.hold_ids), state.cancel_requested,
            state.cancel_requested_boottime_s, state.failure_event_id,
            state.last_sim_time_s, object_view(state))


# 그룹 물체 상태(재고 항목으로 내보내는 필드)
def object_view(state):
    return (state.object_state, state.object_location_id, state.object_carrier_robot_id,
            state.object_assignment_generation, state.object_isolated, state.physical_started)


# 근거 출처 수용 판정(모드·종류·live 허용 종류). 혼합 모드는 종류마다 출처 하나만 받는다
def _accepts(s, kind, provenance):
    return gm.provenance_acceptable(s.mode, provenance, kind, s.live_kinds)


# 단계 결과·응답·상태 사건의 근거 종류: 주행 단계는 NAVIGATION, 그 밖(A3 로컬 작업)은 LOCAL_TASK
def _step_kind(s, step_key):
    if gm.step_by_key(s.spec, step_key).operation == 'NAVIGATE_TO':
        return gm.LIVE_KIND_NAVIGATION

    return gm.EVIDENCE_LOCAL_TASK


# 판정에 쓴 근거의 (종류, 출처)를 작업 요약에 누적한다(중복 없이 정렬)
def _note_evidence(s, kind, provenance):
    pair = (kind, provenance)
    if pair not in s.evidence_used:
        s.evidence_used = tuple(sorted(s.evidence_used + (pair,)))


# 단계 기록을 goal UUID로 찾는다
def _step_by_uuid(s, goal_uuid):
    if not goal_uuid:
        return None

    for run in s.steps.values():
        if run.goal_uuid == goal_uuid:
            return run

    return None


# 단계 기록을 execution_id로 찾는다
def _step_by_execution(s, execution_id):
    for run in s.steps.values():
        if run.execution_id and run.execution_id == execution_id:
            return run

    return None


# 단계가 맡은 로봇
def _robot_of(s, step_key):
    return s.assignment.robot_for(gm.step_by_key(s.spec, step_key).robot_role)


# 단계들이 모두 성공했는지
def _all_succeeded(s, keys):
    return all(s.steps[k].status == 'SUCCEEDED' for k in keys)


# 단계들이 끝난 시각 중 가장 늦은 값(근거 관측은 이 시각 이후에 받은 것만 쓴다)
def _anchor(s, keys):
    return max(s.steps[k].finished_boottime_s for k in keys)


# 최신·유효·출처 허용·물체 일치·anchor 이후 수신된 관측을 돌려준다(없으면 None).
# strict=True면 anchor와 같은 시각 수신도 거부한다(취소 완료 정지 근거)
def _fresh(ctx, kind, robot_id, anchor, object_id=None, strict=False):
    obs = ctx.s.observations.get((kind, robot_id))
    if obs is None or not obs.valid:
        return None

    if not _accepts(ctx.s, kind, obs.provenance):
        return None

    expiry = gm.OBSERVATION_EXPIRY_S[kind]
    age = ctx.now.sim_time_s - obs.stamp_sim_s
    silent = ctx.now.boottime_s - obs.received_boottime_s
    if not (0.0 <= age <= expiry and 0.0 <= silent <= expiry):
        return None

    if obs.received_boottime_s < anchor or (strict and obs.received_boottime_s <= anchor):
        return None

    if object_id is not None and obs.object_id != object_id:
        return None

    return obs


# 관측 값이 기대값인지(최신·유효 관측이 있을 때만 True)
def _fresh_is(ctx, kind, robot_id, value, anchor, object_id=None, strict=False):
    obs = _fresh(ctx, kind, robot_id, anchor, object_id, strict)
    if obs is None or obs.value != value:
        return False

    _note_evidence(ctx.s, kind, obs.provenance)

    return True


# 두 로봇 실제 정지(ActuationState.STOPPED). 허가 필드는 보지 않는다
def _both_stopped(ctx, anchor, strict=False):
    return all(_fresh_is(ctx, 'ACTUATION', robot, gm.ACTUATION_STOPPED, anchor, strict=strict)
               for robot in ctx.s.assignment.robots())


# 팔 후퇴·흡착 해제·적재·정지 근거(출발지 해제 조건)
def _source_release_ready(ctx, anchor):
    s = ctx.s
    m, t = s.assignment.robots()
    obj = s.spec.object_id

    if (_fresh_is(ctx, 'HOLD', m, gm.HOLD_HELD, anchor, obj)
            and _fresh_is(ctx, 'LOAD', t, gm.LOAD_STOWED, anchor, obj)):
        ctx.inhibit('EVIDENCE_CONFLICT')
        return False

    return (_fresh_is(ctx, 'HOLD', m, gm.HOLD_RELEASED, anchor, obj)
            and _fresh_is(ctx, 'LOAD', t, gm.LOAD_STOWED, anchor, obj)
            and _fresh_is(ctx, 'ARM_RETRACTED', m, 1, anchor)
            and _both_stopped(ctx, anchor))


# 목적지 배치·해제·후퇴·EMPTY·정지 근거(그룹 완료 조건)
def _destination_release_ready(ctx, anchor):
    s = ctx.s
    m, t = s.assignment.robots()
    obj = s.spec.object_id

    return (_fresh_is(ctx, 'PLACEMENT', m, 1, anchor, obj)
            and _fresh_is(ctx, 'HOLD', m, gm.HOLD_RELEASED, anchor, obj)
            and _fresh_is(ctx, 'LOAD', t, gm.LOAD_EMPTY, anchor, obj)
            and _fresh_is(ctx, 'ARM_RETRACTED', m, 1, anchor)
            and _both_stopped(ctx, anchor))


# 인계 xfer의 잠금 요청을 낼 수 있는 이전 상태인지. 출발은 NONE, 목적지는 출발 xfer의 UNLOCKED.
# 같은 xfer의 UNKNOWN(한쪽 잠금 요청 예약됨)도 같은 잠금 단계로 본다
def _lock_open(s, transfer_id):
    if s.transfer_state == gm.TRANSFER_UNKNOWN:
        return s.transfer_id == transfer_id

    if transfer_id == s.spec.source_transfer_id:
        return s.transfer_state == gm.TRANSFER_NONE

    return (s.transfer_state == gm.TRANSFER_UNLOCKED
            and s.transfer_id == s.spec.source_transfer_id)


# LOCK_HANDOFF goal UUID를 원장에 예약한 시점의 인계 상태 전환. 두 결과·정지 확인 전에는
# TRANSFER_UNKNOWN(계약상 두 로봇 주행 금지)과 그 xfer를 낸다. 예약 전 철회된 의도는 여기 오지 않아
# 이전 상태(NONE·출발 xfer의 UNLOCKED)가 남는다. 예약 뒤에는 결과와 무관하게 해제하지 않는다
def mark_lock_reserved(state, step_key):
    plan_step = gm.step_by_key(state.spec, step_key)
    if plan_step.operation != 'LOCK_HANDOFF':
        return

    transfer_id = gm.transfer_id_for(state.spec, plan_step)
    if state.transfer_state == gm.TRANSFER_UNKNOWN or not _lock_open(state, transfer_id):
        return

    state.transfer_state = gm.TRANSFER_UNKNOWN
    state.transfer_id = transfer_id


# 단계별 추가 조건(선행 단계 성공 외). True면 의도를 낼 수 있다
def _gate(ctx, plan_step):
    s = ctx.s
    key = plan_step.step_key
    deps = plan_step.depends_on
    anchor = _anchor(s, deps) if deps else -1.0
    _, t = s.assignment.robots()
    obj = s.spec.object_id

    if key in SRC_NAV:
        return s.transfer_state == gm.TRANSFER_NONE

    if key in SRC_LOCK:
        return _lock_open(s, s.spec.source_transfer_id) and _both_stopped(ctx, anchor)

    if key == 'load_pick_and_stow':
        return s.transfer_state == gm.TRANSFER_LOCKED and _both_stopped(ctx, anchor)

    if key == 'load_confirm':
        return s.transfer_state == gm.TRANSFER_IN_PROGRESS

    if key == 'load_retract':
        return (s.steps['load_confirm'].load_state == gm.LOAD_STOWED
                and _fresh_is(ctx, 'LOAD', t, gm.LOAD_STOWED, anchor, obj))

    if key in SRC_UNLOCK:
        return _source_release_ready(ctx, anchor)

    if key in DST_NAV:
        # 출발지 인계가 UNLOCKED로 확인된 뒤에만, 운송은 적재 유지 근거도 함께
        if s.transfer_state != gm.TRANSFER_UNLOCKED or s.transfer_id != s.spec.source_transfer_id:
            return False
        if plan_step.robot_role == gm.TRANSPORTER:
            return _fresh_is(ctx, 'LOAD', t, gm.LOAD_STOWED, anchor, obj)
        return True

    if key in DST_LOCK:
        return _lock_open(s, s.spec.destination_transfer_id) and _both_stopped(ctx, anchor)

    if key == 'unload_place':
        return (s.transfer_state == gm.TRANSFER_LOCKED
                and s.transfer_id == s.spec.destination_transfer_id
                and _both_stopped(ctx, anchor)
                and _fresh_is(ctx, 'LOAD', t, gm.LOAD_STOWED, anchor, obj))

    if key in ('unload_retract', 'unload_confirm_empty'):
        return s.transfer_state == gm.TRANSFER_IN_PROGRESS

    if key in DST_UNLOCK:
        return (s.steps['unload_confirm_empty'].load_state == gm.LOAD_EMPTY
                and _destination_release_ready(ctx, anchor))

    return False


# 새 의도를 낼 수 없는 이유(없으면 None)
def _blocked(ctx):
    s = ctx.s
    if s.task_state != gm.TASK_RUNNING:
        return 'TASK_NOT_RUNNING'
    if s.inhibit:
        return 'INHIBITED'
    if s.hold_ids:
        return 'HOLD_ACTIVE'
    if s.cancel_requested:
        return 'CANCEL_REQUESTED'
    if not ctx.now.time_valid:
        return 'TIME_INVALID'
    return None


# 단계 하나의 명령 의도를 만들고 단계 기록을 INTENDED로 바꾼다
def _emit(ctx, plan_step):
    s = ctx.s
    run = s.steps[plan_step.step_key]
    execution, args = gm.build_execution(s.spec, s.assignment, plan_step, run.attempt_index)
    kind = 'NAVIGATION' if plan_step.operation == 'NAVIGATE_TO' else 'LOCAL_TASK'

    run.status = 'INTENDED'
    run.execution_id = execution['execution_id']
    run.transfer_id = execution['transfer_id']
    run.assignment_generation = execution['assignment_generation']
    ctx.effects.append(CommandIntent(
        kind=kind, task_id=s.spec.task_id, step_key=plan_step.step_key,
        operation=plan_step.operation, robot_id=args['robot_id'],
        peer_robot_id=args['peer_robot_id'], execution=execution, args=args,
        args_digest=gm.args_digest({'execution': execution, 'args': args})))

    # 적재·하역 의도를 원장에 남기는 순간부터 인계는 진행 중(전송 전에 더 강한 잠금)
    if plan_step.operation in ('PICK_AND_STOW', 'UNLOAD_AND_PLACE'):
        s.transfer_state = gm.TRANSFER_IN_PROGRESS


# 결과·관측으로 정해지는 인계·작업 상태 전이
def _transitions(ctx):
    s = ctx.s
    if s.assignment is None or s.task_state != gm.TASK_RUNNING:
        return

    # 잠금 확정: 같은 xfer의 UNKNOWN(두 잠금 요청 예약)에서만, 두 결과 성공과 둘째 결과 뒤 두 로봇 정지
    if (s.transfer_state == gm.TRANSFER_UNKNOWN
            and s.transfer_id == s.spec.source_transfer_id
            and _all_succeeded(s, SRC_LOCK)
            and _both_stopped(ctx, _anchor(s, SRC_LOCK))):
        s.transfer_state = gm.TRANSFER_LOCKED

    if (s.transfer_state == gm.TRANSFER_IN_PROGRESS
            and s.transfer_id == s.spec.source_transfer_id
            and _all_succeeded(s, SRC_UNLOCK)
            and _source_release_ready(ctx, _anchor(s, SRC_UNLOCK))):
        s.transfer_state = gm.TRANSFER_UNLOCKED

    if (s.transfer_state == gm.TRANSFER_UNKNOWN
            and s.transfer_id == s.spec.destination_transfer_id
            and _all_succeeded(s, DST_LOCK)
            and _both_stopped(ctx, _anchor(s, DST_LOCK))):
        s.transfer_state = gm.TRANSFER_LOCKED

    if (s.transfer_state == gm.TRANSFER_IN_PROGRESS
            and s.transfer_id == s.spec.destination_transfer_id
            and _all_succeeded(s, DST_UNLOCK)
            and _destination_release_ready(ctx, _anchor(s, DST_UNLOCK))):
        s.transfer_state = gm.TRANSFER_UNLOCKED
        s.task_state = gm.TASK_COMPLETED


# 취소 완료 판정. 하위 실행이 모두 확실히 끝나야 하고, 한 번이라도 예약·전송한 작업은 취소 요청과
# 마지막 하위 종료 이후에 받은 두 로봇 최신 STOPPED가 있어야 CANCELED. 전송이 전혀 없던 작업은
# 물리 근거를 요구하지 않는다. inhibit·hold·인계 상태는 바꾸지 않는다
def _cancel_completion(ctx):
    s = ctx.s
    if not s.cancel_requested or s.task_state in gm.TERMINAL_TASK_STATES:
        return

    unresolved = INFLIGHT + ('INTENDED', 'UNCERTAIN')
    if any(run.status in unresolved for run in s.steps.values()):
        ctx.reason('CANCEL_WAITING_FOR_TERMINAL_RESULTS')
        return

    dispatched = [run for run in s.steps.values() if run.goal_uuid]
    if dispatched:
        anchor = max([s.cancel_requested_boottime_s]
                     + [run.finished_boottime_s for run in dispatched])
        if not _both_stopped(ctx, anchor, strict=True):
            ctx.reason('CANCEL_WAITING_FOR_STOP')
            return

    s.task_state = gm.TASK_CANCELED


# 진행 중 단계의 결과 대기 시간초과 → 미확정(재전송 없음)
def _timeouts(ctx):
    for run in ctx.s.steps.values():
        if run.status in INFLIGHT and run.sent_boottime_s >= 0.0:
            if ctx.now.boottime_s - run.sent_boottime_s > ctx.limits.result_timeout_s:
                run.status = 'UNCERTAIN'
                ctx.inhibit('RESULT_TIMEOUT')


# 운송 중 적재 이상 감시(출발지 적재 확인 뒤 ~ 목적지 하역 결과 전)
def _load_watch(ctx, obs):
    s = ctx.s
    if s.assignment is None or obs.kind != 'LOAD' or not obs.valid:
        return

    if obs.robot_id != s.assignment.transporter_robot_id:
        return

    loaded = s.steps['load_confirm'].status == 'SUCCEEDED'
    unloaded = s.steps['unload_place'].status not in ('PENDING',)
    if not loaded or unloaded:
        return

    if obs.value == gm.LOAD_LOST:
        ctx.inhibit('LOAD_LOST')
    elif obs.value == gm.LOAD_UNKNOWN:
        ctx.inhibit('LOAD_UNKNOWN')
    elif obs.value == gm.LOAD_EMPTY:
        ctx.inhibit('EVIDENCE_CONFLICT')


# 관측 수용 규칙(STATE 소비 규칙: 세션·revision·출처·물체·로봇)
def _on_observation(ctx, obs):
    s = ctx.s
    if s.assignment is None or obs.robot_id not in s.assignment.robots():
        ctx.reason('OBSERVATION_ROBOT_NOT_ASSIGNED')
        return

    if obs.kind not in gm.OBSERVATION_EXPIRY_S:
        ctx.reason('OBSERVATION_KIND_UNKNOWN')
        return

    if not _accepts(s, obs.kind, obs.provenance):
        ctx.reason('PROVENANCE_REJECTED')
        return

    if obs.kind in OBJECT_KINDS and obs.object_id != s.spec.object_id:
        ctx.reason('OBSERVATION_OBJECT_MISMATCH')
        return

    if obs.stamp_sim_s > ctx.now.sim_time_s:
        ctx.reason('OBSERVATION_FROM_FUTURE')
        return

    key = (obs.kind, obs.robot_id)
    retired = s.retired_sessions.setdefault(key, [])
    if obs.session_id in retired:
        ctx.reason('OBSERVATION_SESSION_RETIRED')
        return

    previous = s.observations.get(key)
    if previous is not None:
        if previous.session_id == obs.session_id and obs.revision < previous.revision:
            ctx.reason('OBSERVATION_REVISION_REGRESSED')
            return

        # 새 세션의 첫 관측을 받으면 이전 세션을 은퇴시킨다(세션끼리 선후 비교 없음)
        if previous.session_id != obs.session_id:
            retired.append(previous.session_id)
            del retired[:-ctx.limits.max_retired_sessions]

    s.observations[key] = obs
    _load_watch(ctx, obs)


# CONFIRM_LOAD 결과 내용 검사: 기대 적재 상태와 최신 관측 모순 확인
def _check_confirm_result(ctx, run, plan_step):
    expected = gm.LOAD_STOWED if plan_step.location == gm.SOURCE else gm.LOAD_EMPTY
    t = ctx.s.assignment.transporter_robot_id

    if run.load_state == gm.LOAD_UNKNOWN:
        ctx.inhibit('LOAD_UNKNOWN')
        return

    if run.load_state == gm.LOAD_LOST:
        ctx.inhibit('LOAD_LOST')
        return

    if run.load_state != expected:
        ctx.inhibit('EVIDENCE_CONFLICT')
        return

    obs = _fresh(ctx, 'LOAD', t, -1.0, ctx.s.spec.object_id)
    if obs is None:
        return

    _note_evidence(ctx.s, 'LOAD', obs.provenance)
    if obs.value != expected:
        ctx.inhibit('EVIDENCE_CONFLICT')


# 하위 Action 결과 처리. goal UUID로 결합하고, 주행 단계는 D1 execution_id로도 결합한다
def _on_result(ctx, event):
    s = ctx.s
    run = _step_by_uuid(s, event.goal_uuid)
    if run is None and event.execution_id:
        candidate = _step_by_execution(s, event.execution_id)
        if (candidate is not None and candidate.goal_uuid
                and gm.step_by_key(s.spec, candidate.step_key).operation == 'NAVIGATE_TO'):
            run = candidate

    if run is None:
        ctx.reason('UNBOUND_RESULT')
        return

    if event.robot_id != _robot_of(s, run.step_key):
        ctx.reason('RESULT_ROBOT_MISMATCH')
        return

    if not _accepts(s, _step_kind(s, run.step_key), event.provenance):
        ctx.reason('PROVENANCE_REJECTED')
        return

    if event.outcome not in gm.OUTCOMES:
        ctx.inhibit('INVALID_RESULT')
        return

    # 결과를 낸 실행이 지금의 배정 세대·인계와 같은지 대조한다(다르면 결과를 쓰지 않는다)
    if run.assignment_generation != s.assignment.assignment_generation:
        ctx.inhibit('ASSIGNMENT_STALE')
        return

    plan_step = gm.step_by_key(s.spec, run.step_key)
    if (plan_step.operation in ('LOCK_HANDOFF', 'UNLOCK_HANDOFF')
            and run.transfer_id != s.transfer_id):
        ctx.inhibit('TRANSFER_MISMATCH')
        return

    if run.outcome != -1:
        if run.outcome == event.outcome:
            ctx.reason('DUPLICATE_RESULT')
        else:
            ctx.inhibit('EVIDENCE_CONFLICT')
        return

    run.outcome = event.outcome
    _note_evidence(s, _step_kind(s, run.step_key), event.provenance)
    run.error_code = event.error_code
    run.object_hold_state = event.object_hold_state
    run.load_state = event.load_state
    run.finished_boottime_s = ctx.now.boottime_s

    # 시간초과·상태 보고로 이미 미확정인 단계: 늦은 결과는 기록만 하고 진행하지 않는다
    if run.status == 'UNCERTAIN':
        ctx.reason('LATE_RESULT_RECORDED')
        return

    if run.status not in INFLIGHT:
        ctx.inhibit('RESULT_FOR_UNSENT_STEP')
        return

    run.status = OUTCOME_STATUS[event.outcome]
    if event.outcome == gm.OUTCOME_SUCCEEDED:
        if plan_step.operation == 'CONFIRM_LOAD':
            _check_confirm_result(ctx, run, plan_step)
        return

    if event.outcome == gm.OUTCOME_CANCELED and s.cancel_requested:
        return

    ctx.inhibit(f'STEP_{run.status}')


# A3 RobotTaskStatus 관측: ExecutionRef로 결합하되 성공 근거로는 쓰지 않는다
def _on_task_status(ctx, event):
    s = ctx.s
    if s.assignment is None or event.robot_id not in s.assignment.robots():
        ctx.reason('STATUS_ROBOT_NOT_ASSIGNED')
        return

    # 출처 판정에 쓸 근거 종류만 미리 찾는다(결합 검사 순서는 아래에서 그대로 한다)
    target = _step_by_execution(s, event.execution.get('execution_id', ''))
    kind = _step_kind(s, target.step_key) if target is not None else gm.EVIDENCE_LOCAL_TASK
    if not _accepts(s, kind, event.provenance):
        ctx.reason('PROVENANCE_REJECTED')
        return

    ex = event.execution
    if (ex.get('task_id') != s.spec.task_id
            or ex.get('assignment_id') != s.assignment.assignment_id
            or ex.get('assignment_generation') != s.assignment.assignment_generation):
        ctx.reason('STATUS_STALE_ASSIGNMENT')
        return

    run = _step_by_execution(s, ex.get('execution_id', ''))
    if run is None or _robot_of(s, run.step_key) != event.robot_id:
        ctx.reason('STATUS_UNKNOWN_EXECUTION')
        return

    run.status_phase = event.local_phase
    failing = (gm.OUTCOME_FAILED, gm.OUTCOME_CANCELED, gm.OUTCOME_TIMED_OUT,
               gm.OUTCOME_UNCERTAIN)
    if run.status in INFLIGHT and event.last_outcome in failing:
        run.status = 'UNCERTAIN'
        ctx.inhibit('STATUS_REPORTS_FAILURE')


# 배정 사건 처리. 여러 작업 사이 충돌 검사는 registry가 먼저 한다
def _on_assign(ctx, assignment):
    s = ctx.s
    try:
        gm.validate_assignment(assignment)
    except gm.SpecError as error:
        ctx.reason(f'ASSIGNMENT_INVALID_{error.code}')
        return

    if s.task_state not in (gm.TASK_QUEUED, gm.TASK_ASSIGNED):
        ctx.reason('REASSIGNMENT_UNSUPPORTED')
        return

    if s.assignment is not None and (
            assignment.assignment_generation <= s.assignment.assignment_generation):
        ctx.reason('ASSIGNMENT_GENERATION_NOT_INCREASED')
        return

    s.assignment = assignment
    s.task_state = gm.TASK_ASSIGNED


# 취소 요청: 진행 중 단계에 취소 의도, 아직 기록만 된 의도는 철회
def _on_cancel(ctx):
    s = ctx.s
    if s.task_state in gm.TERMINAL_TASK_STATES or s.cancel_requested:
        ctx.reason('CANCEL_IGNORED')
        return

    s.cancel_requested = True
    s.cancel_requested_boottime_s = ctx.now.boottime_s
    if s.task_state in (gm.TASK_QUEUED, gm.TASK_ASSIGNED):
        return

    for run in s.steps.values():
        if run.status == 'INTENDED':
            run.status = 'WITHDRAWN'
        elif run.status in INFLIGHT:
            plan_step = gm.step_by_key(s.spec, run.step_key)
            execution, args = gm.build_execution(s.spec, s.assignment, plan_step,
                                                 run.attempt_index)
            ctx.effects.append(CommandIntent(
                kind='CANCEL', task_id=s.spec.task_id, step_key=run.step_key,
                operation=plan_step.operation, robot_id=args['robot_id'],
                peer_robot_id=args['peer_robot_id'], execution=execution, args=args,
                args_digest=gm.args_digest({'execution': execution, 'args': args}),
                goal_uuid=run.goal_uuid))


# 사건별 처리 분기
def _dispatch_event(ctx, event):
    s = ctx.s

    if isinstance(event, AssignRoles):
        _on_assign(ctx, event.assignment)

    elif isinstance(event, StartTask):
        if s.task_state == gm.TASK_ASSIGNED:
            s.task_state = gm.TASK_RUNNING
        else:
            ctx.reason('START_REJECTED')

    elif isinstance(event, DispatchReserved):
        run = _step_by_execution(s, event.execution_id)
        if run is None or run.status != 'INTENDED' or not event.goal_uuid:
            ctx.reason('RESERVE_MISMATCH')
        else:
            run.status = 'RESERVED'
            run.goal_uuid = event.goal_uuid
            run.sent_boottime_s = ctx.now.boottime_s
            mark_lock_reserved(s, run.step_key)

    elif isinstance(event, DispatchSent):
        run = _step_by_execution(s, event.execution_id)
        if run is None or run.status not in INFLIGHT + ('SUCCEEDED', 'FAILED', 'CANCELED',
                                                        'TIMED_OUT', 'UNCERTAIN'):
            ctx.reason('SENT_MISMATCH')
        elif run.status == 'RESERVED':
            run.status = 'SENT'

    elif isinstance(event, DispatchFailed):
        run = _step_by_execution(s, event.execution_id)
        if run is None or run.status not in INFLIGHT + ('INTENDED',):
            ctx.reason('DISPATCH_FAILED_MISMATCH')
        elif run.status == 'INTENDED' and event.uncertain:
            # 주행은 D1이 UUID를 만들므로 엔진 예약 반영 전에 전송이 시작된다: 미확정이면 보냈을 수 있다
            run.status = 'UNCERTAIN'
            ctx.inhibit('DISPATCH_UNCERTAIN')
        elif run.status == 'INTENDED':
            # 확실한 미전송(로컬 작업은 예약이 항상 전송보다 먼저, 주행은 포트가 미전송을 확정)
            run.status = 'NOT_SENT'
            ctx.inhibit('DISPATCH_FAILED')
            ctx.reason(event.reason or 'NOT_RESERVED')
        elif event.uncertain:
            run.status = 'UNCERTAIN'
            ctx.inhibit('DISPATCH_UNCERTAIN')
        else:
            run.status = 'NOT_SENT'
            ctx.inhibit('DISPATCH_FAILED')

    elif isinstance(event, GoalResponse):
        run = _step_by_uuid(s, event.goal_uuid)
        if run is None:
            ctx.reason('UNBOUND_GOAL_RESPONSE')
        elif not _accepts(s, _step_kind(s, run.step_key), event.provenance):
            ctx.reason('PROVENANCE_REJECTED')
        elif run.status not in ('RESERVED', 'SENT'):
            ctx.reason('LATE_GOAL_RESPONSE')
        elif event.accepted:
            run.status = 'ACCEPTED'
        else:
            run.status = 'REJECTED'
            ctx.inhibit('GOAL_REJECTED')

    elif isinstance(event, StepResult):
        _on_result(ctx, event)

    elif isinstance(event, TaskStatusSeen):
        _on_task_status(ctx, event)

    elif isinstance(event, ObservationReceived):
        _on_observation(ctx, event.observation)

    elif isinstance(event, HoldApplied):
        if type(event.hold_id) is not str or not gm.ID_PATTERN.match(event.hold_id):
            ctx.reason('INVALID_HOLD_ID')
        elif event.hold_id in s.hold_ids:
            ctx.reason('DUPLICATE_HOLD_ID')
        elif len(s.hold_ids) >= ctx.limits.max_hold_ids:
            ctx.inhibit('HOLD_LIMIT')
        else:
            s.hold_ids.append(event.hold_id)

    elif isinstance(event, HoldReleaseRequested):
        ctx.reason('HOLD_RELEASE_UNSUPPORTED')

    elif isinstance(event, CancelRequested):
        _on_cancel(ctx)

    elif isinstance(event, CancelAcknowledged):
        ctx.reason('CANCEL_ACK_RECORDED')

    elif isinstance(event, StoreFault):
        ctx.inhibit('STORE_FAULT')

    elif isinstance(event, FailTerminate):
        if type(event.failure_event_id) is not str or not gm.ID_PATTERN.match(
                event.failure_event_id):
            ctx.reason('INVALID_FAILURE_EVENT_ID')
        elif s.task_state in gm.TERMINAL_TASK_STATES:
            ctx.reason('ALREADY_TERMINAL')
        elif any(run.status in INFLIGHT for run in s.steps.values()):
            ctx.reason('INFLIGHT_STEPS')
        else:
            s.task_state = gm.TASK_FAILED
            s.failure_event_id = event.failure_event_id
            for run in s.steps.values():
                if run.status == 'INTENDED':
                    run.status = 'WITHDRAWN'

    elif isinstance(event, Tick):
        pass

    else:
        ctx.reason('UNKNOWN_EVENT')


# 사건 하나를 적용해 다음 상태·명령 의도·사유를 돌려준다
def apply_event(state, event, now, limits=EngineLimits()):
    ctx = _Context(state.copy(), now, limits)
    s = ctx.s

    # sim time 역행은 지속 차단(리셋 등은 실제 상대 재대조 전까지 해제하지 않음)
    if now.sim_time_s < s.last_sim_time_s:
        ctx.inhibit('CLOCK_REVERSED')
    else:
        s.last_sim_time_s = now.sim_time_s

    _dispatch_event(ctx, event)
    _timeouts(ctx)
    _transitions(ctx)
    _cancel_completion(ctx)

    # 새 단계 의도: 선행 성공 + 단계 조건 + 차단 없음. 전이 뒤 다시 확인해 연속 단계를 반영
    blocked = _blocked(ctx)
    if blocked:
        ctx.reason(blocked)
    elif s.assignment is not None:
        for plan_step in s.spec.steps:
            run = s.steps[plan_step.step_key]
            if run.status != 'PENDING':
                continue
            if not _all_succeeded(s, plan_step.depends_on):
                continue
            if _gate(ctx, plan_step) and not s.inhibit:
                _emit(ctx, plan_step)

    _object_tracking(ctx)

    # 잠금이 걸린 채 차단·종료된 작업: 명시적 인계 중단·잠금 해제 절차(9.7)는 아직 없다
    stuck = s.inhibit or s.task_state in (gm.TASK_FAILED, gm.TASK_CANCELED)
    if stuck and s.transfer_state not in gm.RELEASED_TRANSFER_STATES:
        ctx.reason('HANDOFF_RECOVERY_UNSUPPORTED')

    if persistent_view(s) != persistent_view(state):
        s.revision = state.revision + 1

    return Decision(s, tuple(ctx.effects), tuple(ctx.reasons))


# 물체를 운반 역할 로봇에 둔다(carrier·배정 세대 함께)
def _carry(s, state, robot_id):
    s.object_state = state
    s.object_location_id = ''
    s.object_carrier_robot_id = robot_id
    s.object_assignment_generation = s.assignment.assignment_generation


# 그룹 물체 상태 추적(W5). 계획·전송·수락만으로 바꾸지 않고, 단계 결과와 그 뒤 최신 관측이 함께
# 있을 때만 승격한다. 집기 요청 예약 전에는 출발 지점, 예약 뒤 실패·미확정·모순은 UNKNOWN(격리).
# 도착 지점은 COMPLETED와 같은 판정에서만 된다(E-31). 복원 미대조 중에는 바꾸지 않는다
def _object_tracking(ctx):
    s = ctx.s
    if s.assignment is None or 'RESTORED_UNRECONCILED' in s.inhibit:
        return

    if s.object_state in (gm.INV_AT_DESTINATION, gm.INV_LOST):
        return

    m, t = s.assignment.robots()
    obj = s.spec.object_id
    pick = s.steps['load_pick_and_stow']
    if pick.goal_uuid:
        s.physical_started = True

    # 완료 판정과 같은 사건: 배치·해제·후퇴·EMPTY·정지·두 해제 결과가 모두 확인됨
    if s.task_state == gm.TASK_COMPLETED:
        s.object_state = gm.INV_AT_DESTINATION
        s.object_location_id = s.spec.destination_location_id
        s.object_carrier_robot_id = ''
        s.object_isolated = False
        return

    # 운반 중 이탈 확인(적재 감시·적재 확인 결과)
    lost = ('LOAD_LOST' in s.inhibit
            or any(run.load_state == gm.LOAD_LOST for run in s.steps.values()
                   if run.outcome != -1))
    if lost and s.physical_started:
        s.object_state = gm.INV_LOST
        s.object_location_id = ''
        s.object_isolated = True
        return

    if not s.physical_started:
        return

    # 물리 동작 시작 뒤 실패·미확정·모순·실패 종료·취소: 위치를 확정할 수 없다
    if s.inhibit or s.task_state in (gm.TASK_FAILED, gm.TASK_CANCELED):
        s.object_state = gm.INV_UNKNOWN
        s.object_location_id = ''
        s.object_isolated = True
        return

    confirm = s.steps['load_confirm']
    unload = s.steps['unload_place']

    # 하역 요청 예약 뒤 조작 로봇 보유 관측
    if (s.object_state == gm.INV_STOWED_ON_TRANSPORTER and unload.goal_uuid
            and _fresh_is(ctx, 'HOLD', m, gm.HOLD_HELD, unload.sent_boottime_s, obj)):
        _carry(s, gm.INV_HELD_BY_MANIPULATOR, m)
        return

    # 적재 확인 결과 STOWED와 그 뒤 최신 적재 관측 STOWED(집기 중 보유 관측이 없어도 바로 승격).
    # 하역 요청 예약 뒤에는 출발 적재 근거로 되돌리지 않는다
    if (s.object_state in (gm.INV_AT_SOURCE, gm.INV_HELD_BY_MANIPULATOR)
            and not unload.goal_uuid and confirm.status == 'SUCCEEDED'
            and confirm.load_state == gm.LOAD_STOWED
            and _fresh_is(ctx, 'LOAD', t, gm.LOAD_STOWED, confirm.finished_boottime_s, obj)):
        _carry(s, gm.INV_STOWED_ON_TRANSPORTER, t)
        return

    # 집기 요청 예약 뒤 조작 로봇 보유 관측
    if (s.object_state == gm.INV_AT_SOURCE
            and _fresh_is(ctx, 'HOLD', m, gm.HOLD_HELD, pick.sent_boottime_s, obj)):
        _carry(s, gm.INV_HELD_BY_MANIPULATOR, m)


# 원장에서 읽은 상태의 재시작 복원: 관측 폐기, 진행 중 단계 미확정, 새 명령 차단
def restore_state(state):
    s = state.copy()
    s.observations = {}
    s.retired_sessions = {}

    for run in s.steps.values():
        if run.status in INFLIGHT:
            run.status = 'UNCERTAIN'

    # 예약 기록이 있는 잠금 요청은 잠금 확정 전이라도 UNKNOWN으로 둔다(이전 원장 포함)
    for run in s.steps.values():
        if run.goal_uuid:
            mark_lock_reserved(s, run.step_key)

    if 'RESTORED_UNRECONCILED' not in s.inhibit:
        s.inhibit.append('RESTORED_UNRECONCILED')

    return replace(s, revision=state.revision + 1)
