"""엔진·원장·포트를 잇는 실행 결합(개발 모의 전용).

순서: 평가 → 원장 기록(상태·의도) → 반영 → 실행 ID별 goal UUID 예약 기록 → 포트 전송.
원장 기록이 실패하면 반영·전송하지 않고 이 runtime을 고장 상태로 두어 새 명령을 막는다.
예약 뒤 중단·응답 실종은 미확정으로 남고 자동 재전송하지 않는다.
"""
import time
from dataclasses import asdict, is_dataclass

from . import group_model as gm
from .group_events import (
    Decision, DispatchFailed, DispatchReserved, DispatchSent, GoalResponse, StepResult,
    StoreFault,
)
from .group_machine import mark_lock_reserved, restore_state
from .ports import PortError
from .registry import GroupRegistry
from .task_store import DispatchAlreadyReserved, StoreError


# 이 프로세스의 CLOCK_BOOTTIME과 sim time을 함께 읽는 기본 시계(sim은 호출자가 주입)
def boottime_s():
    return time.clock_gettime(time.CLOCK_BOOTTIME)


# 사건을 원장 사건 기록용 사전으로 바꾼다
def _payload(event):
    if not is_dataclass(event):
        return {}

    data = asdict(event)
    return {k: (v if isinstance(v, (str, int, float, bool, type(None))) else str(v))
            for k, v in data.items()}


# 예약·전송 기록과 복원 상태를 맞춘다: 예약됐던 실행은 미확정(재전송 없음). 상태 기록 전에 멈춰
# 예약 표에만 남은 잠금 요청도 인계 상태를 UNKNOWN으로 둔다
def reconcile_dispatch_rows(state, rows):
    for row in rows:
        run = state.steps[row['step_key']]
        if (row['status'] in ('SUBMITTING', 'RESERVED', 'SENT')
                and run.status in ('INTENDED', 'UNCERTAIN')):
            run.status = 'UNCERTAIN'
            run.goal_uuid = row['goal_uuid'] or run.goal_uuid
            run.execution_id = row['execution_id']

        if row['goal_uuid']:
            mark_lock_reserved(state, row['step_key'])


# 원장 작업의 모드·live 허용 종류가 runtime 설정과 같은지. 다르면 STORE_MODE_MISMATCH로 거부한다
def check_store_mode(states, mode, live_kinds):
    for state in states:
        if state.mode != mode or tuple(state.live_kinds) != tuple(live_kinds):
            raise gm.SpecError('STORE_MODE_MISMATCH',
                               f'{state.spec.task_id}: {state.mode}{list(state.live_kinds)} '
                               f'!= {mode}{list(live_kinds)}')


class GroupRuntime:
    """작업 모음 하나의 실행 결합. ports는 {'LOCAL_TASK': 포트, 'NAVIGATION': 포트}."""

    # 원장에서 상태를 복원하고(새 명령 차단) 포트·시계를 연결한다. 원장 작업의 모드·live 허용 종류가
    # 이 runtime과 다르면 아무것도 쓰지 않고 거부한다(모의 원장을 혼합 모드로 열지 않는다)
    def __init__(self, store, ports, clock, map_revision, locations,
                 mode=gm.MODE_DEVELOPMENT_MOCK, max_tasks=64, live_kinds=()):
        self.store = store
        self.ports = ports
        self.clock = clock
        self.registry = GroupRegistry(map_revision, locations, max_tasks=max_tasks, mode=mode,
                                      live_kinds=live_kinds)
        self.fault = None
        self.notes = []

        states = store.load_states()
        check_store_mode(states, self.registry.mode, self.registry.live_kinds)

        # 복원 상태(관측 폐기·진행 중 단계 미확정·RESTORED_UNRECONCILED)를 먼저 원장에 남긴다
        for state in states:
            restored = restore_state(state)
            self._reconcile_dispatch_rows(restored)
            store.record_decision(state.spec.task_id, state.revision, Decision(restored),
                                  'Restore', {'previous_revision': state.revision})
            self.registry.commit(state.spec.task_id, restored)

    # 원장의 예약·전송 기록과 복원 상태를 맞춘다
    def _reconcile_dispatch_rows(self, state):
        reconcile_dispatch_rows(state, self.store.dispatch_rows(state.spec.task_id))

    # 새 작업 수용: 검사 → 원장 기록 → 등록
    def admit(self, spec):
        if self.fault:
            raise StoreError(f'원장 고장으로 새 작업 거부: {self.fault}')

        self.registry.check_admission(spec)
        state = gm.new_group_state(spec, self.registry.mode, self.registry.live_kinds)
        self.store.create_task(state)
        self.registry.commit(spec.task_id, state)

        return state

    # 원장 고장: 메모리 상태에 차단을 남기고 이후 모든 전송을 막는다(원장에는 쓰지 못할 수 있음)
    def _enter_fault(self, task_id, reason):
        self.fault = reason
        decision = self.registry.evaluate(task_id, StoreFault(reason), self.clock())
        self.registry.commit(task_id, decision.next_state)
        self.notes.append(('STORE_FAULT', task_id, reason))

        return decision.next_state

    # 사건 하나 처리. 원장 기록이 끝난 판정만 반영·전송한다
    def handle(self, task_id, event):
        if self.fault:
            state = self.registry.states[task_id]
            return Decision(state, (), ('RUNTIME_STORE_FAULT',))

        now = self.clock()
        previous = self.registry.states[task_id]
        decision = self.registry.evaluate(task_id, event, now)

        try:
            self.store.record_decision(task_id, previous.revision, decision,
                                       type(event).__name__, _payload(event))
        except StoreError as error:
            state = self._enter_fault(task_id, f'STORE_WRITE_FAILED:{error}')
            return Decision(state, (), decision.reasons + ('STORE_WRITE_FAILED',))

        self.registry.commit(task_id, decision.next_state)
        self._record_result(event, decision)

        for intent in decision.effects:
            self._execute(task_id, intent)

        return decision

    # 결과·수락 사건은 실행 기록 표에도 남긴다(goal UUID 결합 근거)
    def _record_result(self, event, decision):
        if not isinstance(event, (StepResult, GoalResponse)):
            return

        for run in decision.next_state.steps.values():
            if run.goal_uuid != event.goal_uuid or not run.execution_id:
                continue
            try:
                result = _payload(event) if isinstance(event, StepResult) else None
                self.store.update_dispatch(run.execution_id, run.status, result)
            except StoreError as error:
                self._enter_fault(decision.next_state.spec.task_id,
                                  f'STORE_WRITE_FAILED:{error}')

    # 의도 하나 실행: 예약 기록 → 전송. 미지원 포트·고장이면 보내지 않는다
    def _execute(self, task_id, intent):
        if self.fault:
            return

        port = self.ports.get('LOCAL_TASK' if intent.kind == 'CANCEL' else intent.kind)
        mode = self.registry.mode
        live_kinds = self.registry.live_kinds

        if intent.kind == 'CANCEL':
            if intent.operation == 'NAVIGATE_TO':
                port = self.ports.get('NAVIGATION')
            try:
                if port is None or not port.supported(mode, live_kinds):
                    raise PortError('PORT_UNSUPPORTED', uncertain=False)
                port.cancel(intent)
            except PortError as error:
                self.notes.append(('CANCEL_NOT_SENT', intent.step_key, error.reason))
            return

        execution_id = intent.execution['execution_id']
        if port is None or not port.supported(mode, live_kinds):
            self.handle(task_id, DispatchFailed(execution_id, False, 'PORT_UNSUPPORTED'))
            return

        # 같은 판정의 앞선 의도가 실패·미확정이 되었으면 남은 의도도 보내지 않는다
        state = self.registry.states[task_id]
        run = state.steps[intent.step_key]
        if state.inhibit or state.hold_ids or state.cancel_requested or run.status != 'INTENDED':
            if run.status == 'INTENDED':
                self.handle(task_id, DispatchFailed(execution_id, False, 'BLOCKED_BEFORE_SEND'))
            return

        # 주행은 D1이 goal UUID를 만든다(execution_id 결합). 로컬 작업은 D2가 먼저 예약한다
        if getattr(port, 'generates_goal_uuid', False):
            self._submit_navigation(task_id, intent, port)
            return

        try:
            goal_uuid = self.store.reserve_dispatch(execution_id)
        except DispatchAlreadyReserved as error:
            self.notes.append(('ALREADY_RESERVED_NOT_RESENT', execution_id, str(error)))
            return
        except StoreError as error:
            self._enter_fault(task_id, f'STORE_WRITE_FAILED:{error}')
            return

        self.handle(task_id, DispatchReserved(execution_id, goal_uuid))
        if self.fault:
            return

        try:
            port.dispatch(intent, goal_uuid)
        except PortError as error:
            status = 'UNCERTAIN' if error.uncertain else 'NOT_SENT'
            try:
                self.store.update_dispatch(execution_id, status)
            except StoreError as store_error:
                self._enter_fault(task_id, f'STORE_WRITE_FAILED:{store_error}')
                return
            self.handle(task_id, DispatchFailed(execution_id, error.uncertain, error.reason))
            return

        try:
            self.store.update_dispatch(execution_id, 'SENT')
        except StoreError as error:
            self._enter_fault(task_id, f'STORE_WRITE_FAILED:{error}')
            return

        self.handle(task_id, DispatchSent(execution_id))

    # 주행 전송: SUBMITTING 기록 → D1 submit(같은 execution_id) → D1 goal UUID 기록 → 상태 반영.
    # D2는 주행 UUID를 만들지 않는다. UUID 기록 전에 멈추면 재시작에서 재전송 없이 미확정으로 남는다
    def _submit_navigation(self, task_id, intent, port):
        execution_id = intent.execution['execution_id']

        try:
            self.store.begin_submit(execution_id)
        except DispatchAlreadyReserved as error:
            self.notes.append(('ALREADY_SUBMITTED_NOT_RESENT', execution_id, str(error)))
            return
        except StoreError as error:
            self._enter_fault(task_id, f'STORE_WRITE_FAILED:{error}')
            return

        try:
            goal_uuid = port.dispatch(intent, None)
        except PortError as error:
            status = 'UNCERTAIN' if error.uncertain else 'NOT_SENT'
            try:
                self.store.update_dispatch(execution_id, status)
            except StoreError as store_error:
                self._enter_fault(task_id, f'STORE_WRITE_FAILED:{store_error}')
                return
            self.handle(task_id, DispatchFailed(execution_id, error.uncertain, error.reason))
            return

        # D1이 UUID를 돌려주지 않으면 보냈는지 알 수 없다(미확정)
        try:
            self.store.record_submitted(execution_id, goal_uuid)
        except StoreError as error:
            self._enter_fault(task_id, f'STORE_WRITE_FAILED:{error}')
            return

        self.handle(task_id, DispatchReserved(execution_id, goal_uuid))
        if self.fault:
            return

        try:
            self.store.update_dispatch(execution_id, 'SENT')
        except StoreError as error:
            self._enter_fault(task_id, f'STORE_WRITE_FAILED:{error}')
            return

        self.handle(task_id, DispatchSent(execution_id))

    # 현재 작업 상태 목록
    def states(self):
        return list(self.registry.states.values())
