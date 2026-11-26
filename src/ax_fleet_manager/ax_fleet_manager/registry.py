"""여러 그룹 작업 사이의 수용·배정 충돌 검사(순수, ROS 없음).

작업 하나의 단계 진행은 group_machine.apply_event가 맡는다. 이 모듈은 작업 ID·물체·로봇·배정 ID·
인계 ID의 동시 사용과 지도·등록 지점 revision 불일치를 거부한다. 지도·지점 revision은 호출자가
명시 입력으로 준 값이며 B2·D 원장에서 읽은 실제 값이 아니다.
"""
from . import group_model as gm
from .group_events import AssignRoles, Decision
from .group_machine import EngineLimits, apply_event, restore_state


# 다른 작업과 자원을 공유하면 안 되는 상태인지(진행 중이거나 인계 잠금이 남음)
def _occupies(state):
    if state.task_state not in gm.TERMINAL_TASK_STATES:
        return True

    return state.transfer_state not in gm.RELEASED_TRANSFER_STATES


class GroupRegistry:
    """작업 상태 모음과 작업 간 상호 배제 규칙."""

    # 지도 revision·등록 지점 revision·작업 상한·모드·live 허용 종류를 명시로 받는다
    def __init__(self, map_revision, locations, max_tasks=64, limits=EngineLimits(),
                 mode=gm.MODE_DEVELOPMENT_MOCK, live_kinds=()):
        gm.check_mode(mode)
        self.live_kinds = gm.parse_live_kinds(mode, live_kinds)
        if type(map_revision) is not int or map_revision < 0:
            raise gm.SpecError('INVALID_NUMBER', 'map_revision')

        self.map_revision = map_revision
        self.locations = dict(locations)
        self.max_tasks = max_tasks
        self.limits = limits
        self.mode = mode
        self.states = {}

    # 새 작업 수용 전 작업 간 충돌 검사
    def check_admission(self, spec):
        gm.validate_spec(spec)

        if spec.task_id in self.states:
            raise gm.SpecError('DUPLICATE_TASK_ID', spec.task_id)

        if len(self.states) >= self.max_tasks:
            raise gm.SpecError('TASK_LIMIT', str(self.max_tasks))

        if spec.map_revision != self.map_revision:
            raise gm.SpecError('MAP_REVISION_MISMATCH',
                               f'{spec.map_revision} != {self.map_revision}')

        for location_id, revision in (
                (spec.source_location_id, spec.source_location_revision),
                (spec.destination_location_id, spec.destination_location_revision)):
            if location_id not in self.locations:
                raise gm.SpecError('UNKNOWN_LOCATION', location_id)
            if self.locations[location_id] != revision:
                raise gm.SpecError('LOCATION_REVISION_MISMATCH', location_id)

        for other in self.states.values():
            o = other.spec
            if o.root_execution_id == spec.root_execution_id:
                raise gm.SpecError('DUPLICATE_ROOT_EXECUTION_ID', spec.root_execution_id)

            ids = {o.source_transfer_id, o.destination_transfer_id}
            if {spec.source_transfer_id, spec.destination_transfer_id} & ids:
                raise gm.SpecError('DUPLICATE_TRANSFER_ID', spec.task_id)

            if _occupies(other) and o.object_id == spec.object_id:
                raise gm.SpecError('OBJECT_IN_ACTIVE_TASK', spec.object_id)

    # 검사 후 QUEUED 상태로 수용한다
    def admit(self, spec):
        self.check_admission(spec)
        state = gm.new_group_state(spec, self.mode, self.live_kinds)
        self.states[spec.task_id] = state

        return state

    # 배정 충돌: 다른 작업이 쓰는 로봇·배정 ID
    def assignment_conflict(self, task_id, assignment):
        for other_id, other in self.states.items():
            if other_id == task_id or other.assignment is None:
                continue

            if other.assignment.assignment_id == assignment.assignment_id:
                return 'DUPLICATE_ASSIGNMENT_ID'

            if _occupies(other) and set(other.assignment.robots()) & set(assignment.robots()):
                return 'ROBOT_IN_ACTIVE_TASK'

        return None

    # 사건 하나를 평가한다(반영하지 않음). runtime은 원장 기록 후 commit한다
    def evaluate(self, task_id, event, now):
        state = self.states[task_id]

        if isinstance(event, AssignRoles):
            conflict = self.assignment_conflict(task_id, event.assignment)
            if conflict:
                return Decision(state.copy(), (), (conflict,))

        return apply_event(state, event, now, self.limits)

    # 평가 결과 상태를 반영한다
    def commit(self, task_id, state):
        self.states[task_id] = state

    # 평가 후 바로 반영(순수 시험용)
    def apply(self, task_id, event, now):
        decision = self.evaluate(task_id, event, now)
        self.commit(task_id, decision.next_state)

        return decision

    # 원장 상태를 복원 규칙으로 불러온다(새 명령 차단)
    def restore(self, states):
        for state in states:
            if state.spec.task_id in self.states:
                raise gm.SpecError('DUPLICATE_TASK_ID', state.spec.task_id)
            self.states[state.spec.task_id] = restore_state(state)
