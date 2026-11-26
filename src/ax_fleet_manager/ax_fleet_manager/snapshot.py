"""D2 상태 → GroupTaskStatus 필드 사전·목록(ROS 없음).

목록 규칙(계약 4.3): 진행 중 작업과, 종료됐어도 transfer_state가 NONE·UNLOCKED가 아닌 작업은
잠금 해제를 확인할 때까지 목록에 남긴다. 보류·실행 억제 사유는 공개 필드가 없어 hold_ids만 싣는다.
"""
from . import group_model as gm

GROUP_TASK_STATUS_FIELDS = (
    'team_id', 'task_id', 'task_revision', 'state', 'priority', 'schedule_revision',
    'deferred', 'object_id', 'source_location_id', 'destination_location_id',
    'manipulator_robot_id', 'transporter_robot_id', 'assignment_generation', 'transfer_id',
    'transfer_state', 'hold_ids', 'failure_event_id',
)


# 작업 하나의 GroupTaskStatus 필드
def task_status_fields(state):
    spec = state.spec
    assignment = state.assignment

    return {
        'team_id': spec.team_id,
        'task_id': spec.task_id,
        'task_revision': spec.task_revision,
        'state': state.task_state,
        'priority': spec.priority,
        'schedule_revision': spec.schedule_revision,
        'deferred': False,
        'object_id': spec.object_id,
        'source_location_id': spec.source_location_id,
        'destination_location_id': spec.destination_location_id,
        'manipulator_robot_id': assignment.manipulator_robot_id if assignment else '',
        'transporter_robot_id': assignment.transporter_robot_id if assignment else '',
        'assignment_generation': assignment.assignment_generation if assignment else 0,
        'transfer_id': state.transfer_id,
        'transfer_state': state.transfer_state,
        'hold_ids': list(state.hold_ids),
        'failure_event_id': state.failure_event_id,
    }


# 목록에 남겨야 하는 작업인지
def visible_in_list(state):
    if state.task_state not in gm.TERMINAL_TASK_STATES:
        return True

    return state.transfer_state not in gm.RELEASED_TRANSFER_STATES


# 출력 source_id 접두어. 노드 파라미터가 아니라 작업 상태의 모드로 정한다:
# 혼합 모드 작업이 하나라도 있으면 d2_dev_mixed, 아니면 d2_dev_mock
def source_prefix(states):
    if any(s.mode == gm.MODE_DEVELOPMENT_MIXED for s in states):
        return 'd2_dev_mixed:'

    return 'd2_dev_mock:'


# 목록 전체(작업 ID 순서 고정)
def task_list_fields(states):
    return [task_status_fields(s) for s in sorted(states, key=lambda s: s.spec.task_id)
            if visible_in_list(s)]
