"""A3의 작업 없는 모의 대기 상태를 명시합니다. D4 처리 방식은 공동 검토 대상입니다."""
from ax_interfaces.msg import ExecutionRef, ResultStatus, RobotTaskStatus


def make_unassigned_mock_idle(robot_id, session_id):
    """실제 배정이나 실행 번호를 만들지 않고 모의 대기 메시지를 생성합니다."""
    if robot_id not in ('robot_1', 'robot_2', 'robot_3', 'robot_4') or not session_id:
        raise ValueError('known robot and nonempty source session required')
    status = RobotTaskStatus(robot_id=robot_id)
    status.meta.schema_version = 1
    status.meta.source_id = f'a3_mock_executor_{robot_id}'
    status.meta.source_session_id = session_id
    status.meta.revision = 1
    # 대기 heartbeat이지 운영 권한이 아닙니다. D4 오류를 피하려고 valid=true로 바꾸지 않습니다.
    status.meta.valid = False
    status.meta.validity_reason = 'MOCK_ONLY'
    status.local_phase = 'MOCK_IDLE'
    # ExecutionRef와 last_result는 메시지 기본값으로 둡니다. 가짜 task/assignment ID는 만들지 않습니다.
    return status


def is_unassigned_mock_idle(status):
    """정확히 작업 없는 모의 대기 형태인지 확인합니다. 로봇 가용성 판단은 아닙니다."""
    return (
        status.robot_id in ('robot_1', 'robot_2', 'robot_3', 'robot_4')
        and status.meta.schema_version == 1 and status.meta.revision > 0
        and status.meta.source_id == f'a3_mock_executor_{status.robot_id}'
        and bool(status.meta.source_session_id)
        and not status.meta.valid and status.meta.validity_reason == 'MOCK_ONLY'
        and status.local_phase == 'MOCK_IDLE' and not status.paused
        # execution_id 하나만 빈 것이 아니라 전체 실행 필드가 기본값이어야 합니다.
        and status.execution == ExecutionRef() and status.last_result == ResultStatus())
