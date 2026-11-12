"""
현재 CONFIRM_LOAD 모의 실행기에 연결하는 단계 전환 규칙.

ROS·센서·제어기를 호출하지 않고 허용된 순서인지 검사한다.
취소는 외부 구동 없는 모의 작업에 한해 처리한다. 물리 작업·운영 복구는 별도 대상이다.
"""
from enum import Enum


class TaskPhase(str, Enum):
    IDLE = 'MOCK_IDLE'
    VALIDATING = 'MOCK_VALIDATING'
    CONFIRMING_LOAD = 'MOCK_CONFIRM_LOAD'
    CANCELING = 'MOCK_CANCELING'
    CANCELED = 'MOCK_CANCELED'
    SUCCEEDED = 'MOCK_SUCCEEDED'
    FAILED = 'MOCK_FAILED'
    UNCERTAIN = 'MOCK_UNCERTAIN'


class ConfirmLoadStateMachine:
    # 순서를 명시한다. 종료 단계에는 출구가 없으므로 다시 성공으로 바꿀 수 없다.
    ALLOWED = {
        TaskPhase.IDLE: frozenset({TaskPhase.VALIDATING}),
        TaskPhase.VALIDATING: frozenset({
            TaskPhase.CONFIRMING_LOAD, TaskPhase.FAILED, TaskPhase.UNCERTAIN}),
        TaskPhase.CONFIRMING_LOAD: frozenset({
            TaskPhase.SUCCEEDED, TaskPhase.FAILED, TaskPhase.UNCERTAIN,
            TaskPhase.CANCELING}),
        TaskPhase.CANCELING: frozenset({TaskPhase.CANCELED, TaskPhase.UNCERTAIN}),
        TaskPhase.CANCELED: frozenset(),
        TaskPhase.SUCCEEDED: frozenset(),
        TaskPhase.FAILED: frozenset(),
        TaskPhase.UNCERTAIN: frozenset(),
    }

    def __init__(self):
        # Action 요청마다 별도 객체를 만든다. 로봇·작업 사이 상태를 공유하지 않는다.
        self.phase = TaskPhase.IDLE

    def transition(self, next_phase):
        """허용된 전환만 적용한다. 잘못된 전환은 상태를 바꾸지 않고 거절한다."""
        if next_phase not in self.ALLOWED[self.phase]:
            raise ValueError(f'허용하지 않는 단계 전환: {self.phase.value} -> {next_phase.value}')
        self.phase = next_phase
