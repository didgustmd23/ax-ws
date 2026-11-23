"""D2 그룹 엔진의 입력 사건과 출력(명령 의도·판정).

사건은 실제 ROS 메시지가 아니라 포트·어댑터가 경계에서 바꾼 내부 자료다. 명령 의도는
실제 구동 명령이 아니며, 포트가 지원·허가하지 않으면 전송되지 않는다.
"""
from dataclasses import dataclass, field


@dataclass(frozen=True)
class AssignRoles:
    """명시 역할 배정(조작·운송 robot_id 쌍과 배정 세대)."""

    assignment: object


@dataclass(frozen=True)
class StartTask:
    """배정된 작업의 실행 시작 요청."""


@dataclass(frozen=True)
class DispatchReserved:
    """원장에 실행 ID·goal UUID를 기록했다(아직 전송 전)."""

    execution_id: str
    goal_uuid: str


@dataclass(frozen=True)
class DispatchSent:
    """포트가 Goal 전송 호출을 마쳤다(수락 아님)."""

    execution_id: str


@dataclass(frozen=True)
class DispatchFailed:
    """전송 호출 실패. uncertain=True면 상대가 받았는지 알 수 없다."""

    execution_id: str
    uncertain: bool
    reason: str = ''


@dataclass(frozen=True)
class GoalResponse:
    """Goal 수락·거절 응답. goal UUID로 결합한다."""

    goal_uuid: str
    accepted: bool
    provenance: str


@dataclass(frozen=True)
class StepResult:
    """하위 Action 결과. 로컬 작업은 goal UUID로 결합한다(Result에 ExecutionRef 없음).

    주행 단계는 goal UUID를 D1이 만들므로 D1 결과를 execution_id로도 결합할 수 있다(W5).
    """

    goal_uuid: str
    robot_id: str
    outcome: int
    provenance: str
    error_code: str = ''
    object_hold_state: int = 0
    load_state: int = 0
    execution_id: str = ''


@dataclass(frozen=True)
class TaskStatusSeen:
    """A3 RobotTaskStatus 관측. execution은 ExecutionRef 필드 사전이다."""

    robot_id: str
    session_id: str
    revision: int
    execution: dict
    local_phase: str
    last_outcome: int
    provenance: str


@dataclass(frozen=True)
class ObservationReceived:
    """상대 소유 상태(정지·보유·적재·후퇴·배치) 관측."""

    observation: object


@dataclass(frozen=True)
class HoldApplied:
    """보류 ID 등록. 해제 절차(D4 9.6)는 W3에서 지원하지 않는다."""

    hold_id: str


@dataclass(frozen=True)
class HoldReleaseRequested:
    """보류 해제 요청. W3에서는 항상 미지원으로 거부한다."""

    hold_id: str


@dataclass(frozen=True)
class CancelRequested:
    """작업 취소 요청. 진행 중 단계에 취소 의도를 낸다."""


@dataclass(frozen=True)
class CancelAcknowledged:
    """하위 취소 수락 응답. 기록만 하며 취소 완료·잠금 해제 근거가 아니다."""

    goal_uuid: str


@dataclass(frozen=True)
class StoreFault:
    """원장 쓰기 실패·손상. 새 일반 명령을 막는다."""

    reason: str


@dataclass(frozen=True)
class FailTerminate:
    """명시 실패 종료(개발 운영자 결정). 물리 잠금·인계 상태는 유지한다."""

    failure_event_id: str


@dataclass(frozen=True)
class Tick:
    """시간 경과 확인(결과 시간초과·관측 만료 재평가)."""


@dataclass(frozen=True)
class CommandIntent:
    """명령 의도. kind는 LOCAL_TASK·NAVIGATION·CANCEL. 실제 전송은 포트가 한다."""

    kind: str
    task_id: str
    step_key: str
    operation: str
    robot_id: str
    peer_robot_id: str
    execution: dict
    args: dict
    args_digest: str
    goal_uuid: str = ''


@dataclass(frozen=True)
class Decision:
    """apply_event의 결과: 다음 상태, 명령 의도, 판정 사유 코드."""

    next_state: object
    effects: tuple = ()
    reasons: tuple = field(default_factory=tuple)
