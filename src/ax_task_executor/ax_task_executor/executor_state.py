"""
로컬 작업 실행기 한 프로세스 안의 공유 상태(Action 실행 상태와 앞으로의 lease 상태 자리).

잠금 규칙(교착 방지):
- `ExecutorState.lock`은 사전·집합 값을 읽고 쓰는 동안만 잡는다.
- 잠금 안에서 SQLite 쓰기(fsync)·ROS 발행·서비스 호출·await를 하지 않는다.
- 다른 잠금(ExecutionStore.lock, LeaseLedger.lock, 상태 메시지 잠금)을 이 잠금 안에서 잡지 않는다.
  즉 이 잠금은 항상 마지막(가장 안쪽)에 잡고 곧바로 놓는 잎(leaf) 잠금이다.
"""
from dataclasses import dataclass, field
import threading


@dataclass
class LeaseSlot:
    """앞으로 C4 활성 Goal·lease 세대를 넣을 자리. 지금은 비어 있고 판정에 쓰지 않는다."""

    goal_id: str = ''
    execution_id: str = ''
    lease_generation: int = 0
    revoked_generations: set = field(default_factory=set)


class ExecutorState:
    """현재 실행·진행 중 처리·취소 표시를 잠금 하나로 보호한다."""

    # 빈 상태로 시작한다
    def __init__(self):
        # 역할: 설정과 내부 상태를 준비합니다.
        self.lock = threading.Lock()
        self.active_execution_id = None
        # execution_id → 실제로 처리 중인 Goal 키(Goal UUID 16진 문자열)
        self.owner_goal = {}
        # execution_id → 처리 완료 Future(재요청 Goal이 기다린다)
        self.inflight = {}
        # 취소를 받아들인 Goal 키. 재요청 Goal 취소가 원래 작업을 멈추지 않게 Goal 단위로 둔다.
        self.cancel_requested = set()
        # 운영 lease 판정용 자리(C4 활성 Goal·세대). 개발 정책은 보지 않는다.
        self.lease = LeaseSlot()

    # 처리 중 작업 표시를 확인하고, 비어 있으면 이 실행으로 잡는다. (busy, claimed)를 돌려준다
    def claim(self, execution_id, has_result):
        # 로봇의 실행 자리만 선점합니다. 이 표시 자체가 실제 구동 권한은 아닙니다.
        # 역할: 다른 작업과 겹치지 않도록 현재 실행 처리권을 선점합니다.
        with self.lock:
            busy = self.active_execution_id not in (None, execution_id)

            if busy or has_result:
                return busy, False

            claimed = self.active_execution_id is None
            self.active_execution_id = execution_id

            return False, claimed

    # 이 실행이 처리 중 작업 표시를 가지고 있으면 놓는다
    def release(self, execution_id):
        # 역할: 현재 소유권과 이탈 증거를 확인하거나 실행 중 선점을 해제합니다.
        with self.lock:
            if self.active_execution_id == execution_id:
                self.active_execution_id = None

    # 처리 시작을 기록한다. 이미 처리 중이면 그 Future를 돌려준다(재요청). 새로 시작하면 None
    def begin(self, execution_id, goal_key, future):
        # 역할: 해당 실행의 처리 중 요청과 완료 대기 객체를 등록합니다.
        with self.lock:
            if execution_id in self.inflight:
                return self.inflight[execution_id]

            self.inflight[execution_id] = future
            self.owner_goal[execution_id] = goal_key

            return None

    # 처리 종료를 기록하고 처리 중 표시를 정리한다
    def finish(self, execution_id, goal_key):
        # 역할: 현재 실행의 처리 표시와 대기 객체를 정리합니다.
        with self.lock:
            self.inflight.pop(execution_id, None)
            self.owner_goal.pop(execution_id, None)
            self.cancel_requested.discard(goal_key)

            if self.active_execution_id == execution_id:
                self.active_execution_id = None

    # 원래 Goal이면 취소 표시를 남기고 True, 재요청 Goal이면 False
    def request_cancel(self, execution_id, goal_key):
        # 원래 Goal의 취소만 받습니다. 결과를 재조회한 Goal이 원래 작업을 멈추면 안 됩니다.
        # 역할: 현재 처리 중인 원래 요청에 취소 의사를 기록합니다.
        with self.lock:
            if self.owner_goal.get(execution_id) != goal_key:
                return False

            self.cancel_requested.add(goal_key)

            return True

    # 이 Goal에 취소 표시가 있는지 확인한다
    def is_cancel_requested(self, goal_key):
        # 역할: 현재 요청에 취소 표시가 있는지 확인합니다.
        with self.lock:
            return goal_key in self.cancel_requested

    # 이 실행을 다른 Goal이 처리 중인지 확인한다
    def is_inflight(self, execution_id):
        # 역할: 해당 실행이 아직 처리 중인지 확인합니다.
        with self.lock:
            return execution_id in self.inflight
