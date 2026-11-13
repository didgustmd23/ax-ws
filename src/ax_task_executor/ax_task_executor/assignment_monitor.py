"""기록된 실제 작업에 한해 상위 배정을 주기적으로 확인합니다."""
from copy import deepcopy
import threading


class AssignmentMonitor:
    """실제 작업의 배정 확인 주기와 권한 무효화를 연결합니다."""

    def __init__(self, authority, send_check, period_s=0.5):
        # 역할: 설정과 내부 상태를 준비합니다.
        self.authority = authority
        self.send_check = send_check
        self.period_s = period_s
        self.lock = threading.RLock()
        self.execution = None
        self.next_check = 0.0

    def start(self, execution):
        """기록된 실제 작업을 복사해 배정 확인 추적을 시작합니다."""
        # 실제로 접수·기록한 작업만 추적합니다. 모의 CONFIRM_LOAD는 자동으로 등록하지 않습니다.
        if (not execution.execution_id or not execution.assignment_id
                or execution.assignment_generation <= 0):
            raise ValueError('recorded execution and assignment required')
        with self.lock:
            if self.execution is not None:
                raise ValueError('stop the current task before tracking another')
            self.execution = deepcopy(execution)
            # 원본 요청이 나중에 바뀌어도 추적 대상이 몰래 바뀌지 않도록 복사합니다.
            self.next_check = 0.0
            try:
                self.tick()
            except Exception:
                # 첫 요청이 실패하면 임시 작업·대기 권한을 함께 정리해 재시도를 허용합니다.
                self.authority.invalidate_assignment()
                self.execution = None
                self.next_check = 0.0
                # 실패를 성공처럼 숨기지 않고 원래 오류를 호출자에게 다시 전달합니다.
                raise

    def tick(self):
        """기한 만료를 먼저 반영하고 주기가 됐을 때 배정을 재확인합니다."""
        # 타이머가 호출돼도 매번 요청하지 않습니다. 기본 0.5초 간격이 됐을 때만 재확인합니다.
        with self.lock:
            if self.execution is None:
                return
            self.authority.expire_assignment()
            # 새 확인보다 만료 처리가 먼저입니다. 끊긴 권한의 옛 Goal을 되살리지 않습니다.
            now = self.authority.clock()
            if now >= self.next_check:
                self.send_check(self.execution)
                self.next_check = now + self.period_s

    def stop(self, execution_id):
        """현재 작업의 권한을 무효화하고 추적을 종료합니다."""
        # 현재 작업만 종료합니다. 여기서 권한을 없애도 실제 팔 정지 확인은 별도로 해야 합니다.
        with self.lock:
            if self.execution is None or self.execution.execution_id != execution_id:
                return False
            self.authority.invalidate_assignment()
            self.execution = None
            return True

    def timer_tick(self, log_error):
        """주기 확인의 예외를 잡아 권한과 추적을 무효화하고 로그를 남깁니다."""
        with self.lock:
            try:
                self.tick()
            except Exception as error:
                # pending 상한·SQLite·발행 오류가 ROS 실행기 밖으로 전파되지 않게 합니다.
                # 타이머 오류 뒤에는 자동 재개하지 않습니다. 새 작업 시작이 필요합니다.
                self.execution = None
                self.next_check = 0.0
                try:
                    self.authority.invalidate_assignment()
                except Exception as invalidation_error:
                    log_error(f'Assignment invalidation failed: {invalidation_error!r}')
                log_error(f'Assignment timer failed; tracking stopped: {error!r}')
