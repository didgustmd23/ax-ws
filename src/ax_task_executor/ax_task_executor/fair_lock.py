"""원장 대기 요청을 접수 순서로 처리해 반복 요청이 앞선 대기자를 추월하지 않게 합니다."""
from collections import deque
import threading


class FifoLock:
    """한 번에 한 스레드만 허용하며 이미 대기 중인 요청에 먼저 차례를 줍니다."""

    def __init__(self):
        """실제 상태 보호 잠금과 대기 순서를 분리해 준비합니다."""
        self._condition = threading.Condition(threading.Lock())
        self._waiters = deque()
        self._held = False

    def acquire(self):
        """대기열 앞에 도착한 순서대로 잠금을 얻습니다. 재진입 잠금이 아닙니다."""
        token = object()
        with self._condition:
            self._waiters.append(token)
            try:
                # 단순 Lock은 해제 직후 새 요청이 대기자를 추월할 수 있다. 순번과 점유를 함께 확인한다.
                while self._held or self._waiters[0] is not token:
                    self._condition.wait()
            except BaseException:
                # 대기 취소가 생겨도 사라진 요청이 대기열 앞을 영원히 막지 않게 정리한다.
                self._waiters.remove(token)
                self._condition.notify_all()
                raise
            self._waiters.popleft()
            self._held = True
        return True

    def release(self):
        """원장 처리가 끝난 뒤 다음 대기자에게 차례를 넘깁니다."""
        with self._condition:
            if not self._held:
                raise RuntimeError('release unlocked FifoLock')
            self._held = False
            # 임의 한 스레드만 깨우면 맨 앞 요청이 아닌 스레드가 깨어 진행이 막힐 수 있다.
            self._condition.notify_all()

    def __enter__(self):
        """기존 원장의 with lock 사용 방식과 호환합니다."""
        self.acquire()
        return self

    def __exit__(self, *args):
        """SQL·정책 예외가 발생해도 다음 요청이 처리될 수 있게 해제합니다."""
        self.release()
