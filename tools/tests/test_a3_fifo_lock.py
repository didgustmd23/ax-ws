"""A3 원장 잠금의 대기 순서·상호 배제·예외 해제를 검사합니다."""
from pathlib import Path
import sys
import threading
import time
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/ax_task_executor'))
from ax_task_executor.fair_lock import FifoLock


class FifoLockTests(unittest.TestCase):
    """실제 스레드로 재진입 요청의 추월과 영구 대기를 검사합니다."""

    def test_waiting_request_is_not_overtaken_by_repeated_request(self):
        """이미 줄 선 요청이 먼저 처리되며 같은 호출자의 다음 요청은 뒤에 섭니다."""
        lock = FifoLock()
        order, threads = [], []
        lock.acquire()

        def worker(number):
            for repeat in range(2):
                with lock:
                    order.append((number, repeat))

        try:
            for number in range(4):
                thread = threading.Thread(target=worker, args=(number,), daemon=True)
                threads.append(thread)
                thread.start()
                deadline = time.monotonic() + 3
                while time.monotonic() < deadline:
                    with lock._condition:
                        waiting = len(lock._waiters)
                    if waiting == number + 1:
                        break
                    time.sleep(0.001)
                self.assertEqual(waiting, number + 1)
        finally:
            lock.release()
        for thread in threads:
            thread.join(timeout=3)
            self.assertFalse(thread.is_alive())
        self.assertEqual(order[:4], [(number, 0) for number in range(4)])
        self.assertEqual(sorted(order), [(number, repeat) for number in range(4) for repeat in range(2)])

    def test_exception_releases_lock(self):
        """원장 처리 중 예외가 발생해도 다음 요청이 진행됩니다."""
        lock = FifoLock()
        with self.assertRaises(ValueError):
            with lock:
                raise ValueError('injected SQL/policy failure')
        acquired = threading.Event()

        def next_request():
            with lock:
                acquired.set()

        thread = threading.Thread(target=next_request, daemon=True)
        thread.start()
        thread.join(timeout=3)
        self.assertTrue(acquired.is_set())
        self.assertFalse(thread.is_alive())

    def test_unheld_release_is_rejected(self):
        """잘못된 해제가 다음 요청의 점유 상태를 조용히 바꾸지 않습니다."""
        with self.assertRaises(RuntimeError):
            FifoLock().release()

    def test_interrupted_waiter_does_not_block_following_request(self):
        """대기 중 취소된 요청의 순번을 제거해 뒤 요청이 영구 대기하지 않게 합니다."""
        lock = FifoLock()
        original_wait = lock._condition.wait
        cancelled, completed = threading.Event(), threading.Event()

        def wait(timeout=None):
            if threading.current_thread().name == 'cancelled-waiter':
                raise InterruptedError('injected cancellation')
            return original_wait(timeout)

        def cancelled_request():
            try:
                with lock:
                    pass
            except InterruptedError:
                cancelled.set()

        def following_request():
            with lock:
                completed.set()

        lock.acquire()
        with patch.object(lock._condition, 'wait', side_effect=wait):
            thread = threading.Thread(target=cancelled_request, name='cancelled-waiter', daemon=True)
            thread.start()
            thread.join(timeout=3)
            self.assertTrue(cancelled.is_set())
        survivor = threading.Thread(target=following_request, daemon=True)
        survivor.start()
        lock.release()
        survivor.join(timeout=3)
        self.assertTrue(completed.is_set())
        self.assertFalse(survivor.is_alive())
