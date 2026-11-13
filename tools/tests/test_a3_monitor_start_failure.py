"""Reproduce a failed first send blocking a subsequent task-start retry."""
# [검사 범위] A3 시작 실패: 전송·저장 오류 뒤 임시 상태를 정리하고 다시 시작할 수 있는지 확인합니다.
from pathlib import Path
import sqlite3
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src/ax_task_executor'))
from ax_task_executor.assignment_monitor import AssignmentMonitor  # noqa: E402
from ax_task_executor.runtime_authority import RuntimeAuthority  # noqa: E402


class MonitorStartFailureTests(unittest.TestCase):
    """A failed start must not leave an invisible running task behind."""

    def test_retry_after_first_send_error_is_not_blocked_by_failed_start(self):
        # [핵심] 시작 실패를 재현한 뒤 재시도합니다. 실패한 작업이 바쁜 상태로 남으면 이 검사가 실패합니다.
        authority = RuntimeAuthority('robot_1', lambda: 100.0)
        self.addCleanup(authority.close)
        execution = SimpleNamespace(execution_id='task-1', assignment_id='assignment',
                                    assignment_generation=1)

        def failed_send(task):
            authority.begin_assignment_check('failed-send', task.assignment_id,
                                             task.assignment_generation)
            raise OSError('simulated publish failure')

        monitor = AssignmentMonitor(authority, failed_send)
        with self.assertRaises(OSError):
            monitor.start(execution)
        self.assertIsNone(monitor.execution)
        self.assertEqual(authority.pending, {})
        # 시작 실패 뒤 늦게 온 허용 응답도 권한을 만들면 안 됩니다.
        self.assertFalse(authority.confirm_assignment(SimpleNamespace(
            request_id='failed-send', a_session_id=authority.session_id,
            d_session_id='d4', assignment_generation=1, sequence=1,
            valid=True, validity_s=2.0)))
        sent = []
        monitor.send_check = sent.append  # peer transport has recovered
        monitor.tick()
        self.assertEqual(sent, [])  # 실패한 작업을 타이머가 몰래 재개하지 않습니다.
        try:
            monitor.start(execution)
        except ValueError as error:
            self.fail(f'failed start left the task busy and blocked retry: {error}')
        self.assertEqual(len(sent), 1)

    def test_storage_or_ros_error_is_preserved_and_clears_tracking(self):
        for error in (sqlite3.OperationalError('disk full'), RuntimeError('publisher unavailable')):
            with self.subTest(error=type(error).__name__):
                authority = RuntimeAuthority('robot_1', lambda: 100.0)
                self.addCleanup(authority.close)
                execution = SimpleNamespace(execution_id='task', assignment_id='assignment',
                                            assignment_generation=1)

                def failed_send(_task):
                    raise error

                monitor = AssignmentMonitor(authority, failed_send)
                with self.assertRaises(type(error)) as raised:
                    monitor.start(execution)
                self.assertIs(raised.exception, error)
                self.assertIsNone(monitor.execution)
                self.assertEqual(authority.pending, {})

    def test_failed_start_invalidates_an_already_received_confirmation(self):
        # [핵심] 시작 도중 확인 응답을 받았더라도 전체 시작이 실패하면 그 권한도 정리해야 합니다.
        authority = RuntimeAuthority('robot_1', lambda: 100.0)
        self.addCleanup(authority.close)
        execution = SimpleNamespace(execution_id='task', assignment_id='assignment',
                                    assignment_generation=1)

        def partial_send(task):
            authority.begin_assignment_check('partial', task.assignment_id,
                                             task.assignment_generation)
            authority.confirm_assignment(SimpleNamespace(
                request_id='partial', a_session_id=authority.session_id, d_session_id='d4',
                assignment_generation=1, sequence=1, valid=True, validity_s=2.0))
            raise OSError('failed after confirmation')

        monitor = AssignmentMonitor(authority, partial_send)
        with self.assertRaises(OSError):
            monitor.start(execution)
        self.assertIsNone(authority.assignment)
        self.assertIsNone(monitor.execution)


if __name__ == '__main__':
    unittest.main()
