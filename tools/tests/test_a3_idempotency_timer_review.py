"""Exercise ledger and timer regressions without starting a ROS graph."""
from pathlib import Path
import sqlite3
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_a3_runtime_authority_ros as operational
import kim_ros_test_domain as domains
from ax_task_executor.assignment_monitor import AssignmentMonitor
from ax_task_executor.runtime_authority import RuntimeAuthority


class IdempotencyReviewTests(operational.OperationalLedgerTests):
    """Include prior ledger regressions and the reported duplicate scenarios."""

    def test_denial_stays_denied_after_authority_recovers(self):
        # 핵심: 권한이 복구돼도 같은 요청의 최초 거절과 수락 기록 개수는 그대로입니다.
        current = self.checker.authority
        missing = RuntimeAuthority('robot_1', lambda: self.now)
        self.addCleanup(missing.close)
        self.checker.authority = missing
        denied = self.checker.check(self.request)
        self.assertEqual(denied['reason_code'], 'ASSIGNMENT_STALE')
        self.checker.authority = current
        self.assertEqual(self.checker.check(self.request), denied)
        self.assertEqual(self.ledger.accept_count(), 0)

    def test_changed_digest_wins_over_revoke(self):
        # 핵심: 철회 여부보다 요청·명령 인자 충돌을 먼저 검사해야 합니다.
        self.assertTrue(self.checker.check(self.request)['allowed'])
        self.authority.revoke(self.request.execution_id, self.request.goal_id, 1)
        self.request.command_digest = 'f' * 64
        self.assertEqual(self.checker.check(self.request)['reason_code'], 'DUPLICATE_REQUEST')
        self.request.request_id += '-new'
        self.assertEqual(self.checker.check(self.request)['reason_code'], 'DUPLICATE_REQUEST')
        self.assertEqual(self.ledger.accept_count(), 1)

    def test_rechecked_denial_survives_ledger_reopen(self):
        # 핵심: 원장을 다시 열어도 재검사 거절이 유지돼야 합니다.
        self.assertTrue(self.checker.check(self.request)['allowed'])
        self.authority.revoke(self.request.execution_id, self.request.goal_id, 1)
        denied = self.checker.check(self.request)
        with self.ledger.lock:
            row = self.ledger.db.execute('SELECT response FROM checks').fetchone()
        self.assertIn('LEASE_REVOKED', row[0])
        reopened = operational.LeaseLedger(self.ledger.path, 'robot_1')
        self.addCleanup(reopened.close)
        self.checker.ledger = reopened
        self.assertEqual(self.checker.check(self.request), denied)
        self.assertEqual(self.ledger.accept_count(), 1)


class TimerReviewTests(unittest.TestCase):
    """Periodic failures stop tracking and revoke authority instead of escaping."""

    def test_real_pending_limit_is_contained(self):
        # 핵심: 실제 대기 상한을 채워 타이머 예외가 밖으로 나가지 않는지 재현합니다.
        now = [100.0]
        authority = RuntimeAuthority('robot_1', lambda: now[0])
        self.addCleanup(authority.close)
        execution = SimpleNamespace(execution_id='task', assignment_id='assignment',
                                    assignment_generation=1)
        serial = [0]

        def send(task):
            serial[0] += 1
            authority.begin_assignment_check(str(serial[0]), task.assignment_id, 1)

        monitor = AssignmentMonitor(authority, send)
        monitor.start(execution)
        for index in range(authority.MAX_PENDING_CHECKS - 1):
            authority.begin_assignment_check(f'filled-{index}', 'assignment', 1)
        now[0] += 0.5
        logs = []
        monitor.timer_tick(logs.append)
        self.assertIn('too many outstanding assignment checks', logs[-1])
        self.assertIsNone(monitor.execution)
        self.assertEqual(authority.pending, {})

    def test_timer_contains_pending_storage_and_publish_errors(self):
        # 핵심: 오류 종류와 관계없이 권한을 무효화하고 자동 재개하지 않습니다.
        for error in (ValueError('too many pending checks'),
                      sqlite3.OperationalError('disk full'), RuntimeError('publish failed')):
            with self.subTest(error=error):
                now = [100.0]
                authority = RuntimeAuthority('robot_1', lambda: now[0])
                self.addCleanup(authority.close)
                execution = SimpleNamespace(execution_id='task', assignment_id='assignment',
                                            assignment_generation=1)
                sender = Mock()
                monitor = AssignmentMonitor(authority, sender)
                monitor.start(execution)
                authority.begin_assignment_check('pending', 'assignment', 1)
                sender.side_effect = error
                now[0] += 1
                logs = []
                monitor.timer_tick(logs.append)
                self.assertIsNone(monitor.execution)
                self.assertIsNone(authority.assignment)
                self.assertEqual(authority.pending, {})
                self.assertIn(str(error), logs[-1])
                count = sender.call_count
                monitor.timer_tick(logs.append)
                self.assertEqual(sender.call_count, count)


class DomainGuardTests(unittest.TestCase):
    """Check isolation setup using mocks; never publish into any ROS domain."""

    def test_missing_range_skips_before_reservation(self):
        with patch.dict(domains.os.environ, {}, clear=True), patch.object(
                domains.ros_test_domain, 'reserve_domain') as reserve:
            with self.assertRaises(unittest.SkipTest):
                domains.setUpModule()
            reserve.assert_not_called()

    def test_conflict_releases_lock_without_changing_environment(self):
        lock = object()
        with patch.dict(domains.os.environ, {'AX_KIM_TEST_DOMAINS': '1-2'}, clear=True), \
                patch.object(domains.ros_test_domain, 'reserve_domain', return_value=(1, lock)), \
                patch.object(domains.ros_test_domain, 'conflicting_nodes', return_value=['/busy']), \
                patch.object(domains.ros_test_domain, 'conflicting_services', return_value=[]), \
                patch.object(domains.ros_test_domain, 'release') as release:
            with self.assertRaises(AssertionError):
                domains.setUpModule()
            release.assert_called_once_with(lock)
            self.assertNotIn('ROS_DOMAIN_ID', domains.os.environ)

    def test_team_and_existing_ranges_are_rejected(self):
        for configured in ('138-140', '180-219', '0-2'):
            with patch.dict(domains.os.environ, {'AX_KIM_TEST_DOMAINS': configured}), \
                    patch.object(domains.ros_test_domain, 'reserve_domain') as reserve:
                with self.assertRaises(ValueError):
                    domains.setUpModule()
                reserve.assert_not_called()


if __name__ == '__main__':
    unittest.main()
