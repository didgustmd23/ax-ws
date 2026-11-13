"""D4 heartbeat lifecycle and suspension recovery, without changing D4 code."""
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src/ax_task_executor'))
from ax_task_executor.assignment_monitor import AssignmentMonitor  # noqa: E402
from ax_task_executor.runtime_authority import RuntimeAuthority  # noqa: E402


class AssignmentMonitorTests(unittest.TestCase):
    """Verify the consumer remains fail closed when its D4 peer is unavailable."""

    def setUp(self):
        self.now = 100.0
        self.authority = RuntimeAuthority('robot_1', lambda: self.now)
        self.sent = []
        self.execution = SimpleNamespace(
            execution_id='task-1', assignment_id='assignment-1', assignment_generation=2)
        self.monitor = AssignmentMonitor(self.authority, self.send)

    def send(self, execution):
        request_id = f'check-{len(self.sent)}'
        self.sent.append((request_id, execution))
        self.authority.begin_assignment_check(
            request_id, execution.assignment_id, execution.assignment_generation)

    def reply(self, sequence):
        return self.authority.confirm_assignment(SimpleNamespace(
            request_id=self.sent[-1][0], a_session_id=self.authority.session_id,
            d_session_id='d4', assignment_generation=2, sequence=sequence,
            validity_s=2.0, valid=True))

    def activate(self):
        self.monitor.start(self.execution)
        self.assertTrue(self.reply(1))
        self.authority.activate_goal(self.execution, local_task_kind=6)

    def fields(self):
        return dict(authority=2, robot_id='robot_1', execution_id='task-1',
                    local_task_kind=6, goal_id='', lease_generation=0, command_source=2)

    def test_idle_sends_nothing_and_active_task_renews_at_half_second(self):
        self.monitor.tick()
        self.assertEqual(self.sent, [])
        self.monitor.start(self.execution)
        self.assertEqual(len(self.sent), 1)
        self.now += 0.49
        self.monitor.tick()
        self.assertEqual(len(self.sent), 1)
        self.now += 0.01
        self.monitor.tick()
        self.assertEqual(len(self.sent), 2)
        self.assertNotEqual(self.sent[0][0], self.sent[1][0])

    def test_missing_d4_response_never_grants_authority(self):
        self.monitor.start(self.execution)
        self.now += 5
        self.monitor.tick()
        self.assertEqual(self.authority.decide(self.fields()), 'ASSIGNMENT_STALE')

    def test_stop_invalidates_and_does_not_publish_again(self):
        self.activate()
        self.assertEqual(self.authority.decide(self.fields()), '')
        self.assertTrue(self.monitor.stop('task-1'))
        self.now += 10
        self.monitor.tick()
        self.assertEqual(len(self.sent), 1)
        self.assertEqual(self.authority.decide(self.fields()), 'ASSIGNMENT_STALE')
        self.assertFalse(self.reply(2))

    def test_old_task_completion_cannot_stop_current_task(self):
        self.activate()
        self.assertFalse(self.monitor.stop('old-task'))
        self.assertEqual(self.authority.decide(self.fields()), '')
        with self.assertRaises(ValueError):
            self.monitor.start(self.execution)

    def test_renewal_before_expiry_preserves_goal(self):
        self.activate()
        self.now += 0.5
        self.monitor.tick()
        self.assertTrue(self.reply(2))
        self.assertEqual(self.authority.assignment_deadline, 102.5)
        self.assertEqual(self.authority.decide(self.fields()), '')

    def test_recovery_after_suspend_does_not_resurrect_local_goal(self):
        self.activate()
        self.now += 4
        self.monitor.tick()
        self.assertTrue(self.reply(2))
        self.assertEqual(self.authority.decide(self.fields()), 'LEASE_REVOKED')

    def test_reply_after_expiry_without_timer_cannot_resurrect_goal(self):
        self.activate()
        self.now = 101.9
        self.monitor.tick()
        self.now = 102.1
        self.assertTrue(self.reply(2))
        self.assertEqual(self.authority.decide(self.fields()), 'LEASE_REVOKED')


if __name__ == '__main__':
    unittest.main()
