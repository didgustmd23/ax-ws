"""Bound request memory without allowing expired or completed IDs to be reused."""
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src/ax_task_executor'))
from ax_task_executor.runtime_authority import RuntimeAuthority  # noqa: E402


class AssignmentHistoryTests(unittest.TestCase):
    """Exercise long-running traffic and replay after pending cleanup."""

    def setUp(self):
        self.now = 100.0
        self.authority = RuntimeAuthority('robot_1', lambda: self.now)
        self.addCleanup(self.authority.close)

    def send(self, request_id):
        self.authority.begin_assignment_check(request_id, 'assignment', 1)

    def reply(self, request_id, sequence):
        return SimpleNamespace(
            request_id=request_id, a_session_id=self.authority.session_id,
            d_session_id='d4', assignment_generation=1, sequence=sequence,
            valid=True, validity_s=2.0)

    def test_unanswered_10000_checks_have_only_live_requests_in_memory(self):
        for index in range(10000):
            self.now += 0.5
            self.send(f'missing-{index}')
            self.assertLessEqual(len(self.authority.pending), 4)
        self.now += 2
        self.authority.expire_assignment()
        self.assertEqual(self.authority.pending, {})

    def test_completed_10000_checks_leave_no_pending_requests(self):
        for index in range(10000):
            self.now += 0.5
            request_id = f'answered-{index}'
            self.send(request_id)
            self.assertTrue(self.authority.confirm_assignment(self.reply(request_id, index + 1)))
            self.assertEqual(self.authority.pending, {})
        self.authority.invalidate_assignment()
        # Lifetime duplicate knowledge must survive cleanup without RAM history.
        self.send('answered-0')
        self.assertFalse(self.authority.confirm_assignment(self.reply('answered-0', 20000)))
        self.assertIsNone(self.authority.assignment)

    def test_expired_request_cannot_be_reenrolled_or_extend_first_send_time(self):
        self.send('expired')
        reply = self.reply('expired', 1)
        self.now += 3
        self.authority.expire_assignment()
        self.assertEqual(self.authority.pending, {})
        self.send('expired')
        self.assertFalse(self.authority.confirm_assignment(reply))
        self.assertIsNone(self.authority.assignment)

    def test_reused_id_cannot_change_assignment_after_cleanup(self):
        self.send('old')
        self.authority.invalidate_assignment()
        with self.assertRaises(ValueError):
            self.authority.begin_assignment_check('old', 'other-assignment', 2)

    def test_pending_burst_is_bounded_and_recovers_after_expiry(self):
        for index in range(self.authority.MAX_PENDING_CHECKS):
            self.send(f'burst-{index}')
        with self.assertRaises(ValueError):
            self.send('overflow')
        self.now += 2.0
        self.send('overflow')
        self.assertEqual(len(self.authority.pending), 1)

    def test_closed_authority_does_not_authorize_and_close_is_idempotent(self):
        self.send('last')
        self.assertTrue(self.authority.confirm_assignment(self.reply('last', 1)))
        self.authority.close()
        self.authority.close()
        self.assertIsNone(self.authority.assignment)
        self.assertFalse(self.authority.confirm_assignment(self.reply('last', 2)))


if __name__ == '__main__':
    unittest.main()
