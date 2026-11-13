"""A3 operational authority regression tests; no ROS or Gazebo required."""
# [검사 범위] A3 권한 검사: 배정·활성 Goal·세대·만료/철회를 검사합니다. 실제 팔 동작 시험은 아닙니다.
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src/ax_task_executor'))
from ax_task_executor.runtime_authority import RuntimeAuthority  # noqa: E402


class RuntimeAuthorityTests(unittest.TestCase):
    """Use a controllable BOOTTIME clock, not sleep-based deadline tests."""

    def setUp(self):
        self.now = 100.0
        self.authority = RuntimeAuthority('robot_1', clock=lambda: self.now)
        self.execution = SimpleNamespace(
            execution_id='pick-1', assignment_id='assignment-1', assignment_generation=3)
        self.fields = dict(robot_id='robot_1', execution_id='pick-1', authority=1,
                           goal_id='a' * 32, lease_generation=7, local_task_kind=0,
                           command_source=2)
        self.sequence = 0

    def confirm(self, request_id='check-1', generation=3, **changes):
        self.authority.begin_assignment_check(request_id, 'assignment-1', generation)
        self.sequence += 1
        data = dict(request_id=request_id, a_session_id=self.authority.session_id,
                    d_session_id='d-session-1', assignment_generation=generation,
                    sequence=self.sequence, valid=True, validity_s=2.0)
        data.update(changes)
        reply = SimpleNamespace(**data)
        return self.authority.confirm_assignment(reply)

    def ready(self):
        self.assertTrue(self.confirm())
        self.authority.activate_goal(self.execution, self.fields['goal_id'])
        self.authority.set_lease('pick-1', 'a' * 32, 7, 101.5)

    def test_no_state_and_restart_fail_closed(self):
        # [핵심] 기록이 없거나 재시작한 상태를 정상 권한으로 추측하지 않아야 합니다.
        self.assertEqual(self.authority.decide(self.fields), 'ASSIGNMENT_STALE')
        self.ready()
        restarted = RuntimeAuthority('robot_1', clock=lambda: self.now)
        self.assertNotEqual(restarted.session_id, self.authority.session_id)
        self.assertEqual(restarted.decide(self.fields), 'ASSIGNMENT_STALE')

    def test_current_goal_allowed_and_duration_capped(self):
        self.ready()
        self.assertEqual(self.authority.decide(self.fields), '')
        self.assertEqual(self.authority.remaining(self.fields), 1.5)
        self.now = 101.4
        self.assertAlmostEqual(self.authority.remaining(self.fields), 0.1)

    def test_expiry_and_no_late_renewal(self):
        self.ready()
        self.now = 101.5
        self.assertEqual(self.authority.decide(self.fields), 'LEASE_EXPIRED')
        with self.assertRaises(ValueError):
            self.authority.set_lease('pick-1', 'a' * 32, 7, 101.9)

    def test_revoked_generation_cannot_be_restored(self):
        self.ready()
        self.assertFalse(self.authority.revoke('pick-1', 'b' * 32, 7))
        self.assertTrue(self.authority.revoke('pick-1', 'a' * 32, 7))
        self.assertEqual(self.authority.decide(self.fields), 'LEASE_REVOKED')
        with self.assertRaises(ValueError):
            self.authority.set_lease('pick-1', 'a' * 32, 7, 101.4)

    def test_old_lease_generation_and_wrong_goal_rejected(self):
        # [핵심] 다른 로봇·실행·부모 Goal·이전 lease 세대의 명령을 현재 명령처럼 허용하지 않아야 합니다.
        self.ready()
        for field, value, reason in (
                ('lease_generation', 6, 'LEASE_REVOKED'),
                ('goal_id', 'b' * 32, 'GOAL_MISMATCH'),
                ('execution_id', 'previous-pick', 'GOAL_MISMATCH'),
                ('robot_id', 'robot_2', 'AUTHORITY_INVALID')):
            with self.subTest(field=field):
                self.assertEqual(self.authority.decide(dict(self.fields, **{field: value})), reason)
        self.authority.finish_goal('pick-1', 'a' * 32)
        self.assertEqual(self.authority.decide(self.fields), 'GOAL_MISMATCH')

    def test_assignment_expiry_and_reassignment(self):
        self.ready()
        self.now = 102.0
        self.assertEqual(self.authority.decide(self.fields), 'ASSIGNMENT_STALE')
        self.assertTrue(self.confirm('check-2', generation=4))
        self.assertEqual(self.authority.decide(self.fields), 'ASSIGNMENT_STALE')
        with self.assertRaises(ValueError):
            self.authority.activate_goal(self.execution, 'b' * 32)

    def test_late_duplicate_wrong_session_and_generation_responses(self):
        self.assertFalse(self.confirm(a_session_id='old-a-session'))
        self.assertFalse(self.confirm(assignment_generation=2))
        self.authority.begin_assignment_check('delayed', 'assignment-1', 3)
        self.now = 103.0
        reply = SimpleNamespace(
            request_id='delayed', a_session_id=self.authority.session_id,
            d_session_id='d-session-1', assignment_generation=3,
            sequence=10, valid=True, validity_s=2.0)
        self.assertFalse(self.authority.confirm_assignment(reply))
        self.assertFalse(self.authority.confirm_assignment(reply))

    def test_first_send_deadline_and_duplicate_does_not_extend(self):
        # [핵심] 허용 시간은 최초 요청부터 셉니다. 중복 응답이나 재전송으로 늘어나면 안 됩니다.
        self.authority.begin_assignment_check('slow', 'assignment-1', 3)
        self.now = 101.0
        self.assertTrue(self.confirm('slow'))
        self.assertEqual(self.authority.assignment_deadline, 102.0)
        # Even a higher D sequence cannot turn a consumed request into a renewal.
        self.assertFalse(self.confirm('slow', sequence=99))
        self.assertEqual(self.authority.assignment_deadline, 102.0)

    def test_previous_assignment_reply_cannot_restore_old_generation(self):
        self.authority.begin_assignment_check('old', 'assignment-1', 3)
        self.assertTrue(self.confirm('new', generation=4))
        reply = SimpleNamespace(
            request_id='old', a_session_id=self.authority.session_id,
            d_session_id='d-session-1', assignment_generation=3,
            sequence=99, valid=True, validity_s=2.0)
        self.assertFalse(self.authority.confirm_assignment(reply))
        self.assertEqual(self.authority.assignment, ('assignment-1', 4))

    def test_d_restart_invalidates_active_goal(self):
        self.ready()
        self.assertTrue(self.confirm('new-session', d_session_id='d-session-2', sequence=1))
        self.assertEqual(self.authority.decide(self.fields), 'LEASE_REVOKED')

    def test_local_retract_requires_active_local_task_and_assignment(self):
        # [핵심] 일반 팔 후퇴는 현재 RETRACT_ARM 작업과 배정이 있어야 하며 C 조작 권한과 구별합니다.
        self.assertTrue(self.confirm())
        self.authority.activate_goal(self.execution, local_task_kind=6)
        fields = dict(self.fields, authority=2, goal_id='', lease_generation=0,
                      local_task_kind=6)
        self.assertEqual(self.authority.decide(fields), '')
        self.assertEqual(self.authority.decide(dict(fields, command_source=1)),
                         'AUTHORITY_INVALID')
        self.authority.finish_goal('pick-1')
        self.assertEqual(self.authority.decide(fields), 'GOAL_MISMATCH')

    def test_invalid_durations_and_unconfirmed_goals(self):
        for duration in (float('nan'), float('inf'), 0.0, -1.0):
            with self.subTest(duration=duration):
                other = RuntimeAuthority('robot_1', clock=lambda: self.now)
                other.begin_assignment_check('x', 'assignment-1', 3)
                self.assertFalse(other.confirm_assignment(SimpleNamespace(
                    request_id='x', a_session_id=other.session_id,
                    d_session_id='d', assignment_generation=3, sequence=1,
                    valid=True, validity_s=duration)))
        with self.assertRaises(ValueError):
            self.authority.activate_goal(self.execution, 'a' * 32)
        self.ready()
        for deadline in (float('nan'), float('inf'), 103.0):
            with self.assertRaises(ValueError):
                self.authority.set_lease('pick-1', 'a' * 32, 7, deadline)


if __name__ == '__main__':
    unittest.main()
