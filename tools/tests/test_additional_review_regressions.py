"""Guard reassignment rejection and B2 session timestamp validation fixes."""
# [검사 범위] 재배정 거절과 오래된 B2 세션에서 발견했던 오류가 다시 생기지 않는지 확인합니다.
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

from ax_interfaces.msg import LocalizationHealth

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/ax_task_executor'))
sys.path.insert(0, str(ROOT / 'src/ax_environment'))
from ax_environment.corridor_observer import CorridorObserver  # noqa: E402
from ax_task_executor.runtime_authority import RuntimeAuthority  # noqa: E402


def observer_at(now=100):
    return SimpleNamespace(robot_id='robot_2', retired_sessions=set(),
                           health_session=None, health_revision=0,
                           health=None, health_received=None,
                           future_tolerance_s=1.0, health_timeout_s=1.0,
                           get_clock=lambda: SimpleNamespace(
                               now=lambda: SimpleNamespace(nanoseconds=int(now * 1e9))))


def b2_message(session, revision, stamp):
    message = LocalizationHealth()
    message.robot_id = 'robot_2'
    message.state = message.OK
    message.meta.valid = True
    message.meta.source_session_id = session
    message.meta.revision = revision
    message.meta.header.stamp.sec = stamp
    message.position_std_m = 0.01
    message.yaw_std_rad = 0.01
    return message


class AdditionalReviewTests(unittest.TestCase):
    """Use the real decision/callback code with controlled peer messages."""

    def test_current_d4_reassignment_rejection_must_revoke_old_permission(self):
        # [핵심] 현재 D4가 재배정으로 거절하면 옛 세대의 권한을 즉시 무효화해야 합니다.
        authority = RuntimeAuthority('robot_1', lambda: 100.0)
        self.addCleanup(authority.close)
        authority.begin_assignment_check('initial', 'assignment', 3)
        authority.confirm_assignment(SimpleNamespace(
            request_id='initial', a_session_id=authority.session_id, d_session_id='d4',
            assignment_generation=3, sequence=1, valid=True, validity_s=2.0))
        authority.activate_goal(SimpleNamespace(
            execution_id='pick', assignment_id='assignment', assignment_generation=3),
            'a' * 32)
        authority.set_lease('pick', 'a' * 32, 1, 101.0)
        fields = dict(authority=1, execution_id='pick', robot_id='robot_1',
                      goal_id='a' * 32, lease_generation=1, local_task_kind=0, command_source=2)
        self.assertEqual(authority.decide(fields), '')
        authority.begin_assignment_check('refresh', 'assignment', 3)
        # Same current D4 explicitly rejects because its assignment is now generation 4.
        authority.confirm_assignment(SimpleNamespace(
            request_id='refresh', a_session_id=authority.session_id, d_session_id='d4',
            assignment_generation=4, sequence=2, valid=False, validity_s=0.0))
        self.assertNotEqual(authority.decide(fields), '',
                            'current D4 rejection must block the old generation immediately')

    def test_stale_new_session_must_not_prevent_live_publisher_recovery(self):
        # [핵심] 오래되거나 잘못된 시각의 새 세션 뒤에도 정상 세션의 새 메시지로 복구할 수 있어야 합니다.
        for stamp in (0, 50, 103):
            with self.subTest(stamp=stamp):
                observer = observer_at()
                CorridorObserver.on_health(observer, b2_message('b2-live', 10, 100))
                self.assertEqual(CorridorObserver.health_problem(observer, 100.0), (None, False))
                CorridorObserver.on_health(observer, b2_message('bad-time-session', 1, stamp))
                self.assertNotIn('b2-live', observer.retired_sessions)
                self.assertEqual(CorridorObserver.health_problem(observer, 100.0)[0], 'HEALTH_STALE')
                CorridorObserver.on_health(observer, b2_message('b2-live', 11, 100))
                self.assertEqual(CorridorObserver.health_problem(observer, 100.0), (None, False))

    def test_fresh_new_session_within_future_tolerance_can_replace_live_session(self):
        observer = observer_at()
        CorridorObserver.on_health(observer, b2_message('old', 10, 100))
        CorridorObserver.on_health(observer, b2_message('new', 1, 101))
        self.assertEqual(observer.health_session, 'new')
        self.assertIn('old', observer.retired_sessions)
        self.assertEqual(CorridorObserver.health_problem(observer, 100.0), (None, False))

    def test_zero_clock_does_not_retire_previous_session(self):
        observer = observer_at()
        CorridorObserver.on_health(observer, b2_message('live', 1, 100))
        observer.get_clock = observer_at(0).get_clock
        CorridorObserver.on_health(observer, b2_message('new', 1, 100))
        self.assertEqual(observer.health_session, 'live')
        self.assertNotIn('live', observer.retired_sessions)

    def test_unrelated_or_out_of_order_d4_rejections_do_not_clear_current_assignment(self):
        # [핵심] 현재 D4의 거절과 무관한·오래된 응답을 구별합니다. 엉뚱한 응답으로 정상 권한을 지우면 안 됩니다.
        cases = ({'sequence': 0}, {'a_session_id': 'old-a-session'},
                 {'request_id': 'unknown'}, {'valid': True, 'validity_s': 2.0})
        for changes in cases:
            with self.subTest(changes=changes):
                authority = RuntimeAuthority('robot_1', lambda: 100.0)
                self.addCleanup(authority.close)
                authority.begin_assignment_check('initial', 'assignment', 3)
                authority.confirm_assignment(SimpleNamespace(
                    request_id='initial', a_session_id=authority.session_id, d_session_id='d4',
                    assignment_generation=3, sequence=1, valid=True, validity_s=2.0))
                authority.begin_assignment_check('refresh', 'assignment', 3)
                fields = dict(request_id='refresh', a_session_id=authority.session_id,
                              d_session_id='d4', assignment_generation=4, sequence=2,
                              valid=False, validity_s=0.0)
                fields.update(changes)
                self.assertFalse(authority.confirm_assignment(SimpleNamespace(**fields)))
                self.assertEqual(authority.assignment, ('assignment', 3))

    def test_rejection_discards_other_pending_positive_responses(self):
        # [핵심] 거절 뒤에 늦게 온 허용 응답이 권한을 되살리지 못하게 대기 요청도 정리해야 합니다.
        authority = RuntimeAuthority('robot_1', lambda: 100.0)
        self.addCleanup(authority.close)
        authority.begin_assignment_check('negative', 'assignment', 3)
        authority.begin_assignment_check('late-positive', 'assignment', 3)
        self.assertFalse(authority.confirm_assignment(SimpleNamespace(
            request_id='negative', a_session_id=authority.session_id, d_session_id='d4',
            assignment_generation=4, sequence=2, valid=False, validity_s=0.0)))
        self.assertFalse(authority.confirm_assignment(SimpleNamespace(
            request_id='late-positive', a_session_id=authority.session_id, d_session_id='d4',
            assignment_generation=3, sequence=3, valid=True, validity_s=2.0)))
        self.assertIsNone(authority.assignment)


if __name__ == '__main__':
    unittest.main()
