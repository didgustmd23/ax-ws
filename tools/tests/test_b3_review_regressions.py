"""Review reproductions: invalid B2 session must not retire a healthy session.

These preserve the fixes for the session handling defects found during review.
No production source is modified by these tests.
"""
# [검사 범위] B3 세션 복구 회귀 검사: 잘못된 새 메시지가 정상 세션을 영구히 막지 않게 합니다.
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

from ax_interfaces.msg import LocalizationHealth

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/ax_environment'))
from ax_environment.corridor_observer import CorridorObserver  # noqa: E402


def health(session, valid=True, revision=1):
    message = LocalizationHealth()
    message.robot_id = 'robot_2'
    message.state = message.OK
    message.meta.source_session_id = session
    message.meta.valid = valid
    message.meta.revision = revision
    message.meta.header.stamp.sec = 100
    return message


class SessionValidationReviewTests(unittest.TestCase):
    """Exercise the real callback without running ROS or moving a robot."""

    def observer(self):
        return SimpleNamespace(robot_id='robot_2', retired_sessions=set(),
                               health_session=None, health_revision=0,
                               health=None, health_received=None,
                               future_tolerance_s=1.0, health_timeout_s=1.0,
                               get_clock=lambda: SimpleNamespace(
                                   now=lambda: SimpleNamespace(nanoseconds=100_000_000_000)))

    def test_empty_session_must_not_retire_valid_publisher(self):
        # [핵심] 세션 ID가 비어 있는 메시지 하나 때문에 정상 B2 세션을 폐기하면 안 됩니다.
        observer = self.observer()
        CorridorObserver.on_health(observer, health('b2-live', revision=10))
        CorridorObserver.on_health(observer, health('', revision=1))
        CorridorObserver.on_health(observer, health('b2-live', revision=11))
        self.assertEqual(observer.health_session, 'b2-live')
        self.assertEqual(observer.health_revision, 11)

    def test_zero_revision_session_must_not_retire_valid_publisher(self):
        observer = self.observer()
        CorridorObserver.on_health(observer, health('b2-live', revision=10))
        CorridorObserver.on_health(observer, health('uninitialized-session', revision=0))
        CorridorObserver.on_health(observer, health('b2-live', revision=11))
        self.assertEqual(observer.health_session, 'b2-live')
        self.assertEqual(observer.health_revision, 11)

    def test_invalid_new_session_blocks_observation_but_allows_live_session_recovery(self):
        observer = self.observer()
        CorridorObserver.on_health(observer, health('b2-live', revision=10))
        invalid = health('b2-not-ready', valid=False)
        CorridorObserver.on_health(observer, invalid)
        self.assertIs(observer.health, invalid)
        self.assertEqual(observer.health_session, 'b2-live')
        self.assertNotIn('b2-live', observer.retired_sessions)
        CorridorObserver.on_health(observer, health('b2-live', revision=11))
        self.assertTrue(observer.health.meta.valid)
        self.assertEqual(observer.health_revision, 11)

    def test_valid_new_session_retires_old_and_old_cannot_replace_it(self):
        # [핵심] 정상 새 세션으로 바뀐 뒤에는 예전 세션의 늦은 메시지가 현재 상태를 덮어쓰지 못해야 합니다.
        observer = self.observer()
        CorridorObserver.on_health(observer, health('b2-old', revision=10))
        CorridorObserver.on_health(observer, health('b2-new', revision=1))
        CorridorObserver.on_health(observer, health('b2-old', revision=11))
        self.assertEqual(observer.health_session, 'b2-new')
        self.assertEqual(observer.health_revision, 1)

    def test_invalid_same_session_is_not_ignored(self):
        observer = self.observer()
        CorridorObserver.on_health(observer, health('b2-live', revision=10))
        CorridorObserver.on_health(observer, health('b2-live', valid=False, revision=11))
        self.assertFalse(observer.health.meta.valid)
        self.assertEqual(observer.health_revision, 11)


if __name__ == '__main__':
    unittest.main()
