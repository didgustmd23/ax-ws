"""통로 허가 계약과 실제 상태 게시 함수의 회귀 오류를 검사합니다."""
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src/ax_environment'))
import test_d3_corridor_manager as core
from ax_corridor_manager.node import CorridorNode


class CorridorContractTests(core.CorridorTests):
    """합의 가정의 개발 구현에서 원장과 owner 재확인 및 다중 통로 허가를 검사합니다."""

    def test_owner_recheck_requires_fresh_assignment(self):
        """기존 허가가 있어도 새 요청의 만료된 배정을 거절합니다."""
        self.ready()
        grant = self.manager.request('corridor', 'robot_1', 'entry')
        self.manager.assign([])
        denied = self.manager.request('corridor', 'robot_1', 'recheck')
        self.assertFalse(denied['granted'])
        self.assertEqual(denied['reason_code'], 'ASSIGNMENT_STALE')
        self.assertEqual(self.state()['grant_id'], grant['grant_id'])
        self.assertEqual(self.manager.request('corridor', 'robot_1', 'entry'), grant)

    def test_owner_recheck_requires_stop_and_known_corridor(self):
        """정지 만료와 관측 불명 상태에서 재진입을 허용하지 않습니다."""
        self.ready()
        grant = self.manager.request('corridor', 'robot_1', 'entry')
        self.now += 0.3
        stopped = self.manager.request('corridor', 'robot_1', 'stop-expired')
        self.assertEqual(stopped['reason_code'], 'STOP_NOT_CONFIRMED')
        self.ready(second=core.UNKNOWN)
        blocked = self.manager.request('corridor', 'robot_1', 'unknown')
        self.assertEqual(blocked['reason_code'], 'CORRIDOR_BLOCKED')
        self.assertEqual(self.state()['grant_id'], grant['grant_id'])

    def test_other_observation_expiry_blocks_release_until_recovery(self):
        """다른 로봇이 불명일 때 owner를 보존하고 전체 관측 회복 후 해제합니다."""
        self.ready()
        grant = self.manager.request('corridor', 'robot_1', 'entry')
        self.now += 1.0
        self.manager.observe('robot_1', {'corridor': (core.OUTSIDE, True)})
        denied = self.manager.release('corridor', 'robot_1', grant['grant_id'], 'stale-peer')
        self.assertEqual(denied['reason_code'], 'CORRIDOR_BLOCKED')
        self.assertEqual(self.state()['owner'], 'robot_1')
        self.ready()
        self.assertTrue(self.manager.release(
            'corridor', 'robot_1', grant['grant_id'], 'recovered')['accepted'])

    def test_robot_can_hold_different_corridors_but_each_has_one_owner(self):
        """미합의 로봇당 제한은 적용하지 않고 각 통로의 단일 소유권은 유지합니다."""
        self.manager.close()
        self.path = Path(self.folder.name) / 'two-corridors.db'
        self.manager = core.CorridorManager(
            self.path, ['first', 'second'], ['robot_1', 'robot_2'],
            enabled=True, clock=lambda: self.now, grant_policy_approved=True)

        def ready():
            # 두 통로의 모든 로봇에 같은 시험 시각의 신선한 관측을 제공합니다.
            for robot in self.manager.robot_ids:
                self.manager.observe(robot, {name: (core.OUTSIDE, True)
                                             for name in self.manager.corridor_ids})
                self.manager.stopped(robot, True)
            self.manager.assign(self.manager.robot_ids)

        ready()
        first = self.manager.request('first', 'robot_1', 'first')
        self.assertTrue(first['granted'])
        second = self.manager.request('second', 'robot_1', 'second')
        self.assertTrue(second['granted'])
        self.assertFalse(self.manager.request('second', 'robot_2', 'other-robot')['granted'])
        ready()
        self.assertTrue(self.manager.release('first', 'robot_1', first['grant_id'], 'exit')['accepted'])
        rows = {row['id']: row for row in self.manager.snapshot()[1]}
        self.assertEqual(rows['first']['owner'], '')
        self.assertEqual(rows['second']['owner'], 'robot_1')
        self.assertEqual(rows['second']['grant_id'], second['grant_id'])


class PendingPolicyTests(unittest.TestCase):
    """미합의 기본값에서 새 허가 차단과 기존 원장 보존을 확인합니다."""

    def test_default_pending_policy_never_creates_grants(self):
        # 핵심: 영역·관측·배정·정지가 정상이어도 정책 미합의 기본값이면 발급을 거절해야 합니다.
        """영역과 배정 및 정지가 유효해도 정책 미합의면 허가를 발급하지 않습니다."""
        with core.tempfile.TemporaryDirectory() as directory:
            manager = core.CorridorManager(
                Path(directory) / 'pending.db', ['corridor'], ['robot_1'],
                enabled=True, clock=lambda: 100.0)
            self.addCleanup(manager.close)
            manager.observe('robot_1', {'corridor': (core.OUTSIDE, True)})
            manager.assign(['robot_1'])
            manager.stopped('robot_1', True)
            result = manager.request('corridor', 'robot_1', 'pending')
            self.assertFalse(result['granted'])
            self.assertEqual(result['reason_code'], 'AUTHORITY_INVALID')
            row = manager.snapshot()[1][0]
            self.assertEqual(row['owner'], '')
            self.assertEqual(row['state'], core.BLOCKED_UNKNOWN)

    def test_suspension_preserves_old_grant_and_allows_confirmed_release(self):
        # 핵심: 기존 허가를 가진 원장을 재시작해도 보존하고, 이탈 확인 뒤 정상 해제할 수 있어야 합니다.
        """재시작 보류는 기존 허가를 지우지 않으며 새 확인과 정상 해제를 구분합니다."""
        with core.tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'existing.db'
            manager = core.CorridorManager(path, ['corridor'], ['robot_1'], enabled=True,
                                           clock=lambda: 100.0, grant_policy_approved=True)
            try:
                manager.observe('robot_1', {'corridor': (core.OUTSIDE, True)})
                manager.assign(['robot_1'])
                manager.stopped('robot_1', True)
                grant = manager.request('corridor', 'robot_1', 'old')
                self.assertTrue(grant['granted'])
            finally:
                manager.close()
            pending = core.CorridorManager(path, ['corridor'], ['robot_1'], enabled=True,
                                           clock=lambda: 100.0)
            try:
                row = pending.snapshot()[1][0]
                self.assertEqual(row['owner'], 'robot_1')
                self.assertEqual(row['grant_id'], grant['grant_id'])
                self.assertFalse(pending.request('corridor', 'robot_1', 'new')['granted'])
                # 동일 요청의 과거 응답은 재현하지만 현재 상태는 계속 무효·불명입니다.
                self.assertEqual(pending.request('corridor', 'robot_1', 'old'), grant)
                pending.observe('robot_1', {'corridor': (core.OUTSIDE, True)})
                self.assertTrue(pending.release(
                    'corridor', 'robot_1', grant['grant_id'], 'exit')['accepted'])
                self.assertEqual(pending.snapshot()[1][0]['owner'], '')
            finally:
                pending.close()


class StatePublicationTests(unittest.TestCase):
    """ROS 그래프 없이 실제 게시 함수의 변경 발행과 1Hz 제한을 확인합니다."""

    def test_pending_policy_publishes_invalid_blocked_state(self):
        # 핵심: 내부 보류뿐 아니라 상대 파트가 받는 메시지도 무효·불명 상태여야 합니다.
        """영역이 승인돼도 미합의 정책을 유효한 진입 상태로 게시하지 않습니다."""
        from rclpy.time import Time

        with core.tempfile.TemporaryDirectory() as directory:
            manager = core.CorridorManager(
                Path(directory) / 'state.db', ['corridor'], ['robot_1'],
                enabled=True, clock=lambda: 100.0)
            try:
                manager.observe('robot_1', {'corridor': (core.OUTSIDE, True)})
                node = SimpleNamespace(
                    manager=manager, io_failed=False, session='pending-test',
                    get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=100)),
                    publisher=Mock(), last_state_key=None, last_state_time=float('-inf'))
                CorridorNode.publish_state(node)
                message = node.publisher.publish.call_args.args[0]
                self.assertFalse(message.meta.valid)
                self.assertEqual(message.meta.validity_reason, 'AUTHORITY_INVALID')
                self.assertEqual(message.corridors[0].state, core.BLOCKED_UNKNOWN)
                self.assertEqual(message.corridors[0].active_grant_id, '')
            finally:
                manager.close()

    def test_unchanged_state_waits_one_second_but_change_is_immediate(self):
        """같은 상태를 5Hz로 내보내지 않고 revision 변경은 바로 발행합니다."""
        from rclpy.time import Time

        manager = Mock(enabled=True, grant_policy_approved=True)
        manager.snapshot.return_value = (1, [])
        clock = Mock()
        clock.now.return_value = Time(seconds=10)
        node = SimpleNamespace(manager=manager, io_failed=False, session='test',
                               get_clock=lambda: clock, publisher=Mock(),
                               last_state_key=None, last_state_time=float('-inf'))
        with patch('ax_corridor_manager.node.boottime') as steady:
            # 핵심: 0.2초 만료 감시가 그대로 5Hz 출력이 되면 실패합니다.
            for instant in (100, 100.2, 100.4, 100.6, 100.8):
                steady.return_value = instant
                CorridorNode.publish_state(node)
            self.assertEqual(node.publisher.publish.call_count, 1)
            steady.return_value = 101
            CorridorNode.publish_state(node)
            self.assertEqual(node.publisher.publish.call_count, 2)
            manager.snapshot.return_value = (2, [])
            steady.return_value = 101.01
            CorridorNode.publish_state(node)
            self.assertEqual(node.publisher.publish.call_count, 3)
            node.io_failed = True
            CorridorNode.publish_state(node)
            self.assertEqual(node.publisher.publish.call_count, 4)
            self.assertFalse(node.publisher.publish.call_args.args[0].meta.valid)


if __name__ == '__main__':
    unittest.main()
