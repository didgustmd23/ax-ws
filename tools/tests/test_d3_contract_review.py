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
    """기존 원장 시험과 owner 재확인·다중 통로 제한을 검사합니다."""

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

    def test_robot_cannot_hold_two_corridors(self):
        """같은 로봇의 두 번째 통로 허가는 첫 통로 해제 후에만 발급합니다."""
        self.manager.close()
        self.path = Path(self.folder.name) / 'two-corridors.db'
        self.manager = core.CorridorManager(
            self.path, ['first', 'second'], ['robot_1', 'robot_2'],
            enabled=True, clock=lambda: self.now)

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
        self.assertFalse(self.manager.request('second', 'robot_1', 'blocked')['granted'])
        ready()
        self.assertTrue(self.manager.release('first', 'robot_1', first['grant_id'], 'exit')['accepted'])
        self.assertTrue(self.manager.request('second', 'robot_1', 'next')['granted'])


class StatePublicationTests(unittest.TestCase):
    """ROS 그래프 없이 실제 게시 함수의 변경 발행과 1Hz 제한을 확인합니다."""

    def test_unchanged_state_waits_one_second_but_change_is_immediate(self):
        """같은 상태를 5Hz로 내보내지 않고 revision 변경은 바로 발행합니다."""
        from rclpy.time import Time

        manager = Mock(enabled=True)
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
