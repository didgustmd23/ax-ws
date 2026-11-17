"""B3 실제 출력과 A4/D2 계약 메시지를 내 D3 콜백에 전달합니다. ROS 노드는 시작하지 않습니다."""
from copy import deepcopy
import json
from pathlib import Path
import sys
import tempfile
from types import MethodType, SimpleNamespace
import unittest
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[2]
for package in ('ax_environment', 'ax_corridor_manager'):
    sys.path.insert(0, str(ROOT / 'src' / package))
sys.path.insert(0, str(ROOT / 'tools/tests'))
from ax_corridor_manager.manager import CorridorManager, OUTSIDE, UNKNOWN
from ax_corridor_manager.node import CorridorNode
from ax_interfaces.msg import ActuationState, GroupTaskList, GroupTaskStatus
from rclpy.time import Time
from test_team_consumers_offline import B3B4ConsumerTests


class D3TeamInputTests(unittest.TestCase):
    """수신 형식·유효기한·보류를 확인하며 실제 상대 발행/주행 검증과 구분합니다."""

    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.now = 100.0
        catalog = json.loads((ROOT / 'src/ax_environment/config/ax_warehouse_regions.json').read_text())
        corridors = [row['id'] for row in catalog['regions'] if row['kind'] == 'corridor']
        self.manager = CorridorManager(Path(self.folder.name) / 'ledger.db', corridors,
                                       ['robot_1', 'robot_2'], clock=lambda: self.now)
        self.addCleanup(self.manager.close)
        self.node = SimpleNamespace(manager=self.manager, sources={}, retired={}, io_failed=False,
                                    publish_state=Mock(), get_clock=lambda: SimpleNamespace(
                                        now=lambda: Time(seconds=self.now)))
        for name in ('remaining', 'ordered'):
            setattr(self.node, name, MethodType(getattr(CorridorNode, name), self.node))

    def meta(self, meta):
        """시험 시각의 명시적 정상 메타를 만들며 실제 상대의 검증 완료를 뜻하지 않습니다."""
        meta.header.stamp = Time(seconds=self.now).to_msg()
        meta.valid = True
        meta.source_session_id = 'test-peer'
        meta.revision = 1

    def test_real_b3_message_is_consumed_without_enabling_policy(self):
        """B3의 실제 생성 함수를 거친 미승인 출력은 D3에서도 불명·발급 보류입니다."""
        message = B3B4ConsumerTests().produce(False)
        CorridorNode.on_observation(self.node, message, 'robot_1')
        states = [self.manager._observation('robot_1', corridor)[0]
                  for corridor in self.manager.corridor_ids]
        self.assertEqual(set(states), {UNKNOWN})
        denied = self.manager.request('corridor_1', 'robot_1', 'pending-policy')
        self.assertFalse(denied['granted'])
        self.assertEqual(denied['reason_code'], 'REGIONS_NOT_APPROVED')

    def test_valid_b3_fixture_reaches_all_corridors(self):
        """임시 승인 사본의 실제 B3 메시지는 D3 관측까지 전달되지만 권한은 만들지 않습니다."""
        message = B3B4ConsumerTests().produce(True)
        CorridorNode.on_observation(self.node, message, 'robot_1')
        self.assertTrue(all(self.manager._observation('robot_1', corridor)[0] == OUTSIDE
                            for corridor in self.manager.corridor_ids))
        stale = deepcopy(message)
        stale.meta.header.stamp.sec = 90
        CorridorNode.on_observation(self.node, stale, 'robot_1')
        self.assertTrue(all(self.manager._observation('robot_1', corridor)[0] == UNKNOWN
                            for corridor in self.manager.corridor_ids))

    def test_stop_requested_unknown_or_speed_excess_never_confirm_stop(self):
        """A4 계약상 실제 STOPPED와 낮은 속도가 함께 있어야 정지 확인으로 받습니다."""
        message = ActuationState(robot_id='robot_1', state=ActuationState.STOPPED)
        self.meta(message.meta)
        CorridorNode.on_stop(self.node, message, 'robot_1')
        self.assertTrue(self.manager.stops['robot_1'][0])
        for state, speed in ((message.STOP_REQUESTED, 0.0), (message.UNKNOWN, 0.0),
                             (message.STOPPED, 0.03)):
            message.state, message.angular_speed = state, speed
            CorridorNode.on_stop(self.node, message, 'robot_1')
            self.assertFalse(self.manager.stops['robot_1'][0])

    def test_d2_assignment_hold_and_expiry_do_not_create_authority(self):
        """현재 배정만 반영하고 보류·연기·만료된 작업에서 자격을 제거합니다."""
        message = GroupTaskList()
        self.meta(message.meta)
        task = GroupTaskStatus(state=GroupTaskStatus.RUNNING, assignment_generation=3,
                               manipulator_robot_id='robot_1', transporter_robot_id='robot_2')
        message.tasks = [task]
        CorridorNode.on_tasks(self.node, message)
        self.assertEqual(set(self.manager.assignments), {'robot_1', 'robot_2'})
        task.hold_ids = ['hold-1']
        message.tasks = [task]
        CorridorNode.on_tasks(self.node, message)
        self.assertFalse(self.manager.assignments)
        task.hold_ids, task.deferred = [], True
        message.tasks = [task]
        CorridorNode.on_tasks(self.node, message)
        self.assertFalse(self.manager.assignments)
        task.deferred = False
        message.tasks = [task]
        message.meta.header.stamp.sec = 90
        CorridorNode.on_tasks(self.node, message)
        self.assertTrue(all(deadline <= self.now for deadline in self.manager.assignments.values()))
        self.assertFalse(self.manager.grant_policy_approved)
