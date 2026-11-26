"""원본 D2 개발 발행자에서 내 D3로 실제 DDS 작업 목록을 전달합니다. 운영 실행은 아닙니다."""
from pathlib import Path
import sys
import tempfile
import time
import unittest

import rclpy
from rclpy.executors import SingleThreadedExecutor

ROOT = Path(__file__).resolve().parents[2]
for package in ('ax_corridor_manager', 'ax_environment', 'ax_fleet_manager'):
    sys.path.insert(0, str(ROOT / 'src' / package))
sys.path.insert(0, str(ROOT / 'tools/tests'))
import ros_test_domain
from ax_corridor_manager.node import CorridorNode
from ax_fleet_manager.group_node import GroupTaskDevPublisher

_DOMAIN_STATE = None


def setUpModule():
    """기존 김성민 시험 domain만 잠금 예약하고 서비스 충돌을 확인합니다."""
    global _DOMAIN_STATE
    _DOMAIN_STATE = ros_test_domain.enter_module_domain(
        ros_test_domain.A3_DOMAINS, r'^/ax/(fleet/corridor|robot_)')


def tearDownModule():
    """시험 후 domain 설정과 잠금을 복구합니다."""
    ros_test_domain.leave_module_domain(_DOMAIN_STATE)


class D2SelectedTopicTests(unittest.TestCase):
    """상대 소스·토픽·안전 설정을 바꾸지 않고 내 입력 경로만 바꿔 확인합니다."""

    def test_real_dev_publisher_reaches_d3_without_operational_grant(self):
        """지연 구독·작업 수신·무수신 만료를 확인하되 개발 자료로 통로를 열지 않습니다."""
        with tempfile.TemporaryDirectory() as folder:
            fixture = ROOT / 'src/ax_fleet_manager/config/dev_fixture_transfer.json'
            topic = '/ax/dev/d2_mock/fleet_tasks'
            rclpy.init(args=['--ros-args', '-p', f'fixture_file:={fixture}',
                             '-p', f'database:={folder}/corridors.db',
                             '-p', f'fleet_tasks_topic:={topic}', '-p', 'use_sim_time:=false'])
            source = target = executor = None
            try:
                # D2가 먼저 발행한다. 원본 STATE transient_local QoS로 늦게 붙은 D3도 받는다.
                source = GroupTaskDevPublisher()
                target = CorridorNode()
                executor = SingleThreadedExecutor()
                executor.add_node(source)
                executor.add_node(target)
                end = time.monotonic() + 6.0
                while time.monotonic() < end and not target.manager.assignments:
                    executor.spin_once(timeout_sec=0.05)
                self.assertEqual(source.publisher.topic_name, topic)
                self.assertEqual(target.fleet_tasks_topic, topic)
                self.assertEqual(set(target.manager.assignments), {'robot_1', 'robot_2'})
                self.assertFalse(target.manager.grant_policy_approved)
                self.assertFalse(target.manager.request('corridor_1', 'robot_1', 'dev-entry')['granted'])
                self.assertFalse(target.get_publishers_info_by_topic('/ax/fleet/tasks'))
                # 발행자가 사라지면 저장된 로봇 이름이 남아도 그 기한을 권한으로 재사용하지 않는다.
                executor.remove_node(source)
                source.destroy_node()
                source = None
                end = time.monotonic() + 3.2
                while time.monotonic() < end:
                    executor.spin_once(timeout_sec=0.05)
                self.assertTrue(all(deadline <= target.manager.clock()
                                    for deadline in target.manager.assignments.values()))
            finally:
                if executor is not None:
                    executor.shutdown()
                for node in (target, source):
                    if node is not None:
                        node.destroy_node()
                rclpy.try_shutdown()
