"""승인 사본·합성 B2 입력으로 B3 OUTSIDE를 검증합니다. 운영 승인이나 실측 위치는 아닙니다."""
import json
import os
from pathlib import Path
import sys
import tempfile
import time
import unittest

import rclpy
from ax_interfaces.msg import CorridorObservationList, LocalizationHealth
from geometry_msgs.msg import TransformStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/ax_environment'))
sys.path.insert(0, str(ROOT / 'tools/tests'))
import ros_test_domain
from ax_environment.corridor_observer import CorridorObserver, STATE_QOS

_DOMAIN_STATE = None


def setUpModule():
    """기존 B3 시험 범위에서만 실제 ROS 발행을 확인합니다."""
    global _DOMAIN_STATE
    _DOMAIN_STATE = ros_test_domain.enter_module_domain(
        ros_test_domain.CORRIDOR_OBSERVER_DOMAINS, r'^/ax/robot_2/')


def tearDownModule():
    """시험 domain을 돌려줍니다."""
    ros_test_domain.leave_module_domain(_DOMAIN_STATE)


class OutsideEvidenceTests(unittest.TestCase):
    """생산 설정 파일은 건드리지 않고 실제 B3 메시지 발행 로직만 검사합니다."""

    def test_outside_and_position_valid_are_published_on_test_copy(self):
        """현재 좌표계에서 시험 원점이 통로 밖일 때 OUTSIDE·position_valid를 실제 ROS로 받습니다."""
        original_path = ROOT / 'src/ax_environment/config/ax_warehouse_regions.json'
        original = original_path.read_bytes()
        with tempfile.TemporaryDirectory() as folder:
            catalog = json.loads(original)
            catalog['approved'] = True
            path = Path(folder) / 'regions.json'
            path.write_text(json.dumps(catalog))
            rclpy.init(args=['--ros-args', '-r', '__ns:=/ax/robot_2', '-p', 'robot_id:=robot_2',
                             '-p', 'robot_model:=AX_2', '-p', f'regions_file:={path}',
                             '-p', 'use_sim_time:=false',
                             '-p', f'description_config_dir:={ROOT}/src/ax_description/config'])
            observer = peer = executor = None
            try:
                observer = CorridorObserver()
                peer = Node('b3_outside_fixture')
                publisher = peer.create_publisher(LocalizationHealth, 'localization/health', STATE_QOS)
                received = []
                peer.create_subscription(CorridorObservationList, 'perception/corridor_observation',
                                         received.append, STATE_QOS)
                executor = SingleThreadedExecutor()
                executor.add_node(observer)
                executor.add_node(peer)
                end = time.monotonic() + 6
                while time.monotonic() < end:
                    stamp = peer.get_clock().now().to_msg()
                    health = LocalizationHealth(robot_id='robot_2', state=LocalizationHealth.OK,
                                                position_std_m=0.01, yaw_std_rad=0.01)
                    health.meta.valid = True
                    health.meta.schema_version = 1
                    health.meta.source_id = 'test-b2'
                    health.meta.source_session_id = 'test-b2-session'
                    health.meta.revision = 1
                    health.meta.header.stamp = stamp
                    publisher.publish(health)
                    pose = TransformStamped()
                    pose.header.stamp = stamp
                    pose.header.frame_id = 'map'
                    pose.child_frame_id = 'robot_2/base_link'
                    pose.transform.rotation.w = 1.0
                    observer.tf_buffer.set_transform(pose, 'synthetic-test-b2')
                    executor.spin_once(timeout_sec=0.02)
                    if received and received[-1].meta.valid:
                        break
                self.assertTrue(received)
                message = received[-1]
                self.assertTrue(message.meta.valid)
                self.assertTrue(message.observations)
                self.assertTrue(all(item.position_valid and item.observation == item.OUTSIDE
                                    for item in message.observations))
                if os.environ.get('AX_B3_OUTSIDE_EVIDENCE'):
                    evidence = {'scope': 'approved temporary copy; synthetic B2 pose/health',
                        'production_approved': json.loads(original)['approved'],
                        'observations': [{'corridor_id': item.corridor_id, 'state': 'OUTSIDE',
                                          'position_valid': item.position_valid}
                                         for item in message.observations]}
                    Path(os.environ['AX_B3_OUTSIDE_EVIDENCE']).write_text(json.dumps(evidence, indent=2))
            finally:
                if executor is not None:
                    executor.shutdown()
                for node in (peer, observer):
                    if node is not None:
                        node.destroy_node()
                rclpy.try_shutdown()
        self.assertEqual(original_path.read_bytes(), original)
