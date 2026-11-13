"""Run unchanged team B4 against B3 output; synthetic inputs, no robot motion."""
# [검사 범위] B3와 승연님 B4 원본 코드를 연결합니다. 위치·거리 입력과 승인 영역 사본은 시험용입니다.
import json
from pathlib import Path
import sys
import tempfile
import time
import unittest

import rclpy
from ax_interfaces.msg import LocalizationHealth, SafetyState
from geometry_msgs.msg import TransformStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Range

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/ax_environment'))
sys.path.insert(0, str(ROOT / 'src/ax_safety/src/ax_safety'))
from ax_environment.corridor_observer import CorridorObserver, STATE_QOS  # noqa: E402
from ax_safety.b4.b4_risk_node import B4RiskNode  # noqa: E402


class B3B4HandoffTests(unittest.TestCase):
    """Exercise real ROS subscription and B4 validity decisions."""

    robot_id = 'robot_2'
    robot_model = 'AX_2'

    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        config = json.loads((ROOT / 'src/ax_environment/config/ax_warehouse_regions.json').read_text())
        config['approved'] = True  # test-only copy; repository approval is untouched
        self.config_path = Path(self.folder.name) / 'test-regions.json'
        self.config_path.write_text(json.dumps(config))
        rclpy.init(args=['--ros-args', '-r', f'__ns:=/ax/{self.robot_id}',
                         '-p', f'robot_id:={self.robot_id}', '-p', f'robot_model:={self.robot_model}',
                         '-p', 'use_sim_time:=false', '-p', f'regions_file:={self.config_path}',
                         '-p', f'description_config_dir:={ROOT}/src/ax_description/config',
                         '-p', 'front_range_topic:=/test_b3_b4/front',
                         '-p', 'rear_range_topic:=/test_b3_b4/rear'])
        self.addCleanup(rclpy.shutdown)
        self.observer = CorridorObserver()
        self.risk = B4RiskNode()
        self.peer = Node('b3_b4_test_peer')
        self.executor = SingleThreadedExecutor()
        for node in (self.observer, self.risk, self.peer):
            self.executor.add_node(node)
        self.addCleanup(self.cleanup_nodes)
        self.health = self.peer.create_publisher(
            LocalizationHealth, f'/ax/{self.robot_id}/localization/health', STATE_QOS)
        self.front = self.peer.create_publisher(Range, '/test_b3_b4/front', 10)
        self.rear = self.peer.create_publisher(Range, '/test_b3_b4/rear', 10)
        self.received = []
        self.peer.create_subscription(
            SafetyState, f'/ax/{self.robot_id}/safety/state', self.received.append, 10)
        self.publish_pose = True

    def cleanup_nodes(self):
        self.executor.shutdown()
        for node in (self.peer, self.risk, self.observer):
            node.destroy_node()

    def inputs(self):
        stamp = self.peer.get_clock().now().to_msg()
        health = LocalizationHealth()
        health.robot_id = self.robot_id
        health.state = health.OK
        health.position_std_m = 0.01
        health.yaw_std_rad = 0.01
        health.meta.valid = True
        health.meta.revision = 1
        health.meta.source_session_id = 'b2-test-session'
        health.meta.header.stamp = stamp
        health.meta.header.frame_id = 'map'
        self.health.publish(health)
        for publisher in (self.front, self.rear):
            distance = Range()
            distance.header.stamp = stamp
            distance.min_range = 0.01
            distance.max_range = 10.0
            distance.range = 5.0
            publisher.publish(distance)
        if self.publish_pose:
            pose = TransformStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = 'map'
            pose.child_frame_id = f'{self.robot_id}/base_link'
            pose.transform.rotation.w = 1.0
            self.observer.tf_buffer.set_transform(pose, 'test-b2')

    def wait_state(self, predicate, timeout=5):
        start = len(self.received)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.inputs()
            self.executor.spin_once(timeout_sec=0.01)
            for message in self.received[start:]:
                if predicate(message):
                    return message
        self.fail('B4 did not publish expected state')

    def test_normal_unknown_and_restored_observations_reach_actual_b4(self):
        # [핵심] 정상→무효→회복→TF 만료를 실제 B4 코드에 전달합니다. 물리 구동 검증은 아닙니다.
        clear = self.wait_state(lambda m: m.meta.valid and m.allow_forward)
        self.assertEqual(clear.robot_id, self.robot_id)
        self.observer.catalog['approved'] = False
        blocked = self.wait_state(lambda m: not m.meta.valid and not m.allow_forward)
        self.assertFalse(blocked.allow_backward)
        self.assertFalse(blocked.allow_rotate)
        self.observer.catalog['approved'] = True
        self.wait_state(lambda m: m.meta.valid and m.allow_forward)
        # B2 health remains live but TF stops updating: B3 must invalidate pose.
        self.publish_pose = False
        self.wait_state(lambda m: not m.meta.valid and not m.allow_forward)

    def test_empty_catalog_is_invalid_not_vacuously_clear(self):
        # [핵심] 통로가 0개인 목록을 빈 통로가 확인됐다는 뜻으로 오인하지 않아야 합니다.
        self.wait_state(lambda m: m.meta.valid and m.allow_forward)
        self.observer.corridor_ids = []
        self.assertEqual(self.observer.evaluate(time.time()), ({}, 'REGIONS_INVALID'))
        self.wait_state(lambda m: not m.meta.valid and not m.allow_forward)

    def test_nonfinite_clock_is_invalid_and_has_safe_stamp(self):
        result, reason = self.observer.evaluate(float('nan'))
        self.assertEqual(reason, 'CLOCK_INVALID')
        self.assertTrue(all(value == ('UNKNOWN', False) for value in result.values()))
        meta = self.observer.meta(float('nan'), reason)
        self.assertEqual(meta.header.stamp.sec, 0)
        self.assertFalse(meta.valid)


class Robot1B3B4HandoffTests(B3B4HandoffTests):
    """Run the same real B4 contract against the AX_1 manipulator outline."""

    robot_id = 'robot_1'
    robot_model = 'AX_1'


if __name__ == '__main__':
    unittest.main()
