"""실제 JointState DDS 입력이 A3 내부 후퇴 판정까지 전달되는지 검증한다."""

from pathlib import Path
import sys
import tempfile
import time
import unittest

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/ax_task_executor'))
sys.path.insert(0, str(ROOT / 'tools/tests'))
import ros_test_domain
from ax_task_executor.local_task_executor import LocalTaskExecutor

_DOMAIN_STATE = None


def setUpModule():
    """김성민에게 배정된 domain만 잠그고 운영 이름 충돌을 검사한다."""
    global _DOMAIN_STATE
    _DOMAIN_STATE = ros_test_domain.enter_module_domain(ros_test_domain.A3_DOMAINS, r'^/ax/')


def tearDownModule():
    """시험 domain 잠금을 반환한다."""
    ros_test_domain.leave_module_domain(_DOMAIN_STATE)


class RetractionROSTests(unittest.TestCase):
    """실제 로봇 구동 없이 시험용 관절 자세로 수신·연속 정지·만료를 확인한다."""

    def test_joint_subscription_and_expiry(self):
        """합성 관절값만 사용하므로 실제 후퇴 자세 승인 근거로 쓰지 않는다."""
        with tempfile.TemporaryDirectory() as folder:
            rclpy.init(args=['--ros-args', '-p', 'robot_id:=robot_1',
                             '-p', 'name_check_s:=0.0',
                             '-p', f'lease_db:={folder}/lease.db',
                             '-p', f'execution_db:={folder}/execution.db',
                             '-p', 'retraction_pose_approved:=true',
                             '-p', 'retraction_joint_names:=[arm_joint]',
                             '-p', 'retraction_joint_positions:=[0.4]'])
            self.addCleanup(rclpy.try_shutdown)
            a3 = LocalTaskExecutor()
            # 기존 모의 실행기는 sim time이 기본이다. /clock 없는 이번 시험만 벽시계로 맞춘다.
            a3.set_parameters([Parameter('use_sim_time', value=False)])
            self.addCleanup(a3.destroy_node)
            peer = Node('a3_retraction_test_input')
            self.addCleanup(peer.destroy_node)
            publisher = peer.create_publisher(
                JointState, '/ax/robot_1/joint_states', qos_profile_sensor_data)
            executor = SingleThreadedExecutor()
            self.addCleanup(executor.shutdown)
            executor.add_node(a3)
            executor.add_node(peer)
            deadline = time.monotonic() + 5
            while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.02)
            self.assertGreater(publisher.get_subscription_count(), 0)
            deadline = time.monotonic() + 2
            while time.monotonic() < deadline:
                publisher.publish(JointState(header=peer_header(peer), name=['arm_joint'],
                                             position=[0.4], velocity=[0.0]))
                executor.spin_once(timeout_sec=0.03)
                if a3.arm_retraction_evidence().state == 'RETRACTED':
                    break
                time.sleep(0.02)
            self.assertEqual(a3.arm_retraction_evidence().state, 'RETRACTED',
                             repr(a3.arm_retraction_evidence()))
            time.sleep(0.35)
            self.assertEqual(a3.arm_retraction_evidence().state, 'UNKNOWN')


def peer_header(node):
    """시험 입력에 현재 ROS 시각을 붙인다."""
    from std_msgs.msg import Header
    return Header(stamp=node.get_clock().now().to_msg())
