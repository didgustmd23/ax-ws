"""원본 C4의 lease 요청 함수를 실제 DDS로 내 A3·명령 확인 서비스에 연결합니다."""
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest

import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.action import GoalResponse

ROOT = Path(__file__).resolve().parents[2]
for package in ('ax_task_executor', 'ax_c4'):
    sys.path.insert(0, str(ROOT / 'src' / package))
sys.path.insert(0, str(ROOT / 'tools/tests'))
import ros_test_domain
from ax_c4.manipulate_object_server import ManipulateObjectServer
from ax_interfaces.action import ManipulateObject
from ax_interfaces.msg import BudgetGrant, ExecutionLeaseRequest, ExecutionLeaseRevoke
from ax_interfaces.srv import CheckCommandLease
from ax_task_executor.local_task_executor import LocalTaskExecutor, lease_service_name
from test_a3_lease_check import tool_request

_DOMAIN_STATE = None


def setUpModule():
    """기존 김성민 domain만 예약하고 운영 이름 충돌을 확인합니다."""
    global _DOMAIN_STATE
    _DOMAIN_STATE = ros_test_domain.enter_module_domain(ros_test_domain.A3_DOMAINS, r'^/ax/')


def tearDownModule():
    """예약한 domain을 반환합니다."""
    ros_test_domain.leave_module_domain(_DOMAIN_STATE)


class C4LeaseTopicTests(unittest.TestCase):
    """C4 요청은 원본 함수이며 배정 응답·활성 Goal·예산 등록만 시험에서 제공합니다."""

    def setUp(self):
        """실제 A3/C4 노드와 DDS 콜백을 띄우되 물리 Action은 보내지 않습니다."""
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        rclpy.init(args=['--ros-args', '-p', 'robot_id:=robot_2', '-p', 'name_check_s:=0.0',
                         '-p', f'lease_db:={self.folder.name}/lease.db',
                         '-p', f'execution_db:={self.folder.name}/execution.db'])
        self.addCleanup(rclpy.try_shutdown)
        self.a3 = LocalTaskExecutor()
        self.addCleanup(self.a3.destroy_node)
        self.c4 = ManipulateObjectServer()
        self.addCleanup(self.destroy_c4)
        self.peer = Node('lease_topic_observer')
        self.addCleanup(self.peer.destroy_node)
        self.revokes = []
        self.peer.create_subscription(ExecutionLeaseRevoke,
                                      '/ax/robot_2/manipulation/lease/revoke', self.revokes.append, 50)
        self.service = self.peer.create_client(CheckCommandLease, lease_service_name('robot_2'))
        self.executor = MultiThreadedExecutor(num_threads=4)
        for node in (self.a3, self.c4, self.peer):
            self.executor.add_node(node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        self.addCleanup(self.stop_executor)
        self.wait(lambda: self.c4.lease_request_pub.get_subscription_count() > 0
                  and self.a3.lease_topics.grants.get_subscription_count() > 0)
        self.execution = SimpleNamespace(execution_id='c4-topic-exec', assignment_id='asg',
                                          assignment_generation=1)
        self.budget = BudgetGrant(budget_grant_id='topic-budget', kind='GRASP_ATTEMPT',
                                  granted_attempts=1, allowed_execution_ids=['c4-topic-exec'],
                                  time_budget_s=10.0)
        self.goal = SimpleNamespace(goal_id='a' * 32, execution_id='c4-topic-exec', budgets=[self.budget])
        self.c4_execution = SimpleNamespace(lease=None, resync_attempts=0)

    def stop_executor(self):
        """노드·원장을 닫기 전에 콜백 스레드를 멈춥니다."""
        self.executor.shutdown()
        self.thread.join(5)

    def destroy_c4(self):
        """시험에서 만든 상대 Action endpoint를 명시적으로 닫아 다음 domain 검사에 남기지 않습니다."""
        self.c4.server.destroy()
        self.c4.arm_client.destroy()
        self.c4.destroy_node()

    def wait(self, predicate, seconds=5):
        """실제 DDS 발견·응답을 정해진 시간 안에서 기다립니다."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if predicate():
                return
            time.sleep(0.01)
        self.fail('DDS condition not observed')

    def ready(self):
        """시험 배정 확인 후 내부 dispatcher 등록 입구를 사용합니다. 실제 D4 연결은 아닙니다."""
        authority = self.a3.authority
        authority.begin_assignment_check('check', 'asg', 1)
        self.assertTrue(authority.confirm_assignment(SimpleNamespace(
            request_id='check', a_session_id=authority.session_id, d_session_id='test-d4',
            assignment_generation=1, sequence=1, valid=True, validity_s=2.0)))
        self.a3.register_manipulation_goal(self.execution, self.goal.goal_id, [self.budget])

    def test_original_c4_request_grant_service_and_revoke(self):
        """실제 C4 request→A3 grant→서비스 허용→revoke→갱신 거절을 확인합니다."""
        self.ready()
        grant = self.c4._request_lease(self.goal, self.c4_execution)
        self.assertIsNotNone(grant)
        self.assertTrue(grant.granted)
        self.assertEqual((grant.execution_id, grant.goal_id, grant.lease_generation),
                         (self.execution.execution_id, self.goal.goal_id, 1))
        request = tool_request(robot='robot_2')
        request.execution_id, request.goal_id = grant.execution_id, grant.goal_id
        request.lease_generation = grant.lease_generation
        self.assertTrue(self.service.wait_for_service(timeout_sec=2))
        future = self.service.call_async(request)
        self.wait(future.done)
        self.assertTrue(future.result().allowed)
        self.a3.revoke_manipulation_lease('SAFETY_STOP')
        self.wait(lambda: self.revokes and self.execution.execution_id in self.c4.revoked)
        self.assertEqual(self.revokes[-1].lease_generation, 1)
        self.assertEqual(self.revokes[-1].goal_id, self.goal.goal_id)
        self.c4_execution.lease = grant
        renewed = self.c4._request_lease(self.goal, self.c4_execution, ExecutionLeaseRequest.RENEW)
        self.assertFalse(renewed.granted)
        self.assertEqual(renewed.reason_code, 'LEASE_REVOKED')

    def test_renewal_then_expired_request_is_not_resurrected(self):
        """C4의 실제 RENEW 발행도 같은 세대를 쓰고 만료 뒤에는 허용하지 않습니다."""
        self.ready()
        grant = self.c4._request_lease(self.goal, self.c4_execution)
        self.c4_execution.lease = grant
        renewed = self.c4._request_lease(self.goal, self.c4_execution, ExecutionLeaseRequest.RENEW)
        self.assertTrue(renewed.granted)
        self.assertEqual(renewed.lease_generation, grant.lease_generation)
        time.sleep(renewed.lease_s + 0.05)
        late = self.c4._request_lease(self.goal, self.c4_execution, ExecutionLeaseRequest.RENEW)
        self.assertFalse(late.granted)
        self.assertIn(late.reason_code, ('LEASE_EXPIRED', 'ASSIGNMENT_STALE'))

    def test_real_c4_request_without_registered_authority_is_denied(self):
        """C4 발행만으로 내 A3에 작업이나 예산이 자동 생성되지 않습니다."""
        grant = self.c4._request_lease(self.goal, self.c4_execution)
        self.assertFalse(grant.granted)
        self.assertEqual(grant.reason_code, 'ASSIGNMENT_STALE')

    @pytest.mark.xfail(
        'ACCEPT_AND_EXECUTE' in ManipulateObjectServer._on_goal.__code__.co_names
        and not hasattr(GoalResponse, 'ACCEPT_AND_EXECUTE'),
        reason='상대 C4의 알려진 ROS Python 수락 상수 오류이며 A3 변경으로 해결하지 않음',
        raises=AttributeError, strict=True)
    def test_original_c4_accepts_supported_action_goal(self):
        """원본 C4 Action 수락 콜백의 ROS Python 호환 오류를 숨기지 않고 재현합니다."""
        # 해당 오류가 코드에 남은 경우만 예상 실패다. C4 수정 후에는 정상 통과를 검사한다.
        request = ManipulateObject.Goal(operation=ManipulateObject.Goal.PICK)
        self.assertEqual(self.c4._on_goal(request), GoalResponse.ACCEPT)

    def test_finished_goal_cannot_obtain_new_lease(self):
        """종료된 부모 Goal의 지연 요청이 새 허가를 만들지 않습니다."""
        self.ready()
        grant = self.c4._request_lease(self.goal, self.c4_execution)
        self.assertTrue(grant.granted)
        self.assertTrue(self.a3.finish_manipulation_goal(self.execution.execution_id, self.goal.goal_id))
        self.assertIsNone(self.a3.authority.context)
        late = self.c4._request_lease(self.goal, self.c4_execution)
        self.assertFalse(late.granted)
        self.assertEqual(late.reason_code, 'GOAL_MISMATCH')
