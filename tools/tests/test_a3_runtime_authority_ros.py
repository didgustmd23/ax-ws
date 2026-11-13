"""Operational command checks through SQLite and the actual ROS service."""
# [검사 범위] A3 실제 ROS 서비스와 SQLite 수락 기록을 검사합니다. 입력 배정·Goal은 시험용입니다.
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest

# 공용 잠금·그래프 검사 후 합의한 domain에서만 ROS 시험을 실행합니다.
import sys as _domain_sys
from pathlib import Path as _DomainPath
_domain_sys.path.insert(0, str(_DomainPath(__file__).resolve().parent))
from kim_ros_test_domain import setUpModule, tearDownModule  # noqa: E402, F401
from unittest.mock import patch

import rclpy
from ax_interfaces.srv import CheckCommandLease
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src/ax_task_executor'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from ax_task_executor.lease_check import LeaseChecker, LeaseLedger  # noqa: E402
from ax_task_executor.local_task_executor import LocalTaskExecutor  # noqa: E402
from ax_task_executor.runtime_authority import RuntimeAuthority  # noqa: E402
from test_a3_lease_check import tool_request  # noqa: E402


class OperationalLedgerTests(unittest.TestCase):
    """Verify a real acceptance record cannot bypass live authorization."""

    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.now = 100.0
        self.authority = RuntimeAuthority('robot_1', lambda: self.now)
        self.ledger = LeaseLedger(Path(self.folder.name) / 'checks.db', 'robot_1')
        self.addCleanup(self.ledger.close)
        self.checker = LeaseChecker('robot_1', 'operational', '', 2.0, self.ledger,
                                    authority=self.authority)
        self.request = tool_request()
        self.authority.begin_assignment_check('check', 'assignment', 3)
        self.authority.confirm_assignment(SimpleNamespace(
            request_id='check', a_session_id=self.authority.session_id, d_session_id='d',
            assignment_generation=3, sequence=1, valid=True, validity_s=2.0))
        self.authority.activate_goal(SimpleNamespace(
            execution_id=self.request.execution_id, assignment_id='assignment',
            assignment_generation=3), self.request.goal_id)
        self.authority.set_lease(self.request.execution_id, self.request.goal_id, 1, 101.5)

    def test_allow_record_and_cached_allow_capped_to_real_deadline(self):
        result = self.checker.check(self.request)
        self.assertTrue(result['allowed'])
        self.assertEqual(result['reason_code'], '')
        self.assertTrue(result['accept_record_id'].startswith('accept-'))
        self.assertEqual(result['valid_for_s'], 1.5)
        self.now = 101.4
        cached = self.checker.check(self.request)
        self.assertAlmostEqual(cached['valid_for_s'], 0.1)
        self.assertEqual(self.ledger.accept_count(), 1)

    def test_cached_allow_rejected_after_revoke(self):
        # [핵심] 예전에 허용한 응답이 장부에 있어도 현재 철회된 명령은 거절해야 합니다.
        self.assertTrue(self.checker.check(self.request)['allowed'])
        self.authority.revoke(self.request.execution_id, self.request.goal_id, 1)
        self.assertEqual(self.checker.check(self.request)['reason_code'], 'LEASE_REVOKED')
        self.assertEqual(self.ledger.accept_count(), 1)

    def test_expiry_goal_end_and_restart_do_not_replay_allow(self):
        self.assertTrue(self.checker.check(self.request)['allowed'])
        self.now = 101.5
        self.assertEqual(self.checker.check(self.request)['reason_code'], 'LEASE_EXPIRED')
        self.authority.finish_goal(self.request.execution_id, self.request.goal_id)
        self.assertEqual(self.checker.check(self.request)['reason_code'], 'LEASE_EXPIRED')
        self.checker.authority = RuntimeAuthority('robot_1', lambda: self.now)
        self.assertEqual(self.checker.check(self.request)['reason_code'], 'LEASE_EXPIRED')

    def test_continue_requires_previous_accept(self):
        self.request.phase = CheckCommandLease.Request.CONTINUE
        self.assertEqual(self.checker.check(self.request)['reason_code'], 'AUTHORITY_INVALID')

    def test_slow_durable_write_cannot_return_expired_allow(self):
        # [핵심] 저장 중 기한이 끝나는 상황입니다. 저장 전 유효했다고 허용을 반환하면 안 됩니다.
        original = self.ledger.check

        def slow_write(*args, **kwargs):
            result = original(*args, **kwargs)
            self.now = 101.5
            return result

        with patch.object(self.ledger, 'check', side_effect=slow_write):
            self.assertEqual(self.checker.check(self.request)['reason_code'], 'LEASE_EXPIRED')

    def test_revoke_waits_until_acceptance_is_recorded(self):
        # [핵심] 수락 저장과 철회가 겹칠 때 순서가 분명하고, 철회 이후 요청은 거절되는지 봅니다.
        entered, release, revoked = threading.Event(), threading.Event(), threading.Event()
        original = self.ledger.check

        def paused_write(*args, **kwargs):
            entered.set()
            if not release.wait(3):
                raise TimeoutError('test did not release acceptance')
            return original(*args, **kwargs)

        result = []
        with patch.object(self.ledger, 'check', side_effect=paused_write):
            worker = threading.Thread(target=lambda: result.append(self.checker.check(self.request)))
            worker.start()
            self.assertTrue(entered.wait(3))

            def revoke():
                self.authority.revoke(self.request.execution_id, self.request.goal_id, 1)
                revoked.set()

            revoker = threading.Thread(target=revoke)
            revoker.start()
            try:
                self.assertFalse(revoked.wait(0.05))
            finally:
                release.set()
                worker.join(3)
                revoker.join(3)
        self.assertTrue(result[0]['allowed'])
        self.assertTrue(revoked.is_set())
        self.assertFalse(self.checker.check(self.request)['allowed'])


class OperationalServiceTests(unittest.TestCase):
    """Check the integrated node defaults to operational and denies fake goals."""

    def test_ros_service_and_assignment_response_connection(self):
        from ax_interfaces.msg import AssignmentCheckResponse

        with tempfile.TemporaryDirectory() as folder:
            rclpy.init(args=['--ros-args', '-p', 'robot_id:=robot_1',
                             '-p', 'name_check_s:=0.0',
                             '-p', f'execution_db:={folder}/executions.db',
                             '-p', f'lease_db:={folder}/checks.db'])
            server = LocalTaskExecutor()
            client = Node('runtime_authority_test_client')
            executor = MultiThreadedExecutor(num_threads=4)
            executor.add_node(server)
            executor.add_node(client)
            spinner = threading.Thread(target=executor.spin, daemon=True)
            spinner.start()
            try:
                self.assertEqual(server.authority.policy, 'operational')
                service = client.create_client(
                    CheckCommandLease, '/ax/robot_1/executor/lease/check_command')
                publisher = client.create_publisher(
                    AssignmentCheckResponse, '/ax/robot_1/executor/assignment_check/response', 100)
                self.assertTrue(service.wait_for_service(timeout_sec=5))

                def check(request):
                    future = service.call_async(request)
                    completed = threading.Event()
                    future.add_done_callback(lambda _: completed.set())
                    self.assertTrue(completed.wait(5))
                    return future.result()

                request = tool_request()
                self.assertEqual(check(request).reason_code, 'ASSIGNMENT_STALE')
                execution = SimpleNamespace(execution_id=request.execution_id,
                                            assignment_id='assignment', assignment_generation=3)
                request_id = server.request_assignment_check(execution)
                reply = AssignmentCheckResponse()
                reply.request_id = request_id
                reply.a_session_id = server.authority.session_id
                reply.d_session_id = 'd-session'
                reply.assignment_generation = 3
                reply.sequence = 1
                reply.valid = True
                reply.validity_s = 2.0
                deadline = time.monotonic() + 1.0
                while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
                    time.sleep(0.01)
                publisher.publish(reply)
                while server.authority.assignment is None and time.monotonic() < deadline:
                    time.sleep(0.01)
                self.assertEqual(server.authority.assignment, ('assignment', 3))
                self.assertEqual(check(tool_request()).reason_code, 'GOAL_MISMATCH')
                server.authority.activate_goal(execution, request.goal_id)
                server.authority.set_lease(execution.execution_id, request.goal_id, 1,
                                           server.authority.clock() + 0.5)
                self.assertTrue(check(tool_request()).allowed)
                server.authority.revoke(execution.execution_id, request.goal_id, 1)
                self.assertEqual(check(tool_request()).reason_code, 'LEASE_REVOKED')
            finally:
                executor.shutdown()
                spinner.join(5)
                client.destroy_node()
                server.destroy_node()
                rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
