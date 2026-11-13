"""A3 -> mock D4 checks and real A3 service requests shaped like team A4."""
from pathlib import Path
import sys
import tempfile
import time
import unittest

import rclpy
from ax_interfaces.msg import AssignmentCheckRequest, AssignmentCheckResponse, ExecutionRef
from ax_interfaces.srv import CheckCommandLease
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src/ax_task_executor'))
from ax_task_executor.local_task_executor import LocalTaskExecutor  # noqa: E402


class AssignmentHandoffTests(unittest.TestCase):
    """Do not implement or modify the other owner's D4 assignment ledger."""

    def test_periodic_d4_confirmation_a4_continue_and_lost_peer(self):
        with tempfile.TemporaryDirectory() as folder:
            rclpy.init(args=['--ros-args', '-p', 'robot_id:=robot_1',
                             '-p', 'name_check_s:=0.0', '-p', 'use_sim_time:=true',
                             '-p', f'execution_db:={folder}/e.db',
                             '-p', f'lease_db:={folder}/l.db'])
            server = LocalTaskExecutor()
            peer = Node('a3_handoff_test_peer')
            executor = SingleThreadedExecutor()
            executor.add_node(server)
            executor.add_node(peer)
            received = []
            respond = [True]
            replies = peer.create_publisher(
                AssignmentCheckResponse, '/ax/robot_1/executor/assignment_check/response', 100)

            def on_request(request):
                received.append(request)
                if not respond[0]:
                    return
                reply = AssignmentCheckResponse()
                reply.request_id = request.request_id
                reply.a_session_id = request.a_session_id
                reply.d_session_id = 'test-d4-session'
                reply.assignment_generation = request.assignment_generation
                reply.sequence = len(received)
                reply.valid = True
                reply.validity_s = 2.0
                replies.publish(reply)

            peer.create_subscription(
                AssignmentCheckRequest, '/ax/robot_1/executor/assignment_check/request',
                on_request, 100)
            service = peer.create_client(
                CheckCommandLease, '/ax/robot_1/executor/lease/check_command')

            def spin_until(predicate, timeout=5):
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=0.02)
                    if predicate():
                        return
                self.fail('ROS handoff condition timed out')

            counter = 0

            def check(phase):
                nonlocal counter
                counter += 1
                request = CheckCommandLease.Request()
                request.request_id = f'a4-check-{counter}'
                request.robot_id = 'robot_1'
                request.authority = request.AUTH_A3_LOCAL_TASK
                request.execution_id = 'retract-1'
                request.local_task_kind = 6
                request.phase = phase
                request.command_source = request.SOURCE_A4_ARM
                request.command_id = 'a4-arm-command-1'
                request.command_digest = 'a' * 64
                future = service.call_async(request)
                spin_until(future.done)
                return future.result()

            try:
                spin_until(lambda: server.assignment_requests.get_subscription_count() > 0)
                self.assertTrue(service.wait_for_service(timeout_sec=3))
                execution = ExecutionRef()
                execution.execution_id = 'retract-1'
                execution.operation = 'RETRACT_ARM'
                execution.assignment_id = 'assignment-1'
                execution.assignment_generation = 2
                server.start_task_authority(execution)
                spin_until(lambda: server.authority.assignment is not None)
                server.authority.activate_goal(execution, local_task_kind=6)
                accepted = check(CheckCommandLease.Request.ACCEPT)
                self.assertTrue(accepted.allowed)
                self.assertTrue(accepted.accept_record_id)
                spin_until(lambda: len(received) >= 3)
                continued = check(CheckCommandLease.Request.CONTINUE)
                self.assertTrue(continued.allowed)
                self.assertEqual(continued.accept_record_id, '')
                self.assertEqual(server.ledger.accept_count(), 1)
                # /clock has never been published: renewals must use steady time.
                self.assertEqual(server.get_clock().now().nanoseconds, 0)
                respond[0] = False
                spin_until(lambda: server.authority.clock() > server.authority.assignment_deadline)
                self.assertEqual(check(CheckCommandLease.Request.CONTINUE).reason_code,
                                 'ASSIGNMENT_STALE')
                respond[0] = True
                spin_until(lambda: server.authority.assignment is not None)
                # Restored D4 does not revive the expired physical goal.
                self.assertEqual(check(CheckCommandLease.Request.CONTINUE).reason_code,
                                 'LEASE_REVOKED')
                self.assertTrue(server.stop_task_authority('retract-1'))
                requests_at_stop = len(received)
                end = time.monotonic() + 0.7
                while time.monotonic() < end:
                    executor.spin_once(timeout_sec=0.02)
                self.assertEqual(len(received), requests_at_stop)
            finally:
                executor.shutdown()
                peer.destroy_node()
                server.destroy_node()
                rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
