"""Check robot routing, mock outcomes, and isolated Gazebo entities using ROS."""
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock
import xml.etree.ElementTree as ET

import rclpy
from rclpy.action import ActionClient, GoalResponse
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from ax_interfaces.action import ExecuteLocalTask
from ax_interfaces.msg import LoadState, ResultStatus

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src/ax_task_executor'))
from ax_task_executor.mock_executor import MockExecutor
from ax_task_executor.gazebo_mock_display import MockDisplay


class MockRobotTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)

    def init_ros(self, robot):
        rclpy.init(args=['--ros-args', '-p', f'robot_id:={robot}', '-p',
                        f'execution_db:={self.temp.name}/executions.sqlite3'])

    @staticmethod
    def request(execution_id):
        request = ExecuteLocalTask.Goal()
        request.execution.execution_id = execution_id
        request.execution.operation = 'CONFIRM_LOAD'
        request.task_kind = ExecuteLocalTask.Goal.CONFIRM_LOAD
        request.object_id = 'test_box'
        return request

    def test_action_client_retransmission_on_both_robots(self):
        for robot in ('robot_1', 'robot_2'):
            with self.subTest(robot=robot):
                self.init_ros(robot)
                node = MockExecutor()
                client_node = Node('a3_mock_test_client')
                client = ActionClient(client_node, ExecuteLocalTask,
                                      f'/a3_mock/{robot}/execute_local_task')
                executor = SingleThreadedExecutor()
                executor.add_node(node)
                executor.add_node(client_node)
                try:
                    self.assertTrue(client.wait_for_server(timeout_sec=5))
                    node.make_result = Mock(wraps=node.make_result)
                    node.set_parameters([rclpy.parameter.Parameter('mock_load', value='STOWED')])
                    for retry in range(2):
                        request = self.request('action-retry')
                        future = client.send_goal_async(request)
                        executor.spin_until_future_complete(future, timeout_sec=5)
                        self.assertTrue(future.done())
                        handle = future.result()
                        self.assertTrue(handle.accepted)
                        result_future = handle.get_result_async()
                        executor.spin_until_future_complete(result_future, timeout_sec=5)
                        self.assertTrue(result_future.done())
                        self.assertEqual(result_future.result().result.load_state, LoadState.STOWED)
                        node.set_parameters([rclpy.parameter.Parameter('mock_load', value='EMPTY')])
                    self.assertEqual(node.make_result.call_count, 1)
                    changed = self.request('action-retry')
                    changed.object_id = 'different'
                    future = client.send_goal_async(changed)
                    executor.spin_until_future_complete(future, timeout_sec=5)
                    self.assertTrue(future.done())
                    self.assertFalse(future.result().accepted)
                finally:
                    executor.shutdown()
                    client.destroy()
                    client_node.destroy_node()
                    node.destroy_node()
                    rclpy.shutdown()

    def test_duplicate_changed_request_restart_and_robot_isolation(self):
        for robot in ('robot_1', 'robot_2'):
            for restart in (False, True):
                with self.subTest(robot=robot, restart=restart):
                    self.init_ros(robot)
                    node = MockExecutor()
                    try:
                        request = self.request('same-id')
                        node.set_parameters([rclpy.parameter.Parameter(
                            'mock_load', value='EMPTY' if restart else 'STOWED')])
                        self.assertEqual(node.accept_goal(request), GoalResponse.ACCEPT)
                        # Duplicate while accepted, before execute: same record.
                        request.execution.task_revision = 10
                        self.assertEqual(node.accept_goal(request), GoalResponse.ACCEPT)
                        node.make_result = Mock(wraps=node.make_result)
                        for _ in range(2):
                            goal = Mock(request=request)
                            result = node.execute(goal)
                            self.assertEqual(result.load_state, LoadState.STOWED)
                            goal.succeed.assert_called_once()
                        self.assertEqual(node.make_result.call_count, 0 if restart else 1)
                        for field, value in (('object_id', 'other'),
                                             ('inventory_revision', 2),
                                             ('peer_robot_id', 'other')):
                            changed = self.request('same-id')
                            setattr(changed, field, value)
                            self.assertEqual(node.accept_goal(changed), GoalResponse.REJECT)
                        pending = self.request('pending-id')
                        self.assertEqual(node.accept_goal(pending), GoalResponse.ACCEPT)
                        if restart:
                            # Accepted before restart: preserve its sampled input.
                            result = node.execute(Mock(request=pending))
                            self.assertEqual(result.load_state, LoadState.STOWED)
                    finally:
                        node.destroy_node()
                        rclpy.shutdown()

    def test_both_robots_route_results_and_keep_indicators_separate(self):
        for robot in ('robot_1', 'robot_2'):
            with self.subTest(robot=robot):
                self.init_ros(robot)
                executor = MockExecutor()
                display = MockDisplay()
                try:
                    self.assertIn(
                        f'/a3_mock/{robot}/execute_local_task/_action/send_goal',
                        [name for name, _ in executor.get_service_names_and_types()])
                    topic = ('/a3_mock/result_display' if robot == 'robot_2'
                             else '/a3_mock/robot_1/result_display')
                    self.assertEqual(executor.result_display.topic_name, topic)
                    self.assertIn(topic, [s.topic_name for s in display.subscriptions])
                    for value, outcome, load in (
                        ('STOWED', ResultStatus.SUCCEEDED, LoadState.STOWED),
                        ('EMPTY', ResultStatus.FAILED, LoadState.EMPTY),
                        ('UNKNOWN', ResultStatus.UNCERTAIN, LoadState.UNKNOWN),
                    ):
                        executor.set_parameters([rclpy.parameter.Parameter('mock_load', value=value)])
                        request = self.request(value)
                        self.assertEqual(executor.accept_goal(request), GoalResponse.ACCEPT)
                        goal = Mock(request=request)
                        result = executor.execute(goal)
                        self.assertEqual(result.status.outcome, outcome)
                        self.assertEqual(result.load_state, load)
                        (goal.succeed if value == 'STOWED' else goal.abort).assert_called_once()
                        display.receive(SimpleNamespace(data=value))
                        display.service = Mock(return_value=True)
                        display.update()
                        calls = display.service.call_args_list
                        prefix = ('a3_mock_indicator_' if robot == 'robot_2'
                                  else 'a3_mock_indicator_robot_1_')
                        self.assertEqual(len(calls), 4)
                        for call in calls[:3]:
                            self.assertTrue(call.args[2].startswith(f'name: "{prefix}'))
                            if robot == 'robot_2':
                                self.assertNotIn('robot_1', call.args[2])
                        sdf = calls[-1].args[2][5:]
                        # The service request wraps the XML in protobuf quotes.
                        model = ET.fromstring(json.loads(sdf)).find('model')
                        self.assertEqual(model.attrib['name'], prefix + value)
                        self.assertEqual(model.findtext('static'), 'true')
                        self.assertIsNone(model.find('link/collision'))
                        self.assertEqual(model.findtext('pose').split()[0],
                                         '-1' if robot == 'robot_1' else '0')
                finally:
                    display.destroy_node()
                    executor.destroy_node()
                    rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
