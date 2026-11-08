import json
import os
from pathlib import Path
import sqlite3

from ax_interfaces.action import ExecuteLocalTask
from ax_interfaces.msg import LoadState, ResultStatus
from ax_task_executor.execution_store import ExecutionStore
import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields
from std_msgs.msg import String


class MockExecutor(Node):
    def __init__(self):
        super().__init__('a3_mock_executor')
        self.declare_parameter('robot_id', 'robot_2')
        self.robot_id = self.get_parameter('robot_id').value
        if self.robot_id not in ('robot_1', 'robot_2'):
            raise ValueError('robot_id must be robot_1 or robot_2')
        # 시험 입력일 뿐 실제 센서나 Gazebo 관측이 아니다.
        self.declare_parameter('mock_load', 'UNKNOWN')
        default_store = Path(os.environ.get('ROS_HOME', str(Path.home() / '.ros')))
        self.declare_parameter('execution_db', str(default_store / 'a3_mock/executions.sqlite3'))
        self.store = ExecutionStore(
            self.get_parameter('execution_db').value, self.robot_id)
        self.result_display = self.create_publisher(
            String, ('/a3_mock/result_display' if self.robot_id == 'robot_2'
                     else '/a3_mock/robot_1/result_display'),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.server = ActionServer(
            self,
            ExecuteLocalTask,
            f'/a3_mock/{self.robot_id}/execute_local_task',
            execute_callback=self.execute,
            goal_callback=self.accept_goal,
        )
        self.get_logger().info(
            'A3 모의 서버 준비 완료. 실제 로봇을 구동하지 않습니다.'
        )

    def accept_goal(self, request):
        execution_id = request.execution.execution_id
        if request.task_kind != ExecuteLocalTask.Goal.CONFIRM_LOAD:
            self.get_logger().warning('CONFIRM_LOAD만 지원합니다.')
            return GoalResponse.REJECT
        if not execution_id or not request.object_id:
            self.get_logger().warning('execution_id와 object_id가 필요합니다.')
            return GoalResponse.REJECT
        data = message_to_ordereddict(request)
        # Contract: task revision may change on retransmission.
        data['execution'].pop('task_revision')
        fingerprint = json.dumps(data, sort_keys=True, separators=(',', ':'))
        try:
            accepted = self.store.reserve(
                execution_id, fingerprint, self.get_parameter('mock_load').value.upper())
        except sqlite3.Error as error:
            self.get_logger().error(f'실행 기록 저장 실패: {error}')
            return GoalResponse.REJECT
        if not accepted:
            self.get_logger().warning('같은 execution_id의 요청 내용이 다릅니다.')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def execute(self, goal_handle):
        feedback = ExecuteLocalTask.Feedback()
        feedback.phase = 'MOCK_CONFIRM_LOAD'
        feedback.progress = 0.5
        goal_handle.publish_feedback(feedback)

        try:
            saved = self.store.result(
                goal_handle.request.execution.execution_id, self.make_result)
        except (sqlite3.Error, RuntimeError) as error:
            result = ExecuteLocalTask.Result()
            result.status.outcome = ResultStatus.UNCERTAIN
            result.status.error_code = 'MOCK_STORE_ERROR'
            result.status.detail = str(error)
            goal_handle.abort()
            return result
        result = ExecuteLocalTask.Result()
        set_message_fields(result, saved)
        if result.status.outcome == ResultStatus.SUCCEEDED:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        self.get_logger().info(result.status.detail)
        value = {LoadState.STOWED: 'STOWED', LoadState.EMPTY: 'EMPTY'}.get(
            result.load_state, 'UNKNOWN')
        self.result_display.publish(String(data=value))
        return result

    @staticmethod
    def make_result(value):
        result = ExecuteLocalTask.Result()

        if value == 'STOWED':
            result.load_state = LoadState.STOWED
            result.status.outcome = ResultStatus.SUCCEEDED
            result.status.detail = '시험 입력: 적재 있음. 실제 관측 아님.'
        elif value == 'EMPTY':
            result.load_state = LoadState.EMPTY
            result.status.outcome = ResultStatus.FAILED
            result.status.error_code = 'MOCK_EMPTY'
            result.status.detail = '시험 입력: 적재함 비어 있음.'
        else:
            result.load_state = LoadState.UNKNOWN
            result.status.outcome = ResultStatus.UNCERTAIN
            result.status.error_code = 'MOCK_UNKNOWN'
            result.status.detail = '시험 입력: 적재 여부 불명. 성공 처리하지 않음.'
        return message_to_ordereddict(result)

    def destroy_node(self):
        self.server.destroy()
        self.store.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MockExecutor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
