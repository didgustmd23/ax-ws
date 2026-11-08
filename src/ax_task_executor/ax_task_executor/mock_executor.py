from ax_interfaces.action import ExecuteLocalTask
from ax_interfaces.msg import LoadState, ResultStatus
import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from std_msgs.msg import String


class MockExecutor(Node):
    def __init__(self):
        super().__init__('a3_mock_executor')
        # 시험 입력일 뿐 실제 센서나 Gazebo 관측이 아니다.
        self.declare_parameter('mock_load', 'UNKNOWN')
        self.seen = set()
        self.result_display = self.create_publisher(
            String, '/a3_mock/result_display',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.server = ActionServer(
            self,
            ExecuteLocalTask,
            '/a3_mock/robot_2/execute_local_task',
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
        if execution_id in self.seen:
            self.get_logger().warning('이미 받은 실행 ID입니다.')
            return GoalResponse.REJECT
        self.seen.add(execution_id)
        return GoalResponse.ACCEPT

    def execute(self, goal_handle):
        feedback = ExecuteLocalTask.Feedback()
        feedback.phase = 'MOCK_CONFIRM_LOAD'
        feedback.progress = 0.5
        goal_handle.publish_feedback(feedback)

        value = self.get_parameter('mock_load').value.upper()
        result = ExecuteLocalTask.Result()

        if value == 'STOWED':
            result.load_state = LoadState.STOWED
            result.status.outcome = ResultStatus.SUCCEEDED
            result.status.detail = '시험 입력: 적재 있음. 실제 관측 아님.'
            goal_handle.succeed()
        elif value == 'EMPTY':
            result.load_state = LoadState.EMPTY
            result.status.outcome = ResultStatus.FAILED
            result.status.error_code = 'MOCK_EMPTY'
            result.status.detail = '시험 입력: 적재함 비어 있음.'
            goal_handle.abort()
        else:
            result.load_state = LoadState.UNKNOWN
            result.status.outcome = ResultStatus.UNCERTAIN
            result.status.error_code = 'MOCK_UNKNOWN'
            result.status.detail = '시험 입력: 적재 여부 불명. 성공 처리하지 않음.'
            goal_handle.abort()

        self.get_logger().info(result.status.detail)
        self.result_display.publish(
            String(data=value if value in ('STOWED', 'EMPTY') else 'UNKNOWN'))
        return result


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
