import json
import os
from pathlib import Path
import sqlite3
import math
import time
from copy import deepcopy
from uuid import uuid4

from ax_interfaces.action import ExecuteLocalTask
from ax_interfaces.msg import LoadState, ResultStatus, RobotTaskStatus
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future

from std_msgs.msg import String
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields

from ax_task_executor.execution_store import ExecutionStore
from ax_task_executor.task_state_machine import ConfirmLoadStateMachine, TaskPhase


class MockExecutor(Node):
    def __init__(self):
        super().__init__('a3_mock_executor', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self.declare_parameter('robot_id', 'robot_2')
        self.robot_id = self.get_parameter('robot_id').value
        if self.robot_id not in ('robot_1', 'robot_2'):
            raise ValueError('robot_id must be robot_1 or robot_2')
        # 시험 입력일 뿐 실제 센서나 Gazebo 관측이 아니다.
        self.declare_parameter('mock_load', 'UNKNOWN')
        # 확인 대기 시간 동안 취소 요청을 받을 수 있다. 실제 센서 대기는 아니다.
        self.declare_parameter('mock_delay_s', 0.5)
        self._active_execution_id = None
        self._inflight = {}
        self._cancel_requested = set()
        default_store = Path(os.environ.get('ROS_HOME', str(Path.home() / '.ros')))
        self.declare_parameter('execution_db', str(default_store / 'a3_mock/executions.sqlite3'))
        self.store = ExecutionStore(
            self.get_parameter('execution_db').value, self.robot_id)
        # Publish mock execution state on the contract Topic, explicitly invalid
        # for operational decisions. A new process always has a new session ID.
        self.task_status = RobotTaskStatus(robot_id=self.robot_id)
        self.task_status.meta.schema_version = 1
        self.task_status.meta.source_id = f'a3_mock_executor_{self.robot_id}'
        self.task_status.meta.source_session_id = uuid4().hex
        self.task_status.meta.valid = False
        self.task_status.meta.validity_reason = 'MOCK_ONLY'
        self.task_status_publisher = self.create_publisher(
            RobotTaskStatus, f'/ax/{self.robot_id}/executor/status',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # Heartbeats use steady time even while Gazebo sim time is paused.
        self.status_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.status_timer = self.create_timer(
            1.0, self.publish_task_status, clock=self.status_clock)
        self.update_task_status('MOCK_IDLE')
        self.result_display = self.create_publisher(
            String, ('/a3_mock/result_display' if self.robot_id == 'robot_2'
                     else '/a3_mock/robot_1/result_display'),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.server = ActionServer(
            self,
            ExecuteLocalTask,
            f'/a3_mock/{self.robot_id}/execute_local_task',
            execute_callback=self.execute_action,
            goal_callback=self.accept_goal,
            cancel_callback=self.cancel_goal,
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
        if self._active_execution_id not in (None, execution_id):
            self.get_logger().warning('이 로봇이 다른 작업을 처리 중입니다.')
            return GoalResponse.REJECT
        delay = self.get_parameter('mock_delay_s').value
        if not isinstance(delay, (int, float)) or not math.isfinite(delay) or not 0 <= delay <= 30:
            self.get_logger().warning('mock_delay_s는 0~30초여야 합니다.')
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
        self._active_execution_id = execution_id
        return GoalResponse.ACCEPT

    def cancel_goal(self, goal_handle):
        """완료 전 모의 작업만 취소한다. 물리 정지를 확인한 것으로 해석하지 않는다."""
        execution_id = goal_handle.request.execution.execution_id
        try:
            if self.store.cached_result(execution_id) is not None:
                return CancelResponse.REJECT
        except sqlite3.Error:
            return CancelResponse.REJECT
        if execution_id != self._active_execution_id:
            return CancelResponse.REJECT
        self._cancel_requested.add(execution_id)
        return CancelResponse.ACCEPT

    async def execute_action(self, goal_handle):
        """ROS 요청 경로: 기다리는 동안에도 상태 발행·취소 콜백을 처리한다."""
        execution_id = goal_handle.request.execution.execution_id
        if execution_id in self._inflight:
            # 같은 실행 ID 재요청은 기존 계산이 끝나기를 기다리고 결과만 재사용한다.
            await self._inflight[execution_id]
            return self.execute(goal_handle)
        done = Future()
        self._inflight[execution_id] = done
        try:
            advance, invalid = self._begin_execution(goal_handle)
            if invalid is not None:
                return invalid
            try:
                cached = self.store.cached_result(execution_id)
            except sqlite3.Error:
                cached = None  # 아래 결과 저장 경로에서 오류를 UNCERTAIN으로 보고한다.
            if cached is None:
                deadline = time.monotonic() + self.get_parameter('mock_delay_s').value
                while time.monotonic() < deadline and execution_id not in self._cancel_requested:
                    # sleep 대신 ROS 타이머를 기다려 취소·heartbeat 처리를 막지 않는다.
                    wake = Future()
                    timer = self.create_timer(
                        0.05, lambda: wake.set_result(True) if not wake.done() else None,
                        clock=self.status_clock)
                    try:
                        await wake
                    finally:
                        self.destroy_timer(timer)
            return self._complete_execution(
                goal_handle, advance, execution_id in self._cancel_requested)
        finally:
            self._inflight.pop(execution_id, None)
            self._cancel_requested.discard(execution_id)
            if self._active_execution_id == execution_id:
                self._active_execution_id = None
            done.set_result(True)

    def execute(self, goal_handle):
        """동기 결과 처리 경로. 실제 ROS Action은 execute_action을 통해 호출된다."""
        try:
            advance, invalid = self._begin_execution(goal_handle)
            return invalid if invalid is not None else self._complete_execution(goal_handle, advance)
        finally:
            if self._active_execution_id == goal_handle.request.execution.execution_id:
                self._active_execution_id = None

    def _begin_execution(self, goal_handle):
        # 현재 Action 콜백에서 실제로 사용하는 단계 머신이다.
        # 재요청은 새 물리 작업이 아니라 기존 SQLite 결과 조회로 연결된다.
        flow = ConfirmLoadStateMachine()
        execution = goal_handle.request.execution

        def advance(phase, progress, status=None):
            # 1. 순서 검사 → 2. 상태 Topic → 3. Action Feedback·로그.
            # 전환이 잘못되면 상태를 발행하기 전에 예외가 발생한다.
            flow.transition(phase)
            self.update_task_status(phase.value, execution, status)
            feedback = ExecuteLocalTask.Feedback(phase=phase.value, progress=progress)
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(
                f'[{self.robot_id}] {execution.execution_id}: {phase.value}')

        advance(TaskPhase.VALIDATING, 0.1)
        # 접수 후에도 실행 대상의 필수 정보를 확인한다.
        # 유효하지 않으면 장부 결과 계산과 표시등 발행으로 진행하지 않는다.
        request = goal_handle.request
        if (request.task_kind != ExecuteLocalTask.Goal.CONFIRM_LOAD
                or not execution.execution_id or not request.object_id):
            result = ExecuteLocalTask.Result()
            result.status.outcome = ResultStatus.FAILED
            result.status.error_code = 'MOCK_INVALID_GOAL'
            result.status.detail = 'CONFIRM_LOAD 실행에 필요한 요청 정보가 유효하지 않습니다.'
            advance(TaskPhase.FAILED, 1.0, result.status)
            goal_handle.abort()
            return advance, result

        advance(TaskPhase.CONFIRMING_LOAD, 0.5)
        return advance, None

    def _complete_execution(self, goal_handle, advance, cancel_requested=False):
        try:
            saved = self.store.result(
                goal_handle.request.execution.execution_id,
                self.make_canceled_result if cancel_requested else self.make_result)
        except (sqlite3.Error, RuntimeError) as error:
            result = ExecuteLocalTask.Result()
            result.status.outcome = ResultStatus.UNCERTAIN
            result.status.error_code = 'MOCK_STORE_ERROR'
            result.status.detail = str(error)
            advance(TaskPhase.UNCERTAIN, 1.0, result.status)
            goal_handle.abort()
            return result
        result = ExecuteLocalTask.Result()
        set_message_fields(result, saved)
        if result.status.outcome == ResultStatus.CANCELED:
            advance(TaskPhase.CANCELING, 0.9)
            advance(TaskPhase.CANCELED, 1.0, result.status)
            if goal_handle.is_cancel_requested is True:
                goal_handle.canceled()
            else:
                # 완료된 취소 결과의 재요청은 취소 요청 중인 Goal이 아니므로 abort한다.
                goal_handle.abort()
            return result
        phase = {
            ResultStatus.SUCCEEDED: TaskPhase.SUCCEEDED,
            ResultStatus.FAILED: TaskPhase.FAILED,
        }.get(result.status.outcome, TaskPhase.UNCERTAIN)
        advance(phase, 1.0, result.status)
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
    def make_canceled_result(_value):
        result = ExecuteLocalTask.Result()
        result.status.outcome = ResultStatus.CANCELED
        result.status.error_code = 'MOCK_CANCELED'
        result.status.detail = '외부 구동 없는 모의 작업을 취소했습니다. 물리 정지 판정이 아닙니다.'
        return message_to_ordereddict(result)

    def update_task_status(self, phase, execution=None, result=None):
        """Publish state changes; keep the last completed result while running."""
        updated = deepcopy(self.task_status)
        updated.local_phase = phase
        updated.paused = False  # No pause/resume operation in the mock server.
        if execution is not None:
            updated.execution = deepcopy(execution)
        if result is not None:
            updated.last_result = deepcopy(result)
        if updated != self.task_status:
            updated.meta.revision += 1
            self.task_status = updated
            self.publish_task_status()

    def publish_task_status(self):
        """Re-assess the in-memory mock state and refresh its ROS timestamp."""
        self.task_status.meta.header.stamp = self.get_clock().now().to_msg()
        self.task_status_publisher.publish(self.task_status)

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
