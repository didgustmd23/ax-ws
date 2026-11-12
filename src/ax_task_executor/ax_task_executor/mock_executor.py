from copy import deepcopy
import json
import math
import os
from pathlib import Path
import sqlite3
import time
from uuid import uuid4

from ax_interfaces.action import ExecuteLocalTask
from ax_interfaces.msg import LoadState, ResultStatus, RobotTaskStatus
from ax_task_executor.execution_store import ExecutionStore
from ax_task_executor.task_state_machine import ConfirmLoadStateMachine, TaskPhase
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields
from std_msgs.msg import String


# 모의 서버를 띄울 수 있는 로봇 ID(시험 구성 robot_1~robot_4)
ROBOT_IDS = ('robot_1', 'robot_2', 'robot_3', 'robot_4')


# 로봇별 모의 결과 표시 Topic. robot_2는 이전 경로를 유지한다.
def result_display_topic(robot_id):
    if robot_id == 'robot_2':
        return '/a3_mock/result_display'

    return f'/a3_mock/{robot_id}/result_display'


# 계약 4.3절의 로컬 작업 Action 경로
def execute_action_name(robot_id):
    return f'/ax/{robot_id}/executor/execute_local_task'


# 로컬 작업 실행기 기록 보존 폴더: ${ROS_HOME:-~/.ros}/ax_task_executor/<robot_id>
def record_dir(robot_id):
    ros_home = os.environ.get('ROS_HOME') or str(Path.home() / '.ros')

    return Path(ros_home) / 'ax_task_executor' / robot_id


# 지정하지 않으면 로봇별 보존 경로의 원장을 쓴다(재시작해도 이전 수락·결과가 남는다)
def default_execution_db(robot_id):
    return str(record_dir(robot_id) / 'executions.sqlite3')


class MockExecutor(Node):
    # 로봇 ID·원장·상태 Topic·Action 서버를 준비한다
    def __init__(self):
        super().__init__('a3_mock_executor', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self.declare_parameter('robot_id', 'robot_2')
        self.robot_id = self.get_parameter('robot_id').value
        if self.robot_id not in ROBOT_IDS:
            raise ValueError('robot_id must be one of robot_1~robot_4')
        # 시험 입력일 뿐 실제 센서나 Gazebo 관측이 아니다.
        self.declare_parameter('mock_load', 'UNKNOWN')
        # 확인 대기 시간 동안 취소 요청을 받을 수 있다. 실제 센서 대기는 아니다.
        self.declare_parameter('mock_delay_s', 0.5)
        self._active_execution_id = None
        # execution_id → 실제로 처리 중인 Goal의 UUID 문자열과 완료 Future
        self._owner_goal = {}
        self._inflight = {}
        # 취소를 받아들인 Goal UUID 문자열. 재요청 Goal 취소가 원래 작업을 멈추지 않게 Goal 단위로 둔다.
        self._cancel_requested = set()
        self.declare_parameter('execution_db', '')
        if not self.get_parameter('execution_db').value:
            self.set_parameters([
                Parameter('execution_db', value=default_execution_db(self.robot_id))])
        self.store = ExecutionStore(
            self.get_parameter('execution_db').value, self.robot_id)
        self.get_logger().info(f"모의 실행 원장: {self.get_parameter('execution_db').value}")
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
            String, result_display_topic(self.robot_id),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.server = ActionServer(
            self,
            ExecuteLocalTask,
            execute_action_name(self.robot_id),
            execute_callback=self.execute_action,
            goal_callback=self.accept_goal,
            cancel_callback=self.cancel_goal,
        )
        self.get_logger().info(
            'A3 모의 서버 준비 완료. 실제 로봇을 구동하지 않습니다.'
        )

    # Goal 수락 여부를 정한다. 끝난 실행의 재요청은 다른 작업 처리 중에도 저장 결과 조회로 받는다
    def accept_goal(self, request):
        execution_id = request.execution.execution_id
        if request.task_kind != ExecuteLocalTask.Goal.CONFIRM_LOAD:
            self.get_logger().warning('CONFIRM_LOAD만 지원합니다.')
            return GoalResponse.REJECT
        if not execution_id or not request.object_id:
            self.get_logger().warning('execution_id와 object_id가 필요합니다.')
            return GoalResponse.REJECT
        busy = self._active_execution_id not in (None, execution_id)
        if busy and not self._has_result(execution_id):
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
        # 끝난 실행의 재요청은 현재 처리 중 작업 표시를 바꾸지 않는다.
        if not busy and not self._has_result(execution_id):
            self._active_execution_id = execution_id
        return GoalResponse.ACCEPT

    # 저장된 최종 결과가 있는지 확인한다. 원장을 읽지 못하면 없다고 본다
    def _has_result(self, execution_id):
        try:
            return self.store.cached_result(execution_id) is not None
        except sqlite3.Error:
            return False

    def cancel_goal(self, goal_handle):
        """완료 전 모의 작업을 처리 중인 그 Goal만 취소한다. 물리 정지를 확인한 것으로 해석하지 않는다."""
        execution_id = goal_handle.request.execution.execution_id
        goal_key = goal_key_of(goal_handle)
        try:
            if self.store.cached_result(execution_id) is not None:
                return CancelResponse.REJECT
        except sqlite3.Error:
            return CancelResponse.REJECT
        # 재요청 Goal은 저장 결과를 기다릴 뿐이라 취소하지 않는다(원래 작업은 계속).
        if self._owner_goal.get(execution_id) != goal_key:
            self.get_logger().info('재요청 Goal 취소는 받지 않습니다. 원래 Goal을 취소하세요.')
            return CancelResponse.REJECT
        self._cancel_requested.add(goal_key)
        return CancelResponse.ACCEPT

    async def execute_action(self, goal_handle):
        """ROS 요청 경로: 기다리는 동안에도 상태 발행·취소 콜백을 처리한다."""
        execution_id = goal_handle.request.execution.execution_id
        if execution_id in self._inflight or self._has_result(execution_id):
            # 같은 실행 ID 재요청은 기존 처리가 끝나기를 기다린 뒤 저장 결과만 돌려준다.
            # 상태 Topic·revision·결과 표시는 다시 발행하지 않는다.
            if execution_id in self._inflight:
                await self._inflight[execution_id]
            return self._replay(goal_handle)
        goal_key = goal_key_of(goal_handle)
        done = Future()
        self._inflight[execution_id] = done
        self._owner_goal[execution_id] = goal_key
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
                while time.monotonic() < deadline and goal_key not in self._cancel_requested:
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
                goal_handle, advance, goal_key in self._cancel_requested)
        finally:
            self._inflight.pop(execution_id, None)
            self._owner_goal.pop(execution_id, None)
            self._cancel_requested.discard(goal_key)
            if self._active_execution_id == execution_id:
                self._active_execution_id = None
            done.set_result(True)

    def execute(self, goal_handle):
        """동기 결과 처리 경로. 실제 ROS Action은 execute_action을 통해 호출된다."""
        execution_id = goal_handle.request.execution.execution_id
        if self._has_result(execution_id):
            return self._replay(goal_handle)
        try:
            advance, invalid = self._begin_execution(goal_handle)
            if invalid is not None:
                return invalid

            return self._complete_execution(goal_handle, advance)
        finally:
            if self._active_execution_id == execution_id:
                self._active_execution_id = None

    # 저장 결과로 이 Goal만 끝낸다. 상태 Topic·결과 표시·처리 중 작업 표시는 건드리지 않는다
    def _replay(self, goal_handle):
        execution_id = goal_handle.request.execution.execution_id
        result = ExecuteLocalTask.Result()

        try:
            saved = self.store.cached_result(execution_id)
        except sqlite3.Error as error:
            saved = None
            result.status.detail = str(error)

        if saved is None:
            # 원래 처리가 결과 없이 끝났다(저장 실패 등). 성공으로 보고하지 않는다.
            result.status.outcome = ResultStatus.UNCERTAIN
            result.status.error_code = 'MOCK_STORE_ERROR'
            result.status.detail = result.status.detail or '저장된 결과가 없습니다.'
            goal_handle.abort()
            return result

        set_message_fields(result, saved)

        # 재요청 Goal은 취소 요청 중이 아니므로 CANCELED 결과도 abort로 끝낸다(README 참고).
        if result.status.outcome == ResultStatus.SUCCEEDED:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        self.get_logger().info(f'[{self.robot_id}] {execution_id}: 저장 결과 재사용')
        return result

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


# Goal UUID를 문자열 키로 만든다. Mock 등 UUID가 없는 Goal은 객체 ID를 쓴다
def goal_key_of(goal_handle):
    uuid = getattr(getattr(goal_handle, 'goal_id', None), 'uuid', None)

    try:
        return bytes(uuid).hex()
    except TypeError:
        return f'object-{id(goal_handle)}'


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
