"""C4 ManipulateObject ROS2 Action 서버.

운영 모드에서는 A3 lease, A4 ProtectedArmTrajectory, A2 ToolCommand의
기존 계약을 그대로 호출한다. 공통 계약에 없는 궤적 생성·물체 인식·차체
이동은 이 노드가 임의로 만들지 않는다. 해당 입력이 아직 연결되지 않은
개발 환경에서는 ``mock_execution:=true``로 모의 응답을 주입할 수 있다.
"""

from dataclasses import dataclass, replace
import math
import time
import uuid

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from ax_interfaces.action import ManipulateObject, ProtectedArmTrajectory
from ax_interfaces.msg import (
    AttachState,
    ExecutionLeaseGrant,
    ExecutionLeaseRequest,
    ExecutionLeaseRevoke,
    ToolHoldState,
)
from ax_interfaces.srv import ToolCommand

from .c4_state_machine import (
    Budget,
    C4StateMachine,
    ExecutionResult,
    GoalSnapshot,
    LeaseGrant,
    Phase,
    Validation,
)


@dataclass
class _AttachCache:
    """A2 원시 AttachState의 최신 안전 검증 캐시."""

    message: AttachState | None = None
    received_at: float = 0.0


class ManipulateObjectServer(Node):
    """한 로봇의 조작 Goal을 순차적으로 보호 실행하는 C4 노드."""

    def __init__(self):
        super().__init__("c4_manipulate_object_server")
        p = self.declare_parameter
        self.robot_id = str(p("robot_id", "robot_1").value)
        self.mock_execution = bool(p("mock_execution", False).value)
        self.feedback_period_s = float(p("feedback_period_s", 0.1).value)
        self.state_ttl_s = float(p("state_ttl_s", 0.3).value)
        self.arm_joint_names = list(p("mock_arm_joint_names", ["joint_1"]).value)
        self.mock_arm_positions = list(p("mock_arm_positions", [0.0]).value)
        self.session_id = uuid.uuid4().hex
        self.machine = C4StateMachine(self.robot_id)
        self.attach = _AttachCache()
        self.grants: dict[str, ExecutionLeaseGrant] = {}
        self.revoked: dict[str, str] = {}
        self.active_goal = None
        self.active_arm_goal = None
        self.attempt_started_ros_time = None
        self._callback_group = ReentrantCallbackGroup()

        base = f"/ax/{self.robot_id}"
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        event_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # C4 → A3: lease 요청, A3 → C4: grant/revoke.
        self.lease_request_pub = self.create_publisher(
            ExecutionLeaseRequest, f"{base}/manipulation/lease/request", event_qos)
        self.create_subscription(
            ExecutionLeaseGrant, f"{base}/manipulation/lease/grant",
            self._on_lease_grant, event_qos, callback_group=self._callback_group)
        self.create_subscription(
            ExecutionLeaseRevoke, f"{base}/manipulation/lease/revoke",
            self._on_lease_revoke, event_qos, callback_group=self._callback_group)

        # C4 → A2: 공구 요청, A2 → C4: 원시 부착 상태.
        self.tool_client = self.create_client(
            ToolCommand, f"{base}/manipulation/tool_command",
            callback_group=self._callback_group)
        self.create_subscription(
            AttachState, f"{base}/manipulation/attach_state",
            self._on_attach, state_qos, callback_group=self._callback_group)

        # C4 → D2: 공구 보유 판정. 수락 응답만으로 HELD를 발행하지 않는다.
        self.hold_pub = self.create_publisher(
            ToolHoldState, f"{base}/manipulation/hold_state", state_qos)

        # C4 → A4: 실제 팔 제어기는 A4만 호출한다.
        self.arm_client = self.create_action_client(
            ProtectedArmTrajectory, f"{base}/arm/execute_trajectory")

        self.server = ActionServer(
            self,
            ManipulateObject,
            f"{base}/manipulation/execute",
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=self._callback_group,
        )
        self.get_logger().info(
            f"C4 ready for {self.robot_id}; mock_execution={self.mock_execution}")

    def create_action_client(self, action_type, name):
        """ActionClient 생성을 작은 함수로 감싸 테스트 대체 지점을 만든다."""

        from rclpy.action import ActionClient
        return ActionClient(self, action_type, name, callback_group=self._callback_group)

    @staticmethod
    def _goal_id(goal_handle) -> str:
        """ROS Action UUID를 계약의 소문자 32자리 goal_id로 변환한다."""

        return bytes(goal_handle.goal_id.uuid).hex()

    def _snapshot(self, goal_handle) -> GoalSnapshot:
        """ROS Goal을 상태 머신이 이해하는 불변 값으로 복사한다."""

        goal = goal_handle.request
        budgets = tuple(
            Budget(
                budget_grant_id=b.budget_grant_id,
                kind=b.kind,
                granted_attempts=int(b.granted_attempts),
                allowed_execution_ids=tuple(b.allowed_execution_ids),
                time_budget_s=float(b.time_budget_s),
            )
            for b in goal.budgets
        )
        return GoalSnapshot(
            goal_id=self._goal_id(goal_handle),
            robot_id=self.robot_id,
            execution_id=goal.execution.execution_id,
            execution_operation=str(goal.execution.operation),
            assignment_id=goal.execution.assignment_id,
            task_revision=int(goal.execution.task_revision),
            root_execution_id=goal.execution.root_execution_id,
            step_key=goal.execution.step_key,
            derive_rule_version=goal.execution.derive_rule_version,
            attempt_index=int(goal.execution.attempt_index),
            task_id=goal.execution.task_id,
            assignment_generation=int(goal.execution.assignment_generation),
            object_id=goal.object_id,
            operation=int(goal.operation),
            peer_robot_id=goal.peer_robot_id,
            target_frame=goal.target_pose.header.frame_id,
            target_values_finite=all(
                math.isfinite(float(value))
                for value in (
                    goal.target_pose.pose.position.x,
                    goal.target_pose.pose.position.y,
                    goal.target_pose.pose.position.z,
                    goal.target_pose.pose.orientation.x,
                    goal.target_pose.pose.orientation.y,
                    goal.target_pose.pose.orientation.z,
                    goal.target_pose.pose.orientation.w,
                )
            ),
            target_stamp_valid=(
                goal.target_pose.header.stamp.sec > 0
                or goal.target_pose.header.stamp.nanosec > 0
            ),
            peer_verified=self.mock_execution or not goal.peer_robot_id,
            budgets=budgets,
            has_candidate_preference=bool(goal.has_candidate_preference),
        )

    def _on_goal(self, goal_request):
        """Goal 접수 시에는 형식과 중복만 확인하고 물리 동작은 시작하지 않는다."""

        # 실제 Goal UUID는 on_goal 시점의 handle에 없으므로, 상세 검증은
        # execute callback에서 다시 수행한다. 여기서는 operation만 빠르게 거절한다.
        if goal_request.operation not in (ManipulateObject.Goal.PICK,
                                          ManipulateObject.Goal.PLACE):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT_AND_EXECUTE

    def _on_cancel(self, goal_handle):
        """취소는 무조건 새 하위 명령을 만들지 않는 방향으로 수락한다."""

        execution_id = goal_handle.request.execution.execution_id
        self.machine.request_cancel(execution_id)
        return CancelResponse.ACCEPT

    def _on_lease_grant(self, message):
        """A3 grant를 request_id별로 저장한다. 다른 Goal의 grant는 실행 중 무시된다."""

        self.grants[message.request_id] = message

    def _on_lease_revoke(self, message):
        """현재 Goal과 일치하는 lease 철회만 기록한다."""

        reason = message.reason_code or "LEASE_REVOKED"
        self.revoked[message.execution_id] = reason
        self.machine.revoke(
            message.execution_id,
            message.goal_id,
            int(message.lease_generation),
            reason,
        )

    def _on_attach(self, message):
        """A2의 원시 부착 신호를 저장한다. 이 신호만으로 성공을 확정하지 않는다."""

        if message.robot_id != self.robot_id:
            return
        self.attach = _AttachCache(message=message, received_at=time.monotonic())

    def _attempt_usage_messages(self, execution):
        """상태 머신의 시도 기록을 Action의 AttemptUsage 배열로 변환한다."""

        from ax_interfaces.msg import AttemptUsage

        usage = []
        for attempt in execution.attempts:
            record = AttemptUsage()
            record.budget_grant_id = attempt.budget_grant_id
            record.execution_id = attempt.execution_id
            record.attempt_index = attempt.attempt_index
            record.kind = attempt.kind
            record.state = {
                "STARTED": AttemptUsage.STARTED,
                "FINISHED": AttemptUsage.FINISHED,
                "UNCERTAIN": AttemptUsage.UNCERTAIN,
            }.get(attempt.state, AttemptUsage.UNSPECIFIED)
            if attempt.started_at_ros_s > 0:
                record.started_at.sec = int(attempt.started_at_ros_s)
                record.started_at.nanosec = int(
                    (attempt.started_at_ros_s - record.started_at.sec) * 1e9)
            if attempt.started_at_ros_s > 0:
                record.started_at.sec = int(attempt.started_at_ros_s)
                record.started_at.nanosec = int(
                    (attempt.started_at_ros_s - record.started_at.sec) * 1e9)
            record.elapsed_s = float(attempt.elapsed_s)
            usage.append(record)
        return usage

    def _publish_feedback(self, goal_handle, phase: Phase, progress: float, execution):
        """현재 단계·진행률·예산 사용 기록을 Action Feedback으로 발행한다."""

        feedback = ManipulateObject.Feedback()
        feedback.phase = phase.value
        feedback.progress = float(max(0.0, min(1.0, progress)))
        feedback.usage = self._attempt_usage_messages(execution)
        goal_handle.publish_feedback(feedback)

    def _publish_hold(self, goal: GoalSnapshot, state: int, evidence: str):
        """C4가 판정한 보유 상태를 D2에 발행한다."""

        message = ToolHoldState()
        message.robot_id = self.robot_id
        message.object_id = goal.object_id
        message.state = state
        message.evidence = evidence
        message.meta.valid = bool(evidence)
        message.meta.schema_version = 1
        message.meta.source_id = "c4_manipulate_object_server"
        message.meta.source_session_id = self.session_id
        message.meta.revision = int(time.monotonic_ns() & 0x7FFFFFFFFFFFFFFF)
        message.meta.header.stamp = self.get_clock().now().to_msg()
        self.hold_pub.publish(message)

    def _request_lease(self, goal: GoalSnapshot, execution, kind=None) -> LeaseGrant | None:
        """A3에 FIRST lease를 요청하고 같은 request_id의 grant만 기다린다."""

        if kind is None:
            kind = ExecutionLeaseRequest.FIRST
        for _ in range(self.machine.max_resync_attempts + 1):
            request_id = uuid.uuid4().hex
            if self.mock_execution:
                generation = (
                    execution.lease.lease_generation
                    if kind == ExecutionLeaseRequest.RENEW else 1
                )
                return LeaseGrant(request_id, goal.goal_id, goal.execution_id,
                                  generation, True, 1.5)

            request = ExecutionLeaseRequest()
            request.kind = kind
            request.goal_id = goal.goal_id
            request.c_session_id = self.session_id
            request.request_id = request_id
            request.execution_id = goal.execution_id
            request.budget_grant_id = next(
                b.budget_grant_id for b in goal.budgets if b.kind == "GRASP_ATTEMPT")
            request.lease_generation = (
                execution.lease.lease_generation
                if kind == ExecutionLeaseRequest.RENEW else 0
            )
            self.lease_request_pub.publish(request)
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline and rclpy.ok():
                message = self.grants.pop(request_id, None)
                if message is not None:
                    return LeaseGrant(
                        message.request_id,
                        message.goal_id,
                        message.execution_id,
                        int(message.lease_generation),
                        bool(message.granted),
                        float(message.lease_s),
                        message.reason_code,
                    )
                time.sleep(0.01)
            execution.resync_attempts += 1
        return None

    def _ensure_runtime(self, goal: GoalSnapshot, execution):
        """물리 명령 직전에 lease와 실행 시간 예산을 다시 확인한다."""

        now = time.monotonic()
        check = self.machine.check_runtime(goal.execution_id, now)
        if not check.ok:
            return check

        remaining = execution.lease.lease_s - (now - execution.lease_started_at)
        if remaining <= 0.5:
            grant = self._request_lease(goal, execution, ExecutionLeaseRequest.RENEW)
            if grant is None:
                return Validation(
                    False,
                    "LEASE_RESYNC_UNCERTAIN",
                    "A3 lease renewal response was not received",
                )
            renewed = self.machine.renew_lease(
                goal.execution_id, grant, time.monotonic())
            if not renewed.ok:
                return renewed
        return self.machine.check_runtime(goal.execution_id, time.monotonic())

    def _wait_future(self, future, timeout_s: float, goal_handle=None):
        """ROS Future를 제한 시간 안에서 기다린다. 무한 대기는 안전하지 않다."""

        deadline = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < deadline and rclpy.ok():
            if goal_handle is not None and goal_handle.is_cancel_requested:
                return None
            time.sleep(0.01)
        return future.result() if future.done() else None

    def _make_mock_trajectory(self) -> JointTrajectory:
        """연동 전 테스트용 궤적만 만든다.

        실제 운영 궤적은 C2/C3 관측·후보 및 로봇 모델 검증을 거쳐야 하므로
        공통 계약에 없는 IK나 충돌 회피를 이 함수에서 구현하지 않는다.
        """

        trajectory = JointTrajectory()
        trajectory.joint_names = self.arm_joint_names
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in self.mock_arm_positions]
        point.time_from_start.sec = 1
        trajectory.points = [point]
        return trajectory

    def _call_arm(self, goal: GoalSnapshot, goal_handle, execution) -> tuple[bool, str]:
        """A4 ProtectedArmTrajectory를 호출한다."""

        if execution.cancel_requested:
            return False, "CANCELED"
        if execution.revoke_reason:
            return False, execution.revoke_reason
        runtime = self._ensure_runtime(goal, execution)
        if not runtime.ok:
            return False, runtime.reason_code
        if not self.mock_execution and not self.arm_client.wait_for_server(timeout_sec=1.0):
            return False, "A4_UNAVAILABLE"
        if not self.mock_execution:
            # C4에는 IK·충돌 검사·관절 궤적 생성 계약이 없다.
            # 실제 궤적 공급자가 연결되기 전에는 모의 궤적을 A4로 보내지 않는다.
            return False, "TRAJECTORY_UNAVAILABLE"
        trajectory = self._make_mock_trajectory()
        arm_goal = ProtectedArmTrajectory.Goal()
        arm_goal.request_id = uuid.uuid4().hex
        arm_goal.authority = ProtectedArmTrajectory.Goal.AUTH_C_MANIPULATION
        arm_goal.execution_id = goal.execution_id
        arm_goal.goal_id = goal.goal_id
        arm_goal.lease_generation = execution.lease.lease_generation
        arm_goal.trajectory = trajectory
        if self.mock_execution:
            return True, ""
        future = self.arm_client.send_goal_async(arm_goal)
        handle = self._wait_future(future, 1.0, goal_handle)
        if goal_handle.is_cancel_requested:
            if handle is not None and handle.accepted:
                self._wait_future(handle.cancel_goal_async(), 1.0)
            return False, "CANCEL_UNCERTAIN"
        if handle is None or not handle.accepted:
            return False, "A4_REJECTED"
        self.active_arm_goal = handle
        result = self._wait_future(
            handle.get_result_async(),
            min(5.0, execution.lease.lease_s,
                max(0.0, execution.budget_deadline_s - time.monotonic())),
            goal_handle,
        )
        self.active_arm_goal = None
        if goal_handle.is_cancel_requested:
            self._wait_future(handle.cancel_goal_async(), 1.0)
            return False, "CANCEL_UNCERTAIN"
        if result is None:
            return False, "TIMED_OUT"
        if result.result.status.outcome != result.result.status.SUCCEEDED:
            return False, result.result.status.error_code or "A4_FAILED"
        return True, ""

    def _call_tool(self, goal: GoalSnapshot, command: int, goal_handle, execution) -> tuple[bool, str]:
        """A2 ToolCommand를 호출한다. accepted는 물리 완료가 아니다."""

        if goal_handle.is_cancel_requested or execution.cancel_requested:
            return False, "CANCELED"
        if execution.revoke_reason:
            return False, execution.revoke_reason
        runtime = self._ensure_runtime(goal, execution)
        if not runtime.ok:
            return False, runtime.reason_code
        if self.mock_execution:
            return True, ""
        if not self.tool_client.wait_for_service(timeout_sec=1.0):
            return False, "A2_UNAVAILABLE"
        request = ToolCommand.Request()
        request.request_id = uuid.uuid4().hex
        request.robot_id = self.robot_id
        request.execution_id = goal.execution_id
        request.goal_id = goal.goal_id
        request.object_id = goal.object_id
        request.lease_generation = execution.lease.lease_generation
        request.command = command
        response = self._wait_future(
            self.tool_client.call_async(request), 1.0, goal_handle)
        if goal_handle.is_cancel_requested:
            return False, "CANCEL_UNCERTAIN"
        if response is None or not response.accepted:
            return False, response.reason_code if response else "A2_TIMEOUT"
        return True, ""

    def _verify_attach(self, goal: GoalSnapshot, want_attached: bool) -> tuple[bool, str]:
        """A2 AttachState와 최신성으로 PICK/PLACE의 물리 결과를 확인한다."""

        if self.mock_execution:
            return True, "MOCK_ATTACH_STATE"
        message = self.attach.message
        if message is None or time.monotonic() - self.attach.received_at > self.state_ttl_s:
            return False, "ATTACH_STATE_STALE"
        stamp = message.meta.header.stamp.sec + message.meta.header.stamp.nanosec / 1e9
        now = self.get_clock().now().nanoseconds / 1e9
        check = C4StateMachine.validate_attach_evidence(
            robot_id=self.robot_id,
            object_id=goal.object_id,
            message_robot_id=message.robot_id,
            message_object_id=message.object_id,
            attached=bool(message.attached),
            want_attached=want_attached,
            meta_valid=bool(message.meta.valid),
            revision=int(message.meta.revision),
            source_session_id=message.meta.source_session_id,
            stamp_s=stamp,
            now_s=now,
            ttl_s=self.state_ttl_s,
        )
        return check.ok, check.reason_code or "ATTACH_STATE_CONFIRMED"

    def _result_message(self, result: ExecutionResult):
        """내부 결과를 기존 Action Result 메시지로 변환한다."""

        message = ManipulateObject.Result()
        message.status.outcome = {
            "SUCCEEDED": message.status.SUCCEEDED,
            "FAILED": message.status.FAILED,
            "CANCELED": message.status.CANCELED,
            "TIMED_OUT": message.status.TIMED_OUT,
            "UNCERTAIN": message.status.UNCERTAIN,
        }.get(result.outcome, message.status.UNSPECIFIED)
        message.status.error_code = result.error_code
        message.status.detail = result.detail
        message.object_hold_state = result.hold_state
        message.attempt_count = len(result.attempts)
        message.usage = self._attempt_usage_messages_from_result(result)
        message.need_reobservation_move = result.need_reobservation_move
        message.observation_reason = result.observation_reason
        return message

    def _attempt_usage_messages_from_result(self, result: ExecutionResult):
        """캐시된 결과도 동일한 AttemptUsage 형식으로 반환한다."""

        from ax_interfaces.msg import AttemptUsage

        usage = []
        for attempt in result.attempts:
            record = AttemptUsage()
            record.budget_grant_id = attempt.budget_grant_id
            record.execution_id = attempt.execution_id
            record.attempt_index = attempt.attempt_index
            record.kind = attempt.kind
            record.state = {
                "STARTED": AttemptUsage.STARTED,
                "FINISHED": AttemptUsage.FINISHED,
                "UNCERTAIN": AttemptUsage.UNCERTAIN,
            }.get(attempt.state, AttemptUsage.UNSPECIFIED)
            record.elapsed_s = float(attempt.elapsed_s)
            usage.append(record)
        return usage

    def _finish(self, goal_handle, execution, result: ExecutionResult):
        """결과를 저장하고 Action을 종료한다."""

        if execution.attempts and execution.attempts[-1].state == "STARTED":
            attempt_state = "UNCERTAIN" if result.outcome == "UNCERTAIN" else "FINISHED"
            self.machine.finish_attempt(
                execution.goal.execution_id,
                attempt_state,
                time.monotonic(),
                result.error_code,
            )
        final = self.machine.finish(execution.goal.execution_id, result)
        message = self._result_message(final)
        if final.outcome == "SUCCEEDED":
            goal_handle.succeed()
        elif final.outcome == "CANCELED":
            goal_handle.canceled()
        else:
            goal_handle.abort()
        self.active_goal = None
        return message

    def _run_one_attempt(self, goal: GoalSnapshot, goal_handle, execution) -> ExecutionResult:
        """현재 예산으로 PICK 또는 PLACE 한 번을 실행한다.

        부착 결과를 확인하지 못하면 성공으로 올리지 않고 UNCERTAIN으로 남긴다.
        이 함수는 차체를 움직이거나 자동 RELEASE를 수행하지 않는다.
        """

        if goal.operation == ManipulateObject.Goal.PICK:
            execution.phase = Phase.PICKING
            self._publish_feedback(goal_handle, Phase.PICKING, 0.3, execution)
            ok, reason = self._call_arm(goal, goal_handle, execution)
            if not ok:
                outcome = "UNCERTAIN" if reason == "CANCEL_UNCERTAIN" else (
                    "CANCELED" if reason == "CANCELED" else "FAILED")
                return ExecutionResult(outcome, reason, "A4 arm execution failed")
            execution.phase = Phase.ATTACHING
            self._publish_feedback(goal_handle, Phase.ATTACHING, 0.55, execution)
            ok, reason = self._call_tool(
                goal, ToolCommand.Request.ATTACH, goal_handle, execution)
            if not ok:
                outcome = "UNCERTAIN" if reason == "CANCEL_UNCERTAIN" else (
                    "CANCELED" if reason == "CANCELED" else "FAILED")
                return ExecutionResult(outcome, reason, "A2 rejected attach")
            execution.phase = Phase.VERIFYING_PICK
            ok, reason = self._verify_attach(goal, True)
            if not ok:
                self._publish_hold(goal, C4StateMachine.UNKNOWN_HOLD, reason)
                return ExecutionResult(
                    "UNCERTAIN", reason, "PICK result is not confirmed",
                    need_reobservation_move=True,
                    observation_reason="NEED_REOBSERVATION_MOVE",
                )
            self._publish_hold(goal, C4StateMachine.HELD, reason)
            return ExecutionResult("SUCCEEDED", hold_state=C4StateMachine.HELD)

        execution.phase = Phase.PLACING
        self._publish_feedback(goal_handle, Phase.PLACING, 0.45, execution)
        ok, reason = self._call_arm(goal, goal_handle, execution)
        if not ok:
            outcome = "UNCERTAIN" if reason == "CANCEL_UNCERTAIN" else (
                "CANCELED" if reason == "CANCELED" else "FAILED")
            return ExecutionResult(outcome, reason, "A4 arm execution failed")
        execution.phase = Phase.RELEASING
        self._publish_feedback(goal_handle, Phase.RELEASING, 0.7, execution)
        ok, reason = self._call_tool(
            goal, ToolCommand.Request.RELEASE, goal_handle, execution)
        if not ok:
            outcome = "UNCERTAIN" if reason == "CANCEL_UNCERTAIN" else (
                "CANCELED" if reason == "CANCELED" else "FAILED")
            return ExecutionResult(outcome, reason, "A2 rejected release")
        execution.phase = Phase.VERIFYING_PLACE
        ok, reason = self._verify_attach(goal, False)
        if not ok:
            self._publish_hold(goal, C4StateMachine.UNKNOWN_HOLD, reason)
            return ExecutionResult(
                "UNCERTAIN", reason, "PLACE result is not confirmed",
                need_reobservation_move=True,
                observation_reason="NEED_REOBSERVATION_MOVE",
            )
        self._publish_hold(goal, C4StateMachine.RELEASED, reason)
        return ExecutionResult("SUCCEEDED", hold_state=C4StateMachine.RELEASED)

    def _execute(self, goal_handle):
        """PICK/PLACE 전체 실행 순서.

        순서가 중단되면 새 하위 명령을 만들지 않고 실패 이유를 결과로
        남긴다. 취소 시에는 안전 정책상 자동 RELEASE를 하지 않는다.
        """

        goal = self._snapshot(goal_handle)
        execution, cached, validation = self.machine.accept(goal)
        if cached is not None:
            message = self._result_message(cached)
            if cached.outcome == "SUCCEEDED":
                goal_handle.succeed()
            elif cached.outcome == "CANCELED":
                goal_handle.canceled()
            else:
                goal_handle.abort()
            return message
        if execution is None:
            goal_handle.abort()
            return self._result_message(ExecutionResult("FAILED", validation.reason_code, validation.detail))

        self.active_goal = goal_handle
        self._publish_feedback(goal_handle, Phase.CHECKING, 0.05, execution)
        if goal.has_candidate_preference:
            return self._finish(goal_handle, execution, ExecutionResult(
                "FAILED", "CANDIDATE_NOT_CONNECTED",
                "candidate preference validation is not connected to C3"))
        if not self.mock_execution:
            return self._finish(goal_handle, execution, ExecutionResult(
                "FAILED", "TRAJECTORY_UNAVAILABLE",
                "C4 has no contract-defined trajectory provider"))

        execution.phase = Phase.REQUESTING_LEASE
        self._publish_feedback(goal_handle, Phase.REQUESTING_LEASE, 0.1, execution)
        grant = self._request_lease(goal, execution)
        if grant is None:
            return self._finish(goal_handle, execution, ExecutionResult(
                "UNCERTAIN", "LEASE_RESYNC_UNCERTAIN", "A3 lease response was not received"))
        lease_check = self.machine.set_lease(
            goal.execution_id, grant, time.monotonic())
        if not lease_check.ok:
            return self._finish(goal_handle, execution, ExecutionResult(
                "FAILED", lease_check.reason_code, lease_check.detail))
        if execution.revoke_reason:
            return self._finish(goal_handle, execution, ExecutionResult(
                "FAILED", execution.revoke_reason, "lease was revoked before execution"))
        if goal_handle.is_cancel_requested:
            self.machine.request_cancel(goal.execution_id)
            return self._finish(goal_handle, execution, ExecutionResult(
                "CANCELED", "CANCELED", "cancelled before physical execution"))

        # 물리 시도마다 남은 lease와 budget을 확인한다. 실패가 확정된 경우에만
        # 남은 GRASP_ATTEMPT 예산으로 재시도하고, UNKNOWN/CANCELED는 재시도하지 않는다.
        while True:
            runtime = self._ensure_runtime(goal, execution)
            if not runtime.ok:
                return self._finish(goal_handle, execution, ExecutionResult(
                    "UNCERTAIN" if runtime.reason_code == "LEASE_RESYNC_UNCERTAIN" else "FAILED",
                    runtime.reason_code, runtime.detail))
            attempt_check = self.machine.start_attempt(
                goal.execution_id, time.monotonic())
            if not attempt_check.ok:
                return self._finish(goal_handle, execution, ExecutionResult(
                    "FAILED", attempt_check.reason_code, attempt_check.detail))
            self.attempt_started_ros_time = self.get_clock().now().to_msg()
            execution.attempts[-1] = replace(
                execution.attempts[-1],
                started_at_ros_s=(
                    self.attempt_started_ros_time.sec
                    + self.attempt_started_ros_time.nanosec / 1e9),
            )
            result = self._run_one_attempt(goal, goal_handle, execution)
            if result.outcome != "FAILED":
                return self._finish(goal_handle, execution, result)

            remaining_attempts = any(
                budget.kind == "GRASP_ATTEMPT"
                and goal.execution_id in budget.allowed_execution_ids
                and execution.attempt_count < budget.granted_attempts
                for budget in goal.budgets
            )
            if not remaining_attempts:
                return self._finish(goal_handle, execution, result)
            # A lease 거부·권한 오류는 재시도하지 않는다. 물리 하위 동작이
            # 명확히 실패했고 lease가 유효한 경우에만 다음 예산을 사용한다.
            if result.error_code in {
                "LEASE_EXPIRED", "LEASE_REVOKED", "GOAL_MISMATCH",
                "AUTHORITY_INVALID", "TRAJECTORY_UNAVAILABLE",
            }:
                return self._finish(goal_handle, execution, result)


def main(args=None):
    """ROS2 실행 진입점."""

    rclpy.init(args=args)
    node = ManipulateObjectServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
