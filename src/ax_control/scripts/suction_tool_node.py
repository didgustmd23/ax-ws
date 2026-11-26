#!/usr/bin/env python3
"""A2 흡착 공구 노드. tool_command를 받아 Gazebo 플러그인에 전달하고 attach_state를 발행한다.

수락은 부착·해제 완료가 아니다. 실제 결과는 플러그인 상태로 만든 attach_state가 알린다.
attached는 원시 신호이며 보유 판정은 C4 담당이다. 압력값을 만들지 않는다.
안전 정지나 A3 무응답으로 물체를 자동 해제하지 않는다. 해제는 명시적 RELEASE로만 한다.

담당: A2. simulation.launch.py가 로봇마다 하나씩 실행한다.

팀원별 연결 방법
- C4(요청자): tool_command를 호출하고 attach_state로 결과를 확인한다.
  accepted=true만 보고 성공 처리하면 안 된다. 물체가 멀면 accepted=true여도 붙지 않는다.
  object_id는 Gazebo 모델 이름과 같아야 하며 공백을 쓸 수 없다.
  부착 뒤에는 먼저 들어 올린다. 받침면에 닿은 채 수평으로 끌면 팔이 움직이지 않았다.
- A3: /ax/<robot_id>/executor/lease/check_command 서버를 제공해야 한다.
  command_source는 SOURCE_A2_TOOL, command_id는 ToolCommand.request_id로 온다.
- A4: /ax/<robot_id>/safety/actuation_state를 20 Hz로 발행해야 한다.
  없거나 0.3 s 넘게 끊기면 모든 요청이 거절된다.
  ActuationState는 Schema 2만 받는다. 자기 robot_id, 비어 있지 않은 source_id·
  source_session_id, revision > 0, 유한·양수 stamp가 필요하다. 같은 세션에서
  revision·stamp가 줄면 버리고, 새 세션이 오면 이전 세션은 다시 받지 않는다.
  valid=false도 받아 즉시 거절 상태로 만든다. 허가는 arm_allowed만 보며
  STOPPED는 허가로 쓰지 않는다. 미래 stamp는 그 순간의 평가만 거절한다.

수동 호출 예(A3·A4가 있거나 check_tool_attach.py의 모의 상대가 있을 때):
  ros2 service call /ax/robot_1/manipulation/tool_command ax_interfaces/srv/ToolCommand \\
    "{request_id: r1, robot_id: robot_1, execution_id: e1,
      goal_id: 0123456789abcdef0123456789abcdef, object_id: box_11,
      lease_generation: 1, command: 1}"
  command 1은 ATTACH, 2는 RELEASE다.

처리 순서
  형식 검사 → arm_allowed → A3 확인 → arm_allowed 재확인 → 플러그인에 명령 전달

알려진 제한
- arm_allowed 거부(SAFETY_STOP)는 계약에 원인별 코드가 없어 임시로 지정했다(아래 REASON_ 상수).
- A3 확인 거절 사유: 무응답(서비스 미준비·0.5 s 무응답)은 A3_LEASE_TIMEOUT, A3 거절은 A3가 준 사유 그대로
  (빈 사유면 LEASE_EXPIRED), 허용 뒤 기한 경과는 LEASE_EXPIRED(계약 D-13, A4와 같음).
- 플러그인이 보고한 실패 원인(too_far 등)은 아직 attach_state로 전달하지 않는다.
- responses 기록은 프로세스가 끝나면 사라지고 크기 제한도 없다.
"""

import hashlib
import math
import threading
import time
import uuid

import rclpy
from ax_interfaces.msg import ActuationState, AttachState
from ax_interfaces.srv import CheckCommandLease, ToolCommand
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

# 계약 D-05의 구동 상태·부착 신호 만료 시간(s)
STATE_EXPIRY_SECONDS = 0.3
# A3 응답 대기 한도(s). 넘기면 무응답으로 보고 거절한다. 계약 값이 아닌 개발용 값이다
LEASE_TIMEOUT_SECONDS = 0.5
# attach_state 발행 주기(s). 계약의 20 Hz다
ATTACH_STATE_PERIOD_SECONDS = 0.05
SCHEMA_VERSION = 1
# A4 ActuationState가 따라야 하는 schema 버전(W2 Schema 2 전환)
ACTUATION_SCHEMA_VERSION = 2

# 계약 D-12·D-13에 실패 원인별 코드가 없어 구동 차단 코드 중 가장 보수적인 값을 쓴다.
REASON_ARM_NOT_ALLOWED = "SAFETY_STOP"
# A3 무응답(서비스 미준비·응답 대기 한도 초과). 계약 D-13, A4 보호 팔 서버와 같은 코드(2026-11-24)
REASON_LEASE_NO_RESPONSE = "A3_LEASE_TIMEOUT"
# 허용 뒤 기한(t_first + valid_for_s) 경과, 또는 A3가 사유 없이 거절한 경우(A4와 같은 대체값)
REASON_LEASE_EXPIRED = "LEASE_EXPIRED"
REASON_DUPLICATE = "DUPLICATE_REQUEST"
REASON_INVALID = "AUTHORITY_INVALID"


# 부팅 이후 단조 시각을 돌려준다. 계약의 무수신 시간·기한 계산 기준이다
def boottime_seconds():
    return time.clock_gettime(time.CLOCK_BOOTTIME)


# 명령 인자의 SHA-256을 계산한다
# 같은 request_id에 다른 인자가 왔는지 구분하는 데 쓰고 A3에도 그대로 보낸다.
def command_digest(request):
    fields = [
        request.robot_id,
        request.execution_id,
        request.goal_id,
        request.object_id,
        str(request.lease_generation),
        str(request.command),
    ]

    return hashlib.sha256("|".join(fields).encode("utf-8")).hexdigest()


# 상태 Topic용 QoS를 만든다
# 계약의 STATE 프로필이다. 늦게 접속한 노드도 마지막 값을 받는다.
def state_qos():
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


# A4 ActuationState의 수신 단계 검사를 맡는다(ROS 비의존)
# 수용 여부만 판단하고 허가 여부(arm_allowed)·신선도는 판단하지 않는다.
# valid=false는 받아서 무효 상태로 덮어쓴다(fail-closed). 미래 stamp는 여기서 거르지 않는다.
class ActuationScreen:
    # 자기 로봇 이름과 세션 추적 상태를 준비한다
    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.session = None
        self.revision = 0
        self.stamp = 0.0
        self.retired_sessions = set()

    # 메시지를 검사해 거부 사유를 돌려준다. 빈 문자열이면 수용이며 추적 상태를 갱신한다
    def check(self, message):
        meta = message.meta
        stamp = meta.header.stamp.sec + meta.header.stamp.nanosec / 1e9

        # 메시지 자체의 식별·형식 검사
        if message.robot_id != self.robot_id:
            return "ROBOT_MISMATCH"

        if meta.schema_version != ACTUATION_SCHEMA_VERSION:
            return "SCHEMA_MISMATCH"

        if meta.source_id == "" or meta.source_session_id == "":
            return "SOURCE_EMPTY"

        if meta.revision == 0:
            return "REVISION_ZERO"

        if not math.isfinite(stamp) or stamp <= 0.0:
            return "STAMP_INVALID"

        # 세션 순서 검사. 은퇴한 세션은 다시 받지 않는다.
        session = meta.source_session_id

        if session in self.retired_sessions:
            return "SESSION_RETIRED"

        if session == self.session and (meta.revision < self.revision or stamp < self.stamp):
            return "REVISION_REGRESSED"

        # 수용: 새 세션이면 이전 세션을 은퇴시키고 기준값을 갱신한다.
        if self.session is not None and session != self.session:
            self.retired_sessions.add(self.session)

        self.session = session
        self.revision = meta.revision
        self.stamp = stamp

        return ""


# 공구 명령 수락 조건 확인과 부착 상태 발행을 맡는 노드
# 서비스 콜백이 A3 응답을 기다리는 동안 다른 콜백이 돌아야 해서 MultiThreadedExecutor로 실행한다.
class SuctionToolNode(Node):
    # 파라미터를 읽고 서비스·토픽을 연결한다
    def __init__(self):
        super().__init__("suction_tool_node")

        self.declare_parameter("robot_id", "robot_1")
        self.declare_parameter("tool_id", "epick")

        self.robot_id = self.get_parameter("robot_id").value
        self.tool_id = self.get_parameter("tool_id").value

        namespace = f"/ax/{self.robot_id}"

        # 서비스 처리 중 A3 응답을 받아야 해서 재진입 가능한 콜백 그룹에 묶는다.
        group = ReentrantCallbackGroup()

        # 콜백이 여러 스레드에서 돌기 때문에 아래 공유 상태는 lock으로 보호한다.
        self.lock = threading.Lock()

        # request_id → (digest, 수락 여부, 사유). 재전송에 같은 응답을 주기 위한 기록이다.
        self.responses = {}

        self.actuation = None
        self.actuation_received = None

        # Schema 2 수신 검사와 거부 로그 억제용 직전 사유
        self.actuation_screen = ActuationScreen(self.robot_id)
        self.actuation_reject_reason = ""

        # arm_allowed 평가 시 미래 stamp로 거절한 횟수(교차 프로세스 /clock 지연 관측용)
        self.future_stamp_denials = 0
        self.future_stamp_last = False

        # 플러그인이 마지막으로 보고한 값. 플러그인 상태만이 attached의 근거다.
        self.attached = False
        self.object_id = ""
        self.plugin_received = None

        # StateMeta용 값. 프로세스마다 새 세션이고 상태가 바뀔 때마다 revision이 오른다.
        self.session_id = uuid.uuid4().hex
        self.revision = 0
        self.last_published = None

        self.gz_command = self.create_publisher(
            String,
            f"{namespace}/tool/gz_command",
            10,
        )

        self.attach_publisher = self.create_publisher(
            AttachState,
            f"{namespace}/manipulation/attach_state",
            state_qos(),
        )

        self.create_subscription(
            String,
            f"{namespace}/tool/gz_state",
            self.on_plugin_state,
            10,
        )

        self.create_subscription(
            ActuationState,
            f"{namespace}/safety/actuation_state",
            self.on_actuation,
            state_qos(),
        )

        self.lease_client = self.create_client(
            CheckCommandLease,
            f"{namespace}/executor/lease/check_command",
            callback_group=group,
        )

        self.create_service(
            ToolCommand,
            f"{namespace}/manipulation/tool_command",
            self.on_tool_command,
            callback_group=group,
        )

        # 재발행 주기는 /clock 정지·시스템 시계 역행과 무관하게 STEADY로 돌리고, stamp는 ROS(sim) 시간을 유지한다.
        self.create_timer(ATTACH_STATE_PERIOD_SECONDS, self.publish_attach_state,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))

    # 플러그인이 보고한 실제 연결 상태를 저장한다
    def on_plugin_state(self, message):
        # 형식: "<0|1> <물체 또는 -> <사건>". 형식이 맞지 않으면 이전 값을 유지한다.
        fields = message.data.split()

        if len(fields) < 2:
            return

        with self.lock:
            self.attached = fields[0] == "1"
            self.object_id = fields[1] if self.attached else ""
            self.plugin_received = boottime_seconds()

    # A4 구동 상태를 검사해 수용한 메시지만 수신 시각과 함께 저장한다
    # 거부 메시지는 만료를 늦추지 않는다. 로그는 사유가 바뀔 때만 남긴다.
    def on_actuation(self, message):
        with self.lock:
            reason = self.actuation_screen.check(message)
            previous = self.actuation_reject_reason
            self.actuation_reject_reason = reason

            if reason == "":
                self.actuation = message
                self.actuation_received = boottime_seconds()

        # 20 Hz 수신이라 사유 변화 시점에만 기록한다.
        if reason != previous:
            if reason:
                self.get_logger().warn(
                    f"ActuationState 거부: {reason} "
                    f"(robot_id={message.robot_id}, schema={message.meta.schema_version}, "
                    f"source={message.meta.source_id}, revision={message.meta.revision})"
                )
            else:
                self.get_logger().info(f"ActuationState 다시 수용 (이전 거부 {previous})")

    # 구동 상태가 유효·최신이고 팔 동작을 허용하는지 확인한다
    # 허가는 arm_allowed만 본다. STOPPED는 정지 상태일 뿐 허가가 아니다.
    def arm_allowed(self):
        with self.lock:
            state = self.actuation
            received = self.actuation_received

        # 계약상 무효·미수신·만료는 모두 허용이 아니다.
        if state is None or not state.meta.valid:
            return False

        if state.meta.schema_version != ACTUATION_SCHEMA_VERSION:
            return False

        # 무수신 시간은 단조 시각, 표본 나이는 sim time으로 둘 다 본다.
        if boottime_seconds() - received > STATE_EXPIRY_SECONDS:
            return False

        stamp = state.meta.header.stamp
        sample_time = stamp.sec + stamp.nanosec / 1e9
        now = self.get_clock().now().nanoseconds / 1e9
        age = now - sample_time

        if not math.isfinite(age) or age > STATE_EXPIRY_SECONDS:
            return False

        # 미래 stamp는 이 순간만 거절한다. 메시지는 버리지 않아 시계가 따라오면 다시 허가된다.
        future = age < 0.0

        # 서비스 콜백이 여러 스레드에서 부르므로 집계는 lock 안에서 한다.
        with self.lock:
            changed = future != self.future_stamp_last
            self.future_stamp_last = future

            if future:
                self.future_stamp_denials += 1

            denials = self.future_stamp_denials

        # 거절 상태로 바뀌는 순간에만 기록한다.
        if future and changed:
            self.get_logger().warn(
                f"ActuationState 미래 stamp로 거절 (나이 {age:.4f} s, 누적 {denials})"
            )

        if future:
            return False

        return state.arm_allowed

    # A3에 명령 수락을 확인하고 (허용 여부, 사유)를 돌려준다
    # lease 표는 A3만 가진다. 이 노드는 판단하지 않고 A3의 답을 따른다.
    def check_lease(self, request, digest, first_sent):
        if not self.lease_client.service_is_ready():
            return False, REASON_LEASE_NO_RESPONSE

        # 권한 필드는 받은 값을 바꾸지 않고 그대로 넘긴다(계약 5.2절).
        lease = CheckCommandLease.Request()
        lease.request_id = request.request_id
        lease.robot_id = request.robot_id
        lease.authority = CheckCommandLease.Request.AUTH_C_MANIPULATION
        lease.execution_id = request.execution_id
        lease.goal_id = request.goal_id
        lease.lease_generation = request.lease_generation
        lease.phase = CheckCommandLease.Request.ACCEPT
        lease.command_source = CheckCommandLease.Request.SOURCE_A2_TOOL
        lease.command_id = request.request_id
        lease.command_digest = digest

        # 서비스 콜백 안이라 spin을 호출할 수 없어 Event로 응답을 기다린다.
        done = threading.Event()
        future = self.lease_client.call_async(lease)
        future.add_done_callback(lambda _: done.set())

        if not done.wait(LEASE_TIMEOUT_SECONDS):
            future.cancel()

            return False, REASON_LEASE_NO_RESPONSE

        response = future.result()

        # A3 거절 사유는 그대로 전달한다. 빈 사유는 호출자가 구분할 수 있게 LEASE_EXPIRED로 채운다.
        if not response.allowed:
            return False, response.reason_code or REASON_LEASE_EXPIRED

        # 기한은 응답 수신이 아니라 최초 송신 시각부터 계산한다.
        if boottime_seconds() > first_sent + response.valid_for_s:
            return False, REASON_LEASE_EXPIRED

        return True, ""

    # 플러그인에 보낼 명령 문자열을 만든다
    def plugin_command(self, request):
        action = "attach" if request.command == ToolCommand.Request.ATTACH else "release"

        return f"{action} {request.object_id}"

    # 수락 조건을 순서대로 확인해 (수락 여부, 사유)를 돌려준다
    def evaluate(self, request, digest, first_sent):
        commands = (ToolCommand.Request.ATTACH, ToolCommand.Request.RELEASE)

        # 플러그인 명령이 공백으로 나뉘므로 물체 이름에 공백을 허용하지 않는다.
        valid = (
            request.command in commands
            and request.robot_id == self.robot_id
            and request.object_id != ""
            and " " not in request.object_id
        )

        if not valid:
            return False, REASON_INVALID

        # A3에 수락 기록을 남기기 전에 먼저 거른다.
        if not self.arm_allowed():
            return False, REASON_ARM_NOT_ALLOWED

        allowed, reason = self.check_lease(request, digest, first_sent)

        if not allowed:
            return False, reason

        # A3 확인을 기다리는 사이 상태가 바뀔 수 있어 실행 직전에 다시 본다.
        if not self.arm_allowed():
            return False, REASON_ARM_NOT_ALLOWED

        return True, ""

    # tool_command 요청을 처리한다. 같은 request_id는 같은 응답을 돌려준다
    # 멱등 처리: 통신 재시도로 같은 요청이 다시 와도 물리 명령을 두 번 내지 않는다.
    # 같은 request_id에 인자가 다르면 호출자 오류이므로 DUPLICATE_REQUEST로 거절한다.
    def on_tool_command(self, request, response):
        first_sent = boottime_seconds()
        digest = command_digest(request)

        with self.lock:
            cached = self.responses.get(request.request_id)

        if cached is not None:
            if cached[0] != digest:
                response.accepted = False
                response.reason_code = REASON_DUPLICATE

                return response

            response.accepted = cached[1]
            response.reason_code = cached[2]

            return response

        accepted, reason = self.evaluate(request, digest, first_sent)

        # 거절도 기록한다. 같은 request_id는 조건이 바뀌어도 같은 응답을 준다.
        with self.lock:
            self.responses[request.request_id] = (digest, accepted, reason)

        if accepted:
            self.gz_command.publish(String(data=self.plugin_command(request)))

        # accepted는 명령을 넘겼다는 뜻일 뿐이다. 근접 조건 미달이면 붙지 않을 수 있다.
        response.accepted = accepted
        response.reason_code = reason

        return response

    # 플러그인 상태로 attach_state를 만들어 발행한다
    # 명령 수락 여부와 무관하게 플러그인이 보고한 값만 내보낸다.
    def publish_attach_state(self):
        with self.lock:
            attached = self.attached
            object_id = self.object_id
            received = self.plugin_received

        fresh = (
            received is not None
            and boottime_seconds() - received <= STATE_EXPIRY_SECONDS
        )

        # 값이 바뀔 때만 revision을 올린다. 주기 발행만으로는 올리지 않는다.
        current = (attached, object_id, fresh)

        if current != self.last_published:
            self.revision += 1
            self.last_published = current

        message = AttachState()
        message.meta.header.stamp = self.get_clock().now().to_msg()
        message.meta.schema_version = SCHEMA_VERSION
        message.meta.source_id = "a2_suction_tool"
        message.meta.source_session_id = self.session_id
        message.meta.revision = self.revision
        # 플러그인 상태가 끊기면 마지막 값을 보내되 valid=false로 표시한다.
        message.meta.valid = fresh
        message.meta.validity_reason = "" if fresh else "TOOL_STATE_STALE"
        message.robot_id = self.robot_id
        message.tool_id = self.tool_id
        message.object_id = object_id
        message.attached = attached

        self.attach_publisher.publish(message)


# 노드를 다중 스레드로 실행한다. 서비스 처리 중 A3 응답을 받아야 해서 필요하다
def main():
    rclpy.init()

    node = SuctionToolNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
