#!/usr/bin/env python3
"""A2 개발용 흡착 부착·해제 점검. A3·A4 모의 상대를 띄우고 상자로 시험한다.

모의 상대는 이 시험 안에서만 쓴다. 실제 lease 판정이나 구동 차단이 아니다.
상자와 받침대를 월드에 만들고 팔을 움직이므로 격리된 개발 시뮬레이션에서만 실행한다.

실행 순서
  터미널 1: ros2 launch ax_control simulation.launch.py robot_ids:=robot_1
  터미널 2: python3 src/ax_control/scripts/check_tool_attach.py --robot-id robot_1

주의
- 실제 A3·A4가 떠 있으면 같은 서비스·토픽이 겹치므로 그 환경에서 실행하지 않는다.
- 상자 이름이 고정이라 같은 시뮬레이션에서 두 번 실행할 수 없다. 재시작 후 실행한다.
- 판정은 [PASS]/[FAIL]로 출력하고 실패가 있으면 종료 코드 1을 돌려준다.

시험 항목
  초기 상태, 먼 물체, A4 불허, A3 거부, A3 무응답, 부착, 재전송, 인자 다른 재전송,
  들어 올림, 팔 추종, 안전 정지 중 유지, 안전 정지 중 해제 거부, 다른 물체 해제, 해제
"""

import argparse
import re
import subprocess
import sys
import threading
import time
import uuid

import rclpy
import tf2_ros
from ax_interfaces.msg import ActuationState, AttachState
from ax_interfaces.srv import CheckCommandLease, ToolCommand
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from trajectory_msgs.msg import JointTrajectoryPoint

ARM_JOINTS = [
    "column_yaw_joint",
    "shoulder_joint",
    "elbow_joint",
    "wrist_joint",
    "gripper_rotation_joint",
]
# 공구가 차체 옆(+Y)으로 나오는 자세. 받침대가 차체와 겹치지 않는다.
PICK_YAW_RAD = 0.64
CARRY_YAW_RAD = 0.0
# 어깨 양의 방향이 공구를 올린다. 받침대에 닿은 채 수평으로 끌지 않도록 먼저 들어 올린다.
LIFT_SHOULDER_RAD = 0.25
MIN_LIFT_M = 0.05

# 시험 물체 치수와 판정 여유값(m). 시험용 값이며 실제 물품 규격이 아니다.
BOX_SIZE_M = 0.15
STAND_SIZE_M = 0.25
# 상자 중심과 TCP 사이 거리. 플러그인 max_attach_distance보다 작아야 한다.
BOX_BELOW_TCP_M = 0.085
FLOOR_BOX_Z_M = BOX_SIZE_M / 2.0
MOVE_TOLERANCE_M = 0.05
ATTACH_TIMEOUT_SECONDS = 3.0
# 모의 A3는 내용을 검사하지 않지만 계약 형식(소문자 16진수 32자)은 맞춘다.
GOAL_ID = "0123456789abcdef0123456789abcdef"

# 상자와 받침대가 같이 쓰는 SDF. 플러그인이 첫 링크를 쓰므로 링크는 하나만 둔다.
BOX_SDF = """<sdf version="1.9"><model name="{name}"><static>{static}</static>
<link name="link"><inertial><mass>1.0</mass><inertia><ixx>0.004</ixx><iyy>0.004</iyy>
<izz>0.004</izz></inertia></inertial>
<collision name="c"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>
<visual name="v"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></visual>
</link></model></sdf>"""


# 월드에 상자 모델을 만든다
def spawn_box(name, position, size, static):
    sdf = BOX_SDF.format(
        name=name,
        static=str(static).lower(),
        sx=size[0],
        sy=size[1],
        sz=size[2],
    )

    subprocess.run(
        [
            "ros2", "run", "ros_gz_sim", "create",
            "-world", "ax_test",
            "-string", sdf,
            "-name", name,
            "-x", str(position[0]),
            "-y", str(position[1]),
            "-z", str(position[2]),
        ],
        check=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        timeout=20,
    )


# Gazebo에서 모델의 실제 위치를 읽는다
def model_position(name):
    output = subprocess.run(
        ["gz", "model", "-m", name, "-p"],
        capture_output=True,
        text=True,
        timeout=10,
    ).stdout

    # 출력의 첫 대괄호가 XYZ, 다음이 RPY다.
    match = re.search(r"Pose.*?\n\s*\[([^\]]+)\]", output, re.S)

    if match is None:
        raise RuntimeError(f"{name} 위치를 읽지 못함: {output[:120]}")

    return [float(value) for value in match.group(1).split()]


# A3·A4 모의 상대와 시험 클라이언트를 겸하는 노드
class ToolAttachTester(Node):
    # 모의 서비스·토픽과 시험 클라이언트를 연결한다
    def __init__(self, robot_id):
        super().__init__(
            "check_tool_attach",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )

        self.robot_id = robot_id
        self.namespace = f"/ax/{robot_id}"

        # 시나리오가 이 값을 바꿔 A3·A4 응답을 조작한다.
        self.arm_allowed = True
        self.lease_allowed = True
        self.lease_reason = "LEASE_EXPIRED"

        # 재전송이 A3를 다시 호출하지 않는지 세는 값이다.
        self.lease_calls = 0
        self.attach_state = None

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.actuation_publisher = self.create_publisher(
            ActuationState,
            f"{self.namespace}/safety/actuation_state",
            qos,
        )

        self.create_timer(0.05, self.publish_actuation)

        self.create_subscription(
            AttachState,
            f"{self.namespace}/manipulation/attach_state",
            self.on_attach_state,
            qos,
        )

        self.lease_server = self.start_lease_server()

        self.tool_client = self.create_client(
            ToolCommand,
            f"{self.namespace}/manipulation/tool_command",
        )

        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            f"{self.namespace}/arm_controller/follow_joint_trajectory",
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    # 모의 A3 서비스를 시작한다
    def start_lease_server(self):
        return self.create_service(
            CheckCommandLease,
            f"{self.namespace}/executor/lease/check_command",
            self.on_lease,
        )

    # 모의 A3 응답을 만든다
    def on_lease(self, request, response):
        self.lease_calls += 1

        response.allowed = self.lease_allowed
        response.reason_code = "" if self.lease_allowed else self.lease_reason
        response.accept_record_id = request.command_id
        response.valid_for_s = 2.0

        return response

    # 모의 A4 구동 상태를 발행한다
    def publish_actuation(self):
        message = ActuationState()
        message.meta.header.stamp = self.get_clock().now().to_msg()
        message.meta.schema_version = 1
        message.meta.source_id = "mock_a4"
        message.meta.valid = True
        message.robot_id = self.robot_id
        message.state = ActuationState.RUNNING
        message.drive_allowed = True
        message.arm_allowed = self.arm_allowed

        self.actuation_publisher.publish(message)

    # 최신 attach_state를 저장한다
    def on_attach_state(self, message):
        self.attach_state = message

    # 공구 명령을 보내고 응답을 돌려준다
    def send_tool(self, command, object_id, request_id=None, generation=1):
        request = ToolCommand.Request()
        request.request_id = request_id or uuid.uuid4().hex
        request.robot_id = self.robot_id
        request.execution_id = "exec_test"
        request.goal_id = GOAL_ID
        request.object_id = object_id
        request.lease_generation = generation
        request.command = command

        done = threading.Event()
        future = self.tool_client.call_async(request)
        future.add_done_callback(lambda _: done.set())

        if not done.wait(5.0):
            raise RuntimeError("tool_command 응답 없음")

        return future.result()

    # attach_state가 조건을 만족할 때까지 기다린다
    def wait_attached(self, attached, object_id=""):
        deadline = time.monotonic() + ATTACH_TIMEOUT_SECONDS

        while time.monotonic() < deadline:
            state = self.attach_state

            if (
                state is not None
                and state.meta.valid
                and state.attached == attached
                and state.object_id == object_id
            ):
                return True

            time.sleep(0.02)

        return False

    # 기둥 회전과 어깨를 움직인다
    def move_arm(self, yaw, shoulder=0.0):
        point = JointTrajectoryPoint()
        point.positions = [yaw, shoulder, 0.0, 0.0, 0.0]
        point.time_from_start = Duration(seconds=2).to_msg()

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS
        goal.trajectory.points = [point]

        done = threading.Event()
        send_future = self.arm_client.send_goal_async(goal)
        send_future.add_done_callback(lambda _: done.set())

        if not done.wait(5.0) or not send_future.result().accepted:
            raise RuntimeError("팔 궤적 거절")

        done.clear()
        result_future = send_future.result().get_result_async()
        result_future.add_done_callback(lambda _: done.set())

        if not done.wait(10.0):
            raise RuntimeError("팔 궤적 결과 없음")

    # 공구 TCP의 world 좌표를 구한다. 빈 월드에서 odom과 world가 같다고 본다
    # 차체 world 위치에 base_link 기준 TCP 위치를 더한다. 차체 회전은 0이라고 가정한다.
    # 이 시험은 주행하지 않으므로 맞다. 주행 후 쓰려면 회전을 반영해야 한다.
    def tcp_position(self):
        base = model_position(self.robot_id)
        frame = f"{self.robot_id}/epick_tcp"
        parent = f"{self.robot_id}/base_link"
        deadline = time.monotonic() + 10.0

        while time.monotonic() < deadline:
            try:
                transform = self.tf_buffer.lookup_transform(
                    parent,
                    frame,
                    rclpy.time.Time(),
                )
            except tf2_ros.TransformException:
                time.sleep(0.1)
                continue

            offset = transform.transform.translation

            return [base[0] + offset.x, base[1] + offset.y, base[2] + offset.z]

        raise RuntimeError("epick_tcp TF 없음")


# 시나리오를 실행하고 실패 건수를 돌려준다
def run_checks(node):
    failures = 0
    attach = ToolCommand.Request.ATTACH
    release = ToolCommand.Request.RELEASE

    # 판정 한 줄을 출력하고 실패를 센다
    def report(name, passed, detail):
        nonlocal failures

        failures += 0 if passed else 1
        print(f"[{'PASS' if passed else 'FAIL'}] {name}: {detail}", flush=True)

    # 준비 확인. 모의 A4 상태가 공구 노드에 도착할 시간도 준다.
    if not node.tool_client.wait_for_service(timeout_sec=15.0):
        report("tool_command", False, "서비스 없음")

        return failures

    node.arm_client.wait_for_server(timeout_sec=10.0)
    time.sleep(1.0)

    report(
        "attach_state 초기",
        node.wait_attached(False),
        "유효, attached=false",
    )

    node.move_arm(PICK_YAW_RAD)
    time.sleep(1.0)

    # 시험 무대: TCP 바로 아래 받침대 위 상자(box_11)와 멀리 떨어진 바닥 상자(box_12)
    tcp = node.tcp_position()
    box_z = tcp[2] - BOX_BELOW_TCP_M
    stand_height = box_z - BOX_SIZE_M / 2.0 - 0.002

    spawn_box(
        "test_stand",
        [tcp[0], tcp[1], stand_height / 2.0],
        [STAND_SIZE_M, STAND_SIZE_M, stand_height],
        True,
    )
    spawn_box("box_11", [tcp[0], tcp[1], box_z], [BOX_SIZE_M] * 3, False)
    spawn_box("box_12", [1.5, 1.5, FLOOR_BOX_Z_M], [BOX_SIZE_M] * 3, False)
    time.sleep(2.0)

    # 먼 물체는 수락돼도 실제로 붙지 않아야 한다.
    response = node.send_tool(attach, "box_12")
    report(
        "먼 물체",
        response.accepted and not node.wait_attached(True, "box_12"),
        f"수락 {response.accepted}, attached 없음",
    )

    # 거절 시나리오: A4 불허, A3 거부, A3 무응답 순서로 확인한다.
    node.arm_allowed = False
    time.sleep(0.2)
    response = node.send_tool(attach, "box_11")
    report("arm_allowed 거부", not response.accepted, f"사유 {response.reason_code}")
    node.arm_allowed = True
    time.sleep(0.2)

    node.lease_allowed = False
    response = node.send_tool(attach, "box_11")
    report("lease 거부", not response.accepted, f"사유 {response.reason_code}")
    node.lease_allowed = True

    # 서비스를 지워 A3가 사라진 상황을 만든다.
    node.destroy_service(node.lease_server)
    time.sleep(0.3)
    response = node.send_tool(attach, "box_11")
    report("A3 무응답 거부", not response.accepted, f"사유 {response.reason_code}")
    node.lease_server = node.start_lease_server()
    time.sleep(1.0)

    # 정상 부착. 이후 재전송 시험을 위해 request_id를 저장해 둔다.
    request_id = uuid.uuid4().hex
    response = node.send_tool(attach, "box_11", request_id=request_id)
    attached = node.wait_attached(True, "box_11")
    report("부착", response.accepted and attached, f"수락 {response.accepted}, 신호 {attached}")

    calls = node.lease_calls
    response = node.send_tool(attach, "box_11", request_id=request_id)
    report(
        "재전송",
        response.accepted and node.lease_calls == calls,
        f"같은 응답, 추가 lease 확인 {node.lease_calls - calls}회",
    )

    response = node.send_tool(attach, "box_11", request_id=request_id, generation=2)
    report("인자 다른 재전송", not response.accepted, f"사유 {response.reason_code}")

    # 신호만 믿지 않고 Gazebo 실제 위치로 물체가 팔을 따라오는지 확인한다.
    before = model_position("box_11")
    node.move_arm(PICK_YAW_RAD, LIFT_SHOULDER_RAD)
    time.sleep(1.0)
    lifted = model_position("box_11")
    report(
        "들어 올림",
        lifted[2] - before[2] > MIN_LIFT_M,
        f"상승 {lifted[2] - before[2]:.3f} m",
    )

    node.move_arm(CARRY_YAW_RAD, LIFT_SHOULDER_RAD)
    time.sleep(1.0)
    after = model_position("box_11")
    moved = sum((a - b) ** 2 for a, b in zip(after, lifted)) ** 0.5
    report(
        "팔 추종",
        moved > MOVE_TOLERANCE_M and after[2] > FLOOR_BOX_Z_M + 0.5,
        f"이동 {moved:.3f} m, 높이 {after[2]:.3f} m",
    )

    # 안전 정지 상태를 흉내 낸다. 물체가 자동으로 떨어지면 안 된다.
    node.arm_allowed = False
    time.sleep(1.5)
    held = model_position("box_11")
    state = node.attach_state
    report(
        "안전 정지 중 유지",
        state.attached and held[2] > FLOOR_BOX_Z_M + 0.5,
        f"attached {state.attached}, 높이 {held[2]:.3f} m",
    )

    response = node.send_tool(release, "box_11")
    report("안전 정지 중 해제 거부", not response.accepted, f"사유 {response.reason_code}")
    node.arm_allowed = True
    time.sleep(0.2)

    # 들고 있지 않은 물체 해제는 수락돼도 box_11을 놓지 않아야 한다.
    response = node.send_tool(release, "box_12")
    report(
        "다른 물체 해제 무시",
        node.wait_attached(True, "box_11"),
        f"수락 {response.accepted}, box_11 유지",
    )

    # 해제 신호와 실제 낙하를 함께 확인한다.
    response = node.send_tool(release, "box_11")
    released = node.wait_attached(False)
    time.sleep(2.0)
    final = model_position("box_11")
    report(
        "해제",
        response.accepted and released and final[2] < FLOOR_BOX_Z_M + 0.05,
        f"신호 {released}, 바닥 높이 {final[2]:.3f} m",
    )

    return failures


# 명령행 인자를 읽어 점검을 실행한다
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-id", default="robot_1")
    arguments = parser.parse_args()

    rclpy.init()

    node = ToolAttachTester(arguments.robot_id)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    # 시나리오가 순차로 대기하므로 콜백은 별도 스레드에서 처리한다.
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    try:
        failures = run_checks(node)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

    print(f"실패 {failures}건", flush=True)
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
