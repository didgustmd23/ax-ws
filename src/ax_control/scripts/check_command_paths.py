#!/usr/bin/env python3
"""A2 개발용 명령 경로 점검. 시뮬레이션 실행 중 제어기를 직접 구동해 측정한다.

운영 경로가 아니며 A4 게이트를 거치지 않는다. 격리된 개발 시뮬레이션에서만 실행한다.

실행 순서
  터미널 1: ros2 launch ax_control simulation.launch.py robot_ids:=robot_1
  터미널 2: python3 src/ax_control/scripts/check_command_paths.py --robot-id robot_1

점검 항목
  joint_states 수신, 팔 Action 서버, 바퀴 구동, 명령 끊김 후 정지, 팔 궤적 추종,
  없는 관절 거부, TF odom→base_link, 프레임 접두사

A4 담당자는 여기 쓰인 제어기 입력 이름을 게이트·보호 서버의 출력으로 연결하면 된다.
"""

import argparse
import re
import sys
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import TwistStamped
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from trajectory_msgs.msg import JointTrajectoryPoint

ARM_JOINTS = [
    "column_yaw_joint",
    "shoulder_joint",
    "elbow_joint",
    "wrist_joint",
    "gripper_rotation_joint",
]
WHEEL_JOINTS = [
    "front_left_wheel_joint",
    "front_right_wheel_joint",
    "rear_left_wheel_joint",
    "rear_right_wheel_joint",
]
ARM_TARGET = [0.3, 0.2, 0.2, 0.2, 0.3]
ARM_TOLERANCE_RAD = 0.01
WHEEL_STOPPED_RAD_S = 0.01
DRIVE_SPEED_M_S = 0.3
DRIVE_SECONDS = 2.0
STOP_WAIT_SECONDS = 3.0


# 관절 상태와 TF를 수집하며 제어기를 시험하는 노드
class CommandPathChecker(Node):
    # 로봇별 토픽·Action 이름을 계약 경로로 연결한다
    def __init__(self, robot_id):
        super().__init__(
            "check_command_paths",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )

        self.robot_id = robot_id
        namespace = f"/ax/{robot_id}"

        self.joint_state = None
        self.tf_pairs = set()

        self.create_subscription(
            JointState,
            f"{namespace}/joint_states",
            self.on_joint_state,
            10,
        )

        self.create_subscription(TFMessage, "/tf", self.on_tf, 50)

        static_qos = QoSProfile(
            depth=50,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(TFMessage, "/tf_static", self.on_tf, static_qos)

        self.drive_publisher = self.create_publisher(
            TwistStamped,
            f"{namespace}/diff_drive_controller/cmd_vel",
            10,
        )

        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            f"{namespace}/arm_controller/follow_joint_trajectory",
        )

    # 최신 관절 상태를 이름 기준으로 저장한다
    def on_joint_state(self, message):
        self.joint_state = message

    # 관측한 부모·자식 프레임 쌍을 저장한다
    def on_tf(self, message):
        for transform in message.transforms:
            pair = (transform.header.frame_id, transform.child_frame_id)
            self.tf_pairs.add(pair)

    # 벽시계 기준으로 지정 시간 동안 콜백을 처리한다
    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds

        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)

    # 바퀴 속도 절댓값의 최댓값을 구한다
    def max_wheel_speed(self):
        names = list(self.joint_state.name)
        indices = [names.index(joint) for joint in WHEEL_JOINTS]

        speeds = [
            abs(self.joint_state.velocity[index])
            for index in indices
        ]

        return max(speeds)

    # 현재 시뮬레이션 시각을 표시한 전진 명령을 발행한다
    def publish_drive(self, speed):
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.twist.linear.x = speed
        self.drive_publisher.publish(message)

    # 명령을 끊은 뒤 바퀴가 멈출 때까지의 시뮬레이션 시간을 잰다
    def measure_drive_stop(self):
        deadline = time.monotonic() + DRIVE_SECONDS

        while time.monotonic() < deadline:
            self.publish_drive(DRIVE_SPEED_M_S)
            self.spin_for(0.05)

        running_speed = self.max_wheel_speed()
        last_command = self.get_clock().now()
        stop_deadline = time.monotonic() + STOP_WAIT_SECONDS

        while time.monotonic() < stop_deadline:
            rclpy.spin_once(self, timeout_sec=0.01)

            if self.max_wheel_speed() < WHEEL_STOPPED_RAD_S:
                elapsed = self.get_clock().now() - last_command

                return running_speed, elapsed.nanoseconds / 1e9

        return running_speed, None

    # FollowJointTrajectory로 팔을 움직이고 결과와 최종 오차를 돌려준다
    def move_arm(self, joint_names, positions):
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(seconds=3).to_msg()

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joint_names
        goal.trajectory.points = [point]

        accepted_future = self.arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, accepted_future, timeout_sec=10.0)

        handle = accepted_future.result()

        if handle is None or not handle.accepted:
            return False, None, None

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=15.0)

        if result_future.result() is None:
            return True, None, None

        self.spin_for(0.3)

        names = list(self.joint_state.name)
        errors = [
            abs(self.joint_state.position[names.index(joint)] - target)
            for joint, target in zip(joint_names, positions)
        ]

        return True, result_future.result().result.error_code, max(errors)


# 측정 결과를 출력하고 실패 건수를 돌려준다
def run_checks(node):
    failures = 0

    # 판정 한 줄을 출력하고 실패를 센다
    def report(name, passed, detail):
        nonlocal failures

        failures += 0 if passed else 1
        print(f"[{'PASS' if passed else 'FAIL'}] {name}: {detail}", flush=True)

    deadline = time.monotonic() + 15.0

    while node.joint_state is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    if node.joint_state is None:
        report("joint_states", False, "수신 없음")

        return failures

    report("joint_states", True, f"관절 {len(node.joint_state.name)}개")

    server_ready = node.arm_client.wait_for_server(timeout_sec=10.0)
    report("arm_action_server", server_ready, "follow_joint_trajectory")

    running_speed, stop_seconds = node.measure_drive_stop()
    report(
        "drive_running",
        running_speed > 0.5,
        f"명령 중 최대 바퀴 속도 {running_speed:.3f} rad/s",
    )
    report(
        "drive_stop_after_cut",
        stop_seconds is not None,
        f"명령 중단 후 바퀴 정지까지 {stop_seconds} s (시뮬레이션 시간)",
    )

    if server_ready:
        accepted, code, error = node.move_arm(ARM_JOINTS, ARM_TARGET)
        report(
            "arm_trajectory",
            accepted and code == 0 and error is not None and error < ARM_TOLERANCE_RAD,
            f"수락 {accepted}, error_code {code}, 최대 오차 {error} rad",
        )

        accepted, code, _ = node.move_arm(["no_such_joint"], [0.0])
        report(
            "arm_rejects_unknown_joint",
            not accepted or code not in (None, 0),
            f"수락 {accepted}, error_code {code}",
        )

    node.spin_for(1.0)

    robot_id = node.robot_id
    expected = (f"{robot_id}/odom", f"{robot_id}/base_link")
    report("tf_odom_base", expected in node.tf_pairs, f"{expected[0]} -> {expected[1]}")

    # 여러 대가 같은 /tf를 쓰므로 robot_<n>/ 접두사가 붙은 프레임은 모두 허용한다.
    # 접두사가 없는 프레임이 하나라도 있으면 실패다.
    foreign = sorted(
        frame
        for pair in node.tf_pairs
        for frame in pair
        if not re.match(r"robot_\d+/", frame)
    )
    report("tf_prefix_only", not foreign, f"접두사 없는 프레임 {foreign[:5]}")

    return failures


# 명령행 인자를 읽어 점검을 실행한다
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-id", default="robot_1")
    arguments = parser.parse_args()

    rclpy.init()
    node = CommandPathChecker(arguments.robot_id)

    try:
        failures = run_checks(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print(f"실패 {failures}건", flush=True)
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
