#!/usr/bin/env python3
"""A2 개발용 정지 피드백 측정. 제어기 측정값과 Gazebo 실제 속도를 비교한다.

정지 기준은 INTERFACE_CONTRACT D-06이다. 판정 소유자는 A4이며 이 도구는 측정만 한다.
Gazebo 실제 값: 차체는 모델 자세, 팔은 링크 자세로 다시 계산한 관절각의 차분 속도다(gazebo_observer.py).
joint_states와 다른 경로이므로 두 정지 시각을 독립 관측끼리 비교할 수 있다.
Gazebo 실제 값은 시험 비교용이므로 운영 판정에 쓰지 않는다.
제어기를 A4 없이 직접 구동하므로 격리된 개발 시뮬레이션에서만 실행한다.

실행 순서
  터미널 1: ros2 launch ax_control simulation.launch.py robot_ids:=robot_1
  터미널 2: python3 src/ax_control/scripts/check_stop_feedback.py --robot-id robot_1

출력 읽는 법
  [INFO]  피드백 주기·최대 간격·지연과 NaN이 있는 관절
  [PASS]  시나리오별 정지 시간. PASS는 정지가 확인됐다는 뜻이지 값이 좋다는 뜻이 아니다
  "odom 정지 후 실제 이동"이 크면 odom이 먼저 정지를 보고한 것이다

정지 시간은 명령 종료부터 D-06 값 안으로 들어오기 시작한 시점까지다.
A4 판정에 필요한 0.5 s 유지 시간은 포함하지 않는다.
팔이 원점이 아닌 자세로 시작하면 팔 시나리오 동작량이 달라진다. 재시작 후 실행한다.
"""

import argparse
import json
import math
import queue
import subprocess
import sys
import threading
import time
from collections import deque

import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from gazebo_observer import GazeboObserver, fetch_robot_description, joint_speed_series

# D-06 실제 정지 판정 값
STOP_LINEAR_M_S = 0.01
STOP_ANGULAR_RAD_S = 0.02
STOP_ARM_RAD_S = 0.01
STOP_HOLD_SECONDS = 0.5

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

TRUTH_TOPIC = "/world/ax_test/dynamic_pose/info"
TRUTH_WINDOW_SECONDS = 0.1
SAMPLE_PERIOD_SECONDS = 0.02
# 명령 종료 뒤 기록 구간(시뮬레이션 시간 s). 정지(최대 약 1.7 s)와 D-06 유지 0.5 s를 담는다.
SETTLE_SIM_SECONDS = 3.0
IDLE_WALL_SECONDS = 3.0
# 시뮬레이션 시간 대기의 벽시계 상한(s). 실시간 비율이 낮아도 여기까지 기다린다.
SIM_WAIT_WALL_LIMIT_SECONDS = 120.0


# 각도 차이를 -pi~pi 범위로 정규화한다
def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


# JSON 쿼터니언에서 yaw를 구한다. gz는 값이 0인 필드를 생략하고 w의 기본값은 1이다
def yaw_from_quaternion(rotation):
    w = rotation.get("w", 1.0)
    x = rotation.get("x", 0.0)
    y = rotation.get("y", 0.0)
    z = rotation.get("z", 0.0)

    sine = 2.0 * (w * z + x * y)
    cosine = 1.0 - 2.0 * (y ** 2 + z ** 2)

    return math.atan2(sine, cosine)


# gz topic의 JSON 출력을 읽어 로봇 자세를 큐에 넣는다
def read_truth_stream(stream, robot_id, poses):
    for line in stream:
        try:
            message = json.loads(line)
        except json.JSONDecodeError:
            continue

        stamp = message.get("header", {}).get("stamp", {})
        seconds = float(stamp.get("sec", 0)) + float(stamp.get("nsec", 0)) / 1e9

        for pose in message.get("pose", []):
            if pose.get("name") != robot_id:
                continue

            position = pose.get("position", {})
            poses.put((
                seconds,
                position.get("x", 0.0),
                position.get("y", 0.0),
                yaw_from_quaternion(pose.get("orientation", {})),
            ))

            break


# 수신 주기와 최대 간격을 집계한다
class RateMeter:
    # 빈 집계로 시작한다
    def __init__(self):
        self.reset()

    # 측정 구간을 새로 시작한다
    def reset(self):
        self.count = 0
        self.last = None
        self.max_gap = 0.0
        self.first = None

    # 수신 한 건을 시뮬레이션 시각과 함께 기록한다
    def tick(self, stamp):
        self.count += 1

        if self.first is None:
            self.first = stamp

        if self.last is not None:
            self.max_gap = max(self.max_gap, stamp - self.last)

        self.last = stamp

    # 평균 주기(Hz)를 구한다
    def rate(self):
        if self.count < 2 or self.last <= self.first:
            return 0.0

        return (self.count - 1) / (self.last - self.first)


# 제어기 측정값과 Gazebo 실제 값을 같은 시간축으로 기록하는 노드
class StopFeedbackRecorder(Node):
    # 로봇별 토픽을 연결한다
    def __init__(self, robot_id, truth_poses):
        super().__init__(
            "check_stop_feedback",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )

        namespace = f"/ax/{robot_id}"
        self.truth_poses = truth_poses

        self.joint_meter = RateMeter()
        self.odom_meter = RateMeter()
        self.joint_nan_names = set()
        self.max_joint_age = 0.0

        self.wheel_speed = None
        self.arm_speed = None
        self.odom_linear = None
        self.odom_angular = None
        self.truth_history = deque()
        self.truth_linear = None
        self.truth_angular = None
        self.truth_xy = None
        # 팔 실제 속도는 main에서 연결하는 Gazebo 링크 관측기로 계산한다.
        self.observer = None
        self.truth_arm = None

        self.samples = []
        self.last_sample_time = None

        self.create_subscription(
            JointState,
            f"{namespace}/joint_states",
            self.on_joint_state,
            20,
        )

        self.create_subscription(
            Odometry,
            f"{namespace}/diff_drive_controller/odom",
            self.on_odom,
            20,
        )

        self.create_timer(0.01, self.drain_truth)

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

    # 현재 시뮬레이션 시각을 초로 돌려준다
    def now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    # 바퀴·팔 실측 속도와 수신 품질을 갱신한다
    def on_joint_state(self, message):
        now = self.now_seconds()
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9

        self.joint_meter.tick(now)
        self.max_joint_age = max(self.max_joint_age, now - stamp)

        names = list(message.name)

        for values in (message.position, message.velocity):
            for name, value in zip(names, values):
                if math.isnan(value):
                    self.joint_nan_names.add(name)

        self.wheel_speed = max(
            abs(message.velocity[names.index(joint)])
            for joint in WHEEL_JOINTS
        )
        self.arm_speed = max(
            abs(message.velocity[names.index(joint)])
            for joint in ARM_JOINTS
        )

    # odom 속도와 수신 품질을 갱신한다
    def on_odom(self, message):
        self.odom_meter.tick(self.now_seconds())

        self.odom_linear = abs(message.twist.twist.linear.x)
        self.odom_angular = abs(message.twist.twist.angular.z)

    # 큐에 쌓인 Gazebo 자세로 실제 속도를 계산하고 표본을 남긴다
    def drain_truth(self):
        while True:
            try:
                pose = self.truth_poses.get_nowait()
            except queue.Empty:
                return

            stamp, x, y, _ = pose

            self.truth_history.append(pose)
            self.truth_xy = (x, y)

            while stamp - self.truth_history[0][0] > TRUTH_WINDOW_SECONDS:
                self.truth_history.popleft()

            self.update_truth_speed()
            self.record_sample(stamp)

    # 시간 창의 양 끝 자세 차이로 실제 속도를 구한다
    def update_truth_speed(self):
        start = self.truth_history[0]
        end = self.truth_history[-1]
        elapsed = end[0] - start[0]

        if elapsed < TRUTH_WINDOW_SECONDS / 2.0:
            return

        distance = math.hypot(end[1] - start[1], end[2] - start[2])

        self.truth_linear = distance / elapsed
        self.truth_angular = abs(wrap_angle(end[3] - start[3])) / elapsed

    # Gazebo 링크 자세로 다시 계산한 팔 관절각에서 최근 창의 최대 관절 속도(rad/s)를 구한다
    def update_truth_arm(self, now):
        history = [
            (item["stamp"], item["joints"])
            for item in self.observer.since(now - 2.0 * TRUTH_WINDOW_SECONDS)
            if item["stamp"] <= now
        ]
        series = joint_speed_series(history, TRUTH_WINDOW_SECONDS)

        if series:
            self.truth_arm = series[-1][1]

    # 모든 값이 준비되면 일정 간격으로 표본을 저장한다
    def record_sample(self, now):
        if self.observer is not None:
            self.update_truth_arm(now)

        values = (
            self.odom_linear,
            self.odom_angular,
            self.truth_linear,
            self.truth_angular,
            self.arm_speed,
            self.wheel_speed,
            self.truth_arm,
        )

        if any(value is None for value in values):
            return

        if (
            self.last_sample_time is not None
            and now - self.last_sample_time < SAMPLE_PERIOD_SECONDS
        ):
            return

        self.last_sample_time = now
        self.samples.append((now, self.truth_xy) + values)

    # 벽시계 기준으로 지정 시간 동안 콜백을 처리한다
    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds

        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.005)

    # 시뮬레이션 시간이 지정 시각에 이를 때까지 콜백을 처리한다. 도달하면 True
    def spin_until_sim(self, target_seconds):
        wall_deadline = time.monotonic() + SIM_WAIT_WALL_LIMIT_SECONDS

        while self.now_seconds() < target_seconds:
            if time.monotonic() > wall_deadline:
                return False

            rclpy.spin_once(self, timeout_sec=0.005)

        return True

    # 명령 종료 시각부터 시뮬레이션 시간 기준 기록 구간을 채운다
    def settle_after(self, cut_time):
        if not self.spin_until_sim(cut_time + SETTLE_SIM_SECONDS):
            print("[WARN] 시뮬레이션 시간이 진행되지 않아 기록 구간을 다 채우지 못함", flush=True)

    # 속도 명령을 시뮬레이션 시간 기준으로 지정 시간 보낸 뒤 끊고 그 시각을 돌려준다
    def run_drive(self, linear, angular, seconds):
        end_time = self.now_seconds() + seconds
        wall_deadline = time.monotonic() + SIM_WAIT_WALL_LIMIT_SECONDS

        while self.now_seconds() < end_time and time.monotonic() < wall_deadline:
            message = TwistStamped()
            message.header.stamp = self.get_clock().now().to_msg()
            message.twist.linear.x = linear
            message.twist.angular.z = angular
            self.drive_publisher.publish(message)

            self.spin_for(0.05)

        return self.now_seconds()

    # 팔 궤적을 실행하고 Action 종료 시각을 돌려준다
    def run_arm(self, positions, seconds):
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(seconds=seconds).to_msg()

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS
        goal.trajectory.points = [point]

        accepted = self.arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, accepted, timeout_sec=10.0)

        handle = accepted.result()

        if handle is None or not handle.accepted:
            return None

        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result, timeout_sec=15.0)

        return self.now_seconds()


# 이후 구간이 끊기지 않고 D-06을 만족하기 시작한 시각을 찾는다
def first_stable_stop(samples, is_stopped):
    start = None

    for sample in samples:
        if not is_stopped(sample):
            start = None
        elif start is None:
            start = sample[0]

    if start is None or samples[-1][0] - start < STOP_HOLD_SECONDS:
        return None

    return start


# 명령 종료 뒤 표본에서 두 관측의 정지 시각과 차이를 계산한다
def evaluate(name, samples, cut_time):
    after = [sample for sample in samples if sample[0] >= cut_time]

    # 제어기 측정값 기준 D-06 판정
    def odom_stopped(sample):
        _, _, odom_v, odom_w, _, _, arm, _, _ = sample

        return (
            odom_v <= STOP_LINEAR_M_S
            and odom_w <= STOP_ANGULAR_RAD_S
            and arm <= STOP_ARM_RAD_S
        )

    # Gazebo 실제 값 기준 D-06 판정. 팔도 joint_states가 아닌 Gazebo 링크 자세 기준이다
    def truth_stopped(sample):
        _, _, _, _, truth_v, truth_w, _, _, truth_arm = sample

        return (
            truth_v <= STOP_LINEAR_M_S
            and truth_w <= STOP_ANGULAR_RAD_S
            and truth_arm <= STOP_ARM_RAD_S
        )

    odom_stop = first_stable_stop(after, odom_stopped)
    truth_stop = first_stable_stop(after, truth_stopped)

    if odom_stop is None or truth_stop is None:
        print(f"[FAIL] {name}: 정지 미확인 odom {odom_stop}, 실제 {truth_stop}", flush=True)

        return False

    moving = [sample for sample in samples if sample[0] < cut_time]
    linear_error = max(
        (abs(sample[4] - sample[2]) for sample in moving),
        default=0.0,
    )
    angular_error = max(
        (abs(sample[5] - sample[3]) for sample in moving),
        default=0.0,
    )

    odom_xy = next(sample[1] for sample in after if sample[0] >= odom_stop)
    final_xy = after[-1][1]
    slide = math.hypot(final_xy[0] - odom_xy[0], final_xy[1] - odom_xy[1])

    print(
        f"[PASS] {name}: 정지까지 odom {odom_stop - cut_time:.3f} s, "
        f"실제 {truth_stop - cut_time:.3f} s, "
        f"odom 정지 후 실제 이동 {slide * 1000:.1f} mm, "
        f"구동 중 최대 차이 선속도 {linear_error:.3f} m/s, "
        f"각속도 {angular_error:.3f} rad/s",
        flush=True,
    )

    return True


# 시나리오를 실행하고 실패 건수를 돌려준다
def run_checks(node):
    failures = 0

    deadline = time.monotonic() + 20.0

    while node.last_sample_time is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)

    if node.last_sample_time is None:
        print("[FAIL] 측정값 수신 없음(joint_states·odom·ground_truth)", flush=True)

        return 1

    node.joint_meter.reset()
    node.odom_meter.reset()
    node.max_joint_age = 0.0
    node.spin_for(IDLE_WALL_SECONDS)

    print(
        f"[INFO] joint_states {node.joint_meter.rate():.1f} Hz, "
        f"최대 간격 {node.joint_meter.max_gap * 1000:.1f} ms, "
        f"최대 지연 {node.max_joint_age * 1000:.1f} ms",
        flush=True,
    )
    print(
        f"[INFO] odom {node.odom_meter.rate():.1f} Hz, "
        f"최대 간격 {node.odom_meter.max_gap * 1000:.1f} ms",
        flush=True,
    )
    print(f"[INFO] 위치·속도 NaN 관절: {sorted(node.joint_nan_names)}", flush=True)

    scenarios = [
        ("직진 0.3 m/s", 0.3, 0.0, 3.0),
        ("직진 0.8 m/s", 0.8, 0.0, 3.0),
        ("제자리 회전 0.5 rad/s", 0.0, 0.5, 3.0),
        ("호 주행", 0.3, 0.3, 3.0),
    ]

    for name, linear, angular, seconds in scenarios:
        node.samples.clear()

        cut_time = node.run_drive(linear, angular, seconds)
        node.settle_after(cut_time)

        if not evaluate(name, node.samples, cut_time):
            failures += 1

    if not node.arm_client.wait_for_server(timeout_sec=10.0):
        print("[FAIL] 팔 Action 서버 없음", flush=True)

        return failures + 1

    arm_targets = [
        ("팔 이동", [0.6, 0.3, 0.3, 0.2, 0.3]),
        ("팔 복귀", [0.0, 0.0, 0.0, 0.0, 0.0]),
    ]

    for name, positions in arm_targets:
        node.samples.clear()

        cut_time = node.run_arm(positions, 2)

        if cut_time is None:
            print(f"[FAIL] {name}: 궤적 거절", flush=True)
            failures += 1

            continue

        node.settle_after(cut_time)

        if not evaluate(name, node.samples, cut_time):
            failures += 1

    return failures


# 시험 전용으로 Gazebo 자세 출력을 읽기 시작한다. launch에는 넣지 않는다
def start_truth_reader(robot_id, poses):
    process = subprocess.Popen(
        ["gz", "topic", "-e", "-t", TRUTH_TOPIC, "--json-output"],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
    )

    threading.Thread(
        target=read_truth_stream,
        args=(process.stdout, robot_id, poses),
        daemon=True,
    ).start()

    return process


# 명령행 인자를 읽어 측정을 실행한다
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-id", default="robot_1")
    arguments = parser.parse_args()

    truth_poses = queue.Queue()
    reader = start_truth_reader(arguments.robot_id, truth_poses)

    rclpy.init()
    node = StopFeedbackRecorder(arguments.robot_id, truth_poses)

    try:
        # 팔 실제 속도용 Gazebo 링크 관측기. 관절각 재계산에 실행 중인 모델의 URDF를 쓴다.
        urdf = fetch_robot_description(
            node,
            arguments.robot_id,
            spin=lambda: rclpy.spin_once(node, timeout_sec=0.05),
        )
        node.observer = GazeboObserver(arguments.robot_id, urdf, ARM_JOINTS, "gripper_base_link", None)
        node.observer.start()

        failures = run_checks(node)
    finally:
        # 종료 중 Gazebo 전송 스레드가 Python 콜백을 부르지 않도록 구독부터 해제한다.
        if node.observer is not None:
            node.observer.stop()

        node.destroy_node()
        rclpy.shutdown()
        reader.terminate()
        reader.wait(timeout=5)

    print(f"실패 {failures}건", flush=True)
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
