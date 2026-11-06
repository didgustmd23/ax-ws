#!/usr/bin/env python3
"""A2 개발용 정지 피드백 측정. 제어기 측정값과 Gazebo 실제 속도를 비교한다.

정지 기준은 INTERFACE_CONTRACT D-06이다. 판정 소유자는 A4이며 이 도구는 측정만 한다.
Gazebo 실제 값: 차체는 모델 자세, 팔은 링크 자세로 다시 계산한 관절각의 차분 속도다(gazebo_observer.py).
joint_states와 다른 경로이므로 두 정지 시각을 독립 관측끼리 비교할 수 있다.
Gazebo 실제 값은 시험 비교용이므로 운영 판정에 쓰지 않는다.
제어기를 A4 없이 직접 구동하므로 격리된 개발 시뮬레이션에서만 실행한다.

실행 순서
  터미널 1: ros2 launch ax_control simulation.launch.py robot_ids:=robot_1
  터미널 2: python3 src/ax_control/scripts/check_stop_feedback.py --robot-id robot_1 [--repeats 5]

--repeats N이면 시나리오마다 N회 측정하고 평균±표준편차를 출력한다. 주행 시나리오는 측정 뒤
반대 명령으로 되돌아가 출발점 근처를 유지한다. 결과는 기본으로 log/a2_stop_feedback/<KST 시각>/result.json이다.
물리 설정(질량·마찰·effort)이 잠정값이면 정지 시간도 잠정값이며 성능값으로 쓰지 않는다.

출력 읽는 법
  [INFO]  피드백 주기·최대 간격·지연과 NaN이 있는 관절, 시나리오별 반복 통계
  [PASS]  시나리오별 정지 시간. PASS는 정지가 확인됐다는 뜻이지 값이 좋다는 뜻이 아니다
  "odom 정지 후 실제 이동"이 크면 odom이 먼저 정지를 보고한 것이다

정지 시간은 명령 종료부터 D-06 값 안으로 들어오기 시작한 시점까지다.
A4 판정에 필요한 0.5 s 유지 시간은 포함하지 않는다.
팔이 원점이 아닌 자세로 시작하면 팔 시나리오 동작량이 달라진다. 재시작 후 실행한다.
"""

import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import queue
import subprocess
import sys
import threading
import time
import traceback
import yaml
from collections import deque
from zoneinfo import ZoneInfo

import rclpy
from rclpy.executors import ExternalShutdownException
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from gazebo_observer import (
    GazeboObserver,
    describe_environment,
    fetch_robot_description,
    joint_speed_series,
    read_urdf_joints,
    summarize,
)
from result_record import write_json_atomic

WORKSPACE = Path(__file__).resolve().parents[3]

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
# 명령 종료 뒤 최소 기록 구간(시뮬레이션 시간 s). 주행 시나리오는 감속 제한으로 계산한 값과 비교해 긴 쪽을 쓴다.
SETTLE_SIM_SECONDS = 3.0
# 감속 시간과 D-06 유지 시간 뒤에 더 기록하는 여유(s). Gazebo 속도 차분 창·표본 간격의 지연을 담는다.
SETTLE_MARGIN_SECONDS = 1.0
IDLE_WALL_SECONDS = 3.0
# 시뮬레이션 시간 대기의 벽시계 상한(s). 실시간 비율이 낮아도 여기까지 기다린다.
SIM_WAIT_WALL_LIMIT_SECONDS = 120.0


# 주행 명령 종료 뒤 기록 구간(s)을 계산한다. drive는 diff_drive_controller 파라미터 사전이다.
# 명령 속도에서 감속 제한으로 멈추는 시간 + D-06 유지 시간 + 여유를 쓰며 최소 SETTLE_SIM_SECONDS다.
# 가속 제한으로 명령 속도에 못 미친 채 끊겨도 감속 시간은 이 값보다 짧으므로 보수적인 상한이다.
def drive_settle_seconds(linear, angular, drive):
    stop_times = []

    for speed, prefix in ((linear, "linear.x"), (angular, "angular.z")):
        deceleration = abs(drive.get(f"{prefix}.max_deceleration") or 0.0)

        if speed and deceleration > 0.0:
            stop_times.append(abs(speed) / deceleration)

    window = max(stop_times, default=0.0) + STOP_HOLD_SECONDS + SETTLE_MARGIN_SECONDS

    return max(SETTLE_SIM_SECONDS, window)


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

        # 팔이 없는 로봇이면 속도를 갱신하지 않는다. 대상 오류는 main에서 URDF로 안내한다.
        if any(joint not in names for joint in WHEEL_JOINTS + ARM_JOINTS):
            return

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
    def settle_after(self, cut_time, seconds=SETTLE_SIM_SECONDS):
        if not self.spin_until_sim(cut_time + seconds):
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


# 명령 종료 뒤 표본에서 두 관측의 정지 시각과 차이를 계산한다. 정지를 확인하지 못하면 None
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

        return None

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

    return {
        "odom_stop_s": odom_stop - cut_time,
        "truth_stop_s": truth_stop - cut_time,
        "slide_after_odom_stop_m": slide,
        "max_linear_difference_m_s": linear_error,
        "max_angular_difference_rad_s": angular_error,
    }


# 시나리오별 반복 결과의 평균·표준편차·범위를 출력하고 사전으로 돌려준다
def summarize_results(results, report=True):
    summary = {}

    for name, runs in results.items():
        values = [run for run in runs if run is not None]
        summary[name] = {
            "runs": len(runs),
            "stopped": len(values),
            **{key: summarize([run[key] for run in values]) for key in (
                "odom_stop_s", "truth_stop_s", "slide_after_odom_stop_m",
            )},
        }

        odom, truth, slide = (summary[name][key] for key in ("odom_stop_s", "truth_stop_s", "slide_after_odom_stop_m"))

        if not report:
            continue

        if odom is None or truth is None:
            print(f"[INFO] {name}: 정지 확인 {len(values)}/{len(runs)}회", flush=True)
            continue

        print(
            f"[INFO] {name} ({len(values)}/{len(runs)}회): "
            f"odom {odom['mean']:.3f}±{odom['std']:.3f} s, 실제 {truth['mean']:.3f}±{truth['std']:.3f} s, "
            f"odom 정지 후 실제 이동 최대 {slide['max'] * 1000:.1f} mm",
            flush=True,
        )

    return summary


# 결과 기록의 뼈대를 만든다. status는 running → completed·failed·interrupted로 바뀐다.
def new_record(robot_id, repeats):
    return {
        "robot_id": robot_id,
        "repeats": repeats,
        "status": "running",
        "error": None,
        "last_scenario": None,
        "failures": 0,
        "criterion": {
            "linear_m_s": STOP_LINEAR_M_S,
            "angular_rad_s": STOP_ANGULAR_RAD_S,
            "arm_rad_s": STOP_ARM_RAD_S,
            "hold_s": STOP_HOLD_SECONDS,
        },
        "feedback": {},
        "runs": {},
        "summary": {},
        "note": "Provisional physics (mass, friction, effort). Measured values are not stop performance.",
    }


# 기록을 임시 파일에 쓴 뒤 바꿔 넣는다. 쓰는 도중 중단돼도 이전 result.json이 깨지지 않는다.
def write_record(path, record, report=False):
    record["summary"] = summarize_results(record["runs"], report)
    write_json_atomic(path, record)


# 시나리오 한 건의 결과를 기록에 넣고 바로 저장한다
def add_result(record, save, name, label, result):
    record["runs"].setdefault(name, []).append(result)
    record["last_scenario"] = label
    record["failures"] += 0 if result else 1
    save()


# 시나리오를 repeats회 실행하고 결과를 record에 쌓는다. 시나리오가 끝날 때마다 save()로 저장한다.
# 주행 시나리오마다 측정 뒤 반대 명령으로 되돌아가 출발점 근처를 유지한다(되돌아가는 구간은 측정하지 않음).
def run_checks(node, record, save, repeats=1):
    deadline = time.monotonic() + 20.0

    while node.last_sample_time is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)

    if node.last_sample_time is None:
        raise RuntimeError("측정값 수신 없음(joint_states·odom·ground_truth)")

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

    # 저속 비교 조건은 유지하고 최대 속도 시험은 현재 공통 상한을 사용한다.
    control = yaml.safe_load((Path(__file__).resolve().parents[1] /
        'config/common_controllers.yaml').read_text(encoding='utf-8'))
    drive = control['/**/diff_drive_controller']['ros__parameters']
    maximum_linear = drive['linear.x.max_velocity']
    maximum_angular = drive['angular.z.max_velocity']
    record["feedback"] = {
        "joint_states_hz": node.joint_meter.rate(),
        "joint_states_max_gap_s": node.joint_meter.max_gap,
        "joint_states_max_age_s": node.max_joint_age,
        "odom_hz": node.odom_meter.rate(),
        "odom_max_gap_s": node.odom_meter.max_gap,
        "nan_joints": sorted(node.joint_nan_names),
    }
    save()

    scenarios = [
        ("직진 0.3 m/s", 0.3, 0.0, 3.0),
        (f"직진 상한 {maximum_linear:g} m/s", maximum_linear, 0.0, 3.0),
        (f"제자리 회전 상한 {maximum_angular:g} rad/s", 0.0, maximum_angular, 3.0),
        ("호 주행", 0.3, 0.3, 3.0),
    ]

    for repeat in range(1, repeats + 1):
        for name, linear, angular, seconds in scenarios:
            node.samples.clear()

            # 감속 제한으로 정지가 길어져도 D-06 유지 구간까지 기록하도록 창을 계산한다.
            window = drive_settle_seconds(linear, angular, drive)

            cut_time = node.run_drive(linear, angular, seconds)
            node.settle_after(cut_time, window)

            result = evaluate(f"{name} #{repeat}", node.samples, cut_time)

            if result:
                result["record_window_s"] = window

            add_result(record, save, name, f"{name} #{repeat}", result)

            # 같은 경로를 거꾸로 되돌아간다. 측정하지 않는다.
            node.settle_after(node.run_drive(-linear, -angular, seconds), window)

    if not node.arm_client.wait_for_server(timeout_sec=10.0):
        raise RuntimeError("팔 Action 서버 없음\n" + describe_environment(node, node.robot_id))

    arm_targets = [
        ("팔 이동", [0.6, 0.3, 0.3, 0.2, 0.3]),
        ("팔 복귀", [0.0, 0.0, 0.0, 0.0, 0.0]),
    ]

    for repeat in range(1, repeats + 1):
        for name, positions in arm_targets:
            node.samples.clear()

            cut_time = node.run_arm(positions, 2)

            if cut_time is None:
                print(f"[FAIL] {name} #{repeat}: 궤적 거절", flush=True)
                add_result(record, save, name, f"{name} #{repeat}", None)

                continue

            node.settle_after(cut_time)

            result = evaluate(f"{name} #{repeat}", node.samples, cut_time)
            add_result(record, save, name, f"{name} #{repeat}", result)


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
    parser.add_argument("--repeats", type=int, default=1, help="시나리오별 반복 횟수")
    parser.add_argument("--output", type=Path, default=None, help="결과 폴더(기본 log/a2_stop_feedback/<KST 시각>)")
    arguments = parser.parse_args()

    if arguments.repeats < 1:
        parser.error("--repeats는 1 이상")

    stamp = datetime.now(ZoneInfo("Asia/Seoul")).strftime("%Y%m%d_%H%M%S")
    output = arguments.output or WORKSPACE / "log/a2_stop_feedback" / stamp

    if output.exists() and any(output.iterdir()):
        parser.error(f"{output}이 비어 있지 않음")

    output.mkdir(parents=True, exist_ok=True)
    path = output / "result.json"
    record = new_record(arguments.robot_id, arguments.repeats)

    # 중간 저장 함수. 시작 즉시 running 상태로 한 번 써 둔다.
    def save():
        write_record(path, record)

    save()

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

        # 팔 시나리오가 있으므로 팔 관절이 없는 로봇(robot_2)은 측정 대상이 아니다.
        missing = [joint for joint in ARM_JOINTS + WHEEL_JOINTS if joint not in read_urdf_joints(urdf)]

        if missing:
            raise RuntimeError(f"{arguments.robot_id}에 없는 관절: {', '.join(missing)}. 이 도구는 팔 로봇(robot_1 등) 전용")

        node.observer = GazeboObserver(arguments.robot_id, urdf, ARM_JOINTS, "gripper_base_link", None)

        try:
            node.observer.start()
        except RuntimeError as error:
            raise RuntimeError(f"{error}\n{describe_environment(node, arguments.robot_id)}") from None

        run_checks(node, record, save, arguments.repeats)
        record["status"] = "completed"

    # Ctrl+C는 rclpy 종료 처리에 따라 두 예외 중 하나로 올라온다.
    except (KeyboardInterrupt, ExternalShutdownException):
        record["status"] = "interrupted"
        record["error"] = "사용자 중단(Ctrl+C)"

    except Exception as error:
        traceback.print_exc()
        record["status"] = "failed"
        record["error"] = f"{type(error).__name__}: {error}"

    finally:
        # 종료 중 Gazebo 전송 스레드가 Python 콜백을 부르지 않도록 구독부터 해제한다.
        if node.observer is not None:
            node.observer.stop()

        node.destroy_node()
        rclpy.try_shutdown()
        reader.terminate()
        reader.wait(timeout=5)

        # 정리 뒤 최종 기록을 남긴다. 중간 저장본을 마지막 상태로 덮어쓴다.
        write_record(path, record, report=True)

    failures = record["failures"]
    status = record["status"]

    print(f"결과: {path} (status={status})", flush=True)

    if status != "completed":
        print(f"[FAIL] 측정 중단: {record['error']} (마지막 완료: {record['last_scenario'] or '없음'})", flush=True)
        sys.exit(130 if status == "interrupted" else 1)

    print(f"실패 {failures}건", flush=True)
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
