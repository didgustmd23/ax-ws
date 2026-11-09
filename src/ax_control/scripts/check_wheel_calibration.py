#!/usr/bin/env python3
"""A2 개발용 바퀴 보정 측정. odom이 계산한 회전·이동을 Gazebo 실제 값과 비교해 권장 보정값을 계산한다.

스키드 조향 로봇은 바퀴가 옆으로 미끄러지므로 diff_drive_controller가 쓰는 유효 윤거가 실제 윤거와 다르다.
diff_drive는 회전을 (오른쪽 이동 - 왼쪽 이동) / (wheel_separation × wheel_separation_multiplier)로 계산한다.
그래서 권장 multiplier = 현재 multiplier × (odom 회전 / 실제 회전)이다.
직진은 같은 방식으로 left/right_wheel_radius_multiplier 참고값을 계산한다(좌우 공통 값).

물리 설정(질량·마찰·차체 형상)이 잠정값이면 계산값도 잠정값이다. 이 도구는 설정 파일을 바꾸지 않는다.
결과를 `ax_control/config/ax*_controllers.yaml`에 반영할지는 물리 설정 확정 뒤 사람이 정한다.

제어기를 A4 없이 직접 구동하므로 격리된 개발 시뮬레이션에서만 실행한다.
로봇은 제자리에서 좌우로 돌고, 앞으로 갔다가 같은 거리만큼 되돌아온다. 주변 1.5 m를 비워 둔다.

실행 순서
  터미널 1: ros2 launch ax_control simulation.launch.py robot_ids:=robot_2 gui:=false
  터미널 2: python3 src/ax_control/scripts/check_wheel_calibration.py --robot-id robot_2

결과는 기본으로 log/a2_wheel_calibration/<KST 시각>/result.json에 남는다.

회차마다 차체 최대 기울기(Gazebo 모델 자세)와 회차 끝 정지(Gazebo 실제 속도가 D-06 값 안에 0.5 s)를 확인한다.
기울기가 --tilt-abort를 넘거나 정지가 확인되지 않으면 측정을 중단한다(이상 감지용, 성능 기준 아님).
회전 감속 제한(0.20 rad/s²)으로 0.5 rad/s에서 정지까지 약 2.5 s가 걸리므로 --settle은 3.0 s 이상을 권한다.
팔 로봇은 --arm-pose로 정지 상태에서 팔 자세를 먼저 바꾸고, joint_states가 --arm-tolerance 안에 정착한 뒤 주행한다.
주행 중 팔 자세 편차(joint_states 기준)를 회차마다 기록한다. 차체 충돌 센서가 없어 접촉은 기울기·궤적으로만 간접 확인한다.
"""

import argparse
from collections import deque
from datetime import datetime
import json
import math
from pathlib import Path
import sys
import threading
import time
from zoneinfo import ZoneInfo

import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from gazebo_observer import (
    ModelPoseTracker,
    describe_environment,
    quaternion_yaw,
    summarize,
    tilt_angle,
    unwrap_angles,
    yaw_of,
)

WORKSPACE = Path(__file__).resolve().parents[3]

# 실제로 돌았다고 볼 최소 회전(rad)과 이동(m). 이보다 작으면 비율이 잡음에 묻힌다.
MIN_ROTATION_RAD = 0.5
MIN_DISTANCE_M = 0.3

# 명령 발행 주기(벽시계 s). diff_drive cmd_vel_timeout(0.5 s)보다 충분히 짧게 둔다.
COMMAND_PERIOD_SECONDS = 0.05

# 읽어 올 diff_drive_controller 파라미터(Jazzy 이름). 없는 이름이 하나라도 있으면 응답 전체가 빈다.
PARAMETERS = [
    "wheel_separation",
    "wheel_radius",
    "wheel_separation_multiplier",
    "left_wheel_radius_multiplier",
    "right_wheel_radius_multiplier",
]

# D-06 실제 정지 판정 값(계약). 회차 끝 정지 확인에 쓴다. 판정 소유는 A4이며 여기서는 측정만 한다.
STOP_LINEAR_M_S = 0.01
STOP_ANGULAR_RAD_S = 0.02
STOP_HOLD_SECONDS = 0.5

# 팔 관절 순서(기둥 yaw·어깨·팔꿈치·손목·공구 회전). A1 travel_pose_rad와 같은 순서다.
ARM_JOINTS = ["column_yaw_joint", "shoulder_joint", "elbow_joint", "wrist_joint", "gripper_rotation_joint"]


# 유지 판정 속도 창(s). check_stop_feedback.py의 Gazebo 속도 창(TRUTH_WINDOW_SECONDS)과 같은 값이다.
SPEED_WINDOW_SECONDS = 0.1

# 유지 구간 안 Gazebo 표본 사이 최대 허용 간격(s). 자료 연속성 확인값이며 정지 임계값이 아니다.
# 속도 창보다 길면 창 하나가 관측 공백만으로 채워져 그 안의 움직임을 볼 수 없으므로 창 길이와 같게 둔다.
MAX_SAMPLE_GAP_SECONDS = SPEED_WINDOW_SECONDS


# 유지 판정 결과 사전을 만든다. problem이 있으면 속도를 None으로 두어 정지로 읽히지 않게 한다
def hold_result(problem, linear=None, angular=None, window=None, count=0, max_gap=None):
    return {
        "linear_m_s": linear,
        "angular_rad_s": angular,
        "window_s": window,
        "speed_window_s": SPEED_WINDOW_SECONDS,
        "samples": count,
        "max_gap_s": max_gap,
        "method": "max_short_window_path",
        "problem": problem,
    }


# 시각순 Gazebo 자세 [(시각, 4×4)]의 마지막 hold 초 유지 구간 속도.
# 구간 안 모든 SPEED_WINDOW_SECONDS 창에서 인접 표본 이동 길이 합(선: m, 회전: |Δyaw| 합 rad)을 창 시간으로 나눈
# 최대 선속도(m/s)·각속도(rad/s)를 돌려준다. 순변위가 아니라 경로 길이라 왕복 움직임이 상쇄되지 않는다.
# 표본 부족·간격 초과·시각 중복/역행·비유한 값이면 problem에 사유를 넣고 속도는 None이다.
def end_speed(samples, hold=STOP_HOLD_SECONDS):
    if len(samples) < 2:
        return hold_result("표본 부족", count=len(samples))

    # 1. 경계 표본: 유지 구간 시작(end - hold) 이전 또는 같은 시각의 마지막 표본부터 쓴다
    end_time = samples[-1][0]
    start_index = None

    for index in range(len(samples) - 1, -1, -1):
        if end_time - samples[index][0] >= hold:
            start_index = index
            break

    if start_index is None:
        return hold_result("유지 구간 길이 부족", count=len(samples))

    window = samples[start_index:]

    # 2. 시각·자세 품질: 유한값, 엄격히 증가하는 시각, 최대 간격
    times = [item[0] for item in window]

    if not all(math.isfinite(stamp) for stamp in times):
        return hold_result("비유한 시각", count=len(window))

    if not all(np.all(np.isfinite(item[1])) for item in window):
        return hold_result("비유한 자세", count=len(window))

    gaps = [later - earlier for earlier, later in zip(times, times[1:])]

    if min(gaps) <= 0.0:
        return hold_result("시각 중복·역행", count=len(window))

    max_gap = max(gaps)

    if max_gap > MAX_SAMPLE_GAP_SECONDS:
        return hold_result("표본 간격 초과", count=len(window), max_gap=max_gap)

    # 3. 인접 표본 사이 이동 길이(m)·회전 크기(rad)
    steps = []

    for (_, earlier), (_, later) in zip(window, window[1:]):
        distance = math.hypot(later[0, 3] - earlier[0, 3], later[1, 3] - earlier[1, 3])
        turn = yaw_of(later) - yaw_of(earlier)
        steps.append((distance, abs(math.atan2(math.sin(turn), math.cos(turn)))))

    # 4. 창 끝마다 SPEED_WINDOW_SECONDS 이상 거슬러 올라간 창의 경로 속도. 마지막 창은 끝 표본까지 포함한다
    linear_max = 0.0
    angular_max = 0.0
    first = 0

    for last in range(1, len(window)):
        while first + 1 < last and times[last] - times[first + 1] >= SPEED_WINDOW_SECONDS:
            first += 1

        elapsed = times[last] - times[first]

        if elapsed < SPEED_WINDOW_SECONDS:
            continue

        distance = sum(step[0] for step in steps[first:last])
        turn = sum(step[1] for step in steps[first:last])
        linear_max = max(linear_max, distance / elapsed)
        angular_max = max(angular_max, turn / elapsed)

    return hold_result(
        None,
        linear=linear_max,
        angular=angular_max,
        window=times[-1] - times[0],
        count=len(window),
        max_gap=max_gap,
    )


# 결과 파일에 남길 회차 끝 정지 유지 판정 방식. 이전 결과(순변위 기준)와 구분하는 데 쓴다
def stop_hold_conditions():
    return {
        "method": "max_short_window_path",
        "revision": "2026-11-09 짧은 창 경로 속도 최대값(이전: 유지 구간 양끝 순변위)",
        "linear_threshold_m_s": STOP_LINEAR_M_S,
        "angular_threshold_rad_s": STOP_ANGULAR_RAD_S,
        "hold_s": STOP_HOLD_SECONDS,
        "speed_window_s": SPEED_WINDOW_SECONDS,
        "max_sample_gap_s": MAX_SAMPLE_GAP_SECONDS,
        "source": "Gazebo dynamic_pose 모델 자세",
    }


# 유지 구간 속도가 D-06 값 안이면 True. 판정 불가(problem)는 정지로 보지 않는다
def hold_stopped(speed):
    return (
        speed["problem"] is None
        and speed["linear_m_s"] < STOP_LINEAR_M_S
        and speed["angular_rad_s"] < STOP_ANGULAR_RAD_S
    )


# 시각순 (시각, yaw) 표본에서 감김을 풀어 누적 회전(rad)을 구한다. 표본이 둘 미만이면 None
def accumulated_rotation(samples):
    if len(samples) < 2:
        return None

    unwrapped = unwrap_angles([yaw for _, yaw in samples])

    return unwrapped[-1] - unwrapped[0]


# 시작·끝 평면 위치의 직선거리(m)에 이동 방향 부호를 붙인다. heading은 시작 시 차체 yaw(rad)
def signed_distance(start, end, heading):
    dx, dy = end[0] - start[0], end[1] - start[1]
    forward = dx * math.cos(heading) + dy * math.sin(heading)

    return math.copysign(math.hypot(dx, dy), forward)


# 시작 자세 차체 좌표계로 본 이동량(전방, 좌측). start·end는 (x, y), heading은 시작 yaw다.
def body_displacement(start, end, heading):
    dx, dy = end[0] - start[0], end[1] - start[1]
    c, s = math.cos(heading), math.sin(heading)

    return (c * dx + s * dy, -s * dx + c * dy)


# 위치 궤적(x, y 목록)에 원을 최소제곱으로 맞춰 (중심 x, 중심 y, 반지름)을 돌려준다. 점이 3개 미만이면 None.
def fit_circle(points):
    import numpy as np

    points = np.asarray(points, float).reshape(-1, 2)

    if len(points) < 3:
        return None

    # 점이 거의 한 곳이면(차체 중심 회전) 그 점을 중심, 반지름 0으로 본다.
    mean = points.mean(axis=0)

    if np.abs(points - mean).max() < 1e-4:
        return float(mean[0]), float(mean[1]), 0.0

    x, y = points[:, 0], points[:, 1]
    a = np.c_[2.0 * x, 2.0 * y, np.ones_like(x)]
    cx, cy, c = np.linalg.lstsq(a, x * x + y * y, rcond=None)[0]

    return float(cx), float(cy), float(math.sqrt(max(c + cx * cx + cy * cy, 0.0)))


# 제자리 회전의 실제 차체 병진과 odom 병진 오차를 계산한다.
# truth·odom은 [(시각, x, y, yaw)] 목록이며 같은 시험 구간의 표본이다. 실제 회전이 min_turn 이상이면 회전 중심도 구한다.
def rotation_translation(truth, odom, min_turn=1.0):
    true_move = body_displacement(truth[0][1:3], truth[-1][1:3], truth[0][3])
    odom_move = body_displacement(odom[0][1:3], odom[-1][1:3], odom[0][3])
    result = {
        "truth_translation_m": math.hypot(*true_move),
        "odom_translation_m": math.hypot(*odom_move),
        "translation_error_m": math.hypot(true_move[0] - odom_move[0], true_move[1] - odom_move[1]),
        "truth_displacement_body_m": list(true_move),
        "odom_displacement_body_m": list(odom_move),
        "rotation_centre_body_m": None,
        "rotation_radius_m": None,
    }

    turn = accumulated_rotation([(item[0], item[3]) for item in truth])

    if turn is not None and abs(turn) >= min_turn:
        circle = fit_circle([item[1:3] for item in truth])

        if circle is not None:
            result["rotation_centre_body_m"] = list(body_displacement(truth[0][1:3], circle[:2], truth[0][3]))
            result["rotation_radius_m"] = circle[2]

    return result


# 현재 multiplier와 (odom 값 / 실제 값) 비율 평균으로 권장 multiplier를 계산한다
def recommended_separation_multiplier(current, ratios):
    stats = summarize(ratios)

    return None if stats is None else current * stats["mean"]


# odom·Gazebo를 기록하고 바퀴 명령을 내는 측정 노드
class WheelCalibrator(Node):
    # 로봇별 토픽을 연결한다
    def __init__(self, robot_id):
        super().__init__("check_wheel_calibration", parameter_overrides=[Parameter("use_sim_time", value=True)])

        self.robot_id = robot_id
        self.namespace = f"/ax/{robot_id}"
        self.lock = threading.Lock()
        self.odom = deque()

        self.drive = self.create_publisher(TwistStamped, f"{self.namespace}/diff_drive_controller/cmd_vel", 10)
        self.create_subscription(Odometry, f"{self.namespace}/diff_drive_controller/odom", self.on_odom, 20)

        # 팔 자세(팔 로봇만): joint_states 표본과 팔 궤적 Action
        self.arm = deque()
        self.arm_target = None
        self.create_subscription(JointState, f"{self.namespace}/joint_states", self.on_joint_state, 50)
        self.arm_client = ActionClient(self, FollowJointTrajectory, f"{self.namespace}/arm_controller/follow_joint_trajectory")

        self.parameters = AsyncParameterClient(self, f"{self.namespace}/diff_drive_controller")
        self.truth = ModelPoseTracker([robot_id])

    # odom 위치(m)·yaw(rad)를 측정 시각과 함께 저장한다
    def on_odom(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        pose = message.pose.pose
        q = pose.orientation

        with self.lock:
            if self.odom and stamp < self.odom[-1][0]:
                self.odom.clear()

            self.odom.append((stamp, pose.position.x, pose.position.y, quaternion_yaw(q.x, q.y, q.z, q.w)))

            while self.odom and stamp - self.odom[0][0] > 60.0:
                self.odom.popleft()

    # 팔 관절 위치를 시각과 함께 저장한다(팔 관절이 모두 있을 때만).
    def on_joint_state(self, message):
        names = list(message.name)

        if any(joint not in names for joint in ARM_JOINTS):
            return

        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9

        with self.lock:
            self.arm.append((stamp, [message.position[names.index(joint)] for joint in ARM_JOINTS]))

            while self.arm and stamp - self.arm[0][0] > 60.0:
                self.arm.popleft()

    # 최근 팔 관절 위치. 없으면 None
    def arm_now(self):
        with self.lock:
            return self.arm[-1][1] if self.arm else None

    # 정지 상태에서 팔을 target으로 옮기고 joint_states가 tolerance 안에 hold 초 머물 때까지 기다린다.
    # 정착 결과(최대 오차·걸린 시간)를 돌려준다. 정착하지 못하면 RuntimeError.
    def set_arm_pose(self, target, tolerance, seconds=5.0, hold=1.0, limit=30.0):
        if self.arm_now() is None:
            raise RuntimeError("팔 관절 joint_states 없음(팔 로봇이 아니거나 제어기 미동작)")

        if not self.arm_client.wait_for_server(timeout_sec=15.0):
            raise RuntimeError("arm_controller follow_joint_trajectory Action 없음")

        self.settle(1.0)
        point = JointTrajectoryPoint()
        point.positions = list(target)
        point.time_from_start = Duration(seconds=seconds).to_msg()
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS
        goal.trajectory.points = [point]
        start = self.sim_now()
        accepted = self.arm_client.send_goal_async(goal)
        deadline = time.monotonic() + 30.0

        while not accepted.done():
            if time.monotonic() > deadline:
                raise RuntimeError("팔 목표 수락 응답 없음")

            self.publish(0.0, 0.0)
            time.sleep(COMMAND_PERIOD_SECONDS)

        if not accepted.result().accepted:
            raise RuntimeError("팔 목표 거절")

        # 바퀴 정지 명령을 유지하면서 정착을 기다린다.
        settled_since = None
        end = self.sim_now() + seconds + limit
        wall = time.monotonic() + 60.0 + 10.0 * (seconds + limit)

        while self.sim_now() < end and time.monotonic() < wall:
            self.publish(0.0, 0.0)
            error = max(abs(a - b) for a, b in zip(self.arm_now(), target))
            now = self.sim_now()

            if error <= tolerance:
                settled_since = settled_since if settled_since is not None else now

                if now - settled_since >= hold and now - start >= seconds:
                    self.arm_target = list(target)
                    return {"target_rad": list(target), "max_error_rad": error, "settle_sim_s": now - start,
                            "tolerance_rad": tolerance, "source": "joint_states"}
            else:
                settled_since = None

            time.sleep(COMMAND_PERIOD_SECONDS)

        raise RuntimeError(f"팔 자세가 {tolerance} rad 안에 정착하지 않음(현재 {self.arm_now()})")

    # 현재 시뮬레이션 시각(s)
    def sim_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # 실행 중인 diff_drive_controller 파라미터를 읽는다
    def read_parameters(self):
        if not self.parameters.wait_for_services(timeout_sec=15.0):
            raise RuntimeError(
                "diff_drive_controller 파라미터 서비스 없음\n" + describe_environment(self, self.robot_id),
            )

        future = self.parameters.get_parameters(PARAMETERS)
        deadline = time.monotonic() + 10.0

        while not future.done():
            if time.monotonic() > deadline:
                raise RuntimeError("diff_drive_controller 파라미터 응답 없음")

            time.sleep(0.02)

        values = future.result().values

        if len(values) != len(PARAMETERS):
            raise RuntimeError("diff_drive_controller 파라미터 일부가 없음: " + ", ".join(PARAMETERS))

        return {name: value.double_value for name, value in zip(PARAMETERS, values)}

    # 속도 명령 하나를 보낸다
    def publish(self, linear, angular):
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.twist.linear.x = linear
        message.twist.angular.z = angular
        self.drive.publish(message)

    # 시뮬레이션 시간 seconds 동안 같은 명령을 유지한 뒤 0을 보낸다
    def command_for(self, linear, angular, seconds):
        end = self.sim_now() + seconds
        deadline = time.monotonic() + 60.0 + 10.0 * seconds

        while self.sim_now() < end:
            if time.monotonic() > deadline:
                raise RuntimeError("시뮬레이션 시간이 진행되지 않음")

            self.publish(linear, angular)
            time.sleep(COMMAND_PERIOD_SECONDS)

        self.publish(0.0, 0.0)

    # 시뮬레이션 시간이 seconds만큼 지나도록 기다린다. 정지 명령을 계속 보낸다
    def settle(self, seconds):
        self.command_for(0.0, 0.0, seconds)

    # [start, end] 구간의 odom 표본 목록
    def odom_between(self, start, end):
        with self.lock:
            return [item for item in self.odom if start <= item[0] <= end]

    # 두 관측이 stamp까지 도착할 때까지 기다린다
    def wait_records(self, stamp, wall_limit_seconds=30.0):
        if not self.truth.wait_until(self.robot_id, stamp, wall_limit_seconds):
            raise RuntimeError(
                "Gazebo 관측이 구간 끝까지 도착하지 않음\n" + describe_environment(self, self.robot_id),
            )

        deadline = time.monotonic() + wall_limit_seconds

        while time.monotonic() < deadline:
            with self.lock:
                if self.odom and self.odom[-1][0] >= stamp:
                    return

            time.sleep(0.02)

        raise RuntimeError("odom이 구간 끝까지 도착하지 않음")

    # 명령 한 번을 실행하고 같은 구간의 odom·Gazebo 회전(rad)과 이동(m)을 돌려준다.
    # 차체 기울기가 tilt_abort를 넘거나 회차 끝에 실제 정지(D-06)가 확인되지 않으면 RuntimeError로 중단한다.
    def trial(self, linear, angular, seconds, settle_seconds, tilt_abort=float("inf")):
        self.settle(settle_seconds)
        start = self.sim_now()

        self.command_for(linear, angular, seconds)
        self.settle(settle_seconds)

        end = self.sim_now()
        self.wait_records(end)

        odom = self.odom_between(start, end)
        truth = [item for item in self.truth.since(self.robot_id, start) if item[0] <= end]

        if len(odom) < 2 or len(truth) < 2:
            raise RuntimeError(f"표본 부족: odom {len(odom)}, Gazebo {len(truth)}")

        truth_heading = yaw_of(truth[0][1])
        odom_heading = odom[0][3]

        # 이상 감지: 차체 기울기와 회차 끝 정지
        max_tilt = max(tilt_angle(item[1]) for item in truth)
        speed = end_speed(truth)
        stopped = hold_stopped(speed)

        if max_tilt > tilt_abort:
            raise RuntimeError(f"차체 기울기 {max_tilt:.3f} rad > {tilt_abort} rad(예상 밖 접촉·전도 가능성), 측정 중단")

        if not stopped:
            raise RuntimeError(f"회차 끝 실제 정지 미확인(Gazebo 마지막 {STOP_HOLD_SECONDS} s 유지 구간 "
                               f"{SPEED_WINDOW_SECONDS} s 창 최대 속도 {speed}), 측정 중단. "
                               "--settle을 늘리거나 원인을 확인한다")

        # 팔 자세 유지(팔 로봇·자세 지정 시): 구간 안 joint_states의 목표 대비 최대 편차
        arm_deviation = None

        if self.arm_target is not None:
            with self.lock:
                arm = [item for item in self.arm if start <= item[0] <= end]

            if arm:
                arm_deviation = max(max(abs(a - b) for a, b in zip(item[1], self.arm_target)) for item in arm)

        # 모델 원점이 base_link와 같으므로(Gazebo dynamic_pose 확인) 모델 자세를 차체 실제 자세로 쓴다.
        truth_samples = [(item[0], item[1][0, 3], item[1][1, 3], yaw_of(item[1])) for item in truth]
        odom_samples = [(item[0], item[1], item[2], item[3]) for item in odom]

        odom_rotation = accumulated_rotation([(item[0], item[3]) for item in odom])
        truth_rotation = accumulated_rotation([(item[0], yaw_of(item[1])) for item in truth])
        odom_distance = signed_distance(odom[0][1:3], odom[-1][1:3], odom_heading)
        truth_distance = signed_distance(truth[0][1][:2, 3], truth[-1][1][:2, 3], truth_heading)

        return {
            "command": {"linear_m_s": linear, "angular_rad_s": angular, "seconds": seconds},
            "translation": rotation_translation(truth_samples, odom_samples) if linear == 0.0 else None,
            "odom_rotation_rad": odom_rotation,
            "truth_rotation_rad": truth_rotation,
            "rotation_error_rad": odom_rotation - truth_rotation,
            "odom_distance_m": odom_distance,
            "truth_distance_m": truth_distance,
            "distance_error_m": odom_distance - truth_distance,
            "max_tilt_rad": max_tilt,
            "end_speed": speed,
            "stop_confirmed": stopped,
            "arm_max_deviation_rad": arm_deviation,
            "samples": {"odom": len(odom), "gazebo": len(truth)},
        }


# 판정 한 줄을 출력하고 결과 목록에 남긴다
def check(checks, name, passed, detail):
    checks.append({"name": name, "passed": bool(passed), "detail": detail})
    print(f"[{'PASS' if passed else 'FAIL'}] {name}: {detail}", flush=True)


# 회전·직진 시험을 반복하고 결과를 정리한다
def run(node, arguments, checks):
    parameters = node.read_parameters()
    print("[INFO] 현재 파라미터: " + ", ".join(f"{key} {value}" for key, value in parameters.items()), flush=True)

    arm_pose = None

    if arguments.arm_pose is not None:
        arm_pose = node.set_arm_pose(arguments.arm_pose, arguments.arm_tolerance)
        check(checks, "팔 자세 정착", True, f"목표 {arm_pose['target_rad']} rad, 최대 오차 {arm_pose['max_error_rad']:.4f} rad, "
                                       f"{arm_pose['settle_sim_s']:.1f} s")

    turns = []

    # 제자리 회전: 왼쪽(+)·오른쪽(-)을 번갈아 같은 횟수 반복한다.
    for repeat in range(1, arguments.repeats + 1):
        for direction in (1.0, -1.0):
            label = f"회전 {'왼쪽' if direction > 0 else '오른쪽'} {repeat}"
            result = node.trial(0.0, direction * arguments.angular, arguments.turn_seconds, arguments.settle,
                                arguments.tilt_abort)
            result["label"] = label

            odom, truth = result["odom_rotation_rad"], result["truth_rotation_rad"]
            valid = abs(truth) >= MIN_ROTATION_RAD and odom * truth > 0
            result["ratio_odom_over_truth"] = odom / truth if valid else None

            check(checks, label, valid, f"odom {odom:.4f} rad, 실제 {truth:.4f} rad")
            turns.append(result)

    drives = []

    # 직진: 앞으로 갔다가 같은 명령으로 되돌아와 출발점 근처를 유지한다.
    for repeat in range(1, arguments.repeats + 1):
        for direction in (1.0, -1.0):
            label = f"직진 {'전진' if direction > 0 else '후진'} {repeat}"
            result = node.trial(direction * arguments.linear, 0.0, arguments.drive_seconds, arguments.settle,
                                arguments.tilt_abort)
            result["label"] = label

            odom, truth = result["odom_distance_m"], result["truth_distance_m"]
            valid = abs(truth) >= MIN_DISTANCE_M and odom * truth > 0
            result["ratio_odom_over_truth"] = odom / truth if valid else None

            check(checks, label, valid, f"odom {odom:.4f} m, 실제 {truth:.4f} m")
            drives.append(result)

    turn_ratios = [item["ratio_odom_over_truth"] for item in turns]

    # 제자리 회전 중 차체 병진: 판정 없이 기록만 한다. 원인 확인 전 보정 계수로 가리지 않는다.
    translation = {}

    for side, sign in (("left", 1.0), ("right", -1.0)):
        items = [item["translation"] for item in turns if item["command"]["angular_rad_s"] * sign > 0]
        centres = [item["rotation_centre_body_m"] for item in items if item["rotation_centre_body_m"] is not None]
        translation[side] = {
            "truth_translation_m": summarize([item["truth_translation_m"] for item in items]),
            "odom_translation_m": summarize([item["odom_translation_m"] for item in items]),
            "translation_error_m": summarize([item["translation_error_m"] for item in items]),
            "rotation_centre_forward_m": summarize([centre[0] for centre in centres]),
            "rotation_centre_left_m": summarize([centre[1] for centre in centres]),
        }
    left = summarize([item["ratio_odom_over_truth"] for item in turns if item["command"]["angular_rad_s"] > 0])
    right = summarize([item["ratio_odom_over_truth"] for item in turns if item["command"]["angular_rad_s"] < 0])
    drive_stats = summarize([item["ratio_odom_over_truth"] for item in drives])

    separation = recommended_separation_multiplier(parameters["wheel_separation_multiplier"], turn_ratios)
    # 직진은 좌우 바퀴를 함께 쓰므로 두 반지름 보정의 평균을 기준으로 참고값을 낸다.
    current_radius = (parameters["left_wheel_radius_multiplier"] + parameters["right_wheel_radius_multiplier"]) / 2.0
    radius = None if drive_stats is None else current_radius * (1.0 / drive_stats["mean"])

    trials = turns + drives
    arm_deviations = [item["arm_max_deviation_rad"] for item in trials if item["arm_max_deviation_rad"] is not None]

    return {
        "robot_id": node.robot_id,
        "parameters": parameters,
        "arm_pose": arm_pose,
        "arm_hold": None if arm_pose is None else {
            "max_deviation_rad": summarize(arm_deviations),
            "within_tolerance": bool(arm_deviations) and max(arm_deviations) <= arguments.arm_tolerance,
            "tolerance_rad": arguments.arm_tolerance, "source": "joint_states(제어기 피드백)",
        },
        "max_tilt_rad": summarize([item["max_tilt_rad"] for item in trials]),
        "rotation_error_rad": {
            "left": summarize([item["rotation_error_rad"] for item in turns if item["command"]["angular_rad_s"] > 0]),
            "right": summarize([item["rotation_error_rad"] for item in turns if item["command"]["angular_rad_s"] < 0]),
        },
        "distance_error_m": {
            "forward": summarize([item["distance_error_m"] for item in drives if item["command"]["linear_m_s"] > 0]),
            "reverse": summarize([item["distance_error_m"] for item in drives if item["command"]["linear_m_s"] < 0]),
        },
        "turns": turns,
        "drives": drives,
        "turn_ratio": {"all": summarize(turn_ratios), "left": left, "right": right},
        "turn_translation": translation,
        "drive_ratio": drive_stats,
        "recommended": {
            "wheel_separation_multiplier": separation,
            "wheel_radius_multiplier_reference_left_right": radius,
        },
        "stop_hold_check": stop_hold_conditions(),
        "note": "Provisional physics (mass, friction, body shape). Recommendation only; config files are not changed.",
    }


# 결과 요약을 출력한다
def print_summary(result):
    turn = result["turn_ratio"]["all"]
    recommended = result["recommended"]

    if turn is not None:
        print(
            f"[INFO] 회전 비율(odom/실제) 평균 {turn['mean']:.4f}, 표준편차 {turn['std']:.4f}, "
            f"범위 {turn['min']:.4f}~{turn['max']:.4f} ({turn['count']}회)",
            flush=True,
        )

    for side in ("left", "right"):
        stats = result["turn_ratio"][side]

        if stats is not None:
            print(f"[INFO] {('왼쪽' if side == 'left' else '오른쪽')} 회전 비율 평균 {stats['mean']:.4f}", flush=True)

    if recommended["wheel_separation_multiplier"] is not None:
        print(
            f"[INFO] wheel_separation_multiplier 현재 {result['parameters']['wheel_separation_multiplier']:.4f} "
            f"→ 권장 {recommended['wheel_separation_multiplier']:.4f} (잠정 물리 설정 기준, 설정 파일은 바꾸지 않음)",
            flush=True,
        )

    for side, stats in result.get("turn_translation", {}).items():
        error, centre = stats["translation_error_m"], stats["rotation_centre_forward_m"]

        if error is not None:
            centre_text = f", 회전 중심 차체 앞 {centre['mean']:.3f} m" if centre is not None else ""
            print(
                f"[INFO] {('왼쪽' if side == 'left' else '오른쪽')} 회전 중 실제 병진 평균 "
                f"{stats['truth_translation_m']['mean']:.3f} m, odom 병진 {stats['odom_translation_m']['mean']:.3f} m, "
                f"odom 병진 오차 {error['mean']:.3f} m{centre_text}",
                flush=True,
            )

    if result["drive_ratio"] is not None:
        print(
            f"[INFO] 직진 비율(odom/실제) 평균 {result['drive_ratio']['mean']:.4f} "
            f"→ left/right_wheel_radius_multiplier 참고값 {recommended['wheel_radius_multiplier_reference_left_right']:.4f}",
            flush=True,
        )


# 명령행 인자를 읽어 측정을 실행하고 결과 파일을 남긴다
def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--robot-id", default="robot_2")
    parser.add_argument("--repeats", type=int, default=3, help="방향별 반복 횟수")
    parser.add_argument("--angular", type=float, default=0.5, help="회전 속도(rad/s)")
    parser.add_argument("--turn-seconds", type=float, default=4.0, help="회전 명령 시간(시뮬레이션 s)")
    parser.add_argument("--linear", type=float, default=0.3, help="직진 속도(m/s)")
    parser.add_argument("--drive-seconds", type=float, default=3.0, help="직진 명령 시간(시뮬레이션 s)")
    parser.add_argument("--settle", type=float, default=2.0, help="명령 전후 정지 대기(시뮬레이션 s)")
    parser.add_argument("--output", type=Path, default=None, help="결과 폴더(기본 log/a2_wheel_calibration/<KST 시각>)")
    parser.add_argument("--tilt-abort", type=float, default=0.15,
                        help="이 기울기(rad)를 넘으면 중단(A1 mobility 시험과 같은 0.15 rad, 이상 감지용)")
    parser.add_argument("--arm-pose", type=lambda text: [float(v) for v in text.split(",")], default=None,
                        help="주행 전 팔 자세(rad, 기둥·어깨·팔꿈치·손목·공구 순). 예: -1,0,0,0,0")
    parser.add_argument("--arm-tolerance", type=float, default=0.02, help="팔 자세 정착·유지 허용 오차(rad, A1 joint_tolerance_rad)")
    arguments = parser.parse_args()

    if arguments.arm_pose is not None and len(arguments.arm_pose) != len(ARM_JOINTS):
        parser.error(f"--arm-pose는 {len(ARM_JOINTS)}개 값")

    if arguments.repeats < 1 or min(arguments.angular, arguments.turn_seconds, arguments.linear,
                                     arguments.drive_seconds, arguments.settle) <= 0:
        parser.error("반복 횟수·속도·시간은 양수")

    if arguments.angular * arguments.turn_seconds < MIN_ROTATION_RAD or arguments.linear * arguments.drive_seconds < MIN_DISTANCE_M:
        parser.error(f"회전은 {MIN_ROTATION_RAD} rad, 이동은 {MIN_DISTANCE_M} m 이상이 되도록 속도·시간을 정한다")

    stamp = datetime.now(ZoneInfo("Asia/Seoul")).strftime("%Y%m%d_%H%M%S")
    output = arguments.output or WORKSPACE / "log/a2_wheel_calibration" / stamp

    if output.exists() and any(output.iterdir()):
        parser.error(f"{output}이 비어 있지 않음")

    output.mkdir(parents=True, exist_ok=True)

    rclpy.init()

    node = WheelCalibrator(arguments.robot_id)

    # 콜백은 별도 스레드 하나에서 처리한다(/clock 1 kHz 부하로 다중 스레드 executor가 밀리는 것을 피함).
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    # Ctrl+C로 rclpy가 닫히면 executor 스레드는 조용히 끝낸다. 중단 기록은 main 스레드가 남긴다.
    def spin():
        try:
            executor.spin()
        except ExternalShutdownException:
            pass

    threading.Thread(target=spin, daemon=True).start()

    checks = []
    result = {}
    status = "running"

    try:
        node.truth.start()
        result = run(node, arguments, checks)
        print_summary(result)
        status = "completed"

    except RuntimeError as error:
        check(checks, "측정 진행", False, str(error))
        status = "failed"

    # Ctrl+C는 rclpy 종료 처리에 따라 두 예외 중 하나로 올라온다.
    except (KeyboardInterrupt, ExternalShutdownException):
        check(checks, "측정 진행", False, "사용자 중단(Ctrl+C)")
        status = "interrupted"

    finally:
        # 정지 명령을 남기고 Gazebo 구독을 먼저 해제한다.
        # Ctrl+C로 rclpy 컨텍스트가 이미 닫혔으면 발행이 실패하므로 제어기의 cmd_vel 시간 초과 정지에 맡긴다.
        try:
            node.publish(0.0, 0.0)
        except Exception as error:
            print(f"[INFO] 정지 명령 발행 생략(rclpy 종료): {error}", flush=True)

        node.truth.stop()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()

    result["status"] = status
    result["checks"] = checks
    (output / "result.json").write_text(json.dumps(result, indent=2, ensure_ascii=False, default=float) + "\n")

    failures = sum(1 for item in checks if not item["passed"])
    print(f"결과: {output / 'result.json'}", flush=True)
    print(f"구조 판정 실패 {failures}건 (status={status})", flush=True)

    if status == "interrupted":
        sys.exit(130)

    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
