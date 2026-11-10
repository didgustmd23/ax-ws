#!/usr/bin/env python3
"""A2 개발용 정지 피드백 측정. 제어기 측정값과 Gazebo 실제 속도를 비교한다.

정지 기준은 INTERFACE_CONTRACT D-06이다. 판정 소유자는 A4이며 이 도구는 측정만 한다.
Gazebo 실제 값: 차체는 모델 자세, 팔은 링크 자세로 다시 계산한 관절각의 차분 속도다(gazebo_observer.py).
joint_states와 다른 경로이므로 두 정지 시각을 독립 관측끼리 비교할 수 있다.
Gazebo 실제 값은 시험 비교용이므로 운영 판정에 쓰지 않는다.
제어기를 A4 없이 직접 구동하므로 격리된 개발 시뮬레이션에서만 실행한다.

대상 로봇
  실행 중 모델의 robot_description(URDF)으로 팔 유무를 정한다.
  - 공통 주행 시험(직진·상한 직진·제자리 회전·호 주행): 바퀴 joint_states·odom·Gazebo 차체 자세. 모든 로봇.
  - 팔 시나리오와 팔 정지 조건(joint_states 팔 관절·팔 Action·Gazebo 팔 링크): 팔 관절이 모두 있는 로봇만.
    팔이 없으면(robot_2) 결과에 not_applicable로 남기며 팔 정지 성공으로 기록하지 않는다.
  - 팔 관절이 일부만 있거나 바퀴 관절이 없으면 대상 오류로 실패한다.

실행 순서
  터미널 1: ros2 launch ax_control simulation.launch.py robot_ids:=robot_1   (또는 robot_2)
  터미널 2: python3 src/ax_control/scripts/check_stop_feedback.py --robot-id robot_1 [--repeats 5]
            python3 src/ax_control/scripts/check_stop_feedback.py --robot-id robot_2 [--payload-kg 0.5|1|2]

--repeats N이면 시나리오마다 N회 측정하고 평균±표준편차를 출력한다. 주행 시나리오는 측정 뒤
반대 명령으로 되돌아가 출발점 근처를 유지한다. 결과는 기본으로 log/a2_stop_feedback/<KST 시각>/result.json이다.
물리 설정(질량·마찰·effort)이 잠정값이면 정지 시간도 잠정값이며 성능값으로 쓰지 않는다.

적재 조건(--payload-kg, 팔 없는 적재 로봇 전용)
  A1 이동 시험(tools/verify_a1_envelope.py --mobility)과 같은 방법으로 20 cm 상자를 cargo_link 중심
  0.12 m 위에 만들어 적재판에 올린다. 고정 관절·마찰 변경 없이 기본 표면 접촉으로만 놓인다(passive).
  질량은 a1_validation.yaml mobility.robot_2.test_payloads_kg(0.5·1·2 kg)만 허용한다.
  적재 결과는 그 상자·질량 조건에서만 의미가 있고, 무부하 결과를 적재 정지 성능으로 일반화하지 않는다.

시험 중단 조건(시험 보호용이며 성능 기준이 아니다)
  차체 기울기 --tilt-abort(기본 0.15 rad) 초과, 차체 높이 변화 0.02 m 초과(예상 밖 접촉·올라탐),
  적재물 이탈(A1 이동 시험 값: cargo_link 중심 수평 0.02 m, 높이 0.01 m 초과)이면 즉시 중단하고 사유를 남긴다.

출력 읽는 법
  [INFO]  피드백 주기·최대 간격·지연과 NaN이 있는 관절, 시나리오별 반복 통계
  [PASS]  시나리오별 정지 시간. PASS는 정지가 확인됐다는 뜻이지 값이 좋다는 뜻이 아니다
  [FAIL]  정지 미확인과 사유(odom·실제 기준 미확인, 표본 부족, 입력 단절, 기록 구간 부족)
  "odom 정지 후 실제 이동"이 크면 odom이 먼저 정지를 보고한 것이다

주행 정지 방식(--stop-mode, 주행 시나리오만. 팔 Action 시나리오 기준은 바뀌지 않는다)
  command_loss(기본, 기존 방식): 주행 명령 발행을 끊는다. 측정 구간에 0 명령을 보내지 않으므로
    제어기는 cmd_vel_timeout(현재 0.5 s)이 지난 뒤 스스로 감속한다. 기준 시각은 발행 중단 시각이며
    결과에는 timeout 대기와 감속이 함께 들어간다.
  zero_command: 주행 직후 선속도·각속도 0의 TwistStamped를 바로 보내고, 기록 구간 끝까지 유효한 stamp로
    0을 반복 발행한다. 기준 시각은 최초 0 발행 시각(도구 시계)이다. 제어기 수신 시각은 관측하지 않는다.
  결과 파일마다 stop_mode가 하나이며, 다른 방식의 반복 통계를 섞지 않는다.

정지 시간(*_stop_s)은 기준 시각부터 D-06 값 안으로 들어오기 시작한 시점까지다.
A4 판정에 필요한 0.5 s 유지 시간은 포함하지 않는다. 유지까지 확인된 시각은 *_hold_confirmed_s로 따로 남긴다.
팔이 원점이 아닌 자세로 시작하면 팔 시나리오 동작량이 달라진다. 재시작 후 실행한다.
"""

import argparse
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import queue
import subprocess
import sys
import threading
import time
import traceback
import xml.etree.ElementTree as ET
import yaml
from collections import deque
from zoneinfo import ZoneInfo

import numpy as np

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
    pose_matrix,
    read_urdf_joints,
    summarize,
    tilt_angle,
)
from result_record import write_json_atomic

WORKSPACE = Path(__file__).resolve().parents[3]
CONTROL_CONFIG = Path(__file__).resolve().parents[1] / "config"
A1_VALIDATION = WORKSPACE / "src/ax_description/config/a1_validation.yaml"

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

# 팔 시나리오 목표(rad, ARM_JOINTS 순서). 이동 뒤 원점으로 복귀한다.
ARM_TARGETS = [
    ("팔 이동", [0.6, 0.3, 0.3, 0.2, 0.3]),
    ("팔 복귀", [0.0, 0.0, 0.0, 0.0, 0.0]),
]

TRUTH_WORLD = "ax_test"
TRUTH_TOPIC = f"/world/{TRUTH_WORLD}/dynamic_pose/info"
TRUTH_WINDOW_SECONDS = 0.1
SAMPLE_PERIOD_SECONDS = 0.02
# 명령 종료 뒤 최소 기록 구간(시뮬레이션 시간 s). 주행 시나리오는 감속 제한으로 계산한 값과 비교해 긴 쪽을 쓴다.
SETTLE_SIM_SECONDS = 3.0
# 감속 시간과 D-06 유지 시간 뒤에 더 기록하는 여유(s). Gazebo 속도 차분 창·표본 간격의 지연을 담는다.
SETTLE_MARGIN_SECONDS = 1.0
IDLE_WALL_SECONDS = 3.0
# 시뮬레이션 시간 대기의 벽시계 상한(s). 실시간 비율이 낮아도 여기까지 기다린다.
SIM_WAIT_WALL_LIMIT_SECONDS = 120.0

# 입력 품질 확인(시뮬레이션 s). 성능 기준이 아니라 측정 자료가 끊기지 않았는지 보는 값이다.
# 각 입력의 마지막 수신이 이보다 오래되면 표본을 만들지 않고, 명령 종료 뒤 표본 간격이 이보다 길면 입력 단절로 본다.
INPUT_STALE_SECONDS = 0.25
# 명령 종료 뒤 판정에 필요한 최소 표본 수. D-06 유지 0.5 s를 표본 간격 0.02 s로 담는 수다.
MIN_SAMPLES_AFTER_CUT = int(STOP_HOLD_SECONDS / SAMPLE_PERIOD_SECONDS)

# 시험 보호용 중단 조건(성능 기준 아님). 차체 높이 변화는 낮은 물체에 올라탐·예상 밖 접촉을 잡는다.
DEFAULT_TILT_ABORT_RAD = 0.15
BASE_HEIGHT_ABORT_M = 0.02
# 적재물 이탈: A1 이동 시험(tools/verify_a1_envelope.py run_mobility)의 상자 이동·높이 허용값을 그대로 쓴다.
CARGO_DRIFT_ABORT_M = 0.02
CARGO_HEIGHT_ABORT_M = 0.01
# 상자는 cargo_link 중심 위 이 높이(m)에 만든다(A1 이동 시험과 같음). 만든 뒤 이 시간(s) 안정시킨다.
CARGO_SPAWN_HEIGHT_M = 0.12
CARGO_SETTLE_SECONDS = 2.0
PAYLOAD_MODEL = "a2_stop_payload"

# 주행 정지 방식과 기준 시각 정의
STOP_MODES = ("command_loss", "zero_command")
STOP_MODE_REFERENCE = {
    "command_loss": "마지막 비영 속도 명령 발행 뒤 발행을 중단한 시각(도구의 시뮬레이션 시계). "
                    "측정 구간에 0 명령을 보내지 않아 제어기 cmd_vel_timeout 경과 뒤 감속한다",
    "zero_command": "주행 뒤 최초 0 속도 TwistStamped 발행 시각(도구의 시뮬레이션 시계). "
                    "기록 구간 끝까지 0을 반복 발행한다. 제어기 수신 시각은 관측하지 않는다",
}
# zero_command에서 0 명령을 반복 발행하는 간격(시뮬레이션 s). 주행 명령 발행 간격과 같다.
ZERO_COMMAND_PERIOD_SECONDS = 0.05
# 감속 시작 판단: 기준 시각 직전 값의 이 비율 아래로 처음 내려간 시점. 기록용이며 판정 기준이 아니다.
DECELERATION_ONSET_RATIO = 0.95
DECELERATION_MIN_SPEED = {"odom_v": 0.05, "truth_v": 0.05, "odom_w": 0.05, "truth_w": 0.05}
CARGO_FRAME = "cargo_link"


# 주행 명령 종료 뒤 기록 구간(s)을 계산한다. drive는 diff_drive_controller 파라미터 사전이다.
# 명령 속도에서 감속 제한으로 멈추는 시간 + D-06 유지 시간 + 여유를 쓰며 최소 SETTLE_SIM_SECONDS다.
# 가속 제한으로 명령 속도에 못 미친 채 끊겨도 감속 시간은 이 값보다 짧으므로 보수적인 상한이다.
# command_timeout(s)은 감속이 시작되기 전 기다리는 시간이다(command_loss의 cmd_vel_timeout).
def drive_settle_seconds(linear, angular, drive, command_timeout=0.0):
    stop_times = []

    for speed, prefix in ((linear, "linear.x"), (angular, "angular.z")):
        deceleration = abs(drive.get(f"{prefix}.max_deceleration") or 0.0)

        if speed and deceleration > 0.0:
            stop_times.append(abs(speed) / deceleration)

    window = max(stop_times, default=0.0) + STOP_HOLD_SECONDS + SETTLE_MARGIN_SECONDS

    return max(SETTLE_SIM_SECONDS, window + command_timeout)


# 정지 방식별 기록 구간(s). command_loss는 실행 중 cmd_vel_timeout만큼 감속이 늦어지므로 그만큼 늘린다
def mode_settle_seconds(stop_mode, linear, angular, drive, command_timeout):
    if stop_mode == "command_loss":
        return drive_settle_seconds(linear, angular, drive, command_timeout or 0.0)

    return drive_settle_seconds(linear, angular, drive)


# 공통 주행 시나리오 목록(이름, 선속도, 각속도, 명령 시간 s). 저속 비교 조건은 유지하고 상한은 현재 공통 설정을 쓴다.
def drive_scenarios(maximum_linear, maximum_angular):
    return [
        ("직진 0.3 m/s", 0.3, 0.0, 3.0),
        (f"직진 상한 {maximum_linear:g} m/s", maximum_linear, 0.0, 3.0),
        (f"제자리 회전 상한 {maximum_angular:g} rad/s", 0.0, maximum_angular, 3.0),
        ("호 주행", 0.3, 0.3, 3.0),
    ]


# URDF 관절 목록으로 팔 유무를 정한다. 팔 관절이 모두 있으면 True, 하나도 없으면 False.
# 바퀴 관절이 없거나 팔 관절이 일부만 있으면 이 도구의 대상이 아니므로 RuntimeError다.
def arm_presence(joint_names, robot_id):
    joint_names = set(joint_names)
    missing_wheels = [joint for joint in WHEEL_JOINTS if joint not in joint_names]

    if missing_wheels:
        raise RuntimeError(f"{robot_id}에 없는 바퀴 관절: {', '.join(missing_wheels)}. 주행 정지 시험 대상이 아님")

    present_arm = [joint for joint in ARM_JOINTS if joint in joint_names]

    if present_arm and len(present_arm) != len(ARM_JOINTS):
        missing_arm = [joint for joint in ARM_JOINTS if joint not in joint_names]
        raise RuntimeError(f"{robot_id}의 팔 관절이 일부만 있음(없음: {', '.join(missing_arm)}). 모델 확인 필요")

    return bool(present_arm)


# 표본을 만드는 데 필요한 입력 이름. 팔이 없으면 팔 관련 입력을 요구하지 않는다.
def required_inputs(has_arm):
    inputs = ["odom", "truth", "wheels"]

    if has_arm:
        inputs += ["arm", "truth_arm"]

    return inputs


# 각 입력의 최근 수신 시각(stamps)이 now 기준 INPUT_STALE_SECONDS 안이면 True
def inputs_fresh(stamps, now, has_arm):
    return all(
        stamps.get(name) is not None and now - stamps[name] <= INPUT_STALE_SECONDS
        for name in required_inputs(has_arm)
    )


# URDF에서 root 링크 기준 link의 4×4 변환을 고정·가동 관절 원점(관절각 0)으로 누적한다
def link_frame(urdf_joints, link, root="base_link"):
    parent_joint = {child: (parent, origin) for _, parent, child, origin, _, _ in urdf_joints.values()}
    frame = np.eye(4)
    current = link

    while current != root:
        if current not in parent_joint:
            raise RuntimeError(f"{link}에서 {root}까지 관절 경로가 없음")

        parent, origin = parent_joint[current]
        frame = origin @ frame
        current = parent

    return frame


# 적재물 자세를 cargo_link 좌표계로 옮겨 중심 수평 거리와 기대 높이와의 차이(m)를 구한다
def cargo_offset(robot_world, cargo_frame, box_world, box_height):
    local = np.linalg.inv(robot_world @ cargo_frame) @ box_world
    drift = float(math.hypot(local[0, 3], local[1, 3]))
    height_error = float(abs(local[2, 3] - box_height / 2.0))

    return drift, height_error


# 실제 상태가 시험 보호 조건을 넘었는지 확인한다. 넘으면 사유 문자열, 아니면 None
def guard_violation(tilt, height_change, cargo, tilt_abort):
    if tilt > tilt_abort:
        return f"차체 기울기 {tilt:.3f} rad > {tilt_abort} rad"

    if height_change > BASE_HEIGHT_ABORT_M:
        return f"차체 높이 변화 {height_change:.3f} m > {BASE_HEIGHT_ABORT_M} m(예상 밖 접촉·올라탐 가능)"

    if cargo is not None:
        drift, height_error = cargo

        if drift > CARGO_DRIFT_ABORT_M or height_error > CARGO_HEIGHT_ABORT_M:
            return f"적재물 이탈: 중심 수평 {drift:.3f} m, 높이 차 {height_error:.3f} m"

    return None


# URDF gazebo 태그에서 바퀴 접촉 마찰(mu1·mu2·slip1·slip2)을 읽어 시험 조건으로 남긴다
def wheel_contacts(urdf_xml):
    root = ET.fromstring(urdf_xml)
    contacts = {}

    for gazebo in root.iter("gazebo"):
        if gazebo.find("mu1") is None and gazebo.find("mu2") is None:
            continue

        values = {}

        for key, path in (("mu1", "mu1"), ("mu2", "mu2"),
                          ("slip1", "collision/surface/friction/ode/slip1"),
                          ("slip2", "collision/surface/friction/ode/slip2")):
            text = (gazebo.findtext(path) or "").strip()
            values[key] = float(text) if text else None

        contacts[gazebo.get("reference")] = values

    return contacts


# 파일 SHA-256. 없으면 None
def file_sha256(path):
    path = Path(path)

    return hashlib.sha256(path.read_bytes()).hexdigest() if path.exists() else None


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


# JSON 자세 한 건을 4×4 변환으로 바꾼다. 생략된 필드는 기본값(위치 0, w 1)이다
def json_pose_matrix(pose):
    position = pose.get("position", {})
    rotation = pose.get("orientation", {})

    return pose_matrix(
        (position.get("x", 0.0), position.get("y", 0.0), position.get("z", 0.0)),
        (rotation.get("x", 0.0), rotation.get("y", 0.0), rotation.get("z", 0.0), rotation.get("w", 1.0)),
    )


# gz topic의 JSON 출력을 읽어 로봇 자세(와 있으면 적재물 자세)를 큐에 넣는다.
# 큐 항목: (시각, x, y, yaw, 로봇 4×4, 적재물 4×4 또는 None)
def read_truth_stream(stream, robot_id, poses, payload_name=PAYLOAD_MODEL):
    for line in stream:
        try:
            message = json.loads(line)
        except json.JSONDecodeError:
            continue

        stamp = message.get("header", {}).get("stamp", {})
        seconds = float(stamp.get("sec", 0)) + float(stamp.get("nsec", 0)) / 1e9
        robot = None
        payload = None

        for pose in message.get("pose", []):
            if pose.get("name") == robot_id:
                robot = pose
            elif pose.get("name") == payload_name:
                payload = pose

        if robot is None:
            continue

        position = robot.get("position", {})
        poses.put((
            seconds,
            position.get("x", 0.0),
            position.get("y", 0.0),
            yaw_from_quaternion(robot.get("orientation", {})),
            json_pose_matrix(robot),
            json_pose_matrix(payload) if payload is not None else None,
        ))


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
    # 로봇별 토픽을 연결한다. 팔 Action 클라이언트는 팔이 있을 때 enable_arm()으로 만든다
    def __init__(self, robot_id, truth_poses, tilt_abort=DEFAULT_TILT_ABORT_RAD):
        super().__init__(
            "check_stop_feedback",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )

        self.robot_id = robot_id
        namespace = f"/ax/{robot_id}"
        self.namespace = namespace
        self.truth_poses = truth_poses
        self.has_arm = False
        self.tilt_abort = tilt_abort

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
        # 입력별 최근 수신 시각(시뮬레이션 s). 표본은 필요한 입력이 모두 최근일 때만 만든다.
        self.input_stamps = {}

        # 시험 보호 상태. 적재 시험이면 cargo_frame·payload_height를 main에서 정한다.
        self.robot_matrix = None
        self.base_height = None
        self.cargo_frame = None
        self.payload_height = None
        self.payload_matrix = None
        self.guard_error = None
        self.last_guard = {"tilt": 0.0, "height_change": 0.0, "cargo": None}

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

        self.arm_client = None
        # zero_command 0 유지 상태. None이면 0을 반복 발행하지 않는다.
        self.zero_hold = None

    # 팔이 있는 로봇이면 팔 관측을 요구하고 팔 Action 클라이언트를 만든다
    def enable_arm(self):
        self.has_arm = True
        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            f"{self.namespace}/arm_controller/follow_joint_trajectory",
        )

    # 현재 시뮬레이션 시각을 초로 돌려준다
    def now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    # 바퀴·팔 실측 속도와 수신 품질을 갱신한다. 필요한 관절이 빠진 메시지는 속도를 갱신하지 않는다
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

        if len(message.velocity) != len(names):
            return

        if all(joint in names for joint in WHEEL_JOINTS):
            self.wheel_speed = max(
                abs(message.velocity[names.index(joint)])
                for joint in WHEEL_JOINTS
            )
            self.input_stamps["wheels"] = now

        if self.has_arm and all(joint in names for joint in ARM_JOINTS):
            self.arm_speed = max(
                abs(message.velocity[names.index(joint)])
                for joint in ARM_JOINTS
            )
            self.input_stamps["arm"] = now

    # odom 속도와 수신 품질을 갱신한다
    def on_odom(self, message):
        now = self.now_seconds()
        self.odom_meter.tick(now)

        self.odom_linear = abs(message.twist.twist.linear.x)
        self.odom_angular = abs(message.twist.twist.angular.z)
        self.input_stamps["odom"] = now

    # 큐에 쌓인 Gazebo 자세로 실제 속도를 계산하고 보호 조건을 확인한 뒤 표본을 남긴다
    def drain_truth(self):
        while True:
            try:
                pose = self.truth_poses.get_nowait()
            except queue.Empty:
                return

            stamp, x, y, yaw, robot_matrix, payload_matrix = pose

            self.truth_history.append((stamp, x, y, yaw))
            self.truth_xy = (x, y)
            self.robot_matrix = robot_matrix

            if payload_matrix is not None:
                self.payload_matrix = payload_matrix

            while stamp - self.truth_history[0][0] > TRUTH_WINDOW_SECONDS:
                self.truth_history.popleft()

            self.update_truth_speed(stamp)
            self.update_guard(robot_matrix, payload_matrix)
            self.record_sample(stamp)

    # 시간 창의 양 끝 자세 차이로 실제 속도를 구한다
    def update_truth_speed(self, stamp):
        start = self.truth_history[0]
        end = self.truth_history[-1]
        elapsed = end[0] - start[0]

        if elapsed < TRUTH_WINDOW_SECONDS / 2.0:
            return

        distance = math.hypot(end[1] - start[1], end[2] - start[2])

        self.truth_linear = distance / elapsed
        self.truth_angular = abs(wrap_angle(end[3] - start[3])) / elapsed
        self.input_stamps["truth"] = stamp

    # 차체 기울기·높이 변화·적재물 위치를 계산하고 보호 조건을 넘으면 첫 사유를 남긴다
    def update_guard(self, robot_matrix, payload_matrix):
        if self.base_height is None:
            self.base_height = float(robot_matrix[2, 3])

        cargo = None

        if self.payload_height is not None and payload_matrix is not None:
            cargo = cargo_offset(robot_matrix, self.cargo_frame, payload_matrix, self.payload_height)

        self.last_guard = {
            "tilt": tilt_angle(robot_matrix),
            "height_change": abs(float(robot_matrix[2, 3]) - self.base_height),
            "cargo": cargo,
        }

        violation = guard_violation(
            self.last_guard["tilt"], self.last_guard["height_change"], cargo, self.tilt_abort,
        )

        if violation and self.guard_error is None:
            self.guard_error = violation

    # 보호 조건 위반이 있으면 정지 명령을 보내고 RuntimeError로 시험을 멈춘다
    def check_guard(self):
        if self.guard_error is not None:
            self.stop_drive()
            raise RuntimeError(f"시험 중단: {self.guard_error}")

    # 정지(0) 속도 명령을 몇 번 보낸다. 중단·시험 보호·종료 때 쓰는 기존 정지 경로다
    def stop_drive(self):
        for _ in range(3):
            self.publish_drive(0.0, 0.0)

    # 현재 시각 stamp를 가진 속도 명령을 만든다
    def make_twist(self, linear, angular):
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.twist.linear.x = linear
        message.twist.angular.z = angular

        return message

    # 메시지 stamp를 초로 바꾼다
    def stamp_seconds(self, message):
        return message.header.stamp.sec + message.header.stamp.nanosec / 1e9

    # 속도 명령 하나를 발행하고 그 stamp(s)를 돌려준다
    def publish_drive(self, linear, angular):
        message = self.make_twist(linear, angular)
        self.drive_publisher.publish(message)

        return self.stamp_seconds(message)

    # zero_command 0 유지를 시작한다. 최초 호출 때만 0을 바로 발행하고 기준 시각을 정한다.
    # 이미 유지 중이면 기준 시각을 바꾸지 않고 그대로 돌려준다
    def begin_zero_hold(self):
        if self.zero_hold is None:
            published = self.now_seconds()
            stamp = self.publish_drive(0.0, 0.0)
            self.zero_hold = {"first": published, "first_stamp": stamp, "last": published, "count": 1, "max_gap": 0.0}

        return self.zero_hold["first"]

    # 유지 중이면 주기마다 0을 다시 발행한다
    def maintain_zero(self):
        hold = self.zero_hold

        if hold is None:
            return

        now = self.now_seconds()

        if now - hold["last"] < ZERO_COMMAND_PERIOD_SECONDS:
            return

        self.publish_drive(0.0, 0.0)
        hold["max_gap"] = max(hold["max_gap"], now - hold["last"])
        hold["last"] = now
        hold["count"] += 1

    # 0 유지를 끝내고 발행 기록을 돌려준다. 유지 중이 아니었으면 빈 기록이다
    def end_zero_hold(self):
        hold, self.zero_hold = self.zero_hold, None

        if hold is None:
            return {"zero_commands_published": 0, "zero_publish_max_gap_s": None, "last_zero_published_sim_s": None}

        return {
            "zero_commands_published": hold["count"],
            "zero_publish_max_gap_s": hold["max_gap"],
            "last_zero_published_sim_s": hold["last"],
        }

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
            self.input_stamps["truth_arm"] = now

    # 필요한 입력이 모두 최근이면 일정 간격으로 표본을 저장한다. 팔이 없으면 팔 값은 None이다
    def record_sample(self, now):
        if self.has_arm and self.observer is not None:
            self.update_truth_arm(now)

        if not inputs_fresh(self.input_stamps, now, self.has_arm):
            return

        if (
            self.last_sample_time is not None
            and now - self.last_sample_time < SAMPLE_PERIOD_SECONDS
        ):
            return

        self.last_sample_time = now
        self.samples.append({
            "t": now,
            "xy": self.truth_xy,
            "odom_v": self.odom_linear,
            "odom_w": self.odom_angular,
            "truth_v": self.truth_linear,
            "truth_w": self.truth_angular,
            "arm": self.arm_speed if self.has_arm else None,
            "wheel": self.wheel_speed,
            "truth_arm": self.truth_arm if self.has_arm else None,
            "tilt": self.last_guard["tilt"],
            "height_change": self.last_guard["height_change"],
            "cargo": self.last_guard["cargo"],
        })

    # 벽시계 기준으로 지정 시간 동안 콜백을 처리한다
    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds

        while time.monotonic() < deadline:
            self.spin_step()
            self.maintain_zero()
            self.check_guard()

    # 콜백을 한 번 처리한다
    def spin_step(self):
        rclpy.spin_once(self, timeout_sec=0.005)

    # 시뮬레이션 시간이 지정 시각에 이를 때까지 콜백을 처리한다. 도달하면 True
    def spin_until_sim(self, target_seconds):
        wall_deadline = time.monotonic() + SIM_WAIT_WALL_LIMIT_SECONDS

        while self.now_seconds() < target_seconds:
            if time.monotonic() > wall_deadline:
                return False

            self.spin_step()
            self.maintain_zero()
            self.check_guard()

        return True

    # 명령 종료 시각부터 시뮬레이션 시간 기준 기록 구간을 채운다
    def settle_after(self, cut_time, seconds=SETTLE_SIM_SECONDS):
        if not self.spin_until_sim(cut_time + seconds):
            print("[WARN] 시뮬레이션 시간이 진행되지 않아 기록 구간을 다 채우지 못함", flush=True)

    # 속도 명령을 시뮬레이션 시간 기준으로 지정 시간 반복 발행한 뒤 발행을 멈춘다(0을 보내지 않음).
    # 발행 기록(마지막 비영 명령 stamp·발행 시각, 발행 중단 시각, 명령 구간 완료 여부)을 돌려준다
    def run_drive(self, linear, angular, seconds):
        start = self.now_seconds()
        end_time = start + seconds
        wall_deadline = time.monotonic() + SIM_WAIT_WALL_LIMIT_SECONDS
        last_stamp = None
        last_published = None
        count = 0

        while self.now_seconds() < end_time and time.monotonic() < wall_deadline:
            last_published = self.now_seconds()
            last_stamp = self.publish_drive(linear, angular)
            count += 1

            self.spin_for(0.05)

        stopped = self.now_seconds()

        return {
            "command_started_sim_s": start,
            "command_seconds": seconds,
            "commands_published": count,
            "last_nonzero_stamp_s": last_stamp,
            "last_nonzero_published_sim_s": last_published,
            "command_stopped_sim_s": stopped,
            "completed": stopped >= end_time,
        }

    # 주행 명령을 보내고 정지 방식대로 멈춘 뒤 (기준 시각, 명령 기록)을 돌려준다.
    # 명령 구간을 다 채우지 못하면(벽시계 상한·시뮬레이션 정지) RuntimeError로 시험을 멈춘다
    def drive_and_stop(self, linear, angular, seconds, stop_mode):
        command = self.run_drive(linear, angular, seconds)

        if not command["completed"]:
            self.stop_drive()
            raise RuntimeError(
                f"주행 명령 구간 미완료: {command['command_stopped_sim_s'] - command['command_started_sim_s']:.3f} s"
                f" < {seconds} s(시뮬레이션 시간 정지 또는 벽시계 상한)"
            )

        command["stop_mode"] = stop_mode
        command["first_zero_published_sim_s"] = None
        command["first_zero_stamp_s"] = None

        if stop_mode == "zero_command":
            reference = self.begin_zero_hold()
            command["first_zero_published_sim_s"] = reference
            command["first_zero_stamp_s"] = self.zero_hold["first_stamp"]
        else:
            reference = command["command_stopped_sim_s"]

        command["reference_sim_s"] = reference

        return reference, command

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
            start = sample["t"]

    if start is None or samples[-1]["t"] - start < STOP_HOLD_SECONDS:
        return None

    return start


# 제어기 측정값 기준 D-06 판정. 팔 값이 None(팔 없음)이면 팔 조건을 보지 않는다
def odom_stopped(sample):
    arm = sample["arm"]

    return (
        sample["odom_v"] <= STOP_LINEAR_M_S
        and sample["odom_w"] <= STOP_ANGULAR_RAD_S
        and (arm is None or arm <= STOP_ARM_RAD_S)
    )


# Gazebo 실제 값 기준 D-06 판정. 팔도 joint_states가 아닌 Gazebo 링크 자세 기준이다
def truth_stopped(sample):
    arm = sample["truth_arm"]

    return (
        sample["truth_v"] <= STOP_LINEAR_M_S
        and sample["truth_w"] <= STOP_ANGULAR_RAD_S
        and (arm is None or arm <= STOP_ARM_RAD_S)
    )


# 기준 시각 직전 값에서 DECELERATION_ONSET_RATIO 아래로 처음 내려간 시점(기준 시각부터 s).
# 기준 직전 속도가 작거나 내려가지 않으면 None. 명시적 0 발행과 실제 감속 시작을 대조하는 기록용이다
def deceleration_onset(samples, cut_time, key):
    before = [sample for sample in samples if sample["t"] < cut_time]

    if not before or before[-1][key] is None or before[-1][key] < DECELERATION_MIN_SPEED[key]:
        return None

    limit = before[-1][key] * DECELERATION_ONSET_RATIO

    for sample in samples:
        if sample["t"] >= cut_time and sample[key] <= limit:
            return sample["t"] - cut_time

    return None


# 명령 종료 뒤 표본의 양과 연속성을 확인한다. 문제가 있으면 사유, 없으면 None
def coverage_problem(after, cut_time, window, moving_count=None):
    if len(after) < MIN_SAMPLES_AFTER_CUT:
        return f"표본 부족({len(after)}개 < {MIN_SAMPLES_AFTER_CUT}개)"

    if moving_count == 0:
        return "명령 구간 관측 없음(기준 시각 전 표본 0개)"

    times = [cut_time] + [sample["t"] for sample in after]
    gap = max(later - earlier for earlier, later in zip(times, times[1:]))

    if gap > INPUT_STALE_SECONDS:
        return f"입력 단절(표본 최대 간격 {gap:.3f} s > {INPUT_STALE_SECONDS} s)"

    if window is not None and after[-1]["t"] < cut_time + window - INPUT_STALE_SECONDS:
        return f"기록 구간 부족({after[-1]['t'] - cut_time:.3f} s < {window:.3f} s)"

    return None


# 명령 종료 뒤 표본으로 정지 시각과 차이를 계산한다. 정지·자료를 확인하지 못하면 stopped False와 사유를 돌려준다.
# 정지 시간(*_stop_s)은 D-06 안으로 들어오기 시작한 시점, 유지 확인(*_hold_confirmed_s)은 그 뒤 유지 시간까지다.
def assess_stop(samples, cut_time, window=None):
    after = [sample for sample in samples if sample["t"] >= cut_time]
    moving_count = sum(1 for sample in samples if sample["t"] < cut_time)
    problem = coverage_problem(after, cut_time, window, moving_count)

    if problem:
        return {"stopped": False, "reason": problem, "samples_after_cut": len(after)}

    odom_stop = first_stable_stop(after, odom_stopped)
    truth_stop = first_stable_stop(after, truth_stopped)

    if odom_stop is None or truth_stop is None:
        missing = [label for label, value in (("odom", odom_stop), ("실제", truth_stop)) if value is None]
        last = after[-1]

        return {
            "stopped": False,
            "reason": f"{'·'.join(missing)} 기준 정지 미확인(기록 끝 odom {last['odom_v']:.4f} m/s·"
                      f"{last['odom_w']:.4f} rad/s, 실제 {last['truth_v']:.4f} m/s·{last['truth_w']:.4f} rad/s)",
            "odom_stop_s": None if odom_stop is None else odom_stop - cut_time,
            "truth_stop_s": None if truth_stop is None else truth_stop - cut_time,
            "samples_after_cut": len(after),
        }

    moving = [sample for sample in samples if sample["t"] < cut_time]
    linear_error = max(
        (abs(sample["truth_v"] - sample["odom_v"]) for sample in moving),
        default=0.0,
    )
    angular_error = max(
        (abs(sample["truth_w"] - sample["odom_w"]) for sample in moving),
        default=0.0,
    )

    odom_xy = next(sample["xy"] for sample in after if sample["t"] >= odom_stop)
    final_xy = after[-1]["xy"]
    slide = math.hypot(final_xy[0] - odom_xy[0], final_xy[1] - odom_xy[1])
    times = [cut_time] + [sample["t"] for sample in after]
    cargo = [sample["cargo"] for sample in samples if sample["cargo"] is not None]

    return {
        "stopped": True,
        "odom_stop_s": odom_stop - cut_time,
        "truth_stop_s": truth_stop - cut_time,
        "stop_time_difference_s": truth_stop - odom_stop,
        "odom_hold_confirmed_s": odom_stop - cut_time + STOP_HOLD_SECONDS,
        "truth_hold_confirmed_s": truth_stop - cut_time + STOP_HOLD_SECONDS,
        "slide_after_odom_stop_m": slide,
        "max_linear_difference_m_s": linear_error,
        "max_angular_difference_rad_s": angular_error,
        "samples_after_cut": len(after),
        "recorded_after_cut_s": after[-1]["t"] - cut_time,
        "max_sample_gap_s": max(later - earlier for earlier, later in zip(times, times[1:])),
        "max_tilt_rad": max(sample["tilt"] for sample in samples),
        "max_base_height_change_m": max(sample["height_change"] for sample in samples),
        "cargo_max_offset_m": max((value[0] for value in cargo), default=None),
        "cargo_max_height_error_m": max((value[1] for value in cargo), default=None),
        "deceleration_onset_s": {
            key: deceleration_onset(samples, cut_time, key)
            for key in ("odom_v", "odom_w", "truth_v", "truth_w")
        },
    }


# 정지 판정 결과를 출력하고 결과 사전(정지 미확인이면 None)과 실패 사유를 돌려준다
def evaluate(name, samples, cut_time, window=None, has_arm=True):
    assessment = assess_stop(samples, cut_time, window)

    if not assessment["stopped"]:
        print(f"[FAIL] {name}: 정지 미확인 - {assessment['reason']}", flush=True)

        return None, assessment

    result = {key: value for key, value in assessment.items() if key != "stopped"}

    if not has_arm:
        result["arm_stop"] = "not_applicable"

    print(
        f"[PASS] {name}: 정지까지 odom {result['odom_stop_s']:.3f} s, "
        f"실제 {result['truth_stop_s']:.3f} s(차이 {result['stop_time_difference_s'] * 1000:+.0f} ms), "
        f"odom 정지 후 실제 이동 {result['slide_after_odom_stop_m'] * 1000:.1f} mm, "
        f"구동 중 최대 차이 선속도 {result['max_linear_difference_m_s']:.3f} m/s, "
        f"각속도 {result['max_angular_difference_rad_s']:.3f} rad/s",
        flush=True,
    )

    return result, None


SUMMARY_KEYS = (
    "odom_stop_s", "truth_stop_s", "slide_after_odom_stop_m",
    "stop_time_difference_s", "odom_hold_confirmed_s", "truth_hold_confirmed_s",
)


# 시나리오별 반복 결과의 평균·표준편차·범위를 출력하고 사전으로 돌려준다
def summarize_results(results, report=True):
    summary = {}

    for name, runs in results.items():
        values = [run for run in runs if run is not None]
        modes = {run["stop_mode"] for run in values if "stop_mode" in run}

        if len(modes) > 1:
            raise ValueError(f"{name}: 다른 stop_mode 결과를 함께 집계할 수 없음 {sorted(modes)}")

        summary[name] = {
            "runs": len(runs),
            "stopped": len(values),
            "stop_mode": modes.pop() if modes else None,
            **{key: summarize([run.get(key) for run in values]) for key in SUMMARY_KEYS},
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
# runs의 실패 항목은 기존처럼 null이고, 사유는 failure_details에 따로 쌓는다.
def new_record(robot_id, repeats, payload_kg=0.0, stop_mode="command_loss"):
    return {
        "robot_id": robot_id,
        "repeats": repeats,
        "stop_mode": stop_mode,
        "status": "running",
        "error": None,
        "last_scenario": None,
        "failures": 0,
        "failure_details": [],
        "criterion": {
            "linear_m_s": STOP_LINEAR_M_S,
            "angular_rad_s": STOP_ANGULAR_RAD_S,
            "arm_rad_s": STOP_ARM_RAD_S,
            "hold_s": STOP_HOLD_SECONDS,
            "arm_applies": None,
        },
        "observations": None,
        "arm_scenarios": None,
        "conditions": {
            "payload_kg": payload_kg,
            "stop_mode": stop_mode,
            "stop_reference": STOP_MODE_REFERENCE[stop_mode],
            "stop_mode_scope": "drive scenarios only; arm scenarios use the arm Action result time",
        },
        "feedback": {},
        "runs": {},
        "summary": {},
        "note": "Provisional physics (mass, friction, effort). Measured values are not stop performance.",
    }


# 팔 유무에 맞춰 관측 항목과 팔 시나리오 적용 여부를 기록한다
def set_observations(record, has_arm):
    record["criterion"]["arm_applies"] = has_arm
    record["observations"] = {
        "drive": ["joint_states wheels", "diff_drive_controller/odom", "gazebo model pose"],
        "arm": ["joint_states arm", "arm_controller action", "gazebo arm link poses"] if has_arm else "not_applicable",
    }
    record["arm_scenarios"] = (
        "measured" if has_arm
        else {"status": "not_applicable", "reason": "팔 관절 없음(URDF). 팔 정지 성공으로 기록하지 않음"}
    )


# 기록을 임시 파일에 쓴 뒤 바꿔 넣는다. 쓰는 도중 중단돼도 이전 result.json이 깨지지 않는다.
def write_record(path, record, report=False):
    record["summary"] = summarize_results(record["runs"], report)
    write_json_atomic(path, record)


# 시나리오 한 건의 결과를 기록에 넣고 바로 저장한다. 실패면 failure가 사유 사전이다
def add_result(record, save, name, label, result, failure=None):
    record["runs"].setdefault(name, []).append(result)
    record["last_scenario"] = label
    record["failures"] += 0 if result else 1

    if not result:
        detail = {"scenario": label}
        detail.update(failure or {"reason": "unknown"})
        record.setdefault("failure_details", []).append(detail)

    save()


# 실행 중 diff_drive_controller 파라미터를 읽는다. 읽지 못하면 None(조건 기록용, 실패로 보지 않음)
def live_drive_parameters(node, keys):
    from rcl_interfaces.srv import GetParameters
    from rclpy.parameter import parameter_value_to_python

    client = node.create_client(GetParameters, f"{node.namespace}/diff_drive_controller/get_parameters")

    try:
        if not client.wait_for_service(timeout_sec=5.0):
            return None

        future = client.call_async(GetParameters.Request(names=list(keys)))
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

        if future.result() is None:
            return None

        return {key: parameter_value_to_python(value) for key, value in zip(keys, future.result().values)}

    finally:
        node.destroy_client(client)


# 로봇·월드·모델·마찰·제어기 설정을 시험 조건으로 남긴다
def record_conditions(record, node, urdf, drive):
    keys = [
        "wheel_separation", "wheel_radius", "wheel_separation_multiplier",
        "left_wheel_radius_multiplier", "right_wheel_radius_multiplier", "cmd_vel_timeout",
    ] + [f"{axis}.{limit}" for axis in ("linear.x", "angular.z")
         for limit in ("max_velocity", "max_acceleration", "max_deceleration")]

    record["conditions"].update({
        "robot_id": node.robot_id,
        "has_arm": node.has_arm,
        "world": TRUTH_WORLD,
        "truth_topic": TRUTH_TOPIC,
        "robot_description_sha256": hashlib.sha256(urdf.encode("utf-8")).hexdigest(),
        "wheel_contacts": wheel_contacts(urdf),
        "controller_config_sha256": {
            name: file_sha256(CONTROL_CONFIG / name)
            for name in ("common_controllers.yaml", "ax1_controllers.yaml", "ax2_controllers.yaml")
        },
        "scenario_limits_from_config": {key: drive.get(key) for key in keys if key in drive},
        "diff_drive_live": live_drive_parameters(node, keys),
        "tilt_abort_rad": node.tilt_abort,
        "base_height_abort_m": BASE_HEIGHT_ABORT_M,
    })

    live = record["conditions"]["diff_drive_live"] or {}
    timeout = live.get("cmd_vel_timeout")
    record["conditions"]["cmd_vel_timeout_s"] = timeout
    record["conditions"]["cmd_vel_timeout_source"] = "live" if timeout is not None else "config"

    if timeout is None:
        record["conditions"]["cmd_vel_timeout_s"] = drive.get("cmd_vel_timeout")


# 적재 조건(A1 이동 시험과 같은 20 cm 상자)을 준비한다. 상자를 cargo_link 중심 위에 만들고
# 적재판에 지지되는지 확인한다. 고정 관절·마찰 변경은 하지 않는다.
def prepare_payload(node, record, urdf, mass, box_size):
    from check_tool_attach import spawn_box

    node.cargo_frame = link_frame(read_urdf_joints(urdf), CARGO_FRAME)
    spawn = (node.robot_matrix @ node.cargo_frame)[:3, 3] + np.array([0.0, 0.0, CARGO_SPAWN_HEIGHT_M])

    record["conditions"]["payload"] = {
        "mass_kg": mass,
        "box_size_m": list(box_size),
        "model": PAYLOAD_MODEL,
        "method": "ros_gz_sim create, cargo_link 중심 +0.12 m, 기본 표면 접촉(고정 관절·마찰 변경 없음). "
                  "A1 verify_a1_envelope.py --mobility와 같은 방법",
        "spawn_world_m": spawn.tolist(),
        "drift_abort_m": CARGO_DRIFT_ABORT_M,
        "height_abort_m": CARGO_HEIGHT_ABORT_M,
    }

    # 생성 명령이 실패해도 모델이 남았을 수 있으므로 먼저 표시해 종료 때 지우게 한다.
    node.payload_spawned = True
    spawn_box(PAYLOAD_MODEL, spawn.tolist(), box_size, static=False, mass=mass)

    deadline = time.monotonic() + 30.0

    while node.payload_matrix is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)

    if node.payload_matrix is None:
        raise RuntimeError("적재 상자 자세를 받지 못함")

    node.settle_after(node.now_seconds(), CARGO_SETTLE_SECONDS)
    drift, height_error = cargo_offset(node.robot_matrix, node.cargo_frame, node.payload_matrix, box_size[2])
    record["conditions"]["payload"]["settled_offset_m"] = drift
    record["conditions"]["payload"]["settled_height_error_m"] = height_error

    if drift > CARGO_DRIFT_ABORT_M or height_error > CARGO_HEIGHT_ABORT_M:
        raise RuntimeError(f"적재판이 상자를 지지하지 못함: 중심 수평 {drift:.3f} m, 높이 차 {height_error:.3f} m")

    # 보호 조건 감시는 상자가 자리 잡은 뒤부터 켠다.
    node.payload_height = box_size[2]

    print(f"[INFO] 적재 {mass:g} kg 상자 준비: 중심 수평 {drift * 1000:.1f} mm, 높이 차 {height_error * 1000:.1f} mm", flush=True)


# 시나리오를 repeats회 실행하고 결과를 record에 쌓는다. 시나리오가 끝날 때마다 save()로 저장한다.
# 주행 시나리오마다 측정 뒤 반대 명령으로 되돌아가 출발점 근처를 유지한다(되돌아가는 구간은 측정하지 않음).
# 팔 시나리오는 팔이 있는 로봇에서만 실행한다.
def run_checks(node, record, save, repeats=1, urdf=None, payload=None, stop_mode="command_loss"):
    deadline = time.monotonic() + 20.0

    while node.last_sample_time is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)

    if node.last_sample_time is None:
        missing = [name for name in required_inputs(node.has_arm) if name not in node.input_stamps]
        raise RuntimeError(f"측정값 수신 없음({', '.join(missing) or '최근 입력'})")

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
    control = yaml.safe_load((CONTROL_CONFIG / 'common_controllers.yaml').read_text(encoding='utf-8'))
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

    if urdf is not None:
        record_conditions(record, node, urdf, drive)

    timeout = record["conditions"].get("cmd_vel_timeout_s", drive.get("cmd_vel_timeout"))
    save()

    if payload is not None:
        prepare_payload(node, record, urdf, *payload)
        save()

    for repeat in range(1, repeats + 1):
        for name, linear, angular, seconds in drive_scenarios(maximum_linear, maximum_angular):
            node.samples.clear()

            # 감속 제한(command_loss는 timeout 대기 포함)으로 정지가 길어져도 D-06 유지 구간까지 기록하도록 창을 계산한다.
            window = mode_settle_seconds(stop_mode, linear, angular, drive, timeout)

            reference, command = node.drive_and_stop(linear, angular, seconds, stop_mode)
            node.settle_after(reference, window)
            command.update(node.end_zero_hold())

            result, failure = evaluate(f"{name} #{repeat}", node.samples, reference, window, node.has_arm)
            entry = result if result else failure
            entry["record_window_s"] = window
            entry["stop_mode"] = stop_mode
            entry["command"] = command

            add_result(record, save, name, f"{name} #{repeat}", result, failure)

            # 같은 경로를 거꾸로 되돌아간다. 측정하지 않으며 같은 정지 방식으로 멈춘다.
            back_reference, _ = node.drive_and_stop(-linear, -angular, seconds, stop_mode)
            node.settle_after(back_reference, window)
            node.end_zero_hold()

    if not node.has_arm:
        print("[INFO] 팔 시나리오: 해당 없음(팔 관절 없음). 팔 정지 성공으로 기록하지 않음", flush=True)

        return

    if not node.arm_client.wait_for_server(timeout_sec=10.0):
        raise RuntimeError("팔 Action 서버 없음\n" + describe_environment(node, node.robot_id))

    for repeat in range(1, repeats + 1):
        for name, positions in ARM_TARGETS:
            node.samples.clear()

            cut_time = node.run_arm(positions, 2)

            if cut_time is None:
                print(f"[FAIL] {name} #{repeat}: 궤적 거절", flush=True)
                add_result(record, save, name, f"{name} #{repeat}", None, {"reason": "궤적 거절"})

                continue

            node.settle_after(cut_time)

            result, failure = evaluate(f"{name} #{repeat}", node.samples, cut_time, SETTLE_SIM_SECONDS)
            add_result(record, save, name, f"{name} #{repeat}", result, failure)


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


# A1 시험 조건의 적재 질량 목록과 상자 크기를 읽는다
def a1_payload_conditions():
    settings = yaml.safe_load(A1_VALIDATION.read_text(encoding="utf-8"))
    robot = settings["mobility"]["robot_2"]

    return [float(value) for value in robot["test_payloads_kg"]], [float(value) for value in robot["test_box_size_m"]]


# 명령행 인자 해석기를 만든다
def build_parser():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--robot-id", default="robot_1")
    parser.add_argument("--repeats", type=int, default=1, help="시나리오별 반복 횟수")
    parser.add_argument("--output", type=Path, default=None, help="결과 폴더(기본 log/a2_stop_feedback/<KST 시각>)")
    parser.add_argument("--payload-kg", type=float, default=0.0,
                        help="팔 없는 적재 로봇의 시험 상자 질량(kg). a1_validation.yaml test_payloads_kg만 허용. 기본 0(무부하)")
    parser.add_argument("--tilt-abort", type=float, default=DEFAULT_TILT_ABORT_RAD,
                        help="차체 기울기가 이 값(rad)을 넘으면 시험 중단. 시험 보호용이며 성능 기준이 아님")
    parser.add_argument("--stop-mode", choices=STOP_MODES, default="command_loss",
                        help="주행 정지 방식. command_loss(기본, 발행 중단·timeout 감속) 또는 zero_command(0 명령 즉시·유지)")

    return parser


# 명령행 인자를 읽어 측정을 실행한다
def main():
    parser = build_parser()
    arguments = parser.parse_args()

    if arguments.repeats < 1:
        parser.error("--repeats는 1 이상")

    payload = None

    if arguments.payload_kg:
        allowed, box_size = a1_payload_conditions()

        if not any(abs(arguments.payload_kg - value) < 1e-9 for value in allowed):
            parser.error(f"--payload-kg는 A1 시험 조건 {allowed} 중 하나")

        payload = (arguments.payload_kg, box_size)

    stamp = datetime.now(ZoneInfo("Asia/Seoul")).strftime("%Y%m%d_%H%M%S")
    output = arguments.output or WORKSPACE / "log/a2_stop_feedback" / stamp

    if output.exists() and any(output.iterdir()):
        parser.error(f"{output}이 비어 있지 않음")

    output.mkdir(parents=True, exist_ok=True)
    path = output / "result.json"
    record = new_record(arguments.robot_id, arguments.repeats, arguments.payload_kg, arguments.stop_mode)

    # 중간 저장 함수. 시작 즉시 running 상태로 한 번 써 둔다.
    def save():
        write_record(path, record)

    save()

    truth_poses = queue.Queue()
    reader = start_truth_reader(arguments.robot_id, truth_poses)

    rclpy.init()
    node = StopFeedbackRecorder(arguments.robot_id, truth_poses, arguments.tilt_abort)
    node.payload_spawned = False

    try:
        urdf = fetch_robot_description(
            node,
            arguments.robot_id,
            spin=lambda: rclpy.spin_once(node, timeout_sec=0.05),
        )
        urdf_joints = read_urdf_joints(urdf)

        # 팔 유무는 실행 중 모델의 URDF로 정한다. 팔이 있으면 팔 관측·Action·팔 시나리오를 요구한다.
        has_arm = arm_presence(urdf_joints, arguments.robot_id)
        set_observations(record, has_arm)

        if payload is not None:
            if has_arm:
                raise RuntimeError("--payload-kg는 팔 없는 적재 로봇 전용. 팔 로봇 하중 시험은 check_payload_tracking.py")

            if not any(child == CARGO_FRAME for _, _, child, _, _, _ in urdf_joints.values()):
                raise RuntimeError(f"{arguments.robot_id}에 {CARGO_FRAME} 없음. 적재 시험 대상이 아님")

        if has_arm:
            node.enable_arm()

            # 팔 실제 속도용 Gazebo 링크 관측기. 관절각 재계산에 실행 중인 모델의 URDF를 쓴다.
            node.observer = GazeboObserver(arguments.robot_id, urdf, ARM_JOINTS, "gripper_base_link", None)

            try:
                node.observer.start()
            except RuntimeError as error:
                raise RuntimeError(f"{error}\n{describe_environment(node, arguments.robot_id)}") from None

        print(f"[INFO] {arguments.robot_id}: 팔 {'있음' if has_arm else '없음(팔 시나리오 해당 없음)'}, "
              f"하중 {arguments.payload_kg:g} kg, 정지 방식 {arguments.stop_mode}", flush=True)

        run_checks(node, record, save, arguments.repeats, urdf, payload, arguments.stop_mode)
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
        # 중단·실패에도 바퀴 정지 명령을 보낸다.
        if rclpy.ok():
            node.stop_drive()

        # 시험이 만든 적재 상자를 지운다. 지우지 못하면 기록에 남긴다.
        if node.payload_spawned:
            from check_tool_attach import remove_model

            record["conditions"].setdefault("payload", {})["removed"] = remove_model(PAYLOAD_MODEL)

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
