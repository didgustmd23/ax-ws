#!/usr/bin/env python3
"""A2 개발용 단계 하중 추종·유지·정지 시험. 흡착으로 붙인 상자를 하중으로 쓴다.

시스템 보고값과 Gazebo 독립 관측을 짝으로 기록한다.
- 추종·유지: joint_states 위치 ↔ Gazebo 링크 자세로 다시 계산한 관절각
- 정지: joint_states 속도 ↔ Gazebo 관절각 차분 속도 (같은 D-06 팔 기준)
- 부착: attach_state ↔ Gazebo 상자 실제 위치(들어 올림·TCP 기준 미끄러짐·낙하)

판정은 두 종류다.
- 구조 판정(PASS/FAIL): Action 성공, 두 관측의 일치, 부착 신호와 실제 위치의 일치, effort 포화, 차체 기울기
- 수치 기록: 추종 오차·정지 시간·effort 비율. 물리 설정(effort·질량·흡착 모델)이 잠정값이므로
  합격 기준 없이 기록만 한다. 이 값을 AX 하중 성능으로 보고하지 않는다.

모의 A3·A4를 띄우고 제어기를 A4 없이 직접 구동하므로 격리된 개발 시뮬레이션에서만 실행한다.
진공·흡착력 물리는 없으며 부착은 고정 관절이다. 상자가 떨어지지 않은 것은 흡착력 검증이 아니다.

실행 순서
  터미널 1: ros2 launch ax_control simulation.launch.py robot_ids:=robot_1 gui:=false
  터미널 2: python3 src/ax_control/scripts/check_payload_tracking.py --robot-id robot_1

결과는 기본으로 log/a2_payload/<KST 시각>/에 단계별 JSON과 summary.json으로 남는다.
실제 A3·A4가 떠 있는 환경에서는 실행하지 않는다(같은 서비스·토픽이 겹친다).
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
import uuid
from zoneinfo import ZoneInfo

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from ax_interfaces.srv import ToolCommand
from control_msgs.action import FollowJointTrajectory
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from check_stop_feedback import STOP_ARM_RAD_S, STOP_HOLD_SECONDS
from check_tool_attach import (
    ARM_JOINTS,
    BOX_BELOW_TCP_M,
    BOX_SIZE_M,
    FLOOR_BOX_Z_M,
    LIFT_SHOULDER_RAD,
    MAX_BOX_MASS_KG,
    MIN_LIFT_M,
    PICK_YAW_RAD,
    STAND_SIZE_M,
    ToolAttachTester,
    remove_model,
    spawn_box,
)
from gazebo_observer import (
    GazeboObserver,
    describe_environment,
    fetch_robot_description,
    first_stable_below,
    in_tcp_frame,
    joint_speed_series,
    nearest_pairs,
    tilt_angle,
)

WORKSPACE = Path(__file__).resolve().parents[3]

# 시험 자세(rad, ARM_JOINTS 순서). 앞의 셋은 A1 검증 자세(a1_validation.yaml)와 같다.
# reach_side·reach_max는 어깨 축에서 TCP까지 수평 거리가 약 1.0 m·1.29 m인 공구 하향 자세다.
# tools/arm_geometry.py 보수 충돌 선별(매단 0.15 m 상자 포함)을 통과하고 관절 한계에서 0.1 rad 이상 떨어지게 골랐다.
POSES = {
    "neutral": [0.0, 0.0, 0.0, 0.0, 0.0],
    "reach_left": [0.35, -0.30, 0.35, -0.20, 0.25],
    "reach_right": [-0.35, 0.30, -0.35, 0.20, -0.25],
    "reach_side": [0.0, -0.90, 1.25, -0.40, 0.0],
    "reach_max": [0.0, -1.35, 1.90, -0.25, 0.0],
}
PICK_POSE = [PICK_YAW_RAD, 0.0, 0.0, 0.0, 0.0]
LIFT_POSE = [PICK_YAW_RAD, LIFT_SHOULDER_RAD, 0.0, 0.0, 0.0]

# 운반 중 취소 정지: STOP_FROM에서 STOP_TO로 가는 긴 궤적을 도중에 취소한다(시뮬레이션 s).
STOP_FROM = "neutral"
STOP_TO = "reach_side"
STOP_GOAL_SECONDS = 6.0
STOP_CANCEL_AFTER_SECONDS = 1.5
STOP_RECORD_SECONDS = 3.0

# 자세 도달 뒤 안정화 시간(s). 유지 구간은 --hold로 정한다.
SETTLE_SECONDS = 0.3

# 개발용 구조 판정 기준. 성능 합격 기준이 아니다.
# 두 관측 경로의 일치(rad): 같은 물리 상태를 읽는지 확인하는 값이다.
AGREEMENT_TOLERANCE_RAD = 1e-3
# effort가 URDF 한계의 이 비율 이상이면 포화로 본다. 이때 추종 수치는 모델 한계에 걸린 값이다.
EFFORT_SATURATION_RATIO = 0.95
# 차체 기울기(rad). A1 검증과 같은 중단 기준이다.
TILT_LIMIT_RAD = 0.25
# 해제 뒤 상자 중심이 바닥 높이 + 이 값 안에 있으면 떨어진 것으로 본다(m).
FALL_MARGIN_M = 0.05

# 관측 처리 값. Gazebo 속도 차분 창(s), 두 관측 짝짓기 최대 시각 차(s), joint_states 이력 길이(s).
GAZEBO_SPEED_WINDOW_SECONDS = 0.1
PAIR_MAX_GAP_SECONDS = 0.02
JOINT_HISTORY_SECONDS = 30.0


# 차체 기울기가 기준을 넘어 시험을 멈출 때 쓰는 예외
class TiltExceeded(RuntimeError):
    pass


# 모의 A3·A4와 공구 클라이언트(ToolAttachTester)에 joint_states 기록과 궤적 실행을 더한 시험 노드
class PayloadTester(ToolAttachTester):
    # joint_states 이력과 관절 제한을 준비한다
    def __init__(self, robot_id, limits):
        super().__init__(robot_id)

        self.limits = limits
        self.observer = None
        self.state_lock = threading.Lock()
        self.joint_history = deque()

        self.create_subscription(
            JointState,
            f"{self.namespace}/joint_states",
            self.on_joint_state,
            20,
        )

    # 팔 관절의 위치·속도·effort를 측정 시각과 함께 저장한다
    def on_joint_state(self, message):
        names = list(message.name)

        if not all(joint in names for joint in ARM_JOINTS):
            return

        indices = [names.index(joint) for joint in ARM_JOINTS]
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9

        sample = {
            "stamp": stamp,
            "position": [message.position[index] for index in indices],
            "velocity": [message.velocity[index] for index in indices],
            "effort": [message.effort[index] if message.effort else math.nan for index in indices],
        }

        with self.state_lock:
            if self.joint_history and stamp < self.joint_history[-1]["stamp"]:
                self.joint_history.clear()

            self.joint_history.append(sample)

            while self.joint_history and stamp - self.joint_history[0]["stamp"] > JOINT_HISTORY_SECONDS:
                self.joint_history.popleft()

    # 현재 시뮬레이션 시각(s)
    def sim_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # 최신 팔 관절 위치(rad)
    def current_positions(self):
        with self.state_lock:
            if not self.joint_history:
                raise RuntimeError("joint_states 없음")

            return list(self.joint_history[-1]["position"])

    # 구간 [start, end]의 joint_states 표본 목록
    def joint_samples(self, start, end=math.inf):
        with self.state_lock:
            return [sample for sample in self.joint_history if start <= sample["stamp"] <= end]

    # 두 관측이 stamp(시뮬레이션 s)까지 도착할 때까지 기다린다. 처리 지연으로 구간 끝이 비지 않게 한다
    def wait_records(self, stamp, wall_limit_seconds=30.0):
        if not self.observer.wait_until(stamp, wall_limit_seconds):
            raise RuntimeError("Gazebo 관측이 구간 끝까지 도착하지 않음")

        deadline = time.monotonic() + wall_limit_seconds

        while time.monotonic() < deadline:
            with self.state_lock:
                if self.joint_history and self.joint_history[-1]["stamp"] >= stamp:
                    return

            time.sleep(0.02)

        raise RuntimeError("joint_states가 구간 끝까지 도착하지 않음")

    # 차체 기울기가 기준을 넘으면 예외를 낸다
    def guard_tilt(self):
        latest = self.observer.latest()

        if latest is not None and tilt_angle(latest["base_world"]) > TILT_LIMIT_RAD:
            raise TiltExceeded(f"차체 기울기 {tilt_angle(latest['base_world']):.3f} rad 초과")

    # 관절 속도·가속도 제한으로 궤적 시간(s)을 정한다. A1 검증 도구와 같은 계산이다
    def trajectory_seconds(self, start, target):
        seconds = 1.5

        for joint, first, last in zip(ARM_JOINTS, start, target):
            distance = abs(last - first)
            limit = self.limits[joint]

            seconds = max(
                seconds,
                1.6 * distance / limit["max_velocity"],
                1.05 * math.sqrt(6 * distance / limit["max_acceleration"]),
            )

        return seconds

    # 한 점 궤적 목표를 보내고 수락된 handle을 돌려준다
    def send_goal(self, target, seconds):
        point = JointTrajectoryPoint()
        point.positions = list(target)
        point.velocities = [0.0] * len(target)
        point.time_from_start = Duration(seconds=seconds).to_msg()

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS
        goal.trajectory.points = [point]

        done = threading.Event()
        future = self.arm_client.send_goal_async(goal)
        future.add_done_callback(lambda _: done.set())

        if not done.wait(10.0) or not future.result().accepted:
            raise RuntimeError("팔 궤적 거절")

        return future.result()

    # 비동기 결과를 벽시계 상한 안에서 기다린다. 기다리는 동안 차체 기울기를 감시한다
    def wait_future(self, future, wall_limit_seconds):
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        deadline = time.monotonic() + wall_limit_seconds

        while not done.wait(0.05):
            self.guard_tilt()

            if time.monotonic() > deadline:
                raise RuntimeError("팔 Action 응답 시간 초과")

        return future.result()

    # 목표 자세로 움직이고 (Action 상태, error_code)를 돌려준다
    def move(self, target):
        seconds = self.trajectory_seconds(self.current_positions(), target)
        handle = self.send_goal(target, seconds)
        response = self.wait_future(handle.get_result_async(), 60.0 + 10.0 * seconds)

        return response.status, response.result.error_code

    # 시뮬레이션 시간 대기 중에도 차체 기울기를 감시한다
    def wait_sim_guarded(self, seconds):
        target = self.sim_now() + seconds
        deadline = time.monotonic() + 120.0

        while self.sim_now() < target:
            self.guard_tilt()

            if time.monotonic() > deadline:
                raise RuntimeError("시뮬레이션 시간이 진행되지 않음")

            time.sleep(0.01)


# 단계 하나의 결과를 모으고 구조 판정을 출력한다
class StageReport:
    # 하중 질량(kg)으로 빈 결과를 만든다
    def __init__(self, mass):
        self.data = {
            "box_mass_kg": mass,
            "load": "suction-attached box (fixed joint, no vacuum physics)" if mass > 0 else "none",
            "checks": [],
            "hold_cases": [],
            "stop_cases": [],
            "attach": {},
        }

    # 구조 판정 한 건을 기록·출력한다
    def check(self, name, passed, detail):
        self.data["checks"].append({"name": name, "passed": bool(passed), "detail": detail})
        print(f"[{'PASS' if passed else 'FAIL'}] {self.data['box_mass_kg']} kg {name}: {detail}", flush=True)

    # 구조 판정 실패 수
    def failures(self):
        return sum(1 for item in self.data["checks"] if not item["passed"])


# 자세로 이동해 안정화 뒤 유지 구간을 두 경로로 기록한다
def hold_case(node, report, label, target, hold_seconds, box_name, box_reference):
    status, error_code = node.move(target)
    report.check(f"{label} 궤적", status == 4 and error_code == 0, f"상태 {status}, error_code {error_code}")

    node.wait_sim_guarded(SETTLE_SECONDS)

    start = node.sim_now()
    node.wait_sim_guarded(hold_seconds)
    end = node.sim_now()
    node.wait_records(end)

    joint_samples = node.joint_samples(start, end)
    gazebo_samples = [item for item in node.observer.since(start) if item["stamp"] <= end]

    if not joint_samples or not gazebo_samples:
        report.check(f"{label} 관측", False, f"표본 joint_states {len(joint_samples)}, Gazebo {len(gazebo_samples)}")

        return

    # 추종·유지 오차: 두 경로 각각 목표와의 최대 차이
    reported_error = max(
        abs(position - goal)
        for sample in joint_samples
        for position, goal in zip(sample["position"], target)
    )
    observed_error = max(
        abs(item["joints"][joint] - goal)
        for item in gazebo_samples
        for joint, goal in zip(ARM_JOINTS, target)
    )

    # 같은 시각의 두 관측 차이
    pairs = nearest_pairs(
        [(sample["stamp"], sample["position"]) for sample in joint_samples],
        [(item["stamp"], item["joints"]) for item in gazebo_samples],
        PAIR_MAX_GAP_SECONDS,
    )
    disagreement = max(
        (abs(position - observed[joint]) for reported, observed in pairs for position, joint in zip(reported, ARM_JOINTS)),
        default=math.inf,
    )
    residual = max(item["residual"] for item in gazebo_samples)

    # 유지 중 TCP 이동(m)
    tcp_start = gazebo_samples[0]["tcp_world"][:3, 3]
    tcp_drift = max(float(np.linalg.norm(item["tcp_world"][:3, 3] - tcp_start)) for item in gazebo_samples)

    # effort(N·m)와 URDF 한계 대비 비율
    limits = node.observer.effort_limits()
    effort = {
        joint: max(abs(sample["effort"][index]) for sample in joint_samples)
        for index, joint in enumerate(ARM_JOINTS)
    }
    ratio = {joint: effort[joint] / limits[joint] for joint in ARM_JOINTS if limits[joint]}

    case = {
        "pose": label,
        "target_rad": target,
        "samples": {"joint_states": len(joint_samples), "gazebo": len(gazebo_samples), "pairs": len(pairs)},
        "max_error_joint_states_rad": reported_error,
        "max_error_gazebo_rad": observed_error,
        "max_joint_states_vs_gazebo_rad": disagreement,
        "max_reconstruction_residual_rad": residual,
        "tcp_drift_m": tcp_drift,
        "max_base_tilt_rad": max(tilt_angle(item["base_world"]) for item in gazebo_samples),
        "max_effort_nm": effort,
        "effort_ratio": ratio,
    }

    # 부착 하중: TCP 기준 상자 상대 위치 변화(미끄러짐)와 부착 신호
    if box_name:
        slips = [
            float(np.linalg.norm(in_tcp_frame(item["tcp_world"], item["models"][box_name])[:3, 3] - box_reference))
            for item in gazebo_samples
            if box_name in item["models"]
        ]
        case["box_slip_in_tcp_m"] = max(slips, default=math.inf)
        case["attach_signal"] = bool(node.attach_state and node.attach_state.attached)

        report.check(
            f"{label} 부착 유지",
            case["attach_signal"] and case["box_slip_in_tcp_m"] < 0.01,
            f"신호 {case['attach_signal']}, TCP 기준 상자 이동 {case['box_slip_in_tcp_m'] * 1000:.2f} mm",
        )

    report.data["hold_cases"].append(case)

    report.check(
        f"{label} 관측 일치",
        disagreement <= AGREEMENT_TOLERANCE_RAD and residual <= AGREEMENT_TOLERANCE_RAD,
        f"joint_states↔Gazebo {disagreement:.2e} rad, 축 밖 잔여 {residual:.2e} rad",
    )

    saturated = [joint for joint, value in ratio.items() if value >= EFFORT_SATURATION_RATIO]
    report.check(
        f"{label} effort 포화 없음",
        not saturated,
        "최대 비율 " + ", ".join(f"{joint} {value:.0%}" for joint, value in ratio.items()),
    )

    print(
        f"[INFO] {label}: 오차 joint_states {reported_error:.2e} rad, Gazebo {observed_error:.2e} rad, "
        f"TCP 이동 {tcp_drift * 1000:.2f} mm, 차체 기울기 {case['max_base_tilt_rad']:.4f} rad",
        flush=True,
    )


# 이동 도중 취소하고 두 경로의 정지 시각과 정지 뒤 이동을 기록한다
def stop_case(node, report, label, box_name, box_reference):
    status, error_code = node.move(POSES[STOP_FROM])
    report.check(f"{label} 시작 자세", status == 4 and error_code == 0, f"상태 {status}")

    handle = node.send_goal(POSES[STOP_TO], STOP_GOAL_SECONDS)
    node.wait_sim_guarded(STOP_CANCEL_AFTER_SECONDS)

    cancel_time = node.sim_now()
    cancel = node.wait_future(handle.cancel_goal_async(), 20.0)
    response = node.wait_future(handle.get_result_async(), 30.0)

    node.wait_sim_guarded(STOP_RECORD_SECONDS)
    node.wait_records(node.sim_now())

    # 보고 경로: joint_states 속도 최대값
    reported = [
        (sample["stamp"], max(abs(value) for value in sample["velocity"]))
        for sample in node.joint_samples(cancel_time)
    ]

    # 독립 경로: Gazebo 관절각 차분 속도 최대값
    gazebo_samples = node.observer.since(cancel_time - GAZEBO_SPEED_WINDOW_SECONDS)
    observed = [
        item for item in joint_speed_series(
            [(item["stamp"], item["joints"]) for item in gazebo_samples],
            GAZEBO_SPEED_WINDOW_SECONDS,
        )
        if item[0] >= cancel_time
    ]

    reported_stop = first_stable_below(reported, STOP_ARM_RAD_S, STOP_HOLD_SECONDS)
    observed_stop = first_stable_below(observed, STOP_ARM_RAD_S, STOP_HOLD_SECONDS)

    # Gazebo 정지 시각 뒤 TCP 이동(m)
    after_stop = [item for item in gazebo_samples if observed_stop is not None and item["stamp"] >= observed_stop]
    drift = math.inf

    if after_stop:
        anchor = after_stop[0]["tcp_world"][:3, 3]
        drift = max(float(np.linalg.norm(item["tcp_world"][:3, 3] - anchor)) for item in after_stop)

    case = {
        "case": label,
        "cancel_accepted": bool(cancel.goals_canceling),
        "action_status": response.status,
        "stop_time_joint_states_s": None if reported_stop is None else reported_stop - cancel_time,
        "stop_time_gazebo_s": None if observed_stop is None else observed_stop - cancel_time,
        "tcp_drift_after_stop_m": drift,
        "criterion": {"arm_rad_s": STOP_ARM_RAD_S, "hold_s": STOP_HOLD_SECONDS},
    }

    if box_name:
        slips = [
            float(np.linalg.norm(in_tcp_frame(item["tcp_world"], item["models"][box_name])[:3, 3] - box_reference))
            for item in gazebo_samples
            if box_name in item["models"] and item["stamp"] >= cancel_time
        ]
        case["box_slip_in_tcp_m"] = max(slips, default=math.inf)
        case["attach_signal"] = bool(node.attach_state and node.attach_state.attached)

        report.check(
            f"{label} 부착 유지",
            case["attach_signal"] and case["box_slip_in_tcp_m"] < 0.01,
            f"신호 {case['attach_signal']}, TCP 기준 상자 이동 {case['box_slip_in_tcp_m'] * 1000:.2f} mm",
        )

    report.data["stop_cases"].append(case)

    report.check(
        f"{label} 취소·정지 관측",
        case["cancel_accepted"] and response.status == 5 and reported_stop is not None and observed_stop is not None,
        f"취소 {case['cancel_accepted']}, 상태 {response.status}, "
        f"정지 joint_states {case['stop_time_joint_states_s']}, Gazebo {case['stop_time_gazebo_s']}",
    )

    print(
        f"[INFO] {label}: 정지 뒤 TCP 이동 {drift * 1000:.2f} mm",
        flush=True,
    )


# 상자를 만들고 부착해 들어 올린 뒤 TCP 기준 상자 위치를 돌려준다
def attach_box(node, report, mass, box_name, stand_name):
    status, _ = node.move(PICK_POSE)
    report.check("집기 자세", status == 4, f"상태 {status}")
    node.wait_sim_guarded(1.0)

    # 독립 관측의 TCP 위치 아래에 받침대와 상자를 만든다
    tcp = node.observer.latest()["tcp_world"][:3, 3]
    box_z = tcp[2] - BOX_BELOW_TCP_M
    stand_height = box_z - BOX_SIZE_M / 2.0 - 0.002

    spawn_box(stand_name, [tcp[0], tcp[1], stand_height / 2.0], [STAND_SIZE_M, STAND_SIZE_M, stand_height], True)
    spawn_box(box_name, [tcp[0], tcp[1], box_z], [BOX_SIZE_M] * 3, False, mass)
    node.observer.track_model(box_name)
    node.wait_sim_guarded(2.0)

    before = node.observer.latest()["models"].get(box_name)

    if before is None:
        raise RuntimeError(f"{box_name} 자세 없음")

    response = node.send_tool(ToolCommand.Request.ATTACH, box_name)
    signal = node.wait_attached(True, box_name)
    report.check("부착 신호", response.accepted and signal, f"수락 {response.accepted}, 신호 {signal}")

    status, _ = node.move(LIFT_POSE)
    node.wait_sim_guarded(0.5)

    latest = node.observer.latest()
    rise = float(latest["models"][box_name][2, 3] - before[2, 3])
    report.check("들어 올림(Gazebo)", status == 4 and rise > MIN_LIFT_M, f"상승 {rise:.3f} m")

    # 이후 자세 이동에서 받침대와 부딪히지 않도록 지운다
    report.check("받침대 제거", remove_model(stand_name), stand_name)
    node.wait_sim_guarded(0.5)

    latest = node.observer.latest()
    reference = in_tcp_frame(latest["tcp_world"], latest["models"][box_name])[:3, 3]

    report.data["attach"] = {
        "box_name": box_name,
        "lift_m": rise,
        "box_in_tcp_m": reference.tolist(),
    }

    return reference


# 들어 올린 자세에서 해제하고 신호와 실제 낙하를 확인한 뒤 상자를 지운다
def release_box(node, report, box_name):
    status, _ = node.move(LIFT_POSE)
    report.check("해제 자세", status == 4, f"상태 {status}")

    response = node.send_tool(ToolCommand.Request.RELEASE, box_name)
    signal = node.wait_attached(False)
    node.wait_sim_guarded(2.0)

    height = float(node.observer.latest()["models"][box_name][2, 3])
    report.check(
        "해제·낙하",
        response.accepted and signal and height < FLOOR_BOX_Z_M + FALL_MARGIN_M,
        f"수락 {response.accepted}, 신호 {signal}, 상자 높이 {height:.3f} m",
    )

    report.data["attach"]["released_height_m"] = height
    node.observer.untrack_model(box_name)
    remove_model(box_name)


# 질량 한 단계를 실행한다. 0 kg은 상자 없이 같은 자세·정지 시험만 한다
def run_stage(node, mass, repeats, hold_seconds):
    report = StageReport(mass)
    suffix = uuid.uuid4().hex[:8]
    box_name = f"payload_box_{suffix}" if mass > 0 else ""
    stand_name = f"payload_stand_{suffix}"
    reference = None

    print(f"[INFO] ===== {mass} kg 단계 =====", flush=True)

    try:
        status, _ = node.move(POSES["neutral"])
        report.check("시작 자세", status == 4, f"상태 {status}")

        if mass > 0:
            reference = attach_box(node, report, mass, box_name, stand_name)

        for repeat in range(1, repeats + 1):
            for label, target in POSES.items():
                hold_case(node, report, f"{label}/{repeat}", target, hold_seconds, box_name, reference)

        for repeat in range(1, repeats + 1):
            stop_case(node, report, f"stop/{repeat}", box_name, reference)

        if mass > 0:
            release_box(node, report, box_name)

        status, _ = node.move(POSES["neutral"])
        report.check("복귀", status == 4, f"상태 {status}")

    except TiltExceeded as error:
        report.check("차체 기울기", False, str(error))
        report.data["aborted"] = str(error)

    except RuntimeError as error:
        report.check("시험 진행", False, str(error))
        report.data["aborted"] = str(error)

    return report


# numpy 값을 JSON으로 바꿀 수 있게 변환한다
def to_json(value):
    if isinstance(value, dict):
        return {key: to_json(item) for key, item in value.items()}

    if isinstance(value, (list, tuple)):
        return [to_json(item) for item in value]

    if isinstance(value, (np.floating, float)):
        return None if not math.isfinite(float(value)) else float(value)

    if isinstance(value, np.integer):
        return int(value)

    return value


# 명령행 인자를 읽어 단계별 시험을 실행하고 결과 파일을 남긴다
def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--robot-id", default="robot_1")
    parser.add_argument("--masses", type=float, nargs="+", default=[0.0, 0.5, 1.0, 2.0], help="단계 질량(kg)")
    parser.add_argument("--repeats", type=int, default=3, help="자세·정지 반복 횟수")
    parser.add_argument("--hold", type=float, default=1.0, help="유지 기록 구간(시뮬레이션 s)")
    parser.add_argument("--output", type=Path, default=None, help="결과 폴더(기본 log/a2_payload/<KST 시각>)")
    arguments = parser.parse_args()

    if any(not 0.0 <= mass <= MAX_BOX_MASS_KG for mass in arguments.masses):
        parser.error(f"--masses는 0 이상 {MAX_BOX_MASS_KG} kg 이하")

    if arguments.repeats < 1 or arguments.hold <= 0:
        parser.error("--repeats는 1 이상, --hold는 양수")

    stamp = datetime.now(ZoneInfo("Asia/Seoul")).strftime("%Y%m%d_%H%M%S")
    output = arguments.output or WORKSPACE / "log/a2_payload" / stamp

    if output.exists() and any(output.iterdir()):
        parser.error(f"{output}이 비어 있지 않음")

    output.mkdir(parents=True, exist_ok=True)

    share = Path(get_package_share_directory("ax_description"))
    limits = yaml.safe_load((share / "config/arm_joint_limits.yaml").read_text())["joint_limits"]

    rclpy.init()

    node = PayloadTester(arguments.robot_id, limits)

    # 콜백은 별도 스레드 하나에서 처리한다. use_sim_time의 /clock(1 kHz)과 joint_states를
    # 다중 스레드 executor로 받으면 Python 처리 부하로 표본이 밀리므로 단일 스레드를 쓴다.
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    reports = []

    try:
        urdf = fetch_robot_description(node, arguments.robot_id)

        node.observer = GazeboObserver(arguments.robot_id, urdf, ARM_JOINTS, "gripper_base_link", "epick_tcp")

        try:
            node.observer.start()
        except RuntimeError as error:
            raise RuntimeError(f"{error}\n{describe_environment(node, arguments.robot_id)}") from None

        if not node.observer.wait_ready():
            raise RuntimeError("Gazebo 관측 없음\n" + describe_environment(node, arguments.robot_id))

        if not node.tool_client.wait_for_service(timeout_sec=15.0):
            raise RuntimeError("tool_command 서비스 없음\n" + describe_environment(node, arguments.robot_id))

        if not node.arm_client.wait_for_server(timeout_sec=15.0):
            raise RuntimeError("팔 Action 서버 없음\n" + describe_environment(node, arguments.robot_id))

        # 모의 A4 상태가 공구 노드에 도착하고 joint_states가 쌓일 시간을 준다.
        time.sleep(1.0)

        for mass in arguments.masses:
            report = run_stage(node, mass, arguments.repeats, arguments.hold)
            reports.append(report)

            (output / f"stage_{mass:g}kg.json").write_text(
                json.dumps(to_json(report.data), indent=2, ensure_ascii=False) + "\n",
            )

            # 기울기·진행 실패 뒤에는 로봇 상태를 알 수 없으므로 다음 단계를 하지 않는다.
            if "aborted" in report.data:
                print("[INFO] 시험 중단으로 다음 단계를 건너뜀", flush=True)
                break

    finally:
        if node.observer is not None:
            node.observer.stop()

        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

    failures = sum(report.failures() for report in reports)

    summary = {
        "robot_id": arguments.robot_id,
        "masses_kg": arguments.masses,
        "completed_masses_kg": [report.data["box_mass_kg"] for report in reports if "aborted" not in report.data],
        "repeats": arguments.repeats,
        "hold_seconds": arguments.hold,
        "structural_failures": failures,
        "note": "Provisional physics (development effort limits, virtual masses, fixed-joint attach). "
                "Recorded numbers are not AX payload performance.",
    }
    (output / "summary.json").write_text(json.dumps(to_json(summary), indent=2, ensure_ascii=False) + "\n")

    print(f"결과: {output}", flush=True)
    print(f"구조 판정 실패 {failures}건", flush=True)
    sys.exit(1 if failures or len(reports) < len(arguments.masses) else 0)


if __name__ == "__main__":
    main()
