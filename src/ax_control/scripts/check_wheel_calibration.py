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

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient

from gazebo_observer import ModelPoseTracker, quaternion_yaw, summarize, unwrap_angles, yaw_of

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

    # 현재 시뮬레이션 시각(s)
    def sim_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # 실행 중인 diff_drive_controller 파라미터를 읽는다
    def read_parameters(self):
        if not self.parameters.wait_for_services(timeout_sec=15.0):
            raise RuntimeError("diff_drive_controller 파라미터 서비스 없음")

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
            raise RuntimeError("Gazebo 관측이 구간 끝까지 도착하지 않음")

        deadline = time.monotonic() + wall_limit_seconds

        while time.monotonic() < deadline:
            with self.lock:
                if self.odom and self.odom[-1][0] >= stamp:
                    return

            time.sleep(0.02)

        raise RuntimeError("odom이 구간 끝까지 도착하지 않음")

    # 명령 한 번을 실행하고 같은 구간의 odom·Gazebo 회전(rad)과 이동(m)을 돌려준다
    def trial(self, linear, angular, seconds, settle_seconds):
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

        return {
            "command": {"linear_m_s": linear, "angular_rad_s": angular, "seconds": seconds},
            "odom_rotation_rad": accumulated_rotation([(item[0], item[3]) for item in odom]),
            "truth_rotation_rad": accumulated_rotation([(item[0], yaw_of(item[1])) for item in truth]),
            "odom_distance_m": signed_distance(odom[0][1:3], odom[-1][1:3], odom_heading),
            "truth_distance_m": signed_distance(truth[0][1][:2, 3], truth[-1][1][:2, 3], truth_heading),
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

    turns = []

    # 제자리 회전: 왼쪽(+)·오른쪽(-)을 번갈아 같은 횟수 반복한다.
    for repeat in range(1, arguments.repeats + 1):
        for direction in (1.0, -1.0):
            label = f"회전 {'왼쪽' if direction > 0 else '오른쪽'} {repeat}"
            result = node.trial(0.0, direction * arguments.angular, arguments.turn_seconds, arguments.settle)
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
            result = node.trial(direction * arguments.linear, 0.0, arguments.drive_seconds, arguments.settle)
            result["label"] = label

            odom, truth = result["odom_distance_m"], result["truth_distance_m"]
            valid = abs(truth) >= MIN_DISTANCE_M and odom * truth > 0
            result["ratio_odom_over_truth"] = odom / truth if valid else None

            check(checks, label, valid, f"odom {odom:.4f} m, 실제 {truth:.4f} m")
            drives.append(result)

    turn_ratios = [item["ratio_odom_over_truth"] for item in turns]
    left = summarize([item["ratio_odom_over_truth"] for item in turns if item["command"]["angular_rad_s"] > 0])
    right = summarize([item["ratio_odom_over_truth"] for item in turns if item["command"]["angular_rad_s"] < 0])
    drive_stats = summarize([item["ratio_odom_over_truth"] for item in drives])

    separation = recommended_separation_multiplier(parameters["wheel_separation_multiplier"], turn_ratios)
    # 직진은 좌우 바퀴를 함께 쓰므로 두 반지름 보정의 평균을 기준으로 참고값을 낸다.
    current_radius = (parameters["left_wheel_radius_multiplier"] + parameters["right_wheel_radius_multiplier"]) / 2.0
    radius = None if drive_stats is None else current_radius * (1.0 / drive_stats["mean"])

    return {
        "robot_id": node.robot_id,
        "parameters": parameters,
        "turns": turns,
        "drives": drives,
        "turn_ratio": {"all": summarize(turn_ratios), "left": left, "right": right},
        "drive_ratio": drive_stats,
        "recommended": {
            "wheel_separation_multiplier": separation,
            "wheel_radius_multiplier_reference_left_right": radius,
        },
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
    arguments = parser.parse_args()

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
    threading.Thread(target=executor.spin, daemon=True).start()

    checks = []
    result = {}

    try:
        node.truth.start()
        result = run(node, arguments, checks)
        print_summary(result)

    except RuntimeError as error:
        check(checks, "측정 진행", False, str(error))

    finally:
        # 어떤 경우에도 정지 명령을 남기고, Gazebo 구독을 먼저 해제한다.
        node.publish(0.0, 0.0)
        node.truth.stop()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

    result["checks"] = checks
    (output / "result.json").write_text(json.dumps(result, indent=2, ensure_ascii=False, default=float) + "\n")

    failures = sum(1 for item in checks if not item["passed"])
    print(f"결과: {output / 'result.json'}", flush=True)
    print(f"구조 판정 실패 {failures}건", flush=True)
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
