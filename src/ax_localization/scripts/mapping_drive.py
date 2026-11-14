#!/usr/bin/env python3
"""B2 개발용 지도 작성 주행. 경유점을 odom 기준으로 따라가며 SLAM 입력을 모으고 끝나면 지도·pose graph를 저장한다.

격리된 개발 시뮬레이션 전용 시험 경로다. A4(최종 구동 차단)가 아직 없어 diff_drive_controller에 직접 명령하므로
운영 주행 경로가 아니며 Nav2·D1 주행을 대신하지 않는다.

실행 순서(같은 ROS_DOMAIN_ID·GZ_PARTITION)
  1. ros2 launch ax_control simulation.launch.py world:=planned/ax_warehouse.world robot_ids:=robot_2
  2. ros2 launch ax_perception sensors.launch.py robot_ids:=robot_2          (B1 scan_filtered)
  3. ros2 launch ax_localization mapping.launch.py robot_id:=robot_2          (기본 scan:=merged, 병합 노드 포함)
  4. python3 src/ax_localization/scripts/mapping_drive.py --world planned/ax_warehouse.world \
       --route src/ax_localization/config/routes/ax_warehouse_center.yaml [--scan merged|filtered|raw] [--gate-dir ...]
     --scan은 3의 SLAM 입력 종류를 기록하고 장애물 감시 입력을 고른다. merged(기본)·filtered는 장애물 감시에
     앞·뒤 B1 scan_filtered 원 스캔을 그대로 쓴다(병합 스캔으로 감시하지 않음). raw는 앞·뒤 원시 scan이다.
     (--check-only: 경로의 world·gz_world·robot_id 대조만 하고 ROS 연결 없이 끝낸다)

정지 조건: 스캔·odom·TF 만료, 차체 외곽 기준 장애물, 경유점 제한시간·진척 없음, 다른 cmd_vel 발행자, Ctrl+C.
정지는 명령(0 속도)과 odom 기준 정지 확인(odom 속도가 D-06 값 안에 0.5 s 유지)을 구분해 기록한다. 확인하지 못하면 정상 종료가 아니다.
odom 기준 정지 확인은 Gazebo 실제 자세로 확인한 정지가 아니다. 실제 자세는 recording.npz에서 따로 비교한다.

Gazebo 실제 자세는 검증 기록(recording.npz)에만 저장하고 주행 제어·SLAM에는 쓰지 않는다.
결과: log/b2_mapping/<KST 시각>/drive_result.json, recording.npz, slam_map.yaml/pgm, pose_graph.posegraph/.data
"""

import argparse
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import traceback
from zoneinfo import ZoneInfo

import numpy as np
import yaml

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
import tf2_ros

sys.path.insert(0, str(Path(__file__).resolve().parent))

from map_tools import describe_model, same_world_structure, se2, se2_params, transform_points, wrap_angle, yaw_from_quaternion  # noqa: E402
import rotation_align  # noqa: E402

# D-06 정지 판정 값을 odom 속도에 적용한다(odom 기준 정지 확인). 판정 소유는 A4이며 여기서는 시험 기록용으로 같은 값을 쓴다.
STOP_LINEAR_M_S = 0.01
STOP_ANGULAR_RAD_S = 0.02
STOP_HOLD_S = 0.5
STOP_CONFIRM_TIMEOUT_S = 8.0
# odom이 한 번도 오지 않을 때 odom 기준 정지 확인을 포기하기까지의 벽시계 대기(s)
ODOM_FIRST_WAIT_S = 5.0

# 장애물로 멈춘 뒤 길이 열리기를 기다리는 시간(시뮬레이션 s)
OBSTACLE_WAIT_S = 10.0

# 제어 주기(벽시계 s)와 SLAM 상태 기록 주기(시뮬레이션 기준 타이머)
CONTROL_PERIOD_S = 0.05
HEALTH_RECORD_PERIOD_S = 0.5

# SLAM 상태 기록 열 이름(recording.npz slam_health)
HEALTH_COLUMNS = ["sim_time_s", "tf_receive_age_boottime_s", "tf_stamp_lag_sim_s", "front_scan_age_sim_s",
                  "rear_scan_age_sim_s", "travel_since_slam_update_m", "heading_since_slam_update_rad"]

# 저장 전 SLAM 지도 갱신을 기다리는 시간(시뮬레이션 s). slam_mapping.yaml map_update_interval보다 길게 둔다.
MAP_UPDATE_WAIT_S = 4.0

# map_saver_cli 설정. 미관측 205는 p=0.196이므로 free 기준을 그보다 크게 두면 다시 읽을 때 빈 공간이 된다.
MAP_FREE_THRESH = 0.196
MAP_OCCUPIED_THRESH = 0.65
MAP_SAVE_TIMEOUT_S = 30.0


# 저장소 루트: 이 파일 위로 올라가며 src/ax_localization이 있는 폴더를 찾는다. 설치본에서 실행하면 현재 폴더.
def workspace_root():
    for parent in Path(__file__).resolve().parents:
        if (parent / "src/ax_localization").is_dir():
            return parent

    return Path.cwd()


WORKSPACE = workspace_root()
PACKAGE = WORKSPACE / "src/ax_localization"


# 파일의 SHA-256. 없으면 None
def sha256_of(path):
    path = Path(path)

    if not path.is_file():
        return None

    return hashlib.sha256(path.read_bytes()).hexdigest()


# 임시 파일에 쓴 뒤 바꿔 넣는다. 쓰는 도중 중단돼도 이전 파일이 깨지지 않는다.
def write_json_atomic(path, data):
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(data, indent=2, ensure_ascii=False, default=float) + "\n", encoding="utf-8")
    os.replace(temporary, path)


# 관문 신호 파일 경로(중단이 앞: 중단 우선)
def gate_signal_paths(gate_dir, token, name):
    gate_dir = Path(gate_dir)
    return (("abort", gate_dir / f"{token}.abort"), ("continue", gate_dir / f"{token}_{name}.continue"))


# 관문 진입 시점에 이미 있는 신호 파일의 식별값(inode, mtime_ns). 이 값과 같은 파일은 이전 신호(stale)다.
# 벽시계 시각 비교를 쓰지 않으므로 시스템 시계가 앞뒤로 바뀌어도(WSL 시간 동기화 등) 판정이 흔들리지 않는다.
def gate_signal_baseline(gate_dir, token, name):
    baseline = {}

    for _, path in gate_signal_paths(gate_dir, token, name):
        try:
            stat = path.stat()
        except FileNotFoundError:
            continue

        baseline[str(path)] = (stat.st_ino, stat.st_mtime_ns)

    return baseline


# 관측 관문 신호 파일을 읽는다. 중단(<token>.abort)이 진행(<token>_<name>.continue)보다 우선이다.
# 진입 시 기준(baseline)과 식별값이 같은 파일은 쓰지 않는다(stale). 새로 만들거나 다시 touch한 파일만 신호다.
# 돌려주는 값: ("abort" | "continue" | None, 파일 경로 또는 None)
def read_gate_signal(gate_dir, token, name, baseline):
    for kind, path in gate_signal_paths(gate_dir, token, name):
        try:
            stat = path.stat()
        except FileNotFoundError:
            continue

        if baseline.get(str(path)) != (stat.st_ino, stat.st_mtime_ns):
            return kind, path

    return None, None


# 진행 신호 파일을 한 번만 쓰도록 이름을 바꿔 소비한다(같은 파일로 다시 출발하지 않는다).
def consume_gate_signal(path):
    consumed = path.with_name(path.name + f".consumed_{time.time_ns()}")
    os.replace(path, consumed)
    return consumed


# 자료 하나의 시각·나이. 없으면 missing으로 표시한다(값을 지어 채우지 않는다).
def stamped(stamp, now, value):
    if stamp is None:
        return {"missing": True}

    return {"missing": False, "stamp_s": stamp, "age_s": now - stamp, "value": value}


# 정지 시점 기록을 만든다(관문·실패 공통). 입력은 읽어 둔 값이며 Gazebo 실제 자세는 평가 전용으로만 담는다.
# odom 이동량 대비 실제 이동량이 아주 작으면 물리 걸림 "의심"으로만 표시한다(판정 아님).
def build_snapshot(kind, name, reason, now, wall, odom, scans, map_odom, truth, command, checks, motion):
    snapshot = {
        "kind": kind, "name": name, "reason": reason, "sim_time_s": now, "wall_time_s": wall,
        "odom": stamped(odom[0] if odom else None, now, {"pose": list(odom[1:4]), "velocity": list(odom[4:6])}
                        if odom else None),
        "scans": {key: stamped(item["stamp"] if item else None, now, item["data"] if item else None)
                  for key, item in scans.items()},
        "map_to_odom": stamped(map_odom[0] if map_odom else None, now, list(map_odom[1:4]) if map_odom else None),
        "map_pose": None,
        "truth_world_eval_only": stamped(truth[0] if truth else None, now, list(truth[1:4]) if truth else None),
        "last_command": command,
        "checks": checks,
        "motion": motion,
        "note": "truth는 Gazebo 실제 자세(평가 전용). 주행·지도 입력이 아니다. 물리 걸림은 의심 표시일 뿐이다",
    }

    if odom and map_odom:
        snapshot["map_pose"] = list(se2_params(se2(*map_odom[1:4]) @ se2(*odom[1:4])))

    missing = [key for key in ("odom", "map_to_odom", "truth_world_eval_only") if snapshot[key]["missing"]]
    missing += [f"scan_{key}" for key, item in snapshot["scans"].items() if item["missing"]]
    snapshot["missing"] = missing
    snapshot["complete"] = not snapshot["missing"]

    return snapshot


# 최근 window_s 동안 odom·실제 자세 이동 거리(경로 길이). 표본이 부족하면 None.
def recent_motion(odom_record, truth_array, now, window_s=3.0):
    def path_length(rows):
        rows = [row for row in rows if now - window_s <= row[0] <= now]

        if len(rows) < 2:
            return None

        xy = np.array([row[1:3] for row in rows], float)
        return float(np.sum(np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1]))))

    odom_path = path_length(odom_record)
    truth_path = path_length(truth_array.tolist() if len(truth_array) else [])
    suspected = bool(odom_path is not None and truth_path is not None
                     and odom_path > 0.05 and truth_path < 0.2 * odom_path)

    return {"window_s": window_s, "odom_path_m": odom_path, "truth_path_m_eval_only": truth_path,
            "physical_stuck_suspected": suspected,
            "rule": "odom 이동 > 0.05 m이고 실제 이동 < odom의 20 %면 의심(확정 아님)"}


# 경로 YAML을 읽고 필수 항목을 검사한다.
def load_route(path):
    route = yaml.safe_load(Path(path).read_text(encoding="utf-8"))

    for key in ("robot_id", "frame", "limits", "footprint", "safety", "inputs", "progress", "waypoints"):
        if key not in route:
            raise ValueError(f"경로 파일에 {key} 없음: {path}")

    if not route["frame"].endswith("/odom"):
        raise ValueError("최초 구현은 odom 기준 경유점만 지원")

    if len(route["waypoints"]) < 2:
        raise ValueError("경유점은 2개 이상")

    # 경유점은 [x, y](전진) 또는 {xy: [x, y], reverse: true}(후진)다. (x, y, 후진 여부)로 통일한다.
    # dict 경유점의 선택 표시 gate: true는 관측 관문이다(도달 뒤 정지·기록·진행 신호 대기, --gate-dir가 있을 때만).
    # 관문 이름은 나타난 순서대로 P1, P2, …다(프로젝트 안전 관문 G1~G6과 구분).
    waypoints = []
    gates = {}
    alignment = {}

    for index, item in enumerate(route["waypoints"]):
        if isinstance(item, dict):
            waypoints.append((float(item["xy"][0]), float(item["xy"][1]), bool(item.get("reverse", False))))
            gate = item.get("gate", False)

            if not isinstance(gate, bool):
                raise ValueError(f"경유점 {index + 1} gate는 true/false만 가능")

            if gate:
                gates[index] = f"P{len(gates) + 1}"

            # 선택 align: 이 경유점에서 멈춘 뒤 odom 자세가 기준선(경유점을 지나 heading_rad 방향)에
            # 측면·방향 허용값 안에 있어야 다음 구간으로 출발한다(좁은 통로 입구 앞 정렬).
            if "align" in item:
                alignment[index] = parse_alignment(item["align"], index)
        else:
            waypoints.append((float(item[0]), float(item[1]), False))

    route["waypoints"] = waypoints
    route["gates"] = gates
    route["alignment"] = alignment
    # 선택 모드: 회전과 직진을 분리하고 정지 뒤 방향을 다시 확인한다(rotation_align.py, C 조건 시험 근거).
    # 키가 없으면 None이며 기존 waypoint_command 주행을 그대로 쓴다.
    route["rotation_align"] = (rotation_align.parse(route["rotation_align"], route["limits"]["angular_rad_s"])
                               if "rotation_align" in route else None)

    return route


# 경로 파일과 실행 조건(월드 파일·Gazebo 내부 이름·로봇)의 불일치 목록. 비어 있으면 일치다.
# 월드 파일 경로(world, 예: planned/ax_warehouse.world)와 Gazebo 내부 이름(gz_world, 예: ax_test)은 다른 값이다.
# 삭제된 AWS 월드 경로는 어떤 조건에서도 거절한다. world·gz_world·robot_id 인자가 None이면 그 항목은 비교하지 않는다.
def route_world_problems(route, world=None, gz_world=None, robot_id=None):
    problems = []
    route_world = route.get("world")

    if route_world == "aws_warehouse.world":
        problems.append("AWS 월드는 삭제됐습니다. 현재 두 창고 월드에 맞춰 새로 검증한 --route를 지정하세요")

    # 재질만 다른 월드(같은 structure, config/worlds.yaml)는 같은 경로를 쓴다.
    if world is not None and not same_world_structure(route_world, world):
        problems.append(f"경로 world {route_world!r} ≠ 실행 월드 {world!r}(구조가 다른 월드)")

    if gz_world is not None and "gz_world" in route and route["gz_world"] != gz_world:
        problems.append(f"경로 gz_world {route['gz_world']!r} ≠ Gazebo 내부 이름 {gz_world!r}")

    if robot_id is not None and route["robot_id"] != robot_id:
        problems.append(f"경로 robot_id {route['robot_id']!r} ≠ 실행 로봇 {robot_id!r}")

    if not route["frame"].startswith(route["robot_id"] + "/"):
        problems.append(f"경로 frame {route['frame']!r}이 robot_id {route['robot_id']!r}의 odom이 아님")

    return problems


# 경유점 align 항목을 검사해 {heading_rad, max_lateral_m, max_heading_rad}로 돌려준다. 값은 모두 유한해야 하고
# 허용값은 양수다. 틀리면 ValueError.
def parse_alignment(item, index):
    if not isinstance(item, dict) or set(item) != {"heading_rad", "max_lateral_m", "max_heading_rad"}:
        raise ValueError(f"경유점 {index + 1} align은 heading_rad·max_lateral_m·max_heading_rad 세 값만 가능")

    values = {key: float(value) for key, value in item.items()}

    if not all(math.isfinite(value) for value in values.values()):
        raise ValueError(f"경유점 {index + 1} align 값은 유한해야 함")

    if values["max_lateral_m"] <= 0.0 or values["max_heading_rad"] <= 0.0:
        raise ValueError(f"경유점 {index + 1} align 허용값은 양수")

    return values


# odom 자세(x, y, yaw)의 정렬 오차. 기준선은 경유점 goal을 지나 align["heading_rad"] 방향이다.
# along은 기준선 방향 위치, lateral은 왼쪽 +, heading_error는 yaw − heading_rad(감김 정리).
# 비유한 자세는 정렬을 확인할 수 없으므로 within=False다.
def alignment_error(pose, goal, align):
    if not all(math.isfinite(value) for value in pose):
        return {"along_m": None, "lateral_m": None, "heading_error_rad": None, "within": False,
                "reason": "odom 비유한 값"}

    heading = align["heading_rad"]
    dx, dy = pose[0] - goal[0], pose[1] - goal[1]
    along = math.cos(heading) * dx + math.sin(heading) * dy
    lateral = -math.sin(heading) * dx + math.cos(heading) * dy
    heading_error = wrap_angle(pose[2] - heading)
    within = abs(lateral) <= align["max_lateral_m"] and abs(heading_error) <= align["max_heading_rad"]

    return {"along_m": along, "lateral_m": lateral, "heading_error_rad": heading_error, "within": within,
            "reason": None if within else (f"정렬 허용 초과(측면 {lateral:+.3f} m/{align['max_lateral_m']} m, "
                                           f"방향 {heading_error:+.4f} rad/{align['max_heading_rad']} rad)")}


# SLAM 상태 문제 목록. 스캔 최신성은 스캔 검사에서 따로 본다.
# - 변환 수신: 이 로봇의 map→odom이 계속 들어오는지(SLAM 노드 생존). 변환 시각은 처리한 스캔 시각이라 늦을 수 있어 쓰지 않는다.
# - 처리 진척: 마지막 변환 시각 갱신 뒤 odom이 기준보다 많이 움직였는데 갱신이 없으면 SLAM이 멈춘 것으로 본다.
def slam_problems(health, inputs):
    problems = []
    age = health["tf_receive_age_boottime_s"]

    if age is None:
        problems.append("map→odom 미수신")
    elif age > inputs["tf_receive_max_age_boottime_s"]:
        problems.append(f"map→odom 수신 끊김 {age:.2f} s(CLOCK_BOOTTIME)")

    travel, heading = health["travel_since_slam_update_m"], health["heading_since_slam_update_rad"]

    if travel is not None and (travel > inputs["slam_progress_distance_m"] or heading > inputs["slam_progress_heading_rad"]):
        problems.append(f"SLAM 처리 진척 없음(마지막 갱신 뒤 {travel:.2f} m, {heading:.2f} rad)")

    return problems


# 한 로봇의 map→odom 변환만 추적하는 SLAM 상태 감시.
# - 수신 경과: CLOCK_BOOTTIME 기준. 시뮬레이션이 멈춰도 늘어나므로 SLAM 노드·전달 경로가 끊긴 것을 잡는다.
# - 변환 시각 지연: sim time 기준(SLAM이 마지막으로 처리한 스캔 시각까지의 차이). 판정에 쓰지 않고 기록만 한다.
# - 처리 진척: 변환 시각이 앞으로 간 순간의 odom을 기준점으로 두고 그 뒤 이동·회전량을 본다.
# 다른 로봇·다른 변환은 무시하고, 같은 시각의 오래된 변환이 반복되면 수신으로만 세고 진척으로 세지 않는다.
class SlamMonitor:
    # 추적할 자식 프레임(예: robot_2/odom)과 벽시계 함수(시험에서 바꿔 끼움)
    def __init__(self, child_frame, wall_clock=None):
        self.child = child_frame
        self.wall = wall_clock or (lambda: time.clock_gettime(time.CLOCK_BOOTTIME))
        self.received = None
        self.stamp = None
        self.anchor = None
        self.receipts = 0
        self.updates = 0
        self.gaps = []

    # 변환 하나를 받는다. 이 로봇의 map→odom이고 변환 시각이 앞으로 갔으면 True
    def on_transform(self, frame_id, child_frame, stamp, odom_pose):
        if frame_id != "map" or child_frame != self.child:
            return False

        now = self.wall()

        if self.received is not None:
            self.gaps.append(now - self.received)

        self.received = now
        self.receipts += 1

        if self.stamp is not None and stamp <= self.stamp:
            return False

        self.stamp = stamp
        self.anchor = odom_pose
        self.updates += 1

        return True

    # 수신 경과(CLOCK_BOOTTIME s), 변환 시각 지연(sim s), 기준점 뒤 이동·회전량. 값이 없으면 None
    def health(self, sim_now, odom_pose):
        travel = heading = None

        if self.anchor is not None and odom_pose is not None:
            travel = math.hypot(odom_pose[0] - self.anchor[0], odom_pose[1] - self.anchor[1])
            heading = abs(wrap_angle(odom_pose[2] - self.anchor[2]))

        return {
            "tf_receive_age_boottime_s": self.wall() - self.received if self.received is not None else None,
            "tf_stamp_lag_sim_s": sim_now - self.stamp if self.stamp is not None else None,
            "travel_since_slam_update_m": travel,
            "heading_since_slam_update_rad": heading,
        }


# 차체 외곽 앞쪽 통로 안에서 정지 거리 안에 있는 장애물 점을 센다.
# points는 base_link 기준(N×2), direction은 +1(전진) 또는 -1(후진)이다.
def corridor_blocked(points, footprint, stop_distance, margin, direction):
    if len(points) == 0:
        return 0

    edge = footprint["front_m"] if direction > 0 else footprint["rear_m"]
    ahead = points[:, 0] * direction
    lateral = np.abs(points[:, 1])

    blocked = (ahead >= edge - margin) & (ahead <= edge + stop_distance) & (lateral <= footprint["half_width_m"] + margin)

    return int(blocked.sum())


# 제자리 회전 시 외곽 원(차체 외접원 + 여유) 안의 장애물 점을 센다.
def rotation_blocked(points, footprint, margin):
    if len(points) == 0:
        return 0

    radius = math.hypot(max(footprint["front_m"], footprint["rear_m"]), footprint["half_width_m"]) + margin

    return int((np.hypot(points[:, 0], points[:, 1]) <= radius).sum())


# 정지 거리 = 속도²/(2·감속) + 속도·처리 지연 + 여유
def stop_distance(speed, deceleration, latency, margin):
    speed = abs(speed)

    return speed * speed / (2.0 * max(deceleration, 1e-3)) + speed * latency + margin


# 경유점 하나에 대한 명령(선속도, 각속도)과 단계를 정한다. 제자리 회전 → 직진 순서다.
# goal의 세 번째 값이 참이면 차체 뒤쪽을 경유점으로 향하게 돌린 뒤 후진한다(후방 스캔으로 감시).
def waypoint_command(pose, goal, limits, heading_tolerance):
    x, y, yaw = pose
    dx, dy = goal[0] - x, goal[1] - y
    reverse = len(goal) > 2 and goal[2]
    distance = math.hypot(dx, dy)
    target = math.atan2(dy, dx) + (math.pi if reverse else 0.0)
    error = wrap_angle(target - yaw)

    if abs(error) > heading_tolerance:
        angular = math.copysign(min(limits["angular"], max(0.1, 1.5 * abs(error))), error)

        return 0.0, angular, "rotate", distance, error

    linear = min(limits["linear"], 0.6 * distance + 0.05) * (-1.0 if reverse else 1.0)
    angular = max(-limits["angular"], min(limits["angular"], 2.0 * error))

    return linear, angular, "reverse" if reverse else "drive", distance, error


# Gazebo 실제 차체 자세 기록기. 검증용 기록에만 쓰며 주행 제어에 전달하지 않는다.
class GazeboTruthRecorder:
    # 기록할 모델과 Gazebo 월드 이름
    def __init__(self, model, world):
        self.model = model
        self.topic = f"/world/{world}/dynamic_pose/info"
        self.lock = threading.Lock()
        self.samples = []
        self.node = None

    # 자세 토픽을 구독한다.
    def start(self):
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node as GzNode

        self.node = GzNode()

        if not self.node.subscribe(Pose_V, self.topic, self.on_pose):
            raise RuntimeError(f"Gazebo 자세 구독 실패: {self.topic}")

    # 대상 모델의 (시뮬레이션 시각, x, y, yaw)를 저장한다.
    def on_pose(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nsec * 1e-9

        for pose in message.pose:
            if pose.name != self.model:
                continue

            q = pose.orientation

            with self.lock:
                self.samples.append((stamp, pose.position.x, pose.position.y, yaw_from_quaternion(q.x, q.y, q.z, q.w)))

    # 기록 사본(N×4)
    def array(self):
        with self.lock:
            return np.array(self.samples, float).reshape(-1, 4)

    # 구독 해제. 프로세스 종료 전에 부른다.
    def stop(self):
        if self.node is not None:
            self.node.unsubscribe(self.topic)
            self.node = None


# 지도 작성 주행 노드. 콜백은 별도 executor 스레드가 처리하고 제어는 main 스레드가 한다.
class MappingDriver(Node):
    # 구독·발행·기록 버퍼를 만든다.
    def __init__(self, robot_id, scan_suffix):
        super().__init__("ax_mapping_drive", parameter_overrides=[Parameter("use_sim_time", value=True)])

        self.robot_id = robot_id
        self.namespace = f"/ax/{robot_id}"
        self.lock = threading.Lock()

        self.command_topic = f"{self.namespace}/diff_drive_controller/cmd_vel"
        self.command = self.create_publisher(TwistStamped, self.command_topic, 10)

        self.scans = {"front": None, "rear": None}
        self.front_record = []
        self.odom = None
        self.odom_record = []
        self.map_record = []

        self.create_subscription(LaserScan, f"{self.namespace}/sensors/front_lidar/{scan_suffix}",
                                 lambda message: self.on_scan("front", message), qos_profile_sensor_data)
        self.create_subscription(LaserScan, f"{self.namespace}/sensors/rear_lidar/{scan_suffix}",
                                 lambda message: self.on_scan("rear", message), qos_profile_sensor_data)
        self.create_subscription(Odometry, f"{self.namespace}/diff_drive_controller/odom", self.on_odom, 20)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # SLAM 상태는 이 로봇의 map→odom 변환 하나만 보고 판단한다(SlamMonitor).
        self.slam = SlamMonitor(f"{robot_id}/odom")
        self.health_record = []
        self.create_subscription(TFMessage, "/tf", self.on_tf, 100)

        # 시뮬레이션이 멈춰도 기록하도록 벽시계 타이머를 쓴다.
        self.create_timer(HEALTH_RECORD_PERIOD_S, self.record_health, clock=Clock(clock_type=ClockType.STEADY_TIME))

        self.drive_parameters = AsyncParameterClient(self, f"{self.namespace}/diff_drive_controller")
        self.mounts = {}

        # Gazebo spawn에 쓰인 robot_description(latched). 모델·마찰·센서 설정 기록용이다.
        self.robot_description = None
        self.create_subscription(String, f"{self.namespace}/robot_description", self.on_robot_description,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    # robot_description 문자열을 보관한다.
    def on_robot_description(self, message):
        with self.lock:
            self.robot_description = message.data

    # 현재 시뮬레이션 시각(s)
    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # 최신 스캔을 저장하고, 앞 스캔은 참조 지도용으로 기록한다.
    def on_scan(self, name, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9

        with self.lock:
            self.scans[name] = (stamp, message)

            if name == "front":
                self.front_record.append((stamp, np.asarray(message.ranges, np.float32)))
                self.front_layout = (message.angle_min, message.angle_increment, message.range_min,
                                     message.range_max, message.header.frame_id, len(message.ranges))

    # odom 자세·속도를 저장한다.
    def on_odom(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        pose = message.pose.pose
        q = pose.orientation
        twist = message.twist.twist
        sample = (stamp, pose.position.x, pose.position.y, yaw_from_quaternion(q.x, q.y, q.z, q.w),
                  twist.linear.x, twist.angular.z)

        with self.lock:
            self.odom = sample
            self.odom_record.append(sample)

    # /tf에서 이 로봇의 map→odom만 골라 수신 시각·변환 시각을 갱신한다.
    # 변환 시각이 앞으로 갈 때만 SLAM 처리 진척으로 보고 그 순간의 odom을 기준점으로 둔다.
    def on_tf(self, message):
        for transform in message.transforms:
            t = transform.transform
            stamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9

            with self.lock:
                odom_pose = self.odom[1:4] if self.odom else None

                if self.slam.on_transform(transform.header.frame_id, transform.child_frame_id, stamp, odom_pose):
                    self.map_record.append((stamp, t.translation.x, t.translation.y,
                                            yaw_from_quaternion(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)))

    # SLAM 상태 세 항목(변환 수신 경과, 입력 스캔 최신성, 처리 진척)을 따로 계산한다. 값이 없으면 None.
    def slam_health(self):
        now = self.now_seconds()

        with self.lock:
            scans = {name: item[0] if item else None for name, item in self.scans.items()}
            health = self.slam.health(now, self.odom[1:4] if self.odom else None)

        return dict(health, sim_time_s=now,
                    front_scan_age_sim_s=now - scans["front"] if scans["front"] is not None else None,
                    rear_scan_age_sim_s=now - scans["rear"] if scans["rear"] is not None else None)

    # SLAM 상태를 주기적으로 기록한다.
    def record_health(self):
        health = self.slam_health()
        row = [health[key] if health[key] is not None else math.nan for key in HEALTH_COLUMNS]

        with self.lock:
            self.health_record.append(row)

    # 0 속도 명령을 보낸다.
    def publish_stop(self):
        self.publish(0.0, 0.0)

    # 속도 명령을 보낸다.
    def publish(self, linear, angular):
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = f"{self.robot_id}/base_link"
        message.twist.linear.x = float(linear)
        message.twist.angular.z = float(angular)
        self.command.publish(message)

    # 같은 cmd_vel에 이 노드 말고 다른 발행자가 있는지
    def other_publishers(self):
        own = (self.get_name(), self.get_namespace())

        return [
            f"{info.node_namespace.rstrip('/')}/{info.node_name}"
            for info in self.get_publishers_info_by_topic(self.command_topic)
            if (info.node_name, info.node_namespace) != own
        ]

    # base_link → LiDAR 고정 변환(x, y, yaw)을 TF에서 읽는다.
    def lookup_mount(self, name, timeout_s=10.0):
        frame = f"{self.robot_id}/{name}_lidar_link"
        deadline = time.monotonic() + timeout_s

        while time.monotonic() < deadline:
            try:
                transform = self.tf_buffer.lookup_transform(f"{self.robot_id}/base_link", frame, Time())
                t = transform.transform

                return (t.translation.x, t.translation.y,
                        yaw_from_quaternion(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w))
            except tf2_ros.TransformException:
                time.sleep(0.1)

        raise RuntimeError(f"TF 없음: {self.robot_id}/base_link → {frame}")

    # 실행 중인 diff_drive_controller의 속도·감속 제한을 읽는다. 없으면 빈 값.
    def read_drive_limits(self, timeout_s=10.0):
        names = ["linear.x.max_velocity", "angular.z.max_velocity",
                 "linear.x.max_deceleration", "angular.z.max_deceleration"]

        if not self.drive_parameters.wait_for_services(timeout_sec=timeout_s):
            raise RuntimeError("diff_drive_controller 파라미터 서비스 없음")

        future = self.drive_parameters.get_parameters(names)
        deadline = time.monotonic() + timeout_s

        while not future.done():
            if time.monotonic() > deadline:
                raise RuntimeError("diff_drive_controller 파라미터 응답 없음")

            time.sleep(0.02)

        values = {}

        for name, value in zip(names, future.result().values):
            number = value.double_value if value.type == 3 else None
            values[name] = abs(number) if number is not None and math.isfinite(number) and number != 0 else None

        return values

    # 최신 스캔을 base_link 기준 장애물 점(N×2)으로 바꾼다. (점, 측정 시각) 또는 (None, None)
    def obstacle_points(self, name):
        with self.lock:
            item = self.scans[name]

        if item is None or name not in self.mounts:
            return None, None

        stamp, message = item
        ranges = np.asarray(message.ranges, float)
        angles = message.angle_min + message.angle_increment * np.arange(len(ranges))
        valid = np.isfinite(ranges) & (ranges >= message.range_min) & (ranges <= message.range_max)

        # B1이 너무 가까움(-inf)으로 표시한 빔은 거리를 알 수 없지만 가까운 장애물이므로 최소 거리에 둔다.
        close = ranges == -np.inf
        distances = np.where(close, message.range_min, ranges)
        use = valid | close

        local = np.stack([distances[use] * np.cos(angles[use]), distances[use] * np.sin(angles[use])], axis=1)

        return transform_points(se2(*self.mounts[name]), local), stamp


# 실행 상태와 기록. drive_result.json으로 저장한다.
class Run:
    # 결과 폴더와 기록 뼈대
    def __init__(self, output, route_path, route, scan_suffix, slam_scan_input=None):
        self.output = output
        self.record = {
            "status": "running",
            "error": None,
            "development_test_path": "A4 미구현 상태에서 diff_drive_controller에 직접 명령하는 격리 개발 시험 경로",
            "robot_id": route["robot_id"],
            "route": {"path": str(route_path), "sha256": sha256_of(route_path), "frame": route["frame"],
                      "waypoints": route["waypoints"], "partial": route.get("partial_route", False)},
            # scan_input: 장애물 감시·기록(recording.npz 앞 스캔)에 쓰는 앞·뒤 원 스캔 접미사.
            # slam_scan_input: SLAM 입력 종류(merged = 앞·뒤 B1 scan_filtered 병합, filtered = 앞만, raw = 앞 원시).
            "scan_input": scan_suffix,
            "slam_scan_input": slam_scan_input,
            "limits": {},
            "waypoints_reached": 0,
            "events": [],
            "stops": [],
            "loop": {},
            "saved": {},
            # 저장 직전·종료 시 odom 기준 정지 확인 결과. 둘 다 확인돼야 지도 저장·채택·정상 완료가 가능하다.
            "stop_gate": {"save": None, "completion": None},
            "saved_map_usable": None,
        }

    # 사건 하나를 기록하고 출력한다.
    def event(self, sim_time, kind, detail):
        self.record["events"].append({"sim_time_s": sim_time, "kind": kind, "detail": detail})
        print(f"[{kind}] {detail}", flush=True)

    # 중간 저장
    def save(self):
        write_json_atomic(self.output / "drive_result.json", self.record)


# 관측 관문 대기 상한(벽시계). 넘으면 정지 상태로 실패 처리한다(입력 보호는 대기 중에도 계속).
GATE_MAX_WAIT_WALL_S = 1800.0
# 재출발 전 sim time이 이 벽시계 시간 안에 진행했어야 한다(일시정지·시계 정체 중에는 출발하지 않음).
GATE_CLOCK_FRESH_WALL_S = 2.0


# 관문 대기·재출발용 odom 정지 판정. stamp·자세·선/각속도가 모두 유한하고 속도가 D-06 값 안이어야 정지다.
# 비유한 값(NaN·inf)이나 미수신은 정지를 확인할 수 없는 것으로 본다(NaN 비교가 거짓이 되어 통과하는 것을 막는다).
# 돌려주는 값: (정지 확인 여부, 사유 또는 None)
def gate_odom_stopped(odom):
    if odom is None:
        return False, "odom 없음"

    if not all(math.isfinite(value) for value in odom):
        return False, "odom 비유한 값(stamp·자세·속도 중)"

    if abs(odom[4]) >= STOP_LINEAR_M_S or abs(odom[5]) >= STOP_ANGULAR_RAD_S:
        return False, f"정지 아님(odom {odom[4]:.3f} m/s, {odom[5]:.3f} rad/s)"

    return True, None


# 비유한 값은 문자열로 남겨 JSON에서 의미를 잃지 않게 한다.
def finite_or_text(value):
    return float(value) if math.isfinite(value) else str(value)


# 경로를 따라 주행한다. 중단 사유가 생기면 RuntimeError를 낸다.
class RouteFollower:
    # 노드·경로·실행 기록·종료 신호 플래그를 받는다. gate_dir가 있으면 경로의 관측 관문에서 멈춰 기다린다.
    def __init__(self, node, route, run, stop_flag, gate_dir=None, gate_token=None, truth=None):
        self.node = node
        self.route = route
        self.run = run
        self.stop_flag = stop_flag
        self.gate_dir = Path(gate_dir) if gate_dir else None
        self.gate_token = gate_token
        self.truth = truth
        self.last_command = None
        self.snapshot_count = 0
        self.final_outcomes = {}    # 경유점 index → 최종 방향 정렬 기록(final_align). 거부·실패는 재출발까지 유지

        self.limits = {}
        self.deceleration = {}

    # 경로 상한과 제어기 제한 중 작은 값을 쓴다.
    def configure_limits(self, controller):
        route_limits = self.route["limits"]

        self.limits = {
            "linear": min(v for v in (route_limits["linear_m_s"], controller.get("linear.x.max_velocity")) if v),
            "angular": min(v for v in (route_limits["angular_rad_s"], controller.get("angular.z.max_velocity")) if v),
        }

        # 감속 제한이 없으면 보수적으로 0.5 m/s², 0.5 rad/s²를 정지 거리 계산에 쓴다.
        self.deceleration = {
            "linear": controller.get("linear.x.max_deceleration") or 0.5,
            "angular": controller.get("angular.z.max_deceleration") or 0.5,
        }

        self.run.record["limits"] = {"used": self.limits, "deceleration": self.deceleration,
                                     "controller": controller, "route": route_limits}

    # 입력(스캔·odom·TF) 신선도를 검사해 문제 목록을 돌려준다.
    def input_problems(self):
        inputs = self.route["inputs"]
        now = self.node.now_seconds()
        problems = []

        with self.node.lock:
            scans = dict(self.node.scans)
            odom = self.node.odom

        health = self.node.slam_health()

        for name, item in scans.items():
            if item is None:
                problems.append(f"{name} 스캔 미수신")
            elif now - item[0] > inputs["scan_max_age_s"]:
                problems.append(f"{name} 스캔 만료 {now - item[0]:.2f} s")

        if odom is None:
            problems.append("odom 미수신")
        elif now - odom[0] > inputs["odom_max_age_s"]:
            problems.append(f"odom 만료 {now - odom[0]:.2f} s")

        return problems + slam_problems(health, inputs)

    # 명령 방향에 맞춰 장애물을 검사한다. 막혔으면 사유 문자열, 아니면 None.
    def obstacle_problem(self, linear, angular):
        footprint = self.route["footprint"]
        safety = self.route["safety"]

        with self.node.lock:
            odom = self.node.odom

        speed = max(abs(linear), abs(odom[4]) if odom else 0.0)

        if linear > 0.0 or linear < 0.0:
            name = "front" if linear > 0.0 else "rear"
            points, _ = self.node.obstacle_points(name)
            distance = stop_distance(speed, self.deceleration["linear"], safety["processing_latency_s"], safety["margin_m"])
            count = corridor_blocked(points, footprint, distance, safety["margin_m"], 1 if linear > 0 else -1)

            if count:
                return f"{'전방' if linear > 0 else '후방'} 장애물 {count}점(정지 거리 {distance:.2f} m)"

        if angular != 0.0 and linear == 0.0:
            count = 0

            for name in ("front", "rear"):
                points, _ = self.node.obstacle_points(name)
                count += rotation_blocked(points, footprint, safety["rotation_margin_m"])

            if count:
                return f"회전 반경 안 장애물 {count}점"

        return None

    # odom 기준 정지 확인: 정지 명령 뒤 odom 속도가 D-06 값 안에 STOP_HOLD_S 동안 머무는지 확인한다.
    # Gazebo 실제 자세는 보지 않는다.
    def stop_and_confirm(self, reason):
        start = self.node.now_seconds()
        wall_deadline = time.monotonic() + 60.0
        held_since = None
        confirmed = False

        wall_start = time.monotonic()

        while self.node.now_seconds() - start < STOP_CONFIRM_TIMEOUT_S and time.monotonic() < wall_deadline:
            self.node.publish_stop()

            with self.node.lock:
                odom = self.node.odom

            # odom이 한 번도 오지 않으면 정지를 확인할 수단이 없으므로 오래 기다리지 않는다.
            if odom is None and time.monotonic() - wall_start > ODOM_FIRST_WAIT_S:
                break

            now = self.node.now_seconds()

            if odom is not None and abs(odom[4]) < STOP_LINEAR_M_S and abs(odom[5]) < STOP_ANGULAR_RAD_S:
                held_since = held_since if held_since is not None else now

                if now - held_since >= STOP_HOLD_S:
                    confirmed = True
                    break
            else:
                held_since = None

            time.sleep(CONTROL_PERIOD_S)

        entry = {"reason": reason, "requested_sim_s": start, "confirmed": confirmed,
                 "confirm_sim_s": self.node.now_seconds() - start if confirmed else None,
                 "odom_received": odom is not None}
        self.run.record["stops"].append(entry)
        self.run.event(start, "STOP", f"{reason}: odom 기준 정지 {'확인' if confirmed else '미확인'}")

        return confirmed

    # 입력 문제가 풀리기를 기다린다. 시간을 넘기면 중단한다.
    def wait_inputs(self, problems):
        self.stop_and_confirm("입력 문제: " + ", ".join(problems))
        start = self.node.now_seconds()
        wall_deadline = time.monotonic() + 120.0

        while self.node.now_seconds() - start < self.route["inputs"]["input_timeout_s"] and time.monotonic() < wall_deadline:
            self.check_interrupt()
            self.node.publish_stop()

            if not self.input_problems():
                self.run.event(self.node.now_seconds(), "RESUME", "입력 회복")
                return

            time.sleep(CONTROL_PERIOD_S)

        raise RuntimeError("입력이 회복되지 않음: " + ", ".join(self.input_problems()))

    # 장애물이 사라지기를 기다린다. 시간을 넘기면 중단한다.
    def wait_obstacle(self, problem, linear, angular):
        self.stop_and_confirm(problem)
        start = self.node.now_seconds()
        wall_deadline = time.monotonic() + 120.0

        while self.node.now_seconds() - start < OBSTACLE_WAIT_S and time.monotonic() < wall_deadline:
            self.check_interrupt()
            self.node.publish_stop()

            if self.obstacle_problem(linear, angular) is None:
                self.run.event(self.node.now_seconds(), "RESUME", "장애물 해소")
                return

            time.sleep(CONTROL_PERIOD_S)

        raise RuntimeError("장애물이 해소되지 않음: " + problem)

    # 종료 신호와 다른 발행자를 확인한다.
    def check_interrupt(self):
        if self.stop_flag.is_set():
            raise KeyboardInterrupt

        others = self.node.other_publishers()

        if others:
            raise RuntimeError("다른 cmd_vel 발행자와 동시 구동 불가: " + ", ".join(others))

    # 경유점으로 출발할 때 처음 낼 명령(선속도, 각속도). 재출발 검사(보호창·회전 원)에 쓴다.
    # 회전 정렬 모드에서는 방향 오차가 최종 허용을 넘으면 회전(선속도 0), 아니면 각속도 0 직진이다.
    def first_command(self, pose, goal):
        law = self.route.get("rotation_align")

        if not law:
            linear, angular, _, _, _ = waypoint_command(pose, goal, self.limits, self.route["heading_tolerance_rad"])
            return linear, angular

        reverse = len(goal) > 2 and goal[2]
        distance = math.hypot(goal[0] - pose[0], goal[1] - pose[1])
        error = wrap_angle(math.atan2(goal[1] - pose[1], goal[0] - pose[0]) + (math.pi if reverse else 0.0) - pose[2])

        if abs(error) > law["final_tolerance_rad"]:
            omega = rotation_align.omega_command(error, law, self.limits["angular"])
            return 0.0, omega if omega != 0.0 else math.copysign(law["omega_min_rad_s"], error)

        return min(self.limits["linear"], 0.6 * distance + 0.05) * (-1.0 if reverse else 1.0), 0.0

    # 명령 단계가 바뀔 때만 기록한다(직진·회전 명령 구간 추적).
    def note_phase(self, index, phase):
        phases = self.run.record.setdefault("command_phases", [])

        if not phases or phases[-1]["phase"] != phase or phases[-1]["waypoint"] != index + 1:
            phases.append({"sim_time_s": self.node.now_seconds(), "waypoint": index + 1, "phase": phase})

    # 회전 정렬 절차 하나(rotation_align.align). 회전 명령마다 중단·다른 발행자·입력·회전 원 검사를 거친다.
    # 결과를 run.record["rotation_align"]에 남기고, 실패하면 RuntimeError(직진하지 않음).
    def align_to(self, index, target, trigger):
        law = self.route["rotation_align"]
        follower = self

        class IO:
            def heading(self):
                with follower.node.lock:
                    return follower.node.odom[3]

            def now_sim(self):
                return follower.node.now_seconds()

            def now_wall(self):
                return time.monotonic()

            def rotate(self, angular):
                follower.check_interrupt()
                problems = follower.input_problems()

                if problems:
                    follower.wait_inputs(problems)

                problem = follower.obstacle_problem(0.0, angular)

                if problem:
                    follower.wait_obstacle(problem, 0.0, angular)

                follower.last_command = {"linear": 0.0, "angular": angular, "phase": "align_rotate", "waypoint": index + 1}
                follower.note_phase(index, "align_rotate")
                follower.node.publish(0.0, angular)

            def tick(self):
                time.sleep(CONTROL_PERIOD_S)

            def stop_confirm(self):
                follower.note_phase(index, "align_stop")

                if not follower.stop_and_confirm(f"회전 정렬 정지 확인(경유점 {index + 1})"):
                    raise rotation_align.AlignFailed("odom 기준 정지를 확인하지 못함")

        entry = {"waypoint": index + 1, "trigger": trigger, "target_rad": target, "start_sim_s": self.node.now_seconds()}
        records = self.run.record.setdefault("rotation_align", [])

        try:
            result = rotation_align.align(IO(), target, law, self.limits["angular"])
        except rotation_align.AlignFailed as error:
            entry.update({"aligned": False, "error": str(error)})
            records.append(entry)
            self.run.event(self.node.now_seconds(), "ALIGN", f"경유점 {index + 1} 회전 정렬 실패({trigger}): {error}")
            raise RuntimeError(f"경유점 {index + 1} 회전 정렬 실패({trigger}): {error}") from error

        entry.update(result)
        records.append(entry)
        self.run.event(self.node.now_seconds(), "ALIGN",
                       f"경유점 {index + 1} 회전 정렬({trigger}) 오차 {result['final_error_rad']:+.5f} rad, "
                       f"재정렬 {result['realigns']}회, {result['sim_s']:.1f} s")

    # 회전 정렬 모드의 경유점 주행: 정지 확인 → 제자리 회전 정렬 → 각속도 0 직진. 직진 중 방향 오차가
    # heading_tolerance를 넘고 남은 거리가 0.3 m보다 크면 정지 뒤 다시 정렬한다. 경유점마다 정렬 절차는
    # 1 + max_realign회까지(시작 1회 + 복구), 경유점 제한시간·진척 감시는 기존과 같다.
    def drive_to_aligned(self, index, goal):
        law = self.route["rotation_align"]
        progress = self.route["progress"]
        tolerance = self.route["goal_tolerance_m"]
        reverse = len(goal) > 2 and goal[2]

        with self.node.lock:
            odom = self.node.odom

        start_distance = math.hypot(goal[0] - odom[1], goal[1] - odom[2])
        limit = start_distance / self.limits["linear"] * progress["time_factor"] + progress["time_margin_s"]
        start = self.node.now_seconds()
        best = {"distance": start_distance, "time": start}
        procedures = 0
        need_check = True       # 경유점 시작·보호 정지·입력 회복 뒤에는 방향을 다시 확인한다

        self.run.event(start, "GOAL", f"경유점 {index + 1}/{len(self.route['waypoints'])} {goal} 거리 {start_distance:.2f} m, "
                                      f"제한 {limit:.0f} s(회전 정렬 모드)")

        while True:
            self.check_interrupt()
            now = self.node.now_seconds()
            problems = self.input_problems()

            if problems:
                self.wait_inputs(problems)
                best["time"] = self.node.now_seconds()
                need_check = True
                continue

            with self.node.lock:
                odom = self.node.odom

            if not all(math.isfinite(value) for value in odom[1:4]):
                raise RuntimeError(f"경유점 {index + 1}: odom 비유한 값 {odom}")

            x, y, yaw = odom[1:4]
            distance = math.hypot(goal[0] - x, goal[1] - y)

            if distance <= tolerance:
                self.run.record["waypoints_reached"] = index + 1
                self.run.event(now, "REACHED", f"경유점 {index + 1} 도달, 남은 거리 {distance:.3f} m")
                self.run.save()
                return

            if now - start > limit:
                raise RuntimeError(f"경유점 {index + 1} 제한시간 {limit:.0f} s 초과(남은 거리 {distance:.2f} m)")

            target = math.atan2(goal[1] - y, goal[0] - x) + (math.pi if reverse else 0.0)
            error = wrap_angle(target - yaw)
            trigger = None

            if need_check and abs(error) > law["final_tolerance_rad"]:
                trigger = "start" if procedures == 0 else "after_stop"
            elif abs(error) > self.route["heading_tolerance_rad"] and distance > 0.3:
                trigger = "heading_drift"

            need_check = False

            if trigger:
                procedures += 1

                if procedures > 1 + law["max_realign"]:
                    raise RuntimeError(f"경유점 {index + 1} 정렬 절차 {1 + law['max_realign']}회 초과({trigger})")

                # 감속 중 전진 속도가 남은 채 회전하지 않도록 먼저 정지를 확인한다.
                self.note_phase(index, "align_stop")

                if not self.stop_and_confirm(f"회전 정렬 전 정지(경유점 {index + 1}, {trigger})"):
                    raise RuntimeError(f"경유점 {index + 1}: 회전 정렬 전 odom 기준 정지를 확인하지 못함")

                with self.node.lock:
                    odom = self.node.odom

                target = math.atan2(goal[1] - odom[2], goal[0] - odom[1]) + (math.pi if reverse else 0.0)
                self.align_to(index, target, trigger)
                best["time"] = self.node.now_seconds()
                continue

            linear = min(self.limits["linear"], 0.6 * distance + 0.05) * (-1.0 if reverse else 1.0)

            if best["distance"] - distance >= progress["stall_min_improvement"]:
                best = {"distance": distance, "time": now}
            elif now - best["time"] > progress["stall_window_s"]:
                raise RuntimeError(f"경유점 {index + 1} 진척 없음 {progress['stall_window_s']} s(직진)")

            self.last_command = {"linear": linear, "angular": 0.0, "phase": "reverse" if reverse else "drive",
                                 "waypoint": index + 1}
            problem = self.obstacle_problem(linear, 0.0)

            if problem:
                self.wait_obstacle(problem, linear, 0.0)
                best["time"] = self.node.now_seconds()
                need_check = True
                continue

            self.note_phase(index, "reverse" if reverse else "straight")
            self.node.publish(linear, 0.0)
            time.sleep(CONTROL_PERIOD_S)

    # 경유점 하나까지 주행한다.
    def drive_to(self, index, goal):
        if self.route.get("rotation_align"):
            return self.drive_to_aligned(index, goal)

        progress = self.route["progress"]
        tolerance = self.route["goal_tolerance_m"]

        with self.node.lock:
            odom = self.node.odom

        start_distance = math.hypot(goal[0] - odom[1], goal[1] - odom[2])
        limit = start_distance / self.limits["linear"] * progress["time_factor"] + progress["time_margin_s"]
        start = self.node.now_seconds()
        best = {"distance": start_distance, "heading": math.inf, "time": start}

        self.run.event(start, "GOAL", f"경유점 {index + 1}/{len(self.route['waypoints'])} {goal} 거리 {start_distance:.2f} m, 제한 {limit:.0f} s")

        while True:
            self.check_interrupt()
            now = self.node.now_seconds()

            problems = self.input_problems()

            if problems:
                self.wait_inputs(problems)
                best["time"] = self.node.now_seconds()
                continue

            with self.node.lock:
                odom = self.node.odom

            linear, angular, phase, distance, heading = waypoint_command(
                odom[1:4], goal, self.limits, self.route["heading_tolerance_rad"])

            if distance <= tolerance:
                self.run.record["waypoints_reached"] = index + 1
                self.run.event(now, "REACHED", f"경유점 {index + 1} 도달, 남은 거리 {distance:.3f} m")
                self.run.save()
                return

            if now - start > limit:
                raise RuntimeError(f"경유점 {index + 1} 제한시간 {limit:.0f} s 초과(남은 거리 {distance:.2f} m)")

            # 진척: 거리 또는 방향 오차가 최소 개선량 이상 줄면 갱신한다.
            if (best["distance"] - distance >= progress["stall_min_improvement"]
                    or best["heading"] - abs(heading) >= progress["stall_min_improvement"]):
                best = {"distance": min(best["distance"], distance), "heading": min(best["heading"], abs(heading)),
                        "time": now}
            elif now - best["time"] > progress["stall_window_s"]:
                raise RuntimeError(f"경유점 {index + 1} 진척 없음 {progress['stall_window_s']} s(단계 {phase})")

            self.last_command = {"linear": linear, "angular": angular, "phase": phase, "waypoint": index + 1}
            problem = self.obstacle_problem(linear, angular)

            if problem:
                self.wait_obstacle(problem, linear, angular)
                best["time"] = self.node.now_seconds()
                continue

            self.node.publish(linear, angular)
            time.sleep(CONTROL_PERIOD_S)

    # 정지 시점 기록을 gate_dir에 쓴다(관문 대기·재출발 거부·실패). gate_dir가 없으면 쓰지 않는다.
    def write_snapshot(self, kind, name, reason):
        if self.gate_dir is None:
            return None

        now = self.node.now_seconds()

        with self.node.lock:
            odom = self.node.odom
            scans = dict(self.node.scans)
            map_odom = self.node.map_record[-1] if self.node.map_record else None
            odom_record = list(self.node.odom_record[-400:])

        scan_data = {}

        for key, item in scans.items():
            if item is None:
                scan_data[key] = None
                continue

            stamp, message = item
            scan_data[key] = {"stamp": stamp, "data": {
                "frame": message.header.frame_id, "angle_min": message.angle_min,
                "angle_increment": message.angle_increment, "range_min": message.range_min,
                "range_max": message.range_max, "ranges": [finite_or_text(v) for v in message.ranges]}}

        truth_array = self.truth.array() if self.truth is not None else np.zeros((0, 4))
        truth = tuple(truth_array[-1]) if len(truth_array) else None
        checks = {"slam_health": self.node.slam_health(), "input_problems": self.input_problems()}

        if self.last_command is not None and odom is not None:
            checks["last_command_obstacle_problem"] = self.obstacle_problem(self.last_command["linear"],
                                                                            self.last_command["angular"])

        snapshot = build_snapshot(kind, name, reason, now, time.time(), odom, scan_data, map_odom, truth,
                                  self.last_command, checks, recent_motion(odom_record, truth_array, now))
        self.snapshot_count += 1
        path = self.gate_dir / f"{self.gate_token}_{self.snapshot_count:03d}_{kind}{'_' + name if name else ''}.json"
        write_json_atomic(path, snapshot)
        self.run.record.setdefault("gate_snapshots", []).append(path.name)

        return path

    # 재출발을 막는 사유 목록. 입력 신선도, sim time 진행, odom 정지, 다음 명령의 외곽·회전 검사를 다시 본다.
    def restart_blockers(self, index, last_sim_change_wall):
        blockers = list(self.input_problems())

        if time.monotonic() - last_sim_change_wall > GATE_CLOCK_FRESH_WALL_S:
            blockers.append("sim time 정체(일시정지·시계 이상)")

        with self.node.lock:
            odom = self.node.odom

        stopped, reason = gate_odom_stopped(odom)

        if not stopped:
            blockers.append(reason)

            # 자세·속도를 믿을 수 없으면 다음 명령 검사도 하지 않는다(출발 거절).
            if odom is None or not all(math.isfinite(value) for value in odom):
                return blockers

        # 정렬 경유점이면 재출발 직전 odom 자세가 기준선 허용값 안이어야 한다.
        alignment = self.check_alignment(index, "재출발 전")

        if alignment is not None and not alignment["within"]:
            blockers.append(alignment["reason"])

        final = self.final_align_blocker(index)

        if final:
            blockers.append(final)

        if index + 1 < len(self.route["waypoints"]):
            linear, angular = self.first_command(odom[1:4], self.route["waypoints"][index + 1])
            phase = "rotate" if linear == 0.0 and angular != 0.0 else "drive"
            problem = self.obstacle_problem(linear, angular)

            if problem:
                blockers.append(f"다음 명령({phase}) 검사: {problem}")

        return blockers

    # 정렬 경유점의 방향 허용값. 회전 정렬 모드에서는 min(final_tolerance, max_heading)을 관문 도달·재출발·관문 없는
    # 출발 검사에 모두 쓴다(정렬 뒤 대기 중 방향 변화가 새 허용을 우회하지 못하게). 모드가 없으면 max_heading 그대로.
    def heading_limit(self, index):
        align = self.route["alignment"][index]
        law = self.route.get("rotation_align")

        return min(law["final_tolerance_rad"], align["max_heading_rad"]) if law else align["max_heading_rad"]

    # 최종 방향 정렬 결과가 출발을 막으면 사유, 아니면 None. aligned·already_within이 아닌 상태(refused·failed·미완료)는
    # 그 방문 동안 유지되며 입력 회복만으로 성공이 되지 않고 다시 회전하지도 않는다(새 실행 필요).
    def final_align_blocker(self, index):
        entry = self.final_outcomes.get(index)

        if entry is None or entry.get("outcome") in ("aligned", "already_within"):
            return None

        return (f"경유점 {index + 1} 최종 방향 정렬 {entry.get('outcome') or '미완료'}"
                f"({entry.get('reason')}): 재정렬하지 않음, 새 실행 필요")

    # 정렬 경유점(index)의 현재 odom 정렬 오차를 계산해 기록(run.record["alignment"])하고 돌려준다.
    # 정렬 항목이 없는 경유점이면 None. odom 미수신·비유한 값은 within=False다.
    def check_alignment(self, index, stage):
        align = self.route.get("alignment", {}).get(index)

        if align is None:
            return None

        with self.node.lock:
            odom = self.node.odom

        if odom is None:
            result = {"along_m": None, "lateral_m": None, "heading_error_rad": None, "within": False, "reason": "odom 없음"}
        else:
            result = alignment_error(odom[1:4], self.route["waypoints"][index],
                                     {**align, "max_heading_rad": self.heading_limit(index)})

        entry = {"waypoint": index + 1, "stage": stage, "sim_time_s": self.node.now_seconds(), "limits": align,
                 "heading_limit_rad": self.heading_limit(index),
                 "odom_pose": list(odom[1:4]) if odom else None, **result}
        self.run.record.setdefault("alignment", []).append(entry)
        self.run.event(entry["sim_time_s"], "ALIGN",
                       f"경유점 {index + 1} {stage} 정렬 {'통과' if result['within'] else '거부'}: "
                       f"측면 {result['lateral_m']}, 방향 {result['heading_error_rad']}")

        return result

    # 관측 관문: 기존 정지 확인이 성공해야 들어간다. 대기 중에는 0 속도를 계속 내고 중단·다른 발행자·
    # 입력 만료·시계 역행·정지 상실을 감시한다(콜백은 executor 스레드가 계속 처리한다).
    # 진행 신호는 관문마다 고유 파일을 한 번만 쓰며, 재출발 직전에 입력·정지·다음 명령 검사를 다시 한다.
    def wait_gate(self, index):
        name = self.route["gates"][index]

        if not self.stop_and_confirm(f"관측 관문 {name}"):
            self.write_snapshot("gate_refused", name, "odom 기준 정지 미확인")
            raise RuntimeError(f"관측 관문 {name}: odom 기준 정지를 확인하지 못해 관문에 들어가지 않음")

        baseline = gate_signal_baseline(self.gate_dir, self.gate_token, name)
        self.check_alignment(index, "관문 도달")
        self.write_snapshot("gate", name, "관문 도달·정지 확인")
        self.run.event(self.node.now_seconds(), "GATE",
                       f"관측 관문 {name} 대기: 진행 {self.gate_dir / (self.gate_token + '_' + name + '.continue')}, "
                       f"중단 {self.gate_dir / (self.gate_token + '.abort')}")
        self.run.save()
        wall_start = time.monotonic()
        last_sim = self.node.now_seconds()
        last_sim_change = time.monotonic()
        input_bad_since = None

        while True:
            self.check_interrupt()
            self.node.publish_stop()
            now = self.node.now_seconds()

            if now < last_sim - 1e-6:
                raise RuntimeError(f"관측 관문 {name} 대기 중 sim time 역행")

            if now > last_sim:
                last_sim, last_sim_change = now, time.monotonic()

            with self.node.lock:
                odom = self.node.odom

            stopped, reason = gate_odom_stopped(odom)

            # 미수신은 입력 감시(input_problems)가 다룬다. 값이 있는데 정지가 아니거나 비유한이면 실패다.
            if odom is not None and not stopped:
                self.write_snapshot("gate_stop_lost", name, reason)
                raise RuntimeError(f"관측 관문 {name} 대기 중 정지 상실·확인 불가: {reason}")

            problems = self.input_problems()

            if problems:
                input_bad_since = now if input_bad_since is None else input_bad_since

                if now - input_bad_since > self.route["inputs"]["input_timeout_s"]:
                    raise RuntimeError(f"관측 관문 {name} 대기 중 입력 미회복: " + ", ".join(problems))
            else:
                input_bad_since = None

            if time.monotonic() - wall_start > GATE_MAX_WAIT_WALL_S:
                raise RuntimeError(f"관측 관문 {name} 대기 {GATE_MAX_WAIT_WALL_S:.0f} s 초과")

            kind, path = read_gate_signal(self.gate_dir, self.gate_token, name, baseline)

            if kind == "abort":
                raise RuntimeError(f"관측 관문 {name} 중단 신호({path.name})")

            if kind == "continue":
                consume_gate_signal(path)
                blockers = self.restart_blockers(index, last_sim_change)

                if blockers:
                    self.write_snapshot("restart_blocked", name, "; ".join(blockers))
                    self.run.event(now, "GATE", f"관측 관문 {name} 재출발 거부: {'; '.join(blockers)}. 새 진행 파일 필요")
                    continue

                self.run.event(now, "RESUME", f"관측 관문 {name} 진행")
                return

            time.sleep(CONTROL_PERIOD_S)

    # 회전 정렬 모드의 정렬 경유점 최종 방향 정렬(경유점 도착 뒤 1회, 관문 폴더 유무와 무관).
    # 순서: 0 명령·정지 확인 → 입력·odom·측면 검사 → align.heading_rad로 rotation_align 절차 1회 → 정지 뒤
    # 방향(final_tolerance와 max_heading 모두)·측면 재검사. 측면 초과·입력 문제·odom 비유한·재검사 실패는
    # 회전으로 해소하지 않고 refused로 기록한다. refused는 final_align_blocker로 관문 재출발·관문 없는 출발을
    # 끝까지 막으며 입력 회복만으로 풀리지 않고 다시 회전하지도 않는다(새 실행 필요). 정지 확인 실패와 회전 중
    # 실패(입력 미회복·회전 원 장애물·시계·중단·다른 발행자·재정렬 상한)는 예외로 끝낸다.
    # 절차는 경유점 방문마다 한 번이며 rotation_align의 재정렬 횟수·sim/벽시계 제한을 그대로 쓴다.
    def final_align(self, index, gate):
        align = self.route["alignment"][index]
        law = self.route["rotation_align"]
        goal = self.route["waypoints"][index]
        name = self.route["gates"].get(index) if gate else None
        entry = {"waypoint": index + 1, "gate": name, "trigger": "align_final", "target_rad": align["heading_rad"],
                 "final_tolerance_rad": law["final_tolerance_rad"], "limits": align,
                 "start_sim_s": self.node.now_seconds(), "before": None, "after": None}
        self.run.record.setdefault("final_align", []).append(entry)
        self.final_outcomes[index] = entry

        def finish(outcome, reason=None):
            entry.update({"outcome": outcome, "reason": reason, "end_sim_s": self.node.now_seconds()})
            self.run.event(entry["end_sim_s"], "ALIGN",
                           f"경유점 {index + 1} 최종 방향 정렬 {outcome}" + (f": {reason}" if reason else ""))
            self.run.save()

        def measure():
            with self.node.lock:
                odom = self.node.odom

            if odom is None:
                return None, "odom 미수신"

            result = alignment_error(odom[1:4], goal, align)
            snapshot = {key: result[key] for key in ("along_m", "lateral_m", "heading_error_rad")}
            snapshot["odom_pose"] = list(odom[1:4])

            if result["lateral_m"] is None:
                return snapshot, "odom 비유한 값"

            return snapshot, None

        self.note_phase(index, "align_stop")

        if not self.stop_and_confirm(f"최종 방향 정렬 전 정지(경유점 {index + 1})"):
            finish("failed", "odom 기준 정지를 확인하지 못함")

            if name:
                self.write_snapshot("gate_refused", name, "최종 방향 정렬 전 odom 기준 정지 미확인")

            raise RuntimeError(f"경유점 {index + 1}: 최종 방향 정렬 전 odom 기준 정지를 확인하지 못함")

        entry["before"], problem = measure()
        problems = self.input_problems()

        if problem or problems:
            finish("refused", "회전하지 않음: " + ", ".join(([problem] if problem else []) + problems))
            return entry

        if abs(entry["before"]["lateral_m"]) > align["max_lateral_m"]:
            finish("refused", f"측면 {entry['before']['lateral_m']:+.4f} m > {align['max_lateral_m']} m: 회전으로 해소하지 않음")
            return entry

        tolerance = self.heading_limit(index)

        if abs(entry["before"]["heading_error_rad"]) <= tolerance:
            entry["after"] = entry["before"]
            finish("already_within")
            return entry

        try:
            self.align_to(index, align["heading_rad"], "align_final")
        except BaseException as error:
            entry["after"], _ = measure()
            finish("failed", str(error) or type(error).__name__)

            if name and not isinstance(error, KeyboardInterrupt):
                self.write_snapshot("gate_refused", name, f"최종 방향 정렬 실패: {error}")

            raise

        entry["after"], problem = measure()

        if problem:
            finish("refused", f"회전 뒤 {problem}")
        elif abs(entry["after"]["lateral_m"]) > align["max_lateral_m"]:
            finish("refused", f"회전 뒤 측면 {entry['after']['lateral_m']:+.4f} m > {align['max_lateral_m']} m")
        elif abs(entry["after"]["heading_error_rad"]) > tolerance:
            finish("refused", f"회전 뒤 방향 {entry['after']['heading_error_rad']:+.5f} rad > {tolerance} rad")
        else:
            finish("aligned")

        return entry

    # 전체 경로를 주행하고 정지를 확인한다. gate_dir가 있을 때만 관측 관문에서 기다린다.
    # 회전 정렬 모드에서 정렬 경유점이면 관문/출발 검사 전에 최종 방향 정렬을 한 번 한다.
    def follow(self):
        for index, goal in enumerate(self.route["waypoints"]):
            self.drive_to(index, goal)
            gate = self.gate_dir is not None and index in self.route.get("gates", {})

            checked = gate or index + 1 < len(self.route["waypoints"])

            if self.route.get("rotation_align") and index in self.route.get("alignment", {}) and checked:
                self.final_align(index, gate)

            if gate:
                self.wait_gate(index)
            elif index in self.route.get("alignment", {}) and index + 1 < len(self.route["waypoints"]):
                # 관문 대기가 없어도 정렬 경유점에서는 정지를 확인하고 정렬을 검사한 뒤에만 다음 구간으로 간다.
                if not self.stop_and_confirm(f"정렬 확인 경유점 {index + 1}"):
                    raise RuntimeError(f"경유점 {index + 1}: odom 기준 정지를 확인하지 못해 정렬을 검사하지 않음")

                alignment = self.check_alignment(index, "관문 없이 출발 전")
                final = self.final_align_blocker(index)

                if not alignment["within"] or final:
                    reason = "; ".join(item for item in (alignment["reason"], final) if item)
                    raise RuntimeError(f"경유점 {index + 1} 다음 구간 출발 거부: {reason}")

        if not self.stop_and_confirm("경로 완료"):
            raise RuntimeError("경로 완료 뒤 odom 기준 정지를 확인하지 못함")


# 실제 spawn에 쓰인 robot_description을 run 폴더에 저장하고 해시·바퀴 마찰·LiDAR 설정을 기록한다.
# 받지 못하거나 해석하지 못하면 값을 채우지 않고 누락 사유만 남긴다.
def record_model(node, run, timeout_s=10.0):
    deadline = time.monotonic() + timeout_s
    topic = f"{node.namespace}/robot_description"

    while time.monotonic() < deadline:
        with node.lock:
            text = node.robot_description

        if text:
            break

        time.sleep(0.1)
    else:
        run.record["model"] = {"source": topic, "missing_reason": f"{timeout_s:.0f} s 안에 {topic} 미수신"}
        return

    path = run.output / "robot_description.urdf"
    path.write_text(text, encoding="utf-8")
    model = {"source": topic, "file": path.name, "sha256": sha256_of(path)}

    try:
        model.update(describe_model(text))
    except Exception as error:
        model["missing_reason"] = f"URDF 해석 실패: {type(error).__name__}: {error}"

    run.record["model"] = model


# 저장 직전에 odom 기준 정지를 다시 확인하고, 확인됐을 때만 save_fn(지도 저장)을 부른다.
# 확인하지 못하면 저장하지 않고 RuntimeError를 낸다. 저장은 run 폴더에만 하며 채택 지도는 건드리지 않는다.
def save_after_stop(follower, run, save_fn):
    confirmed = follower.stop_and_confirm("저장 전")
    run.record["stop_gate"]["save"] = {"confirmed": confirmed, "stop": run.record["stops"][-1] if run.record["stops"] else None}

    if not confirmed:
        run.record["saved_map_usable"] = False
        raise RuntimeError("저장 전 odom 기준 정지를 확인하지 못해 지도를 저장하지 않음")

    save_fn()


# 종료 시 odom 기준 정지 확인 결과를 기록에 반영하고 종료 코드를 돌려준다.
# 정지를 확인하지 못하면 완료로 남기지 않으며, 이미 저장한 지도도 사용 불가로 표시한다.
def close_run(record, confirmed, exit_code):
    record["stop_gate"]["completion"] = {"confirmed": confirmed}

    if record["saved"]:
        record["saved_map_usable"] = bool(confirmed and record["status"] == "completed")

    if not confirmed and record["status"] == "completed":
        record["status"] = "failed"
        record["error"] = "종료 시 odom 기준 정지를 확인하지 못함"
        exit_code = 1

    return exit_code


# slam_toolbox 저장 Service를 불러 지도와 pose graph를 run 폴더에 저장한다.
def save_map(node, run):
    from slam_toolbox.srv import SerializePoseGraph

    namespace = node.namespace
    saved = {}

    # 마지막 스캔이 지도에 반영되도록 갱신 주기보다 오래 기다린다.
    start = node.now_seconds()
    wall_deadline = time.monotonic() + 60.0

    while node.now_seconds() - start < MAP_UPDATE_WAIT_S and time.monotonic() < wall_deadline:
        node.publish_stop()
        time.sleep(CONTROL_PERIOD_S)

    # 1) 지도: nav2 map_saver_cli를 직접 실행한다.
    # slam_toolbox save_map이 띄우는 map_saver는 2 s만 기다리고(느린 시뮬레이션에서 지도 메시지를 놓침)
    # 미관측 기준값(free 0.25)을 바꿀 수 없다. free 0.196이면 미관측(205)을 다시 읽을 때 빈 공간이 되지 않는다.
    command = [
        "ros2", "run", "nav2_map_server", "map_saver_cli",
        "-f", str(run.output / "slam_map"), "-t", f"{namespace}/map",
        "--fmt", "pgm", "--mode", "trinary", "--free", f"{MAP_FREE_THRESH}", "--occ", f"{MAP_OCCUPIED_THRESH}",
        "--ros-args", "-p", f"save_map_timeout:={MAP_SAVE_TIMEOUT_S}", "-p", "map_subscribe_transient_local:=true",
        "-r", "__node:=ax_mapping_map_saver",
    ]

    try:
        completed = subprocess.run(command, capture_output=True, text=True, timeout=MAP_SAVE_TIMEOUT_S + 60.0)
        code, output = completed.returncode, completed.stdout + completed.stderr
    except subprocess.TimeoutExpired as error:
        code, output = -1, f"시간 초과: {error}"

    (run.output / "map_saver.log").write_text(output, encoding="utf-8")

    # 2) pose graph: slam_toolbox serialize_map 서비스
    client = node.create_client(SerializePoseGraph, f"{namespace}/slam_toolbox/serialize_map")
    graph_code = None

    if client.wait_for_service(timeout_sec=10.0):
        request = SerializePoseGraph.Request()
        request.filename = str(run.output / "pose_graph")
        future = client.call_async(request)
        deadline = time.monotonic() + 60.0

        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)

        graph_code = int(future.result().result) if future.done() else None

    # 성공 코드와 실제 파일 생성을 함께 확인한다.
    checks = (
        ("map_saver_cli", code, [run.output / "slam_map.yaml", run.output / "slam_map.pgm"]),
        ("serialize_map", graph_code, [run.output / "pose_graph.posegraph", run.output / "pose_graph.data"]),
    )

    for name, result, files in checks:
        wait_until = time.monotonic() + 10.0

        while time.monotonic() < wait_until and not all(f.is_file() and f.stat().st_size > 0 for f in files):
            time.sleep(0.2)

        missing = [f.name for f in files if not (f.is_file() and f.stat().st_size > 0)]
        saved[name] = {"result_code": result, "files": {f.name: sha256_of(f) for f in files}, "missing": missing}

    run.record["saved"] = saved
    failed = [name for name, item in saved.items() if item["result_code"] != 0 or item["missing"]]

    if failed:
        raise RuntimeError("저장 실패: " + ", ".join(f"{name}(결과 {saved[name]['result_code']}, 없는 파일 {saved[name]['missing']})"
                                                for name in failed))

    run.event(node.now_seconds(), "SAVED", "지도·pose graph 저장 확인")


# 출발점 복귀와 map→odom 보정량으로 루프 폐쇄 근거를 정리한다(간접 근거).
def loop_evidence(node, route):
    with node.lock:
        odom = list(node.odom_record)
        map_pose = list(node.map_record)

    evidence = {"method": "간접: 루프 기준 경유점 복귀 거리(odom)와 map→odom 보정량. slam_toolbox 루프 폐쇄 사건을 직접 읽지 않는다"}

    # 루프 기준점: 경로 파일의 loop_waypoint(1부터)번 경유점. 없으면 odom 출발점.
    if odom:
        last = odom[-1]
        index = route.get("loop_waypoint")
        point = route["waypoints"][index - 1][:2] if index else odom[0][1:3]
        evidence["loop_point_odom"] = list(point)
        evidence["return_to_loop_point_odom_m"] = math.hypot(last[1] - point[0], last[2] - point[1])
        evidence["returned_to_loop_point"] = evidence["return_to_loop_point_odom_m"] <= route.get("return_tolerance_m", 0.3)

    if len(map_pose) >= 2:
        first, last = map_pose[0], map_pose[-1]
        steps = [math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(map_pose, map_pose[1:])]
        evidence["map_odom_correction_m"] = math.hypot(last[1] - first[1], last[2] - first[2])
        evidence["map_odom_correction_rad"] = wrap_angle(last[3] - first[3])
        evidence["max_map_odom_step_m"] = max(steps)

    return evidence


# 참조 지도용 기록(스캔·odom·map→odom·Gazebo 실제 자세)을 npz로 저장한다.
def save_recording(node, truth, run):
    with node.lock:
        front = list(node.front_record)
        odom = np.array(node.odom_record, float).reshape(-1, 6)
        map_pose = np.array(node.map_record, float).reshape(-1, 4)
        health = np.array(node.health_record, float).reshape(-1, len(HEALTH_COLUMNS))
        gaps = np.array(node.slam.gaps, float)
        layout = getattr(node, "front_layout", None)

    if not front or layout is None:
        return

    np.savez_compressed(
        run.output / "recording.npz",
        scan_stamps=np.array([item[0] for item in front]),
        scan_ranges=np.stack([item[1] for item in front]),
        scan_layout=np.array(layout[:4], float),
        scan_frame=np.array(layout[4]),
        front_mount=np.array(node.mounts.get("front", (np.nan,) * 3), float),
        odom=odom,
        map_odom=map_pose,
        slam_health=health,
        slam_health_columns=np.array(HEALTH_COLUMNS),
        tf_receive_gaps_boottime=gaps,
        truth=truth.array(),
    )
    run.record["recording"] = {"file": "recording.npz", "scans": len(front), "truth_samples": int(len(truth.array())),
                               "note": "truth는 Gazebo 실제 자세(검증 전용)"}

    # map→odom 수신 간격 분포(CLOCK_BOOTTIME s)
    if len(gaps):
        run.record["tf_receive_gap_boottime_s"] = {
            "count": int(len(gaps)), "p50": float(np.percentile(gaps, 50)), "p95": float(np.percentile(gaps, 95)),
            "p99": float(np.percentile(gaps, 99)), "max": float(gaps.max()),
            "receipts": node.slam.receipts, "stamp_updates": node.slam.updates,
        }

    # SLAM 상태 세 항목의 최댓값 요약(NaN 제외)
    if len(health):
        run.record["slam_health_max"] = {
            name: (float(np.nanmax(health[:, index])) if np.isfinite(health[:, index]).any() else None)
            for index, name in enumerate(HEALTH_COLUMNS) if index > 0
        }


# SLAM 입력 종류에 맞는 장애물 감시용 앞·뒤 스캔 토픽 접미사. 병합 스캔은 감시에 쓰지 않는다.
def obstacle_scan_suffix(slam_scan_input):
    if slam_scan_input not in ("merged", "filtered", "raw"):
        raise ValueError(f"알 수 없는 스캔 입력 {slam_scan_input!r}")

    return "scan" if slam_scan_input == "raw" else "scan_filtered"


# 출발 전 문제 목록: 입력 문제에 sim time 미수신을 더한다. sim time이 0이면(/clock 전) 진척 감시 기준 시각이
# 0으로 잡혀 첫 갱신에서 실제 경과로 건너뛰므로 출발하지 않는다(2026-11-14 중앙 경로 AMCL 회차에서 재현).
def start_problems(input_problems, sim_now):
    problems = list(input_problems)

    if not (isinstance(sim_now, (int, float)) and math.isfinite(sim_now) and sim_now > 0.0):
        problems.append("sim time 미수신(/clock)")

    return problems


# 명령행 인자를 읽어 지도 작성 주행을 실행한다.
def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--route", type=Path, default=PACKAGE / "config/routes/aws_warehouse.yaml")
    parser.add_argument("--scan", choices=["merged", "filtered", "raw"], default="merged",
                        help="SLAM 입력 종류(mapping.launch.py scan과 같게). 장애물 감시는 merged·filtered면 앞·뒤 B1 scan_filtered, raw면 원시 scan")
    parser.add_argument("--gz-world", default="ax_test", help="Gazebo 월드 내부 이름(검증 기록용)")
    parser.add_argument("--output", type=Path, default=None, help="결과 폴더(기본 log/b2_mapping/<KST 시각>)")
    parser.add_argument("--no-save", action="store_true", help="주행만 하고 지도를 저장하지 않음")
    parser.add_argument("--max-waypoints", type=int, default=None, help="앞에서부터 이 개수의 경유점까지만 주행(짧은 확인용)")
    parser.add_argument("--gate-dir", type=Path, default=None,
                        help="관측 관문 신호·정지 기록 폴더. 경로의 gate: true 경유점에서 정지 확인 뒤 진행 파일을 기다린다"
                             "(없으면 관문 표시를 무시하고 기존처럼 연속 주행)")
    parser.add_argument("--world", default=None,
                        help="실행 월드 파일(ax_description worlds 기준, 예: planned/ax_warehouse.world). "
                             "주면 경로 파일의 world가 같아야 한다")
    parser.add_argument("--robot-id", default=None, help="실행 로봇 ID. 주면 경로 파일의 robot_id가 같아야 한다")
    parser.add_argument("--check-only", action="store_true",
                        help="경로·월드·로봇 일치만 검사하고 ROS에 연결하지 않고 끝낸다(세션 기동 전 검사)")
    arguments = parser.parse_args()

    try:
        route = load_route(arguments.route)
        problems = route_world_problems(route, arguments.world, arguments.gz_world, arguments.robot_id)
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as error:
        parser.error(f"경로 파일을 읽을 수 없음: {arguments.route}: {error}")

    if problems:
        parser.error("경로·월드 불일치: " + "; ".join(problems))

    if arguments.check_only:
        print(json.dumps({"route": str(arguments.route), "route_sha256": sha256_of(arguments.route),
                          "world": route.get("world"), "gz_world": route.get("gz_world"), "robot_id": route["robot_id"],
                          "waypoints": len(route["waypoints"]),
                          "gates": {str(index + 1): name for index, name in route["gates"].items()},
                          "alignment": {str(index + 1): value for index, value in route["alignment"].items()},
                          "rotation_align": route["rotation_align"]},
                         ensure_ascii=False), flush=True)
        sys.exit(0)

    if arguments.max_waypoints is not None:
        if arguments.max_waypoints < 1:
            parser.error("--max-waypoints는 1 이상")

        route["waypoints"] = route["waypoints"][:arguments.max_waypoints]
        route["partial_route"] = True
    scan_suffix = obstacle_scan_suffix(arguments.scan)

    stamp = datetime.now(ZoneInfo("Asia/Seoul")).strftime("%Y%m%d_%H%M%S")
    output = (arguments.output or WORKSPACE / "log/b2_mapping" / stamp).resolve()

    if output.exists() and any(output.iterdir()):
        parser.error(f"{output}이 비어 있지 않음")

    output.mkdir(parents=True, exist_ok=True)
    run = Run(output, arguments.route, route, scan_suffix, slam_scan_input=arguments.scan)
    run.record["world"] = {"file": route.get("world"), "requested": arguments.world, "gz_world": arguments.gz_world}
    run.save()

    # Ctrl+C로 rclpy가 먼저 닫히면 정지 명령을 보낼 수 없으므로 신호를 직접 받아 정지 절차를 밟는다.
    stop_flag = threading.Event()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, lambda *_: stop_flag.set())
    signal.signal(signal.SIGTERM, lambda *_: stop_flag.set())

    node = MappingDriver(route["robot_id"], scan_suffix)
    truth = GazeboTruthRecorder(route["robot_id"], arguments.gz_world)
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    # executor 스레드. 종료 시 조용히 끝낸다.
    def spin():
        try:
            executor.spin()
        except ExternalShutdownException:
            pass

    spinner = threading.Thread(target=spin, daemon=True)
    spinner.start()
    gate_token = None

    if arguments.gate_dir is not None:
        arguments.gate_dir = arguments.gate_dir.resolve()
        arguments.gate_dir.mkdir(parents=True, exist_ok=True)
        gate_token = f"{output.name}-{os.urandom(4).hex()}"
        run.record["gate"] = {"dir": str(arguments.gate_dir), "token": gate_token,
                              "gates": {str(index + 1): name for index, name in route["gates"].items()},
                              "signals": f"{gate_token}_<관문>.continue(관문마다 1회), {gate_token}.abort(우선)"}
        print(f"[GATE] 관측 관문 토큰 {gate_token}, 폴더 {arguments.gate_dir}", flush=True)
    elif route["gates"]:
        run.record["gate"] = {"ignored": True, "reason": "--gate-dir 없음: 관문 표시를 무시하고 연속 주행"}

    follower = RouteFollower(node, route, run, stop_flag, arguments.gate_dir, gate_token, truth)
    exit_code = 1

    try:
        truth.start()
        record_model(node, run)
        node.mounts = {name: node.lookup_mount(name) for name in ("front", "rear")}
        run.record["mounts"] = node.mounts
        follower.configure_limits(node.read_drive_limits())

        # 입력이 모두 들어오고 다른 발행자가 없으며 sim time을 받은 뒤 출발한다.
        deadline = time.monotonic() + 30.0

        while start_problems(follower.input_problems(), node.now_seconds()) and time.monotonic() < deadline:
            time.sleep(0.2)

        problems = start_problems(follower.input_problems(), node.now_seconds())

        if problems:
            raise RuntimeError("출발 전 입력 없음: " + ", ".join(problems))

        follower.check_interrupt()
        run.event(node.now_seconds(), "START",
                  f"경로 {arguments.route.name}, SLAM 입력 {arguments.scan}, 장애물 감시 입력 {scan_suffix}, 상한 {follower.limits}")
        follower.follow()

        if not arguments.no_save:
            save_after_stop(follower, run, lambda: save_map(node, run))

        run.record["status"] = "completed"
        exit_code = 0

    except KeyboardInterrupt:
        run.record["status"] = "interrupted"
        run.record["error"] = "사용자 중단(Ctrl+C·SIGTERM)"
        exit_code = 130

    except Exception as error:
        traceback.print_exc()
        run.record["status"] = "failed"
        run.record["error"] = f"{type(error).__name__}: {error}"

        try:
            follower.write_snapshot("failure", None, run.record["error"])
        except Exception as snapshot_error:
            run.event(None, "STOP", f"실패 시점 기록 실패: {snapshot_error}")

    finally:
        # 어떤 경우에도 정지를 요청하고 odom 기준 정지를 확인한다. 확인 실패는 정상 종료가 아니다.
        try:
            confirmed = follower.stop_and_confirm("종료")
        except Exception as error:
            confirmed = False
            run.event(None, "STOP", f"종료 정지 절차 실패: {error}")

        exit_code = close_run(run.record, confirmed, exit_code)

        run.record["loop"] = loop_evidence(node, route)
        truth.stop()
        save_recording(node, truth, run)
        run.save()

        # executor 스레드가 C++ 대기에서 빠져나온 뒤 노드를 지운다. 대기 중 종료하면 프로세스가 비정상 종료할 수 있다.
        executor.shutdown()
        spinner.join(timeout=5.0)
        node.destroy_node()
        rclpy.try_shutdown()

    print(f"결과: {output / 'drive_result.json'} (status={run.record['status']})", flush=True)
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
