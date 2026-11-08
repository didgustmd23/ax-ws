#!/usr/bin/env python3
"""B2 개발용 지도 작성 주행. 경유점을 odom 기준으로 따라가며 SLAM 입력을 모으고 끝나면 지도·pose graph를 저장한다.

격리된 개발 시뮬레이션 전용 시험 경로다. A4(최종 구동 차단)가 아직 없어 diff_drive_controller에 직접 명령하므로
운영 주행 경로가 아니며 Nav2·D1 주행을 대신하지 않는다.

실행 순서(같은 ROS_DOMAIN_ID·GZ_PARTITION)
  1. ros2 launch ax_control simulation.launch.py world:=aws_warehouse.world robot_ids:=robot_2
  2. ros2 launch ax_perception sensors.launch.py robot_ids:=robot_2          (B1 scan_filtered)
  3. ros2 launch ax_localization mapping.launch.py robot_id:=robot_2
  4. python3 src/ax_localization/scripts/mapping_drive.py [--route ...] [--scan filtered|raw]

정지 조건: 스캔·odom·TF 만료, 차체 외곽 기준 장애물, 경유점 제한시간·진척 없음, 다른 cmd_vel 발행자, Ctrl+C.
정지는 명령(0 속도)과 실제 정지 확인(odom 기준 D-06 값 0.5 s 유지)을 구분해 기록한다. 확인하지 못하면 정상 종료가 아니다.

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
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
import tf2_ros

sys.path.insert(0, str(Path(__file__).resolve().parent))

from map_tools import se2, se2_params, transform_points, wrap_angle, yaw_from_quaternion  # noqa: E402

# D-06 실제 정지 판정 값(odom 기준). 판정 소유는 A4이며 여기서는 시험 기록용으로 같은 값을 쓴다.
STOP_LINEAR_M_S = 0.01
STOP_ANGULAR_RAD_S = 0.02
STOP_HOLD_S = 0.5
STOP_CONFIRM_TIMEOUT_S = 8.0

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
    waypoints = []

    for item in route["waypoints"]:
        if isinstance(item, dict):
            waypoints.append((float(item["xy"][0]), float(item["xy"][1]), bool(item.get("reverse", False))))
        else:
            waypoints.append((float(item[0]), float(item[1]), False))

    route["waypoints"] = waypoints

    return route


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
    def __init__(self, output, route_path, route, scan_suffix):
        self.output = output
        self.record = {
            "status": "running",
            "error": None,
            "development_test_path": "A4 미구현 상태에서 diff_drive_controller에 직접 명령하는 격리 개발 시험 경로",
            "robot_id": route["robot_id"],
            "route": {"path": str(route_path), "sha256": sha256_of(route_path), "frame": route["frame"],
                      "waypoints": route["waypoints"], "partial": route.get("partial_route", False)},
            "scan_input": scan_suffix,
            "limits": {},
            "waypoints_reached": 0,
            "events": [],
            "stops": [],
            "loop": {},
            "saved": {},
        }

    # 사건 하나를 기록하고 출력한다.
    def event(self, sim_time, kind, detail):
        self.record["events"].append({"sim_time_s": sim_time, "kind": kind, "detail": detail})
        print(f"[{kind}] {detail}", flush=True)

    # 중간 저장
    def save(self):
        write_json_atomic(self.output / "drive_result.json", self.record)


# 경로를 따라 주행한다. 중단 사유가 생기면 RuntimeError를 낸다.
class RouteFollower:
    # 노드·경로·실행 기록·종료 신호 플래그를 받는다.
    def __init__(self, node, route, run, stop_flag):
        self.node = node
        self.route = route
        self.run = run
        self.stop_flag = stop_flag

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

    # 정지 명령 뒤 odom이 D-06 값 안에 STOP_HOLD_S 동안 머무는지 확인한다.
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
            if odom is None and time.monotonic() - wall_start > 5.0:
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
                 "confirm_sim_s": self.node.now_seconds() - start if confirmed else None}
        self.run.record["stops"].append(entry)
        self.run.event(start, "STOP", f"{reason}: 실제 정지 {'확인' if confirmed else '미확인'}")

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

    # 경유점 하나까지 주행한다.
    def drive_to(self, index, goal):
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

            problem = self.obstacle_problem(linear, angular)

            if problem:
                self.wait_obstacle(problem, linear, angular)
                best["time"] = self.node.now_seconds()
                continue

            self.node.publish(linear, angular)
            time.sleep(CONTROL_PERIOD_S)

    # 전체 경로를 주행하고 정지를 확인한다.
    def follow(self):
        for index, goal in enumerate(self.route["waypoints"]):
            self.drive_to(index, goal)

        if not self.stop_and_confirm("경로 완료"):
            raise RuntimeError("경로 완료 뒤 실제 정지를 확인하지 못함")


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


# 명령행 인자를 읽어 지도 작성 주행을 실행한다.
def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--route", type=Path, default=PACKAGE / "config/routes/aws_warehouse.yaml")
    parser.add_argument("--scan", choices=["filtered", "raw"], default="filtered", help="B1 scan_filtered 또는 원시 scan")
    parser.add_argument("--gz-world", default="ax_test", help="Gazebo 월드 내부 이름(검증 기록용)")
    parser.add_argument("--output", type=Path, default=None, help="결과 폴더(기본 log/b2_mapping/<KST 시각>)")
    parser.add_argument("--no-save", action="store_true", help="주행만 하고 지도를 저장하지 않음")
    parser.add_argument("--max-waypoints", type=int, default=None, help="앞에서부터 이 개수의 경유점까지만 주행(짧은 확인용)")
    arguments = parser.parse_args()

    route = load_route(arguments.route)

    if arguments.max_waypoints is not None:
        if arguments.max_waypoints < 1:
            parser.error("--max-waypoints는 1 이상")

        route["waypoints"] = route["waypoints"][:arguments.max_waypoints]
        route["partial_route"] = True
    scan_suffix = "scan_filtered" if arguments.scan == "filtered" else "scan"

    stamp = datetime.now(ZoneInfo("Asia/Seoul")).strftime("%Y%m%d_%H%M%S")
    output = (arguments.output or WORKSPACE / "log/b2_mapping" / stamp).resolve()

    if output.exists() and any(output.iterdir()):
        parser.error(f"{output}이 비어 있지 않음")

    output.mkdir(parents=True, exist_ok=True)
    run = Run(output, arguments.route, route, scan_suffix)
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
    follower = RouteFollower(node, route, run, stop_flag)
    exit_code = 1

    try:
        truth.start()
        node.mounts = {name: node.lookup_mount(name) for name in ("front", "rear")}
        run.record["mounts"] = node.mounts
        follower.configure_limits(node.read_drive_limits())

        # 입력이 모두 들어오고 다른 발행자가 없는지 확인한 뒤 출발한다.
        deadline = time.monotonic() + 30.0

        while follower.input_problems() and time.monotonic() < deadline:
            time.sleep(0.2)

        problems = follower.input_problems()

        if problems:
            raise RuntimeError("출발 전 입력 없음: " + ", ".join(problems))

        follower.check_interrupt()
        run.event(node.now_seconds(), "START", f"경로 {arguments.route.name}, 입력 {scan_suffix}, 상한 {follower.limits}")
        follower.follow()

        if not arguments.no_save:
            save_map(node, run)

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

    finally:
        # 어떤 경우에도 정지를 요청하고 실제 정지를 확인한다. 확인 실패는 정상 종료가 아니다.
        try:
            confirmed = follower.stop_and_confirm("종료")
        except Exception as error:
            confirmed = False
            run.event(None, "STOP", f"종료 정지 절차 실패: {error}")

        if not confirmed and run.record["status"] == "completed":
            run.record["status"] = "failed"
            run.record["error"] = "종료 시 실제 정지를 확인하지 못함"
            exit_code = 1

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
