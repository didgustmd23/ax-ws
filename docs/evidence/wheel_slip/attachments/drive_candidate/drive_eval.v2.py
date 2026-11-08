#!/usr/bin/env python3
"""주행 허용 자세 후보 기준 구동 비교 시험(개발용, 격리 시뮬레이션 전용). plan.yaml의 고정 조건을 따른다.
a2_robot1_rotation/r1_rotation.py를 로봇 일반화·속도 범위·직진/곡선 오차·정지 시험으로 확장했다.

사용: python3 drive_eval.py <robot_1|robot_2> <조건 이름> <결과 json> [--speeds 0.15,0.5]

Gazebo 실제 자세는 관측·기록에만 쓰고 제어에는 쓰지 않는다. A4 안전 기능이나 G1 검증이 아니다.
"""

import argparse
import json
import math
import sys
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

sys.path.insert(0, str(Path.home() / "ax_ws/src/ax_control/scripts"))

from check_wheel_calibration import accumulated_rotation, body_displacement, fit_circle, rotation_translation  # noqa: E402

WORLD = "ax_test"
ROBOT = sys.argv[1] if len(sys.argv) > 1 and sys.argv[1].startswith("robot_") else "robot_1"
ARM_JOINTS = ["column_yaw_joint", "shoulder_joint", "elbow_joint", "wrist_joint", "gripper_rotation_joint"] if ROBOT == "robot_1" else []
WHEELS = ["front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel"]
POSES = {"neutral": [0.0, 0.0, 0.0, 0.0, 0.0], "reach_side": [0.0, -0.90, 1.25, -0.40, 0.0]}
WHEEL_RADIUS = 0.19631 if ROBOT == "robot_1" else 0.149900663548

# plan.yaml 고정값. 회전 속도는 회차마다 정한다.
TARGET_TURN = 3.0
ANGULAR_ACCEL = 0.2
STOP_WINDOW = 6.0
SETTLE = 2.0
STALE = 0.5
TILT = 0.15
WHEEL_LIFT = 0.02
ARM_DEVIATION = 0.05

# 예상 밖 접촉·미끄러짐: odom 선속도가 0.2 m/s 이상인데 실제 속도가 그 절반 미만인 상태가 0.5 s 이어지면 중단
MOTION_MIN_ODOM = 0.2
MOTION_RATIO = 0.5
MOTION_HOLD = 0.5
STOP_V, STOP_W, STOP_HOLD = 0.01, 0.02, 0.5


# 사원수 → (roll, pitch, yaw)
def euler(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    return roll, pitch, yaw


# 3×3 회전 행렬(사원수)
def rotation(q):
    x, y, z, w = q.x, q.y, q.z, q.w

    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


# Gazebo 관측 기록(검증 전용)
class Truth:
    # 구독할 월드와 접촉 센서 사용 여부
    def __init__(self, contact):
        from gz.msgs10.contacts_pb2 import Contacts
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node as GzNode

        self.lock = threading.Lock()
        self.model = []                    # (t, x, y, z, roll, pitch, yaw)
        self.wheel_z = {w: [] for w in WHEELS}
        self.base_in_model = []
        self.contacts = []                 # (t, wheel, points, normal_sum, axle_sum)
        self.node = GzNode()
        self.topics = [f"/world/{WORLD}/dynamic_pose/info"]
        self.node.subscribe(Pose_V, self.topics[0], self.on_pose)

        if contact:
            for wheel in WHEELS:
                topic = f"/world/{WORLD}/model/{ROBOT}/link/{wheel}/sensor/{wheel}_contact/contact"
                self.node.subscribe(Contacts, topic, self.contact_handler(wheel))
                self.topics.append(topic)

    # 모델(월드 기준)과 링크(모델 기준) 자세를 기록한다.
    def on_pose(self, message):
        t = message.header.stamp.sec + message.header.stamp.nsec * 1e-9
        model = None
        links = {}

        for pose in message.pose:
            if pose.name == ROBOT:
                model = pose
            elif pose.name in WHEELS or pose.name == "base_link":
                links[pose.name] = pose

        if model is None:
            return

        r, p, y = euler(model.orientation)
        origin = np.array([model.position.x, model.position.y, model.position.z])
        frame = rotation(model.orientation)

        with self.lock:
            self.model.append((t, model.position.x, model.position.y, model.position.z, r, p, y))

            for name, pose in links.items():
                local = np.array([pose.position.x, pose.position.y, pose.position.z])

                if name == "base_link":
                    self.base_in_model.append(local.tolist() + [euler(pose.orientation)[2]])
                else:
                    self.wheel_z[name].append((t, float((origin + frame @ local)[2])))

    # 바퀴 접촉: 접촉점 수, 수직 하중 합, 차체 옆방향 힘 합
    def contact_handler(self, wheel):
        def handler(message):
            t = message.header.stamp.sec + message.header.stamp.nsec * 1e-9

            with self.lock:
                yaw = self.model[-1][6] if self.model else 0.0

            c, s = math.cos(yaw), math.sin(yaw)
            points = normal = lateral = 0.0

            for contact in message.contact:
                for index in range(len(contact.position)):
                    wrench = contact.wrench[index]
                    force = wrench.body_1_wrench.force if wheel in contact.collision1.name else wrench.body_2_wrench.force
                    points += 1
                    normal += force.z
                    lateral += -s * force.x + c * force.y

            with self.lock:
                self.contacts.append((t, wheel, points, normal, lateral))

        return handler

    # 구간 [t0, t1]의 모델 기록
    def window(self, t0, t1):
        with self.lock:
            return [row for row in self.model if t0 <= row[0] <= t1]

    # 구독 해제
    def stop(self):
        for topic in self.topics:
            self.node.unsubscribe(topic)


# 시험 노드: odom·joint_states 기록, 바퀴 명령, 팔 Action
class Tester(Node):
    # 구독·발행·Action 클라이언트
    def __init__(self):
        super().__init__("r1_rotation", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.lock = threading.Lock()
        self.odom = []
        self.joints = []
        ns = f"/ax/{ROBOT}"
        self.create_subscription(Odometry, f"{ns}/diff_drive_controller/odom", self.on_odom, 50)
        self.create_subscription(JointState, f"{ns}/joint_states", self.on_joints, 50)
        self.command = self.create_publisher(TwistStamped, f"{ns}/diff_drive_controller/cmd_vel", 10)
        self.arm = ActionClient(self, FollowJointTrajectory, f"{ns}/arm_controller/follow_joint_trajectory")

    # odom 기록 (t, x, y, yaw, v, w)
    def on_odom(self, m):
        q = m.pose.pose.orientation

        with self.lock:
            self.odom.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.pose.pose.position.x,
                              m.pose.pose.position.y, euler(q)[2], m.twist.twist.linear.x, m.twist.twist.angular.z))

    # joint_states 기록 (t, {이름: (위치, 속도)})
    def on_joints(self, m):
        with self.lock:
            self.joints.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                                {n: (p, v) for n, p, v in zip(m.name, m.position, m.velocity)}))

    # 현재 sim 시각
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # 속도 명령
    def publish(self, linear, angular):
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.twist.linear.x, message.twist.angular.z = float(linear), float(angular)
        self.command.publish(message)

    # 팔을 자세로 옮기고 결과·도달을 확인한다.
    def move_arm(self, positions, seconds=8.0):
        if not self.arm.wait_for_server(timeout_sec=30.0):
            raise RuntimeError("arm_controller Action 서버 없음")

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS
        point = JointTrajectoryPoint(positions=positions, time_from_start=Duration(seconds=seconds).to_msg())
        goal.trajectory.points = [point]
        future = self.arm.send_goal_async(goal)

        while not future.done():
            time.sleep(0.05)

        handle = future.result()

        if not handle.accepted:
            raise RuntimeError("팔 목표 거절")

        result = handle.get_result_async()

        while not result.done():
            time.sleep(0.05)

        error = self.arm_error(positions)

        if error > 0.02:
            raise RuntimeError(f"팔 자세 미도달 {error:.3f} rad")

        return error

    # 팔 관절 목표 대비 최대 오차
    def arm_error(self, positions):
        with self.lock:
            latest = self.joints[-1][1] if self.joints else {}

        return max(abs(latest[n][0] - p) for n, p in zip(ARM_JOINTS, positions)) if latest else math.inf


# 시험 실행
class Trial:
    # 노드·관측·팔 목표
    def __init__(self, node, truth, arm_target):
        self.node, self.truth, self.arm_target = node, truth, arm_target

    # 입력 신선도·자세 안정성·팔 이탈 검사. 문제가 있으면 사유 문자열
    def problem(self):
        now = self.node.now()
        reason = self.motion_problem(now)

        if reason:
            return reason

        with self.node.lock:
            odom = self.node.odom[-1] if self.node.odom else None
            joints = self.node.joints[-1] if self.node.joints else None

        with self.truth.lock:
            model = self.truth.model[-1] if self.truth.model else None
            wheels = {w: (v[-1][1] if v else None) for w, v in self.truth.wheel_z.items()}

        for name, item in (("odom", odom), ("joint_states", joints), ("Gazebo 자세", model)):
            if item is None or now - item[0] > STALE:
                return f"{name} 만료"

        if abs(model[4]) > TILT or abs(model[5]) > TILT:
            return f"차체 기울기 roll {model[4]:.3f}, pitch {model[5]:.3f}"

        lifted = [w for w, z in wheels.items() if z is not None and z > WHEEL_RADIUS + WHEEL_LIFT]

        if lifted:
            return "바퀴 들림 " + ",".join(lifted)

        deviation = max([abs(joints[1][n][0] - p) for n, p in zip(ARM_JOINTS, self.arm_target)] or [0.0])

        if deviation > ARM_DEVIATION:
            return f"팔 자세 이탈 {deviation:.3f} rad"

        return None

    # odom 선속도와 실제 차체 속도(0.2 s 차분)를 비교해 예상 밖 접촉·미끄러짐을 찾는다.
    def motion_problem(self, now):
        with self.node.lock:
            odom = self.node.odom[-1] if self.node.odom else None

        recent = self.truth.window(now - 0.25, now)

        if odom is None or len(recent) < 2 or recent[-1][0] - recent[0][0] < 0.1:
            return None

        speed = math.hypot(recent[-1][1] - recent[0][1], recent[-1][2] - recent[0][2]) / (recent[-1][0] - recent[0][0])
        mismatch = abs(odom[4]) >= MOTION_MIN_ODOM and speed < MOTION_RATIO * abs(odom[4])
        since = getattr(self, "_mismatch_since", None)

        if not mismatch:
            self._mismatch_since = None
            return None

        self._mismatch_since = since if since is not None else now

        if now - self._mismatch_since >= MOTION_HOLD:
            return f"unexpected_motion: 실제 속도 {speed:.3f} m/s, odom {odom[4]:.3f} m/s"

        return None

    # 명령을 seconds 동안 유지한다(sim). 문제가 생기면 0 명령 뒤 사유를 돌려준다.
    def hold(self, linear, angular, seconds, until=None):
        end = self.node.now() + seconds
        wall = time.monotonic() + 30 + 10 * seconds

        while self.node.now() < end and time.monotonic() < wall:
            reason = self.problem()

            if reason:
                self.node.publish(0, 0)
                return reason

            if until is not None and until():
                return None

            self.node.publish(linear, angular)
            time.sleep(0.05)

        return None if self.node.now() >= end else "sim 시간 정지(벽시계 제한)"

    # 0 명령 뒤 정지를 기다리며 기록한다.
    def stop_window(self):
        return self.hold(0, 0, STOP_WINDOW)

    # 구간 자료
    def series(self, t0, t1):
        with self.node.lock:
            odom = [r for r in self.node.odom if t0 <= r[0] <= t1]
            joints = [r for r in self.node.joints if t0 <= r[0] <= t1]

        truth = self.truth.window(t0, t1)

        with self.truth.lock:
            wheel_z = {w: [z for t, z in v if t0 <= t <= t1] for w, v in self.truth.wheel_z.items()}
            contacts = [c for c in self.truth.contacts if t0 <= c[0] <= t1]

        return odom, joints, truth, wheel_z, contacts

    # 회전 한 회차
    def rotation(self, direction, repeat, omega):
        cutoff = TARGET_TURN - omega * omega / (2 * ANGULAR_ACCEL)
        limit = TARGET_TURN / omega + 15.0
        result = {"repeat": repeat, "direction": "left" if direction > 0 else "right", "omega_rad_s": omega,
                  "cutoff_odom_turn_rad": cutoff, "time_limit_s": limit, "status": "completed", "reasons": []}
        reason = self.hold(0, 0, SETTLE)

        if reason:
            result.update(status="aborted", reasons=[reason])
            return result

        t_start = self.node.now()

        with self.node.lock:
            yaw0 = self.node.odom[-1][3]
            samples = [(t_start, yaw0)]

        # odom 누적 회전이 컷오프에 닿을 때까지 명령한다.
        turned = {"value": 0.0}

        def reached():
            with self.node.lock:
                rows = [(r[0], r[3]) for r in self.node.odom if r[0] >= t_start]

            value = accumulated_rotation(rows) if len(rows) > 1 else 0.0
            turned["value"] = abs(value or 0.0)

            return turned["value"] >= cutoff

        reason = self.hold(0, omega * direction, limit, until=reached)
        t_cut = self.node.now()

        if reason:
            result["status"] = "aborted"
            result["reasons"].append(reason)
        elif turned["value"] < cutoff:
            result["status"] = "target_not_reached"
            result["reasons"].append(f"제한시간 {limit:.0f}s 안에 odom {turned['value']:.3f} rad")

        self.node.publish(0, 0)
        reason = self.stop_window()

        if reason:
            result["status"] = "aborted"
            result["reasons"].append("정지 구간: " + reason)

        t_end = self.node.now()
        result.update(self.analyse(t_start, t_cut, t_end, direction, omega))

        if result["status"] == "completed" and not (result["stop"]["odom_confirmed"] and result["stop"]["truth_confirmed"]):
            result["status"] = "stop_unconfirmed"

        return result

    # 회차 분석: 회전·병진·회전 중심(구간별)·오차·바퀴·안정성·정지
    def analyse(self, t0, t_cut, t1, direction, omega=0.3):
        odom, joints, truth, wheel_z, contacts = self.series(t0, t1)

        if len(odom) < 5 or len(truth) < 5:
            return {"analysis_error": f"표본 부족 odom {len(odom)}, truth {len(truth)}"}

        tr = np.array(truth)
        od = np.array(odom)
        tr_xyyaw = [(r[0], r[1], r[2], r[6]) for r in truth]
        od_xyyaw = [(r[0], r[1], r[2], r[3]) for r in odom]
        out = {"window_sim_s": [t0, t_cut, t1]}

        out["truth_turn_rad"] = accumulated_rotation([(r[0], r[3]) for r in tr_xyyaw])
        out["odom_turn_rad"] = accumulated_rotation([(r[0], r[3]) for r in od_xyyaw])
        out.update(rotation_translation(tr_xyyaw, od_xyyaw))
        out["truth_path_length_m"] = float(np.sum(np.hypot(np.diff(tr[:, 1]), np.diff(tr[:, 2]))))

        # 가속 끝: 0 명령 전 odom |w|가 0.95·명령에 처음 닿은 시각
        accel_end = next((r[0] for r in odom if r[0] <= t_cut and abs(r[5]) >= 0.95 * omega), t_cut)
        phases = {"accel": (t0, accel_end), "steady": (accel_end, t_cut), "decel": (t_cut, t1), "whole": (t0, t1)}
        x0, y0, h0 = tr[0, 1], tr[0, 2], tr[0, 6]
        circles = {}

        for name, (a, b) in phases.items():
            seg = tr[(tr[:, 0] >= a) & (tr[:, 0] <= b)]
            turn = accumulated_rotation([(r[0], r[6]) for r in seg]) if len(seg) > 2 else None
            entry = {"duration_s": b - a, "truth_turn_rad": turn, "points": int(len(seg))}

            if len(seg) >= 5 and turn is not None and abs(turn) >= 0.3:
                circle = fit_circle(seg[:, 1:3])
                residual = float(np.sqrt(np.mean((np.hypot(seg[:, 1] - circle[0], seg[:, 2] - circle[1]) - circle[2]) ** 2)))
                entry.update(centre_body_m=list(body_displacement((x0, y0), circle[:2], h0)), radius_m=circle[2],
                             residual_rms_m=residual, single_centre=bool(residual < 0.01))
            else:
                entry["note"] = "회전량이 0.3 rad 미만이라 원 맞춤 생략"

            circles[name] = entry

        out["circle_fit"] = circles

        # 같은 시각 odom 오차(시작 시각 기준 T_world_odom)
        times = tr[:, 0]

        def truth_at(t):
            i = int(np.clip(np.searchsorted(times, t), 1, len(times) - 1))
            a, b = tr[i - 1], tr[i]
            ratio = 0.0 if b[0] == a[0] else (t - a[0]) / (b[0] - a[0])
            yaw = a[6] + ratio * math.atan2(math.sin(b[6] - a[6]), math.cos(b[6] - a[6]))

            return a[1] + ratio * (b[1] - a[1]), a[2] + ratio * (b[2] - a[2]), yaw

        sx, sy, syaw = truth_at(od[0, 0])
        dyaw = syaw - od[0, 3]
        c, s = math.cos(dyaw), math.sin(dyaw)
        position_errors, yaw_errors = [], []

        for row in od:
            if row[0] < times[0] or row[0] > times[-1]:
                continue

            ox = sx + c * (row[1] - od[0, 1]) - s * (row[2] - od[0, 2])
            oy = sy + s * (row[1] - od[0, 1]) + c * (row[2] - od[0, 2])
            tx, ty, tyaw = truth_at(row[0])
            position_errors.append(math.hypot(ox - tx, oy - ty))
            yaw_errors.append(abs(math.atan2(math.sin(row[3] + dyaw - tyaw), math.cos(row[3] + dyaw - tyaw))))

        out["odom_error"] = {"position_max_m": max(position_errors), "position_end_m": position_errors[-1],
                             "yaw_max_rad": max(yaw_errors)}

        # 바퀴 속도(정상 회전 구간 평균)·중심 높이
        steady = [j for j in joints if accel_end <= j[0] <= t_cut]
        out["wheel_speed_steady_rad_s"] = {w: float(np.mean([j[1][f"{w}_joint"][1] for j in steady])) if steady else None
                                           for w in WHEELS}
        out["wheel_centre_z_m"] = {w: [float(min(v)), float(max(v))] if v else None for w, v in wheel_z.items()}
        out["tilt_max_rad"] = {"roll": float(np.abs(tr[:, 4]).max()), "pitch": float(np.abs(tr[:, 5]).max())}
        out["arm_deviation_max_rad"] = float(max([max([abs(j[1][n][0] - p) for n, p in zip(ARM_JOINTS, self.arm_target)] or [0.0])
                                                  for j in joints] or [0.0]))

        # 정지: 0 명령부터 odom·실제 각각 D-06 값 0.5 s 유지
        def stable(rows):
            start = None

            for t, v, w in rows:
                if abs(v) < STOP_V and abs(w) < STOP_W:
                    start = start if start is not None else t

                    if t - start >= STOP_HOLD:
                        return start
                else:
                    start = None

            return None

        after = tr[tr[:, 0] >= t_cut - 0.2]
        truth_v = []

        for a, b in zip(after, after[10:]):
            dt = b[0] - a[0]

            if dt > 0:
                truth_v.append((b[0], math.hypot(b[1] - a[1], b[2] - a[2]) / dt,
                                abs(math.atan2(math.sin(b[6] - a[6]), math.cos(b[6] - a[6]))) / dt))

        odom_stop = stable([(r[0], r[4], r[5]) for r in odom if r[0] >= t_cut])
        truth_stop = stable([r for r in truth_v if r[0] >= t_cut])
        out["stop"] = {"odom_stop_s": odom_stop - t_cut if odom_stop else None, "truth_stop_s": truth_stop - t_cut if truth_stop else None,
                       "odom_confirmed": odom_stop is not None, "truth_confirmed": truth_stop is not None}

        if contacts:
            out["contact"] = {}

            for w in WHEELS:
                rows = [c_ for c_ in contacts if c_[1] == w and accel_end <= c_[0] <= t_cut]

                if rows:
                    out["contact"][w] = {"min_points": min(r[2] for r in rows), "normal_mean_N": float(np.mean([r[3] for r in rows])),
                                         "lateral_mean_N": float(np.mean([r[4] for r in rows]))}

        # 그림용 경로(시작 차체 기준, 약 0.1 s 간격)
        c0, s0 = math.cos(h0), math.sin(h0)
        step = max(1, len(tr) // 300)
        out["path_body_m"] = [[round(c0 * (r[1] - x0) + s0 * (r[2] - y0), 4), round(-s0 * (r[1] - x0) + c0 * (r[2] - y0), 4)]
                              for r in tr[::step]]

        return out

    # 직진·곡선 한 회차: 실제 궤적과 odom의 위치·방향 오차(시작 시각 기준 정렬)
    def path(self, linear, angular, seconds, kind):
        reason = self.hold(0, 0, SETTLE)
        t0 = self.node.now()
        reason = reason or self.hold(linear, angular, seconds)
        t_cut = self.node.now()
        self.node.publish(0, 0)
        reason = reason or self.stop_window()
        t1 = self.node.now()
        out = self.analyse(t0, t_cut, t1, 1 if angular >= 0 else -1, abs(angular) or 0.3)
        odom, _, truth, _, _ = self.series(t0, t1)
        tr = np.array(truth)
        along, lateral = body_displacement(tr[0, 1:3], tr[-1, 1:3], tr[0, 6])
        oalong, olateral = body_displacement(odom[0][1:3], odom[-1][1:3], odom[0][3])
        status = "aborted" if reason else ("completed" if out["stop"]["odom_confirmed"] and out["stop"]["truth_confirmed"] else "stop_unconfirmed")

        return {"kind": kind, "command": [linear, angular, seconds], "status": status, "reason": reason,
                "truth_displacement_body_m": [along, lateral], "odom_displacement_body_m": [oalong, olateral],
                "truth_turn_rad": out.get("truth_turn_rad"), "odom_turn_rad": out.get("odom_turn_rad"),
                "odom_error": out.get("odom_error"), "stop": out["stop"], "tilt_max_rad": out["tilt_max_rad"],
                "wheel_centre_z_m": out["wheel_centre_z_m"], "path_body_m": out.get("path_body_m")}

    # 정지 시험(명령 뒤 0 명령, 정지 시간)
    def stop_test(self, linear, angular, seconds):
        reason = self.hold(0, 0, SETTLE)
        t0 = self.node.now()
        reason = reason or self.hold(linear, angular, seconds)
        t_cut = self.node.now()
        self.node.publish(0, 0)
        reason = reason or self.stop_window()
        out = self.analyse(t_cut - 0.5, t_cut, self.node.now(), 1)

        # 보고 정지 뒤 실제 추가 이동
        slide = None

        if out["stop"]["odom_stop_s"] is not None:
            tail = self.truth.window(t_cut + out["stop"]["odom_stop_s"], self.node.now())
            slide = math.hypot(tail[-1][1] - tail[0][1], tail[-1][2] - tail[0][2]) if len(tail) > 1 else None

        odom, _, truth, _, _ = self.series(t0, self.node.now())
        moved = {"truth_m": math.hypot(truth[-1][1] - truth[0][1], truth[-1][2] - truth[0][2]) if len(truth) > 1 else None,
                 "odom_m": math.hypot(odom[-1][1] - odom[0][1], odom[-1][2] - odom[0][2]) if len(odom) > 1 else None,
                 "start_world": list(truth[0][1:3]) + [truth[0][6]] if truth else None,
                 "end_world": list(truth[-1][1:3]) + [truth[-1][6]] if truth else None}
        status = "aborted" if reason else ("completed" if out["stop"]["odom_confirmed"] and out["stop"]["truth_confirmed"] else "stop_unconfirmed")

        if reason and reason.startswith("unexpected_motion"):
            status = "unexpected_motion"

        return {"command": [linear, angular, seconds], "status": status, "reason": reason, "stop": out["stop"],
                "slide_after_odom_stop_m": slide, "tilt_max_rad": out["tilt_max_rad"], "motion": moved}


# 실행 중인 diff_drive_controller 제한을 읽는다.
def controller_limits(node):
    from rclpy.parameter_client import AsyncParameterClient

    names = ["linear.x.max_velocity", "linear.x.max_acceleration", "angular.z.max_velocity", "angular.z.max_acceleration"]
    client = AsyncParameterClient(node, f"/ax/{ROBOT}/diff_drive_controller")

    if not client.wait_for_services(timeout_sec=20.0):
        return {"error": "파라미터 서비스 없음"}

    future = client.get_parameters(names)
    deadline = time.monotonic() + 10

    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.05)

    return {n: v.double_value for n, v in zip(names, future.result().values)} if future.done() else {"error": "응답 없음"}


# 실행
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("robot", choices=["robot_1", "robot_2"])
    parser.add_argument("condition")
    parser.add_argument("output", type=Path)
    parser.add_argument("--speeds", default="0.15,0.3,0.5", help="회전 속도 목록(rad/s)")
    args = parser.parse_args()
    speeds = [float(v) for v in args.speeds.split(",") if v]

    truth = Truth(False)
    rclpy.init()
    node = Tester()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    report = {"robot": ROBOT, "condition": args.condition, "status": "running", "rotations": [], "paths": [], "stop_tests": {}}
    wall0 = time.monotonic()
    sim0 = None

    try:
        deadline = time.monotonic() + 60

        while time.monotonic() < deadline and not (node.odom and node.joints and truth.model):
            time.sleep(0.2)

        sim0 = node.now()
        report["controller_limits"] = controller_limits(node)
        target = [0.0] * len(ARM_JOINTS)

        if ARM_JOINTS:
            report["arm_error_rad"] = node.move_arm(target)

        trial = Trial(node, truth, target)
        trial.hold(0, 0, 3.0)

        with truth.lock:
            report["base_link_in_model"] = truth.base_in_model[-1] if truth.base_in_model else None

        for omega in speeds:
            for index in range(6):
                direction = 1 if index % 2 == 0 else -1
                result = trial.rotation(direction, index // 2 + 1, omega)
                report["rotations"].append(result)
                print(args.condition, omega, result["direction"], result["status"], round(result.get("truth_translation_m", float("nan")), 4), flush=True)

        for speed, seconds in ((0.3, 3), (-0.3, 3), (0.3, 3), (-0.3, 3), (0.67, 2.5), (-0.67, 2.5)):
            report["paths"].append(trial.path(speed, 0.0, seconds, "straight"))

        for angular in (0.3, -0.3, 0.3, -0.3):
            report["paths"].append(trial.path(0.3, angular, 6.0, "curve"))

        report["stop_tests"] = {"linear_0.67": trial.stop_test(0.67, 0, 4.0), "angular_0.5": trial.stop_test(0, 0.5, 6.0),
                                "curve_0.3_0.3": trial.stop_test(0.3, 0.3, 4.0)}
        report["status"] = "completed"
    except Exception as error:
        report["status"] = "failed"
        report["error"] = f"{type(error).__name__}: {error}"
    finally:
        node.publish(0, 0)
        sim_s = node.now() - sim0 if sim0 is not None else 0.0
        wall_s = time.monotonic() - wall0
        report["clock"] = {"sim_s": sim_s, "wall_s": wall_s, "rtf": sim_s / wall_s if wall_s else None}
        truth.stop()
        executor.shutdown()
        spinner.join(timeout=3)
        args.output.write_text(json.dumps(report, indent=1, ensure_ascii=False, default=float))
        print("saved", args.output, report["status"], report.get("error"), flush=True)


if __name__ == "__main__":
    main()
