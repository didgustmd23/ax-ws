#!/usr/bin/env python3
"""A2 시험용 Gazebo 독립 관측. 제어기 보고값(joint_states·attach_state)과 다른 경로로 실제 상태를 읽는다.

Gazebo 실제 값은 시험 비교용이며 운영 판정에 쓰지 않는다.

관측 방법
- `/world/<world>/dynamic_pose/info`의 링크 자세(모델 기준)와 모델 자세(월드 기준)를 받는다.
- 링크 이름에는 로봇 접두사가 없으므로 `scene/info`의 엔티티 ID로 해당 로봇 링크만 고른다.
- Gazebo는 고정 관절로 이어진 링크를 하나로 합친다. 합쳐진 링크의 좌표계는 묶음의 뿌리 링크 좌표계다.
  그래서 관절각은 URDF의 고정 변환·관절 원점·축으로 인접 Gazebo 링크의 상대 회전에서 다시 계산한다.
- TCP(`epick_tcp`)도 합쳐져 사라지므로 공구 링크 자세에 URDF 고정 변환을 곱해 구한다.

순수 함수는 Gazebo 없이 단위 시험한다. Gazebo 연결은 `GazeboObserver`만 맡는다.
"""

from collections import deque
import math
import threading
import time
import xml.etree.ElementTree as ET

import numpy as np


# 사원수(x, y, z, w)를 3×3 회전 행렬로 바꾼다.
def quaternion_matrix(x, y, z, w):
    norm = math.sqrt(x * x + y * y + z * z + w * w)

    if norm == 0.0:
        raise ValueError("Zero quaternion")

    x, y, z, w = x / norm, y / norm, z / norm, w / norm

    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


# 위치(m)와 사원수를 4×4 동차 변환으로 만든다.
def pose_matrix(position, orientation):
    result = np.eye(4)
    result[:3, :3] = quaternion_matrix(*orientation)
    result[:3, 3] = position

    return result


# URDF origin(xyz m, 고정축 rpy rad)을 4×4 동차 변환으로 만든다. R = Rz(yaw)·Ry(pitch)·Rx(roll)
def origin_matrix(xyz, rpy):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rotation_x = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    rotation_y = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rotation_z = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])

    result = np.eye(4)
    result[:3, :3] = rotation_z @ rotation_y @ rotation_x
    result[:3, 3] = xyz

    return result


# 회전 행렬 M이 단위 축 axis 둘레로 회전한 각(rad, 부호 포함)을 구한다.
# 순수 축 회전이면 정확하다. 축 밖 성분은 무시하므로 반환값과 함께 잔여 오차를 확인한다.
def angle_about_axis(matrix, axis):
    vee = 0.5 * np.array([
        matrix[2, 1] - matrix[1, 2],
        matrix[0, 2] - matrix[2, 0],
        matrix[1, 0] - matrix[0, 1],
    ])
    cosine = (np.trace(matrix) - 1.0) / 2.0

    return math.atan2(float(np.dot(vee, axis)), cosine)


# 축 회전 행렬(로드리게스 공식)을 만든다. 단위 시험과 잔여 오차 계산에 쓴다.
def axis_rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)

    skew = np.array([
        [0, -axis[2], axis[1]],
        [axis[2], 0, -axis[0]],
        [-axis[1], axis[0], 0],
    ])

    return np.eye(3) + math.sin(angle) * skew + (1 - math.cos(angle)) * (skew @ skew)


# URDF 문자열에서 관절 정보를 읽는다. 반환: 이름 → (종류, 부모, 자식, 원점 4×4, 축, effort 한계)
def read_urdf_joints(urdf_xml):
    robot = ET.fromstring(urdf_xml)
    joints = {}

    for joint in robot.findall("joint"):
        origin = joint.find("origin")
        xyz = [0.0, 0.0, 0.0]
        rpy = [0.0, 0.0, 0.0]

        if origin is not None:
            xyz = [float(value) for value in origin.get("xyz", "0 0 0").split()]
            rpy = [float(value) for value in origin.get("rpy", "0 0 0").split()]

        axis_element = joint.find("axis")
        axis = [1.0, 0.0, 0.0]

        if axis_element is not None:
            axis = [float(value) for value in axis_element.get("xyz").split()]

        limit = joint.find("limit")
        effort = float(limit.get("effort")) if limit is not None and limit.get("effort") else None

        joints[joint.get("name")] = (
            joint.get("type"),
            joint.find("parent").get("link"),
            joint.find("child").get("link"),
            origin_matrix(xyz, rpy),
            np.asarray(axis, dtype=float) / np.linalg.norm(axis),
            effort,
        )

    return joints


# 링크가 Gazebo의 어느 링크 좌표계에 합쳐졌는지와 그 링크 기준 변환을 찾는다.
# 고정 관절을 따라 부모 쪽으로 올라가 Gazebo에 남아 있는 링크를 만날 때까지 변환을 누적한다.
def lumped_frame(link, joints, gazebo_links):
    parent_of = {
        child: (parent, origin)
        for kind, parent, child, origin, _, _ in joints.values()
        if kind == "fixed"
    }

    transform = np.eye(4)

    while link not in gazebo_links:
        if link not in parent_of:
            raise ValueError(f"{link}: no Gazebo link reachable through fixed joints")

        parent, origin = parent_of[link]
        transform = origin @ transform
        link = parent

    return link, transform


# 관절각 재계산에 필요한 관절별 상수(Gazebo 부모 링크, 부모→관절 고정 변환, 자식 링크, 축)를 만든다.
def build_joint_chain(joints, joint_names, gazebo_links):
    chain = {}

    for name in joint_names:
        kind, parent, child, origin, axis, _ = joints[name]

        if kind not in ("revolute", "continuous"):
            raise ValueError(f"{name}: only revolute joints are reconstructed")

        if child not in gazebo_links:
            raise ValueError(f"{name}: child link {child} is not a Gazebo link")

        gazebo_parent, parent_to_joint_parent = lumped_frame(parent, joints, gazebo_links)

        chain[name] = (gazebo_parent, parent_to_joint_parent @ origin, child, axis)

    return chain


# 인접 Gazebo 링크 자세(같은 좌표계 4×4)에서 관절각(rad)과 축 밖 잔여 회전(rad)을 계산한다.
def reconstruct_joints(chain, link_poses):
    angles = {}
    residuals = {}

    for name, (parent, fixed, child, axis) in chain.items():
        relative = np.linalg.inv(link_poses[parent]) @ link_poses[child]
        motion = fixed[:3, :3].T @ relative[:3, :3]

        angle = angle_about_axis(motion, axis)
        remainder = axis_rotation(axis, angle).T @ motion

        angles[name] = angle
        residuals[name] = math.acos(max(-1.0, min(1.0, (np.trace(remainder) - 1.0) / 2.0)))

    return angles, residuals


# 4×4 자세의 Z축과 월드 Z축 사이 각(rad)을 구한다. 차체 기울기 감시에 쓴다.
def tilt_angle(pose):
    return math.acos(max(-1.0, min(1.0, float(pose[2, 2]))))


# (시각, 값) 표본에서 그 뒤로 hold 초 이상 계속 threshold 이하인 첫 시각을 찾는다. 없으면 None
def first_stable_below(samples, threshold, hold):
    start = None

    for stamp, value in samples:
        if value is None or value > threshold:
            start = None
        elif start is None:
            start = stamp

    if start is None or not samples or samples[-1][0] - start < hold:
        return None

    return start


# (시각, 관절각 사전) 이력을 window 초 간격 차분해 시각별 최대 관절 속도(rad/s)로 바꾼다.
def joint_speed_series(history, window):
    series = []
    first = 0

    for index, (stamp, angles) in enumerate(history):
        while first < index and stamp - history[first][0] > window:
            first += 1

        earlier_stamp, earlier = history[first]
        elapsed = stamp - earlier_stamp

        if elapsed < window / 2.0:
            continue

        speed = max(
            abs(math.atan2(math.sin(angles[name] - earlier[name]), math.cos(angles[name] - earlier[name])))
            for name in angles
        ) / elapsed

        series.append((stamp, speed))

    return series


# 4×4 자세 또는 3×3 회전에서 월드 Z축 둘레 yaw(rad, -pi~pi)를 구한다.
def yaw_of(matrix):
    return math.atan2(float(matrix[1, 0]), float(matrix[0, 0]))


# 사원수(x, y, z, w)의 yaw(rad)를 구한다. odom 메시지 방향 해석에 쓴다.
def quaternion_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


# -pi~pi로 감긴 각도 목록을 이어 붙여 누적 각도(rad)로 푼다. 한 표본 사이 회전은 pi 미만이라고 가정한다.
def unwrap_angles(angles):
    result = []

    for angle in angles:
        if not result:
            result.append(angle)
            continue

        step = math.atan2(math.sin(angle - result[-1]), math.cos(angle - result[-1]))
        result.append(result[-1] + step)

    return result


# 값 목록의 평균·표본 표준편차·최소·최대를 돌려준다. 값이 없으면 None
def summarize(values):
    values = [value for value in values if value is not None and math.isfinite(value)]

    if not values:
        return None

    mean = sum(values) / len(values)
    spread = math.sqrt(sum((value - mean) ** 2 for value in values) / (len(values) - 1)) if len(values) > 1 else 0.0

    return {"count": len(values), "mean": mean, "std": spread, "min": min(values), "max": max(values)}


# 시각순 두 표본 목록 [(시각, 값)]을 가장 가까운 시각끼리 짝짓는다. max_gap 초를 넘는 짝은 버린다.
def nearest_pairs(first, second, max_gap):
    pairs = []

    if not second:
        return pairs

    index = 0

    for stamp, value in first:
        while index + 1 < len(second) and abs(second[index + 1][0] - stamp) <= abs(second[index][0] - stamp):
            index += 1

        if abs(second[index][0] - stamp) <= max_gap:
            pairs.append((value, second[index][1]))

    return pairs


# Gazebo 자세 스트림을 받아 로봇 관절각·TCP·추적 물체 자세를 시각별로 기록한다.
class GazeboObserver:
    # 관측 대상과 URDF 상수를 정한다. start()를 불러야 연결된다.
    def __init__(self, robot_id, urdf_xml, joint_names, tool_link, tcp_link, world="ax_test", history_seconds=30.0):
        self.robot_id = robot_id
        self.world = world
        self.joint_names = list(joint_names)
        self.tool_link = tool_link
        self.tcp_link = tcp_link
        self.joints = read_urdf_joints(urdf_xml)
        self.history_seconds = history_seconds

        self.lock = threading.Lock()
        self.link_ids = {}
        self.chain = None
        self.tool_to_tcp = None
        self.tracked_models = set()

        self.latest_links = {}
        self.latest_models = {}
        self.latest_stamp = None
        self.history = deque()
        self.node = None

    # URDF의 관절별 effort 한계(N·m)를 돌려준다.
    def effort_limits(self):
        return {name: self.joints[name][5] for name in self.joint_names}

    # scene/info로 로봇 링크 엔티티 ID를 찾고 자세 토픽을 구독한다.
    def start(self, timeout_seconds=15.0):
        from gz.msgs10.empty_pb2 import Empty
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.msgs10.scene_pb2 import Scene
        from gz.transport13 import Node

        self.node = Node()
        deadline = time.monotonic() + timeout_seconds
        scene = None

        while time.monotonic() < deadline:
            ok, response = self.node.request(f"/world/{self.world}/scene/info", Empty(), Empty, Scene, 3000)

            if ok:
                scene = response
                break

            time.sleep(0.5)

        if scene is None:
            raise RuntimeError("scene/info 응답 없음. GZ_PARTITION과 월드 이름을 확인")

        model = next((item for item in scene.model if item.name == self.robot_id), None)

        if model is None:
            raise RuntimeError(f"{self.robot_id} 모델이 월드에 없음")

        self.link_ids = {link.id: link.name for link in model.link}
        self.chain = build_joint_chain(self.joints, self.joint_names, set(self.link_ids.values()))

        # tcp_link가 없으면 공구 링크 원점을 TCP 자리에 둔다(관절각만 필요한 측정용).
        if self.tcp_link is None:
            self.tool_to_tcp = np.eye(4)
        else:
            tool_root, tool_to_tcp = lumped_frame(self.tcp_link, self.joints, set(self.link_ids.values()))

            if tool_root != self.tool_link:
                raise RuntimeError(f"{self.tcp_link}이 {self.tool_link}이 아닌 {tool_root}에 합쳐짐")

            self.tool_to_tcp = tool_to_tcp

        if not self.node.subscribe(Pose_V, f"/world/{self.world}/dynamic_pose/info", self.on_pose):
            raise RuntimeError("dynamic_pose 구독 실패")

    # 물체 모델(월드 자세)을 기록 대상에 넣는다.
    def track_model(self, name):
        with self.lock:
            self.tracked_models.add(name)

    # 물체 모델을 기록 대상에서 뺀다.
    def untrack_model(self, name):
        with self.lock:
            self.tracked_models.discard(name)
            self.latest_models.pop(name, None)

    # 자세 메시지를 받아 최신값을 갱신하고, 로봇 링크가 모두 있으면 이력 한 건을 남긴다.
    def on_pose(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nsec * 1e-9

        with self.lock:
            for pose in message.pose:
                matrix = pose_matrix(
                    (pose.position.x, pose.position.y, pose.position.z),
                    (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                )

                if pose.id in self.link_ids:
                    self.latest_links[self.link_ids[pose.id]] = matrix
                elif pose.name == self.robot_id or pose.name in self.tracked_models:
                    self.latest_models[pose.name] = matrix

            if self.robot_id not in self.latest_models:
                return

            if not all(link in self.latest_links for parent, _, child, _ in self.chain.values() for link in (parent, child)):
                return

            if self.tool_link not in self.latest_links:
                return

            # 시뮬레이션 리셋으로 시각이 되돌아가면 이력을 비운다.
            if self.latest_stamp is not None and stamp < self.latest_stamp:
                self.history.clear()

            self.latest_stamp = stamp
            self.history.append(self.compute(stamp))

            while self.history and stamp - self.history[0]["stamp"] > self.history_seconds:
                self.history.popleft()

    # 현재 최신값으로 관측 한 건을 계산한다. lock을 잡은 상태에서 부른다.
    def compute(self, stamp):
        angles, residuals = reconstruct_joints(self.chain, self.latest_links)

        base_world = self.latest_models[self.robot_id]
        tcp_world = base_world @ self.latest_links[self.tool_link] @ self.tool_to_tcp

        return {
            "stamp": stamp,
            "joints": angles,
            "residual": max(residuals.values()),
            "tcp_world": tcp_world,
            "base_world": base_world,
            "models": {name: pose.copy() for name, pose in self.latest_models.items() if name in self.tracked_models},
        }

    # 가장 최근 관측 한 건을 돌려준다. 아직 없으면 None
    def latest(self):
        with self.lock:
            return self.history[-1] if self.history else None

    # since 시각(시뮬레이션 s) 이후 관측 목록을 돌려준다.
    def since(self, since):
        with self.lock:
            return [item for item in self.history if item["stamp"] >= since]

    # 관측이 stamp(시뮬레이션 s)까지 들어올 때까지 기다린다. 처리 지연이 있어도 구간을 다 채우기 위해서다.
    def wait_until(self, stamp, timeout_seconds=30.0):
        deadline = time.monotonic() + timeout_seconds

        while time.monotonic() < deadline:
            latest = self.latest()

            if latest is not None and latest["stamp"] >= stamp:
                return True

            time.sleep(0.02)

        return False

    # 구독을 해제한다. 프로세스 종료 중 Gazebo 전송 스레드가 Python 콜백을 부르지 않게 먼저 부른다.
    def stop(self):
        if self.node is not None:
            self.node.unsubscribe(f"/world/{self.world}/dynamic_pose/info")
            self.node = None

    # 관측이 들어올 때까지 벽시계 기준으로 기다린다.
    def wait_ready(self, timeout_seconds=20.0):
        deadline = time.monotonic() + timeout_seconds

        while time.monotonic() < deadline:
            if self.latest() is not None:
                return True

            time.sleep(0.05)

        return False


# 지정한 모델들의 Gazebo 월드 자세를 시각별로 기록한다. 팔이 없는 로봇(robot_2)의 차체 관측에도 쓴다.
class ModelPoseTracker:
    # 기록할 모델 이름과 월드 이름을 정한다. start()를 불러야 연결된다.
    def __init__(self, names, world="ax_test", history_seconds=60.0):
        self.names = set(names)
        self.world = world
        self.history_seconds = history_seconds
        self.lock = threading.Lock()
        self.history = {name: deque() for name in self.names}
        self.node = None

    # 자세 토픽을 구독한다.
    def start(self):
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node

        self.node = Node()

        if not self.node.subscribe(Pose_V, f"/world/{self.world}/dynamic_pose/info", self.on_pose):
            raise RuntimeError("dynamic_pose 구독 실패")

    # 대상 모델 자세를 측정 시각과 함께 저장한다.
    def on_pose(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nsec * 1e-9

        with self.lock:
            for pose in message.pose:
                if pose.name not in self.names:
                    continue

                history = self.history[pose.name]

                # 시뮬레이션 리셋으로 시각이 되돌아가면 이력을 비운다.
                if history and stamp < history[-1][0]:
                    history.clear()

                history.append((stamp, pose_matrix(
                    (pose.position.x, pose.position.y, pose.position.z),
                    (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                )))

                while history and stamp - history[0][0] > self.history_seconds:
                    history.popleft()

    # since 시각 이후 (시각, 4×4 자세) 목록
    def since(self, name, since):
        with self.lock:
            return [item for item in self.history[name] if item[0] >= since]

    # 가장 최근 (시각, 4×4 자세). 없으면 None
    def latest(self, name):
        with self.lock:
            return self.history[name][-1] if self.history[name] else None

    # stamp(시뮬레이션 s)까지 관측이 들어올 때까지 기다린다.
    def wait_until(self, name, stamp, timeout_seconds=30.0):
        deadline = time.monotonic() + timeout_seconds

        while time.monotonic() < deadline:
            latest = self.latest(name)

            if latest is not None and latest[0] >= stamp:
                return True

            time.sleep(0.02)

        return False

    # 구독을 해제한다. 프로세스 종료 전에 부른다.
    def stop(self):
        if self.node is not None:
            self.node.unsubscribe(f"/world/{self.world}/dynamic_pose/info")
            self.node = None


# 물체 자세를 TCP 좌표계로 옮긴다. 부착 중 미끄러짐(상대 이동) 계산에 쓴다.
def in_tcp_frame(tcp_world, object_world):
    return np.linalg.inv(tcp_world) @ object_world


# robot_state_publisher의 robot_description(transient local)을 받아 URDF 문자열을 돌려준다.
# 노드를 별도 executor 스레드가 돌리지 않으면 spin에 rclpy.spin_once 같은 함수를 넘긴다.
def fetch_robot_description(node, robot_id, timeout_seconds=15.0, spin=None):
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    received = {}
    event = threading.Event()

    # 첫 메시지만 보관한다.
    def on_description(message):
        received.setdefault("urdf", message.data)
        event.set()

    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscription = node.create_subscription(String, f"/ax/{robot_id}/robot_description", on_description, qos)

    deadline = time.monotonic() + timeout_seconds

    while not event.is_set() and time.monotonic() < deadline:
        if spin is None:
            time.sleep(0.05)
        else:
            spin()

    node.destroy_subscription(subscription)

    if "urdf" not in received:
        raise RuntimeError(f"/ax/{robot_id}/robot_description 수신 실패")

    return received["urdf"]
