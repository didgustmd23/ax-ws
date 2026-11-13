"""계산된 후보를 읽기 전용으로 게시하며 상대 입력이나 팔 명령을 임의로 만들지 않습니다."""
from dataclasses import asdict
import hashlib
import json
import math
import sqlite3
import threading
import uuid

from ax_interfaces.msg import CandidateSet, CandidateSetList, GraspCandidate
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time


class CandidatePublisher(Node):
    """내부 연결부가 전달한 후보를 검사해 공통 후보 상태로 게시합니다."""

    # 후보를 읽기 전용으로 게시합니다. C2 입력 Topic을 임의로 만들거나 팔 명령을 보내지 않습니다.

    def __init__(self):
        # 역할: 설정과 내부 상태를 준비합니다.
        super().__init__('candidate_publisher')
        self.robot_id = self.declare_parameter('robot_id', 'robot_1').value
        if self.robot_id not in ('robot_1', 'robot_2', 'robot_3', 'robot_4'):
            raise ValueError('known robot_id required')
        self.session = uuid.uuid4().hex
        self.revision = 1
        self.plans = {}
        self._plan_lock = threading.RLock()
        self._ensure_history()
        self.previous_content = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(
            CandidateSetList, f'/ax/{self.robot_id}/manipulation/candidates', qos)
        self.create_timer(
            1.0, self.publish_snapshot, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def publish_plan(self, plan):
        """동시 호출을 잠그고 내부 연결부에서 전달한 후보를 접수합니다."""
        # 버전 확인과 저장을 묶어, 동시에 도착한 결과가 최신 후보를 덮어쓰지 않게 합니다.
        with self._plan_lock:
            self._accept_plan(plan)

    def _accept_plan(self, plan):
        # 실제 연결부가 계산한 Plan만 받습니다. 다른 로봇의 후보와 관측 시각이 잘못된 자료는 거절합니다.
        # 역할: 로봇과 버전 및 관측 시각을 검사해 이전 후보의 덮어쓰기를 막습니다.
        if plan.robot_id != self.robot_id:
            raise ValueError('plan belongs to another robot')
        if not math.isfinite(plan.stamp_s) or plan.stamp_s <= 0 or not plan.frame_id:
            raise ValueError('finite positive observation stamp and frame required')
        self._ensure_history()
        previous = self._plan_history.execute(
            'SELECT scene, constraints, stamp, digest FROM versions WHERE object_id=?',
            (plan.object_id,)).fetchone()
        digest = hashlib.sha256(json.dumps(
            # 버전 번호가 같아도 후보 내용이 바뀌었는지 확인할 내용 지문입니다.
            asdict(plan), sort_keys=True, allow_nan=False).encode()).hexdigest()
        version = (plan.scene_revision, plan.constraint_version, plan.stamp_s)
        # 활성 후보를 지워도 물체별 이력은 세션 동안 임시 원장에 남깁니다.
        if previous is not None:
            # 도착 순서가 최신 순서는 아닙니다. 세 기준 중 하나라도 과거로 돌아가면 거절합니다.
            if any(incoming < current for incoming, current in zip(version, previous[:3])):
                raise ValueError(
                    'older scene, constraints or observation cannot replace current plan')
            if version == previous[:3]:
                if digest != previous[3]:
                    raise ValueError('same observation versions contain different candidate data')
                # 이미 회수한 결과도 같은 관측 재수신으로 부활시키지 않습니다.
                self.publish_snapshot()
                return
        now = self.get_clock().now().nanoseconds * 1e-9
        wall = self._steady_seconds()
        # 타이머가 아직 실행되지 않았어도 새 물체의 공간 검사 전에 만료 후보를 정리합니다.
        self._prune_expired(now, wall)
        if plan.object_id not in self.plans and len(self.plans) >= 8:
            # 여기까지 남은 후보는 미만료입니다. 새 물체를 위해 유효한 다른 후보를 밀어내지 않습니다.
            raise ValueError('candidate snapshot limit (8 objects) reached')
        self._plan_history.execute(
            'INSERT INTO versions VALUES (?, ?, ?, ?, ?) ON CONFLICT(object_id) DO UPDATE '
            # 공간 부족으로 거절된 요청은 여기까지 오지 않아 최신 이력을 잘못 갱신하지 않습니다.
            'SET scene=excluded.scene, constraints=excluded.constraints, '
            'stamp=excluded.stamp, digest=excluded.digest',
            (plan.object_id, *version, digest))
        self.plans[plan.object_id] = (plan, now, wall)
        self.publish_snapshot()

    def _ensure_history(self):
        """물체별 최신 버전과 지문만 임시 디스크에 보관해 활성 후보 공간과 분리합니다."""
        if hasattr(self, '_plan_history'):
            return
        self._plan_history = sqlite3.connect('', check_same_thread=False, isolation_level=None)
        # 후보 본문 대신 작은 버전 이력만 보관합니다. 원장 종료 시 임시 파일도 정리됩니다.
        self._plan_history.execute('PRAGMA cache_size=-256')
        self._plan_history.execute(
            'CREATE TABLE versions (object_id TEXT PRIMARY KEY, scene INTEGER, '
            'constraints INTEGER, stamp REAL, digest TEXT)')

    def _expired(self, record, now, wall):
        """관측 시각과 최초 수신 벽시계 중 어느 기한이라도 끝나면 만료로 판정합니다."""
        plan, received_sim, received_wall = record
        lifetime = min(3.0, plan.valid_for_s)
        # 관측부터 이미 지난 시간은 빼고 계산해 늦게 도착한 후보에 새 유효기간을 주지 않습니다.
        return (now <= 0 or now < plan.stamp_s or now - plan.stamp_s >= lifetime
                or wall - received_wall >= min(
                    3.0, plan.valid_for_s - max(0.0, received_sim - plan.stamp_s)))

    def _prune_expired(self, now, wall):
        """활성 후보만 회수하고 과거 버전 재수신을 막는 이력은 지우지 않습니다."""
        for object_id, record in list(self.plans.items()):
            # 이 사전에서만 삭제하며 별도 버전 원장은 유지해 옛 결과의 재등록을 막습니다.
            if self._expired(record, now, wall):
                del self.plans[object_id]

    def _steady_seconds(self):
        # 역할: 후보의 실제 경과 시간을 재기 위한 벽시계 값을 반환합니다.
        import time
        return time.clock_gettime(time.CLOCK_BOOTTIME)

    def publish_snapshot(self):
        # 주기 발행도 같은 잠금을 사용해 후보 교체 중간의 상태를 읽지 않습니다.
        # 역할: 후보 교체와 겹치지 않도록 잠근 뒤 상태를 게시합니다.
        with self._plan_lock:
            self._publish_snapshot()

    def _publish_snapshot(self):
        # 재발행으로 오래된 후보를 새 후보처럼 만들지 않도록 sim 시각과 벽시계 기한을 함께 봅니다.
        # 역할: 후보의 기한을 검사하고 유효 후보 또는 무효 사유를 게시합니다.
        now = self.get_clock().now().nanoseconds * 1e-9
        message = CandidateSetList()
        message.robot_id = self.robot_id
        valid_sets = []
        for object_id in sorted(self.plans):
            plan, received_sim, received_wall = self.plans[object_id]
            item = CandidateSet()
            item.robot_id, item.object_id = self.robot_id, object_id
            item.candidate_set_id = plan.candidate_set_id
            item.scene_revision = plan.scene_revision
            item.constraint_version = plan.constraint_version
            alive = plan.valid and not self._expired(
                (plan, received_sim, received_wall), now, self._steady_seconds())
            item.meta.header.stamp = Time(nanoseconds=int(max(0.0, plan.stamp_s) * 1e9)).to_msg()
            item.meta.header.frame_id = plan.frame_id
            item.meta.schema_version = 1
            item.meta.source_id = 'c3_candidate_publisher'
            item.meta.source_session_id = self.session
            item.meta.valid = bool(alive)
            item.meta.validity_reason = '' if alive else (plan.reason or 'CANDIDATE_STALE')
            if alive:
                # 유효한 후보만 공통 메시지로 옮깁니다. 만료되면 무효 사유를 남기고 후보 목록은 비웁니다.
                for candidate in plan.candidates:
                    entry = GraspCandidate()
                    entry.candidate_id = candidate.candidate_id
                    entry.tool_target.header = item.meta.header
                    position = entry.tool_target.pose.position
                    position.x, position.y, position.z = candidate.target.position
                    orientation = entry.tool_target.pose.orientation
                    orientation.x, orientation.y, orientation.z, orientation.w = (
                        candidate.target.orientation)
                    entry.rule_score = float(candidate.rule_score)
                    entry.success_probability = -1.0
                    entry.reachable = candidate.reachable
                    entry.collision_free = candidate.collision_free
                    item.candidates.append(entry)
                    item.default_ranking.append(entry.candidate_id)
                valid_sets.append(item.candidate_set_id)
            message.candidate_sets.append(item)
        key = tuple((item.object_id, item.candidate_set_id, item.meta.valid,
                     item.meta.validity_reason) for item in message.candidate_sets)
        if key != self.previous_content:
            # 내용·유효성이 달라질 때만 revision을 올립니다. 주기 재발행은 같은 상태입니다.
            self.revision += 1
            self.previous_content = key
        message.meta.header.stamp = self.get_clock().now().to_msg()
        message.meta.schema_version = 1
        message.meta.source_id = 'c3_candidate_publisher'
        message.meta.source_session_id = self.session
        message.meta.revision = self.revision
        message.meta.valid = bool(valid_sets)
        message.meta.validity_reason = '' if valid_sets else 'NO_VALID_CANDIDATES'
        # 전체 목록의 사유와 물체별 사유는 다를 수 있습니다. 소비자는 양쪽 유효성을 확인해야 합니다.
        for item in message.candidate_sets:
            item.meta.revision = self.revision
        self.publisher.publish(message)
        # 만료 무효 상태를 한 번 게시한 뒤 공간을 회수합니다. 다음 전체 목록에서는 빠집니다.
        self._prune_expired(now, self._steady_seconds())

    def destroy_node(self):
        """게시 작업과 겹치지 않게 임시 이력을 닫고 노드를 정리합니다."""
        with self._plan_lock:
            self._plan_history.close()
        return super().destroy_node()


def main(args=None):
    # 역할: 실행 인자를 받아 노드를 시작하고 종료 시 자원을 정리합니다.
    rclpy.init(args=args)
    node = CandidatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
