"""Publish C3 read-only snapshots; no invented C2 input topic or arm commands."""
import math
import threading
import uuid

from ax_interfaces.msg import CandidateSet, CandidateSetList, GraspCandidate
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time


class CandidatePublisher(Node):
    """C4/C2 integration calls publish_plan with a validated C3 Plan."""

    # 후보를 읽기 전용으로 게시합니다. C2 입력 Topic을 임의로 만들거나 팔 명령을 보내지 않습니다.

    def __init__(self):
        super().__init__('candidate_publisher')
        self.robot_id = self.declare_parameter('robot_id', 'robot_1').value
        if self.robot_id not in ('robot_1', 'robot_2', 'robot_3', 'robot_4'):
            raise ValueError('known robot_id required')
        self.session = uuid.uuid4().hex
        self.revision = 1
        self.plans = {}
        self._plan_lock = threading.RLock()
        self.previous_content = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(
            CandidateSetList, f'/ax/{self.robot_id}/manipulation/candidates', qos)
        self.create_timer(
            1.0, self.publish_snapshot, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def publish_plan(self, plan):
        """Accept a plan from the internal C2/C4 integration API."""
        # 버전 확인과 저장을 묶어, 동시에 도착한 결과가 최신 후보를 덮어쓰지 않게 합니다.
        with self._plan_lock:
            self._accept_plan(plan)

    def _accept_plan(self, plan):
        # 실제 연결부가 계산한 Plan만 받습니다. 다른 로봇의 후보와 관측 시각이 잘못된 자료는 거절합니다.
        if plan.robot_id != self.robot_id:
            raise ValueError('plan belongs to another robot')
        if not math.isfinite(plan.stamp_s) or plan.stamp_s <= 0 or not plan.frame_id:
            raise ValueError('finite positive observation stamp and frame required')
        previous = self.plans.get(plan.object_id)
        if previous is not None:
            current = previous[0]
            # 도착 순서가 최신 순서는 아닙니다. 세 기준 중 하나라도 과거로 돌아가면 거절합니다.
            if (plan.scene_revision < current.scene_revision
                    or plan.constraint_version < current.constraint_version
                    or plan.stamp_s < current.stamp_s):
                raise ValueError(
                    'older scene, constraints or observation cannot replace current plan')
        if plan.object_id in self.plans and self.plans[plan.object_id][0] == plan:
            # 같은 관측의 재게시로 벽시계 유효기간을 다시 시작하지 않습니다.
            self.publish_snapshot()
            return
        if plan.object_id not in self.plans and len(self.plans) >= 8:
            raise ValueError('candidate snapshot limit (8 objects) reached')
        self.plans[plan.object_id] = (plan, self.get_clock().now().nanoseconds * 1e-9,
                                      self._steady_seconds())
        self.publish_snapshot()

    def _steady_seconds(self):
        import time
        return time.clock_gettime(time.CLOCK_BOOTTIME)

    def publish_snapshot(self):
        # 주기 발행도 같은 잠금을 사용해 후보 교체 중간의 상태를 읽지 않습니다.
        with self._plan_lock:
            self._publish_snapshot()

    def _publish_snapshot(self):
        # 재발행으로 오래된 후보를 새 후보처럼 만들지 않도록 sim 시각과 벽시계 기한을 함께 봅니다.
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
            age = now - plan.stamp_s
            alive = (plan.valid and now > 0 and 0 <= age < min(3.0, plan.valid_for_s)
                     and self._steady_seconds() - received_wall < min(
                         3.0, plan.valid_for_s - max(0.0, received_sim - plan.stamp_s)))
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
        for item in message.candidate_sets:
            item.meta.revision = self.revision
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = CandidatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
