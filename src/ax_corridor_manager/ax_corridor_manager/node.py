"""D3 ROS contract adapter; does not command Nav2, motors or safety gates."""
import math
import os
from pathlib import Path
import sqlite3
import uuid

from ament_index_python.packages import get_package_share_directory
from ax_corridor_manager.manager import CorridorManager
from ax_environment.regions import load_regions
from ax_interfaces.msg import ActuationState, CorridorObservationList, CorridorState
from ax_interfaces.msg import CorridorStateList, GroupTaskList
from ax_interfaces.srv import ReleaseCorridor, RequestCorridorEntry
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

STATE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)


class CorridorNode(Node):
    """Publish durable state and serve the existing D3 service types."""

    # 기존 공통 타입으로 B3·A4·D2와 연결합니다. 바퀴나 팔에 직접 명령하지 않습니다.

    def __init__(self):
        super().__init__('corridor_manager')
        self.robots = tuple(self.declare_parameter('robot_ids', ['robot_1', 'robot_2']).value)
        default_regions = (Path(get_package_share_directory('ax_environment'))
                           / 'config/ax_warehouse_regions.json')
        region_path = self.declare_parameter('regions_file', str(default_regions)).value
        regions = load_regions(region_path)
        corridors = [item['id'] for item in regions['regions'] if item['kind'] == 'corridor']
        ros_home = Path(os.environ.get('ROS_HOME', str(Path.home() / '.ros')))
        default_database = str(ros_home / 'ax_corridor_manager/corridors.sqlite3')
        database = self.declare_parameter('database', default_database).value
        self.manager = CorridorManager(
            database, corridors, self.robots, enabled=regions['approved'])
        self.session = uuid.uuid4().hex
        self.sources = {}
        self.retired = {}
        self.io_failed = False
        self.publisher = self.create_publisher(
            CorridorStateList, '/ax/fleet/corridor/state', STATE_QOS)
        self._input_handles = []
        for robot in self.robots:
            self._input_handles.extend([
                self.create_subscription(CorridorObservationList,
                                         f'/ax/{robot}/perception/corridor_observation',
                                         lambda message, key=robot:
                                         self.on_observation(message, key), STATE_QOS),
                self.create_subscription(ActuationState, f'/ax/{robot}/safety/actuation_state',
                                         lambda message, key=robot:
                                         self.on_stop(message, key), STATE_QOS),
            ])
        self._input_handles.append(self.create_subscription(
            GroupTaskList, '/ax/fleet/tasks', self.on_tasks, STATE_QOS))
        self.create_service(
            RequestCorridorEntry, '/ax/fleet/corridor/request_entry', self.on_request)
        self.create_service(ReleaseCorridor, '/ax/fleet/corridor/release', self.on_release)
        # 안전 관련 관측은 sim time 정지 중에도 벽시계 무수신 기한을 넘기면 무효입니다.
        self.create_timer(0.2, self.publish_state, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def remaining(self, meta, ttl):
        # 수신했다는 사실만으로 신선한 정보는 아닙니다. 원본 시각·출처·유효성을 함께 검사합니다.
        now = self.get_clock().now().nanoseconds * 1e-9
        stamp = meta.header.stamp.sec + meta.header.stamp.nanosec * 1e-9
        age = now - stamp
        if (not meta.valid or not meta.source_session_id or meta.revision == 0
                or not math.isfinite(now) or now <= 0 or stamp <= 0 or not 0 <= age < ttl):
            return 0.0
        return ttl - age

    def ordered(self, key, meta):
        session, revision = meta.source_session_id, meta.revision
        if session in self.retired.setdefault(key, set()):
            return False
        previous = self.sources.get(key)
        if previous and session == previous[0] and revision < previous[1]:
            return False
        if previous and previous[0] != session:
            self.retired[key].add(previous[0])
        self.sources[key] = (session, revision)
        return True

    def on_observation(self, message, robot):
        # 목록과 각 통로의 로봇 번호·유효성을 모두 확인한 뒤 점유 판정에 전달합니다.
        if message.robot_id != robot:
            return
        duration = self.remaining(message.meta, 1.0)
        states = {}
        for observation in message.observations:
            lifetime = self.remaining(observation.meta, 1.0)
            if (observation.robot_id != robot or observation.corridor_id in states
                    or observation.meta.source_session_id != message.meta.source_session_id):
                duration = 0.0
            duration = min(duration, lifetime)
            states[observation.corridor_id] = (observation.observation, observation.position_valid)
        if duration > 0 and not self.ordered(('observation', robot), message.meta):
            return
        try:
            self.manager.observe(robot, states, duration)
        except sqlite3.Error:
            self.io_failed = True

    def on_stop(self, message, robot):
        # 정지 요청이 아니라 A4의 STOPPED와 실제 속도 한계 확인을 사용합니다.
        if message.robot_id != robot:
            return
        duration = self.remaining(message.meta, 0.3)
        if duration > 0 and not self.ordered(('stop', robot), message.meta):
            return
        speeds = (message.linear_speed, message.angular_speed, message.max_joint_speed)
        stopped = (message.state == ActuationState.STOPPED
                   and all(math.isfinite(value) and value >= 0 for value in speeds)
                   and speeds[0] <= 0.01 and speeds[1] <= 0.02 and speeds[2] <= 0.01)
        self.manager.stopped(robot, stopped, duration)

    def on_tasks(self, message):
        # 실행 가능한 배정만 사용합니다. 보류·연기·미배정 작업으로 새 통로 허가를 내지 않습니다.
        duration = self.remaining(message.meta, 3.0)
        if duration > 0 and not self.ordered(('tasks', ''), message.meta):
            return
        eligible = set()
        for task in message.tasks:
            if (task.state in (task.ASSIGNED, task.RUNNING) and task.assignment_generation > 0
                    and not task.hold_ids and not task.deferred):
                eligible.update((task.manipulator_robot_id, task.transporter_robot_id))
        self.manager.assign(eligible, duration)

    def _service(self, method, arguments, reply):
        # 장부 오류는 허용으로 처리하지 않습니다. 장애를 고정하고 상태도 무효로 알립니다.
        try:
            if self.io_failed:
                raise sqlite3.OperationalError('ledger fault is latched')
            result = method(*arguments)
            for key, value in result.items():
                setattr(reply, key, value)
        except sqlite3.Error:
            self.io_failed = True
            reply.reason_code = 'CORRIDOR_LEDGER_FAILED'
        self.publish_state()
        return reply

    def on_request(self, request, reply):
        return self._service(self.manager.request,
                             (request.corridor_id, request.robot_id, request.request_id), reply)

    def on_release(self, request, reply):
        return self._service(self.manager.release,
                             (request.corridor_id, request.robot_id,
                              request.grant_id, request.request_id), reply)

    def publish_state(self):
        # 현재 owner/grant와 영속 revision을 게시합니다. 허가 응답만으로 실제 점유를 확정하지 않습니다.
        message = CorridorStateList()
        try:
            revision, rows = self.manager.snapshot()
        except sqlite3.Error:
            self.io_failed = True
            revision, rows = 0, []
        message.meta.header.stamp = self.get_clock().now().to_msg()
        message.meta.schema_version = 1
        message.meta.source_id = 'd3_corridor_manager'
        message.meta.source_session_id = self.session
        message.meta.revision = revision
        now = self.get_clock().now().nanoseconds
        message.meta.valid = self.manager.enabled and not self.io_failed and now > 0
        message.meta.validity_reason = (
            '' if message.meta.valid else
            'CORRIDOR_LEDGER_FAILED' if self.io_failed else
            'REGIONS_NOT_APPROVED' if not self.manager.enabled else 'CLOCK_INVALID')
        for row in rows:
            state = CorridorState()
            state.corridor_id = row['id']
            state.state = row['state']
            state.owner_robot_id = row['owner']
            state.active_grant_id = row['grant_id']
            state.corridor_revision = row['revision']
            message.corridors.append(state)
        self.publisher.publish(message)

    def destroy_node(self):
        self.manager.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CorridorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
