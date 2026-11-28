"""D2 W5 개발 입고 노드: 개발 이름공간에서 입고 관측을 받고 작업 등록·입고 사건·재고를 낸다.

- 입력: /ax/dev/d2/receipt/observations(ReceiptObservation), /ax/dev/d2/tasks/register(RegisterTask).
- 출력: /ax/dev/d2/receipt/events(ReceiptEvent, EVENT),
  /ax/dev/d2/inventory(InventorySnapshot, STATE·변경 시 + 1 Hz).
  운영 이름(/ax/fleet·/ax/robot_)은 remap 결과까지 검사해 거부한다.
- 작업은 QUEUED로 등록만 한다. 배정·실행 시작·Action·속도·팔·공구·안전 해제를 호출하지 않는다
  (포트 없음). 작업 목록은 기존 group_dev_publisher가 같은 원장을 읽기 전용으로 읽어 낸다.
- 지점·입고 규칙은 개발 설정 파일(config/dev_intake.json)이며 실환경 값이 아니다.
- 재고 source_session_id는 시작마다 새로 만들고, 발행 직전 ROS 시각이 직전까지 낸 가장 큰 stamp보다
  CLOCK_REGRESSION_TOLERANCE_NS 넘게 작으면(use_sim_time에서 sim 시간 리셋 등) 새로 발급한다(revision은 원장 값
  그대로, 시스템 시계 역행은 대상 아님).
"""
import json
import math
import sys
import time
import uuid
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from ax_interfaces.msg import (
    InventoryEntry, InventorySnapshot, ReceiptEvent, ReceiptObservation, RegisteredLocation,
    RobotStopPose, StateMeta,
)
from ax_interfaces.srv import RegisterTask
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from . import group_model as gm
from . import receipt as rc
from .intake import ReceiptIntake
from .inventory import inventory_snapshot_fields, parse_catalog
from .node_config import ConfigError, node_mode, validate_output_topic
from .runtime import GroupRuntime
from .task_store import StoreError, TaskStore

SCHEMA_VERSION = 1
SOURCE_ID = 'd2_dev_intake'
# ROS 시각 역행 판정 허용 오차(ns). 개발용 잠정값(미정 수치): 관제 미래 stamp 허용 오차 0.1 s와 맞췄다.
# 같은 값 반복(/clock 정지)은 역행이 아니고, 이보다 작은 sim 역행은 세션을 바꾸지 않는다(sim 시간에만 적용)
CLOCK_REGRESSION_TOLERANCE_NS = 100_000_000
STATE_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST, depth=1)
EVENT_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE,
                       history=HistoryPolicy.KEEP_LAST, depth=50)
DEFAULT_NAMES = {
    'observation_topic': '/ax/dev/d2/receipt/observations',
    'register_service': '/ax/dev/d2/tasks/register',
    'event_topic': '/ax/dev/d2/receipt/events',
    'inventory_topic': '/ax/dev/d2/inventory',
}
RECEIPT_KEYS = ('receipt_zones', 'destination_rules', 'min_identity_confidence',
                'max_observation_age_s')
MAX_INTAKE_BYTES = 256 * 1024


# 개발 설정 파일(입고 규칙 + 지점 카탈로그) 읽기
def load_intake_file(path):
    data_path = Path(path)
    if data_path.stat().st_size > MAX_INTAKE_BYTES:
        raise ConfigError(f'설정 크기 상한 초과: {MAX_INTAKE_BYTES} bytes')

    try:
        data = json.loads(data_path.read_text(encoding='utf-8'))
    except (OSError, ValueError) as error:
        raise ConfigError(f'설정 읽기 실패: {error}') from error

    if not isinstance(data, dict) or set(data) != {'receipt', 'catalog'}:
        raise ConfigError('설정 키는 정확히 receipt·catalog')

    receipt = data['receipt']
    if not isinstance(receipt, dict) or set(receipt) != set(RECEIPT_KEYS):
        raise ConfigError(f'receipt 키는 정확히 {RECEIPT_KEYS}')

    config = rc.ReceiptConfig(
        receipt_zones=tuple(receipt['receipt_zones']),
        destination_rules=dict(receipt['destination_rules']),
        min_identity_confidence=float(receipt['min_identity_confidence']),
        max_observation_age_s=float(receipt['max_observation_age_s']))

    return config, parse_catalog(data['catalog'])


# ROS 시각(초)
def _stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


# 관측 메시지 → 접수 사전(pose는 D 판정에 쓰지 않음). float32 신뢰도는 소수 6자리로 맞춘다
def observation_dict(msg):
    return {
        'observation_id': msg.observation_id,
        'zone_id': msg.zone_id,
        'object_id': msg.object_id,
        'object_class': msg.object_class,
        'identity_confidence': round(float(msg.identity_confidence), 6),
        'source': int(msg.source),
        'source_id': msg.meta.source_id,
        'valid': bool(msg.meta.valid),
        'stamp_sim_s': _stamp_s(msg.meta.header.stamp),
    }


# map 기준 PoseStamped(yaw만)
def _pose(fields, stamp):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = stamp
    pose.pose.position.x = fields['x']
    pose.pose.position.y = fields['y']
    pose.pose.orientation.z = math.sin(fields['yaw'] / 2.0)
    pose.pose.orientation.w = math.cos(fields['yaw'] / 2.0)

    return pose


class ReceiptIntakeDevNode(Node):
    """개발 입고 노드. 원장은 쓰기 가능하게 연다(작업 목록 출력 노드는 읽기 전용)."""

    # 매개변수·이름 검사 → 원장·접수기 준비 → 구독·서비스·발행자 생성 → 해석된 이름 재검사
    def __init__(self):
        super().__init__('d2_dev_intake')
        mode = self.declare_parameter('mode', gm.MODE_DEVELOPMENT_MOCK).value
        live_kinds = self.declare_parameter(
            'live_kinds', [], ParameterDescriptor(dynamic_typing=True)).value
        store_path = self.declare_parameter('store_path', '').value
        default_file = str(Path(get_package_share_directory('ax_fleet_manager'))
                           / 'config' / 'dev_intake.json')
        intake_file = self.declare_parameter('intake_file', default_file).value
        period = self.declare_parameter('publish_period_s', 1.0).value
        names = {k: self.declare_parameter(k, v).value for k, v in DEFAULT_NAMES.items()}

        live_kinds = node_mode(mode, live_kinds)
        for name in names.values():
            validate_output_topic(name)
        if not store_path:
            raise ConfigError('store_path가 필요하다')
        if not 0.1 <= period <= 1.0:
            raise ConfigError('publish_period_s는 0.1~1.0')

        config, self.catalog = load_intake_file(intake_file)
        self.store = TaskStore(store_path)
        self.runtime = GroupRuntime(self.store, {}, self._intake_clock,
                                    map_revision=self.catalog.map_revision,
                                    locations=self.catalog.revisions(), mode=mode,
                                    live_kinds=live_kinds)
        self.intake = ReceiptIntake(self.runtime, config, self.catalog)
        self.session_id = uuid.uuid4().hex
        # 이 세션에서 낸 가장 큰 재고 stamp(ns)와 ROS 시각 역행으로 세션을 새로 발급한 횟수
        self.last_stamp_ns = None
        self.session_renewals = 0

        # remap을 적용한 이름을 endpoint 생성 전에 검사한다
        resolved = {k: (self.resolve_service_name(v) if k == 'register_service'
                        else self.resolve_topic_name(v)) for k, v in names.items()}
        for name in resolved.values():
            validate_output_topic(name)

        self.events = self.create_publisher(ReceiptEvent, names['event_topic'], EVENT_QOS)
        self.inventory = self.create_publisher(InventorySnapshot, names['inventory_topic'],
                                               STATE_QOS)
        self.create_subscription(ReceiptObservation, names['observation_topic'],
                                 self._on_observation, EVENT_QOS)
        self.create_service(RegisterTask, names['register_service'], self._on_register)

        self.last_revision = None
        # 시계가 유효하지 않아(ROS 시각 0, use_sim_time에서 /clock 수신 전) 건너뛴 재고 발행 횟수
        self.skipped_invalid_clock = 0
        # 재고 1 Hz 재발행은 STEADY 시계로 잰다(use_sim_time이어도, 시계 역행에 흔들리지 않게). stamp는 ROS 시간
        self._publish_inventory()
        self.create_timer(period, self._publish_inventory,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))

        self.get_logger().info(
            f'D2 개발 입고 노드 시작: {resolved} session={self.session_id} '
            '(운영 Topic·실제 구동 호출 없음, 등록은 QUEUED만)')

    # 접수기 시계: CLOCK_BOOTTIME과 ROS 시각(Node._clock 속성과 이름이 겹치지 않게 한다)
    def _intake_clock(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        return gm.ClockSnapshot(time.clock_gettime(time.CLOCK_BOOTTIME), now, True)

    # 입고 관측 접수 → 입고 사건 발행 → 재고 바뀌면 바로 발행
    def _on_observation(self, msg):
        result = self.intake.observe(observation_dict(msg))
        self.get_logger().info(
            f'입고 관측 {msg.observation_id}: {result.decision} {result.reason_code}')

        if result.receipt_event_id:
            event = self.store.receipt_event(result.receipt_event_id)
            if event is not None:
                out = ReceiptEvent(**rc.receipt_event_fields(event))
                self.events.publish(out)

        self._publish_inventory()

    # 작업 등록 서비스(수락은 QUEUED 생성이며 운반 완료가 아니다)
    def _on_register(self, request, response):
        reply = self.intake.register_task({
            'request_id': request.request_id,
            'receipt_event_id': request.receipt_event_id,
            'object_id': request.object_id,
            'source_location_id': request.source_location_id,
            'destination_location_id': request.destination_location_id,
        })
        response.accepted = reply['accepted']
        response.task_id = reply['task_id']
        response.reason_code = reply['reason_code']
        self._publish_inventory()

        return response

    # 재고 snapshot 발행. revision은 원장 재고 revision(영속), 1 Hz로 같은 revision 재발행.
    # ROS 시각이 0(sim /clock 수신 전)이면 stamp 0 메시지를 내지 않고 건너뛴다(관측·등록 뒤 발행도 같음)
    def _publish_inventory(self):
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            self.skipped_invalid_clock += 1
            self.get_logger().debug('ROS 시각 0(/clock 수신 전): 재고 발행 건너뜀', once=True)
            return

        self._renew_session_on_regression(now.nanoseconds)

        valid, reason = True, ''
        try:
            fields = inventory_snapshot_fields(self.store, self.catalog)
        except StoreError as error:
            self.get_logger().error(f'재고 읽기 실패: {error}')
            fields = {'revision': self.last_revision or 0, 'map_revision': 0,
                      'locations': [], 'items': []}
            valid, reason = False, 'D2_STORE_UNREADABLE'

        if self.runtime.fault:
            valid, reason = False, 'D2_STORE_FAULT'

        stamp = now.to_msg()
        msg = InventorySnapshot()
        msg.meta = StateMeta()
        msg.meta.header.stamp = stamp
        msg.meta.schema_version = SCHEMA_VERSION
        msg.meta.source_id = SOURCE_ID
        msg.meta.source_session_id = self.session_id
        msg.meta.revision = fields['revision']
        msg.meta.valid = valid
        msg.meta.validity_reason = reason
        msg.map_revision = fields['map_revision']
        msg.locations = [self._location(item, stamp) for item in fields['locations']]
        msg.items = [InventoryEntry(**item) for item in fields['items']]
        self.last_revision = fields['revision']
        self.inventory.publish(msg)
        self.last_stamp_ns = max(self.last_stamp_ns or 0, now.nanoseconds)

    # sim 시간(/clock)이 쓰이는 중이고, 발행 직전 ROS 시각이 직전 최대 stamp보다 허용 오차 넘게 작으면
    # source_session_id를 새로 발급한다. 소비자는 같은 세션에서 stamp가 줄어든 새 revision을 버리므로(계약 2.3),
    # sim 리셋 뒤 새 세션으로 비교를 다시 시작하게 한다. 시스템 시계(WSL 등에서 수 초 역행)는 대상이 아니다:
    # 역행은 기존 소비자 규칙(같은 revision stamp 감소 허용)으로 처리한다. 판정은 발행마다 시계의 현재 상태로 한다
    def _renew_session_on_regression(self, now_ns):
        if self.last_stamp_ns is None or not self.get_clock().ros_time_is_active:
            return

        if now_ns >= self.last_stamp_ns - CLOCK_REGRESSION_TOLERANCE_NS:
            return

        previous, previous_stamp_ns = self.session_id, self.last_stamp_ns
        self.session_id = uuid.uuid4().hex
        self.session_renewals += 1
        self.last_stamp_ns = None

        self.get_logger().info(
            f'ROS 시각 역행({previous_stamp_ns * 1e-9:.3f} s → {now_ns * 1e-9:.3f} s): '
            f'재고 source_session_id 재발급 {previous} → {self.session_id} '
            f'(재발급 {self.session_renewals}회, revision은 원장 값 유지)')

    # 지점 필드 → RegisteredLocation
    def _location(self, item, stamp):
        out = RegisteredLocation()
        for key in ('location_id', 'location_revision', 'map_revision', 'purpose', 'zone_id',
                    'capacity', 'allowed_object_classes', 'max_load_kg', 'state',
                    'reservation_id', 'wait_location_id'):
            setattr(out, key, item[key])
        out.pose = _pose(item['pose'], stamp)
        out.approach_pose = _pose(item['pose'], stamp)
        out.stop_poses = [RobotStopPose(robot_role=role, pose=_pose(pose, stamp))
                          for role, pose in sorted(item['stop_poses'].items())]

        return out

    # 원장 닫기
    def close(self):
        self.store.close()


# 진입점. 설정 거부는 종료 코드 2
def main(args=None):
    rclpy.init(args=args)
    node = None
    code = 0

    try:
        node = ReceiptIntakeDevNode()
        rclpy.spin(node)
    except (ConfigError, gm.SpecError, StoreError, ValueError, OSError) as error:
        print(f'[d2_dev_intake] 시작 거부: {error}', file=sys.stderr)
        code = 2
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return code


if __name__ == '__main__':
    sys.exit(main())
