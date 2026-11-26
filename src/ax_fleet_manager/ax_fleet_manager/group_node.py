"""D2 개발 상태 출력 노드: 개발 이름공간에 GroupTaskList만 발행한다.

- Action·속도·팔·공구·안전 해제를 호출·발행하지 않는다. 구독·Action client를 만들지 않는다.
- 운영 모드(OPERATIONAL)는 시작을 거부한다. /ax/fleet/tasks 등 운영 Topic에는 발행하지 않으며,
  remap으로 출력 이름이 바뀌어도 해석된 이름을 다시 검사해 운영 이름이면 종료한다.
- STATE QoS(reliable·transient_local·keep_last 1). 내용이 바뀌면 바로 revision+1로 발행하고,
  1 Hz로 같은 revision을 새 stamp로 재발행한다(소비자 만료 3.0 s, 계약 D-05).
- source_id는 d2_dev_mock:<fixture_id 또는 store>로 출처를 남긴다. 혼합 모드 작업이 하나라도 있으면
  d2_dev_mixed:<label>이며 파라미터 mode가 아니라 작업 상태의 모드로 정한다. source_session_id는 시작마다
  새로 만든다.
"""
import os
import sys
import uuid

import rclpy
from ax_interfaces.msg import GroupTaskList, GroupTaskStatus, StateMeta
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from . import group_model as gm
from .node_config import (
    DEFAULT_OUTPUT_TOPIC, ConfigError, load_fixture, load_store_states, node_mode,
    validate_output_topic,
)
from .snapshot import source_prefix, task_list_fields
from .task_store import StoreError

SCHEMA_VERSION = 1
STATE_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST, depth=1)


class GroupTaskDevPublisher(Node):
    """fixture·원장 상태를 개발 Topic에 GroupTaskList로 내는 노드."""

    # 매개변수 검사 → 입력 읽기 → 발행자 생성 → 해석된 Topic 재검사 순서로 시작한다
    def __init__(self):
        super().__init__('d2_group_dev_publisher')
        mode = self.declare_parameter('mode', gm.MODE_DEVELOPMENT_MOCK).value
        live_kinds = self.declare_parameter(
            'live_kinds', [], ParameterDescriptor(dynamic_typing=True)).value
        topic = self.declare_parameter('output_topic', DEFAULT_OUTPUT_TOPIC).value
        self.fixture_file = self.declare_parameter('fixture_file', '').value
        self.store_path = self.declare_parameter('store_path', '').value
        period = self.declare_parameter('publish_period_s', 1.0).value
        poll = self.declare_parameter('store_poll_period_s', 0.2).value

        node_mode(mode, live_kinds)
        validate_output_topic(topic)
        if not self.fixture_file and not self.store_path:
            raise ConfigError('fixture_file 또는 store_path가 필요하다')
        if not (0.1 <= period <= 1.0 and 0.05 <= poll <= period):
            raise ConfigError('publish_period_s는 0.1~1.0, store_poll_period_s는 0.05~발행 주기')

        labels = []
        self.fixture_states = []
        if self.fixture_file:
            fixture_id, self.fixture_states = load_fixture(self.fixture_file)
            labels.append(fixture_id)
        if self.store_path:
            labels.append('store')

        self.label = '+'.join(labels)
        self.source_id = source_prefix(self.fixture_states) + self.label
        self.session_id = uuid.uuid4().hex
        self.revision = 0
        self.last = None
        self.valid = True
        self.reason = ''
        self.store_signature = None
        self.store_states = []
        self._reload_store()

        # remap을 적용한 이름을 발행자 생성 전에 검사한다(운영 Topic에 endpoint를 만들지 않음)
        validate_output_topic(self.resolve_topic_name(topic))
        self.publisher = self.create_publisher(GroupTaskList, topic, STATE_QOS)
        validate_output_topic(self.publisher.topic_name)

        self._publish()
        self.create_timer(period, self._publish)
        if self.store_path:
            self.create_timer(poll, self._poll_store)

        self.get_logger().info(
            f'D2 개발 상태 출력 시작: topic={self.publisher.topic_name} source={self.source_id} '
            f'session={self.session_id} (운영 Topic 발행·실제 구동 호출 없음)')

    # 원장 파일 변화(크기·수정 시각) 확인 후 다시 읽는다. 실패하면 목록을 무효로 낸다
    def _reload_store(self):
        if not self.store_path:
            return False

        try:
            stat = os.stat(self.store_path)
            signature = (stat.st_size, stat.st_mtime_ns)
            wal = self.store_path + '-wal'
            if os.path.exists(wal):
                wal_stat = os.stat(wal)
                signature += (wal_stat.st_size, wal_stat.st_mtime_ns)
        except OSError:
            signature = None

        if signature == self.store_signature and signature is not None:
            return False

        self.store_signature = signature
        try:
            self.store_states = load_store_states(self.store_path)
            self.valid, self.reason = True, ''
        except (StoreError, OSError, ValueError) as error:
            self.valid, self.reason = False, 'D2_STORE_UNREADABLE'
            self.get_logger().error(f'원장 읽기 실패: {error}')

        return True

    # 원장 변화가 있으면 바로 발행한다
    def _poll_store(self):
        if self._reload_store():
            self._publish()

    # 현재 목록 발행. 내용·유효성이 바뀌면 revision을 올린다
    def _publish(self):
        states = self.fixture_states + self.store_states
        fields = task_list_fields(states)
        self.source_id = source_prefix(states) + self.label
        current = (fields, self.valid, self.reason, self.source_id)
        if current != self.last:
            self.revision += 1
            self.last = current

        msg = GroupTaskList()
        msg.meta = StateMeta()
        msg.meta.header.stamp = self.get_clock().now().to_msg()
        msg.meta.schema_version = SCHEMA_VERSION
        msg.meta.source_id = self.source_id
        msg.meta.source_session_id = self.session_id
        msg.meta.revision = self.revision
        msg.meta.valid = self.valid
        msg.meta.validity_reason = self.reason
        msg.tasks = [GroupTaskStatus(**item) for item in fields]
        self.publisher.publish(msg)


# 진입점. 설정 거부는 종료 코드 2
def main(args=None):
    rclpy.init(args=args)
    node = None
    code = 0

    try:
        node = GroupTaskDevPublisher()
        rclpy.spin(node)
    except (ConfigError, gm.SpecError, StoreError, ValueError) as error:
        print(f'[d2_group_dev_publisher] 시작 거부: {error}', file=sys.stderr)
        code = 2
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return code


if __name__ == '__main__':
    sys.exit(main())
