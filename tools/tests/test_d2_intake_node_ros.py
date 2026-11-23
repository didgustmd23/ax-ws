"""D2 W5 개발 입고 노드의 실제 DDS 시험(격리 설치본, 가짜 관측, Gazebo·실제 구동 없음).

확인: 개발 이름공간 입고 관측 → ReceiptEvent·InventorySnapshot, RegisterTask 서비스 멱등·사건당 작업 1개·
용량 거절, 재고 STATE QoS·늦은 구독, 재시작 뒤 재고 revision 유지·관측 재판정 없음, 운영 이름·운영 모드
거부, 그래프상 Action·운영 Topic·cmd_vel 없음, SIGINT 종료와 잔여 프로세스 0.
환경: 검증할 격리 install을 source한 셸. domain은 ros_test_domain으로 80~89에서 빈 곳을 예약한다.
"""
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

import rclpy
from ament_index_python.packages import get_package_prefix
from ax_interfaces.msg import InventoryEntry, InventorySnapshot, ReceiptEvent, ReceiptObservation
from ax_interfaces.msg import RegisteredLocation
from ax_interfaces.srv import RegisterTask
from rclpy.action import get_action_client_names_and_types_by_node
from rclpy.action import get_action_server_names_and_types_by_node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

sys.path.insert(0, str(Path(__file__).resolve().parent))
import ros_test_domain  # noqa: E402

NODE_NAME = 'd2_dev_intake'
OBS_TOPIC = '/ax/dev/d2/receipt/observations'
EVENT_TOPIC = '/ax/dev/d2/receipt/events'
INVENTORY_TOPIC = '/ax/dev/d2/inventory'
REGISTER_SERVICE = '/ax/dev/d2/tasks/register'
ZONE = 'loc-dev-receipt-a'
STORAGE = 'loc-dev-storage-1'
STATE_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST, depth=1)
EVENT_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE,
                       history=HistoryPolicy.KEEP_LAST, depth=50)
RUN_TAG = f'd2_w5_intake_ros_{os.getpid()}'
_domain_state = None


# 모듈 domain 예약과 빈 domain 확인
def setUpModule():
    global _domain_state
    _domain_state = ros_test_domain.enter_module_domain(range(80, 90))
    found = ros_test_domain.conflicting_nodes(
        _domain_state['domain'], r'^(?!/ax_test_domain_probe$).+', wait_s=3.0)
    if found:
        ros_test_domain.leave_module_domain(_domain_state)
        raise AssertionError(ros_test_domain.conflict_message(_domain_state['domain'], found))
    os.environ['AX_RUN_TAG'] = RUN_TAG


# domain 반환
def tearDownModule():
    ros_test_domain.leave_module_domain(_domain_state)


class D2IntakeNodeRosTest(unittest.TestCase):
    """설치된 d2_dev_intake를 하위 프로세스로 띄워 확인한다."""

    # rclpy 시험 노드 준비
    @classmethod
    def setUpClass(cls):
        prefix = get_package_prefix('ax_fleet_manager')
        cls.executable = str(Path(prefix) / 'lib' / 'ax_fleet_manager' / 'd2_dev_intake')
        cls.tmp = Path(tempfile.mkdtemp(prefix='ax_d2_intake_ros_'))
        rclpy.init()
        cls.node = rclpy.create_node('d2_intake_ros_probe')
        cls.obs_pub = cls.node.create_publisher(ReceiptObservation, OBS_TOPIC, EVENT_QOS)
        cls.client = cls.node.create_client(RegisterTask, REGISTER_SERVICE)

    # 시험 노드 정리(임시 원장 폴더는 지우지 않고 경로를 남긴다)
    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
        print(f'[d2_intake_ros] 임시 원장 폴더: {cls.tmp}', file=sys.stderr)
        left = ros_test_domain.processes_with(f'AX_RUN_TAG={RUN_TAG}')
        assert [pid for pid in left if pid != os.getpid()] == [], left

    # 시험마다 띄운 하위 프로세스 목록
    def setUp(self):
        self.procs = []

    # 남은 하위 프로세스 정리
    def tearDown(self):
        for proc in self.procs:
            self.stop(proc)

    # 하위 프로세스 SIGINT 정지, 종료 코드
    def stop(self, proc):
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:
                return proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
        return proc.returncode

    # 노드 실행
    def start(self, params, extra=()):
        args = [self.executable, '--ros-args']
        for key, value in params.items():
            args += ['-p', f'{key}:={value}']
        args += list(extra)
        proc = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        self.procs.append(proc)
        return proc

    # 조건까지 수신
    def collect(self, msg_type, topic, qos, duration, until=None, action=None):
        got = []
        sub = self.node.create_subscription(msg_type, topic, got.append, qos)
        end = time.monotonic() + duration
        try:
            if action is not None:
                deadline = time.monotonic() + 5.0
                while (self.node.count_subscribers(OBS_TOPIC) == 0
                       or self.node.count_publishers(topic) == 0) \
                        and time.monotonic() < deadline:
                    rclpy.spin_once(self.node, timeout_sec=0.05)
                time.sleep(0.3)
                action()
            while time.monotonic() < end:
                rclpy.spin_once(self.node, timeout_sec=0.05)
                if until and got and until(got):
                    break
        finally:
            self.node.destroy_subscription(sub)
        return got

    # 입고 관측 발행
    def observe(self, observation_id, object_id='obj-ros-1', object_class='box_small'):
        msg = ReceiptObservation()
        msg.meta.header.stamp = self.node.get_clock().now().to_msg()
        msg.meta.source_id = 'c1_ros_test'
        msg.meta.source_session_id = 'test_session'
        msg.meta.revision = 1
        msg.meta.valid = True
        msg.observation_id = observation_id
        msg.zone_id = ZONE
        msg.object_id = object_id
        msg.object_class = object_class
        msg.identity_confidence = 0.95
        msg.source = ReceiptObservation.SOURCE_SENSOR
        self.obs_pub.publish(msg)

    # 서비스 호출
    def register(self, request_id, object_id, receipt_event_id='', source=ZONE, destination=''):
        self.assertTrue(self.client.wait_for_service(timeout_sec=10.0))
        request = RegisterTask.Request(request_id=request_id, receipt_event_id=receipt_event_id,
                                       object_id=object_id, source_location_id=source,
                                       destination_location_id=destination)
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        self.assertTrue(future.done())
        return future.result()

    # 마지막 재고 snapshot(늦은 구독)
    def inventory(self, until=None):
        got = self.collect(InventorySnapshot, INVENTORY_TOPIC, STATE_QOS, 10.0,
                           until=until or (lambda g: True))
        self.assertTrue(got, '재고 snapshot 수신 없음')
        return got[-1]

    # 입고 → 사건·재고, 반복 관측, 서비스 멱등·용량, 재시작 뒤 revision 유지, 그래프
    def test_receipt_inventory_and_register(self):
        store = self.tmp / 'flow' / 'd2.sqlite'
        store.parent.mkdir()
        self.start({'store_path': str(store)})

        events = self.collect(ReceiptEvent, EVENT_TOPIC, EVENT_QOS, 10.0,
                              until=lambda g: True, action=lambda: self.observe('ob-r1'))
        self.assertTrue(events, 'ReceiptEvent 수신 없음')
        event = events[0]
        self.assertEqual(event.state, ReceiptEvent.TASK_CREATED)
        self.assertTrue(event.receipt_event_id.startswith('rcpt-'))
        self.assertTrue(event.task_id.startswith('task-'))
        self.assertEqual(list(event.observation_ids), ['ob-r1'])

        snapshot = self.inventory(until=lambda g: g[-1].items)
        self.assertTrue(snapshot.meta.valid)
        self.assertEqual(snapshot.meta.source_id, 'd2_dev_intake')
        (item,) = snapshot.items
        self.assertEqual((item.object_id, item.state, item.location_id),
                         ('obj-ros-1', InventoryEntry.AT_SOURCE, ZONE))
        self.assertEqual(item.receipt_event_id, event.receipt_event_id)
        storage = [loc for loc in snapshot.locations if loc.location_id == STORAGE][0]
        self.assertEqual(storage.state, RegisteredLocation.RESERVED)
        self.assertTrue(storage.reservation_id.startswith('rsv-'))
        self.assertEqual({p.robot_role for p in storage.stop_poses},
                         {'MANIPULATOR', 'TRANSPORTER'})
        revision = snapshot.meta.revision

        # 반복 관측: 같은 사건, 관측 ID 누적, 작업 추가 없음
        again = self.collect(ReceiptEvent, EVENT_TOPIC, EVENT_QOS, 10.0,
                             until=lambda g: True, action=lambda: self.observe('ob-r2'))
        self.assertEqual(again[0].receipt_event_id, event.receipt_event_id)
        self.assertEqual(list(again[0].observation_ids), ['ob-r1', 'ob-r2'])

        # 같은 사건 다른 요청 ID → 기존 작업과 DUPLICATE_REQUEST, 같은 요청 재전송은 같은 응답
        dup = self.register('req-ros-1', 'obj-ros-1', receipt_event_id=event.receipt_event_id)
        self.assertEqual((dup.accepted, dup.task_id, dup.reason_code),
                         (False, event.task_id, 'DUPLICATE_REQUEST'))
        # 종류 정보가 없는 수동 요청은 box_small 전용 지점에 거절, 같은 요청 재전송은 같은 응답
        manual = self.register('req-ros-2', 'obj-ros-2')
        self.assertEqual((manual.accepted, manual.reason_code),
                         (False, 'OBJECT_CLASS_NOT_ALLOWED'))
        replay = self.register('req-ros-2', 'obj-ros-2')
        self.assertEqual((replay.accepted, replay.reason_code),
                         (False, 'OBJECT_CLASS_NOT_ALLOWED'))

        # 다른 물품 입고: 용량 1 지점이 예약으로 차 있어 사건 REJECTED, 작업 없음
        full = self.collect(ReceiptEvent, EVENT_TOPIC, EVENT_QOS, 10.0, until=lambda g: True,
                            action=lambda: self.observe('ob-r3', object_id='obj-ros-3'))
        self.assertEqual((full[0].state, full[0].task_id), (ReceiptEvent.REJECTED, ''))
        self.assertNotEqual(full[0].receipt_event_id, event.receipt_event_id)

        # 그래프: Action 없음, 운영 Topic·cmd_vel 없음
        self.assertEqual(get_action_client_names_and_types_by_node(self.node, NODE_NAME, '/'), [])
        self.assertEqual(get_action_server_names_and_types_by_node(self.node, NODE_NAME, '/'), [])
        pubs = {n for n, _ in self.node.get_publisher_names_and_types_by_node(NODE_NAME, '/')}
        self.assertEqual(pubs, {EVENT_TOPIC, INVENTORY_TOPIC, '/rosout', '/parameter_events'})
        topics = {n for n, _ in self.node.get_topic_names_and_types()}
        self.assertFalse([t for t in topics if t.startswith('/ax/fleet')], topics)
        self.assertFalse([t for t in topics if 'cmd_vel' in t or '/arm/' in t], topics)

        # SIGINT 종료 → 재시작: 재고 revision 유지, 같은 관측은 재판정 없이 같은 사건
        self.assertEqual(self.stop(self.procs[0]), 0)
        self.start({'store_path': str(store)})
        restarted = self.inventory()
        self.assertGreaterEqual(restarted.meta.revision, revision)
        self.assertEqual([i.object_id for i in restarted.items], ['obj-ros-1'])
        self.assertEqual(restarted.meta.source_id, 'd2_dev_intake')
        replayed = self.collect(ReceiptEvent, EVENT_TOPIC, EVENT_QOS, 10.0,
                                until=lambda g: True, action=lambda: self.observe('ob-r1'))
        self.assertEqual(replayed[0].receipt_event_id, event.receipt_event_id)
        self.assertEqual(len(self.inventory().items), 1)

    # 운영 모드·운영 이름·remap 우회·원장 경로 없음은 시작 거부(종료 코드 2)
    def test_operational_and_forbidden_names_rejected(self):
        store = self.tmp / 'reject' / 'd2.sqlite'
        store.parent.mkdir()
        cases = [
            ({'store_path': str(store), 'mode': 'OPERATIONAL'}, ()),
            ({'store_path': str(store), 'inventory_topic': '/ax/fleet/inventory'}, ()),
            ({'store_path': str(store), 'register_service': '/ax/fleet/tasks/register'}, ()),
            ({'store_path': str(store)}, ('-r', f'{OBS_TOPIC}:=/ax/fleet/receipt/observations')),
            ({}, ()),
        ]
        for params, extra in cases:
            with self.subTest(params=params, extra=extra):
                proc = self.start(params, extra)
                output, _ = proc.communicate(timeout=20)
                self.assertEqual(proc.returncode, 2, output)
                self.assertIn('시작 거부', output)
        self.assertEqual(self.node.get_publishers_info_by_topic('/ax/fleet/inventory'), [])


if __name__ == '__main__':
    unittest.main()
