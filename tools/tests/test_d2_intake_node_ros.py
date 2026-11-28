"""D2 W5 개발 입고 노드의 실제 DDS 시험(격리 설치본, 가짜 관측, Gazebo·실제 구동 없음).

확인: 개발 이름공간 입고 관측 → ReceiptEvent·InventorySnapshot, RegisterTask 서비스 멱등·사건당 작업 1개·
용량 거절, 재고 STATE QoS·늦은 구독, 재시작 뒤 재고 revision 유지·관측 재판정 없음, 운영 이름·운영 모드
거부, 그래프상 Action·운영 Topic·cmd_vel 없음, SIGINT 종료와 잔여 프로세스 0.
시각: 입고 흐름 시험은 노드를 use_sim_time으로 띄우고 시험이 /clock을 time.monotonic 기반으로 낸다(WSL 시스템
시계 역행이 관측 나이 판정에 섞이지 않게). 실패하면 단언 메시지에 노드 출력 끝부분을 붙인다.
환경: 검증할 격리 install을 source한 셸. domain은 ros_test_domain으로 80~89에서 빈 곳을 예약한다.
"""
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
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
from rosgraph_msgs.msg import Clock

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
# 시험 /clock: 200 Hz(눈금 5 ms, 관측 stamp 여유 0.5 s의 1%), 시작 시각 1000 s(0이면 노드가 재고 발행을 건너뜀)
CLOCK_PERIOD_S = 0.005
CLOCK_START_S = 1000.0
# 관측 stamp는 마지막으로 낸 /clock보다 이만큼 과거로 찍는다. 노드가 받은 /clock이 시험보다 늦어도(최대 0.5 s)
# 미래가 되지 않고, 전달이 1.5 s 넘게 늦지 않으면 나이 한계 2.0 s 안에 든다
OBS_STAMP_BACK_S = 0.5
# 관측 발행 전 매칭 대기 상한(s). 기동~매칭 실측(부하 포함)보다 넉넉하고 서비스 대기 10 s와 맞춘다
MATCH_TIMEOUT_S = 10.0
# 매칭 뒤 관측 발행 전 대기(s). 매칭은 시험 쪽 endpoint 기준이라 노드 쪽(사건 발행자가 시험 구독을 알고,
# /clock을 처음 받기까지)이 늦을 수 있다. 빼면 재시작 뒤 사건 유실·기동 직후 시각 판정 거절이 났다(150회 중 4회)
MATCH_SETTLE_S = 0.3
# 실패 메시지에 붙일 노드 출력 줄 수
OUTPUT_TAIL_LINES = 40
OUTPUT_MARK = '--- 노드 출력 끝'
_domain_state = None


# 시험이 내는 /clock. spin과 무관한 별도 스레드가 time.monotonic 기반으로 단조 증가 값을 계속 낸다
class MonotonicClockSource:
    """시스템 시계(time.time) 역행과 무관한 시험 시각."""

    # 발행 노드·시작 기준
    def __init__(self, node):
        self.pub = node.create_publisher(Clock, '/clock', 10)
        self.mono0 = time.monotonic()
        self.last_ns = 0
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    # 현재 시험 시각(ns), monotonic 기반
    def now_ns(self):
        return int((CLOCK_START_S + time.monotonic() - self.mono0) * 1e9)

    # 마지막으로 발행한 /clock 값(ns)
    def published_ns(self):
        with self.lock:
            return self.last_ns

    # 발행 반복(첫 값은 바로 낸다)
    def _run(self):
        while not self.stop_event.is_set():
            ns = self.now_ns()
            msg = Clock()
            msg.clock.sec, msg.clock.nanosec = divmod(ns, 1_000_000_000)
            self.pub.publish(msg)

            with self.lock:
                self.last_ns = ns

            self.stop_event.wait(CLOCK_PERIOD_S)

    # 시작하고 첫 값이 나갈 때까지 대기
    def start(self):
        self.thread.start()
        while self.published_ns() == 0:
            time.sleep(0.001)

    # 정지
    def stop(self):
        self.stop_event.set()
        self.thread.join(timeout=5.0)


# 시험 메서드 실패(단언·IndexError 등) 시 노드 출력 끝부분을 AssertionError 메시지에 붙인다
def with_node_output(test):
    def wrapper(self):
        try:
            test(self)
        except Exception as error:
            # 이미 노드 출력을 붙인 실패(매칭 대기 초과)는 그대로 올린다
            if OUTPUT_MARK in str(error):
                raise
            raise AssertionError(f'{type(error).__name__}: {error}\n{self.output_tails()}') from error

    wrapper.__name__ = test.__name__
    wrapper.__doc__ = test.__doc__
    return wrapper


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
        cls.log_count = 0
        rclpy.init()
        cls.node = rclpy.create_node('d2_intake_ros_probe')
        # /clock 전용 노드(spin하지 않고 발행만 한다)
        cls.clock_node = rclpy.create_node('d2_intake_ros_clock')
        cls.clock = MonotonicClockSource(cls.clock_node)
        cls.clock.start()
        cls.obs_pub = cls.node.create_publisher(ReceiptObservation, OBS_TOPIC, EVENT_QOS)
        cls.client = cls.node.create_client(RegisterTask, REGISTER_SERVICE)

    # 시험 노드 정리(임시 원장 폴더는 지우지 않고 경로를 남긴다)
    @classmethod
    def tearDownClass(cls):
        cls.clock.stop()
        cls.clock_node.destroy_node()
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

    # 노드 실행. 출력은 시험 임시 폴더의 파일로 받는다(PIPE 버퍼가 차서 노드가 멈추지 않게)
    def start(self, params, extra=()):
        args = [self.executable, '--ros-args']
        for key, value in params.items():
            args += ['-p', f'{key}:={value}']
        args += list(extra)

        log_dir = self.tmp / 'node_logs'
        log_dir.mkdir(exist_ok=True)
        type(self).log_count += 1
        log_path = log_dir / f'{self._testMethodName}_{self.log_count:02d}.log'
        env = dict(os.environ, RCUTILS_LOGGING_BUFFERED_STREAM='0', PYTHONUNBUFFERED='1')

        with open(log_path, 'w') as log:
            proc = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT, text=True, env=env)
        proc.log_path = log_path
        self.procs.append(proc)
        return proc

    # 하위 프로세스 출력 전체
    def output(self, proc):
        return proc.log_path.read_text(errors='replace')

    # 이 시험에서 띄운 노드들의 출력 끝부분(실패 메시지용)
    def output_tails(self):
        if not self.procs:
            return f'{OUTPUT_MARK}: 이 시험에서 띄운 노드 없음 ---'

        parts = []
        for proc in self.procs:
            lines = self.output(proc).splitlines()[-OUTPUT_TAIL_LINES:]
            parts.append(f'{OUTPUT_MARK} {OUTPUT_TAIL_LINES}줄: {proc.log_path} ---')
            parts.extend(lines)
        return '\n'.join(parts)

    # 조건까지 수신. action이 있으면 사건 구독·관측 발행이 양쪽 다 매칭된 뒤 보낸다(상한 넘으면 보내지 않고 실패)
    def collect(self, msg_type, topic, qos, duration, until=None, action=None):
        got = []
        sub = self.node.create_subscription(msg_type, topic, got.append, qos)
        try:
            if action is not None:
                self.wait_matched(sub, topic)
                action()
            end = time.monotonic() + duration
            while time.monotonic() < end:
                rclpy.spin_once(self.node, timeout_sec=0.05)
                if until and got and until(got):
                    break
        finally:
            self.node.destroy_subscription(sub)
        return got

    # 사건 구독이 노드 발행자와, 관측·/clock 발행자가 노드 구독과 매칭될 때까지 대기(endpoint 매칭 기준).
    # 상한을 넘으면 관측을 보내지 않고 실패한다. 매칭 뒤 MATCH_SETTLE_S만큼 더 기다린다
    def wait_matched(self, sub, topic):
        deadline = time.monotonic() + MATCH_TIMEOUT_S

        def counts():
            return (sub.get_publisher_count(), self.obs_pub.get_subscription_count(),
                    self.clock.pub.get_subscription_count())

        while not all(count > 0 for count in counts()):
            if time.monotonic() >= deadline:
                events, observations, clock = counts()
                self.fail(f'매칭 대기 초과: {MATCH_TIMEOUT_S} s 동안 {topic} 발행자 {events}, '
                          f'{OBS_TOPIC} 구독자 {observations}, /clock 구독자 {clock}\n'
                          f'{self.output_tails()}')
            rclpy.spin_once(self.node, timeout_sec=0.05)

        time.sleep(MATCH_SETTLE_S)

    # 입고 관측 발행
    def observe(self, observation_id, object_id='obj-ros-1', object_class='box_small'):
        msg = ReceiptObservation()
        sec, nanosec = divmod(self.clock.published_ns() - int(OBS_STAMP_BACK_S * 1e9), 1_000_000_000)
        msg.meta.header.stamp.sec, msg.meta.header.stamp.nanosec = sec, nanosec
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

    # 입고 → 사건·재고, 반복 관측, 서비스 멱등·용량, 재시작 뒤 revision 유지, 그래프(시험 /clock 기준)
    @with_node_output
    def test_receipt_inventory_and_register(self):
        store = self.tmp / 'flow' / 'd2.sqlite'
        store.parent.mkdir()
        self.start({'store_path': str(store), 'use_sim_time': 'true'})

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
        self.start({'store_path': str(store), 'use_sim_time': 'true'})
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
                proc.wait(timeout=20)
                output = self.output(proc)
                self.assertEqual(proc.returncode, 2, output)
                self.assertIn('시작 거부', output)
        self.assertEqual(self.node.get_publishers_info_by_topic('/ax/fleet/inventory'), [])


if __name__ == '__main__':
    unittest.main()
