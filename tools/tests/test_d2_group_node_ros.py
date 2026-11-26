"""D2 개발 출력 노드의 실제 DDS 시험(격리 설치본, 가짜 입력, Gazebo·실제 구동 없음).

확인: 개발 Topic의 GroupTaskList 형식·출처·세션·revision, STATE QoS·늦은 구독, 1 Hz 재발행,
원장 변경 시 revision 증가, 잠금이 남은 실패 작업 유지, 운영 모드·운영 Topic·remap 거부,
그래프상 구독·Action·운영 Topic·cmd_vel 없음, SIGINT 종료와 잔여 프로세스 0.
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
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from ax_interfaces.msg import GroupTaskList, GroupTaskStatus
from rclpy.action import get_action_client_names_and_types_by_node
from rclpy.action import get_action_server_names_and_types_by_node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

sys.path.insert(0, str(Path(__file__).resolve().parent))
import ros_test_domain  # noqa: E402
from d2_stamp_order import check_stamp_order  # noqa: E402

from ax_fleet_manager import group_model as gm  # noqa: E402
from ax_fleet_manager.group_events import FailTerminate, HoldApplied  # noqa: E402

DEV_TOPIC = '/ax/dev/d2_mock/fleet_tasks'
NODE_NAME = 'd2_group_dev_publisher'
STATE_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST, depth=1)
RUN_TAG = f'd2_w3_node_ros_{os.getpid()}'
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


class D2GroupNodeRosTest(unittest.TestCase):
    """설치된 group_dev_publisher를 하위 프로세스로 띄워 확인한다."""

    # rclpy 시험 노드 준비
    @classmethod
    def setUpClass(cls):
        prefix = get_package_prefix('ax_fleet_manager')
        cls.executable = str(Path(prefix) / 'lib' / 'ax_fleet_manager' / 'group_dev_publisher')
        cls.fixture = str(Path(get_package_share_directory('ax_fleet_manager'))
                          / 'config' / 'dev_fixture_transfer.json')
        cls.tmp = Path(tempfile.mkdtemp(prefix='ax_d2_node_ros_'))
        rclpy.init()
        cls.node = rclpy.create_node('d2_node_ros_probe')

    # 시험 노드 정리(임시 원장 폴더는 지우지 않고 경로를 남긴다)
    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
        print(f'[d2_node_ros] 임시 원장 폴더: {cls.tmp}', file=sys.stderr)
        left = ros_test_domain.processes_with(f'AX_RUN_TAG={RUN_TAG}')
        assert [pid for pid in left if pid != os.getpid()] == [], left

    # 시험마다 띄운 하위 프로세스 목록
    def setUp(self):
        self.procs = []

    # 남은 하위 프로세스 정리
    def tearDown(self):
        for proc in self.procs:
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                try:
                    proc.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()

    # 노드 실행(매개변수·추가 ROS 인자)
    def start(self, params, extra=()):
        args = [self.executable, '--ros-args']
        for key, value in params.items():
            args += ['-p', f'{key}:={value}']
        args += list(extra)
        proc = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        self.procs.append(proc)
        return proc

    # 시스템 시계와 monotonic의 차(s). 줄면 시스템 시계가 역행한 것이다
    @staticmethod
    def clock_offset():
        return time.time() - time.monotonic()

    # 조건까지 수신(구독은 STATE QoS). offsets를 주면 수신마다 시계 offset을 함께 남긴다
    def collect(self, duration, until=None, topic=DEV_TOPIC, offsets=None):
        got = []

        # 수신 기록(monotonic 수신 시각, 메시지)과 선택적 offset 기록
        def receive(msg):
            got.append((time.monotonic(), msg))
            if offsets is not None:
                offsets.append(self.clock_offset())

        sub = self.node.create_subscription(GroupTaskList, topic, receive, STATE_QOS)
        end = time.monotonic() + duration
        try:
            while time.monotonic() < end:
                rclpy.spin_once(self.node, timeout_sec=0.05)
                if until and got and until(got):
                    break
        finally:
            self.node.destroy_subscription(sub)
        return got

    # 시험용 D2 원장: 적재 실패 후 실패 종료(인계 진행 중 잠금 유지)
    def make_store(self, name):
        sys.path.insert(0, str(Path(__file__).resolve().parent))
        from test_d2_workflow_scenarios import begin, make_scenario

        folder = self.tmp / name
        folder.mkdir()
        runtime, mock, _, _ = make_scenario(folder)
        mock.overrides[('task_a', 'load_pick_and_stow')] = gm.OUTCOME_FAILED
        begin(runtime, 'task_a', 'box_a', 'robot_m', 'robot_t')
        mock.run()
        runtime.handle('task_a', FailTerminate('fail_evt_a'))
        runtime.store.close()
        return folder / 'd2.sqlite'

    # fixture 출력: 형식·출처·세션·revision·QoS·늦은 구독·1 Hz 재발행·그래프
    def test_fixture_output_and_graph(self):
        self.start({'fixture_file': self.fixture})
        first = self.collect(15.0, until=lambda g: True)
        self.assertTrue(first, '개발 Topic 수신 없음')
        msg = first[0][1]
        self.assertEqual(msg.meta.source_id, 'd2_dev_mock:dev_fixture_transfer')
        self.assertTrue(msg.meta.source_session_id)
        self.assertEqual(msg.meta.revision, 1)
        self.assertTrue(msg.meta.valid)
        self.assertEqual(len(msg.tasks), 1)
        task = msg.tasks[0]
        self.assertEqual(task.state, GroupTaskStatus.ASSIGNED)
        self.assertEqual(task.transfer_state, GroupTaskStatus.TRANSFER_NONE)
        self.assertEqual((task.manipulator_robot_id, task.transporter_robot_id),
                         ('robot_1', 'robot_2'))
        self.assertEqual(task.task_revision, 1)

        # 늦게 접속해도 마지막 값을 받고, 같은 revision을 약 1 Hz로 새 stamp와 함께 재발행
        # 시계 offset은 대기 전부터 잰다: 창의 첫 메시지(transient_local 마지막 값)는 대기 전에 발행됐을 수 있다
        offsets = [self.clock_offset()]
        time.sleep(1.5)
        offsets.append(self.clock_offset())
        got = self.collect(3.6, offsets=offsets)
        offsets.append(self.clock_offset())
        self.assertGreaterEqual(len(got), 3)
        self.assertEqual({m.meta.revision for _, m in got}, {1})
        self.assertEqual({m.meta.source_session_id for _, m in got}, {msg.meta.source_session_id})
        gaps = [b[0] - a[0] for a, b in zip(got[1:], got[2:])]
        self.assertTrue(all(0.6 < g < 1.6 for g in gaps), gaps)
        stamps = [m.meta.header.stamp.sec * 10**9 + m.meta.header.stamp.nanosec for _, m in got]
        # stamp(시스템 시계)는 단조·중복 없음. 창 안에서 시스템 시계 역행이 측정된 경우에만 그 크기 안의
        # 감소 1회를 허용한다(d2_stamp_order, WSL 실측 역행 1.2~1.75 s)
        ok, detail = check_stamp_order(stamps, offsets)
        if detail:
            print(f'[stamp_order] {detail}')
        self.assertTrue(ok, detail)

        # 발행자 QoS·그래프: 개발 Topic만 발행, 구독·Action·운영 Topic·cmd_vel 없음
        infos = self.node.get_publishers_info_by_topic(DEV_TOPIC)
        self.assertEqual(len(infos), 1)
        self.assertEqual(infos[0].qos_profile.durability, DurabilityPolicy.TRANSIENT_LOCAL)
        self.assertEqual(infos[0].qos_profile.reliability, ReliabilityPolicy.RELIABLE)

        pubs = {n for n, _ in self.node.get_publisher_names_and_types_by_node(NODE_NAME, '/')}
        subs = {n for n, _ in self.node.get_subscriber_names_and_types_by_node(NODE_NAME, '/')}
        self.assertEqual(pubs, {DEV_TOPIC, '/rosout', '/parameter_events'})
        self.assertFalse([n for n in subs if n.startswith('/ax/')], subs)
        self.assertEqual(get_action_client_names_and_types_by_node(self.node, NODE_NAME, '/'), [])
        self.assertEqual(get_action_server_names_and_types_by_node(self.node, NODE_NAME, '/'), [])
        topics = {n for n, _ in self.node.get_topic_names_and_types()}
        self.assertNotIn('/ax/fleet/tasks', topics)
        self.assertFalse([t for t in topics if 'cmd_vel' in t or '/arm/' in t], topics)

        # SIGINT 종료
        proc = self.procs[0]
        proc.send_signal(signal.SIGINT)
        self.assertEqual(proc.wait(timeout=10), 0)

    # 원장 출력: 잠금이 남은 실패 작업 유지, 원장 변경 시 revision 증가·바로 발행
    def test_store_output_retains_locked_task_and_tracks_change(self):
        path = self.make_store('store_change')
        self.start({'store_path': str(path)})
        got = self.collect(15.0, until=lambda g: True)
        self.assertTrue(got, '개발 Topic 수신 없음')
        msg = got[0][1]
        self.assertEqual(msg.meta.source_id, 'd2_dev_mock:store')
        task = msg.tasks[0]
        self.assertEqual(task.state, GroupTaskStatus.FAILED)
        self.assertEqual(task.transfer_state, GroupTaskStatus.TRANSFER_IN_PROGRESS)
        self.assertEqual(task.failure_event_id, 'fail_evt_a')
        self.assertEqual(list(task.hold_ids), [])
        revision = msg.meta.revision

        # 다른 프로세스(이 시험)가 원장을 바꾸면 revision+1로 1 s 안에 발행
        from ax_fleet_manager.ports import MockLocalTaskPort, MockNavigationPort
        from ax_fleet_manager.runtime import GroupRuntime
        from ax_fleet_manager.task_store import TaskStore
        from test_d2_workflow_scenarios import LOCATIONS, Clock
        runtime = GroupRuntime(TaskStore(str(path)), {'LOCAL_TASK': MockLocalTaskPort(),
                                                      'NAVIGATION': MockNavigationPort()},
                               Clock(), map_revision=1, locations=LOCATIONS)
        changed_at = time.monotonic()
        runtime.handle('task_a', HoldApplied('hold_dev_1'))
        runtime.store.close()
        got = self.collect(5.0, until=lambda g: any(m.meta.revision > revision for _, m in g))
        newer = [(t, m) for t, m in got if m.meta.revision > revision]
        self.assertTrue(newer, '원장 변경 후 새 revision 없음')
        self.assertLess(newer[0][0] - changed_at, 1.0)
        self.assertEqual(list(newer[0][1].tasks[0].hold_ids), ['hold_dev_1'])
        self.assertEqual(newer[0][1].tasks[0].transfer_state,
                         GroupTaskStatus.TRANSFER_IN_PROGRESS)

    # 운영 모드·운영 Topic·remap 우회는 시작 거부(종료 코드 2), 운영 Topic 발행자 없음
    def test_operational_and_forbidden_outputs_rejected(self):
        cases = [
            ({'fixture_file': self.fixture, 'mode': 'OPERATIONAL'}, ()),
            ({'fixture_file': self.fixture, 'output_topic': '/ax/fleet/tasks'}, ()),
            ({'fixture_file': self.fixture}, ('-r', f'{DEV_TOPIC}:=/ax/fleet/tasks')),
            ({}, ()),
        ]
        for params, extra in cases:
            with self.subTest(params=params, extra=extra):
                proc = self.start(params, extra)
                output, _ = proc.communicate(timeout=20)
                self.assertEqual(proc.returncode, 2, output)
                self.assertIn('시작 거부', output)
        self.assertEqual(self.node.get_publishers_info_by_topic('/ax/fleet/tasks'), [])


if __name__ == '__main__':
    unittest.main()
