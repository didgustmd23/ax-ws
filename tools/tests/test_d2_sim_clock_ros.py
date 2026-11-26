"""D2 개발 노드의 sim time 첫 발행 시험(격리 설치본, 가짜 /clock, Gazebo 없음).

확인: use_sim_time:=true에서 /clock을 받기 전에는 출력(작업 목록·재고)을 내지 않고(stamp 0 메시지 없음),
/clock을 발행하면 stamp > 0인 첫 메시지가 나온다.
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
from ax_interfaces.msg import GroupTaskList, InventorySnapshot
from rclpy.clock import Clock, ClockType
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock as ClockMessage

sys.path.insert(0, str(Path(__file__).resolve().parent))
import ros_test_domain  # noqa: E402

STATE_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST, depth=1)
OUTPUTS = {'fleet_tasks': ('/ax/dev/d2_mock/fleet_tasks', GroupTaskList),
           'inventory': ('/ax/dev/d2/inventory', InventorySnapshot)}
RUN_TAG = f'd2_sim_clock_ros_{os.getpid()}'
SIM_START_S = 100.0
NO_CLOCK_WAIT_S = 3.5
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


class D2SimClockRosTest(unittest.TestCase):
    """두 개발 노드를 use_sim_time:=true 하위 프로세스로 띄워 /clock 전후 출력을 본다."""

    # 시험 노드·실행 파일 경로 준비
    @classmethod
    def setUpClass(cls):
        lib = Path(get_package_prefix('ax_fleet_manager')) / 'lib' / 'ax_fleet_manager'
        cls.publisher_exe = str(lib / 'group_dev_publisher')
        cls.intake_exe = str(lib / 'd2_dev_intake')
        cls.fixture = str(Path(get_package_share_directory('ax_fleet_manager'))
                          / 'config' / 'dev_fixture_transfer.json')
        cls.tmp = Path(tempfile.mkdtemp(prefix='ax_d2_sim_clock_ros_'))
        rclpy.init()
        cls.node = rclpy.create_node('d2_sim_clock_probe')

    # 시험 노드 정리와 잔여 프로세스 확인(임시 폴더는 지우지 않고 경로를 남긴다)
    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
        print(f'[d2_sim_clock_ros] 임시 폴더: {cls.tmp}', file=sys.stderr)
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

    # 노드 실행(use_sim_time:=true)
    def start(self, executable, params):
        args = [executable, '--ros-args', '-p', 'use_sim_time:=true']
        for key, value in params.items():
            args += ['-p', f'{key}:={value}']
        proc = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        self.procs.append(proc)
        return proc

    # /clock 없이 대기 → /clock 발행(STEADY 200 Hz) 뒤 수신. 결과는 {출력: [(단계, stamp_ns)]}
    def observe(self, wait_clock_s, until_s):
        got = {name: [] for name in OUTPUTS}
        phase = {'clock': False}
        subs = [self.node.create_subscription(
            kind, topic,
            lambda m, n=name: got[n].append(
                (phase['clock'], m.meta.header.stamp.sec * 10**9 + m.meta.header.stamp.nanosec)),
            STATE_QOS) for name, (topic, kind) in OUTPUTS.items()]
        clock_pub = self.node.create_publisher(ClockMessage, '/clock', 10)
        started = time.monotonic()

        # sim 시각 발행(시작 후 wait_clock_s부터)
        def tick():
            elapsed = time.monotonic() - started
            if elapsed < wait_clock_s:
                return
            phase['clock'] = True
            sim = SIM_START_S + elapsed - wait_clock_s
            msg = ClockMessage()
            msg.clock.sec = int(sim)
            msg.clock.nanosec = int((sim - int(sim)) * 1e9)
            clock_pub.publish(msg)

        timer = self.node.create_timer(0.005, tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        try:
            while time.monotonic() - started < until_s:
                rclpy.spin_once(self.node, timeout_sec=0.01)
                if phase['clock'] and all(any(c for c, _ in v) for v in got.values()):
                    break
        finally:
            self.node.destroy_timer(timer)
            self.node.destroy_publisher(clock_pub)
            for sub in subs:
                self.node.destroy_subscription(sub)

        return got

    # /clock 전에는 두 출력 모두 0건(stamp 0 메시지 없음), /clock 뒤 첫 메시지 stamp > 0
    def test_no_publish_before_clock(self):
        self.start(self.publisher_exe, {'fixture_file': self.fixture})
        self.start(self.intake_exe, {'store_path': self.tmp / 'd2.sqlite'})

        got = self.observe(NO_CLOCK_WAIT_S, NO_CLOCK_WAIT_S + 10.0)

        for name, items in got.items():
            before = [stamp for clock, stamp in items if not clock]
            after = [stamp for clock, stamp in items if clock]
            self.assertEqual(before, [], f'{name}: /clock 전 발행 {len(before)}건 stamp={before}')
            self.assertTrue(after, f'{name}: /clock 뒤 수신 없음')
            self.assertGreater(after[0], 0, f'{name}: 첫 메시지 stamp 0')
            self.assertGreaterEqual(after[0], int(SIM_START_S * 1e9), name)

        for proc in self.procs:
            self.assertIsNone(proc.poll(), '노드가 일찍 종료함')


if __name__ == '__main__':
    unittest.main()
