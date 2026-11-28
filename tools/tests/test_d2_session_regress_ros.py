"""D2 개발 노드의 /clock 정지·역행 시험(격리 설치본, 가짜 /clock, Gazebo 없음).

확인(작업 목록·재고 두 출력 모두):
- /clock 정지(같은 값 반복): 1 Hz 재발행 유지, stamp는 정지 시각 고정, source_session_id 불변.
- 허용 오차(0.1 s)보다 작은 역행: 세션 불변.
- 큰 역행(sim 시간 리셋 모사): 세션이 한 번만 바뀌고, 이후 stamp는 증가하며 revision은 그대로.
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
RUN_TAG = f'd2_session_regress_ros_{os.getpid()}'

# 단계(시작 경과 s, 이름, sim 시각 함수(단계 안 경과 s)). 정지 값·역행 크기는 시험용
SIM_START_S = 100.0
FREEZE_AT_S = 103.0
PHASES = [
    (0.0, 'run', lambda dt: SIM_START_S + dt),
    (3.0, 'freeze', lambda dt: FREEZE_AT_S),
    (6.5, 'jitter', lambda dt: FREEZE_AT_S - 0.05 + dt),
    (9.5, 'regress', lambda dt: 50.0 + dt),
]
END_S = 14.0
SETTLE_S = 0.2
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


# 경과 시각 → (단계 이름, 단계 안 경과 s, sim 시각 s)
def phase_at(elapsed):
    start, name, sim = PHASES[0]
    for item in PHASES:
        if elapsed >= item[0]:
            start, name, sim = item
    return name, elapsed - start, sim(elapsed - start)


class D2SessionRegressRosTest(unittest.TestCase):
    """두 개발 노드를 use_sim_time:=true 하위 프로세스로 띄워 /clock 정지·역행 동안 출력을 본다."""

    # 시험 노드·실행 파일 경로 준비
    @classmethod
    def setUpClass(cls):
        lib = Path(get_package_prefix('ax_fleet_manager')) / 'lib' / 'ax_fleet_manager'
        cls.publisher_exe = str(lib / 'group_dev_publisher')
        cls.intake_exe = str(lib / 'd2_dev_intake')
        cls.fixture = str(Path(get_package_share_directory('ax_fleet_manager'))
                          / 'config' / 'dev_fixture_transfer.json')
        cls.tmp = Path(tempfile.mkdtemp(prefix='ax_d2_session_regress_ros_'))
        rclpy.init()
        cls.node = rclpy.create_node('d2_session_regress_probe')

    # 시험 노드 정리와 잔여 프로세스 확인(임시 폴더는 지우지 않고 경로를 남긴다)
    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
        print(f'[d2_session_regress_ros] 임시 폴더: {cls.tmp}', file=sys.stderr)
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

    # 단계별 /clock 발행(STEADY 200 Hz) 중 수신 기록.
    # 결과는 {출력: [(단계, 단계 안 경과 s, 수신 monotonic, stamp_ns, 세션, revision)]}
    def observe(self):
        got = {name: [] for name in OUTPUTS}
        started = time.monotonic()

        # 수신 한 건 기록(단계는 수신 시각 기준)
        def record(name, msg):
            now = time.monotonic()
            phase, within, _ = phase_at(now - started)
            stamp = msg.meta.header.stamp
            got[name].append((phase, within, now, stamp.sec * 10**9 + stamp.nanosec,
                              msg.meta.source_session_id, msg.meta.revision))

        subs = [self.node.create_subscription(
            kind, topic, lambda m, n=name: record(n, m), STATE_QOS)
            for name, (topic, kind) in OUTPUTS.items()]
        clock_pub = self.node.create_publisher(ClockMessage, '/clock', 10)

        # 현재 단계의 sim 시각 발행
        def tick():
            _, _, sim = phase_at(time.monotonic() - started)
            msg = ClockMessage()
            msg.clock.sec = int(sim)
            msg.clock.nanosec = int(round((sim - int(sim)) * 1e9))
            clock_pub.publish(msg)

        timer = self.node.create_timer(0.005, tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        try:
            while time.monotonic() - started < END_S:
                rclpy.spin_once(self.node, timeout_sec=0.01)
        finally:
            self.node.destroy_timer(timer)
            self.node.destroy_publisher(clock_pub)
            for sub in subs:
                self.node.destroy_subscription(sub)

        return got

    # 정지: 1 Hz 유지·stamp 고정·세션 불변. 작은 역행: 세션 불변. 큰 역행: 세션 1회 교체·stamp 증가·revision 유지
    def test_freeze_and_regression(self):
        store = self.tmp / 'd2.sqlite'
        self.start(self.intake_exe, {'store_path': store})
        time.sleep(1.0)
        self.start(self.publisher_exe, {'fixture_file': self.fixture, 'store_path': store})
        time.sleep(1.0)

        got = self.observe()

        for name, items in got.items():
            with self.subTest(output=name):
                self.check_output(name, items)

        for proc in self.procs:
            self.assertIsNone(proc.poll(), '노드가 일찍 종료함')

    # 출력 하나의 단계별 단언
    def check_output(self, name, items):
        settled = {phase: [r for r in items if r[0] == phase and r[1] >= SETTLE_S]
                   for _, phase, _ in PHASES}
        self.assertTrue(settled['run'], f'{name}: 진행 단계 수신 없음')
        session = settled['run'][-1][4]
        revision = settled['run'][-1][5]

        # 정지(3.5 s): 2건 이상, 간격 1.5 s 미만, stamp는 정지 시각, 세션 불변
        frozen = settled['freeze']
        self.assertGreaterEqual(len(frozen), 2, f'{name}: 정지 중 재발행 {len(frozen)}건')
        gaps = [b[2] - a[2] for a, b in zip(frozen, frozen[1:])]
        self.assertLess(max(gaps), 1.5, f'{name}: 정지 중 간격 {gaps}')
        self.assertEqual({r[3] for r in frozen}, {int(FREEZE_AT_S * 1e9)}, f'{name}: 정지 stamp')
        self.assertEqual({r[4] for r in frozen}, {session}, f'{name}: 정지 중 세션 바뀜')

        # 허용 오차보다 작은 역행(0.05 s): 세션 불변
        self.assertTrue(settled['jitter'], f'{name}: 작은 역행 단계 수신 없음')
        self.assertEqual({r[4] for r in settled['jitter']}, {session},
                         f'{name}: 0.05 s 역행에 세션 바뀜')

        # 큰 역행: 새 세션 하나, stamp 증가(리셋 뒤 값), revision 유지
        after = settled['regress']
        self.assertGreaterEqual(len(after), 2, f'{name}: 역행 뒤 수신 {len(after)}건')
        sessions = {r[4] for r in after}
        self.assertEqual(len(sessions), 1, f'{name}: 역행 뒤 세션 {sessions}')
        self.assertNotIn(session, sessions, f'{name}: 역행 뒤에도 같은 세션 {session}')
        stamps = [r[3] for r in after]
        self.assertEqual(stamps, sorted(set(stamps)), f'{name}: 역행 뒤 stamp 증가 아님 {stamps}')
        self.assertLess(stamps[-1], int(SIM_START_S * 1e9), f'{name}: 역행 뒤 stamp {stamps}')
        self.assertEqual({r[5] for r in after}, {revision}, f'{name}: 역행 뒤 revision 바뀜')

        # 전체 수신에서 세션 교체는 한 번뿐
        changes = sum(1 for a, b in zip(items, items[1:]) if a[4] != b[4])
        self.assertEqual(changes, 1, f'{name}: 세션 교체 {changes}회')


if __name__ == '__main__':
    unittest.main()
