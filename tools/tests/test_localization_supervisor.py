# B2 위치 신뢰성 감독(localization_supervisor.py)을 가짜 입력·가짜 AMCL·가짜 A4로 실제 ROS 통신에서 검사한다.
# 이 시험 프로세스가 시계·odom·스캔·TF·지도·ActuationState·AMCL(initialpose·무이동 갱신·상태 조회)을 흉내 내고,
# 로봇 두 대의 감독 노드를 격리 ROS_DOMAIN_ID에서 실행한다. A4·D·AMCL 실제 통합 시험이 아니다.
import importlib.util
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import tempfile
import threading
import time
import unittest
import uuid

ROOT = Path(__file__).resolve().parents[2]
SUPERVISOR = ROOT / 'src/ax_localization/scripts/localization_supervisor.py'
ROBOTS = ('robot_1', 'robot_2')
START = {'robot_1': (-2.5, 1.5, 0.0), 'robot_2': (0.0, 0.0, 0.0)}
ODOM = {'robot_1': (0.3, 0.1, 0.05), 'robot_2': (0.6, -0.2, 0.1), 'robot_3': (-0.4, 0.3, 0.0)}

# 시험용 판단 값: 재위치 제한시간만 줄인다.
HEALTH_CONFIG = 'reinit:\n  timeout_s: 4.0\n'


# ROS 2 실행 환경과 인터페이스 패키지가 있는지
def ros_available():
    return (shutil.which('ros2') is not None and importlib.util.find_spec('rclpy') is not None
            and importlib.util.find_spec('ax_interfaces') is not None)


# SE(2) 합성·역변환(시험용)
def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], math.atan2(math.sin(a[2] + b[2]), math.cos(a[2] + b[2])))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2])


@unittest.skipUnless(ros_available(), 'ROS 2와 ax_interfaces 필요(source install/setup.bash)')
class SupervisorWithStubs(unittest.TestCase):

    # 격리 domain에서 가짜 세계 노드와 감독 노드 두 개를 시작하고 둘 다 OK가 될 때까지 기다린다.
    @classmethod
    def setUpClass(cls):
        cls.folder = Path(tempfile.mkdtemp(prefix='ax_supervisor_'))
        domain = str(200 + os.getpid() % 30)
        os.environ.update(ROS_DOMAIN_ID=domain, ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
                          ROS_LOG_DIR=str(cls.folder / 'roslog'))
        cls.marker = uuid.uuid4().hex
        (cls.folder / 'health.yaml').write_text(HEALTH_CONFIG)

        import rclpy
        from rclpy.executors import MultiThreadedExecutor

        rclpy.init()
        cls.world = make_world(ROBOTS)
        cls.executor = MultiThreadedExecutor(num_threads=4)
        cls.executor.add_node(cls.world)
        cls.spinner = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.spinner.start()
        cls.world.start_clock()
        cls.processes = []

        for robot_id in ROBOTS:
            x, y, yaw = START[robot_id]
            command = [str(SUPERVISOR), '--ros-args', '-r', f'__ns:=/ax/{robot_id}',
                       '-p', f'robot_id:={robot_id}', '-p', f'config_file:={cls.folder / "health.yaml"}',
                       '-p', 'start_pose_enabled:=true', '-p', f'start_pose_x:={x}', '-p', f'start_pose_y:={y}',
                       '-p', f'start_pose_yaw:={yaw}', '-p', 'start_pose_position_std_m:=0.1',
                       '-p', 'start_pose_yaw_std_rad:=0.05']

            # 진단 기록: robot_1은 정상 파일, robot_2는 쓰기마다 실패하는 /dev/full(실패 주입).
            # robot_2는 진단만 꺼지고 아래 모든 판정·재위치 시험을 그대로 통과해야 한다.
            if robot_id == 'robot_1':
                command += ['-p', f'diagnostics_file:={cls.folder / "diag" / "robot_1_supervisor.jsonl"}']
            else:
                command += ['-p', 'diagnostics_file:=/dev/full']
            log = open(cls.folder / f'{robot_id}.log', 'w')
            cls.processes.append(subprocess.Popen(command, env=dict(os.environ, AX_SUPERVISOR_TEST=cls.marker),
                                                  stdout=log, stderr=subprocess.STDOUT, start_new_session=True))

        for robot_id in ROBOTS:
            if not cls.world.wait(lambda r=robot_id: cls.world.state(r) == 1, 30.0):
                raise AssertionError(f'{robot_id} OK 대기 실패: {cls.world.latest.get(robot_id)}\n' + cls.logs())

    # 감독 노드와 가짜 세계를 정리한다.
    @classmethod
    def tearDownClass(cls):
        import rclpy

        for process in cls.processes:
            if process.poll() is None:
                process.send_signal(signal.SIGTERM)

        for process in cls.processes:
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)

        cls.world.stop_clock()
        cls.executor.shutdown(timeout_sec=2.0)
        cls.world.destroy_node()
        rclpy.try_shutdown()
        shutil.rmtree(cls.folder, ignore_errors=True)

    # 감독 노드 로그(실패 진단용)
    @classmethod
    def logs(cls):
        return '\n'.join((cls.folder / f'{robot_id}.log').read_text()[-1500:] for robot_id in ROBOTS)

    # 각 시험 전에 가짜 세계를 정상으로 되돌리고 두 로봇이 OK인지 확인한다.
    # 앞 시험의 재위치 실패는 입력만 정상으로 돌려서는 지워지지 않으므로(회복 근거 필요) 재위치로 복구한다.
    def setUp(self):
        self.world.reset()

        for robot_id in ROBOTS:
            if not self.world.wait(lambda r=robot_id: self.world.state(r) == 1, 3.0) \
                    and 'REINIT_FAILED' in self.world.reason(robot_id):
                self.assertEqual(self.reinit(robot_id)[0], True)

            self.assertTrue(self.world.wait(lambda r=robot_id: self.world.state(r) == 1, 15.0),
                            f'{robot_id} 시험 전 OK 아님: {self.world.latest.get(robot_id)}\n' + self.logs())

    # 재위치 서비스를 부른다.
    def reinit(self, robot_id):
        return self.world.call_reinit(robot_id)

    # map→odom TF를 끊어 LOST(TF_MISSING)로 만든다.
    def lose(self, robot_id):
        self.world.robots[robot_id]['tf'] = False
        self.assertTrue(self.world.wait(lambda: 'TF_MISSING' in self.world.reason(robot_id), 10.0),
                        str(self.world.latest.get(robot_id)))

    # 진단 기록(선택): 켠 로봇만 tick마다 판단 근거·보낸 출발 위치를 JSON 줄로 남기고 끈 로봇은 파일이 없다.
    def test_10_diagnostics_file(self):
        import json

        path = self.folder / 'diag' / 'robot_1_supervisor.jsonl'
        self.assertTrue(self.world.wait(lambda: path.is_file() and path.stat().st_size > 0, 5.0))
        rows = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
        ticks = [row for row in rows if row['kind'] == 'tick']
        sent = [row for row in rows if row['kind'] == 'initialpose_sent']

        self.assertGreater(len(ticks), 3)
        self.assertTrue(any(row['trust_stored'] for row in ticks))
        self.assertTrue(all(row['trust_stored'] == (row['trust_rejection'] is None) for row in ticks))
        self.assertEqual([row['source'] for row in sent], ['START_POSE'])
        self.assertAlmostEqual(sent[0]['pose'][0], START['robot_1'][0])
        self.assertFalse((self.folder / 'diag' / 'robot_2_supervisor.jsonl').exists())

    # 진단 쓰기 실패(/dev/full): 오류를 남기고 진단만 끈다. 노드는 살아 health를 계속 내고 재위치 요청에 응답한다.
    def test_11_diagnostics_failure_keeps_running(self):
        self.assertIsNone(self.processes[1].poll())
        self.assertIn('진단 기록 실패로 진단만 끔', (self.folder / 'robot_2.log').read_text())
        revision = self.world.latest['robot_2'].meta.revision
        self.assertTrue(self.world.wait(lambda: self.world.state('robot_2') == 1, 5.0))
        self.assertEqual(self.reinit('robot_2'), (False, 'REINIT_REJECTED_NOT_NEEDED'))
        self.assertIsNone(self.processes[1].poll())
        self.assertGreaterEqual(self.world.latest['robot_2'].meta.revision, revision)

    # 고정 출발 위치로 초기화되고, 메시지 형식(세션·revision·이상 탐지 없음)이 계약을 따른다.
    def test_1_start_pose_and_message(self):
        for robot_id in ROBOTS:
            message = self.world.latest[robot_id]

            self.assertEqual(self.world.robots[robot_id]['initialposes'][0][:2], START[robot_id][:2])
            self.assertEqual(len(self.world.robots[robot_id]['initialposes']), 1)
            self.assertTrue(message.meta.valid)
            self.assertEqual(message.meta.validity_reason, '')
            self.assertEqual(message.robot_id, robot_id)
            self.assertGreater(message.meta.revision, 0)
            self.assertFalse(message.anomaly_model_available)
            self.assertEqual(message.anomaly_score, 0.0)

        self.assertNotEqual(self.world.latest['robot_1'].meta.source_session_id,
                            self.world.latest['robot_2'].meta.source_session_id)

    # 로봇별 분리: robot_1 스캔을 끊으면 robot_1만 LOST, robot_2는 OK 유지
    def test_2_separation(self):
        self.world.robots['robot_1']['scan'] = False

        self.assertTrue(self.world.wait(lambda: self.world.state('robot_1') == 3, 10.0))
        self.assertIn('INPUT_LOST', self.world.reason('robot_1'))
        self.assertEqual(self.world.state('robot_2'), 1)

    # 정상 복구: TF 상실 → 요청 접수(응답 전 진행 상태 발행) → 중복 요청은 같은 재위치 → 새 추정 회복 → OK.
    # 위치를 잃었다고 출발 위치를 다시 넣지 않는다.
    def test_3_reinit_success(self):
        robot = self.world.robots['robot_2']
        before = len(robot['initialposes'])
        self.lose('robot_2')
        revision = self.world.latest['robot_2'].meta.revision

        self.assertEqual(len(robot['initialposes']), before)
        self.assertEqual(self.reinit('robot_2'), (True, 'REINIT_ACCEPTED'))
        self.assertGreater(self.world.latest['robot_2'].meta.revision, revision)
        self.assertTrue(self.world.wait(lambda: 'REINIT_IN_PROGRESS' in self.world.reason('robot_2'), 2.0))
        self.assertEqual(self.reinit('robot_2'), (True, 'REINIT_IN_PROGRESS'))

        self.assertTrue(self.world.wait(lambda: self.world.state('robot_2') == 1, 10.0), self.logs())
        self.assertEqual(len(robot['initialposes']), before + 1)
        candidate = robot['initialposes'][-1]
        expected = compose(robot['map_odom_before_loss'], ODOM['robot_2'])
        self.assertAlmostEqual(candidate[0], expected[0], places=3)
        self.assertAlmostEqual(candidate[1], expected[1], places=3)
        self.assertGreaterEqual(robot['nomotion_calls'], 1)
        self.assertEqual(self.world.state('robot_1'), 1)

    # 정지 조건 미충족이면 거절하고 상태를 바꾸지 않는다.
    def test_4_reject_not_stopped(self):
        self.lose('robot_2')
        self.world.robots['robot_2']['actuation'] = 1
        time.sleep(0.3)

        self.assertEqual(self.reinit('robot_2'), (False, 'REINIT_REJECTED_NOT_STOPPED'))
        self.assertNotIn('REINIT', self.world.reason('robot_2'))

        self.world.robots['robot_2']['actuation_publish'] = False
        time.sleep(0.6)
        self.assertEqual(self.reinit('robot_2'), (False, 'REINIT_REJECTED_ACTUATION_STALE'))

    # AMCL 무응답: 접수 뒤 새 추정이 없으면 제한시간에 REINIT_FAILED_AMCL_UNAVAILABLE. 다른 로봇은 OK 유지
    def test_5_amcl_silent(self):
        self.world.robots['robot_2']['amcl'] = 'silent'
        self.lose('robot_2')

        self.assertEqual(self.reinit('robot_2'), (True, 'REINIT_ACCEPTED'))
        self.assertTrue(self.world.wait(
            lambda: 'REINIT_FAILED_AMCL_UNAVAILABLE' in self.world.reason('robot_2'), 10.0), self.logs())
        self.assertEqual(self.world.state('robot_2'), 3)
        self.assertEqual(self.world.state('robot_1'), 1)

        # 실패 뒤 TF·입력이 정상으로 돌아와도 AMCL 회복 근거가 없으면 LOST·실패 사유를 유지한다.
        self.world.robots['robot_2']['tf'] = True
        time.sleep(1.5)
        self.assertEqual(self.world.state('robot_2'), 3)
        self.assertIn('REINIT_FAILED_AMCL_UNAVAILABLE', self.world.reason('robot_2'))

    # 진행 중 정지 상태 상실 → REINIT_FAILED_NOT_STOPPED(직접 이동하지 않음: cmd_vel 발행 없음)
    def test_6_stop_lost_during(self):
        self.world.robots['robot_2']['amcl'] = 'echo'
        self.lose('robot_2')

        self.assertEqual(self.reinit('robot_2'), (True, 'REINIT_ACCEPTED'))
        self.world.robots['robot_2']['actuation'] = 1
        self.assertTrue(self.world.wait(
            lambda: 'REINIT_FAILED_NOT_STOPPED' in self.world.reason('robot_2'), 5.0), self.logs())
        self.assertEqual(self.world.cmd_vel_publishers(), [])

    # [검토 3] 재위치 중 sim time이 멈추면 UNKNOWN·valid=false·CLOCK_INVALID를 유지하고 재위치는 실패로 끝난다.
    def test_8_clock_stall_during_reinit(self):
        self.world.robots['robot_2']['amcl'] = 'echo'
        self.lose('robot_2')
        self.assertEqual(self.reinit('robot_2'), (True, 'REINIT_ACCEPTED'))

        self.world.paused = True

        try:
            self.assertTrue(self.world.wait(
                lambda: 'REINIT_FAILED_CLOCK_INVALID' in self.world.reason('robot_2'), 8.0), self.logs())
            message = self.world.latest['robot_2']
            self.assertEqual((message.state, message.meta.valid), (0, False))
            self.assertTrue(message.meta.validity_reason.startswith('CLOCK_INVALID'))
        finally:
            self.world.paused = False

    # [검토 1] odom NaN이면 LOST(ODOM_INVALID), 도약이면 LOST(ODOM_DISCONTINUOUS)이고 재위치 근거도 없다.
    # 다른 로봇은 OK 유지. robot_1을 복구하지 않으므로 마지막에 둔다.
    def test_9_odom_invalid_and_jump(self):
        robot = self.world.robots['robot_1']
        robot['odom_override'] = (float('nan'), 0.0, 0.0)
        self.assertTrue(self.world.wait(lambda: 'ODOM_INVALID' in self.world.reason('robot_1'), 5.0), self.logs())
        self.assertEqual(self.world.state('robot_1'), 3)

        robot['odom_override'] = (ODOM['robot_1'][0] + 2.0, ODOM['robot_1'][1], ODOM['robot_1'][2])
        self.assertTrue(self.world.wait(lambda: 'ODOM_DISCONTINUOUS' in self.world.reason('robot_1'), 5.0), self.logs())
        time.sleep(1.0)
        self.assertEqual(self.world.state('robot_1'), 3)
        self.assertEqual(self.reinit('robot_1'), (False, 'REINIT_REJECTED_ODOM_DISCONTINUOUS'))
        self.assertEqual(self.world.state('robot_2'), 1)

    # 입력 pose만 같은 공분산으로 돌아오면(echo) 완료하지 않고 제한시간에 TIMEOUT
    def test_7_echo_timeout(self):
        self.world.robots['robot_2']['amcl'] = 'echo'
        self.lose('robot_2')

        self.assertEqual(self.reinit('robot_2'), (True, 'REINIT_ACCEPTED'))
        self.assertTrue(self.world.wait(lambda: 'REINIT_FAILED_TIMEOUT' in self.world.reason('robot_2'), 10.0),
                        self.logs())


# [최종 검토 1] AMCL 추정이 odom보다 먼저 온 감독 노드: 최초 유효 odom이 기준이 되고, 새 AMCL 없이 기준 이상
# 움직이면 ESTIMATE_STALE이다. 앞 묶음과 별도 세계·로봇(robot_3)으로 순서를 만든다.
@unittest.skipUnless(ros_available(), 'ROS 2와 ax_interfaces 필요(source install/setup.bash)')
class AmclBeforeOdom(unittest.TestCase):

    # 시험 하나: 세계가 odom 없이 AMCL 추정·TF를 먼저 내고 감독 노드를 띄운 뒤 odom을 켠다.
    def test_estimate_stale_with_first_odom_baseline(self):
        import rclpy
        from rclpy.executors import MultiThreadedExecutor

        folder = Path(tempfile.mkdtemp(prefix='ax_supervisor_order_'))
        os.environ.update(ROS_DOMAIN_ID=str(170 + os.getpid() % 30), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
                          ROS_LOG_DIR=str(folder / 'roslog'))
        rclpy.init()
        world = make_world(('robot_3',))
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(world)
        threading.Thread(target=executor.spin, daemon=True).start()
        robot = world.robots['robot_3']
        robot['odom_publish'] = False
        robot['map_odom'] = (0.0, 0.0, 0.0)
        robot['tf'] = True
        world.start_clock()
        time.sleep(0.5)
        world.publish_amcl('robot_3', ODOM['robot_3'], 0.05, 0.03, world.sim)
        log = open(folder / 'robot_3.log', 'w')
        process = subprocess.Popen([str(SUPERVISOR), '--ros-args', '-r', '__ns:=/ax/robot_3', '-p', 'robot_id:=robot_3'],
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

        try:
            # odom이 없으니 OK가 아니다(INPUT_LOST). 그 뒤 odom을 켜면 최초 odom이 기준이 되어 OK
            self.assertTrue(world.wait(lambda: world.state('robot_3') == 3, 15.0), str(world.latest.get('robot_3')))
            robot['odom_publish'] = True
            self.assertTrue(world.wait(lambda: world.state('robot_3') == 1, 10.0), str(world.latest.get('robot_3')))

            # 새 AMCL 없이 0.01 m씩(속도로 설명되는 연속 이동) 0.7 m 이동 → ESTIMATE_STALE
            x, y, yaw = ODOM['robot_3']

            for step in range(1, 71):
                robot['odom_override'] = (x + 0.01 * step, y, yaw)
                time.sleep(0.02)

            self.assertTrue(world.wait(lambda: 'ESTIMATE_STALE' in world.reason('robot_3'), 5.0),
                            str(world.latest.get('robot_3')) + (folder / 'robot_3.log').read_text()[-1500:])
            self.assertEqual(world.state('robot_3'), 2)
            self.assertNotIn('ODOM_DISCONTINUOUS', world.reason('robot_3'))
        finally:
            process.send_signal(signal.SIGTERM)

            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)

            world.stop_clock()
            executor.shutdown(timeout_sec=2.0)
            world.destroy_node()
            rclpy.try_shutdown()
            shutil.rmtree(folder, ignore_errors=True)


# 가짜 세계 노드를 만든다(rclpy를 함수 안에서 불러 ROS 없는 환경에서도 모듈을 읽을 수 있게 한다).
def make_world(robot_ids):
    from ax_interfaces.msg import ActuationState, LocalizationHealth
    from builtin_interfaces.msg import Time as TimeMsg
    from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
    from lifecycle_msgs.msg import State
    from lifecycle_msgs.srv import GetState
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import LaserScan
    from std_srvs.srv import Empty, Trigger
    from tf2_msgs.msg import TFMessage

    state_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)

    # sim 시각(s)을 메시지 시각으로
    def stamp(seconds):
        message = TimeMsg()
        message.sec = int(seconds)
        message.nanosec = int((seconds - int(seconds)) * 1e9)
        return message

    # 쿼터니언(z, w)
    def quaternion(yaw):
        return math.sin(yaw / 2), math.cos(yaw / 2)

    # 시계·입력·가짜 AMCL·가짜 A4
    class World(Node):
        # 로봇별 상태와 발행·구독·서비스를 만든다.
        def __init__(self):
            super().__init__('ax_stub_world')
            self.lock = threading.Lock()
            self.sim = 1.0
            self.running = False
            self.paused = False
            self.latest = {}
            self.robots = {}
            self.clock_pub = self.create_publisher(Clock, '/clock', 10)
            self.tf_pub = self.create_publisher(TFMessage, '/tf', 100)
            self.map_pub = self.create_publisher(OccupancyGrid, '/map', state_qos)
            self.pubs = {}
            self.clients_reinit = {}

            for robot_id in robot_ids:
                ns = f'/ax/{robot_id}'
                self.robots[robot_id] = {}
                self.pubs[robot_id] = {
                    'odom': self.create_publisher(Odometry, f'{ns}/diff_drive_controller/odom', 50),
                    'scan': self.create_publisher(LaserScan, f'{ns}/sensors/front_lidar/scan_filtered', qos_profile_sensor_data),
                    'amcl': self.create_publisher(PoseWithCovarianceStamped, f'{ns}/amcl_pose', state_qos),
                    'actuation': self.create_publisher(ActuationState, f'{ns}/safety/actuation_state', state_qos),
                }
                self.create_subscription(PoseWithCovarianceStamped, f'{ns}/initialpose',
                                         lambda m, r=robot_id: self.on_initialpose(r, m), 10)
                self.create_subscription(LocalizationHealth, f'{ns}/localization/health',
                                         lambda m, r=robot_id: self.latest.__setitem__(r, m), state_qos)
                self.create_service(Empty, f'{ns}/request_nomotion_update',
                                    lambda req, res, r=robot_id: self.on_nomotion(r, res))
                self.create_service(GetState, f'{ns}/amcl/get_state', self.on_get_state)
                self.clients_reinit[robot_id] = self.create_client(Trigger, f'{ns}/localization/reinitialize')
                self.robots[robot_id].update(initialposes=[], map_odom=None)

            self.reset()
            grid = OccupancyGrid()
            grid.header.frame_id = 'map'
            grid.info.resolution = 0.05
            grid.info.width = grid.info.height = 4
            grid.data = [0] * 16
            self.map_pub.publish(grid)

        # 시험마다 정상 입력으로 되돌린다(초기화 기록·map→odom은 유지).
        def reset(self):
            with self.lock:
                for robot in self.robots.values():
                    robot.update(scan=True, tf=robot.get('map_odom') is not None, actuation=4,
                                 actuation_publish=True, amcl='ok', nomotion_calls=0, odom_override=None,
                                 odom_publish=True)

        # 가짜 시계와 입력 발행 스레드를 시작한다(벽시계 1 s = sim 1 s).
        def start_clock(self):
            self.running = True
            self.thread = threading.Thread(target=self.loop, daemon=True)
            self.thread.start()

        # 발행 스레드를 멈춘다.
        def stop_clock(self):
            self.running = False
            self.thread.join(timeout=2.0)

        # 10 ms마다 시계, 20 ms마다 odom·odom TF, 50 ms마다 A4, 100 ms마다 스캔·map→odom TF
        def loop(self):
            tick = 0

            while self.running:
                time.sleep(0.01)
                tick += 1

                with self.lock:
                    if self.paused:
                        continue

                    self.sim += 0.01
                    now = self.sim
                    clock = Clock()
                    clock.clock = stamp(now)
                    self.clock_pub.publish(clock)

                    for robot_id, robot in self.robots.items():
                        if tick % 2 == 0 and robot['odom_publish']:
                            self.publish_odom(robot_id, now)

                        if tick % 5 == 0 and robot['actuation_publish']:
                            self.publish_actuation(robot_id, robot['actuation'], now)

                        if tick % 10 == 0:
                            if robot['scan']:
                                scan = LaserScan()
                                scan.header.stamp = stamp(now)
                                scan.header.frame_id = f'{robot_id}/front_lidar_link'
                                self.pubs[robot_id]['scan'].publish(scan)

                            if robot['tf'] and robot['map_odom'] is not None:
                                self.publish_tf('map', f'{robot_id}/odom', robot['map_odom'], now + 1.0)

        # odom 메시지와 odom→base_link TF(정지). odom_override가 있으면 그 자세를 낸다(NaN·도약 시험).
        def publish_odom(self, robot_id, now):
            x, y, yaw = self.robots[robot_id]['odom_override'] or ODOM[robot_id]
            odom = Odometry()
            odom.header.stamp = stamp(now)
            odom.header.frame_id = f'{robot_id}/odom'
            odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
            odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = quaternion(yaw)
            self.pubs[robot_id]['odom'].publish(odom)

            if all(math.isfinite(value) for value in (x, y, yaw)):
                self.publish_tf(f'{robot_id}/odom', f'{robot_id}/base_link', (x, y, yaw), now)

        # TF 하나를 /tf로
        def publish_tf(self, parent, child, pose, at):
            transform = TransformStamped()
            transform.header.stamp = stamp(at)
            transform.header.frame_id = parent
            transform.child_frame_id = child
            transform.transform.translation.x, transform.transform.translation.y = pose[0], pose[1]
            transform.transform.rotation.z, transform.transform.rotation.w = quaternion(pose[2])
            self.tf_pub.publish(TFMessage(transforms=[transform]))

        # 가짜 A4 ActuationState
        def publish_actuation(self, robot_id, state, now):
            message = ActuationState()
            message.meta.header.stamp = stamp(now)
            message.meta.revision = 1
            message.meta.valid = True
            message.robot_id = robot_id
            message.state = state
            self.pubs[robot_id]['actuation'].publish(message)

        # 가짜 AMCL: initialpose를 받으면 map→odom을 정하고 입력 공분산 그대로 추정을 낸다(silent면 무시).
        def on_initialpose(self, robot_id, message):
            q = message.pose.pose.orientation
            pose = (message.pose.pose.position.x, message.pose.pose.position.y, 2 * math.atan2(q.z, q.w))

            with self.lock:
                robot = self.robots[robot_id]
                robot['initialposes'].append(pose)

                if robot['amcl'] == 'silent':
                    return

                if robot['map_odom'] is not None:
                    robot['map_odom_before_loss'] = robot['map_odom']

                robot['map_odom'] = compose(pose, inverse(ODOM[robot_id]))
                robot['tf'] = True
                std = math.sqrt(message.pose.covariance[0])
                self.publish_amcl(robot_id, pose, std, math.sqrt(message.pose.covariance[35]), self.sim)
                robot['echo_std'] = (std, math.sqrt(message.pose.covariance[35]))

        # 가짜 AMCL 무이동 갱신: ok면 수렴한 추정, echo면 입력 공분산 그대로, silent면 아무것도 내지 않는다.
        def on_nomotion(self, robot_id, response):
            with self.lock:
                robot = self.robots[robot_id]
                robot['nomotion_calls'] += 1

                if robot['amcl'] == 'silent' or robot['map_odom'] is None:
                    return response

                pose = compose(robot['map_odom'], ODOM[robot_id])
                std = (0.05, 0.03) if robot['amcl'] == 'ok' else robot.get('echo_std', (0.3, 0.2))
                self.publish_amcl(robot_id, pose, std[0], std[1], self.sim)

            return response

        # amcl_pose 발행
        def publish_amcl(self, robot_id, pose, std, yaw_std, at):
            message = PoseWithCovarianceStamped()
            message.header.stamp = stamp(at)
            message.header.frame_id = 'map'
            message.pose.pose.position.x, message.pose.pose.position.y = pose[0], pose[1]
            message.pose.pose.orientation.z, message.pose.pose.orientation.w = quaternion(pose[2])
            covariance = [0.0] * 36
            covariance[0] = covariance[7] = std ** 2
            covariance[35] = yaw_std ** 2
            message.pose.covariance = covariance
            self.pubs[robot_id]['amcl'].publish(message)

        # 가짜 AMCL lifecycle 상태: 항상 active
        def on_get_state(self, request, response):
            response.current_state = State(id=State.PRIMARY_STATE_ACTIVE, label='active')
            return response

        # 재위치 서비스 호출 → (success, message)
        def call_reinit(self, robot_id):
            client = self.clients_reinit[robot_id]

            if not client.wait_for_service(timeout_sec=5.0):
                raise AssertionError('재위치 서비스 없음')

            future = client.call_async(Trigger.Request())
            deadline = time.monotonic() + 5.0

            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.02)

            result = future.result()
            return result.success, result.message

        # 최신 health의 상태·사유
        def state(self, robot_id):
            message = self.latest.get(robot_id)
            return None if message is None else message.state

        def reason(self, robot_id):
            message = self.latest.get(robot_id)
            return '' if message is None else message.meta.validity_reason

        # 조건이 참이 될 때까지 기다린다.
        def wait(self, condition, timeout):
            deadline = time.monotonic() + timeout

            while time.monotonic() < deadline:
                if condition():
                    return True

                time.sleep(0.05)

            return condition()

        # 바퀴 명령 Topic에 발행자가 있는지(감독 노드가 로봇을 움직이지 않는지 확인)
        def cmd_vel_publishers(self):
            return [info.node_name for robot_id in robot_ids for info in
                    self.get_publishers_info_by_topic(f'/ax/{robot_id}/diff_drive_controller/cmd_vel')]

    return World()


if __name__ == '__main__':
    unittest.main()
