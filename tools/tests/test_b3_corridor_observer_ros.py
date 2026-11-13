# [검사 범위] 가짜 시계·TF·위치 정보로 실제 B3 ROS 관측을 검사합니다. B2 실주행 통합은 아닙니다.
# B3 통로 관측 노드(corridor_observer)를 가짜 시계·TF·health로 실제 ROS 통신에서 검사한다(Gazebo 없음).
# 이 시험 프로세스가 /clock, /tf(map→robot_2/odom, robot_2/odom→robot_2/base_link), localization/health를 흉내 내고
# 관측 노드를 격리 ROS_DOMAIN_ID(150~179)에서 실행한다. B2 AMCL·실제 주행 통합 시험이 아니다.
from copy import deepcopy
import importlib.util
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
import unittest
import uuid

ROOT = Path(__file__).resolve().parents[2]
REGIONS = ROOT / 'src/ax_environment/config/ax_warehouse_regions.json'
DESCRIPTION_CONFIG = ROOT / 'src/ax_description/config'
ROBOT = 'robot_2'

# 시험 자세(map 좌표 x, y, yaw). AX_2 외곽 x −0.63~0.67 기준, corridor_2는 map x 2.6~11.0, 서쪽 접근 구역 1.1~2.6.
POSES = {
    'outside': (0.0, 0.0, 0.0),        # 앞 끝 0.67 < 1.1: 접근 구역 밖
    'approaching': (1.2, 0.0, 0.0),    # 앞 끝 1.87: 접근 구역 안, 통로 쪽(+X)을 향함
    'inside': (5.0, 0.0, 0.0),         # 통로 안
    'exited': (13.5, 0.0, 0.0),        # 동쪽 접근 구역(11.0~12.5) 너머, 뒤 끝 12.87
    'far': (-2.0, 0.0, 0.0),           # DEGRADED 여유(0.5 m)를 더해도 앞 끝 −0.78 < 1.1: 접근 구역에서 멀다
}


# ROS 2 실행 환경과 인터페이스 패키지가 있는지
def ros_available():
    return (shutil.which('ros2') is not None and importlib.util.find_spec('rclpy') is not None
            and importlib.util.find_spec('ax_interfaces') is not None)


# 가짜 세계 노드: 시계·TF·health를 내고 관측 목록을 받는다.
def make_world():
    from ax_interfaces.msg import CorridorObservationList, LocalizationHealth
    from geometry_msgs.msg import TransformStamped
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from tf2_msgs.msg import TFMessage

    state_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)

    class World(Node):
        # 발행자·구독자와 상태(시각, 자세, health 상태)를 만든다.
        def __init__(self):
            super().__init__('b3_test_world')
            self.sim = 1.0
            self.pose = POSES['outside']
            self.health_state = LocalizationHealth.OK
            self.health_on = True
            self.position_std = 0.02
            self.yaw_std = 0.005
            self.session = uuid.uuid4().hex
            self.health_revision = 1
            self.lock = threading.Lock()
            self.received = []
            self.health_times = []
            self.clock = self.create_publisher(Clock, '/clock', 10)
            self.tf = self.create_publisher(TFMessage, '/tf', 100)
            self.health = self.create_publisher(LocalizationHealth, f'/ax/{ROBOT}/localization/health', state_qos)
            self.create_subscription(CorridorObservationList, f'/ax/{ROBOT}/perception/corridor_observation',
                                     self.on_observation, state_qos)
            self.tf_count = 0
            # 시계·TF·health는 executor 타이머가 아니라 전용 스레드로 낸다(부하 때 executor 지연으로 시계가 멈추지 않게).
            self.running = True
            self.publisher_thread = threading.Thread(target=self.run, daemon=True)
            self.publisher_thread.start()

        # 벽시계 100 Hz로 sim 시각 0.01 s씩(RTF 1) 진행한다. 늦어지면 다음 마감 시각에 맞춘다.
        def run(self):
            deadline = time.monotonic()
            while self.running:
                self.tick()
                deadline += 0.01
                time.sleep(max(0.0, deadline - time.monotonic()))

        # 관측 수신 기록(벽시계 시각과 함께)
        def on_observation(self, message):
            with self.lock:
                self.received.append((time.monotonic(), message))

        # 시계·TF(20 Hz)·health(5 Hz)를 낸다.
        def tick(self):
            from builtin_interfaces.msg import Time as TimeMsg

            self.sim += 0.01
            stamp = TimeMsg(sec=int(self.sim), nanosec=int((self.sim % 1) * 1e9))
            self.clock.publish(Clock(clock=stamp))
            self.tf_count += 1

            if self.tf_count % 5 == 0:
                x, y, yaw = self.pose
                map_odom = TransformStamped()
                map_odom.header.stamp = stamp
                map_odom.header.frame_id = 'map'
                map_odom.child_frame_id = f'{ROBOT}/odom'
                map_odom.transform.rotation.w = 1.0
                odom_base = TransformStamped()
                odom_base.header.stamp = stamp
                odom_base.header.frame_id = f'{ROBOT}/odom'
                odom_base.child_frame_id = f'{ROBOT}/base_link'
                odom_base.transform.translation.x = x
                odom_base.transform.translation.y = y
                odom_base.transform.rotation.z = math.sin(yaw / 2)
                odom_base.transform.rotation.w = math.cos(yaw / 2)
                self.tf.publish(TFMessage(transforms=[map_odom, odom_base]))

            if self.tf_count % 20 == 0 and self.health_on:
                message = LocalizationHealth()
                message.meta.header.stamp = stamp
                message.meta.schema_version = 1
                message.meta.source_id = 'b3_test_health_stub'
                message.meta.source_session_id = self.session
                message.meta.revision = self.health_revision
                message.meta.valid = True
                message.robot_id = ROBOT
                message.state = self.health_state
                message.position_std_m = self.position_std
                message.yaw_std_rad = self.yaw_std
                self.health.publish(message)
                self.health_times.append(time.monotonic())

        # 다음 상태가 올 때까지 기다린 뒤 마지막 메시지를 돌려준다.
        def wait_for(self, predicate, timeout=6.0):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                with self.lock:
                    last = self.received[-1][1] if self.received else None
                if last is not None and predicate(last):
                    return last
                time.sleep(0.05)
            return last

    return World()


# 목록 메시지 → {corridor_id: (observation, position_valid)}
def table(message):
    return {o.corridor_id: (o.observation, o.position_valid) for o in message.observations}


@unittest.skipUnless(ros_available(), 'ROS 2와 ax_interfaces 필요(source install/setup.bash)')
class CorridorObserverWithStubs(unittest.TestCase):

    # 격리 domain에서 가짜 세계와 관측 노드 두 개(승인 사본·실제 미승인 파일)를 시작한다.
    @classmethod
    def setUpClass(cls):
        cls.folder = Path(tempfile.mkdtemp(prefix='ax_b3_observer_'))
        domain = str(150 + os.getpid() % 30)
        os.environ.update(ROS_DOMAIN_ID=domain, ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
                          ROS_LOG_DIR=str(cls.folder / 'roslog'))

        # 시험 전용 승인 사본(실제 파일의 미승인 표시는 바꾸지 않는다)
        data = deepcopy(json.loads(REGIONS.read_text(encoding='utf-8')))
        data['approved'] = True
        cls.approved = cls.folder / 'approved_regions.json'
        cls.approved.write_text(json.dumps(data))

        import rclpy
        from rclpy.executors import MultiThreadedExecutor

        rclpy.init()
        cls.world = make_world()
        cls.executor = MultiThreadedExecutor(num_threads=3)
        cls.executor.add_node(cls.world)
        cls.spinner = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.spinner.start()

        env = dict(os.environ, PYTHONPATH=f'{ROOT / "src/ax_environment"}:{os.environ.get("PYTHONPATH", "")}')
        cls.processes = []
        for name, regions in (('corridor_observer', cls.approved), ('corridor_observer_unapproved', REGIONS)):
            namespace = f'/ax/{ROBOT}' if name == 'corridor_observer' else f'/ax_unapproved/{ROBOT}'
            command = [sys.executable, '-m', 'ax_environment.corridor_observer', '--ros-args',
                       '-r', f'__ns:={namespace}', '-r', f'__node:={name}',
                       '-r', f'{namespace}/localization/health:=/ax/{ROBOT}/localization/health',
                       '-p', f'robot_id:={ROBOT}', '-p', 'robot_model:=AX_2', '-p', f'regions_file:={regions}',
                       '-p', f'description_config_dir:={DESCRIPTION_CONFIG}', '-p', 'use_sim_time:=true']
            log = open(cls.folder / f'{name}.log', 'w')
            cls.processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, env=env,
                                                  start_new_session=True))

    # 자기 프로세스 그룹만 정리한다.
    @classmethod
    def tearDownClass(cls):
        cls.world.running = False
        cls.world.publisher_thread.join(timeout=2)
        for process in cls.processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
        # 실행기 스레드가 끝난 뒤 노드·rclpy를 닫는다(스레드가 남은 채 종료하면 C++ 쪽에서 abort가 날 수 있다).
        cls.executor.shutdown(timeout_sec=5)
        cls.spinner.join(timeout=5)
        cls.world.destroy_node()
        import rclpy
        rclpy.try_shutdown()

    # 자세를 바꾸고 corridor_2 상태가 기대값이 될 때까지 기다린다.
    def move_and_wait(self, pose_name, expected):
        self.world.pose = POSES[pose_name]
        message = self.world.wait_for(lambda m: table(m).get('corridor_2', (None,))[0] == expected and m.meta.valid)
        self.assertIsNotNone(message)
        self.assertEqual(table(message).get('corridor_2', (None,))[0], expected, pose_name)
        return message

    # 1) OUTSIDE→APPROACHING→INSIDE→OUTSIDE 전이, 2) LOST·만료 UNKNOWN, 3) 회복, 4) revision·주기·QoS
    def test_transitions_health_and_rate(self):
        # [핵심] 가짜 위치를 바꾸며 관측 전이·만료·회복·재발행 주기와 revision을 함께 확인합니다.
        from ax_interfaces.msg import CorridorObservation as O
        from ax_interfaces.msg import LocalizationHealth as H

        # 1) 전이
        first = self.move_and_wait('outside', O.OUTSIDE)
        self.assertEqual(set(table(first)), {'corridor_1', 'corridor_2', 'corridor_3'})
        self.assertEqual(table(first)['corridor_2'], (O.OUTSIDE, True))
        self.assertEqual(first.meta.source_id, 'b3_corridor_observer')
        self.assertEqual(first.robot_id, ROBOT)

        approaching = self.move_and_wait('approaching', O.APPROACHING)
        self.assertEqual(table(approaching)['corridor_1'][0], O.OUTSIDE)
        self.assertGreater(approaching.meta.revision, first.meta.revision)
        self.assertEqual(approaching.meta.source_session_id, first.meta.source_session_id)

        inside = self.move_and_wait('inside', O.INSIDE)
        self.assertTrue(table(inside)['corridor_2'][1])
        exited = self.move_and_wait('exited', O.OUTSIDE)
        self.assertEqual({v for v in table(exited).values()}, {(O.OUTSIDE, True)})

        # 2) 주기(5 Hz)와 같은 내용 재발행 시 revision 유지
        with self.world.lock:
            start = len(self.world.received)
        time.sleep(3.0)
        with self.world.lock:
            batch = [m for _, m in self.world.received[start:]]
        rate = len(batch) / 3.0
        self.assertTrue(4.0 <= rate <= 6.0, rate)
        gaps = [b - a for a, b in zip(self.world.health_times, self.world.health_times[1:])]
        detail = (round(max(gaps), 3), sorted({(m.meta.revision, m.meta.validity_reason, tuple(sorted(table(m).items()))) for m in batch}))
        self.assertEqual({m.meta.revision for m in batch}, {exited.meta.revision}, (exited.meta.revision, detail))
        stamps = [m.meta.header.stamp.sec + m.meta.header.stamp.nanosec * 1e-9 for m in batch]
        self.assertTrue(all(b > a for a, b in zip(stamps, stamps[1:])), '재발행마다 stamp 갱신')

        # 3) health LOST → 모든 통로 UNKNOWN, position_valid=false, meta.valid=false
        self.world.health_state = H.LOST
        self.world.health_revision += 1
        lost = self.world.wait_for(lambda m: m.meta.validity_reason == 'HEALTH_LOST')
        self.assertEqual(lost.meta.validity_reason, 'HEALTH_LOST')
        self.assertFalse(lost.meta.valid)
        self.assertEqual({v for v in table(lost).values()}, {(O.UNKNOWN, False)})
        self.assertTrue(all(not o.meta.valid for o in lost.observations))

        # DEGRADED 완화: 목록 valid·사유 없음. 넓힌 외곽(+0.5 m)이 통로·접근 구역과 겹치는 통로만 UNKNOWN.
        # 'exited'(13.5): 뒤 끝 12.87 − 0.54 < 12.5 → corridor_2 동쪽 접근 구역과 겹침 → UNKNOWN, 나머지 통로는 OUTSIDE.
        self.world.health_state = H.DEGRADED
        self.world.health_revision += 1
        expected = {'corridor_1': (O.OUTSIDE, True), 'corridor_2': (O.UNKNOWN, False), 'corridor_3': (O.OUTSIDE, True)}
        degraded = self.world.wait_for(lambda m: m.meta.valid and table(m) == expected)
        self.assertEqual(table(degraded), expected)
        self.assertEqual(degraded.meta.validity_reason, '')

        # 멀리 떨어진 위치는 모든 통로 OUTSIDE·position_valid=true
        self.world.pose = POSES['far']
        far = self.world.wait_for(lambda m: m.meta.valid and set(table(m).values()) == {(O.OUTSIDE, True)})
        self.assertEqual(set(table(far).values()), {(O.OUTSIDE, True)})

        # 통로 안은 INSIDE를 확정하지 않고 UNKNOWN
        self.world.pose = POSES['inside']
        inside_degraded = self.world.wait_for(lambda m: m.meta.valid and table(m)['corridor_2'] == (O.UNKNOWN, False))
        self.assertEqual(table(inside_degraded)['corridor_2'], (O.UNKNOWN, False))
        self.assertEqual(table(inside_degraded)['corridor_1'], (O.OUTSIDE, True))

        # std 비정상(0·과대·NaN)은 LOST와 같게 모든 통로 UNKNOWN
        self.world.pose = POSES['far']
        for position_std, yaw_std in ((0.0, 0.005), (1.5, 0.005), (0.02, 0.8), (float('nan'), 0.005)):
            self.world.position_std, self.world.yaw_std = position_std, yaw_std
            self.world.health_revision += 1
            bad = self.world.wait_for(lambda m: m.meta.validity_reason == 'HEALTH_STD_INVALID')
            with self.subTest(position_std=position_std, yaw_std=yaw_std):
                self.assertEqual(bad.meta.validity_reason, 'HEALTH_STD_INVALID')
                self.assertFalse(bad.meta.valid)
                self.assertEqual(set(table(bad).values()), {(O.UNKNOWN, False)})
            self.world.position_std, self.world.yaw_std = 0.02, 0.005
            self.world.health_revision += 1
            self.world.wait_for(lambda m: m.meta.valid and set(table(m).values()) == {(O.OUTSIDE, True)})

        # 4) OK 복귀 후 health 중단 → 만료(1.0 s) 뒤 HEALTH_STALE
        self.world.health_state = H.OK
        self.world.health_revision += 1
        self.move_and_wait('inside', O.INSIDE)
        self.world.health_on = False
        stopped = time.monotonic()
        stale = self.world.wait_for(lambda m: m.meta.validity_reason == 'HEALTH_STALE', timeout=4.0)
        self.assertEqual(stale.meta.validity_reason, 'HEALTH_STALE')
        self.assertLess(time.monotonic() - stopped, 2.5)
        self.assertEqual({v for v in table(stale).values()}, {(O.UNKNOWN, False)})

        # 5) health 재개 → 다시 판정
        self.world.health_on = True
        self.move_and_wait('outside', O.OUTSIDE)

        # 6) 같은 세션 revision 감소(LOST)와 대체된 이전 세션 메시지(LOST)는 버린다.
        #    버린 메시지는 health를 갱신하지 않으므로 사유는 HEALTH_LOST가 아니라 만료(HEALTH_STALE)가 된다.
        old_session = self.world.session
        self.world.health_state, self.world.health_revision = H.LOST, 1
        ignored = self.world.wait_for(lambda m: m.meta.validity_reason in ('HEALTH_STALE', 'HEALTH_LOST'), timeout=4.0)
        self.assertEqual(ignored.meta.validity_reason, 'HEALTH_STALE', '같은 세션 revision 감소 무시')

        self.world.health_state, self.world.health_revision = H.OK, 1
        self.world.session = uuid.uuid4().hex
        self.move_and_wait('outside', O.OUTSIDE)

        self.world.health_state, self.world.session, self.world.health_revision = H.LOST, old_session, 99
        ignored = self.world.wait_for(lambda m: m.meta.validity_reason in ('HEALTH_STALE', 'HEALTH_LOST'), timeout=4.0)
        self.assertEqual(ignored.meta.validity_reason, 'HEALTH_STALE', '대체된 이전 세션 무시')

    # 미승인 실제 파일: meta.valid=false, REGIONS_NOT_APPROVED, 모든 통로 UNKNOWN·position_valid=false.
    # 늦게 접속한 구독자도 마지막 값을 받는다(STATE transient_local).
    def test_unapproved_regions_and_late_join(self):
        # [핵심] 미승인 영역은 UNKNOWN이어야 하고, 늦게 구독한 프로그램도 최근 상태를 받아야 합니다.
        from ax_interfaces.msg import CorridorObservation as O
        from ax_interfaces.msg import CorridorObservationList
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

        time.sleep(1.5)
        got = []
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        subscription = self.world.create_subscription(
            CorridorObservationList, f'/ax_unapproved/{ROBOT}/perception/corridor_observation', got.append, qos)
        deadline = time.monotonic() + 5.0
        while not got and time.monotonic() < deadline:
            time.sleep(0.05)
        self.world.destroy_subscription(subscription)

        self.assertTrue(got, '미승인 노드 관측 미수신')
        message = got[-1]
        self.assertFalse(message.meta.valid)
        self.assertEqual(message.meta.validity_reason, 'REGIONS_NOT_APPROVED')
        self.assertEqual(set(table(message)), {'corridor_1', 'corridor_2', 'corridor_3'})
        self.assertEqual({v for v in table(message).values()}, {(O.UNKNOWN, False)})


if __name__ == '__main__':
    unittest.main()
