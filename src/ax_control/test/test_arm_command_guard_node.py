"""
arm_command_guard 노드를 Gazebo 없이 가짜 내부 JTC·가짜 관절 상태로 시험한다.

같은 프로세스에서 보호 노드와 시험 노드를 여러 스레드 실행기로 돌린다. 다른 시험·실행과 섞이지
않도록 시험 전용 ROS_DOMAIN_ID와 localhost 탐색 범위를 쓴다.
"""
import json
import math
import os
import threading
import time
import unittest

os.environ['ROS_DOMAIN_ID'] = str(140 + os.getpid() % 50)
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

from action_msgs.msg import GoalInfo, GoalStatus  # noqa: E402
from action_msgs.srv import CancelGoal  # noqa: E402
from ax_control.arm_command_guard import ArmCommandGuard  # noqa: E402
from ax_control.model_contracts import ARM_JOINTS  # noqa: E402
from builtin_interfaces.msg import Duration  # noqa: E402
from control_msgs.action import FollowJointTrajectory  # noqa: E402
import rclpy  # noqa: E402
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile  # noqa: E402
from rclpy.task import Future  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint  # noqa: E402

LIMITS = {'column_yaw_joint': (-3.14159, 3.14159, 0.35), 'shoulder_joint': (-1.5708, 1.5708, 0.5),
          'elbow_joint': (-2.0, 2.0, 0.5), 'wrist_joint': (-1.57, 1.57, 0.7),
          'gripper_rotation_joint': (-3.14159, 3.14159, 0.7)}


# 시험용 URDF 문자열(팔 5축 limit만 포함).
def urdf():
    joints = ''.join(f'<joint name="{n}" type="revolute"><limit lower="{lo}" upper="{hi}" '
                     f'effort="10" velocity="{v}"/></joint>' for n, (lo, hi, v) in LIMITS.items())
    return f'<robot name="t">{joints}</robot>'


# 조건이 참이 될 때까지 기다린다.
def wait_for(predicate, timeout=5.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.02)
    return predicate()


# 가짜 내부 JTC와 관절 상태·robot_description 발행, 공개 입력 클라이언트를 가진 시험 노드.
class Harness(Node):
    # 가짜 내부 action 서버와 발행자·구독자를 만든다.
    def __init__(self):
        super().__init__('guard_test_harness')
        group = ReentrantCallbackGroup()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.goals = []            # 내부 JTC가 받은 목표(궤적)
        self.outcomes = []         # 가짜 JTC가 끝낸 목표의 (번호, 결과) 기록
        self.accept_goals = True   # False면 내부 목표를 수락 거부한다
        self.behavior = 'wait'     # 'wait': 기다림, 'abort': 즉시 중단, 'succeed': 즉시 성공
        self.finish = threading.Event()
        self.state = ([0.0] * 5, [0.0] * 5)   # (위치, 속도): 한 번에 바꿔 섞인 메시지를 막는다
        self.publishing = True
        self.events = []

        self.create_publisher(String, 'robot_description', latched).publish(String(data=urdf()))
        self.joints = self.create_publisher(JointState, 'joint_states', 10)
        self.topic = self.create_publisher(JointTrajectory, 'arm_controller/joint_trajectory', 10)
        # 실행기 타이머는 시험 중 수 초씩 밀리는 일이 있어(관측 2.9 s) 별도 스레드에서 100 Hz로 발행한다.
        self.stopped = threading.Event()
        threading.Thread(target=self.publish_loop, daemon=True).start()
        self.create_subscription(String, 'arm_command_guard/events',
                                 lambda m: self.events.append(json.loads(m.data)), 100)

        internal = 'arm_controller/internal/follow_joint_trajectory'
        ActionServer(self, FollowJointTrajectory, internal,
                     execute_callback=self.fake_execute,
                     goal_callback=lambda g: (GoalResponse.ACCEPT if self.accept_goals
                                              else GoalResponse.REJECT),
                     cancel_callback=lambda g: CancelResponse.ACCEPT, callback_group=group)
        self.public = ActionClient(self, FollowJointTrajectory,
                                   'arm_controller/follow_joint_trajectory', callback_group=group)
        self.clear = self.create_client(Trigger, 'arm_command_guard/clear_fault',
                                        callback_group=group)
        # 보호 노드가 아닌 쪽에서 내부 목표를 취소하는 경우를 흉내 내는 취소 서비스 클라이언트.
        self.cancel_all = self.create_client(CancelGoal, internal + '/_action/cancel_goal',
                                             callback_group=group)

    # 정리 전까지 0.01 s마다 관절 상태를 발행한다.
    def publish_loop(self):
        while not self.stopped.wait(0.01):
            self.publish_state()

    # 설정한 위치·속도로 관절 상태를 발행한다(발행 중단 가능).
    def publish_state(self):
        if not self.publishing:
            return
        positions, velocities = self.state
        msg = JointState(name=list(ARM_JOINTS), position=positions,
                         velocity=velocities, effort=[0.0] * 5)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.joints.publish(msg)

    # 가짜 JTC 실행: behavior에 따라 즉시 끝내거나, 취소되거나 finish가 설정되거나 다른 목표에
    # 선점될 때까지 기다린다.
    def fake_execute(self, goal_handle):
        self.goals.append(goal_handle.request.trajectory)
        number = len(self.goals)
        outcome = self._run_goal(goal_handle, number)
        self.outcomes.append((number, outcome))
        return FollowJointTrajectory.Result(error_string=outcome)

    # 가짜 목표 하나를 끝낼 때까지 진행하고 결과 종류를 돌려준다.
    def _run_goal(self, goal_handle, number):
        if self.behavior == 'abort':
            goal_handle.abort()
            return 'aborted'
        if self.behavior == 'succeed':
            goal_handle.succeed()
            return 'done'

        while True:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return 'canceled'
            if self.finish.is_set() and number == len(self.goals):
                goal_handle.succeed()
                return 'done'
            if number != len(self.goals):
                goal_handle.abort()
                return 'preempted'
            time.sleep(0.01)

    # 이벤트 중 종류가 맞는 것을 돌려준다.
    def kinds(self, kind):
        return [e for e in self.events if e['kind'] == kind]

    # 받은 내부 목표 중 마지막 점의 어깨 목표가 value인 것이 있는지 돌려준다.
    def has_goal(self, value):
        return any(abs(g.points[-1].positions[list(g.joint_names).index('shoulder_joint')]
                       - value) < 1e-6 for g in list(self.goals))

    # 받은 내부 목표들의 어깨 목표 목록(실패 진단용).
    def targets(self):
        return [round(g.points[-1].positions[list(g.joint_names).index('shoulder_joint')], 4)
                for g in list(self.goals)]


# 보호 노드가 기록한 관절 상태의 위치·속도가 모두 유한한지 돌려준다.
def logic_finite(state):
    return all(math.isfinite(x) for x in list(state[0]) + list(state[1]))


# 어깨 목표 궤적 메시지를 만든다.
def trajectory(shoulder, seconds=8):
    msg = JointTrajectory(joint_names=['shoulder_joint'])
    msg.points = [JointTrajectoryPoint(positions=[float(shoulder)],
                                       time_from_start=Duration(sec=seconds))]
    return msg


# 보호 노드와 가짜 환경을 한 번 띄워 시험마다 상태를 초기화하며 재사용한다.
class GuardNodeTest(unittest.TestCase):
    # ROS를 초기화하고 두 노드를 각자의 백그라운드 실행기로 돌린 뒤 보호 노드 준비를 기다린다.
    # 운영과 같게 보호 노드는 스레드 4개짜리 자기 실행기를 쓴다. 가짜 JTC의 실행 콜백이 목표마다
    # 스레드를 오래 잡으므로, 같은 실행기를 쓰면 발행 타이머가 밀려 상태 만료가 잘못 날 수 있다.
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.harness = Harness()
        # 상태 만료 기준은 운영값(0.5 s)을 그대로 쓴다. 정지 확인 제한시간만 시험 시간을 줄이려고
        # 0.5 s로 둔다(운영값 1.0 s, config/arm_command_guard.yaml).
        cls.guard = ArmCommandGuard(
            parameter_overrides=[Parameter('stop_confirm_timeout_s', value=0.5)])
        cls.executors = [MultiThreadedExecutor(num_threads=16),
                         MultiThreadedExecutor(num_threads=4)]
        cls.executors[0].add_node(cls.harness)
        cls.executors[1].add_node(cls.guard)
        for executor in cls.executors:
            threading.Thread(target=executor.spin, daemon=True).start()
        assert wait_for(lambda: cls.guard.public_ready, 15), 'guard did not become ready'
        assert cls.harness.public.wait_for_server(timeout_sec=10)

    # 진행 중 가짜 목표를 끝내 실행 콜백이 모두 빠져나오게 한 뒤 실행기를 멈추고 노드를 정리한다.
    @classmethod
    def tearDownClass(cls):
        cls.harness.finish.set()
        cls.harness.stopped.set()
        time.sleep(0.5)
        for executor in cls.executors:
            executor.shutdown(timeout_sec=5)
        cls.guard.destroy_node()
        cls.harness.destroy_node()
        rclpy.try_shutdown()

    # 시험마다 관절 상태·가짜 JTC·보호 노드 모드를 처음 상태로 돌린다.
    # 진행 중 공개 목표는 보호 노드의 중단 경로로 끝내고, 남은 내부 목표가 끝날 때까지 기다린다.
    # 시험 중 바꾼 보호 노드 메서드(동기화·고장 주입용)는 addCleanup으로 되돌린다.
    def setUp(self):
        h, g = self.harness, self.guard
        h.state, h.publishing = ([0.0] * 5, [0.0] * 5), True
        h.accept_goals, h.behavior = True, 'wait'
        with g.lock:
            g._interrupt_public('test reset')
        h.finish.set()
        assert wait_for(lambda: g.internal is None or g.internal['result'] is not None
                        or g.internal.get('closed'))
        assert wait_for(lambda: g.state is not None and g.state[0] == [0.0] * 5)
        time.sleep(0.1)
        h.finish.clear()
        with g.lock:
            g.mode, g.mode_reason, g.resume_mode, g.fault_reason = 'ready', 'test', 'ready', ''
            g.epoch += 1
            g.internal = None
        h.goals.clear()
        h.events.clear()
        h.outcomes.clear()

    # 보호 노드 인스턴스의 메서드를 시험 동안만 바꾼다.
    def patch(self, owner, name, replacement):
        setattr(owner, name, replacement)
        self.addCleanup(delattr, owner, name)

    # 어깨가 0.5 rad/s로 하한에 접근하는 상태를 내 보호 정지가 시작될 때까지 기다린다.
    def trigger_protection(self):
        self.harness.state = ([0.0, -1.30, 0.0, 0.0, 0.0], [0.0, -0.5, 0.0, 0.0, 0.0])
        self.assertTrue(wait_for(lambda: self.guard.mode != 'ready'), self.guard.mode)

    # 보호 정지 실패가 고장 잠금이 되고, 사유가 기록되며, 일반 목표·해제가 막히는지 확인한다.
    def assert_stop_failed(self, expected):
        h, g = self.harness, self.guard
        self.assertTrue(wait_for(lambda: g.mode == 'fault', 4), (g.mode, g.mode_reason))
        self.assertIn('protective stop failed', g.fault_reason)
        self.assertIn(expected, g.fault_reason)
        self.assertTrue(wait_for(lambda: h.kinds('stop_failed')), h.events[-5:])
        self.assertFalse(h.kinds('protected_hold'))

        h.state = ([0.0, -1.40, 0.0, 0.0, 0.0], [0.0] * 5)
        self.assertTrue(wait_for(lambda: g.state[0][1] == -1.40))
        refused, _ = self.send_action(trajectory(-1.0))
        self.assertFalse(refused.accepted)
        self.assertEqual(g.mode, 'fault')

    # 공개 action 목표를 보내고 (goal handle, 결과 future)를 돌려준다.
    def send_action(self, msg):
        goal = FollowJointTrajectory.Goal(trajectory=msg)
        future = self.harness.public.send_goal_async(goal)
        assert wait_for(future.done)
        handle = future.result()
        return handle, (handle.get_result_async() if handle.accepted else None)

    # 토픽: 정상 명령은 내부 목표로 전달되고 범위 밖 명령은 사유와 함께 거부되는지 확인한다.
    def test_topic_is_checked(self):
        h = self.harness
        h.topic.publish(trajectory(-1.0))
        self.assertTrue(wait_for(lambda: h.has_goal(-1.0)), h.targets())

        h.topic.publish(trajectory(-1.56))
        self.assertTrue(wait_for(lambda: h.kinds('rejected')))
        self.assertIn('outside command range', h.kinds('rejected')[0]['reason'])
        time.sleep(0.2)
        self.assertFalse(h.has_goal(-1.56), h.targets())

    # action: 정상 목표는 내부 결과까지 이어지고, 잘못된 목표는 수락 전에 거부되는지 확인한다.
    def test_action_is_checked(self):
        handle, result = self.send_action(trajectory(-1.0))
        self.assertTrue(handle.accepted)
        self.assertTrue(wait_for(lambda: self.harness.has_goal(-1.0)), self.harness.targets())
        self.harness.finish.set()
        self.assertTrue(wait_for(result.done))
        self.assertEqual(result.result().status, GoalStatus.STATUS_SUCCEEDED)

        bad, _ = self.send_action(trajectory(1.56))
        self.assertFalse(bad.accepted)
        self.assertTrue(wait_for(lambda: self.harness.kinds('rejected')))
        self.assertIn('outside command range', self.harness.kinds('rejected')[-1]['reason'])

    # 새 목표가 이전 목표를 선점하고, 내부 목표는 항상 하나만 진행되는지 확인한다.
    def test_new_goal_preempts_old(self):
        first, first_result = self.send_action(trajectory(-0.5))
        second, second_result = self.send_action(trajectory(-0.8))
        self.assertTrue(wait_for(first_result.done))
        self.assertEqual(first_result.result().status, GoalStatus.STATUS_ABORTED)
        self.assertIn('preempted', first_result.result().result.error_string)
        self.assertTrue(wait_for(lambda: self.harness.has_goal(-0.8)), self.harness.targets())
        self.assertEqual(self.guard.internal['owner'], 'action')

    # 공개 목표 취소는 내부 목표를 취소해 유지하고 결과가 취소로 끝나는지 확인한다.
    def test_cancel_holds(self):
        handle, result = self.send_action(trajectory(-0.5))
        self.assertTrue(wait_for(lambda: self.harness.has_goal(-0.5)), self.harness.targets())
        handle.cancel_goal_async()
        self.assertTrue(wait_for(result.done))
        self.assertEqual(result.result().status, GoalStatus.STATUS_CANCELED)
        self.assertTrue(wait_for(lambda: self.guard.internal['result'] is not None))

    # 내부 목표의 수락 응답 전에 유지(취소)를 요청해도 수락 즉시 내부 목표가 취소되는지 확인한다.
    def test_hold_before_internal_acceptance(self):
        with self.guard.lock:
            point = {'time': 8.0, 'positions': [0.0, -0.5, 0.0, 0.0, 0.0],
                     'velocities': [0.0] * 5, 'accelerations': []}
            record = self.guard._send_internal([point], 'action')
            self.guard._hold()                       # 아직 handle이 없는 상태
            self.assertTrue(record['cancel_pending'])
        self.assertTrue(wait_for(lambda: record['result'] is not None))
        self.assertEqual(record['result']['status'], GoalStatus.STATUS_CANCELED)

    # 한계 쪽 접근은 보호 정지로 바뀌고, 그동안 새 목표는 거부되며, 정지 후 안쪽 명령은 받는지 확인한다.
    def test_protective_stop_then_inward(self):
        h = self.harness
        handle, result = self.send_action(trajectory(-1.5))
        self.assertTrue(wait_for(lambda: h.has_goal(-1.5)), h.targets())

        h.state = ([0.0, -1.30, 0.0, 0.0, 0.0], [0.0, -0.5, 0.0, 0.0, 0.0])
        self.assertTrue(wait_for(result.done))
        self.assertIn('protective stop', result.result().result.error_string)
        # 정지 목표 = -1.30 + (-0.5) × 1.0 s / 2 = -1.55
        self.assertTrue(wait_for(lambda: h.has_goal(-1.55)), h.targets())
        stop = [g for g in h.goals if abs(g.points[-1].positions[1] + 1.55) < 1e-6][-1].points[-1]
        self.assertEqual(list(stop.velocities), [0.0] * 5)
        self.assertEqual(self.guard.mode, 'protecting')

        blocked, _ = self.send_action(trajectory(-1.0))
        self.assertFalse(blocked.accepted)

        # 보호 노드가 정지한 상태를 실제로 받은 뒤에 정지 완료를 알린다(실제 로봇은 정지 후 멈춰 있음).
        h.state = ([0.0, -1.55, 0.0, 0.0, 0.0], [0.0] * 5)
        self.assertTrue(wait_for(lambda: self.guard.state[1] == [0.0] * 5))
        time.sleep(0.05)
        h.finish.set()
        self.assertTrue(wait_for(lambda: self.guard.mode == 'protected_hold'))
        h.finish.clear()
        h.goals.clear()

        inward, inward_result = self.send_action(trajectory(-1.0))
        recent = [(e['kind'], e.get('reason')) for e in h.events if e['kind'] != 'accepted'][-4:]
        self.assertTrue(inward.accepted, recent)
        self.assertTrue(wait_for(lambda: h.has_goal(-1.0)), h.targets())

    # 이미 처리한 것보다 조금(만료 기준 이내) 오래된 관절 상태는 순서가 뒤바뀐 메시지로 보고
    # 보호 판단에 쓰지 않는지 확인한다.
    def test_older_joint_state_is_ignored(self):
        h = self.harness
        newest = self.guard.last_stamp
        self.assertIsNotNone(newest)

        # 하한 쪽으로 빠르게 움직이는 상태이지만 시각이 0.1 s 과거인 메시지를 직접 넣는다.
        old = JointState(name=list(ARM_JOINTS), position=[0.0, -1.30, 0.0, 0.0, 0.0],
                         velocity=[0.0, -0.5, 0.0, 0.0, 0.0], effort=[0.0] * 5)
        stamp = newest - 100_000_000
        old.header.stamp.sec, old.header.stamp.nanosec = divmod(stamp, 10**9)
        self.guard.on_joint_states(old)
        self.assertEqual(self.guard.mode, 'ready')
        self.assertFalse(h.kinds('protect'))

    # 관절 속도 한계를 크게 넘는 비정상 표본으로는 정지 궤적을 만들지 않고 상태 이상으로 처리하며,
    # 정상 표본이 다시 오면 돌아오는지 확인한다(Gazebo 제어기 활성 직후 손목 2.10 rad/s 관측 사례).
    def test_implausible_velocity_sample_is_not_used(self):
        h, g = self.harness, self.guard
        h.state = ([0.0, 0.0, 0.0, 0.002, 0.0], [0.0, -0.3, -0.88, 2.10, 0.0])
        self.assertTrue(wait_for(lambda: g.mode == 'state_invalid'), g.mode)
        self.assertIn('plausible bound', g.mode_reason)
        self.assertEqual(h.goals, [])
        self.assertFalse(h.kinds('protect'))

        h.state = ([0.0] * 5, [0.0] * 5)
        self.assertTrue(wait_for(lambda: g.mode == 'ready'), (g.mode, g.mode_reason))

    # 발행 쪽 시계가 만료 기준보다 크게 되돌아가면 새 상태로 받아들이고 기록하는지 확인한다
    # (버리면 시계가 따라잡을 때까지 상태 만료가 된다).
    def test_large_stamp_jump_back_is_accepted(self):
        g = self.guard
        jumped = JointState(name=list(ARM_JOINTS), position=[0.0, -0.2, 0.0, 0.0, 0.0],
                            velocity=[0.0] * 5, effort=[0.0] * 5)
        jumped.header.stamp.sec = 1
        with g.lock:
            g.last_stamp = 10**18                        # 아주 늦은 시각을 마지막으로 처리했다고 둠
        g.on_joint_states(jumped)
        self.assertEqual(g.state[0][1], -0.2)
        self.assertEqual(g.last_stamp, 10**9)
        self.assertTrue(wait_for(lambda: self.harness.kinds('stamp_jump_back')))

    # 관절 상태가 끊기면 신규 조작을 막고 진행 중 일반 동작은 취소(유지)되는지 확인한다.
    def test_stale_state_blocks_and_holds(self):
        h = self.harness
        handle, result = self.send_action(trajectory(-0.5))
        self.assertTrue(wait_for(lambda: h.has_goal(-0.5)), h.targets())

        h.publishing = False
        self.assertTrue(wait_for(result.done, 4), h.events[-6:])
        self.assertIn('state invalid', result.result().result.error_string)
        self.assertEqual(self.guard.mode, 'state_invalid')
        refused, _ = self.send_action(trajectory(-0.2))
        self.assertFalse(refused.accepted)

        h.publishing = True
        self.assertTrue(wait_for(lambda: self.guard.mode == 'ready'),
                        (self.guard.mode, self.guard.mode_reason))

    # 수락 검사와 내부 전송 사이에 상태가 바뀌는 경합을 동기화 장치로 재현한다.
    # on_goal의 검사가 끝난 뒤(checked) gate 단계('register': 목표 등록 전, 'start': 실행 시작 전)를
    # release까지 멈춰 두고, 그 사이 change()로 보호 노드 상태를 바꾼다.
    # 일반 목표는 내부로 한 건도 보내지지 않아야 한다.
    def run_accept_race(self, change, gate='register', goal=None):
        g = self.guard
        checked, release, sends = threading.Event(), threading.Event(), []

        check, send = g._check, g._send_internal
        gated = {'register': g._register_goal, 'start': g._start_goal}[gate]

        # on_goal 검사 결과를 그대로 돌려주고 검사가 끝났음을 알린다.
        def gated_check(trajectory):
            verdict = check(trajectory)
            checked.set()
            return verdict

        # release 전까지 해당 단계를 멈춘다.
        def gated_step(argument):
            assert release.wait(10), 'release not set'
            return gated(argument)

        # 내부 전송을 owner별로 기록한다.
        def counted_send(points, owner, *args, **kwargs):
            sends.append(owner)
            return send(points, owner, *args, **kwargs)

        self.patch(g, '_check', gated_check)
        self.patch(g, f'_{gate}_goal', gated_step)
        self.patch(g, '_send_internal', counted_send)

        future = self.harness.public.send_goal_async(
            FollowJointTrajectory.Goal(trajectory=goal or trajectory(-1.0)))
        self.assertTrue(checked.wait(5))
        self.assertTrue(wait_for(future.done))
        self.assertTrue(future.result().accepted)        # 검사 시점에는 정상 목표였다

        change()
        release.set()
        result = future.result().get_result_async()
        self.assertTrue(wait_for(result.done))
        self.assertEqual(result.result().status, GoalStatus.STATUS_ABORTED)
        self.assertNotIn('action', sends)
        return result.result().result.error_string, sends

    # 수락 대기 중 보호 정지가 시작되면 일반 목표가 보호 정지를 선점하거나 ready로 바꾸지 못한다.
    def test_accept_race_with_protecting(self):
        h, g = self.harness, self.guard
        reason, sends = self.run_accept_race(self.trigger_protection)
        self.assertIn('state changed after acceptance', reason)
        self.assertEqual(sends, ['protection'])
        self.assertEqual(g.mode, 'protecting')
        self.assertEqual(g.internal['owner'], 'protection')
        self.assertEqual(h.outcomes, [])                 # 보호 정지 목표가 그대로 진행 중

    # 수락 대기 중 고장 잠금이 되면 일반 목표를 보내지 않는다.
    def test_accept_race_with_fault(self):
        g = self.guard

        # 물리 한계 도달 상태를 보내고 고장 잠금까지 기다린다.
        def to_fault():
            self.harness.state = ([0.0, -1.5708, 0.0, 0.0, 0.0], [0.0] * 5)
            self.assertTrue(wait_for(lambda: g.mode == 'fault'))

        reason, sends = self.run_accept_race(to_fault)
        self.assertIn('state changed after acceptance', reason)
        self.assertEqual(sends, [])
        self.assertEqual(g.mode, 'fault')

    # 수락 대기 중 관절 상태가 만료되면 일반 목표를 보내지 않는다.
    def test_accept_race_with_stale_state(self):
        g = self.guard

        # 관절 상태 발행을 멈추고 만료 판정까지 기다린다.
        def to_stale():
            self.harness.publishing = False
            self.assertTrue(wait_for(lambda: g.mode == 'state_invalid', 4))

        reason, sends = self.run_accept_race(to_stale)
        self.assertIn('state changed after acceptance', reason)
        self.assertEqual(sends, [])

    # 모드가 그대로여도 실행 직전 현재 상태로 다시 검사해 통과하지 못하면 보내지 않는다.
    # 정지 상태에서 4 s 안에 -1.0 rad로 가는 목표(시작 가속도 0.375)는 통과하지만, 어깨가 반대쪽으로
    # 0.5 rad/s로 움직이는 상태에서는 시작 가속도가 (−1.5 − 2.0)/4 = −0.875 rad/s²로 한계를 넘는다
    # (검사 순서상 먼저 걸리는 구간 속도 초과로 거부될 수 있다).
    def test_recheck_before_execution(self):
        g = self.guard

        # 보호 판단에 걸리지 않는 방향·위치로 어깨 속도만 바꾼다(상한까지 1.57 rad).
        def move_away():
            self.harness.state = ([0.0] * 5, [0.0, 0.5, 0.0, 0.0, 0.0])
            self.assertTrue(wait_for(lambda: g.state[1][1] == 0.5))
            self.assertEqual(g.mode, 'ready')

        reason, sends = self.run_accept_race(move_away, goal=trajectory(-1.0, seconds=4))
        self.assertIn('recheck before execution failed', reason)
        self.assertIn('shoulder_joint', reason)
        self.assertEqual(sends, [])

    # 등록 뒤 실행 시작 전에 보호 정지가 시작되면 진행 중 목표 중단 표시로 막힌다.
    def test_accept_race_after_registration(self):
        reason, sends = self.run_accept_race(self.trigger_protection, gate='start')
        self.assertIn('protective stop', reason)
        self.assertEqual(sends, ['protection'])
        self.assertEqual(self.guard.mode, 'protecting')

    # 다축 보호 정지: 관절마다 자기 감속도로 멈추는 점들이 내부 목표로 나가는지 확인한다
    # (어깨 0 rad·0.5 rad/s, 손목 1.53 rad·0.2 rad/s → 손목 0.2 s에 1.55 rad, 어깨 1.0 s에 0.25 rad).
    def test_multi_axis_protective_stop(self):
        h = self.harness
        h.state = ([0.0, 0.0, 0.0, 1.53, 0.0], [0.0, 0.5, 0.0, 0.2, 0.0])
        self.assertTrue(wait_for(lambda: h.goals), h.events[-5:])

        stop = h.goals[-1]
        wrist, shoulder = ARM_JOINTS.index('wrist_joint'), ARM_JOINTS.index('shoulder_joint')
        times = [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in stop.points]
        self.assertEqual([round(t, 6) for t in times], [0.2, 1.0])
        self.assertAlmostEqual(stop.points[0].positions[wrist], 1.55)
        self.assertAlmostEqual(stop.points[-1].positions[wrist], 1.55)
        self.assertAlmostEqual(stop.points[-1].positions[shoulder], 0.25)
        self.assertLess(max(p.positions[wrist] for p in stop.points), 1.57)
        self.assertFalse(h.kinds('stop_reaches_limit'))

    # 내부 정지 목표의 수락 거부는 정지 실패이며, 대체하려던 이전 목표는 취소돼 유지된다.
    def test_stop_goal_rejected(self):
        h = self.harness
        handle, result = self.send_action(trajectory(-1.5))
        self.assertTrue(wait_for(lambda: h.has_goal(-1.5)), h.targets())
        h.accept_goals = False
        self.trigger_protection()
        self.assert_stop_failed('rejected')
        self.assertTrue(wait_for(lambda: (1, 'canceled') in h.outcomes), h.outcomes)

    # 내부 정지 목표의 중단은 정지 실패다.
    def test_stop_goal_aborted(self):
        self.harness.behavior = 'abort'
        self.trigger_protection()
        self.assert_stop_failed('aborted')

    # 다른 쪽에서 내부 정지 목표를 취소해도 정지 성공으로 보지 않는다.
    def test_stop_goal_canceled(self):
        h = self.harness
        self.trigger_protection()
        self.assertTrue(wait_for(lambda: h.goals))
        self.assertTrue(h.cancel_all.wait_for_service(timeout_sec=5))
        h.cancel_all.call_async(CancelGoal.Request(goal_info=GoalInfo()))
        self.assert_stop_failed('canceled')

    # 내부 목표 요청의 통신 예외는 정지 실패다.
    def test_stop_goal_communication_error(self):
        # 요청 즉시 예외로 끝나는 future를 돌려준다.
        def broken(goal):
            future = Future()
            future.set_exception(RuntimeError('link down'))
            return future

        self.patch(self.guard.internal_client, 'send_goal_async', broken)
        self.trigger_protection()
        self.assert_stop_failed('link down')

    # 내부 제어기가 응답하지 않으면 확인 제한시간(정지 1.0 s + 0.5 s) 뒤 정지 실패다.
    def test_stop_goal_no_response(self):
        self.patch(self.guard.internal_client, 'send_goal_async', lambda goal: Future())
        self.trigger_protection()
        self.assertEqual(self.guard.mode, 'protecting')
        self.assert_stop_failed('no result')

    # 내부 정지 목표가 성공해도 실제 속도가 확인 속도 이하로 내려오지 않으면 정지 실패다.
    def test_stop_succeeded_but_still_moving(self):
        self.harness.behavior = 'succeed'
        self.trigger_protection()                       # 상태는 계속 0.5 rad/s로 움직임
        self.assertTrue(wait_for(lambda: self.guard.internal['result'] is not None))
        self.assertEqual(self.guard.mode, 'protecting')
        self.assert_stop_failed('joint velocities not below')

    # 공개 토픽으로 빈 궤적(유지 요청)을 보내고, 보호 노드가 처리했음을 이벤트로 확인한다.
    def send_hold_and_wait(self, kind):
        h = self.harness
        before = len(h.kinds(kind))
        h.topic.publish(JointTrajectory(joint_names=list(ARM_JOINTS)))
        self.assertTrue(wait_for(lambda: len(h.kinds(kind)) > before), h.events[-4:])

    # 감속 정지를 시작하고 가짜 JTC가 정지 목표를 받은 뒤 관절 상태를 끊어 상태 이상을 만든다.
    def protect_then_stale(self):
        h, g = self.harness, self.guard
        self.trigger_protection()
        self.assertTrue(wait_for(lambda: len(h.goals) == 1), h.targets())
        h.publishing = False
        self.assertTrue(wait_for(lambda: g.mode == 'state_invalid', 3), g.mode)
        self.assertEqual(g.resume_mode, 'protecting')
        self.assertTrue(g._protection_active())

    # 보호 정지 목표가 취소·대체되지 않고 진행 중인지 확인한다.
    def assert_stop_untouched(self):
        h, g = self.harness, self.guard
        self.assertEqual(len(h.goals), 1, h.targets())            # 대체 목표 없음
        self.assertEqual(h.outcomes, [])                          # 취소·중단 없음
        self.assertEqual(g.internal['owner'], 'protection')
        self.assertFalse(g.internal['cancel_pending'])

    # 보호 정지 → 상태 만료 → 빈 궤적 순서에서 빈 궤적·일반 목표가 보호 정지를 취소·대체하지 못하고,
    # 상태가 돌아온 뒤에는 정지 결과와 실제 속도를 확인해야만 protected_hold가 되는지 확인한다.
    def test_stale_during_protection_keeps_stop(self):
        h, g = self.harness, self.guard
        self.protect_then_stale()

        self.send_hold_and_wait('rejected')
        self.assertIn('protective stop in progress', h.kinds('rejected')[-1]['reason'])
        refused, _ = self.send_action(trajectory(-1.0))
        self.assertFalse(refused.accepted)
        self.assert_stop_untouched()

        # 정지한 상태로 관절 상태가 돌아와도 정지 목표 결과가 없으면 protecting에 머문다.
        h.state = ([0.0, -1.55, 0.0, 0.0, 0.0], [0.0] * 5)
        h.publishing = True
        self.assertTrue(wait_for(lambda: g.mode == 'protecting'), g.mode)
        self.assertTrue(wait_for(lambda: g.state[0][1] == -1.55))
        self.assertEqual(g.mode, 'protecting')
        self.assert_stop_untouched()

        h.finish.set()
        self.assertTrue(wait_for(lambda: g.mode == 'protected_hold'), (g.mode, g.mode_reason))
        self.assertEqual(h.outcomes, [(1, 'done')])

    # 상태 이상 중에 정지 목표가 성공해도 protected_hold로 바꾸지 않고, 상태가 돌아와 정지 속도를
    # 확인한 뒤에만 바꾸는지 확인한다(움직이는 상태로 돌아오면 바꾸지 않는다).
    def test_stop_result_during_stale_needs_fresh_still_state(self):
        h, g = self.harness, self.guard
        self.protect_then_stale()
        h.finish.set()
        self.assertTrue(wait_for(lambda: g.internal['result'] is not None))
        self.assertEqual(g.mode, 'state_invalid')

        h.state = ([0.0, -1.40, 0.0, 0.0, 0.0], [0.0, -0.05, 0.0, 0.0, 0.0])
        h.publishing = True
        self.assertTrue(wait_for(lambda: g.state[1][1] == -0.05))
        self.assertEqual(g.mode, 'protecting')                    # 아직 움직이는 중

        h.state = ([0.0, -1.40, 0.0, 0.0, 0.0], [0.0] * 5)
        self.assertTrue(wait_for(lambda: g.mode == 'protected_hold'), (g.mode, g.mode_reason))

    # 상태가 돌아오지 않으면 확인 제한시간(정지 1.0 s + 0.5 s) 뒤 정지 실패(fault)가 되고,
    # 그동안 보호 정지 목표는 취소되지 않는지 확인한다.
    def test_stale_during_protection_times_out_to_fault(self):
        h, g = self.harness, self.guard
        self.protect_then_stale()
        self.send_hold_and_wait('rejected')
        self.assertTrue(wait_for(lambda: g.mode == 'fault', 4), (g.mode, g.mode_reason))
        self.assertIn('not confirmed in time', g.fault_reason)
        self.assertEqual(len(h.goals), 1)
        self.assertEqual(h.outcomes, [])

    # 일반 동작 중 빈 궤적은 기존 계약대로 진행 중 목표를 취소해 현재 위치를 유지하는지 확인한다.
    def test_empty_trajectory_holds_normal_motion(self):
        h = self.harness
        handle, result = self.send_action(trajectory(-0.5))
        self.assertTrue(wait_for(lambda: h.has_goal(-0.5)), h.targets())
        self.send_hold_and_wait('hold')
        self.assertTrue(wait_for(lambda: (1, 'canceled') in h.outcomes), h.outcomes)
        self.assertTrue(wait_for(result.done))
        self.assertIn('hold requested', result.result().result.error_string)

    # 물리 한계 도달로 고장 잠금을 만든 뒤 관절 상태 발행을 멈추고, 콜백이 더 오지 않을 때까지 기다린다.
    def fault_and_quiet(self):
        h, g = self.harness, self.guard
        h.state = ([0.0, -1.5708, 0.0, 0.0, 0.0], [0.0] * 5)
        self.assertTrue(wait_for(lambda: g.mode == 'fault'))
        h.publishing = False
        self.assertTrue(wait_for(lambda: time.monotonic() - g.state[2] > 0.1))

    # 고장 해제를 요청하고 응답을 돌려준다.
    def request_clear(self):
        future = self.harness.clear.call_async(Trigger.Request())
        self.assertTrue(wait_for(future.done))
        return future.result()

    # 비정상·누락·오래된 상태, 한계 근처·움직이는 상태에서는 해제를 거부하고 fault를 유지하며,
    # 최신·유한·정지 상태이고 모든 축의 여유가 충분할 때만 해제하는지 확인한다.
    def test_clear_fault_checks_state(self):
        h, g = self.harness, self.guard
        self.fault_and_quiet()
        now = time.monotonic
        good = [0.0, -1.0, 0.0, 0.0, 0.0]
        cases = [
            ('missing', None, 'missing'),
            ('short', ([0.0] * 4, [0.0] * 4, now()), 'incomplete'),
            ('nan position', ([0.0, math.nan, 0.0, 0.0, 0.0], [0.0] * 5, now()), 'not finite'),
            ('+inf position', ([math.inf] + good[1:], [0.0] * 5, now()), 'not finite'),
            ('-inf velocity', (good, [0.0, -math.inf, 0.0, 0.0, 0.0], now()), 'not finite'),
            ('missing velocity', (good, [0.0, 0.0, math.nan, 0.0, 0.0], now()), 'not finite'),
            ('stale', (good, [0.0] * 5, now() - 10.0), 'stale'),
            ('near limit', ([0.0, -1.568, 0.0, 0.0, 0.0], [0.0] * 5, now()), 'near physical'),
            ('moving', (good, [0.0, 0.05, 0.0, 0.0, 0.0], now()), 'still moving'),
        ]
        for name, state, expected in cases:
            with self.subTest(name):
                with g.lock:
                    g.state = (state if state is None
                               else (state[0], state[1], now() - 10.0 if name == 'stale'
                                     else now()))
                before = len(h.kinds('clear_fault_rejected'))
                response = self.request_clear()
                self.assertFalse(response.success)
                self.assertIn(expected, response.message)
                self.assertEqual(g.mode, 'fault')
                self.assertTrue(wait_for(lambda: len(h.kinds('clear_fault_rejected')) > before))

        with g.lock:
            g.state = (good, [0.0] * 5, now())
        response = self.request_clear()
        self.assertTrue(response.success, response.message)
        self.assertEqual(g.mode, 'protected_hold')

    # fault 상태에서 비정상 joint_states를 실제 구독 콜백으로 받은 뒤 해제를 요청하면 거부되고,
    # 정상 상태가 다시 오면 해제되는지 확인한다.
    def test_clear_fault_after_nonfinite_joint_states(self):
        h, g = self.harness, self.guard
        h.state = ([0.0, -1.5708, 0.0, 0.0, 0.0], [0.0] * 5)
        self.assertTrue(wait_for(lambda: g.mode == 'fault'))

        for state in (([0.0, math.nan, 0.0, 0.0, 0.0], [0.0] * 5),
                      ([0.0, -1.0, 0.0, 0.0, 0.0], [0.0, math.inf, 0.0, 0.0, 0.0])):
            h.state = state
            self.assertTrue(wait_for(lambda: not logic_finite(g.state)))
            response = self.request_clear()
            self.assertFalse(response.success)
            self.assertIn('not finite', response.message)
            self.assertEqual(g.mode, 'fault')

        h.state = ([0.0, -1.0, 0.0, 0.0, 0.0], [0.0] * 5)
        self.assertTrue(wait_for(lambda: logic_finite(g.state) and g.state[0][1] == -1.0))
        response = self.request_clear()
        self.assertTrue(response.success, response.message)
        self.assertEqual(g.mode, 'protected_hold')

    # 물리 한계 도달은 고장 잠금이 되고, 한계에 있는 동안 해제·새 목표가 거부되는지 확인한다.
    def test_fault_latches(self):
        h = self.harness
        h.state = ([0.0, -1.5708000000000202, 0.0, 0.0, 0.0], [0.0] * 5)
        self.assertTrue(wait_for(lambda: self.guard.mode == 'fault'))
        refused, _ = self.send_action(trajectory(-1.0))
        self.assertFalse(refused.accepted)
        self.assertTrue(wait_for(lambda: h.kinds('rejected')))
        self.assertIn('fault latched', h.kinds('rejected')[-1]['reason'])

        future = h.clear.call_async(Trigger.Request())
        self.assertTrue(wait_for(future.done))
        self.assertFalse(future.result().success)
        h.state = ([0.0] * 5, [0.0] * 5)
        time.sleep(0.1)
        self.assertEqual(self.guard.mode, 'fault')        # 자동 해제하지 않는다


if __name__ == '__main__':
    unittest.main()
