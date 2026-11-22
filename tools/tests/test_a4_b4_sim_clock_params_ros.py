"""A4 STOPPED의 sim 시계 유지와 A4/B4 매개변수 적용을 실ROS(가짜 입력)로 확인한다.

- 가짜 /clock·odom·joint_states·SafetyState를 시험이 발행한다. 실제 물리 정지 검증이 아니다(시험 fixture).
- 매개변수 파일·주행 자세 배열은 시험용 구별값이다(A1 운영값 아님).
- worktree의 install/ax_safety를 쓴다. domain은 ros_test_domain helper로 90~99에서 예약한다.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

import rclpy
from ax_interfaces.msg import ActuationState, SafetyState
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState

sys.path.insert(0, str(Path(__file__).resolve().parent))
import ros_test_domain  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
# AX_GATE_BIN: 수정 전 빌드와 비교할 때만 다른 실행 파일을 지정한다(기본은 worktree install)
GATE = Path(os.environ.get('AX_GATE_BIN', ROOT / 'install' / 'ax_safety' / 'lib' / 'ax_safety' / 'a4_safety_gate'))
_domain_state = None


def setUpModule():
    global _domain_state
    domain, lock = ros_test_domain.reserve_domain(range(90, 100))
    _domain_state = {'domain': domain, 'lock': lock,
                     'previous': {key: os.environ.get(key) for key in ('ROS_DOMAIN_ID', 'ROS_AUTOMATIC_DISCOVERY_RANGE')}}
    os.environ.update(ROS_DOMAIN_ID=str(domain), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')


def tearDownModule():
    global _domain_state
    ros_test_domain.leave_module_domain(_domain_state)
    _domain_state = None


def to_time(seconds):
    from builtin_interfaces.msg import Time
    sec = int(seconds)
    return Time(sec=sec, nanosec=int(round((seconds - sec) * 1e9)))


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def stop_group(process):
    # 시험이 띄운 프로세스 그룹만 정리한다.
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=5)
    except (subprocess.TimeoutExpired, ProcessLookupError):
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait(timeout=5)


class GatePeer:
    """가짜 /clock과 피드백을 발행하고 ActuationState를 모은다."""

    def __init__(self, robot):
        self.robot = robot
        self.node = Node(f'a4_sim_clock_peer_{robot}')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        base = f'/ax/{robot}'
        state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.clock_pub = self.node.create_publisher(Clock, '/clock', 10)
        self.safety_pub = self.node.create_publisher(SafetyState, f'{base}/safety/state', state_qos)
        self.odom_pub = self.node.create_publisher(Odometry, f'{base}/diff_drive_controller/odom', 10)
        self.joint_pub = self.node.create_publisher(JointState, f'{base}/joint_states', 10)
        self.states = []
        self.node.create_subscription(ActuationState, f'{base}/safety/actuation_state', self.states.append, 10)
        self.sim = 100.0

    def publish(self, linear=0.0, joint_velocity=0.0, safety=True, clock=True, joint_names=('test_joint_a', 'test_joint_b'),
                joint_positions=(0.0, 0.0), joint_velocities=None, feedback_stamp=None, feedback=True):
        stamp = to_time(self.sim) if clock else to_time(0.0)
        fb_stamp = stamp if feedback_stamp is None else to_time(feedback_stamp)
        if clock:
            self.clock_pub.publish(Clock(clock=stamp))
        if safety:
            m = SafetyState()
            m.robot_id = self.robot
            m.level = SafetyState.CLEAR
            m.sensor_valid = m.front_sensor_valid = m.rear_sensor_valid = m.localization_valid = True
            m.allow_forward = m.allow_backward = m.allow_rotate = True
            m.max_linear_speed, m.max_angular_speed = 0.67, 0.5
            m.meta.header.stamp = stamp
            m.meta.schema_version = 1
            m.meta.source_id = 'w2-sim-clock-test-b4'
            m.meta.source_session_id = 'w2-sim-clock-test'
            m.meta.revision = 1
            m.meta.valid = True
            self.safety_pub.publish(m)
        if not feedback:
            return
        self.publish_odom(fb_stamp, linear)
        joints = JointState()
        joints.header.stamp = fb_stamp
        joints.name = list(joint_names)
        joints.position = list(joint_positions)[:len(joint_names)]
        joints.velocity = list(joint_velocities) if joint_velocities is not None else [joint_velocity] * len(joint_names)
        self.joint_pub.publish(joints)

    # odom 표본 하나만 발행(stamp는 builtin Time)
    def publish_odom(self, stamp, linear):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.twist.twist.linear.x = linear
        self.odom_pub.publish(odom)

    # 벽시계 wall_s 동안 rtf 배율로 sim 시각을 진행(advance=False면 정체)하며 발행·수신.
    # extra(peer)가 있으면 매 반복의 기본 발행 뒤에 추가 표본을 보낸다.
    def run(self, wall_s, rtf=1.0, advance=True, extra=None, **kwargs):
        start = len(self.states)
        last = time.monotonic()
        deadline = last + wall_s
        while time.monotonic() < deadline:
            now = time.monotonic()
            if advance:
                self.sim += rtf * (now - last)
            last = now
            self.publish(**kwargs)
            if extra is not None:
                extra(self)
            self.executor.spin_once(timeout_sec=0.01)
        return self.states[start:]

    # 낮은 /clock 발행률: step_s마다 sim을 step_s만큼 올리고 그때만 새 stamp 표본을 낸다(그 사이는 같은 표본 재발행)
    def run_stepped(self, wall_s, step_s):
        start = len(self.states)
        deadline = time.monotonic() + wall_s
        next_step = time.monotonic() + step_s
        while time.monotonic() < deadline:
            if time.monotonic() >= next_step:
                self.sim += step_s
                next_step += step_s
            self.publish()
            self.executor.spin_once(timeout_sec=0.01)
        return self.states[start:]

    def destroy(self):
        self.executor.shutdown()
        self.node.destroy_node()


def stopped(states):
    return [s for s in states if s.state == ActuationState.STOPPED and s.meta.valid]


# 시험용 팔 역할: 팔 있음, 필수 관절은 시험 관절 2개(모델 관절 이름이 아닌 fixture)
ARM_PRESENT = ('stop_arm_presence:=present', 'stop_arm_joint_names:=[test_joint_a,test_joint_b]')


class GateTestBase(unittest.TestCase):
    """게이트 하나와 가짜 입력 peer를 띄우는 공통 준비(시험 없음)."""
    ROBOT = 'robot_w2_simclock'

    def setUp(self):
        rclpy.init()
        self.peer = GatePeer(self.ROBOT)
        self.process = None
        self.addCleanup(self.cleanup)
        self.start_gate(ARM_PRESENT)

    # 진단 매개변수(하위 클래스가 바꿔 진단 활성 상태로 같은 시험을 반복한다)
    DIAG_PARAMS = ()

    # 게이트를 (다시) 띄운다. params: 추가 -p 항목. stdout_path가 있으면 노드 로그를 그 파일에 쓴다
    def start_gate(self, params, stdout_path=None):
        stop_group(self.process)
        args = [str(GATE), '--ros-args', '-r', f'__ns:=/ax/{self.ROBOT}', '-p', f'robot_id:={self.ROBOT}',
                '-p', 'use_sim_time:=true', '-p', 'stop_hold_s:=0.5']
        for item in tuple(params) + tuple(self.DIAG_PARAMS):
            args += ['-p', item]
        out = open(stdout_path, 'w') if stdout_path else None
        env = dict(os.environ, RCUTILS_LOGGING_BUFFERED_STREAM='0')   # 로그 줄을 바로 파일에 쓰게 한다
        self.process = subprocess.Popen(args, env=env, start_new_session=True,
                                        stdout=out, stderr=subprocess.STDOUT if out else None)
        time.sleep(0.3)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and self.peer.safety_pub.get_subscription_count() == 0:
            self.peer.executor.spin_once(timeout_sec=0.05)
        self.assertGreater(self.peer.safety_pub.get_subscription_count(), 0)
        self.peer.states.clear()

    def cleanup(self):
        stop_group(self.process)
        if rclpy.ok():
            self.peer.destroy()
            rclpy.shutdown()



class SimClockGateTest(GateTestBase):
    ROBOT = 'robot_w2_simclock'

    # /clock 전에는 STOPPED가 성립하지 않는다(벽시계 경과만으로 채우지 않음)
    def test_no_clock_never_stopped(self):
        states = self.peer.run(1.5, clock=False)
        self.assertTrue(states)
        self.assertEqual(stopped(states), [])

    # 느린 시계(RTF 0.1): 벽시계 2 s(sim 0.2 s)에는 없고, sim 0.5 s를 넘긴 뒤에 성립
    def test_slow_clock_requires_sim_hold(self):
        start_sim = self.peer.sim
        early = self.peer.run(2.0, rtf=0.1)
        self.assertTrue(early)
        self.assertEqual(stopped(early), [])
        late = self.peer.run(5.0, rtf=0.1)
        hits = stopped(late)
        self.assertTrue(hits)
        self.assertGreaterEqual(stamp_s(hits[0].meta.header.stamp) - start_sim, 0.5 - 0.06)

    # 정체·역행·도약은 새 STOPPED를 만들지 않고, 정상 진행 0.5 s 뒤에만 성립
    def test_stall_reversal_jump(self):
        self.assertEqual(stopped(self.peer.run(0.3, rtf=1.0)), [])
        self.assertEqual(stopped(self.peer.run(1.5, advance=False)), [])      # 정체: 누적 그대로(0.3 미만)
        self.peer.sim -= 1.0                                                   # 역행: 누적 버림
        self.assertEqual(stopped(self.peer.run(0.35, rtf=1.0)), [])
        self.peer.sim += 5.0                                                   # 도약: 누적 버림
        self.assertEqual(stopped(self.peer.run(0.35, rtf=1.0)), [])
        self.assertTrue(stopped(self.peer.run(0.5, rtf=1.0)))

    # 이동·관절 이동·피드백 끊김은 유지 조건을 초기화하고, SafetyState 만료는 허가만 막는다(Schema 2)
    def test_motion_and_validity_reset(self):
        self.assertTrue(stopped(self.peer.run(0.9, rtf=1.0)))
        for kwargs in ({'linear': 0.05}, {'joint_velocity': 0.05}):
            moving = self.peer.run(0.2, rtf=1.0, **kwargs)
            self.assertFalse(moving[-1].state == ActuationState.STOPPED)
            self.assertEqual(stopped(self.peer.run(0.3, rtf=1.0)), [])
            self.assertTrue(stopped(self.peer.run(0.45, rtf=1.0)))

        # Schema 2: SafetyState 만료(0.5 s 초과 무수신)는 drive_allowed·arm_allowed만 끄고
        # SAFETY_STOP을 남긴다. odom·관절 표본이 유효·저속이면 STOPPED와 meta.valid는 유지된다
        # (INTERFACE_CONTRACT.md 141~142행, ax_safety/README.md 28~33행, safety_gate.cpp 372·376~379·413행).
        stale = self.peer.run(0.8, rtf=1.0, safety=False)
        self.assertTrue(stale)
        self.assertFalse(stale[-1].drive_allowed)
        self.assertFalse(stale[-1].arm_allowed)
        self.assertIn('SAFETY_STOP', stale[-1].block_reasons)
        self.assertEqual(stale[-1].state, ActuationState.STOPPED)
        self.assertTrue(stale[-1].meta.valid)
        # Schema 2: safety 만료는 정지 구간을 reset하지 않는다(reset은 clock 무효·state stale만,
        # safety_gate.cpp 363~366행). 만료 구간 내내 STOPPED가 끊기지 않아야 한다.
        self.assertEqual(len(stopped(stale)), len(stale))

        # Schema 2: 회복 뒤에는 새 0.5 s 유지 없이 STOPPED가 이어지고 SAFETY_STOP 사유가 사라진다.
        # 이 게이트는 주행 자세 미검증으로 잠긴 채 시작하므로 drive_allowed 자체는 TRAVEL_POSE_UNVERIFIED로
        # false가 유지된다. 허가 복귀는 SAFETY_STOP 해제로 확인한다(safety_gate.cpp 376~379·413·415행).
        recovered = self.peer.run(0.3, rtf=1.0)
        self.assertTrue(recovered)
        self.assertEqual(len(stopped(recovered)), len(recovered))
        self.assertNotIn('SAFETY_STOP', recovered[-1].block_reasons)
        self.assertIn('TRAVEL_POSE_UNVERIFIED', recovered[-1].block_reasons)

        # 피드백 끊김(odom·관절 무수신 0.3 s 초과)은 state stale이라 구간을 reset한다
        # (safety_gate.cpp 230~240·363~366행). Schema 2에서도 회복 뒤 이전 유지 시간을 재사용하지 않는다.
        lost = self.peer.run(0.6, rtf=1.0, feedback=False)
        self.assertTrue(lost)
        self.assertFalse(lost[-1].meta.valid)
        self.assertEqual(lost[-1].meta.validity_reason, 'STATE_STALE')
        self.assertIn('STATE_STALE', lost[-1].block_reasons)
        self.assertEqual(stopped(self.peer.run(0.3, rtf=1.0)), [])
        self.assertTrue(stopped(self.peer.run(0.45, rtf=1.0)))

    # 정지 확인은 주행 잠금(drive_allowed=false)과 별개다
    def test_stopped_while_drive_locked(self):
        hits = stopped(self.peer.run(0.9, rtf=1.0))
        self.assertTrue(hits)
        self.assertFalse(hits[-1].drive_allowed)
        self.assertIn('TRAVEL_POSE_UNVERIFIED', hits[-1].block_reasons)

    # tick 사이에 들어온 이동 표본 하나(직후 저속 표본으로 덮임)도 유지 구간을 끊는다
    def test_high_sample_between_ticks_resets(self):
        self.assertTrue(stopped(self.peer.run(0.9, rtf=1.0)))

        def one_high(peer):
            peer.publish_odom(to_time(peer.sim + 0.001), 0.05)
            peer.publish_odom(to_time(peer.sim + 0.002), 0.0)
        self.peer.run(0.01, rtf=1.0, extra=one_high)
        self.peer.sim += 0.003
        self.assertEqual(stopped(self.peer.run(0.3, rtf=1.0)), [])
        self.assertTrue(stopped(self.peer.run(0.5, rtf=1.0)))

    # 늦게 도착한(더 오래된 stamp) 저속 표본이 최신 이동 표본을 덮지 못한다
    def test_late_low_does_not_override_motion(self):
        def late_low(peer):
            peer.publish_odom(to_time(peer.sim - 0.05), 0.0)
        states = self.peer.run(1.5, rtf=1.0, linear=0.05, extra=late_low)
        self.assertTrue(states)
        self.assertEqual(stopped(states), [])

    # 같은 stamp에 상충하는 값(이동 뒤 저속)이 오면 정지 근거로 쓰지 않는다
    def test_same_stamp_conflict_not_stopped(self):
        def conflict(peer):
            peer.publish_odom(to_time(peer.sim), 0.05)
            peer.publish_odom(to_time(peer.sim), 0.0)
        states = self.peer.run(1.5, rtf=1.0, feedback=True, extra=conflict)
        self.assertEqual(stopped(states), [])

    # /clock만 진행하고 피드백 stamp가 정체하면(같은 표본 재발행) 새 근거가 없어 STOPPED를 유지하지 않는다
    def test_frozen_feedback_stamp_with_clock_running(self):
        self.assertTrue(stopped(self.peer.run(0.9, rtf=1.0)))
        frozen = self.peer.sim
        states = self.peer.run(1.0, rtf=1.0, feedback_stamp=frozen)
        self.assertFalse(stopped(states[len(states) // 2:]))
        self.assertEqual(stopped(self.peer.run(0.3, rtf=1.0)), [])       # 재개 뒤 이전 유지 시간 재사용 안 함
        self.assertTrue(stopped(self.peer.run(0.5, rtf=1.0)))

    # 이미 STOPPED였어도 /clock 정체(같은 stamp 재수신) 중에는 근거가 만료되고, 3 s 뒤 시간 무효로 표시한다
    def test_clock_stall_after_stopped_invalidates(self):
        self.assertTrue(stopped(self.peer.run(0.9, rtf=1.0)))
        early = self.peer.run(0.6, advance=False)
        self.assertEqual(stopped(early[len(early) // 2:]), [])
        late = self.peer.run(3.0, advance=False)
        self.assertEqual(stopped(late), [])
        self.assertFalse(late[-1].meta.valid)
        self.assertEqual(late[-1].meta.validity_reason, 'CLOCK_INVALID')
        self.assertEqual(late[-1].state, ActuationState.UNKNOWN)
        self.assertIn('TIME_INVALID', late[-1].block_reasons)
        self.assertEqual(stopped(self.peer.run(0.3, rtf=1.0)), [])
        self.assertTrue(stopped(self.peer.run(0.5, rtf=1.0)))

    # 낮은 /clock 발행률(0.26 s 간격, RTF 1): 표본 간격이 기존 0.3 s 한계 안이면 정상 근거로 STOPPED가 성립한다
    def test_low_clock_rate_still_confirms(self):
        self.assertTrue(stopped(self.peer.run_stepped(2.5, 0.26)))

    # 일부 관절 이동·관절 누락·NaN은 즉시 근거를 끊는다
    def test_partial_joint_motion_missing_and_nan(self):
        cases = ({'joint_velocities': (0.0, 0.05)}, {'joint_names': ('test_joint_a',)}, {'linear': float('nan')})
        for kwargs in cases:
            self.assertTrue(stopped(self.peer.run(0.9, rtf=1.0)), kwargs)
            bad = self.peer.run(0.15, rtf=1.0, **kwargs)
            self.assertFalse(bad[-1].state == ActuationState.STOPPED, kwargs)
            self.assertEqual(stopped(self.peer.run(0.3, rtf=1.0)), [], kwargs)


class StopEvidenceBoundaryTest(GateTestBase):
    """미래 표본·필수 관절·역할·중복 비교 경계(시험 fixture, 실제 물리 정지 검증 아님)."""
    ROBOT = 'robot_w2_evidence'

    # 현재 시각보다 미래인 표본만 오면 정지 근거가 되지 않는다
    def test_future_samples_only_never_stopped(self):
        states = self.peer.run(1.2, rtf=1.0, feedback=False,
                               extra=lambda peer: peer.publish(clock=False, safety=False, feedback_stamp=peer.sim + 0.049))
        self.assertTrue(states)
        self.assertEqual(stopped(states), [])

    # 현재 표본과 함께 미래 저속 표본이 반복돼도 STOPPED는 현재 시각 기준 0.5 s를 채운 뒤에만 나온다
    def test_future_low_does_not_advance_hold(self):
        start = self.peer.sim
        states = self.peer.run(1.0, rtf=1.0, extra=lambda peer: peer.publish(
            clock=False, safety=False, feedback_stamp=peer.sim + 0.049))
        hits = stopped(states)
        self.assertTrue(hits)
        self.assertGreaterEqual(stamp_s(hits[0].meta.header.stamp) - start, 0.5 - 1e-3)

    # 역할 미정(기본값)은 정지를 확인하지 않는다
    def test_role_unconfirmed_never_stopped(self):
        self.start_gate(())
        self.assertEqual(stopped(self.peer.run(1.2, rtf=1.0)), [])

    # 팔 있음인데 필수 목록이 비었거나 중복이면 설정 무효로 정지를 확인하지 않는다
    def test_invalid_required_list_never_stopped(self):
        for params in (('stop_arm_presence:=present',),
                       ('stop_arm_presence:=present', 'stop_arm_joint_names:=[test_joint_a,test_joint_a]'),
                       ('stop_arm_presence:=maybe', 'stop_arm_joint_names:=[test_joint_a,test_joint_b]')):
            self.start_gate(params)
            self.assertEqual(stopped(self.peer.run(1.0, rtf=1.0)), [], params)

    # 팔 없음 명시: 바퀴 관절이 돌아도 팔 기준에 넣지 않고 odom 근거로 정지 확인, 기본 허가는 닫힘
    def test_arm_absent_uses_odom_and_keeps_lock(self):
        self.start_gate(('stop_arm_presence:=absent',))
        hits = stopped(self.peer.run(1.0, rtf=1.0, joint_names=('fixture_wheel_joint',), joint_positions=(0.0,),
                                     joint_velocities=(0.5,)))
        self.assertTrue(hits)
        self.assertFalse(hits[-1].drive_allowed)
        self.assertFalse(hits[-1].arm_allowed)
        self.assertIn('TRAVEL_POSE_UNVERIFIED', hits[-1].block_reasons)
        moving = self.peer.run(0.2, rtf=1.0, linear=0.05, joint_names=('fixture_wheel_joint',),
                               joint_positions=(0.0,), joint_velocities=(0.0,))
        self.assertNotEqual(moving[-1].state, ActuationState.STOPPED)

    # 첫 표본부터 필수 관절이 빠지면 정지를 확인하지 않는다
    def test_required_joint_missing_from_first_sample(self):
        self.assertEqual(stopped(self.peer.run(1.2, rtf=1.0, joint_names=('test_joint_a',), joint_positions=(0.0,))), [])

    # 필수가 아닌 관절(바퀴 등)의 속도는 팔 기준에 넣지 않고, 필수 관절 하나라도 움직이면 확인하지 않는다
    def test_only_required_joints_judged(self):
        names = ('test_joint_a', 'test_joint_b', 'fixture_wheel_joint')
        self.assertTrue(stopped(self.peer.run(1.0, rtf=1.0, joint_names=names, joint_positions=(0.0, 0.0, 0.0),
                                              joint_velocities=(0.0, 0.0, 0.5))))
        self.assertEqual(stopped(self.peer.run(0.6, rtf=1.0, joint_names=names, joint_positions=(0.0, 0.0, 0.0),
                                               joint_velocities=(0.0, 0.05, 0.0))[3:]), [])

    # 메시지 안 관절 이름 중복은 근거 상실
    def test_duplicate_names_in_message_rejected(self):
        self.assertEqual(stopped(self.peer.run(1.2, rtf=1.0, joint_names=('test_joint_a', 'test_joint_b', 'test_joint_b'),
                                               joint_positions=(0.0, 0.0, 0.0))), [])

    # 같은 stamp에 배열 순서만 바뀐 같은 대응은 중복(근거 유지), 이름만 바꿔 대응이 달라지면 상충
    def test_same_stamp_order_swap_vs_name_swap(self):
        def reordered(peer):
            j = JointState()
            j.header.stamp = to_time(peer.sim)
            j.name, j.position, j.velocity = ['test_joint_b', 'test_joint_a'], [0.2, 0.1], [0.0, 0.0]
            peer.joint_pub.publish(j)
        self.assertTrue(stopped(self.peer.run(1.0, rtf=1.0, joint_positions=(0.1, 0.2), extra=reordered)))

        def renamed(peer):
            j = JointState()
            j.header.stamp = to_time(peer.sim)
            j.name, j.position, j.velocity = ['test_joint_b', 'test_joint_a'], [0.1, 0.2], [0.0, 0.0]
            peer.joint_pub.publish(j)
        late = self.peer.run(1.0, rtf=1.0, joint_positions=(0.1, 0.2), extra=renamed)
        self.assertEqual(stopped(late[3:]), [])


class SimClockGateDiagOnTest(SimClockGateTest):
    """진단 활성(stop_diagnostics=true)으로 같은 판정 시험을 반복해 판정 결과가 같음을 확인한다."""
    ROBOT = 'robot_w2_diagon'
    DIAG_PARAMS = ('stop_diagnostics:=true',)


# 노드 로그에서 A4_STOP_DIAG JSON 줄을 읽는다
def diag_lines(path):
    rows = []
    for line in Path(path).read_text().splitlines():
        if 'A4_STOP_DIAG ' in line:
            rows.append(json.loads(line.split('A4_STOP_DIAG ', 1)[1]))
    return rows


class StopDiagnosticsTest(GateTestBase):
    """진단 출력 내용: 카운터와 STOPPED 진입·이탈 근거 스냅샷(시험 fixture)."""
    ROBOT = 'robot_w2_diag'
    DIAG_PARAMS = ('stop_diagnostics:=true',)

    # 알려진 표본열: 현재 표본 + 반복마다 미래·중복·늦은 odom 표본 하나씩. 진입 스냅샷의 근거가 판정 규칙과 맞는다
    def test_counters_and_enter_snapshot(self):
        log = Path(tempfile.mkdtemp()) / 'gate.log'
        self.start_gate(ARM_PRESENT, stdout_path=log)
        sent = {'future': 0, 'late': 0, 'dup': 0}

        def extra(peer):
            peer.publish_odom(to_time(peer.sim + 0.02), 0.0)
            peer.publish_odom(to_time(peer.sim), 0.0)
            peer.publish_odom(to_time(peer.sim - 0.1), 0.0)
            sent['future'] += 1
            sent['dup'] += 1
            sent['late'] += 1
        self.assertTrue(stopped(self.peer.run(1.2, rtf=1.0, extra=extra)))
        self.assertNotEqual(self.peer.run(0.2, rtf=1.0, linear=0.05)[-1].state, ActuationState.STOPPED)
        self.peer.run(1.2, rtf=1.0)
        stop_group(self.process)
        rows = diag_lines(log)

        enter = [r for r in rows if r['kind'] == 'enter_stopped']
        self.assertTrue(enter)
        e = enter[0]
        self.assertTrue(e['clock_ok'] and e['state_fresh'] and e['safety_fresh'] and e['low'] and e['valid'])
        self.assertEqual((e['presence'], e['joint_used_for_hold'], e['required']), ('present', True, 2))
        self.assertLessEqual(e['odom']['end'], e['now'])
        self.assertLessEqual(e['joint']['end'], e['now'])
        span = min(e['odom']['end'], e['joint']['end']) - max(e['odom']['start'], e['joint']['start'])
        self.assertAlmostEqual(span, e['held'], places=6)
        self.assertGreaterEqual(e['held'], 0.5)
        self.assertTrue([r for r in rows if r['kind'] == 'leave_stopped'])

        last = rows[-1]['odom']
        # 정확한 수는 C++ 순수 시험에서 본다. 실ROS에서는 /clock과 같은 stamp 표본이 /clock보다 먼저 도착해
        # 미래로 거부되는 경우가 더해지므로(구독 간 순서 차이) 보낸 미래 표본 수의 상한으로 검사하지 않는다.
        self.assertGreater(last['rej_future'], 0)
        self.assertGreater(last['duplicate'], 0)
        self.assertGreater(last['late'], 0)
        self.assertEqual(last['rejected'], last['rej_clock'] + last['rej_stamp'] + last['rej_future'])

    # 팔 없음: 진입 스냅샷에 joint를 유지 근거로 쓰지 않음이 표시되고, 유지 길이는 odom 구간이다
    def test_absent_snapshot_marks_joint_unused(self):
        log = Path(tempfile.mkdtemp()) / 'gate.log'
        self.start_gate(('stop_arm_presence:=absent',), stdout_path=log)
        self.assertTrue(stopped(self.peer.run(1.2, rtf=1.0)))
        stop_group(self.process)
        e = [r for r in diag_lines(log) if r['kind'] == 'enter_stopped'][0]
        self.assertEqual((e['presence'], e['joint_used_for_hold']), ('absent', False))
        self.assertAlmostEqual(min(e['odom']['end'], e['now']) - e['odom']['start'], e['held'], places=6)

    # 진단 비활성(기본)이면 진단 줄이 없다
    def test_default_has_no_diagnostics(self):
        self.DIAG_PARAMS = ()
        log = Path(tempfile.mkdtemp()) / 'gate.log'
        self.start_gate(ARM_PRESENT, stdout_path=log)
        self.assertTrue(stopped(self.peer.run(1.0, rtf=1.0)))
        stop_group(self.process)
        self.assertEqual(diag_lines(log), [])


class TravelPoseArrayTest(unittest.TestCase):
    """로봇별 파일의 주행 자세 배열 검사(시험용 관절 이름·값)."""
    ROBOT = 'robot_w2_travel'

    def start(self, params_text, verified=True):
        handle = tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False)
        handle.write(params_text)
        handle.close()
        self.addCleanup(os.unlink, handle.name)
        self.process = subprocess.Popen([str(GATE), '--ros-args', '-r', f'__ns:=/ax/{self.ROBOT}',
                                         '--params-file', handle.name, '-p', f'robot_id:={self.ROBOT}',
                                         '-p', 'use_sim_time:=true',
                                         '-p', f'travel_envelope_verified:={"true" if verified else "false"}'],
                                        env=os.environ.copy(), start_new_session=True,
                                        stderr=subprocess.PIPE, text=True)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and self.peer.safety_pub.get_subscription_count() == 0:
            self.peer.executor.spin_once(timeout_sec=0.05)
            if self.process.poll() is not None:
                break

    def setUp(self):
        rclpy.init()
        self.peer = GatePeer(self.ROBOT)
        self.process = None
        self.addCleanup(self.cleanup)

    def cleanup(self):
        stop_group(self.process)
        if rclpy.ok():
            self.peer.destroy()
            rclpy.shutdown()

    def reasons(self):
        states = self.peer.run(0.6, rtf=1.0)
        self.assertTrue(states)
        return states[-1].block_reasons

    def test_valid_fixture_clears_travel_reason(self):
        self.start('/**/a4_safety_gate:\n  ros__parameters:\n'
                   '    travel_joint_names: [test_joint_a, test_joint_b]\n    travel_joint_positions: [0.0, 0.0]\n')
        self.assertIsNone(self.process.poll())
        self.assertNotIn('TRAVEL_POSE_UNVERIFIED', self.reasons())

    def test_unverified_flag_keeps_lock(self):
        self.start('/**/a4_safety_gate:\n  ros__parameters:\n'
                   '    travel_joint_names: [test_joint_a, test_joint_b]\n    travel_joint_positions: [0.0, 0.0]\n',
                   verified=False)
        self.assertIn('TRAVEL_POSE_UNVERIFIED', self.reasons())

    def test_length_mismatch_rejected(self):
        self.start('/**/a4_safety_gate:\n  ros__parameters:\n'
                   '    travel_joint_names: [test_joint_a, test_joint_b]\n    travel_joint_positions: [0.0]\n')
        self.assertIsNone(self.process.poll())
        self.assertIn('TRAVEL_POSE_UNVERIFIED', self.reasons())

    def test_nonfinite_rejected(self):
        self.start('/**/a4_safety_gate:\n  ros__parameters:\n'
                   '    travel_joint_names: [test_joint_a, test_joint_b]\n    travel_joint_positions: [.nan, 0.0]\n')
        self.assertIsNone(self.process.poll())
        self.assertIn('TRAVEL_POSE_UNVERIFIED', self.reasons())

    def test_pose_mismatch_rejected(self):
        self.start('/**/a4_safety_gate:\n  ros__parameters:\n'
                   '    travel_joint_names: [test_joint_a, test_joint_b]\n    travel_joint_positions: [1.0, 0.0]\n')
        self.assertIn('TRAVEL_POSE_UNVERIFIED', self.reasons())


class LaunchParameterTest(unittest.TestCase):
    """safety.launch.py로 두 로봇을 띄워 실제 조회값·namespace 분리·플래그 우선을 확인한다(시험용 구별값)."""

    COMMON = ('/**/a4_safety_gate:\n  ros__parameters:\n    stop_hold_s: 0.611\n    travel_envelope_verified: true\n'
              '/**/a4_protected_arm_server:\n  ros__parameters:\n    stop_hold_s: 0.612\n'
              '/**/b4_risk_node:\n  ros__parameters:\n    sensor_stale_s: 0.333\n')
    ROBOT_A = ('/**/a4_safety_gate:\n  ros__parameters:\n'
               '    travel_joint_names: [test_joint_a, test_joint_b]\n    travel_joint_positions: [0.1, 0.2]\n'
               '/**/a4_protected_arm_server:\n  ros__parameters:\n'
               '    arm_joint_names: [test_joint_a]\n    arm_position_min_rad: [-1.0]\n    arm_position_max_rad: [1.0]\n'
               '    arm_max_velocity_rad_s: [0.5]\n    arm_max_acceleration_rad_s2: [0.5]\n')

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        common = Path(self.tmp.name) / 'common.yaml'
        robot_a = Path(self.tmp.name) / 'robot_a.yaml'
        common.write_text(self.COMMON)
        robot_a.write_text(self.ROBOT_A)
        self.processes = []
        # 정지 역할: pa = 패키지 AX_1 파일(팔 있음), pb = AX_2 파일(팔 없음), pc = 없음(기본 미정)
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('ax_safety')) / 'config'
        launches = (('robot_w2_pa', [f'robot_params_file:={robot_a}', f'stop_evidence_file:={share / "stop_evidence_ax1.yaml"}']),
                    ('robot_w2_pb', [f'stop_evidence_file:={share / "stop_evidence_ax2.yaml"}']),
                    ('robot_w2_pc', []))
        for robot, extra in launches:
            self.processes.append(subprocess.Popen(
                ['ros2', 'launch', 'ax_safety', 'safety.launch.py', f'robot_id:={robot}', f'params_file:={common}'] + extra,
                env=os.environ.copy(), start_new_session=True, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True))
        self.addCleanup(self.cleanup)
        rclpy.init()
        self.node = Node('a4_param_query_peer')

    def cleanup(self):
        for process in self.processes:
            stop_group(process)
        if rclpy.ok():
            self.node.destroy_node()
            rclpy.shutdown()

    def get(self, fqn, names):
        client = self.node.create_client(GetParameters, f'{fqn}/get_parameters')
        deadline = time.monotonic() + 20.0
        while not client.wait_for_service(timeout_sec=0.5):
            self.assertLess(time.monotonic(), deadline, f'{fqn} 매개변수 서비스 없음')
        future = client.call_async(GetParameters.Request(names=names))
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        self.assertTrue(future.done())
        values = []
        for value in future.result().values:
            kind = value.type
            values.append({1: value.bool_value, 3: value.double_value, 4: value.string_value,
                           9: list(value.string_array_value),
                           8: list(value.double_array_value), 2: value.integer_value}.get(kind))
        self.node.destroy_client(client)
        return dict(zip(names, values))

    def test_values_applied_and_robots_separated(self):
        a_gate = self.get('/ax/robot_w2_pa/a4_safety_gate',
                          ['stop_hold_s', 'use_sim_time', 'travel_joint_names', 'travel_joint_positions', 'travel_envelope_verified'])
        b_gate = self.get('/ax/robot_w2_pb/a4_safety_gate', ['stop_hold_s', 'travel_joint_names', 'travel_envelope_verified'])
        a_arm = self.get('/ax/robot_w2_pa/a4_protected_arm_server', ['stop_hold_s', 'arm_joint_names'])
        b_arm = self.get('/ax/robot_w2_pb/a4_protected_arm_server', ['stop_hold_s', 'arm_joint_names'])
        a_b4 = self.get('/ax/robot_w2_pa/b4_risk_node', ['sensor_stale_s', 'use_sim_time'])
        self.assertAlmostEqual(a_gate['stop_hold_s'], 0.611)
        self.assertIs(a_gate['use_sim_time'], True)
        self.assertEqual(a_gate['travel_joint_names'], ['test_joint_a', 'test_joint_b'])
        self.assertEqual(a_gate['travel_joint_positions'], [0.1, 0.2])
        self.assertIs(a_gate['travel_envelope_verified'], False)        # 파일 true보다 launch 인자 false가 우선
        self.assertAlmostEqual(b_gate['stop_hold_s'], 0.611)
        self.assertEqual(b_gate['travel_joint_names'], [])               # robot_a 값이 robot_b로 새지 않음
        self.assertAlmostEqual(a_arm['stop_hold_s'], 0.612)
        self.assertEqual(a_arm['arm_joint_names'], ['test_joint_a'])
        self.assertEqual(b_arm['arm_joint_names'], [])
        self.assertAlmostEqual(a_b4['sensor_stale_s'], 0.333)
        self.assertIs(a_b4['use_sim_time'], True)
        # 정지 역할 파일은 지정한 로봇에만 적용되고, 지정이 없으면 기본 미정이다
        evidence = ['stop_arm_presence', 'stop_arm_joint_names']
        a_stop = self.get('/ax/robot_w2_pa/a4_safety_gate', evidence)
        b_stop = self.get('/ax/robot_w2_pb/a4_safety_gate', evidence)
        c_stop = self.get('/ax/robot_w2_pc/a4_safety_gate', evidence + ['travel_envelope_verified', 'enable_simulation_drive'])
        self.assertEqual(a_stop, {'stop_arm_presence': 'present', 'stop_arm_joint_names': [
            'column_yaw_joint', 'shoulder_joint', 'elbow_joint', 'wrist_joint', 'gripper_rotation_joint']})
        self.assertEqual(b_stop, {'stop_arm_presence': 'absent', 'stop_arm_joint_names': []})
        self.assertEqual(c_stop['stop_arm_presence'], 'unconfirmed')
        self.assertEqual(c_stop['stop_arm_joint_names'], [])
        self.assertIs(c_stop['travel_envelope_verified'], False)
        self.assertIs(c_stop['enable_simulation_drive'], False)
        for process in self.processes:
            self.assertIsNone(process.poll())                            # 타입 오류로 종료되지 않음


if __name__ == '__main__':
    unittest.main()
