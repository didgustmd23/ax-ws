"""Verify A4 publishes STOPPED only after fresh D-06 feedback confirmation."""
import os
from pathlib import Path
import subprocess
import time
import unittest

import rclpy
from ax_interfaces.msg import ActuationState, SafetyState
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
import ros_test_domain  # noqa: E402


ROOT = Path(__file__).resolve().parents[2]
ROBOT = 'robot_w2_a4'
_domain_state = None


def setUpModule():
    global _domain_state
    # 이 시험은 별도의 낮은 번호 ROS domain을 예약한다.
    # 공용 팀 격리 구간(141~231)은 호출자 소유 영역이므로 helper가 거부한다.
    domain, lock = ros_test_domain.reserve_domain(range(90, 100))
    _domain_state = {'domain': domain, 'lock': lock,
                     'previous': {key: os.environ.get(key) for key in (
                         'ROS_DOMAIN_ID', 'ROS_AUTOMATIC_DISCOVERY_RANGE')}}
    os.environ.update(ROS_DOMAIN_ID=str(domain), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')


def tearDownModule():
    global _domain_state
    ros_test_domain.leave_module_domain(_domain_state)
    _domain_state = None


class A4StopConfirmationRosTest(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = Node('a4_w2_stop_test_peer')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        base = f'/ax/{ROBOT}'
        state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.safety_pub = self.node.create_publisher(SafetyState, f'{base}/safety/state', state_qos)
        self.odom_pub = self.node.create_publisher(Odometry, f'{base}/diff_drive_controller/odom', 10)
        self.joint_pub = self.node.create_publisher(JointState, f'{base}/joint_states', 10)
        self.cmd_pub = self.node.create_publisher(TwistStamped, f'{base}/nav/cmd_vel', 10)
        self.states = []
        self.node.create_subscription(
            ActuationState, f'{base}/safety/actuation_state', self.states.append, 10)
        executable = ROOT / 'install' / 'ax_safety' / 'lib' / 'ax_safety' / 'a4_safety_gate'
        self.process = subprocess.Popen([
            str(executable), '--ros-args', '-p', f'robot_id:={ROBOT}',
            '-p', 'use_sim_time:=false', '-p', 'stop_hold_s:=0.5'],
            env=os.environ.copy())
        self.addCleanup(self.cleanup)
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and self.safety_pub.get_subscription_count() == 0:
            self.executor.spin_once(timeout_sec=0.05)
        self.assertGreater(self.safety_pub.get_subscription_count(), 0)

    def cleanup(self):
        if getattr(self, 'process', None) is not None:
            self.process.terminate()
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()
        if rclpy.ok():
            self.executor.shutdown()
            self.node.destroy_node()
            rclpy.shutdown()

    def publish_feedback(self, odom_stamp=None, safety=True, joint_feedback=True,
                         safety_schema=1, safety_source_id='w2-a4-test-b4'):
        # A4가 명령이 아니라 실제 odom·관절 피드백을 기준으로 판단하도록 입력한다.
        stamp = odom_stamp or self.node.get_clock().now().to_msg()
        if safety:
            message = SafetyState()
            message.robot_id = ROBOT
            message.level = SafetyState.CLEAR
            message.sensor_valid = True
            message.front_sensor_valid = True
            message.rear_sensor_valid = True
            message.localization_valid = True
            message.allow_forward = True
            message.allow_backward = True
            message.allow_rotate = True
            message.max_linear_speed = 0.67
            message.max_angular_speed = 0.5
            message.meta.header.stamp = stamp
            message.meta.schema_version = safety_schema
            message.meta.source_id = safety_source_id
            message.meta.source_session_id = 'w2-a4-test-session'
            message.meta.revision = 1
            message.meta.valid = True
            self.safety_pub.publish(message)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.angular.z = 0.0
        self.odom_pub.publish(odom)

        joints = JointState()
        joints.header.stamp = stamp
        if joint_feedback:
            joints.name = ['left_wheel_joint']
            joints.position = [0.0]
            joints.velocity = [0.0]
        self.joint_pub.publish(joints)

    def spin_for(self, seconds, publish=True, **kwargs):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if publish:
                self.publish_feedback(**kwargs)
            self.executor.spin_once(timeout_sec=0.01)

    def test_requires_hold_and_rejects_invalid_timestamp(self):
        # 유지 시간 전에는 STOPPED가 나오지 않고, 유효한 정지 피드백이 계속된 뒤에만 나온다.
        self.spin_for(0.25)
        self.assertTrue(self.states)
        self.assertTrue(all(state.state != ActuationState.STOPPED for state in self.states))

        self.spin_for(0.65)
        self.assertTrue(any(state.state == ActuationState.STOPPED and state.meta.valid
                            for state in self.states))

        future = self.node.get_clock().now() + rclpy.duration.Duration(seconds=2.0)
        before = len(self.states)
        self.spin_for(0.25, odom_stamp=future.to_msg())
        self.assertTrue(self.states[before:])
        self.assertFalse(self.states[-1].meta.valid and
                         self.states[-1].state == ActuationState.STOPPED)

        self.spin_for(0.65)
        self.assertTrue(any(state.state == ActuationState.STOPPED and state.meta.valid
                            for state in self.states[before:]))

        before = len(self.states)
        self.spin_for(0.35, joint_feedback=False)
        self.assertTrue(self.states[before:])
        self.assertFalse(self.states[-1].meta.valid and
                         self.states[-1].state == ActuationState.STOPPED)

        before = len(self.states)
        self.spin_for(0.65, safety=False)
        self.assertTrue(self.states[before:])
        # 처음 몇 표본은 안전 상태의 0.5초 TTL 안일 수 있으므로,
        # TTL이 확실히 지난 마지막 표본을 확인한다.
        self.assertFalse(self.states[-1].meta.valid and
                         self.states[-1].state == ActuationState.STOPPED)

    def test_rejects_invalid_safety_metadata(self):
        for kwargs in ({'safety_schema': 2}, {'safety_source_id': ''}):
            before = len(self.states)
            self.spin_for(0.25, **kwargs)
            self.assertTrue(self.states[before:])
            self.assertFalse(self.states[-1].meta.valid)


if __name__ == '__main__':
    unittest.main()
