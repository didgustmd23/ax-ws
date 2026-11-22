"""Verify protected-arm acceptance is gated by ActuationState Schema 2 metadata."""
import os
from pathlib import Path
import subprocess
import time
import unittest

import rclpy
from ax_interfaces.action import ProtectedArmTrajectory
from ax_interfaces.msg import ActuationState, SafetyState
from builtin_interfaces.msg import Duration
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from trajectory_msgs.msg import JointTrajectoryPoint

import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
import ros_test_domain  # noqa: E402


ROBOT = 'robot_a4_protected_arm_test'
_domain_state = None


def setUpModule():
    global _domain_state
    domain, lock = ros_test_domain.reserve_domain(range(90, 100))
    _domain_state = {'domain': domain, 'lock': lock,
                     'previous': {key: os.environ.get(key) for key in (
                         'ROS_DOMAIN_ID', 'ROS_AUTOMATIC_DISCOVERY_RANGE')}}
    os.environ.update(ROS_DOMAIN_ID=str(domain), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')


def tearDownModule():
    global _domain_state
    ros_test_domain.leave_module_domain(_domain_state)
    _domain_state = None


class ProtectedArmSchemaRosTest(unittest.TestCase):
    def setUp(self):
        binary = os.environ.get('AX_PROTECTED_ARM_BIN')
        self.assertTrue(binary, 'AX_PROTECTED_ARM_BIN must point to the current ax_safety build')
        self.assertTrue(Path(binary).is_file(), f'protected-arm executable not found: {binary}')

        rclpy.init()
        self.node = Node('a4_protected_arm_schema_test_peer')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        base = f'/ax/{ROBOT}'
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.safety_pub = self.node.create_publisher(SafetyState, f'{base}/safety/state', qos)
        self.actuation_pub = self.node.create_publisher(
            ActuationState, f'{base}/safety/actuation_state', qos)
        self.client = ActionClient(
            self.node, ProtectedArmTrajectory, f'{base}/arm/execute_trajectory')
        self.process = subprocess.Popen([
            binary, '--ros-args', '-p', f'robot_id:={ROBOT}',
            '-p', 'enable_simulation_arm:=true',
            '-p', 'arm_joint_names:=[joint_1]',
            '-p', 'arm_position_min_rad:=[-2.0]',
            '-p', 'arm_position_max_rad:=[2.0]',
            '-p', 'arm_max_velocity_rad_s:=[1.0]',
            '-p', 'arm_max_acceleration_rad_s2:=[2.0]',
            '-p', 'controller_action:=/unused/controller',
            '-p', 'stop_hold_s:=0.5'], env=os.environ.copy())
        self.addCleanup(self.cleanup)
        self.assertTrue(self.client.wait_for_server(timeout_sec=3.0))

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

    def spin_until(self, future, timeout=3.0):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.02)
        self.assertTrue(future.done(), 'ROS future did not complete')
        return future.result()

    def publish_safety(self):
        stamp = self.node.get_clock().now().to_msg()
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
        message.meta.header.stamp = stamp
        message.meta.schema_version = 1
        message.meta.source_id = 'a4-protected-test-b4'
        message.meta.source_session_id = 'safety-session'
        message.meta.revision = 1
        message.meta.valid = True
        self.safety_pub.publish(message)

    def publish_actuation(self, *, robot_id=ROBOT, session='actuation-session',
                          revision=1, stamp=None, schema=2, valid=True):
        message = ActuationState()
        message.robot_id = robot_id
        message.state = ActuationState.STOPPED
        message.arm_allowed = True
        message.drive_allowed = True
        message.meta.header.stamp = stamp or self.node.get_clock().now().to_msg()
        message.meta.schema_version = schema
        message.meta.source_id = 'a4-protected-test-gate'
        message.meta.source_session_id = session
        message.meta.revision = revision
        message.meta.valid = valid
        self.actuation_pub.publish(message)

    def send_goal(self):
        goal = ProtectedArmTrajectory.Goal()
        goal.request_id = 'request-1'
        goal.authority = ProtectedArmTrajectory.Goal.AUTH_A3_LOCAL_TASK
        goal.execution_id = 'execution-1'
        goal.trajectory.joint_names = ['joint_1']
        point = JointTrajectoryPoint()
        point.positions = [0.0]
        point.time_from_start = Duration(sec=1)
        goal.trajectory.points = [point]
        return self.spin_until(self.client.send_goal_async(goal))

    def prime_safety(self):
        self.publish_safety()
        self.publish_actuation()
        end = time.monotonic() + 0.5
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=0.02)

    def test_schema1_rejected_and_schema2_accepted(self):
        self.publish_safety()
        self.publish_actuation(schema=1)
        for _ in range(10):
            self.executor.spin_once(timeout_sec=0.02)
        self.assertFalse(self.send_goal().accepted)

        self.publish_actuation(schema=2, revision=2)
        for _ in range(10):
            self.executor.spin_once(timeout_sec=0.02)
        goal_handle = self.send_goal()
        self.assertIsNotNone(goal_handle)
        self.assertTrue(goal_handle.accepted)
        self.spin_until(goal_handle.get_result_async())

    def test_robot_id_session_revision_and_timestamp_rejected(self):
        invalid_messages = (
            {'robot_id': 'robot_other'},
            {'session': ''},
            {'revision': 0},
            {'stamp': self.node.get_clock().now().to_msg()},
        )
        for invalid in invalid_messages:
            self.publish_safety()
            if invalid.get('stamp') is not None:
                stamp = invalid['stamp']
                stamp.sec = 0
                stamp.nanosec = 0
                invalid['stamp'] = stamp
            self.publish_actuation(**invalid)
            for _ in range(10):
                self.executor.spin_once(timeout_sec=0.02)
            self.assertFalse(self.send_goal().accepted, invalid)

    def test_old_revision_and_retired_session_do_not_restore_stale_state(self):
        self.prime_safety()
        self.publish_actuation(session='session-a', revision=1)
        for _ in range(10):
            self.executor.spin_once(timeout_sec=0.02)
        self.publish_actuation(session='session-b', revision=1)
        self.publish_actuation(session='session-a', revision=2)
        time.sleep(0.35)
        for _ in range(20):
            self.executor.spin_once(timeout_sec=0.02)
        self.assertFalse(self.send_goal().accepted)


if __name__ == '__main__':
    unittest.main()
