"""Validate Kim's launch against team paths without starting other owners' nodes."""
# [검사 범위] A3·B3 두 로봇 실행 경로를 확인합니다. 구동하지 않고 초기 무효·거절 상태를 검사합니다.
import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]


class LaunchConfigurationTests(unittest.TestCase):
    """Reject robot/model mistakes before a partially configured pair starts."""

    def test_invalid_pairing_and_revision_rejected(self):
        # [핵심] 로봇 ID 중복·모델 개수 불일치·잘못된 지도 버전은 노드 시작 전에 걸러야 합니다.
        from launch import LaunchContext

        spec = importlib.util.spec_from_file_location(
            'kim_handoff_launch', ROOT / 'src/ax_task_executor/launch/team_handoff.launch.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        cases = [dict(robot_ids='robot_1,robot_1'), dict(robot_models='AX_1'),
                 dict(robot_models='AX_1,UNKNOWN'), dict(robot_ids='robot_1,robot_9'),
                 dict(map_revision='0'), dict(map_revision='not-an-integer')]
        for case in cases:
            with self.subTest(case=case):
                context = LaunchContext()
                context.launch_configurations.update(
                    robot_ids='robot_1,robot_2', robot_models='AX_1,AX_2',
                    regions_file='', map_revision='1')
                context.launch_configurations.update(case)
                with self.assertRaises(ValueError):
                    module.setup_nodes(context)


class TeamLaunchSmokeTests(unittest.TestCase):
    """Exercise the installed launch, two B3 publishers and two A3 services."""

    def test_both_robots_publish_to_team_paths_and_remain_fail_closed(self):
        # [핵심] 두 로봇 주소가 분리되고, 입력이 없을 때 모의 상태·통로 무효·명령 거절을 유지해야 합니다.
        import rclpy
        from ax_interfaces.msg import CorridorObservationList, RobotTaskStatus
        from ax_interfaces.srv import CheckCommandLease
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

        with tempfile.TemporaryDirectory() as folder, patch.dict(os.environ, {
                'ROS_DOMAIN_ID': str(180 + os.getpid() % 40),
                'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
                'ROS_LOG_DIR': str(Path(folder) / 'logs')}):
            log_path = Path(folder) / 'launch.log'
            with log_path.open('w') as log:
                process = subprocess.Popen(
                    ['ros2', 'launch', 'ax_task_executor', 'team_handoff.launch.py'],
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                rclpy.init()
                peer = Node('kim_handoff_smoke_peer')
                executor = SingleThreadedExecutor()
                executor.add_node(peer)
                state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
                statuses, observations = {}, {}
                subscriptions = []
                clients = {}
                for robot in ('robot_1', 'robot_2'):
                    subscriptions.append(peer.create_subscription(
                        RobotTaskStatus, f'/ax/{robot}/executor/status',
                        lambda msg, key=robot: statuses.update({key: msg}), state_qos))
                    subscriptions.append(peer.create_subscription(
                        CorridorObservationList, f'/ax/{robot}/perception/corridor_observation',
                        lambda msg, key=robot: observations.update({key: msg}), state_qos))
                    clients[robot] = peer.create_client(
                        CheckCommandLease, f'/ax/{robot}/executor/lease/check_command')

                def spin_until(predicate):
                    deadline = time.monotonic() + 15
                    while time.monotonic() < deadline:
                        executor.spin_once(timeout_sec=0.02)
                        if predicate():
                            return
                        if process.poll() is not None:
                            break
                    self.fail('team launch failed: ' + log_path.read_text()[-4000:])

                try:
                    spin_until(lambda: len(statuses) == len(observations) == 2
                               and all(client.service_is_ready() for client in clients.values()))
                    for robot, client in clients.items():
                        self.assertEqual(statuses[robot].robot_id, robot)
                        self.assertEqual(statuses[robot].meta.validity_reason, 'MOCK_ONLY')
                        self.assertFalse(statuses[robot].meta.valid)
                        self.assertEqual(observations[robot].robot_id, robot)
                        self.assertFalse(observations[robot].meta.valid)
                        self.assertEqual(observations[robot].meta.validity_reason,
                                         'REGIONS_NOT_APPROVED')
                        self.assertEqual(len(observations[robot].observations), 3)
                        request = CheckCommandLease.Request()
                        request.request_id = 'pair-check-' + robot
                        request.robot_id = robot
                        request.authority = request.AUTH_C_MANIPULATION
                        request.execution_id = 'pair-execution'
                        request.goal_id = 'a' * 32
                        request.lease_generation = 1
                        request.phase = request.ACCEPT
                        request.command_source = request.SOURCE_A4_ARM
                        request.command_id = 'pair-command'
                        request.command_digest = 'b' * 64
                        future = client.call_async(request)
                        spin_until(future.done)
                        self.assertFalse(future.result().allowed)
                        self.assertEqual(future.result().reason_code, 'ASSIGNMENT_STALE')
                finally:
                    executor.shutdown()
                    peer.destroy_node()
                    rclpy.shutdown()
                    if process.poll() is None:
                        os.killpg(process.pid, signal.SIGINT)
                        try:
                            process.wait(timeout=10)
                        except subprocess.TimeoutExpired:
                            os.killpg(process.pid, signal.SIGKILL)
                            process.wait(timeout=5)


if __name__ == '__main__':
    unittest.main()
