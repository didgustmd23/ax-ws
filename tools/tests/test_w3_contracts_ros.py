"""W3 actual ROS services/topics with test-only B3, A4, D2 and C2 inputs."""
# [검사 범위] D3 서비스와 C3 게시를 실제 ROS로 검사합니다. B3·A4·D2·C2 입력은 모의입니다.
from dataclasses import replace
import json
from pathlib import Path
import sys
import sqlite3
import tempfile
import time
import unittest

# 공용 잠금·그래프 검사 후 합의한 domain에서만 ROS 시험을 실행합니다.
import sys as _domain_sys
from pathlib import Path as _DomainPath
_domain_sys.path.insert(0, str(_DomainPath(__file__).resolve().parent))
from kim_ros_test_domain import setUpModule, tearDownModule  # noqa: E402, F401
from unittest.mock import patch

import rclpy
from ax_interfaces.msg import ActuationState, CandidateSetList, CorridorObservation, CorridorObservationList
from ax_interfaces.msg import CorridorStateList, GroupTaskList, GroupTaskStatus
from ax_interfaces.srv import ReleaseCorridor, RequestCorridorEntry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

ROOT = Path(__file__).resolve().parents[2]
for package in ('ax_corridor_manager', 'ax_grasp_planner', 'ax_environment'):
    sys.path.insert(0, str(ROOT / 'src' / package))
from ax_corridor_manager.node import CorridorNode, STATE_QOS  # noqa: E402
from ax_grasp_planner.planner import Constraints, plan_candidates, Surface  # noqa: E402
from ax_grasp_planner.publisher import CandidatePublisher  # noqa: E402


class W3RosTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        regions = json.loads((ROOT / 'src/ax_environment/config/ax_warehouse_regions.json').read_text())
        regions['approved'] = True  # temporary fixture, never the repository configuration
        config = Path(self.folder.name) / 'regions.json'
        config.write_text(json.dumps(regions))
        self.corridors = [row['id'] for row in regions['regions'] if row['kind'] == 'corridor']
        rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=false',
                         '-p', f'regions_file:={config}', '-p', f'database:={self.folder.name}/ledger.db'])
        self.addCleanup(rclpy.shutdown)
        self.manager = CorridorNode()
        self.candidates = CandidatePublisher()
        self.peer = Node('w3_test_peers')
        self.executor = SingleThreadedExecutor()
        for node in (self.manager, self.candidates, self.peer):
            self.executor.add_node(node)
        self.addCleanup(self.cleanup_nodes)
        self.b3, self.a4 = {}, {}
        for robot in ('robot_1', 'robot_2'):
            self.b3[robot] = self.peer.create_publisher(
                CorridorObservationList, f'/ax/{robot}/perception/corridor_observation', STATE_QOS)
            self.a4[robot] = self.peer.create_publisher(
                ActuationState, f'/ax/{robot}/safety/actuation_state', STATE_QOS)
        self.d2 = self.peer.create_publisher(GroupTaskList, '/ax/fleet/tasks', STATE_QOS)
        self.entry = self.peer.create_client(RequestCorridorEntry, '/ax/fleet/corridor/request_entry')
        self.release = self.peer.create_client(ReleaseCorridor, '/ax/fleet/corridor/release')
        self.states, self.plans = [], []
        self.peer.create_subscription(CorridorStateList, '/ax/fleet/corridor/state', self.states.append, STATE_QOS)
        self.peer.create_subscription(CandidateSetList, '/ax/robot_1/manipulation/candidates', self.plans.append, STATE_QOS)
        self.first_inside = False
        self.send_enabled = True
        self.serial = 0
        self.request_counter = 0

    def cleanup_nodes(self):
        self.executor.shutdown()
        self.peer.destroy_node()
        self.candidates.destroy_node()
        self.manager.destroy_node()

    def fill_meta(self, meta, source):
        meta.header.stamp = self.peer.get_clock().now().to_msg()
        meta.header.frame_id = 'map'
        meta.valid = True
        meta.schema_version = 1
        meta.source_session_id = source
        meta.source_id = source
        meta.revision = self.serial

    def inputs(self):
        if not self.send_enabled:
            return
        self.serial += 1
        for robot in self.b3:
            observation = CorridorObservationList()
            observation.robot_id = robot
            self.fill_meta(observation.meta, 'b3-test-' + robot)
            for corridor in self.corridors:
                item = CorridorObservation()
                self.fill_meta(item.meta, 'b3-test-' + robot)
                item.robot_id, item.corridor_id = robot, corridor
                item.position_valid = True
                item.observation = item.INSIDE if self.first_inside and robot == 'robot_1' else item.OUTSIDE
                observation.observations.append(item)
            self.b3[robot].publish(observation)
            stop = ActuationState()
            stop.robot_id = robot
            self.fill_meta(stop.meta, 'a4-test-' + robot)
            stop.state = stop.STOPPED
            self.a4[robot].publish(stop)
        tasks = GroupTaskList()
        self.fill_meta(tasks.meta, 'd2-test')
        task = GroupTaskStatus()
        task.task_id = 'task-test'
        task.state = task.RUNNING
        task.assignment_generation = 1
        task.manipulator_robot_id, task.transporter_robot_id = 'robot_1', 'robot_2'
        tasks.tasks.append(task)
        self.d2.publish(tasks)

    def wait_for(self, condition, timeout=4):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.inputs()
            self.executor.spin_once(timeout_sec=0.01)
            if condition():
                return
        self.fail('W3 ROS contract condition timed out')

    def latest(self):
        return next(item for item in self.states[-1].corridors if item.corridor_id == self.corridors[0])

    def call(self, client, request):
        self.assertTrue(client.wait_for_service(timeout_sec=3))
        future = client.call_async(request)
        self.wait_for(future.done)
        return future.result()

    def entry_request(self, robot, request_id):
        request = RequestCorridorEntry.Request()
        request.corridor_id = self.corridors[0]
        request.robot_id, request.request_id = robot, request_id
        return self.call(self.entry, request)

    def release_request(self, grant, request_id):
        request = ReleaseCorridor.Request()
        request.corridor_id, request.robot_id = self.corridors[0], 'robot_1'
        request.grant_id, request.request_id = grant, request_id
        return self.call(self.release, request)

    def test_d3_ros_entry_occupancy_exit_and_stale_owner(self):
        # [핵심] 실제 서비스로 허가·점유·해제를 확인하고, 관측을 끊어도 현재 owner/grant가 보존되는지 봅니다.
        self.wait_for(lambda: bool(self.states) and self.latest().state == self.latest().FREE)
        grant = self.entry_request('robot_1', 'first')
        self.assertTrue(grant.granted)
        self.assertEqual(self.entry_request('robot_1', 'first').grant_id, grant.grant_id)
        self.assertFalse(self.entry_request('robot_2', 'wait').granted)
        self.first_inside = True
        self.wait_for(lambda: self.latest().state == self.latest().OCCUPIED)
        self.assertFalse(self.release_request(grant.grant_id, 'inside-release').accepted)
        self.first_inside = False
        self.wait_for(lambda: self.manager.manager._observation('robot_1', self.corridors[0])[0] == 0)
        self.assertTrue(self.release_request(grant.grant_id, 'exit-release').accepted)
        next_grant = self.entry_request('robot_2', 'next')
        self.assertTrue(next_grant.granted)
        self.send_enabled = False
        self.wait_for(lambda: self.latest().state == self.latest().BLOCKED_UNKNOWN)
        self.assertEqual(self.latest().owner_robot_id, 'robot_2')
        self.assertEqual(self.latest().active_grant_id, next_grant.grant_id)

    def test_c3_ros_empty_valid_and_expired_snapshot(self):
        # [핵심] 입력 없음→유효 후보→만료 상태를 확인합니다. 같은 관측 재게시로 기한을 늘리면 안 됩니다.
        self.wait_for(lambda: bool(self.plans))
        self.assertFalse(self.plans[-1].meta.valid)
        now = self.peer.get_clock().now().nanoseconds * 1e-9
        surface = Surface('robot_1', 'box', 1, now, 'map', (1.0, 0.0, 0.8),
                          (0.0, 0.0, 1.0), (1.0, 0.0, 0.0), 0.2, 0.2, 0.95, True, 0.001, 0.5)
        constraints = Constraints(1, 0.17, 0.13, 2.0, candidate_ttl_s=0.5)
        plan = plan_candidates(surface, constraints, now,
                               reachability=lambda pose: True, collision=lambda *poses: True)
        self.candidates.publish_plan(plan)
        self.wait_for(lambda: self.plans[-1].meta.valid)
        item = self.plans[-1].candidate_sets[0]
        self.assertEqual(item.default_ranking[0], plan.candidates[0].candidate_id)
        self.assertTrue(all(candidate.success_probability == -1 for candidate in item.candidates))
        old_received = self.candidates.plans['box'][2]
        self.candidates.publish_plan(plan)
        self.assertEqual(self.candidates.plans['box'][2], old_received)
        self.wait_for(lambda: not self.plans[-1].meta.valid)
        self.assertEqual(self.plans[-1].candidate_sets[0].meta.validity_reason, 'CANDIDATE_STALE')
        with self.assertRaises(ValueError):
            self.candidates.publish_plan(replace(plan, robot_id='robot_2'))

    def test_d3_ledger_error_latches_rejection_and_invalid_state(self):
        # [핵심] 원장 오류 뒤에는 명령을 거절하고 무효 상태를 유지해야 합니다. 몰래 정상으로 복귀하면 안 됩니다.
        self.wait_for(lambda: bool(self.states) and self.latest().state == self.latest().FREE)
        with patch.object(self.manager.manager, 'request',
                          side_effect=sqlite3.OperationalError('disk unavailable')):
            result = self.entry_request('robot_1', 'io-failed')
        self.assertFalse(result.granted)
        self.assertEqual(result.reason_code, 'CORRIDOR_LEDGER_FAILED')
        self.assertFalse(self.entry_request('robot_1', 'no-auto-recovery').granted)
        self.wait_for(lambda: not self.states[-1].meta.valid)


if __name__ == '__main__':
    unittest.main()
