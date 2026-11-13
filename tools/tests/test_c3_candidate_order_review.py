"""Review replay ordering: delayed C3 results must not replace newer snapshots."""
from dataclasses import replace
from pathlib import Path
import sys
import threading
import unittest
from unittest.mock import Mock

import rclpy

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/ax_grasp_planner'))
from ax_grasp_planner.planner import Constraints, plan_candidates, Surface  # noqa: E402
from ax_grasp_planner.publisher import CandidatePublisher  # noqa: E402


class CandidateOrderingReviewTests(unittest.TestCase):
    """Check the implemented publisher, not an unavailable C2 or IK module."""

    def setUp(self):
        rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=false'])
        self.addCleanup(rclpy.shutdown)
        self.node = CandidatePublisher()
        self.addCleanup(self.node.destroy_node)
        self.node.publisher = Mock(wraps=self.node.publisher)
        self.now = self.node.get_clock().now().nanoseconds * 1e-9
        self.surface = Surface('robot_1', 'box', 2, self.now - 0.1, 'map', (1.0, 0.0, 0.8),
                               (0.0, 0.0, 1.0), (1.0, 0.0, 0.0), 0.2, 0.2, 0.95, True, 0.001, 0.5)
        self.constraints = Constraints(2, 0.17, 0.13, 2.0)

    def plan(self, surface=None, constraints=None):
        return plan_candidates(surface or self.surface, constraints or self.constraints, self.now,
                               reachability=lambda pose: True, collision=lambda *poses: True)

    def assert_delayed_plan_cannot_replace(self, delayed):
        current = self.plan()
        self.assertTrue(current.valid)
        self.assertTrue(delayed.valid)
        self.node.publish_plan(current)
        revision = self.node.revision
        received_wall = self.node.plans['box'][2]
        try:
            self.node.publish_plan(delayed)
        except ValueError:
            pass  # 예전 결과를 거절하거나 무시할 수 있지만 최신 결과를 덮어쓰면 안 됩니다.
        last = self.node.publisher.publish.call_args.args[0]
        self.assertEqual(last.candidate_sets[0].candidate_set_id, current.candidate_set_id)
        self.assertEqual(self.node.revision, revision)
        self.assertEqual(self.node.plans['box'][2], received_wall)

    def test_previous_scene_cannot_replace_current_scene(self):
        # [핵심] 장면 2를 게시한 뒤 늦게 온 장면 1이 최신 후보를 덮어쓰면 안 됩니다.
        self.assert_delayed_plan_cannot_replace(self.plan(replace(self.surface, scene_revision=1)))

    def test_previous_constraints_cannot_replace_current_constraints(self):
        # [핵심] 제약 버전이 바뀐 뒤 예전 설정으로 계산한 후보를 다시 게시하면 안 됩니다.
        self.assert_delayed_plan_cannot_replace(self.plan(constraints=replace(self.constraints, version=1)))

    def test_older_observation_cannot_replace_newer_observation(self):
        # [핵심] 같은 장면·제약이라도 더 오래된 관측의 늦은 결과가 현재 후보를 밀어내면 안 됩니다.
        self.assert_delayed_plan_cannot_replace(self.plan(replace(self.surface, stamp_s=self.now - 0.2)))

    def test_newer_versions_and_timestamp_are_accepted(self):
        self.node.publish_plan(self.plan())
        newer = self.plan(replace(self.surface, scene_revision=3, stamp_s=self.now - 0.05),
                          replace(self.constraints, version=3))
        self.node.publish_plan(newer)
        last = self.node.publisher.publish.call_args.args[0]
        self.assertEqual(last.candidate_sets[0].candidate_set_id, newer.candidate_set_id)
        self.assertTrue(last.candidate_sets[0].meta.valid)

    def test_same_result_republish_keeps_receipt_time_and_revision(self):
        current = self.plan()
        self.node.publish_plan(current)
        received_wall = self.node.plans['box'][2]
        revision = self.node.revision
        self.node.publish_plan(current)
        self.assertEqual(self.node.plans['box'][2], received_wall)
        self.assertEqual(self.node.revision, revision)

    def test_higher_scene_cannot_roll_back_constraints(self):
        self.assert_delayed_plan_cannot_replace(self.plan(
            replace(self.surface, scene_revision=3), replace(self.constraints, version=1)))

    def test_other_objects_have_independent_version_history(self):
        self.node.publish_plan(self.plan())
        other = self.plan(replace(self.surface, object_id='other-box', scene_revision=1),
                          replace(self.constraints, version=1))
        self.node.publish_plan(other)
        self.assertEqual(self.node.plans['other-box'][0], other)
        self.assertEqual(self.node.plans['box'][0].scene_revision, 2)

    def test_concurrent_results_keep_latest_scene(self):
        self.node.publish_plan(self.plan())
        barrier = threading.Barrier(3)
        errors = []

        def publish(revision, age):
            plan = self.plan(replace(self.surface, scene_revision=revision, stamp_s=self.now - age))
            barrier.wait()
            try:
                self.node.publish_plan(plan)
            except ValueError:
                pass  # 최신 결과가 먼저 저장됐다면 늦은 옛 결과의 거절은 정상입니다.
            except Exception as error:
                errors.append(error)

        threads = [threading.Thread(target=publish, args=arguments)
                   for arguments in ((3, 0.05), (4, 0.04))]
        for thread in threads:
            thread.start()
        barrier.wait()
        for thread in threads:
            thread.join(3)
            self.assertFalse(thread.is_alive())
        self.assertEqual(errors, [])
        self.assertEqual(self.node.plans['box'][0].scene_revision, 4)


if __name__ == '__main__':
    unittest.main()
