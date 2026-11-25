import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src" / "ax_c4"))

from ax_c4.c4_state_machine import (  # noqa: E402
    Budget,
    C4StateMachine,
    ExecutionResult,
    GoalSnapshot,
    LeaseGrant,
)


def make_goal(**changes):
    values = dict(
        goal_id="0123456789abcdef0123456789abcdef",
        robot_id="robot_1",
        execution_id="exec-1",
        execution_operation="PICK",
        assignment_id="asg-1",
        task_revision=1,
        root_execution_id="root-1",
        step_key="pick",
        derive_rule_version="derive-v1",
        attempt_index=0,
        task_id="task-1",
        assignment_generation=1,
        object_id="box-1",
        operation=C4StateMachine.PICK,
        peer_robot_id="",
        target_frame="map",
        target_values_finite=True,
        target_stamp_valid=True,
        peer_verified=True,
        budgets=(Budget("budget-1", "GRASP_ATTEMPT", 1, ("exec-1",), 5.0),),
    )
    values.update(changes)
    return GoalSnapshot(**values)


class C4StateMachineTests(unittest.TestCase):
    def setUp(self):
        self.machine = C4StateMachine("robot_1")

    def accepted(self, goal=None):
        execution, cached, validation = self.machine.accept(goal or make_goal())
        self.assertTrue(validation.ok)
        self.assertIsNotNone(execution)
        self.assertIsNone(cached)
        self.assertTrue(self.machine.set_lease(
            execution.goal.execution_id,
            LeaseGrant("lease-request", execution.goal.goal_id,
                        execution.goal.execution_id, 1, True, 10.0),
            0.0).ok)
        return execution

    def test_pick_budget_and_lease_allow_one_attempt(self):
        execution = self.accepted()
        self.assertTrue(self.machine.start_attempt(execution.goal.execution_id).ok)
        self.assertFalse(self.machine.start_attempt(execution.goal.execution_id).ok)

    def test_retry_budget_allows_only_the_granted_number_of_attempts(self):
        goal = make_goal(
            budgets=(Budget("budget-1", "GRASP_ATTEMPT", 2, ("exec-1",), 5.0),))
        execution = self.accepted(goal)
        self.assertTrue(self.machine.start_attempt(
            execution.goal.execution_id, now_s=0.0).ok)
        self.assertTrue(self.machine.finish_attempt(
            execution.goal.execution_id, "FINISHED", 0.2, "A2_TIMEOUT").ok)
        self.assertTrue(self.machine.start_attempt(
            execution.goal.execution_id, now_s=0.3).ok)
        self.assertFalse(self.machine.start_attempt(
            execution.goal.execution_id, now_s=0.4).ok)

    def test_duplicate_active_execution_is_rejected(self):
        self.accepted()
        _, result, validation = self.machine.accept(make_goal())
        self.assertTrue(validation.ok)
        self.assertEqual(result.error_code, "DUPLICATE_REQUEST")

    def test_different_arguments_with_same_execution_id_are_rejected(self):
        self.accepted()
        _, result, _ = self.machine.accept(make_goal(object_id="other-box"))
        self.assertEqual(result.error_code, "DUPLICATE_REQUEST")

    def test_expired_or_wrong_lease_is_blocked(self):
        execution, _, validation = self.machine.accept(make_goal())
        self.assertTrue(validation.ok)
        wrong = LeaseGrant("r", "other-goal", "exec-1", 1, True, 1.0)
        check = self.machine.set_lease(execution.goal.execution_id, wrong, 0.0)
        self.assertEqual(check.reason_code, "GOAL_MISMATCH")
        expired = LeaseGrant("r", execution.goal.goal_id, "exec-1", 1, True, 0.1)
        check = self.machine.set_lease(execution.goal.execution_id, expired, 0.1)
        self.assertEqual(check.reason_code, "LEASE_EXPIRED")

    def test_runtime_budget_timeout_is_blocked(self):
        execution = self.accepted()
        self.assertTrue(self.machine.start_attempt(
            execution.goal.execution_id, now_s=0.0).ok)
        check = self.machine.check_runtime(execution.goal.execution_id, 5.1)
        self.assertEqual(check.reason_code, "TIMED_OUT")

    def test_renew_requires_the_current_generation(self):
        execution = self.accepted()
        wrong = LeaseGrant("renew", execution.goal.goal_id,
                           execution.goal.execution_id, 2, True, 1.0)
        self.assertEqual(
            self.machine.renew_lease(execution.goal.execution_id, wrong, 0.5).reason_code,
            "LEASE_REVOKED")
        right = LeaseGrant("renew", execution.goal.goal_id,
                           execution.goal.execution_id, 1, True, 1.0)
        self.assertTrue(
            self.machine.renew_lease(execution.goal.execution_id, right, 0.5).ok)

    def test_attempt_usage_records_failure_and_elapsed_time(self):
        execution = self.accepted()
        self.assertTrue(self.machine.start_attempt(
            execution.goal.execution_id, now_s=2.0).ok)
        self.assertTrue(self.machine.finish_attempt(
            execution.goal.execution_id, "FINISHED", 2.4, "A2_TIMEOUT").ok)
        attempt = execution.attempts[0]
        self.assertEqual(attempt.failure_code, "A2_TIMEOUT")
        self.assertAlmostEqual(attempt.elapsed_s, 0.4)

    def test_execution_ref_operation_mismatch_is_blocked(self):
        _, _, validation = self.machine.accept(
            make_goal(execution_operation="PLACE"))
        self.assertEqual(validation.reason_code, "AUTHORITY_INVALID")

    def test_execution_ref_missing_field_is_blocked(self):
        _, _, validation = self.machine.accept(
            make_goal(assignment_id=""))
        self.assertEqual(validation.reason_code, "AUTHORITY_INVALID")

    def test_cancel_prevents_physical_attempt(self):
        execution = self.accepted()
        self.assertTrue(self.machine.request_cancel(execution.goal.execution_id).ok)
        self.assertFalse(self.machine.start_attempt(execution.goal.execution_id).ok)

    def test_matching_revoke_blocks_the_current_generation(self):
        execution = self.accepted()
        check = self.machine.revoke(
            execution.goal.execution_id,
            execution.goal.goal_id,
            execution.lease.lease_generation,
            "SAFETY_STOP",
        )
        self.assertTrue(check.ok)
        self.assertEqual(execution.revoke_reason, "SAFETY_STOP")
        self.assertFalse(self.machine.start_attempt(execution.goal.execution_id).ok)

    def test_missing_budget_is_blocked(self):
        goal = make_goal(budgets=())
        _, _, validation = self.machine.accept(goal)
        self.assertEqual(validation.reason_code, "BUDGET_EXHAUSTED")

    def test_invalid_pose_and_unverified_peer_are_blocked(self):
        _, _, validation = self.machine.accept(make_goal(target_values_finite=False))
        self.assertEqual(validation.reason_code, "PRECONDITION_FAILED")
        _, _, validation = self.machine.accept(make_goal(
            execution_id="exec-2", peer_robot_id="robot_2", peer_verified=False))
        self.assertEqual(validation.reason_code, "PRECONDITION_FAILED")

    def test_attach_evidence_requires_fresh_matching_state(self):
        common = dict(
            robot_id="robot_1", object_id="box-1", message_robot_id="robot_1",
            message_object_id="box-1", attached=True, want_attached=True,
            meta_valid=True, revision=1, source_session_id="a2-session",
            stamp_s=10.0, now_s=10.2, ttl_s=0.3)
        self.assertTrue(self.machine.validate_attach_evidence(**common).ok)
        self.assertEqual(
            self.machine.validate_attach_evidence(**{**common, "now_s": 11.0}).reason_code,
            "ATTACH_STATE_STALE")
        self.assertEqual(
            self.machine.validate_attach_evidence(**{**common, "message_object_id": "box-2"}).reason_code,
            "ATTACH_STATE_MISMATCH")

    def test_completed_result_is_idempotent(self):
        execution = self.accepted()
        result = self.machine.finish(
            execution.goal.execution_id,
            ExecutionResult("SUCCEEDED", hold_state=C4StateMachine.HELD),
        )
        _, cached, validation = self.machine.accept(make_goal())
        self.assertTrue(validation.ok)
        self.assertIsNone(_)
        self.assertEqual(cached, result)


if __name__ == "__main__":
    unittest.main()
