"""C3 rules, geometry, explicit validators, stale preferences and revalidation."""
# [검사 범위] C3 후보 생성·순위·선호·재검증 검사입니다. 도달·충돌 함수는 모의이며 실제 IK 성공은 아닙니다.
from dataclasses import replace
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src/ax_grasp_planner'))
from ax_grasp_planner.planner import (  # noqa: E402
    Constraints, plan_candidates, select_candidate, Surface)


class CandidateTests(unittest.TestCase):
    def setUp(self):
        self.surface = Surface('robot_1', 'box', 1, 100.0, 'map', (1.0, 2.0, 0.8),
                               (0.0, 0.0, 1.0), (1.0, 0.0, 0.0),
                               0.20, 0.20, 0.95, True, 0.001, 0.5)
        self.constraints = Constraints(1, 0.17, 0.13, 2.0)

    def plan(self, surface=None, constraints=None, **kwargs):
        return plan_candidates(surface or self.surface, constraints or self.constraints, 100.2,
                               reachability=kwargs.get('reachability', lambda pose: True),
                               collision=kwargs.get('collision', lambda *poses: True))

    def test_center_first_stable_ids_and_model_unavailable_probability(self):
        # [핵심] 중심 후보가 기본 1순위이고 같은 입력은 같은 ID여야 합니다. 학습 모델이 없으면 성공 확률은 -1입니다.
        result = self.plan()
        self.assertTrue(result.valid)
        self.assertEqual(len(result.candidates), 5)
        self.assertEqual(result.candidates[0].target.position, self.surface.center)
        self.assertEqual(result, self.plan())
        self.assertTrue(all(candidate.success_probability == -1 for candidate in result.candidates))

    def test_tcp_orientation_and_approach_retreat_normal(self):
        candidate = self.plan().candidates[0]
        self.assertAlmostEqual(sum(x * x for x in candidate.target.orientation), 1.0)
        x, y, z, w = candidate.target.orientation
        # TCP +Z transformed into map points down, along -surface normal.
        self.assertAlmostEqual(1 - 2 * (x * x + y * y), -1.0)
        self.assertGreater(candidate.approach.position[2], candidate.target.position[2])
        self.assertGreater(candidate.retreat.position[2], candidate.target.position[2])

    def test_missing_ik_or_collision_validator_never_claims_valid_candidates(self):
        # [핵심] 도달·충돌 검증기 없이 실행 가능한 후보라고 표시하면 안 됩니다.
        for validators in ((None, lambda *poses: True), (lambda pose: True, None)):
            result = self.plan(reachability=validators[0], collision=validators[1])
            self.assertFalse(result.valid)
            self.assertEqual(result.reason, 'VALIDATORS_UNAVAILABLE')

    def test_path_pose_unreachable_and_collision_remove_candidates(self):
        self.assertFalse(self.plan(reachability=lambda pose: pose.position[2] <= 0.85).valid)
        self.assertFalse(self.plan(collision=lambda *poses: False).valid)

    def test_tool_surface_mass_and_confidence_filters(self):
        for surface in (replace(self.surface, width_m=0.1), replace(self.surface, suction_suitable=False),
                        replace(self.surface, mass_kg=3.0), replace(self.surface, confidence=0.5),
                        replace(self.surface, flatness_error_m=0.01)):
            with self.subTest(surface=surface):
                self.assertFalse(self.plan(surface).valid)

    def test_bad_axes_nonfinite_and_stale_observation(self):
        for surface in (replace(self.surface, normal=(0, 0, 0)),
                        replace(self.surface, tangent=(0, 0, 1)),
                        replace(self.surface, center=(math.nan, 0, 0)),
                        replace(self.surface, stamp_s=90.0)):
            self.assertFalse(self.plan(surface).valid)

    def test_valid_preference_selects_member_but_stale_preference_uses_default(self):
        # [핵심] 선호는 현재 후보·장면일 때만 참고하고 오래됐으면 기본 순위를 사용합니다.
        plan = self.plan()
        preferred = plan.candidates[-1]
        preference = SimpleNamespace(proposal_id='proposal', candidate_set_id=plan.candidate_set_id,
                                     candidate_id=preferred.candidate_id, scene_revision=1)
        selected = select_candidate(plan, 100.3, scene_revision=1, constraint_version=1,
                                    reachability=lambda pose: True,
                                    collision=lambda *poses: True, preference=preference)
        self.assertEqual(selected, preferred)
        preference.scene_revision = 2
        self.assertEqual(select_candidate(plan, 100.3, scene_revision=1, constraint_version=1,
                                         reachability=lambda pose: True,
                                         collision=lambda *poses: True, preference=preference),
                         plan.candidates[0])

    def test_execution_revalidation_fallback_and_stale_snapshot(self):
        # [핵심] 실행 직전 후보를 재검증합니다. 중심이 막히면 다른 유효 후보를 찾고 만료된 묶음은 쓰지 않습니다.
        plan = self.plan()
        center = plan.candidates[0].target.position
        selected = select_candidate(plan, 100.3, scene_revision=1, constraint_version=1,
                                    reachability=lambda pose: pose.position != center,
                                    collision=lambda *poses: True)
        self.assertIsNotNone(selected)
        self.assertNotEqual(selected.target.position, center)
        self.assertIsNone(select_candidate(plan, 104.0, scene_revision=1, constraint_version=1,
                                           reachability=lambda pose: True,
                                           collision=lambda *poses: True))
        self.assertIsNone(select_candidate(plan, 100.3, scene_revision=2, constraint_version=1,
                                           reachability=lambda pose: True, collision=lambda *poses: True))

    def test_validator_exception_is_not_a_valid_candidate(self):
        def broken(_pose):
            raise RuntimeError('IK unavailable')
        self.assertFalse(self.plan(reachability=broken).valid)

    def test_changed_constraints_change_snapshot_identity(self):
        first = self.plan()
        second = self.plan(constraints=replace(self.constraints, version=2))
        self.assertNotEqual(first.candidate_set_id, second.candidate_set_id)

    def test_exact_expiry_and_changed_constraints_cannot_select_old_candidate(self):
        plan = self.plan()
        self.assertIsNone(select_candidate(plan, 103.0, scene_revision=1, constraint_version=1,
                                           reachability=lambda pose: True, collision=lambda *p: True))
        self.assertIsNone(select_candidate(plan, 100.3, scene_revision=1, constraint_version=2,
                                           reachability=lambda pose: True, collision=lambda *p: True))


if __name__ == '__main__':
    unittest.main()
