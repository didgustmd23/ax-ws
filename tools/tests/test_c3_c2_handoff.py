"""C2 실제 계산·어댑터에서 내 C3 계산·게시 입구까지 검사합니다. 입력과 IK는 합성입니다."""
from dataclasses import replace
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools/tests'))
sys.path.insert(0, str(ROOT / 'src/ax_grasp_planner'))
import test_c2_surface as fixture
from ax_grasp_planner.c2_handoff import prepare_c2_candidates
from ax_grasp_planner.planner import Constraints
from ax_grasp_planner.publisher import CandidatePublisher


class C2C3HandoffTests(unittest.TestCase):
    """다른 담당자의 파일은 읽기만 하며 정상 대응을 실제로 검증했다고 주장하지 않습니다."""

    @classmethod
    def setUpClass(cls):
        """C2의 합성 깊이 계산과 시각이 결합된 변환을 원본 함수로 수행합니다."""
        observation = fixture.estimate(fixture.front_plane())
        cls.observation = fixture.se.transform_observation(
            observation, fixture.np.eye(4), 'robot_1/base_link', fixture.FRAME, fixture.STAMP)

    def adapted(self, **changes):
        """대상 대응·질량·신뢰도는 명시적 시험 fixture로만 제공합니다."""
        return fixture.to_surface(self.observation, fixture.fixture_inputs(self.observation, **changes))

    def prepare(self, adapted, **changes):
        """C3 연결부를 원래 시각과 검증기 설정으로 호출합니다."""
        arguments = dict(robot_id='robot_1', reachability=lambda pose: True,
                         collision=lambda *poses: True)
        arguments.update(changes)
        return prepare_c2_candidates(adapted, Constraints(1, 0.17, 0.13, 2.0),
                                     fixture.STAMP + 100_000_000, **arguments)

    def test_complete_fixture_preserves_registered_id_and_source(self):
        """C2→C3가 관측 ID를 물체 ID로 대체하거나 stamp를 현재로 바꾸지 않습니다."""
        adapted = self.adapted()
        result = self.prepare(adapted)
        self.assertTrue(result.ready, result.reasons)
        self.assertEqual(result.plan.object_id, 'obj-test-0001')
        self.assertEqual(result.plan.frame_id, 'robot_1/base_link')
        self.assertEqual(result.stamp_ns, self.observation.stamp_ns)
        self.assertEqual(result.provenance, adapted.provenance)
        self.assertEqual(result.plan.stamp_s, self.observation.stamp_s)

    def test_missing_association_or_metadata_never_publishes(self):
        """실제 C4 대응 판정이 없으면 평면이 유효해도 게시하지 않습니다."""
        node = SimpleNamespace(robot_id='robot_1', publish_plan=Mock())
        for changes in (dict(target_association=None), dict(confidence=None), dict(mass_kg=None)):
            with self.subTest(changes=changes):
                adapted = self.adapted(**changes)
                result = CandidatePublisher.publish_c2_result(
                    node, adapted, Constraints(1, 0.17, 0.13, 2.0), fixture.STAMP + 100_000_000)
                self.assertFalse(result.ready)
                self.assertEqual(result.reasons, adapted.reasons)
        node.publish_plan.assert_not_called()

    def test_validators_are_required_even_after_adapter_accepts(self):
        """C2 평면·대응 fixture 통과를 IK·충돌 성공으로 해석하지 않습니다."""
        result = self.prepare(self.adapted(), reachability=None, collision=None)
        self.assertFalse(result.ready)
        self.assertEqual(result.reasons, ('VALIDATORS_UNAVAILABLE',))

    def test_wrong_robot_or_inconsistent_stamp_is_held(self):
        """다른 로봇이나 시각을 바꾼 결과는 C3 입력에서도 차단합니다."""
        adapted = self.adapted()
        self.assertEqual(self.prepare(adapted, robot_id='robot_2').reasons, ('C2_ROBOT_MISMATCH',))
        bad = replace(adapted, surface=replace(adapted.surface, stamp_s=101.0))
        self.assertEqual(self.prepare(bad).reasons, ('C2_STAMP_MISMATCH',))

    def test_epoch_stamp_expiry_checks_integer_ns_before_float(self):
        """float로 같아 보이는 1ns 차이도 TTL 초과와 미래 관측을 구분합니다."""
        adapted = self.adapted()
        stamp = 1_794_923_490_123_456_789
        adapted = replace(adapted, surface=replace(adapted.surface, stamp_s=stamp / 1_000_000_000),
                          provenance=dict(adapted.provenance, stamp_ns=stamp))
        constraint = Constraints(1, 0.17, 0.13, 2.0)
        result = prepare_c2_candidates(adapted, constraint, stamp + 1_000_000_001,
                                       robot_id='robot_1')
        self.assertEqual(result.reasons, ('OBSERVATION_STALE_OR_INVALID',))
        result = prepare_c2_candidates(adapted, constraint, stamp - 1, robot_id='robot_1')
        self.assertEqual(result.reasons, ('C2_STAMP_INVALID',))

    def test_ready_plan_uses_existing_publisher_path(self):
        """새 후보는 기존 버전·지문·TTL 검사 입구를 우회하지 않습니다."""
        node = SimpleNamespace(robot_id='robot_1', publish_plan=Mock())
        result = CandidatePublisher.publish_c2_result(
            node, self.adapted(), Constraints(1, 0.17, 0.13, 2.0), fixture.STAMP + 100_000_000,
            reachability=lambda pose: True, collision=lambda *poses: True)
        self.assertTrue(result.ready)
        node.publish_plan.assert_called_once_with(result.plan)

    def test_invalid_ttl_is_held_instead_of_crashing(self):
        """외부 제약이 비정상적으로 커도 예외로 실행기를 종료하지 않고 보류해야 합니다."""
        adapted = self.adapted()
        for ttl in (0, -1, True, float('nan'), float('inf'), 1e308, 10 ** 400):
            with self.subTest(ttl_type=type(ttl).__name__):
                result = prepare_c2_candidates(
                    adapted, replace(Constraints(1, 0.17, 0.13, 2.0), observation_ttl_s=ttl),
                    fixture.STAMP + 100_000_000, robot_id='robot_1')
                self.assertFalse(result.ready)
                self.assertEqual(result.reasons, ('CONSTRAINTS_INVALID',))

    def test_missing_or_wrong_stamp_is_not_published(self):
        """원본 정수 ns가 빠지거나 잘못된 형식이면 float 시각으로 복구하지 않습니다."""
        adapted = self.adapted()
        node = SimpleNamespace(robot_id='robot_1', publish_plan=Mock())
        for stamp in (None, 0, True, 100.0):
            result = CandidatePublisher.publish_c2_result(
                node, replace(adapted, provenance=dict(adapted.provenance, stamp_ns=stamp)),
                Constraints(1, 0.17, 0.13, 2.0), fixture.STAMP + 100_000_000)
            self.assertFalse(result.ready)
            self.assertEqual(result.reasons, ('C2_STAMP_INVALID',))
        node.publish_plan.assert_not_called()

    def test_validator_failure_never_becomes_success(self):
        """실제 검증기 장애·충돌 거절은 후보 성공이나 게시로 바뀌지 않습니다."""
        def failed_validator(*args):
            raise RuntimeError('validator unavailable')

        for changes in (dict(reachability=failed_validator), dict(collision=failed_validator),
                        dict(reachability=lambda pose: False), dict(collision=lambda *poses: False)):
            result = self.prepare(self.adapted(), **changes)
            self.assertFalse(result.ready)
            self.assertFalse(result.plan.candidates)
