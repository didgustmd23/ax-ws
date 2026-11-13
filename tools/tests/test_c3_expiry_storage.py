"""ROS 그래프 없이 실제 C3 게시 함수의 공간 회수와 과거 재수신을 검사합니다."""
from dataclasses import replace
from pathlib import Path
import sys
import threading
from types import MethodType, SimpleNamespace
import unittest
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/ax_grasp_planner'))
from ax_grasp_planner.planner import Constraints, plan_candidates, Surface
from ax_grasp_planner.publisher import CandidatePublisher
from rclpy.time import Time


class CandidateExpiryStorageTests(unittest.TestCase):
    """시계·출력만 시험용으로 바꾸고 게시 구현과 후보 계산은 원본을 사용합니다."""

    def setUp(self):
        # ROS 노드를 띄우지 않고 sim 시계·벽시계를 따로 움직여 만료 경로를 재현합니다.
        self.sim, self.wall = 100.0, 100.0
        self.node = SimpleNamespace(
            robot_id='robot_1', session='expiry-test', revision=1, plans={},
            previous_content=None, publisher=Mock(), _plan_lock=threading.RLock(),
            get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=self.sim)),
            _steady_seconds=lambda: self.wall)
        for name in ('publish_plan', '_accept_plan', 'publish_snapshot', '_publish_snapshot',
                     '_prune_expired', '_expired', '_ensure_history'):
            if hasattr(CandidatePublisher, name):
                setattr(self.node, name, MethodType(getattr(CandidatePublisher, name), self.node))
        self.addCleanup(self.close_history)

    def close_history(self):
        """시험 인스턴스의 임시 버전 이력을 정리합니다."""
        if hasattr(self.node, '_plan_history'):
            self.node._plan_history.close()

    def plan(self, object_id='box', scene=2, constraints=2, stamp=None):
        """공간과 순서 시험용 후보를 실제 생성기로 계산합니다. IK는 모의입니다."""
        stamp = self.sim if stamp is None else stamp
        surface = Surface('robot_1', object_id, scene, stamp, 'map', (1.0, 0.0, 0.8),
                          (0.0, 0.0, 1.0), (1.0, 0.0, 0.0), 0.2, 0.2, 0.95, True, 0.001, 0.5)
        return plan_candidates(surface, Constraints(constraints, 0.17, 0.13, 2.0), stamp,
                               reachability=lambda pose: True, collision=lambda *poses: True)

    def test_ninth_object_accepted_after_all_eight_expire_without_timer(self):
        # 핵심: 타이머를 기다리지 않아도 접수 시 만료 공간을 회수해야 합니다.
        """주기 발행이 없어도 새 후보 접수 전에 만료 공간을 회수해야 합니다."""
        for index in range(8):
            self.node.publish_plan(self.plan(str(index)))
        self.assertEqual(len(self.node.plans), 8)
        self.sim += 4
        self.wall += 4
        self.node.publish_plan(self.plan('ninth'))
        self.assertEqual(set(self.node.plans), {'ninth'})

    def test_live_eight_objects_still_reject_ninth(self):
        """만료되지 않은 후보는 임의로 밀어내지 않습니다."""
        for index in range(8):
            self.node.publish_plan(self.plan(str(index)))
        with self.assertRaises(ValueError):
            self.node.publish_plan(self.plan('ninth'))

    def test_reclaimed_object_still_rejects_older_versions_and_stamp(self):
        """공간 회수와 버전 이력 삭제를 분리해 역순 입력을 계속 거절합니다."""
        original = self.plan()
        self.node.publish_plan(original)
        self.wall += 4
        self.node.publish_snapshot()
        self.assertNotIn('box', self.node.plans)
        for older in (replace(original, scene_revision=1),
                      replace(original, constraint_version=1),
                      replace(original, stamp_s=99.0)):
            with self.subTest(older=older), self.assertRaises(ValueError):
                self.node.publish_plan(older)

    def test_exact_replay_after_wall_expiry_cannot_restart_lifetime(self):
        # 핵심: sim 시각은 멈춰 두고 벽시계만 넘겨 같은 결과가 부활하는지 검사합니다.
        """sim 시각이 멈춰도 같은 결과의 재수신으로 벽시계 기한을 갱신하지 않습니다."""
        original = self.plan()
        self.node.publish_plan(original)
        self.wall += 4
        self.node.publish_snapshot()
        self.node.publish_plan(original)
        self.assertNotIn('box', self.node.plans)

    def test_new_observation_after_cleanup_is_accepted(self):
        """동일 물체의 새 관측은 만료된 이전 결과와 구분해 접수합니다."""
        self.node.publish_plan(self.plan())
        self.wall += 4
        self.sim += 4
        self.node.publish_snapshot()
        self.node.publish_plan(self.plan())
        self.assertIn('box', self.node.plans)

    def test_same_versions_with_changed_payload_are_rejected_after_cleanup(self):
        """같은 버전·시각의 다른 내용을 새 결과로 위장해 다시 넣지 못하게 합니다."""
        original = self.plan()
        self.node.publish_plan(original)
        self.wall += 4
        self.node.publish_snapshot()
        with self.assertRaises(ValueError):
            self.node.publish_plan(replace(original, candidate_set_id='changed'))
        self.assertNotIn('box', self.node.plans)

    def test_cleanup_emits_invalid_once_then_removes_from_full_snapshot(self):
        """만료 이유를 한 번 알린 뒤 전체 목록에서 빠지고 수정 번호가 증가합니다."""
        self.node.publish_plan(self.plan())
        self.wall += 4
        self.node.publish_snapshot()
        expired = self.node.publisher.publish.call_args.args[0]
        self.assertEqual(expired.candidate_sets[0].meta.validity_reason, 'CANDIDATE_STALE')
        self.assertEqual(self.node.plans, {})
        self.node.publish_snapshot()
        removed = self.node.publisher.publish.call_args.args[0]
        self.assertFalse(removed.meta.valid)
        self.assertEqual(list(removed.candidate_sets), [])
        self.assertGreater(removed.meta.revision, expired.meta.revision)

    def test_failed_ninth_admission_does_not_poison_version_history(self):
        # 핵심: 접수 실패한 높은 버전 때문에 이후 정상 낮은 버전까지 거절되면 안 됩니다.
        """공간 부족으로 거절한 물체는 접수 성공한 것처럼 이력을 남기지 않습니다."""
        for index in range(8):
            self.node.publish_plan(self.plan(str(index)))
        with self.assertRaises(ValueError):
            self.node.publish_plan(self.plan('ninth', scene=10))
        self.sim += 4
        self.wall += 4
        self.node.publish_plan(self.plan('ninth', scene=2))
        self.assertIn('ninth', self.node.plans)

    def test_concurrent_newer_results_keep_latest_history(self):
        """동시 후보 교체도 임시 원장과 활성 후보가 서로 다른 버전을 가리키지 않습니다."""
        plans = [self.plan(scene=revision) for revision in (2, 3)]
        barrier = threading.Barrier(3)
        errors = []

        def send(plan):
            """동시에 도착한 후보를 실제 잠금 경로로 접수합니다."""
            barrier.wait()
            try:
                self.node.publish_plan(plan)
            except ValueError:
                pass  # 최신 결과가 먼저 저장된 경우 옛 결과의 거절은 정상입니다.
            except Exception as error:
                errors.append(error)

        threads = [threading.Thread(target=send, args=(plan,)) for plan in plans]
        for thread in threads:
            thread.start()
        barrier.wait()
        for thread in threads:
            thread.join(3)
            self.assertFalse(thread.is_alive())
        self.assertEqual(errors, [])
        self.assertEqual(self.node.plans['box'][0].scene_revision, 3)
        self.assertEqual(self.node._plan_history.execute(
            'SELECT scene FROM versions WHERE object_id=?', ('box',)).fetchone()[0], 3)


if __name__ == '__main__':
    unittest.main()
