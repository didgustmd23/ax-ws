"""A3 미배정 모의 대기의 표현과 실제 게시 경로를 ROS 그래프 없이 검사합니다."""
from copy import deepcopy
from pathlib import Path
import sys
import threading
from types import MethodType, SimpleNamespace
import unittest
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/ax_task_executor'))
from ax_task_executor.mock_executor import MockExecutor
from ax_task_executor.status_contract import is_unassigned_mock_idle, make_unassigned_mock_idle
from rclpy.time import Time


class UnassignedMockIdleTests(unittest.TestCase):
    """대기 상태가 가짜 작업·성공·권한으로 해석되지 않도록 확인합니다."""

    def test_both_robot_idle_messages_have_no_execution_or_authority(self):
        """두 로봇 모두 실제 배정이 없는 대기임을 동일한 형태로 표현합니다."""
        for robot in ('robot_1', 'robot_2'):
            with self.subTest(robot=robot):
                status = make_unassigned_mock_idle(robot, 'session')
                self.assertTrue(is_unassigned_mock_idle(status))
                self.assertFalse(status.meta.valid)
                self.assertEqual(status.execution.execution_id, '')
                self.assertEqual(status.execution.derive_rule_version, '')
                self.assertEqual(status.last_result.outcome, 0)

    def test_partial_execution_is_not_misclassified_as_unassigned(self):
        """실행 정보 한 항목이라도 남아 있으면 단순 대기로 우회 분류하지 않습니다."""
        for field, value in (('task_id', 'task-existing'), ('assignment_generation', 1),
                             ('execution_id', 'existing'), ('derive_rule_version', 'derive-v1')):
            with self.subTest(field=field):
                status = make_unassigned_mock_idle('robot_1', 'session')
                setattr(status.execution, field, value)
                self.assertFalse(is_unassigned_mock_idle(status))

    def test_wrong_metadata_phase_pause_or_result_is_not_idle(self):
        """MOCK_IDLE 이름만 붙여 결과나 유효한 실행 상태의 검사를 우회하지 못하게 합니다."""
        original = make_unassigned_mock_idle('robot_1', 'session')
        variants = []
        for target, field, value in (
                ('meta', 'valid', True), ('meta', 'source_session_id', ''),
                ('meta', 'source_id', 'other'), ('meta', 'revision', 0),
                ('last_result', 'outcome', 1), ('self', 'paused', True),
                ('self', 'local_phase', 'MOCK_CHECKING_LOAD')):
            status = deepcopy(original)
            setattr(status if target == 'self' else getattr(status, target), field, value)
            variants.append(status)
        for status in variants:
            self.assertFalse(is_unassigned_mock_idle(status))

    def test_real_publisher_keeps_idle_shape_and_updates_only_stamp(self):
        """실제 A3 게시 함수의 heartbeat가 작업을 만들거나 수정 번호를 올리지 않습니다."""
        status = make_unassigned_mock_idle('robot_1', 'session')
        clock = [100]
        source = SimpleNamespace(
            task_status=status, _status_lock=threading.Lock(), task_status_publisher=Mock(),
            get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=clock[0])))
        source.publish_task_status = MethodType(MockExecutor.publish_task_status, source)
        source.publish_task_status()
        clock[0] += 1
        source.publish_task_status()
        sent = [call.args[0] for call in source.task_status_publisher.publish.call_args_list]
        self.assertTrue(all(is_unassigned_mock_idle(item) for item in sent))
        self.assertEqual(sent[0].meta.revision, sent[1].meta.revision)
        self.assertNotEqual(sent[0].meta.header.stamp, sent[1].meta.header.stamp)

    def test_unknown_robot_or_missing_session_is_rejected(self):
        """출처를 추적할 수 없는 초기 상태는 생성하지 않습니다."""
        for robot, session in (('unknown', 'session'), ('robot_1', '')):
            with self.assertRaises(ValueError):
                make_unassigned_mock_idle(robot, session)


if __name__ == '__main__':
    unittest.main()
