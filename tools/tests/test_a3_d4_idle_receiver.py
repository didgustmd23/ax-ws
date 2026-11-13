"""A3 초기 모의 대기를 D4 원본 검증·원장에 직접 전달해 검사합니다. ROS 노드는 없습니다."""
from copy import deepcopy
from pathlib import Path
import sqlite3
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
for relative in ('src/ax_task_executor', 'src/ax_d4/ax_d4', 'tools/tests'):
    sys.path.insert(0, str(ROOT / relative))
sys.path.insert(0, str(ROOT.parent / '.ax-ws-backups/d4-review-deps'))

from ax_task_executor.status_contract import make_unassigned_mock_idle
from common_contract import check_status, is_unassigned_mock_idle
from input_source import to_d4_status
from ledger import accept_status
from rclpy.time import Time
import test_d4_w0 as fixture


class IdleReceiverTests(unittest.TestCase):
    """초기 대기만 예외로 구분하고 일반 실행 검증·원장 보존을 확인합니다."""

    def idle(self, robot='robot_1'):
        """A3 실제 생성 함수를 사용해 같은 시계의 초기 상태를 만듭니다."""
        message = make_unassigned_mock_idle(robot, 'session-1')
        message.meta.header.stamp = Time(seconds=100).to_msg()
        return to_d4_status(message)

    def test_idle_for_both_robots_is_unknown_without_creating_ledger(self):
        """초기 대기는 오류 로그 대신 UNKNOWN이며 작업 원장을 만들지 않습니다."""
        with tempfile.TemporaryDirectory() as directory:
            for robot in ('robot_1', 'robot_2'):
                with self.subTest(robot=robot):
                    status = self.idle(robot)
                    self.assertIs(check_status(status), status)
                    path = Path(directory) / (robot + '.db')
                    result = accept_status(status, path, now_sim_s=100, received_boottime_s=100)
                    self.assertTrue(result.startswith('UNKNOWN'))
                    self.assertFalse(path.exists())

    def test_idle_does_not_replace_existing_active_record_or_history(self):
        """같은 출처의 새 대기 세션이 기존 작업을 완료·해제·덮어쓰기하지 않습니다."""
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'ledger.db'
            active = fixture.make_status()
            active['meta']['source_id'] = 'a3_mock_executor_robot_1'
            accept_status(active, path, now_sim_s=100, received_boottime_s=100)
            with sqlite3.connect(path) as database:
                before = {table: database.execute('SELECT * FROM ' + table).fetchall()
                          for table in ('latest', 'history', 'sessions')}
            idle = self.idle()
            idle['meta']['source_session_id'] = 'new-session'
            response = accept_status(idle, path, now_sim_s=100, received_boottime_s=101)
            self.assertTrue(response.startswith('UNKNOWN'))
            with sqlite3.connect(path) as database:
                after = {table: database.execute('SELECT * FROM ' + table).fetchall()
                         for table in ('latest', 'history', 'sessions')}
            self.assertEqual(after, before)

    def test_partial_execution_cannot_bypass_validation(self):
        """부분 작업 정보가 남은 메시지는 기존 실행 번호 검사로 거절합니다."""
        for field, value in (('task_id', 'task-existing'), ('assignment_generation', 1),
                             ('execution_id', 'existing'), ('derive_rule_version', 'derive-v1')):
            with self.subTest(field=field):
                status = self.idle()
                status['execution'][field] = value
                self.assertFalse(is_unassigned_mock_idle(status))
                with self.assertRaises(ValueError):
                    check_status(status)

    def test_metadata_or_result_cannot_disguise_execution_as_idle(self):
        """유효한 상태·성공 결과·잘못된 출처를 초기 모의 대기로 통과시키지 않습니다."""
        for target, field, value in (
                ('meta', 'valid', True), ('meta', 'source_id', 'other'),
                ('meta', 'source_session_id', ''), ('meta', 'revision', 0),
                ('meta', 'schema_version', 2), ('last_result', 'outcome', 1),
                ('self', 'paused', True), ('self', 'local_phase', 'MOCK_CHECKING_LOAD')):
            with self.subTest(field=field, value=value):
                status = self.idle()
                setattr_target = status if target == 'self' else status[target]
                setattr_target[field] = value
                with self.assertRaises(ValueError):
                    check_status(status)

    def test_structure_and_numeric_types_are_still_checked_first(self):
        """모의 상태라도 필드 누락과 잘못된 수치 자료형은 형식 오류입니다."""
        malformed = self.idle()
        del malformed['execution']['task_id']
        with self.assertRaises(ValueError):
            check_status(malformed)
        malformed = self.idle()
        malformed['execution']['assignment_generation'] = False
        with self.assertRaises(ValueError):
            check_status(malformed)

    def test_normal_execution_validation_remains_strict(self):
        """정상 실행은 기존 계산 규칙을 그대로 검사하고 잘못된 실행 ID는 거절합니다."""
        status = fixture.make_status()
        self.assertIs(check_status(status), status)
        changed = deepcopy(status)
        changed['execution']['execution_id'] = 'bad'
        with self.assertRaises(ValueError):
            check_status(changed)


if __name__ == '__main__':
    unittest.main()
