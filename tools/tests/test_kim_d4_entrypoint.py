"""내 A3 메시지를 상대 D4의 실제 main 수신 콜백까지 전달합니다. ROS 그래프는 사용하지 않습니다."""
from contextlib import redirect_stdout
from copy import deepcopy
import io
from pathlib import Path
import sqlite3
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
for relative in ('src/ax_task_executor', 'src/ax_d4', 'tools/tests'):
    sys.path.insert(0, str(ROOT / relative))

from ax_d4 import d4_main
from ax_d4.input_source import to_d4_status
from ax_task_executor.status_contract import make_unassigned_mock_idle
from rclpy.time import Time
import test_d4_w0 as fixture


class KimD4EntrypointTests(unittest.TestCase):
    """검증 함수만이 아니라 실제 D4 진입점의 출력·기록 흐름을 검사합니다."""

    def idle(self, robot):
        """A3가 사용하는 초기 상태 생성기로 시험 메시지를 만듭니다."""
        status = make_unassigned_mock_idle(robot, 'new-idle-session')
        status.meta.header.stamp = Time(seconds=100).to_msg()
        return to_d4_status(status)

    def run_main(self, path, messages):
        """ROS 수신기만 시험 입력으로 대체하고 D4 main 내부 콜백은 원본을 실행합니다."""
        def receive(on_status):
            # 소비자 콜백은 수정하지 않습니다. 실제 A3 메시지와 명시적 정상 형식 fixture를 전달합니다.
            for message in messages:
                on_status(message, now_sim_s=100, received_boottime_s=100)

        output = io.StringIO()
        with patch.dict(d4_main.os.environ, {'AX_D4_LEDGER_PATH': str(path)}), \
                patch.object(d4_main.input_source, 'run', side_effect=receive), \
                redirect_stdout(output):
            d4_main.main()
        return output.getvalue()

    def test_two_robot_idle_reaches_main_without_execution_error_or_database(self):
        """양쪽 로봇 대기가 main까지 도달해도 오류나 가짜 작업 원장을 만들지 않습니다."""
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'idle.db'
            output = self.run_main(path, [self.idle('robot_1'), self.idle('robot_2')])
            self.assertEqual(output.count('UNKNOWN: 유효한 상태가 아님'), 2)
            self.assertFalse(path.exists())

    def test_main_records_valid_fixture_and_idle_preserves_it(self):
        """실제 main이 정상 형식 입력을 저장하고 후속 대기로 이전 작업을 덮어쓰지 않습니다."""
        active = fixture.make_status()
        active['meta']['source_id'] = 'a3_mock_executor_robot_1'
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'active.db'
            first_output = self.run_main(path, [deepcopy(active)])
            self.assertIn('새 상태 기록 완료', first_output)
            with sqlite3.connect(path) as database:
                before = {table: database.execute('SELECT * FROM ' + table).fetchall()
                          for table in ('latest', 'history', 'sessions')}
            output = self.run_main(path, [self.idle('robot_1')])
            self.assertIn('UNKNOWN: 유효한 상태가 아님', output)
            with sqlite3.connect(path) as database:
                after = {table: database.execute('SELECT * FROM ' + table).fetchall()
                         for table in ('latest', 'history', 'sessions')}
            self.assertEqual(after, before)


if __name__ == '__main__':
    unittest.main()
