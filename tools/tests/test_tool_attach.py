# A2 부착 시험 도구의 결과 기록(판정 추가·실패 집계·중간 저장)을 Gazebo 없이 검사한다. 부착 성능 검증이 아니다.
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / 'src/ax_control/scripts'
sys.path.insert(0, str(SCRIPTS))

from result_record import write_json_atomic  # noqa: E402


# 공용 저장 함수: 임시 파일을 남기지 않고 덮어쓴다
class AtomicWrite(unittest.TestCase):
    def test_overwrite_without_temporary(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'result.json'

            write_json_atomic(path, {'status': 'running'})
            write_json_atomic(path, {'status': 'completed', '값': 1.5})

            self.assertEqual(json.loads(path.read_text(encoding='utf-8')), {'status': 'completed', '값': 1.5})
            self.assertEqual([item.name for item in Path(folder).iterdir()], ['result.json'])


# 부착 시험 기록: 판정마다 저장되고 측정값이 숫자로 남아야 한다
class ToolAttachRecord(unittest.TestCase):
    def setUp(self):
        spec = importlib.util.spec_from_file_location('check_tool_attach', SCRIPTS / 'check_tool_attach.py')

        try:
            self.tool = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(self.tool)
        except ImportError as error:
            self.skipTest(f'ROS 환경 필요: {error}')

        self.folder = tempfile.TemporaryDirectory()
        self.path = Path(self.folder.name) / 'result.json'

    def tearDown(self):
        self.folder.cleanup()

    def test_new_record_starts_running(self):
        record = self.tool.new_record('robot_1', 2.0)

        self.assertEqual(record['status'], 'running')
        self.assertEqual(record['box_mass_kg'], 2.0)
        self.assertEqual((record['failures'], record['checks'], record['last_check']), (0, [], None))

    def test_checks_are_saved_each_time(self):
        record = self.tool.new_record('robot_1', 2.0)
        saved = []

        # 저장할 때마다 파일을 다시 읽어 그 시점 내용을 확인한다.
        def save():
            write_json_atomic(self.path, record)
            saved.append(json.loads(self.path.read_text(encoding='utf-8')))

        self.tool.add_check(record, save, '들어 올림', True, '상승 0.081 m', lift_m=0.081)
        self.tool.add_check(record, save, '해제', 0, '신호 False', released=False, height_m=0.6)

        self.assertEqual(len(saved), 2)
        self.assertEqual(saved[0]['checks'][0]['values'], {'lift_m': 0.081})
        self.assertEqual(saved[1]['last_check'], '해제')
        self.assertEqual(saved[1]['failures'], 1)
        self.assertIs(saved[1]['checks'][1]['passed'], False)


if __name__ == '__main__':
    unittest.main()
