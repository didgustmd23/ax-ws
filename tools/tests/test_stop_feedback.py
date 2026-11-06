# A2 정지 피드백 도구의 기록 구간 계산을 Gazebo 없이 검사한다. 정지 성능 검증이 아니다.
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / 'src/ax_control/scripts'


class StopFeedbackWindow(unittest.TestCase):
    def setUp(self):
        sys.path.insert(0, str(SCRIPTS))
        spec = importlib.util.spec_from_file_location('check_stop_feedback', SCRIPTS / 'check_stop_feedback.py')

        try:
            self.tool = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(self.tool)
        except ImportError as error:
            self.skipTest(f'ROS 환경 필요: {error}')

        control = yaml.safe_load((ROOT / 'src/ax_control/config/common_controllers.yaml').read_text(encoding='utf-8'))
        self.drive = control['/**/diff_drive_controller']['ros__parameters']

    def test_window_covers_deceleration_and_hold(self):
        # 시나리오에 쓰는 모든 명령에서 감속 시간 + D-06 유지 시간이 기록 구간 안에 들어와야 한다.
        for linear, angular in ((0.3, 0.0), (self.drive['linear.x.max_velocity'], 0.0),
                                (0.0, self.drive['angular.z.max_velocity']), (0.3, 0.3)):
            window = self.tool.drive_settle_seconds(linear, angular, self.drive)
            stop = max(abs(linear) / abs(self.drive['linear.x.max_deceleration']),
                       abs(angular) / abs(self.drive['angular.z.max_deceleration']))

            self.assertGreaterEqual(window, stop + self.tool.STOP_HOLD_SECONDS + self.tool.SETTLE_MARGIN_SECONDS - 1e-9)
            self.assertGreaterEqual(window, self.tool.SETTLE_SIM_SECONDS)

    def test_window_without_limits_keeps_minimum(self):
        self.assertEqual(self.tool.drive_settle_seconds(0.8, 0.5, {}), self.tool.SETTLE_SIM_SECONDS)

    def test_rotation_at_limit_needs_longer_window(self):
        # 회전 감속 0.20 rad/s²에서 0.5 rad/s 정지는 2.5 s라 기존 고정 3.0 s로는 유지 구간을 담지 못했다.
        window = self.tool.drive_settle_seconds(0.0, 0.5, {'angular.z.max_deceleration': -0.2})
        self.assertAlmostEqual(window, 2.5 + self.tool.STOP_HOLD_SECONDS + self.tool.SETTLE_MARGIN_SECONDS)



# 중간 실패에도 결과가 남는지: 기록 뼈대·시나리오 추가·임시 파일 교체 저장
class StopFeedbackRecord(unittest.TestCase):
    def setUp(self):
        sys.path.insert(0, str(SCRIPTS))
        spec = importlib.util.spec_from_file_location('check_stop_feedback', SCRIPTS / 'check_stop_feedback.py')

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
        record = self.tool.new_record('robot_1', 2)

        self.assertEqual(record['status'], 'running')
        self.assertEqual((record['failures'], record['runs'], record['last_scenario']), (0, {}, None))
        self.assertEqual(record['criterion']['hold_s'], self.tool.STOP_HOLD_SECONDS)

    def test_partial_results_are_saved_each_scenario(self):
        record = self.tool.new_record('robot_1', 2)
        saved = []

        # 저장할 때마다 파일을 다시 읽어 그 시점 내용을 확인한다.
        def save():
            self.tool.write_record(self.path, record)
            saved.append(json.loads(self.path.read_text(encoding='utf-8')))

        run = {'odom_stop_s': 1.0, 'truth_stop_s': 1.1, 'slide_after_odom_stop_m': 0.0}
        self.tool.add_result(record, save, '직진', '직진 #1', run)
        self.tool.add_result(record, save, '직진', '직진 #2', None)

        self.assertEqual(len(saved), 2)
        self.assertEqual(saved[0]['last_scenario'], '직진 #1')
        self.assertEqual(saved[1]['failures'], 1)
        self.assertEqual(saved[1]['summary']['직진']['runs'], 2)
        self.assertEqual(saved[1]['summary']['직진']['stopped'], 1)
        self.assertFalse(self.path.with_name('result.json.tmp').exists())

    def test_failure_state_is_written(self):
        record = self.tool.new_record('robot_1', 1)
        record['status'] = 'failed'
        record['error'] = 'RuntimeError: /ax/robot_1/robot_description 수신 실패'

        self.tool.write_record(self.path, record, report=False)
        written = json.loads(self.path.read_text(encoding='utf-8'))

        self.assertEqual(written['status'], 'failed')
        self.assertIn('robot_description', written['error'])
        self.assertEqual(written['summary'], {})


if __name__ == '__main__':
    unittest.main()
