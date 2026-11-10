# A2 정지 피드백 도구의 기록 구간 계산을 Gazebo 없이 검사한다. 정지 성능 검증이 아니다.
import contextlib
import importlib.util
import io
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



# 모듈을 불러온다. ROS 환경이 없으면 시험을 건너뛴다.
def load_tool(case):
    sys.path.insert(0, str(SCRIPTS))
    spec = importlib.util.spec_from_file_location('check_stop_feedback', SCRIPTS / 'check_stop_feedback.py')

    try:
        tool = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(tool)
    except ImportError as error:
        case.skipTest(f'ROS 환경 필요: {error}')

    return tool


# 합성 표본 한 개를 만든다. 팔 값이 None이면 팔 없는 로봇 표본이다.
def sample(t, odom_v, truth_v, x=0.0, arm=None, truth_arm=None, cargo=None):
    return {'t': t, 'xy': (x, 0.0), 'odom_v': odom_v, 'odom_w': 0.0, 'truth_v': truth_v, 'truth_w': 0.0,
            'arm': arm, 'wheel': 0.0, 'truth_arm': truth_arm, 'tilt': 0.0, 'height_change': 0.0, 'cargo': cargo}


# 명령 종료 cut 뒤 odom·실제 속도가 각각 odom_stop·truth_stop 시각에 0이 되는 표본열(0.02 s 간격)
def decel_series(cut=10.0, window=4.0, odom_stop=1.0, truth_stop=1.0, speed=0.3, arm=None, start=9.0):
    samples, x, t = [], 0.0, start

    while t <= cut + window + 1e-9:
        elapsed = t - cut
        odom = speed if elapsed < 0 else max(0.0, speed * (1.0 - elapsed / odom_stop))
        truth = speed if elapsed < 0 else max(0.0, speed * (1.0 - elapsed / truth_stop))
        x += truth * 0.02
        samples.append(sample(round(t, 6), odom, truth, x, arm, arm))
        t += 0.02

    return samples


# 로봇별 관측 항목 분리: 팔 유무는 URDF 관절로 정하고, 팔 없는 로봇은 팔 입력을 요구하지 않는다
class StopFeedbackObservations(unittest.TestCase):
    def setUp(self):
        self.tool = load_tool(self)

    def test_arm_presence_from_joint_names(self):
        wheels = self.tool.WHEEL_JOINTS

        self.assertTrue(self.tool.arm_presence(wheels + self.tool.ARM_JOINTS, 'robot_1'))
        self.assertFalse(self.tool.arm_presence(wheels + ['shelf_joint'], 'robot_2'))

    def test_partial_arm_or_missing_wheels_fail(self):
        with self.assertRaisesRegex(RuntimeError, '일부'):
            self.tool.arm_presence(self.tool.WHEEL_JOINTS + self.tool.ARM_JOINTS[:2], 'robot_x')

        with self.assertRaisesRegex(RuntimeError, '바퀴'):
            self.tool.arm_presence(self.tool.ARM_JOINTS, 'robot_x')

    def test_real_models_split(self):
        # 실제 모델(Xacro)에서 robot_1은 팔 있음, robot_2는 팔 없음·cargo_link 있음
        import xacro

        joints = {}

        for name in ('ax.urdf.xacro', 'ax_2.urdf.xacro'):
            urdf = xacro.process_file(str(ROOT / 'src/ax_description/urdf' / name)).toxml()
            joints[name] = self.tool.read_urdf_joints(urdf)

        self.assertTrue(self.tool.arm_presence(joints['ax.urdf.xacro'], 'robot_1'))
        self.assertFalse(self.tool.arm_presence(joints['ax_2.urdf.xacro'], 'robot_2'))

        frame = self.tool.link_frame(joints['ax_2.urdf.xacro'], self.tool.CARGO_FRAME)
        self.assertEqual(frame.shape, (4, 4))
        self.assertGreater(frame[2, 3], 0.0)

    def test_required_inputs(self):
        now = 5.0
        drive_only = {'odom': now, 'truth': now, 'wheels': now}

        self.assertTrue(self.tool.inputs_fresh(drive_only, now, has_arm=False))
        self.assertFalse(self.tool.inputs_fresh(drive_only, now, has_arm=True))
        self.assertTrue(self.tool.inputs_fresh(dict(drive_only, arm=now, truth_arm=now), now, has_arm=True))

        # 오래된 odom은 표본을 만들지 않는다.
        stale = dict(drive_only, odom=now - self.tool.INPUT_STALE_SECONDS - 0.01)
        self.assertFalse(self.tool.inputs_fresh(stale, now, has_arm=False))

    def test_observation_record(self):
        record = self.tool.new_record('robot_2', 3)
        self.tool.set_observations(record, has_arm=False)

        self.assertEqual(record['observations']['arm'], 'not_applicable')
        self.assertEqual(record['arm_scenarios']['status'], 'not_applicable')
        self.assertFalse(record['criterion']['arm_applies'])


# 정지 판정: 팔 없는 로봇 판정, 기준 초과·유지 부족·표본 부족, odom과 실제 차이, 입력 단절
class StopFeedbackAssessment(unittest.TestCase):
    def setUp(self):
        self.tool = load_tool(self)

    def test_drive_stop_without_arm(self):
        samples = decel_series(odom_stop=1.0, truth_stop=1.1)
        result, failure = self.tool.evaluate('직진', samples, 10.0, 4.0, has_arm=False)

        self.assertIsNone(failure)
        self.assertAlmostEqual(result['odom_stop_s'], 1.0, delta=0.05)
        self.assertAlmostEqual(result['truth_stop_s'], 1.1, delta=0.05)
        self.assertAlmostEqual(result['stop_time_difference_s'], result['truth_stop_s'] - result['odom_stop_s'])
        self.assertAlmostEqual(result['odom_hold_confirmed_s'], result['odom_stop_s'] + self.tool.STOP_HOLD_SECONDS)
        self.assertEqual(result['arm_stop'], 'not_applicable')
        self.assertGreater(result['slide_after_odom_stop_m'], 0.0)

    def test_arm_value_still_required_for_arm_robot(self):
        # 팔 로봇은 팔이 계속 움직이면 바퀴가 멈춰도 정지가 아니다.
        samples = decel_series(arm=0.05)
        result, failure = self.tool.evaluate('직진', samples, 10.0, 4.0, has_arm=True)

        self.assertIsNone(result)
        self.assertIn('odom·실제', failure['reason'])

    def test_above_threshold(self):
        samples = [sample(9.0 + i * 0.02, 0.0 if i >= 50 else 0.3, 0.02, i * 0.0004) for i in range(250)]
        result, failure = self.tool.evaluate('직진', samples, 10.0, 3.9)

        self.assertIsNone(result)
        self.assertIn('실제 기준 정지 미확인', failure['reason'])
        self.assertIsNotNone(failure['odom_stop_s'])
        self.assertIsNone(failure['truth_stop_s'])

    def test_hold_not_met(self):
        # 기록 끝 0.3 s 전에야 기준 안으로 들어오면 0.5 s 유지를 확인할 수 없다.
        samples = decel_series(window=4.0, odom_stop=3.7, truth_stop=3.7)
        result, failure = self.tool.evaluate('직진', samples, 10.0, 4.0)

        self.assertIsNone(result)
        self.assertIn('정지 미확인', failure['reason'])

    def test_too_few_samples(self):
        samples = [sample(10.0 + i * 0.02, 0.0, 0.0) for i in range(5)]
        result, failure = self.tool.evaluate('직진', samples, 10.0)

        self.assertIsNone(result)
        self.assertIn('표본 부족', failure['reason'])

    def test_odom_stops_but_body_keeps_moving(self):
        # odom은 0.5 s에 정지, 실제 차체는 2.5 s까지 움직임: 차이와 odom 정지 뒤 실제 이동이 남는다.
        samples = decel_series(window=4.0, odom_stop=0.5, truth_stop=2.5)
        result, _ = self.tool.evaluate('직진', samples, 10.0, 4.0)

        self.assertGreater(result['stop_time_difference_s'], 1.5)
        self.assertGreater(result['slide_after_odom_stop_m'], 0.1)

        # 실제 차체가 끝까지 멈추지 않으면 실패다.
        moving = [dict(item, truth_v=0.05) if item['t'] >= 10.0 else item for item in samples]
        result, failure = self.tool.evaluate('직진', moving, 10.0, 4.0)

        self.assertIsNone(result)
        self.assertIn('실제 기준', failure['reason'])

    def test_input_dropout(self):
        samples = decel_series(window=4.0)
        cut_out = [item for item in samples if not 11.0 < item['t'] < 11.6]
        result, failure = self.tool.evaluate('직진', cut_out, 10.0, 4.0)

        self.assertIsNone(result)
        self.assertIn('입력 단절', failure['reason'])

    def test_record_window_short(self):
        # 입력이 기록 도중 끝나면(중단) 정지처럼 보여도 실패로 남긴다.
        samples = [item for item in decel_series(window=4.0) if item['t'] <= 12.0]
        result, failure = self.tool.evaluate('직진', samples, 10.0, 4.0)

        self.assertIsNone(result)
        self.assertIn('기록 구간 부족', failure['reason'])


# 적재물 위치·시험 보호 조건 계산
class StopFeedbackGuards(unittest.TestCase):
    def setUp(self):
        self.tool = load_tool(self)

    def test_cargo_offset(self):
        import numpy as np

        robot = np.eye(4)
        robot[:3, 3] = [1.0, 2.0, 0.0]
        cargo = np.eye(4)
        cargo[2, 3] = 0.5
        box = np.eye(4)
        box[:3, 3] = [1.01, 2.0, 0.6]

        drift, height = self.tool.cargo_offset(robot, cargo, box, 0.2)

        self.assertAlmostEqual(drift, 0.01)
        self.assertAlmostEqual(height, 0.0)

    def test_guard_violation(self):
        tool = self.tool

        self.assertIsNone(tool.guard_violation(0.01, 0.001, (0.005, 0.002), 0.15))
        self.assertIn('기울기', tool.guard_violation(0.2, 0.0, None, 0.15))
        self.assertIn('높이 변화', tool.guard_violation(0.0, 0.03, None, 0.15))
        self.assertIn('적재물 이탈', tool.guard_violation(0.0, 0.0, (0.03, 0.0), 0.15))
        self.assertIn('적재물 이탈', tool.guard_violation(0.0, 0.0, (0.0, 0.02), 0.15))

    def test_payload_conditions_follow_a1(self):
        masses, size = self.tool.a1_payload_conditions()

        self.assertEqual(masses, [0.5, 1.0, 2.0])
        self.assertEqual(size, [0.2, 0.2, 0.2])


# robot_1 기존 결과 형식·시나리오 유지와 실패 사유 보존
class StopFeedbackCompatibility(unittest.TestCase):
    def setUp(self):
        self.tool = load_tool(self)
        self.folder = tempfile.TemporaryDirectory()
        self.path = Path(self.folder.name) / 'result.json'

    def tearDown(self):
        self.folder.cleanup()

    def test_record_keeps_existing_keys(self):
        record = self.tool.new_record('robot_1', 3)
        existing = {'robot_id', 'repeats', 'status', 'error', 'last_scenario', 'failures',
                    'criterion', 'feedback', 'runs', 'summary', 'note'}

        self.assertLessEqual(existing, set(record))
        self.assertLessEqual({'linear_m_s', 'angular_rad_s', 'arm_rad_s', 'hold_s'}, set(record['criterion']))

    def test_scenarios_unchanged(self):
        names = [item[0] for item in self.tool.drive_scenarios(0.67, 0.5)]

        self.assertEqual(names, ['직진 0.3 m/s', '직진 상한 0.67 m/s', '제자리 회전 상한 0.5 rad/s', '호 주행'])
        self.assertEqual([item[0] for item in self.tool.ARM_TARGETS], ['팔 이동', '팔 복귀'])

    def test_failure_reason_preserved_on_interrupt(self):
        record = self.tool.new_record('robot_2', 1)

        # 저장 함수
        def save():
            self.tool.write_record(self.path, record)

        self.tool.add_result(record, save, '직진', '직진 #1', None, {'reason': '입력 단절(표본 최대 간격 0.6 s > 0.25 s)'})
        record['status'] = 'interrupted'
        record['error'] = '사용자 중단(Ctrl+C)'
        self.tool.write_record(self.path, record, report=False)

        written = json.loads(self.path.read_text(encoding='utf-8'))

        self.assertEqual(written['status'], 'interrupted')
        self.assertEqual(written['runs']['직진'], [None])
        self.assertEqual(written['failure_details'][0]['scenario'], '직진 #1')
        self.assertIn('입력 단절', written['failure_details'][0]['reason'])
        self.assertEqual(written['summary']['직진']['stopped'], 0)



# 시뮬레이션 시계·발행기를 흉내 내는 녹화기. 실제 노드의 정지 방식 메서드를 그대로 빌려 쓴다.
def fake_recorder(tool):
    recorder_class = tool.StopFeedbackRecorder

    # 발행 기록기
    class Publisher:
        def __init__(self, owner):
            self.owner = owner
            self.sent = []

        def publish(self, message):
            self.sent.append((self.owner.t, message))

    class Fake:
        run_drive = recorder_class.run_drive
        drive_and_stop = recorder_class.drive_and_stop
        settle_after = recorder_class.settle_after
        spin_until_sim = recorder_class.spin_until_sim
        begin_zero_hold = recorder_class.begin_zero_hold
        maintain_zero = recorder_class.maintain_zero
        end_zero_hold = recorder_class.end_zero_hold
        publish_drive = recorder_class.publish_drive
        stop_drive = recorder_class.stop_drive

        def __init__(self):
            self.t = 100.0
            self.guard_error = None
            self.zero_hold = None
            self.drive_publisher = Publisher(self)

        def now_seconds(self):
            return self.t

        def spin_step(self):
            self.t += 0.005

        def spin_for(self, seconds):
            self.t += seconds

        def check_guard(self):
            pass

        def make_twist(self, linear, angular):
            return {'linear': linear, 'angular': angular, 'stamp': self.t}

        def stamp_seconds(self, message):
            return message['stamp']

    return Fake()


# 정지 방식 분리: command_loss는 측정 구간에 0을 보내지 않고, zero_command는 최초 0 발행 시각을 기준으로 0을 유지한다
class StopFeedbackStopModes(unittest.TestCase):
    def setUp(self):
        self.tool = load_tool(self)

    def test_parser_default_and_invalid(self):
        parser = self.tool.build_parser()

        self.assertEqual(parser.parse_args([]).stop_mode, 'command_loss')
        self.assertEqual(parser.parse_args(['--stop-mode', 'zero_command']).stop_mode, 'zero_command')

        with self.assertRaises(SystemExit), contextlib.redirect_stderr(io.StringIO()):
            parser.parse_args(['--stop-mode', 'brake'])

    def test_command_loss_sends_no_zero_while_measuring(self):
        node = fake_recorder(self.tool)
        reference, command = node.drive_and_stop(0.3, 0.0, 1.0, 'command_loss')
        sent_before = len(node.drive_publisher.sent)
        node.settle_after(reference, 3.0)

        self.assertEqual(len(node.drive_publisher.sent), sent_before)
        self.assertTrue(all(message['linear'] == 0.3 for _, message in node.drive_publisher.sent))
        self.assertEqual(command['stop_mode'], 'command_loss')
        self.assertAlmostEqual(reference, command['command_stopped_sim_s'])
        self.assertLessEqual(command['last_nonzero_published_sim_s'], reference)
        self.assertIsNone(command['first_zero_published_sim_s'])

    def test_zero_command_reference_and_hold(self):
        node = fake_recorder(self.tool)
        reference, command = node.drive_and_stop(0.3, 0.2, 1.0, 'zero_command')
        node.settle_after(reference, 3.0)
        command.update(node.end_zero_hold())

        zeros = [(t, m) for t, m in node.drive_publisher.sent if m['linear'] == 0.0 and m['angular'] == 0.0]

        # 기준 시각은 최초 0 발행 시각이고 이후 반복 발행으로 바뀌지 않는다.
        self.assertAlmostEqual(reference, zeros[0][0])
        self.assertAlmostEqual(command['first_zero_published_sim_s'], reference)
        self.assertGreater(command['zero_commands_published'], 10)
        self.assertGreaterEqual(zeros[-1][0], reference + 3.0 - self.tool.ZERO_COMMAND_PERIOD_SECONDS - 1e-6)
        self.assertLessEqual(command['zero_publish_max_gap_s'], self.tool.ZERO_COMMAND_PERIOD_SECONDS + 0.01)

        # 0 메시지는 발행 시각의 유효한 stamp를 가진다.
        self.assertTrue(all(abs(m['stamp'] - t) < 1e-9 and m['stamp'] > 0 for t, m in zeros))

        # 0 이후에는 비영 명령이 없다.
        self.assertTrue(all(m['linear'] == 0.0 for t, m in node.drive_publisher.sent if t >= reference))

    def test_zero_hold_keeps_first_reference(self):
        node = fake_recorder(self.tool)
        first = node.begin_zero_hold()
        node.t += 1.0
        second = node.begin_zero_hold()

        self.assertEqual(first, second)
        self.assertEqual(node.zero_hold['first'], first)

    def test_incomplete_drive_is_failure(self):
        node = fake_recorder(self.tool)

        # 시뮬레이션 시계가 멈추면 명령 구간을 채우지 못한다.
        node.spin_for = lambda seconds: None
        original = self.tool.SIM_WAIT_WALL_LIMIT_SECONDS
        self.tool.SIM_WAIT_WALL_LIMIT_SECONDS = 0.2

        try:
            with self.assertRaisesRegex(RuntimeError, '주행 명령 구간'):
                node.drive_and_stop(0.3, 0.0, 1.0, 'zero_command')
        finally:
            self.tool.SIM_WAIT_WALL_LIMIT_SECONDS = original

    def test_window_by_mode(self):
        drive = {'linear.x.max_deceleration': -0.67, 'angular.z.max_deceleration': -0.2}
        zero = self.tool.drive_settle_seconds(0.0, 0.5, drive)
        loss = self.tool.drive_settle_seconds(0.0, 0.5, drive, command_timeout=0.5)

        self.assertAlmostEqual(loss - zero, 0.5)
        self.assertAlmostEqual(self.tool.mode_settle_seconds('zero_command', 0.0, 0.5, drive, 0.5), zero)
        self.assertAlmostEqual(self.tool.mode_settle_seconds('command_loss', 0.0, 0.5, drive, 0.5), loss)

    def test_modes_not_mixed_in_summary(self):
        run = {'odom_stop_s': 1.0, 'truth_stop_s': 1.0, 'slide_after_odom_stop_m': 0.0}
        summary = self.tool.summarize_results({'직진': [dict(run, stop_mode='zero_command')]}, report=False)

        self.assertEqual(summary['직진']['stop_mode'], 'zero_command')

        with self.assertRaisesRegex(ValueError, 'stop_mode'):
            self.tool.summarize_results({'직진': [dict(run, stop_mode='zero_command'),
                                                 dict(run, stop_mode='command_loss')]}, report=False)

    def test_record_identifies_mode(self):
        record = self.tool.new_record('robot_2', 3, stop_mode='zero_command')

        self.assertEqual(record['stop_mode'], 'zero_command')
        self.assertIn('0 속도', record['conditions']['stop_reference'])
        self.assertEqual(self.tool.new_record('robot_1', 1)['stop_mode'], 'command_loss')

    def test_deceleration_onset(self):
        samples = decel_series(odom_stop=1.0, truth_stop=1.0)

        # 명령 종료 직후 바로 줄기 시작하는 표본에서는 감속 시작이 0 근처다.
        self.assertLess(self.tool.deceleration_onset(samples, 10.0, 'odom_v'), 0.1)

        # timeout 0.5 s 동안 명령 속도를 유지하는 표본은 0.5 s 뒤에 감속을 시작한다.
        delayed = [dict(item, odom_v=0.3) if 10.0 <= item['t'] < 10.5 else item for item in samples]
        self.assertAlmostEqual(self.tool.deceleration_onset(delayed, 10.0, 'odom_v'), 0.5, delta=0.03)

        self.assertIsNone(self.tool.deceleration_onset(samples, 10.0, 'odom_w'))

    def test_no_samples_during_command_fails(self):
        samples = [item for item in decel_series(window=4.0) if item['t'] >= 10.0]
        result, failure = self.tool.evaluate('직진', samples, 10.0, 4.0)

        self.assertIsNone(result)
        self.assertIn('명령 구간 관측 없음', failure['reason'])


if __name__ == '__main__':
    unittest.main()
