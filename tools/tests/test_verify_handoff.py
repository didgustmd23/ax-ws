"""Gazebo 없이 인계 검증기(verify_handoff.py)의 파싱·판정 로직을 시험한다."""
import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

import yaml


ROOT = Path(__file__).resolve().parents[2]


# 경로의 Python 파일을 지정한 이름의 모듈로 불러온다.
def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


EVIDENCE = ROOT / 'docs/evidence'
_BEFORE_IMPORT = sorted(p.name for p in EVIDENCE.iterdir())
_ENV_BEFORE = {k: os.environ.get(k) for k in ('ROS_DOMAIN_ID', 'GZ_PARTITION')}
vh = load('verify_handoff', ROOT / 'tools/verify_handoff.py')
contracts = load('ax_model_contracts', ROOT / 'src/ax_control/ax_control/model_contracts.py')

GREEN, CYAN, BLUE, END = '\033[92m', '\033[96m', '\033[94m', '\033[0m'


# ros2controlcli list_controllers 출력 행을 흉내 낸다(이름·상태에 색상 코드).
def controllers_text(states, color=True):
    rows = []
    for name, state in states.items():
        c = (GREEN if state == 'active' else CYAN) if color else ''
        e = END if color else ''
        rows.append(f'{c}{name:<24}{e} some_pkg/SomeType  {c}{state:<8}{e}')
    return '\n'.join(rows) + '\n'


# list_hardware_interfaces 출력을 흉내 낸다. command는 이름 → (사용 가능, 점유)다.
def interfaces_text(command, color=True):
    lines = ['command interfaces']
    for name, (available, claimed) in command.items():
        c = BLUE if color else ''
        e = END if color else ''
        lines.append(f'\t{c}{name} [{"available" if available else "unavailable"}]'
                     f' [{"claimed" if claimed else "unclaimed"}]{e}')
    lines += ['state interfaces', '\tshoulder_joint/position', '\tshoulder_joint/velocity']
    return '\n'.join(lines) + '\n'


ALL_ACTIVE = {name: 'active' for name in vh.REQUIRED_CONTROLLERS}
ALL_CLAIMED = {name: (True, True) for name in vh.EXPECTED_COMMAND_INTERFACES}


# 흉내 낸 출력으로 제어기·인터페이스 판정 결과 쌍을 만든다.
def control_state(states=ALL_ACTIVE, command=ALL_CLAIMED, rc_c=0, rc_h=0):
    return (vh.evaluate_controllers(rc_c, vh.parse_controllers(controllers_text(states))),
            vh.evaluate_interfaces(rc_h, vh.parse_hardware_interfaces(interfaces_text(command))))


# 모든 조건을 통과하는 결과 사전을 만든다.
def passing_result():
    controllers, interfaces = control_state()
    motion = {j: {'status': 'motion_observed', 'target': 0.3, 'measured_position': 0.29,
                  'absolute_target_error': 0.01, 'review_tolerance': 0.1, 'unit': 'rad',
                  'within_review_tolerance': True}
              for j in vh.ARM_JOINTS + vh.FINGER_JOINTS}
    result = {'checks': {'readiness': {'ready': True}, 'xacro': {'returncode': 0},
                         'check_urdf': {'returncode': 0}, 'controllers': controllers,
                         'hardware_interfaces': interfaces, 'scan': {'received': 10},
                         'motion': motion},
              'launch_alive_at_end': True, 'cleanup': {'result': 'STOPPED'}}
    result['checks']['runtime_joint_limits'] = {'matches_checked_urdf': True}
    return result


# import만으로 부작용이 없는지 시험한다.
class ImportTest(unittest.TestCase):
    # import 후 증거 폴더와 ROS·Gazebo 환경 변수가 그대로인지 확인한다.
    def test_import_has_no_side_effects(self):
        self.assertEqual(sorted(p.name for p in EVIDENCE.iterdir()), _BEFORE_IMPORT)
        self.assertEqual({k: os.environ.get(k) for k in _ENV_BEFORE}, _ENV_BEFORE)


# 명령행 인자와 launch 명령 반영을 시험한다.
class ArgumentTest(unittest.TestCase):
    BASE = ['ros2', 'launch', 'ax_control', 'simulation.launch.py']

    # 인자가 없으면 launch 기본값과 기존 기본값이 유지되는지 확인한다.
    def test_default_keeps_launch_defaults(self):
        args = vh.parse_arguments([])
        self.assertFalse(args.headless)
        self.assertEqual(vh.launch_command(args.headless), self.BASE)
        self.assertEqual(args.domain_id, 87)
        self.assertEqual(args.ready_timeout, vh.READY_TIMEOUT_S)

    # --headless가 gui:=false 하나만 추가하는지 확인한다.
    def test_headless_adds_exactly_gui_false(self):
        args = vh.parse_arguments(['--headless'])
        self.assertEqual(vh.launch_command(args.headless), self.BASE + ['gui:=false'])

    # domain·준비 제한시간 인자의 해석과 잘못된 값 거부를 확인한다.
    def test_domain_and_ready_timeout(self):
        self.assertIsNone(vh.parse_arguments(['--domain-id', 'auto']).domain_id)
        self.assertEqual(vh.parse_arguments(['--domain-id', '61']).domain_id, 61)
        self.assertEqual(vh.parse_arguments(['--ready-timeout', '15']).ready_timeout, 15.0)
        for bad in (['--domain-id', '233'], ['--domain-id', 'x'], ['--ready-timeout', '0']):
            with mock.patch('sys.stderr'), self.assertRaises(SystemExit):
                vh.parse_arguments(bad)


# 검증기 예상값과 모델 계약·제어기 설정의 일치를 시험한다.
class ContractTest(unittest.TestCase):
    # 필수 제어기·관절 목록이 model_contracts.py와 ax1_controllers.yaml과 같은지 확인한다.
    def test_expectations_match_model_contracts_and_controller_config(self):
        self.assertEqual(vh.REQUIRED_CONTROLLERS, tuple(contracts.controllers('robot_1')))
        self.assertEqual(vh.ARM_JOINTS, contracts.ARM_JOINTS)
        self.assertEqual(vh.FINGER_JOINTS, contracts.FINGER_JOINTS)
        config = yaml.safe_load(
            (ROOT / 'src/ax_control/config/ax1_controllers.yaml').read_text(encoding='utf-8'))
        drive = config['/**/diff_drive_controller']['ros__parameters']
        self.assertEqual(set(vh.WHEEL_JOINTS),
                         set(drive['left_wheel_names'] + drive['right_wheel_names']))
        self.assertEqual(tuple(config['/**/arm_controller']['ros__parameters']['joints']),
                         vh.ARM_JOINTS)
        self.assertEqual(tuple(config['/**/finger_controller']['ros__parameters']['joints']),
                         vh.FINGER_JOINTS)
        self.assertEqual(len(vh.EXPECTED_COMMAND_INTERFACES), 11)


# 제어기 목록 파싱과 활성 판정을 시험한다.
class ControllerParsingTest(unittest.TestCase):
    # 색상 코드가 있는 active 행을 정확히 파싱하는지 확인한다.
    def test_colored_active_rows_are_parsed_exactly(self):
        controllers, _ = control_state()
        self.assertTrue(controllers['ok'], controllers['problems'])
        self.assertEqual(set(controllers['states'].values()), {'active'})

    # inactive를 활성으로 분류하지 않는지 확인한다.
    def test_inactive_is_not_active(self):
        for color in (True, False):
            parsed = vh.parse_controllers(controllers_text(
                dict(ALL_ACTIVE, arm_controller='inactive'), color))
            self.assertEqual(parsed['controllers']['arm_controller']['state'], 'inactive')
            check = vh.evaluate_controllers(0, parsed)
            self.assertFalse(check['ok'])
            self.assertIn('controller arm_controller is inactive', check['problems'])

    # 필수 제어기가 목록에 없으면 실패하는지 확인한다.
    def test_missing_controller_fails(self):
        states = dict(ALL_ACTIVE)
        del states['finger_controller']
        controllers, _ = control_state(states=states)
        self.assertFalse(controllers['ok'])
        self.assertIn('controller finger_controller not listed', controllers['problems'])

    # 조회 실패와 파싱 불가 출력이 실패로 처리되는지 확인한다.
    def test_query_failure_and_unparsable_output_fail(self):
        check = vh.evaluate_controllers(124, vh.parse_controllers(''))
        self.assertFalse(check['ok'])
        self.assertIn('list_controllers exited with 124', check['problems'])
        check = vh.evaluate_controllers(
            0, vh.parse_controllers('Waiting for service /controller_manager ...\n'))
        self.assertFalse(check['ok'])
        self.assertTrue(any('unparsed line' in p for p in check['problems']))


# 명령 인터페이스 파싱과 판정을 시험한다.
class InterfaceParsingTest(unittest.TestCase):
    # 예상 11개가 모두 점유되면 통과하는지 확인한다.
    def test_expected_claimed_set_passes(self):
        _, interfaces = control_state()
        self.assertTrue(interfaces['ok'], interfaces['problems'])
        self.assertEqual(interfaces['claimed'], sorted(vh.EXPECTED_COMMAND_INTERFACES))

    # 상태 인터페이스 절을 명령 인터페이스로 세지 않는지 확인한다.
    def test_state_section_is_not_counted_as_command(self):
        parsed = vh.parse_hardware_interfaces(interfaces_text(ALL_CLAIMED))
        self.assertNotIn('shoulder_joint/velocity', parsed['command'])

    # 누락·예상 밖·미점유·사용 불가 인터페이스가 각각 실패로 분류되는지 확인한다.
    def test_missing_unexpected_unclaimed_and_unavailable_fail(self):
        command = dict(ALL_CLAIMED)
        del command['wrist_joint/position']
        command['shoulder_joint/velocity'] = (True, False)
        command['elbow_joint/position'] = (True, False)
        command['left_finger_joint/position'] = (False, False)
        _, interfaces = control_state(command=command)
        self.assertFalse(interfaces['ok'])
        self.assertEqual(interfaces['missing'], ['wrist_joint/position'])
        self.assertEqual(interfaces['unexpected'], ['shoulder_joint/velocity'])
        self.assertEqual(interfaces['unclaimed'],
                         ['elbow_joint/position', 'left_finger_joint/position'])
        self.assertEqual(interfaces['unavailable'], ['left_finger_joint/position'])

    REFERENCES = {'diff_drive_controller/linear/velocity': (True, False),
                  'diff_drive_controller/angular/velocity': (True, False)}

    # 알려진 참조 인터페이스 2개만 하드웨어와 분리되는지 확인한다.
    def test_known_reference_interfaces_are_kept_apart(self):
        # Jazzy diff_drive_controller는 체인 가능하며 이 두 참조 인터페이스를 함께 내보낸다.
        _, interfaces = control_state(command=dict(ALL_CLAIMED, **self.REFERENCES))
        self.assertTrue(interfaces['ok'], interfaces['problems'])
        self.assertEqual(interfaces['controller_references'], sorted(self.REFERENCES))
        self.assertEqual(interfaces['claimed'], sorted(vh.EXPECTED_COMMAND_INTERFACES))

    # 참조 인터페이스가 점유되면 실패하는지 확인한다.
    def test_claimed_reference_interface_fails(self):
        for name in self.REFERENCES:
            command = dict(ALL_CLAIMED, **self.REFERENCES)
            command[name] = (True, True)
            _, interfaces = control_state(command=command)
            self.assertFalse(interfaces['ok'])
            self.assertIn(f'controller reference interface {name} claimed'
                          ' (no chain configured)', interfaces['problems'])

    # 제어기 이름 접두사가 같아도 알려지지 않은 인터페이스는 실패하는지 확인한다.
    def test_unknown_interface_with_controller_prefix_fails(self):
        for name in ('diff_drive_controller/linear/position', 'diff_drive_controller/x',
                     'arm_controller/shoulder_joint/position', 'base/velocity'):
            _, interfaces = control_state(command=dict(ALL_CLAIMED, **self.REFERENCES,
                                                       **{name: (True, False)}))
            self.assertFalse(interfaces['ok'], name)
            self.assertEqual(interfaces['unexpected'], [name])
            self.assertEqual(interfaces['controller_references'], sorted(self.REFERENCES))

    # 하드웨어 인터페이스 조회 실패가 실패로 처리되는지 확인한다.
    def test_query_failure_fails(self):
        check = vh.evaluate_interfaces(1, vh.parse_hardware_interfaces(''))
        self.assertFalse(check['ok'])
        self.assertIn('list_hardware_interfaces exited with 1', check['problems'])
        self.assertTrue(any('no "command interfaces" section' in p for p in check['problems']))


# 최종 종료 코드와 실패 사유 판정을 시험한다.
class FinalVerdictTest(unittest.TestCase):
    # 모든 조건을 만족하면 종료 코드 0인지 확인한다.
    def test_complete_result_passes(self):
        self.assertEqual(vh.final_verdict(passing_result()), (0, []))

    # 목표 오차 초과는 종료 코드 2이며 관절·목표·측정·오차·허용 오차가 사유에 있는지 확인한다.
    def test_tolerance_miss_is_exit_2_with_reasons(self):
        result = passing_result()
        result['checks']['motion']['shoulder_joint'].update(
            target=-0.4, measured_position=-1.5708, absolute_target_error=1.1708,
            within_review_tolerance=False)
        code, failures = vh.final_verdict(result)
        self.assertEqual(code, 2)
        self.assertEqual(len(failures), 1)
        for text in ('shoulder_joint', 'target=-0.4', 'measured=-1.5708', 'error=1.1708',
                     'tolerance=0.1', 'rad'):
            self.assertIn(text, failures[0])

    # 미완료 조건이 있으면 오차 초과가 있어도 종료 코드 1인지 확인한다.
    def test_incomplete_result_stays_exit_1_even_with_tolerance_miss(self):
        result = passing_result()
        result['checks']['motion']['wrist_joint']['within_review_tolerance'] = False
        result['launch_alive_at_end'] = False
        self.assertEqual(vh.final_verdict(result)[0], 1)

    # 통과 결과를 change로 바꿨을 때 종료 코드 1과 기대 사유가 나오는지 확인한다.
    def check_fails(self, change, expected):
        result = passing_result()
        change(result['checks'])
        code, failures = vh.final_verdict(result)
        self.assertEqual(code, 1)
        self.assertTrue(any(expected in f for f in failures), failures)

    # 제어기·인터페이스 조회 실패가 최종 실패로 이어지는지 확인한다.
    def test_query_failure_fails(self):
        # 제어기 조회를 종료 코드 124 실패로 바꾼다.
        def change(checks):
            checks['controllers'] = vh.evaluate_controllers(124, vh.parse_controllers(''))
        self.check_fails(change, 'list_controllers exited with 124')

        # 하드웨어 인터페이스 조회를 종료 코드 1 실패로 바꾼다.
        def change(checks):
            checks['hardware_interfaces'] = vh.evaluate_interfaces(
                1, vh.parse_hardware_interfaces(''))
        self.check_fails(change, 'list_hardware_interfaces exited with 1')

    # 필수 제어기 누락이 최종 실패로 이어지는지 확인한다.
    def test_missing_controller_fails(self):
        states = dict(ALL_ACTIVE)
        del states['diff_drive_controller']

        # 제어기 판정을 diff_drive_controller가 없는 결과로 바꾼다.
        def change(checks):
            checks['controllers'] = control_state(states=states)[0]
        self.check_fails(change, 'diff_drive_controller not listed')

    # 인터페이스 불일치가 최종 실패로 이어지는지 확인한다.
    def test_interface_mismatch_fails(self):
        command = dict(ALL_CLAIMED, **{'gripper_rotation_joint/position': (True, False)})

        # 인터페이스 판정을 gripper_rotation_joint가 미점유인 결과로 바꾼다.
        def change(checks):
            checks['hardware_interfaces'] = control_state(command=command)[1]
        self.check_fails(change, 'gripper_rotation_joint/position unclaimed')

    # 정리 실패나 정리 미실행이 종료 코드 1인지 확인한다.
    def test_cleanup_failure_fails(self):
        result = passing_result()
        result['cleanup'] = {'result': 'STOP_FAILED'}
        self.assertEqual(vh.final_verdict(result), (1, ['cleanup: STOP_FAILED']))
        del result['cleanup']
        self.assertEqual(vh.final_verdict(result), (1, ['cleanup: not run']))

    # 필수 검사 결과가 빠지면 실패하는지 확인한다.
    def test_missing_checks_fail(self):
        self.check_fails(lambda checks: checks.pop('controllers'), 'controllers: not checked')
        self.check_fails(lambda checks: checks.pop('readiness'), 'readiness')


# 시험용 가짜 시계. 호출하면 현재 시각을, sleep은 시각만 앞당긴다.
class FakeClock:
    # 시각을 0으로 시작한다.
    def __init__(self):
        self.now = 0.0

    # 현재 가짜 시각을 돌려준다.
    def __call__(self):
        return self.now

    # 실제로 기다리지 않고 가짜 시각만 앞당긴다.
    def sleep(self, seconds):
        self.now += seconds


# 준비 대기와 명령 전송 차단을 시험한다.
class ReadinessTest(unittest.TestCase):
    SUBSCRIBED = {'/arm_controller/joint_trajectory': 1, '/gripper/command': 1}

    # 가짜 시계로 start_when_ready를 실행한다. 조회 i는 durations[i] s(기본 1 s)가 걸린다.
    def start(self, statuses, launch_codes=None, timeout_s=20.0, durations=None):
        clock = FakeClock()
        statuses = iter(statuses)
        durations = iter(durations or [])
        launch_codes = iter(launch_codes or [])
        on_ready = mock.Mock()
        self.remaining = []

        # 받은 남은 시간을 기록하고 시각을 앞당긴 뒤 다음 준비 상태를 돌려준다.
        def probe(remaining_s):
            self.remaining.append(remaining_s)
            clock.now += next(durations, 1.0)
            return next(statuses)
        readiness = vh.start_when_ready(
            probe, lambda: next(launch_codes, None), on_ready, timeout_s=timeout_s,
            poll_s=5.0, clock=clock, sleep=clock.sleep)
        return readiness, on_ready

    # 준비 완료 상태를 만든다.
    def ready(self):
        return vh.readiness_status(*control_state(), self.SUBSCRIBED)

    # 제한시간 직전에 준비되면 명령을 보내는지 확인한다.
    def test_ready_just_before_limit_sends(self):
        readiness, on_ready = self.start([self.ready()], timeout_s=10.0, durations=[9.9])
        self.assertTrue(readiness['ready'])
        on_ready.assert_called_once()

    # 제한시간과 같거나 넘은 시점에 준비되면 명령을 보내지 않는지 확인한다.
    def test_ready_at_or_after_limit_does_not_send(self):
        for duration in (10.0, 10.5):
            readiness, on_ready = self.start([self.ready()], timeout_s=10.0,
                                             durations=[duration])
            self.assertFalse(readiness['ready'], duration)
            self.assertIn('ready only at or after the limit', readiness['reason'])
            on_ready.assert_not_called()

    # 조회 도중 제한시간이 지나면 결과가 준비 완료여도 명령을 보내지 않는지 확인한다.
    def test_limit_passing_during_probe_does_not_send(self):
        not_ready = vh.readiness_status(*control_state(rc_c=124), self.SUBSCRIBED)
        # t=1에 미준비, t=6까지 대기, 두 번째 조회는 4 s를 받지만 t=12까지 걸린다.
        readiness, on_ready = self.start([not_ready, self.ready()], timeout_s=10.0,
                                         durations=[1.0, 6.0])
        self.assertEqual(self.remaining, [10.0, 4.0])
        self.assertFalse(readiness['ready'])
        on_ready.assert_not_called()

    # 조회 사이 대기가 남은 시간으로 줄어드는지 확인한다.
    def test_sleep_is_limited_to_remaining_time(self):
        not_ready = vh.readiness_status(*control_state(rc_c=124), self.SUBSCRIBED)
        # t=1에 미준비, 대기는 5 s 대신 2 s로 줄고 그 뒤 남은 시간이 없다.
        readiness, on_ready = self.start([not_ready] * 3, timeout_s=3.0)
        self.assertEqual(self.remaining, [3.0])
        self.assertFalse(readiness['ready'])
        self.assertAlmostEqual(readiness['elapsed_s'], 3.0)
        self.assertIn('list_controllers exited with 124', readiness['reason'])
        on_ready.assert_not_called()

    # 준비 완료면 명령을 한 번 보내는지 확인한다.
    def test_ready_sends_commands(self):
        ready = vh.readiness_status(*control_state(), self.SUBSCRIBED)
        readiness, on_ready = self.start([ready])
        self.assertTrue(readiness['ready'])
        on_ready.assert_called_once()

    # 구독자가 있어도 제어기가 비활성이면 명령을 보내지 않는지 확인한다.
    def test_subscribers_with_inactive_controller_do_not_send(self):
        status = vh.readiness_status(
            *control_state(states=dict(ALL_ACTIVE, arm_controller='inactive')),
            self.SUBSCRIBED)
        self.assertFalse(status['ready'])
        readiness, on_ready = self.start([status] * 10, timeout_s=12.0)
        self.assertFalse(readiness['ready'])
        self.assertIn('arm_controller is inactive', readiness['reason'])
        on_ready.assert_not_called()

    # 구독자가 없거나 확인한 토픽이 없으면 미준비인지 확인한다.
    def test_missing_subscriber_is_not_ready(self):
        status = vh.readiness_status(*control_state(), {'/arm_controller/joint_trajectory': 1,
                                                        '/gripper/command': 0})
        self.assertFalse(status['ready'])
        self.assertIn('no subscriber on /gripper/command', status['reasons'])
        self.assertFalse(vh.readiness_status(*control_state(), {})['ready'])

    # 준비 시간 초과 시 동작 시험을 시작하지 않고 조회 종료 코드를 남기는지 확인한다.
    def test_timeout_does_not_start_motion(self):
        status = vh.readiness_status(*control_state(rc_c=124), self.SUBSCRIBED)
        readiness, on_ready = self.start([status] * 10, timeout_s=12.0)
        self.assertFalse(readiness['ready'])
        self.assertTrue(readiness['reason'].startswith('not ready within 12 s'))
        self.assertEqual([a['controllers_returncode'] for a in readiness['attempts']],
                         [124] * len(readiness['attempts']))
        on_ready.assert_not_called()

    # launch 조기 종료 시 동작 시험을 시작하지 않는지 확인한다.
    def test_launch_exit_does_not_start_motion(self):
        not_ready = vh.readiness_status(*control_state(rc_c=1), self.SUBSCRIBED)
        ready = vh.readiness_status(*control_state(), self.SUBSCRIBED)
        # 첫 조회 전에 launch 종료가 관찰된 경우.
        readiness, on_ready = self.start([ready], launch_codes=[1])
        self.assertFalse(readiness['ready'])
        self.assertIn('launch exited with code 1', readiness['reason'])
        on_ready.assert_not_called()
        # 준비 완료를 보고한 조회 직후 launch 종료가 관찰된 경우.
        readiness, on_ready = self.start([not_ready, ready], launch_codes=[None, None, None, 2])
        self.assertFalse(readiness['ready'])
        self.assertIn('code 2', readiness['reason'])
        on_ready.assert_not_called()

    # 준비 제한시간이 launch의 관리자·전환 대기 합보다 긴지 확인한다.
    def test_ready_timeout_is_longer_than_launch_waits(self):
        self.assertGreater(vh.READY_TIMEOUT_S, 120 + 60)


# 가짜 subprocess 실행 결과를 만든다.
def completed(returncode, stdout='', stderr=''):
    return subprocess.CompletedProcess([], returncode, stdout, stderr)


# 의존성 확인 방법과 --dependencies-only 종료 코드를 시험한다.
class DependencyTest(unittest.TestCase):
    MISSING_YAML = completed(1, stderr="ModuleNotFoundError: No module named 'yaml'\n")

    # PyYAML 설치·누락과 인터프리터 미확인을 각각 올바르게 판정하는지 확인한다.
    def test_pyyaml_installed_and_missing(self):
        ok = vh.check_python_module('/usr/bin/python3', 'yaml',
                                    lambda *a, **k: completed(0, '6.0.1 /x/yaml/__init__.py\n'))
        self.assertTrue(ok['ok'])
        missing = vh.check_python_module('/usr/bin/python3', 'yaml',
                                         lambda *a, **k: self.MISSING_YAML)
        self.assertFalse(missing['ok'])
        self.assertIn('ModuleNotFoundError', missing['detail'])
        self.assertFalse(vh.check_python_module(None, 'yaml')['ok'])

    # 실제 인터프리터로 있는 모듈과 없는 모듈을 구분하는지 확인한다.
    def test_real_interpreter_import(self):
        self.assertTrue(vh.check_python_module(sys.executable, 'json')['ok'])
        self.assertFalse(vh.check_python_module(
            sys.executable, 'ax_module_that_does_not_exist')['ok'])

    # ros2 shebang(직접 경로·env 방식)에서 launch Python을 찾는지 확인한다.
    def test_launch_python_from_shebang(self):
        with tempfile.TemporaryDirectory() as folder:
            ros2 = Path(folder) / 'ros2'
            ros2.write_text('#!/usr/bin/python3\n')
            self.assertEqual(vh.launch_python(lambda n: str(ros2))[0], '/usr/bin/python3')
            ros2.write_text('#!/usr/bin/env python3\n')
            found = vh.launch_python(lambda n: str(ros2) if n == 'ros2' else '/opt/py3')
            self.assertEqual(found[0], '/opt/py3')
        self.assertIsNone(vh.launch_python(lambda n: None)[0])

    # 알 수 없거나 누락된 의존성, ament index 없음이 실패인지 확인한다.
    def test_unknown_or_missing_dependency_fails(self):
        declared = {'ax_control': ['python3-yaml', 'rclpy', 'python3-unknown']}
        result = vh.check_dependencies(declared, {'rclpy': '/opt/ros/jazzy'}.get,
                                       '/usr/bin/python3', lambda *a, **k: completed(0, 'ok'))
        self.assertTrue(result['ax_control']['python3-yaml']['ok'])
        self.assertTrue(result['ax_control']['rclpy']['ok'])
        self.assertFalse(result['ax_control']['python3-unknown']['ok'])
        self.assertFalse(vh.dependencies_ok(result))
        result = vh.check_dependencies({'p': ['rclpy']}, None, '/usr/bin/python3')
        self.assertFalse(vh.dependencies_ok(result))

    # 의존성 조회를 가짜로 바꿔 main(--dependencies-only)을 실행하고 종료 코드를 돌려준다.
    def run_dependencies_only(self, runner, lookup):
        with tempfile.TemporaryDirectory() as folder, \
                mock.patch.dict(os.environ), \
                mock.patch.object(vh, 'ament_prefix_lookup', lambda: lookup), \
                mock.patch.object(vh, 'launch_python', lambda: ('/usr/bin/python3', 'test')), \
                mock.patch.object(vh.subprocess, 'run', runner), \
                mock.patch('builtins.print'):
            return vh.main(['--dependencies-only', '--output-dir', str(Path(folder) / 'out')])

    # 정상 설치는 0, PyYAML·ROS 패키지 누락은 1인지 확인한다.
    def test_dependencies_only_exit_code(self):
        installed = lambda name: f'/opt/{name}'  # noqa: E731
        self.assertEqual(self.run_dependencies_only(
            lambda *a, **k: completed(0, 'ok'), installed), 0)
        self.assertEqual(self.run_dependencies_only(
            lambda *a, **k: self.MISSING_YAML, installed), 1)
        self.assertEqual(self.run_dependencies_only(
            lambda *a, **k: completed(0, 'ok'), lambda name: None), 1)


# 시험용 URDF 문자열을 만든다. 어깨 effort만 인자로 바꾼다.
def urdf_text(shoulder_effort='20.0'):
    joints = ''.join(
        f'<joint name="{name}" type="revolute"><limit lower="-1" upper="1" '
        f'effort="{shoulder_effort if name == "shoulder_joint" else "5"}" velocity="0.5"/></joint>'
        for name in vh.ARM_JOINTS)
    return f'<robot name="ax">{joints}</robot>'


# 실행 모델의 관절 제한 읽기와 검사한 URDF와의 일치 판정을 시험한다.
class RuntimeDescriptionTest(unittest.TestCase):
    # ros2 param get 출력에서 문자열을 꺼내고 팔 관절 제한을 읽는지 확인한다.
    def test_parameter_string_and_limits(self):
        text = 'String value is: ' + urdf_text('100') + '\n'
        limits = vh.arm_joint_limits(vh.parameter_string(text))
        self.assertEqual(limits['shoulder_joint']['effort'], 100.0)
        self.assertEqual(limits['elbow_joint']['velocity'], 0.5)
        self.assertIsNone(vh.parameter_string('Parameter not set'))
        self.assertIsNone(vh.arm_joint_limits('<robot name="x"/>')['shoulder_joint'])

    # 실행 모델 제한이 확인되지 않거나 검사한 URDF와 다르면 종료 코드 1인지 확인한다.
    def test_mismatch_or_missing_runtime_limits_fail(self):
        result = passing_result()
        result['checks']['runtime_joint_limits'] = {
            'matches_checked_urdf': vh.arm_joint_limits(urdf_text('100'))
            == vh.arm_joint_limits(urdf_text('20.0'))}
        code, failures = vh.final_verdict(result)
        self.assertEqual(code, 1)
        self.assertTrue(any('runtime robot_description' in f for f in failures))
        del result['checks']['runtime_joint_limits']
        self.assertEqual(vh.final_verdict(result)[0], 1)


if __name__ == '__main__':
    unittest.main()
