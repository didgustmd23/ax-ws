"""Gazebo-free checks of the handoff verifier's parsing and decision logic."""
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


def controllers_text(states, color=True):
    """Imitate ros2controlcli list_controllers rows (colour on name and state)."""
    rows = []
    for name, state in states.items():
        c = (GREEN if state == 'active' else CYAN) if color else ''
        e = END if color else ''
        rows.append(f'{c}{name:<24}{e} some_pkg/SomeType  {c}{state:<8}{e}')
    return '\n'.join(rows) + '\n'


def interfaces_text(command, color=True):
    """Imitate list_hardware_interfaces; command maps name -> (available, claimed)."""
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


def control_state(states=ALL_ACTIVE, command=ALL_CLAIMED, rc_c=0, rc_h=0):
    return (vh.evaluate_controllers(rc_c, vh.parse_controllers(controllers_text(states))),
            vh.evaluate_interfaces(rc_h, vh.parse_hardware_interfaces(interfaces_text(command))))


def passing_result():
    controllers, interfaces = control_state()
    motion = {j: {'status': 'motion_observed', 'target': 0.3, 'measured_position': 0.29,
                  'absolute_target_error': 0.01, 'review_tolerance': 0.1, 'unit': 'rad',
                  'within_review_tolerance': True}
              for j in vh.ARM_JOINTS + vh.FINGER_JOINTS}
    return {'checks': {'readiness': {'ready': True}, 'xacro': {'returncode': 0},
                       'check_urdf': {'returncode': 0}, 'controllers': controllers,
                       'hardware_interfaces': interfaces, 'scan': {'received': 10},
                       'motion': motion},
            'launch_alive_at_end': True, 'cleanup': {'result': 'STOPPED'}}


class ImportTest(unittest.TestCase):
    def test_import_has_no_side_effects(self):
        self.assertEqual(sorted(p.name for p in EVIDENCE.iterdir()), _BEFORE_IMPORT)
        self.assertEqual({k: os.environ.get(k) for k in _ENV_BEFORE}, _ENV_BEFORE)


class ArgumentTest(unittest.TestCase):
    BASE = ['ros2', 'launch', 'ax_control', 'simulation.launch.py']

    def test_default_keeps_launch_defaults(self):
        args = vh.parse_arguments([])
        self.assertFalse(args.headless)
        self.assertEqual(vh.launch_command(args.headless), self.BASE)
        self.assertEqual(args.domain_id, 87)
        self.assertEqual(args.ready_timeout, vh.READY_TIMEOUT_S)

    def test_headless_adds_exactly_gui_false(self):
        args = vh.parse_arguments(['--headless'])
        self.assertEqual(vh.launch_command(args.headless), self.BASE + ['gui:=false'])

    def test_domain_and_ready_timeout(self):
        self.assertIsNone(vh.parse_arguments(['--domain-id', 'auto']).domain_id)
        self.assertEqual(vh.parse_arguments(['--domain-id', '61']).domain_id, 61)
        self.assertEqual(vh.parse_arguments(['--ready-timeout', '15']).ready_timeout, 15.0)
        for bad in (['--domain-id', '233'], ['--domain-id', 'x'], ['--ready-timeout', '0']):
            with mock.patch('sys.stderr'), self.assertRaises(SystemExit):
                vh.parse_arguments(bad)


class ContractTest(unittest.TestCase):
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


class ControllerParsingTest(unittest.TestCase):
    def test_colored_active_rows_are_parsed_exactly(self):
        controllers, _ = control_state()
        self.assertTrue(controllers['ok'], controllers['problems'])
        self.assertEqual(set(controllers['states'].values()), {'active'})

    def test_inactive_is_not_active(self):
        for color in (True, False):
            parsed = vh.parse_controllers(controllers_text(
                dict(ALL_ACTIVE, arm_controller='inactive'), color))
            self.assertEqual(parsed['controllers']['arm_controller']['state'], 'inactive')
            check = vh.evaluate_controllers(0, parsed)
            self.assertFalse(check['ok'])
            self.assertIn('controller arm_controller is inactive', check['problems'])

    def test_missing_controller_fails(self):
        states = dict(ALL_ACTIVE)
        del states['finger_controller']
        controllers, _ = control_state(states=states)
        self.assertFalse(controllers['ok'])
        self.assertIn('controller finger_controller not listed', controllers['problems'])

    def test_query_failure_and_unparsable_output_fail(self):
        check = vh.evaluate_controllers(124, vh.parse_controllers(''))
        self.assertFalse(check['ok'])
        self.assertIn('list_controllers exited with 124', check['problems'])
        check = vh.evaluate_controllers(
            0, vh.parse_controllers('Waiting for service /controller_manager ...\n'))
        self.assertFalse(check['ok'])
        self.assertTrue(any('unparsed line' in p for p in check['problems']))


class InterfaceParsingTest(unittest.TestCase):
    def test_expected_claimed_set_passes(self):
        _, interfaces = control_state()
        self.assertTrue(interfaces['ok'], interfaces['problems'])
        self.assertEqual(interfaces['claimed'], sorted(vh.EXPECTED_COMMAND_INTERFACES))

    def test_state_section_is_not_counted_as_command(self):
        parsed = vh.parse_hardware_interfaces(interfaces_text(ALL_CLAIMED))
        self.assertNotIn('shoulder_joint/velocity', parsed['command'])

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

    def test_known_reference_interfaces_are_kept_apart(self):
        # Jazzy diff_drive_controller is chainable and lists these two reference interfaces.
        _, interfaces = control_state(command=dict(ALL_CLAIMED, **self.REFERENCES))
        self.assertTrue(interfaces['ok'], interfaces['problems'])
        self.assertEqual(interfaces['controller_references'], sorted(self.REFERENCES))
        self.assertEqual(interfaces['claimed'], sorted(vh.EXPECTED_COMMAND_INTERFACES))

    def test_claimed_reference_interface_fails(self):
        for name in self.REFERENCES:
            command = dict(ALL_CLAIMED, **self.REFERENCES)
            command[name] = (True, True)
            _, interfaces = control_state(command=command)
            self.assertFalse(interfaces['ok'])
            self.assertIn(f'controller reference interface {name} claimed'
                          ' (no chain configured)', interfaces['problems'])

    def test_unknown_interface_with_controller_prefix_fails(self):
        for name in ('diff_drive_controller/linear/position', 'diff_drive_controller/x',
                     'arm_controller/shoulder_joint/position', 'base/velocity'):
            _, interfaces = control_state(command=dict(ALL_CLAIMED, **self.REFERENCES,
                                                       **{name: (True, False)}))
            self.assertFalse(interfaces['ok'], name)
            self.assertEqual(interfaces['unexpected'], [name])
            self.assertEqual(interfaces['controller_references'], sorted(self.REFERENCES))

    def test_query_failure_fails(self):
        check = vh.evaluate_interfaces(1, vh.parse_hardware_interfaces(''))
        self.assertFalse(check['ok'])
        self.assertIn('list_hardware_interfaces exited with 1', check['problems'])
        self.assertTrue(any('no "command interfaces" section' in p for p in check['problems']))


class FinalVerdictTest(unittest.TestCase):
    def test_complete_result_passes(self):
        self.assertEqual(vh.final_verdict(passing_result()), (0, []))

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

    def test_incomplete_result_stays_exit_1_even_with_tolerance_miss(self):
        result = passing_result()
        result['checks']['motion']['wrist_joint']['within_review_tolerance'] = False
        result['launch_alive_at_end'] = False
        self.assertEqual(vh.final_verdict(result)[0], 1)

    def check_fails(self, change, expected):
        result = passing_result()
        change(result['checks'])
        code, failures = vh.final_verdict(result)
        self.assertEqual(code, 1)
        self.assertTrue(any(expected in f for f in failures), failures)

    def test_query_failure_fails(self):
        def change(checks):
            checks['controllers'] = vh.evaluate_controllers(124, vh.parse_controllers(''))
        self.check_fails(change, 'list_controllers exited with 124')

        def change(checks):
            checks['hardware_interfaces'] = vh.evaluate_interfaces(
                1, vh.parse_hardware_interfaces(''))
        self.check_fails(change, 'list_hardware_interfaces exited with 1')

    def test_missing_controller_fails(self):
        states = dict(ALL_ACTIVE)
        del states['diff_drive_controller']

        def change(checks):
            checks['controllers'] = control_state(states=states)[0]
        self.check_fails(change, 'diff_drive_controller not listed')

    def test_interface_mismatch_fails(self):
        command = dict(ALL_CLAIMED, **{'gripper_rotation_joint/position': (True, False)})

        def change(checks):
            checks['hardware_interfaces'] = control_state(command=command)[1]
        self.check_fails(change, 'gripper_rotation_joint/position unclaimed')

    def test_cleanup_failure_fails(self):
        result = passing_result()
        result['cleanup'] = {'result': 'STOP_FAILED'}
        self.assertEqual(vh.final_verdict(result), (1, ['cleanup: STOP_FAILED']))
        del result['cleanup']
        self.assertEqual(vh.final_verdict(result), (1, ['cleanup: not run']))

    def test_missing_checks_fail(self):
        self.check_fails(lambda checks: checks.pop('controllers'), 'controllers: not checked')
        self.check_fails(lambda checks: checks.pop('readiness'), 'readiness')


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class ReadinessTest(unittest.TestCase):
    SUBSCRIBED = {'/arm_controller/joint_trajectory': 1, '/gripper/command': 1}

    def start(self, statuses, launch_codes=None, timeout_s=20.0, durations=None):
        """Run start_when_ready on a fake clock; each probe takes durations[i] s (default 1)."""
        clock = FakeClock()
        statuses = iter(statuses)
        durations = iter(durations or [])
        launch_codes = iter(launch_codes or [])
        on_ready = mock.Mock()
        self.remaining = []

        def probe(remaining_s):
            self.remaining.append(remaining_s)
            clock.now += next(durations, 1.0)
            return next(statuses)
        readiness = vh.start_when_ready(
            probe, lambda: next(launch_codes, None), on_ready, timeout_s=timeout_s,
            poll_s=5.0, clock=clock, sleep=clock.sleep)
        return readiness, on_ready

    def ready(self):
        return vh.readiness_status(*control_state(), self.SUBSCRIBED)

    def test_ready_just_before_limit_sends(self):
        readiness, on_ready = self.start([self.ready()], timeout_s=10.0, durations=[9.9])
        self.assertTrue(readiness['ready'])
        on_ready.assert_called_once()

    def test_ready_at_or_after_limit_does_not_send(self):
        for duration in (10.0, 10.5):
            readiness, on_ready = self.start([self.ready()], timeout_s=10.0,
                                             durations=[duration])
            self.assertFalse(readiness['ready'], duration)
            self.assertIn('ready only at or after the limit', readiness['reason'])
            on_ready.assert_not_called()

    def test_limit_passing_during_probe_does_not_send(self):
        not_ready = vh.readiness_status(*control_state(rc_c=124), self.SUBSCRIBED)
        # t=1 not ready, sleep to t=6, second probe gets 4 s but runs until t=12.
        readiness, on_ready = self.start([not_ready, self.ready()], timeout_s=10.0,
                                         durations=[1.0, 6.0])
        self.assertEqual(self.remaining, [10.0, 4.0])
        self.assertFalse(readiness['ready'])
        on_ready.assert_not_called()

    def test_sleep_is_limited_to_remaining_time(self):
        not_ready = vh.readiness_status(*control_state(rc_c=124), self.SUBSCRIBED)
        # t=1 not ready; the poll sleep is cut to 2 s instead of 5 s, then no time is left.
        readiness, on_ready = self.start([not_ready] * 3, timeout_s=3.0)
        self.assertEqual(self.remaining, [3.0])
        self.assertFalse(readiness['ready'])
        self.assertAlmostEqual(readiness['elapsed_s'], 3.0)
        self.assertIn('list_controllers exited with 124', readiness['reason'])
        on_ready.assert_not_called()

    def test_ready_sends_commands(self):
        ready = vh.readiness_status(*control_state(), self.SUBSCRIBED)
        readiness, on_ready = self.start([ready])
        self.assertTrue(readiness['ready'])
        on_ready.assert_called_once()

    def test_subscribers_with_inactive_controller_do_not_send(self):
        status = vh.readiness_status(
            *control_state(states=dict(ALL_ACTIVE, arm_controller='inactive')),
            self.SUBSCRIBED)
        self.assertFalse(status['ready'])
        readiness, on_ready = self.start([status] * 10, timeout_s=12.0)
        self.assertFalse(readiness['ready'])
        self.assertIn('arm_controller is inactive', readiness['reason'])
        on_ready.assert_not_called()

    def test_missing_subscriber_is_not_ready(self):
        status = vh.readiness_status(*control_state(), {'/arm_controller/joint_trajectory': 1,
                                                        '/gripper/command': 0})
        self.assertFalse(status['ready'])
        self.assertIn('no subscriber on /gripper/command', status['reasons'])
        self.assertFalse(vh.readiness_status(*control_state(), {})['ready'])

    def test_timeout_does_not_start_motion(self):
        status = vh.readiness_status(*control_state(rc_c=124), self.SUBSCRIBED)
        readiness, on_ready = self.start([status] * 10, timeout_s=12.0)
        self.assertFalse(readiness['ready'])
        self.assertTrue(readiness['reason'].startswith('not ready within 12 s'))
        self.assertEqual([a['controllers_returncode'] for a in readiness['attempts']],
                         [124] * len(readiness['attempts']))
        on_ready.assert_not_called()

    def test_launch_exit_does_not_start_motion(self):
        not_ready = vh.readiness_status(*control_state(rc_c=1), self.SUBSCRIBED)
        ready = vh.readiness_status(*control_state(), self.SUBSCRIBED)
        # Exit observed before the first probe.
        readiness, on_ready = self.start([ready], launch_codes=[1])
        self.assertFalse(readiness['ready'])
        self.assertIn('launch exited with code 1', readiness['reason'])
        on_ready.assert_not_called()
        # Exit observed right after a probe that reported ready.
        readiness, on_ready = self.start([not_ready, ready], launch_codes=[None, None, None, 2])
        self.assertFalse(readiness['ready'])
        self.assertIn('code 2', readiness['reason'])
        on_ready.assert_not_called()

    def test_ready_timeout_is_longer_than_launch_waits(self):
        self.assertGreater(vh.READY_TIMEOUT_S, 120 + 60)


def completed(returncode, stdout='', stderr=''):
    return subprocess.CompletedProcess([], returncode, stdout, stderr)


class DependencyTest(unittest.TestCase):
    MISSING_YAML = completed(1, stderr="ModuleNotFoundError: No module named 'yaml'\n")

    def test_pyyaml_installed_and_missing(self):
        ok = vh.check_python_module('/usr/bin/python3', 'yaml',
                                    lambda *a, **k: completed(0, '6.0.1 /x/yaml/__init__.py\n'))
        self.assertTrue(ok['ok'])
        missing = vh.check_python_module('/usr/bin/python3', 'yaml',
                                         lambda *a, **k: self.MISSING_YAML)
        self.assertFalse(missing['ok'])
        self.assertIn('ModuleNotFoundError', missing['detail'])
        self.assertFalse(vh.check_python_module(None, 'yaml')['ok'])

    def test_real_interpreter_import(self):
        self.assertTrue(vh.check_python_module(sys.executable, 'json')['ok'])
        self.assertFalse(vh.check_python_module(
            sys.executable, 'ax_module_that_does_not_exist')['ok'])

    def test_launch_python_from_shebang(self):
        with tempfile.TemporaryDirectory() as folder:
            ros2 = Path(folder) / 'ros2'
            ros2.write_text('#!/usr/bin/python3\n')
            self.assertEqual(vh.launch_python(lambda n: str(ros2))[0], '/usr/bin/python3')
            ros2.write_text('#!/usr/bin/env python3\n')
            found = vh.launch_python(lambda n: str(ros2) if n == 'ros2' else '/opt/py3')
            self.assertEqual(found[0], '/opt/py3')
        self.assertIsNone(vh.launch_python(lambda n: None)[0])

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

    def run_dependencies_only(self, runner, lookup):
        with tempfile.TemporaryDirectory() as folder, \
                mock.patch.dict(os.environ), \
                mock.patch.object(vh, 'ament_prefix_lookup', lambda: lookup), \
                mock.patch.object(vh, 'launch_python', lambda: ('/usr/bin/python3', 'test')), \
                mock.patch.object(vh.subprocess, 'run', runner), \
                mock.patch('builtins.print'):
            return vh.main(['--dependencies-only', '--output-dir', str(Path(folder) / 'out')])

    def test_dependencies_only_exit_code(self):
        installed = lambda name: f'/opt/{name}'  # noqa: E731
        self.assertEqual(self.run_dependencies_only(
            lambda *a, **k: completed(0, 'ok'), installed), 0)
        self.assertEqual(self.run_dependencies_only(
            lambda *a, **k: self.MISSING_YAML, installed), 1)
        self.assertEqual(self.run_dependencies_only(
            lambda *a, **k: completed(0, 'ok'), lambda name: None), 1)


if __name__ == '__main__':
    unittest.main()
