# HTTP 입력 검증·지연 명령 폐기·새 정지 우선순위를 검사한다. 브라우저 클릭 자체의 종단 시험은 별도다.
import importlib.util
from pathlib import Path
import sys
import time
from types import SimpleNamespace
import unittest

TOOLS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS))
spec = importlib.util.spec_from_file_location('remote_control', TOOLS / 'remote_control.py')
remote = importlib.util.module_from_spec(spec)
spec.loader.exec_module(remote)


class BrowserRemote(unittest.TestCase):
    def test_controller_readiness_with_and_without_ansi(self):
        for colored in (False, True):
            for state in ('active', 'inactive'):
                name, status = 'arm_controller', state
                if colored:
                    name = '\x1b[92m' + name + '\x1b[0m'
                    status = '\x1b[92m' + status + '\x1b[0m'
                line = f'{name}\tjoint_trajectory_controller/JointTrajectoryController  {status}\n'
                self.assertEqual(remote.controllers_active(line, ['arm_controller']), state == 'active')
                self.assertFalse(remote.controllers_active(line, ['diff_drive_controller']))
        self.assertFalse(remote.controllers_active('arm_controller_extra type active', ['arm_controller']))
        self.assertFalse(remote.controllers_active('arm_controller type active_error', ['arm_controller']))
        self.assertFalse(remote.controllers_active('warning: arm_controller is active', ['arm_controller']))

    def command(self, action, value=None, seq=1):
        return {'action': action, 'value': value, 'seq': seq, 'client': 'test'}

    def test_rejects_invalid_control_inputs(self):
        for action, value in [('drive', 'q'), ('joint', -1), ('joint', 5), ('joint', True),
                              ('robot', 'robot_3'), ('jog', 3), ('speed', 1.5)]:
            with self.assertRaises(ValueError):
                remote.validate_command(self.command(action, value))

    def test_delayed_drive_command_is_not_executed(self):
        state = remote.Remote()
        calls = []
        node = SimpleNamespace(key=lambda key: calls.append(key))
        state.apply(node, self.command('drive', 'w'), time.monotonic() - 1)
        self.assertEqual(calls, [])

    def test_old_drive_cannot_override_newer_stop(self):
        state = remote.Remote()
        calls = []
        node = SimpleNamespace(key=lambda key: calls.append('drive'), stop=lambda: calls.append('stop'))
        state.apply(node, self.command('stop', seq=2), time.monotonic())
        state.apply(node, self.command('drive', 'w', seq=1), time.monotonic())
        self.assertEqual(calls, ['stop'])

    def test_robot_change_stops_before_switching(self):
        state = remote.Remote()
        calls = []
        node = SimpleNamespace(robot='robot_1', stop=lambda: calls.append('stop'))
        state.apply(node, self.command('robot', 'robot_2'), time.monotonic())
        self.assertEqual(calls, ['stop'])
        self.assertEqual(node.robot, 'robot_2')
