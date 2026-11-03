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
